# STM32G431 FOC 项目仿写还原路线

本文目标：按参考工程 `MPS_FOC_STM32G431-main/program` 的结构，一步一步在当前工程里还原一个可运行的 STM32G431 三相 FOC 控制项目。

不要一开始就写完整 FOC。正确路线是：

```text
CubeMX 外设配置
  -> GPIO / 通信基础验证
  -> TIM1 三相互补 PWM
  -> ADC1 三相电流同步采样
  -> ADC2 慢变量 DMA 采样
  -> 编码器角度读取
  -> 开环 SVPWM
  -> 电流闭环
  -> 速度/位置闭环
```

每一步都要能单独验证，再进入下一步。

---

## 1. 先明确项目功能分层

参考项目可以拆成 5 层：

| 层级 | 内容 | 目标 |
|---|---|---|
| 硬件接口层 | GPIO、SPI、USART、ADC、TIM、DMA | 让 MCU 能控制功率板、采样、通信 |
| 采样层 | ADC1 电流、ADC2 母线/NTC、编码器角度 | 得到 FOC 所需反馈量 |
| PWM 输出层 | TIM1 三相互补 PWM、死区、SVPWM 占空比写入 | 控制三相桥 |
| 控制算法层 | Clarke、Park、反 Park、SVPWM、PI | 产生三相电压命令 |
| 状态机/应用层 | 休眠、使能、校准、开环、闭环、保护、遥测 | 安全组织整个电机流程 |

仿写时按这个顺序做，不要跳着写。

---

## 2. CubeMX 外设配置总览

### 2.1 TIM1：三相互补 PWM

TIM1 是电机驱动核心，负责输出三相上/下桥 PWM。

| 配置项 | 设置 |
|---|---|
| Clock Source | `Internal Clock` |
| Channel1 | `PWM Generation CH1 CH1N` |
| Channel2 | `PWM Generation CH2 CH2N` |
| Channel3 | `PWM Generation CH3 CH3N` |
| Channel4/5/6 | `Disable` |
| Prescaler | `0` |
| Counter Mode | `Center Aligned mode 1` |
| Counter Period | `4249` |
| Internal Clock Division | `No Division` |
| Repetition Counter | `3` |
| auto-reload preload | `Enable` |
| TRGO | `Reset` |
| TRGO2 | `Update Event` |
| Dead Time | `85` |
| BRK / BRK2 | `Disable` |

频率计算：

```text
TIM1_CLK = 170MHz
PWM = 170MHz / (2 * (4249 + 1)) = 20kHz
DeadTime = 85 / 170MHz ~= 500ns
```

参考引脚：

| 相 | 上桥 | 下桥 |
|---|---|---|
| U/A | `PA8  -> TIM1_CH1` | `PC13 -> TIM1_CH1N` |
| V/B | `PA9  -> TIM1_CH2` | `PB0  -> TIM1_CH2N` |
| W/C | `PA10 -> TIM1_CH3` | `PB1  -> TIM1_CH3N` |

生成代码后应出现：

```c
MX_TIM1_Init();
```

启动 PWM 时必须同时启动主输出和互补输出：

```c
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
```

验证目标：

```text
不接电机，只用示波器/逻辑分析仪确认 6 路 PWM。
CHx 与 CHxN 互补。
上下桥之间有死区。
PWM 频率约 20kHz。
```

---

### 2.2 ADC1：三相电流同步采样

ADC1 用注入组采三相电流，由 TIM1_TRGO2 同步触发。

引脚：

| ADC 通道 | 用途 |
|---|---|
| `PA0 -> ADC1_IN1` | 相电流 1 |
| `PA2 -> ADC1_IN3` | 相电流 2 |
| `PA3 -> ADC1_IN4` | 相电流 3 |

基础配置：

| 配置项 | 设置 | 说明 |
|---|---|---|
| Mode | `Independent mode` | ADC1 独立工作 |
| Clock Prescaler | `Synchronous clock mode divided by 4` 或当前可用等效 ADC 时钟 | ADC 工作时钟分频 |
| Resolution | `12-bit` | 结果 0~4095 |
| Data Alignment | `Right alignment` | 右对齐 |
| Scan Conversion Mode | `Enabled` | 一次触发采多个通道 |
| End Of Conversion | `End of sequence of conversion` | 3 路采完才完成 |
| Continuous Conversion | `Disabled` | 不连续自采 |
| DMA Continuous Requests | `Disabled` | ADC1 不用 DMA |
| Regular Conversions | `Disable` | 不用普通组 |
| Injected Conversions | `Enable` | 用注入组 |
| Injected Number | `3` | 三路电流 |
| External Trigger Source | `Timer 1 Trigger Out event 2` | TIM1_TRGO2 触发 |
| Trigger Edge | `Rising edge` | 上升沿触发 |
| Injected Queue | `Disable` | 固定 Rank |

Injected Rank：

| Rank | Channel | Sampling Time | Offset |
|---|---|---|---|
| Rank 1 | `Channel 1` | `24.5 Cycles` | `No offset` |
| Rank 2 | `Channel 3` | `24.5 Cycles` | `No offset` |
| Rank 3 | `Channel 4` | `24.5 Cycles` | `No offset` |

NVIC：

| 中断 | 设置 |
|---|---|
| ADC1 and ADC2 global interrupt | `Enable` |
| Priority | `0` |

启动方式：

```c
HAL_ADCEx_InjectedStart_IT(&hadc1);
```

读取方式：

```c
uint16_t ia_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
uint16_t ib_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
uint16_t ic_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
```

验证目标：

```text
PWM 运行后，ADC1 注入中断能周期进入。
三路 raw 值在零电流附近稳定。
如果电流采样放大器有 1.65V 偏置，零电流 raw 应接近 2048。
```

---

### 2.3 ADC2：母线电压 / NTC 等慢变量采样

ADC2 用 Regular + DMA，由 TIM6_TRGO 触发。

引脚：

| ADC 通道 | 典型用途 |
|---|---|
| `PB2  -> ADC2_IN12` | VBUS 母线电压 |
| `PB11 -> ADC2_IN14` | NTC 温度 |

基础配置：

| 配置项 | 设置 |
|---|---|
| Mode | `Independent mode` |
| Clock Prescaler | `Synchronous clock mode divided by 4` 或当前可用等效 ADC 时钟 |
| Resolution | `12-bit` |
| Data Alignment | `Right alignment` |
| Scan Conversion Mode | `Enabled` |
| End Of Conversion | `End of single conversion` |
| Continuous Conversion | `Disabled` |
| DMA Continuous Requests | `Enabled` |
| Regular Conversions | `Enable` |
| Number Of Conversion | `2` |
| External Trigger Source | `Timer 6 Trigger Out event` |
| Trigger Edge | `Rising edge` |
| Injected Conversions | `Disable` |

Regular Rank：

| Rank | Channel | Sampling Time | Offset |
|---|---|---|---|
| Rank 1 | `Channel 12` | `640.5 Cycles` | `No offset` |
| Rank 2 | `Channel 14` | `640.5 Cycles` | `No offset` |

DMA：

| 配置项 | 设置 |
|---|---|
| DMA Request | `ADC2` |
| Channel | `DMA1 Channel 1` |
| Direction | `Peripheral To Memory` |
| Mode | `Circular` |
| Priority | `Low` |
| Peripheral Increment | `Disable` |
| Memory Increment | `Enable` |
| Peripheral Data Width | `Half Word` |
| Memory Data Width | `Half Word` |

启动方式：

```c
uint16_t adc2_buf[2];
HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, 2);
```

验证目标：

```text
启动 TIM6 后，adc2_buf[0] / adc2_buf[1] 持续更新。
VBUS 分压值随母线电压变化。
NTC 值随温度变化。
```

---

### 2.4 TIM6：ADC2 慢采样触发定时器

TIM6 给 ADC2 提供 1kHz 触发。

| 配置项 | 设置 |
|---|---|
| Activated | `Enable` |
| Prescaler | `169` |
| Counter Mode | `Up` |
| Counter Period | `999` |
| auto-reload preload | `Enable` |
| TRGO | `Update Event` |

频率：

```text
170MHz / (169 + 1) = 1MHz
1MHz / (999 + 1) = 1kHz
```

启动方式：

```c
HAL_TIM_Base_Start_IT(&htim6);
```

如果暂时不用 1ms 中断任务，只想触发 ADC2，也可以：

```c
HAL_TIM_Base_Start(&htim6);
```

参考工程使用中断，所以建议打开：

| 中断 | 设置 |
|---|---|
| TIM6_DAC_IRQn | `Enable` |
| Priority | `2` |

---

### 2.5 SPI / 编码器

参考项目使用 SPI 读取磁编码器角度。你后续如果换成 iC-MU150，需要把编码器驱动替换为 MU150 驱动，但对 FOC 层只暴露统一角度接口。

建议抽象接口：

```c
typedef struct {
    float mech_angle_rad;
    float elec_angle_rad;
    float speed_rad_s;
    uint8_t online;
} encoder_feedback_t;

void encoder_init(void);
void encoder_update(void);
float encoder_get_mech_angle_rad(void);
float encoder_get_speed_rad_s(void);
```

FOC 层不要关心底层是 MA600、AS5047、iC-MU150 还是别的编码器。

验证目标：

```text
手动转动电机轴，角度 0~2pi 连续变化。
正反转方向正确。
速度估算方向正确。
```

---

## 3. 代码模块建议

在 `Core` 保持 CubeMX 生成代码，在 `App` 写应用和算法。

建议目录：

```text
App/
  app_main.c/.h              顶层初始化和主循环
  board_config.h             板级引脚、比例系数、开关
  foc_core.c/.h              Clarke/Park/SVPWM/PI
  motor_params.h             电机参数
  motor_state.c/.h           状态机
  current_sample.c/.h        电流采样换算和偏置校准
  slow_adc.c/.h              VBUS/NTC 换算
  encoder.c/.h               编码器统一接口
  pwm_output.c/.h            TIM1 PWM 启停和占空比写入
  protection.c/.h            过压/欠压/过温/过流保护
  telemetry.c/.h             串口调试输出
```

一开始不要文件太多，可以先写 4 个：

```text
foc_core.c/.h
pwm_output.c/.h
current_sample.c/.h
app_main.c/.h
```

跑通后再拆。

---

## 4. 推荐还原步骤

### 第 1 步：只跑基础工程

目标：

```text
系统时钟 170MHz
GPIO 正常
USART printf 正常
SPI 能初始化
```

检查点：

```text
LED 闪烁
串口能打印 boot 信息
N_SLEEP 引脚可控制功率驱动休眠/使能
N_FAULT 引脚可读取
```

不要开 PWM，不要接电机。

---

### 第 2 步：跑 TIM1 互补 PWM

先写固定 50% 占空比：

```c
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2125);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2125);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2125);
```

然后启动 6 路 PWM。

检查点：

```text
PA8/PA9/PA10 有 20kHz PWM。
PC13/PB0/PB1 有互补 PWM。
死区存在。
```

此阶段不要使能功率级，或只低压限流测试。

---

### 第 3 步：跑 ADC2 + DMA + TIM6

启动顺序：

```c
HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, 2);
HAL_TIM_Base_Start_IT(&htim6);
```

检查点：

```text
adc2_buf[0] = VBUS raw
adc2_buf[1] = NTC raw
每 1ms 更新
```

换算公式示例：

```c
float adc_to_voltage(uint16_t raw)
{
    return 3.3f * (float)raw / 4095.0f;
}
```

VBUS 分压：

```text
Vadc = VBUS * Rlow / (Rhigh + Rlow)
VBUS = Vadc * (Rhigh + Rlow) / Rlow
```

NTC：

```text
Vadc = VCC * R_fixed / (R_ntc + R_fixed)
R_ntc = R_fixed * (VCC - Vadc) / Vadc
```

---

### 第 4 步：跑 ADC1 注入采样

启动顺序建议：

```c
program_start_tim1_pwm_outputs();
HAL_ADCEx_InjectedStart_IT(&hadc1);
```

在回调里读取：

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        ia_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        ib_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        ic_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    }
}
```

检查点：

```text
PWM 启动后 ADC1 回调周期进入。
三路 raw 值稳定。
零电流偏置可以被采集。
```

零点校准：

```text
上电后不使能电机，采样 N 次求平均。
ia_offset = avg(ia_raw)
ib_offset = avg(ib_raw)
ic_offset = avg(ic_raw)
```

电流换算：

```text
电流 = (raw - offset) * Vref / 4095 / 放大倍数 / 采样电阻
```

如果 INA240A1 + 10mR：

```text
Gain = 20
Rsense = 0.01 ohm
I = (raw - offset) * 3.3 / 4095 / 20 / 0.01
```

---

### 第 5 步：实现 SVPWM 开环

先不接编码器，不做闭环，只给一个缓慢旋转电角度：

```c
theta += 2.0f * PI * freq * dt;
```

给固定 q 轴电压：

```text
Ud = 0
Uq = 小电压，例如 1V
```

流程：

```text
Ud/Uq + theta_elec
  -> 反 Park 得到 Ualpha/Ubeta
  -> SVPWM 得到 duty_a/b/c
  -> 写 TIM1 CCR1/2/3
```

检查点：

```text
三相占空比随角度连续变化。
低压限流下电机能缓慢转动。
电机方向可控。
```

---

### 第 6 步：接编码器角度

编码器要提供机械角度：

```text
theta_mech: 0~2pi
```

机械角转电角：

```text
theta_elec = theta_mech * pole_pairs + offset
```

注意：

```text
方向可能需要取反。
机械角需要 wrap 到 0~2pi。
电角也需要 wrap 到 0~2pi。
```

检查点：

```text
手动转轴，机械角连续变化。
电角随极对数变化。
方向和开环旋转方向一致。
```

---

### 第 7 步：做电角度零位对齐

FOC 必须知道编码器角度和电机 d/q 轴之间的偏移。

简单做法：

```text
给固定电角度 theta_align。
给小的 d 轴电压或 q 轴电压让转子吸到固定位置。
等待一段时间。
读取编码器电角 raw_theta_elec。
offset = raw_theta_elec - theta_align。
```

之后使用：

```text
theta_elec_aligned = raw_theta_elec - offset
```

检查点：

```text
每次上电对齐后，电机开环/闭环方向一致。
电流不会异常冲击。
```

---

### 第 8 步：实现电流闭环

电流采样：

```text
ia, ib, ic
```

Clarke：

```text
alpha = ia
beta = (ia + 2 * ib) / sqrt(3)
```

Park：

```text
id = alpha*cos(theta) + beta*sin(theta)
iq = -alpha*sin(theta) + beta*cos(theta)
```

PI：

```text
vd = PI(id_ref - id)
vq = PI(iq_ref - iq)
```

反 Park + SVPWM：

```text
vd/vq -> valpha/vbeta -> duty_a/b/c
```

检查点：

```text
id 能跟随 id_ref。
iq 能跟随 iq_ref。
给小 iq_ref 电机产生稳定转矩。
```

先只做小电流，限幅必须保守。

---

### 第 9 步：速度环 / 位置环

速度估算：

```text
speed = wrap_delta(theta_now - theta_last) / dt
```

速度环：

```text
iq_ref = PI(speed_ref - speed_meas)
```

位置环：

```text
speed_ref 或 iq_ref = position_controller(position_ref - position_meas)
```

建议顺序：

```text
先电压开环
再电流闭环
再速度闭环
最后位置闭环
```

---

## 5. 启动顺序建议

参考工程的思路：

```text
HAL_Init
SystemClock_Config
MX_GPIO_Init
MX_DMA_Init
MX_ADC1_Init
MX_ADC2_Init
MX_SPI1_Init
MX_TIM1_Init
MX_USART1_UART_Init
MX_TIM6_Init

App_Init
  -> 编码器初始化
  -> 电流偏置变量初始化
  -> FOC 状态初始化
  -> 启动 ADC2 DMA
  -> 启动 TIM6
  -> 启动 TIM1 PWM
  -> 启动 ADC1 Injected IT
```

推荐代码：

```c
void app_init(void)
{
    pwm_output_set_center();
    current_sample_reset_calibration();
    slow_adc_start_dma();
    encoder_init();

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, 2);
    HAL_TIM_Base_Start_IT(&htim6);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
}
```

功率级使能 `N_SLEEP` 不要一上电就打开。建议状态机确认采样和通信正常后再使能。

---

## 6. 中断和任务分工

| 触发源 | 频率 | 做什么 |
|---|---:|---|
| ADC1 Injected 完成 | 由 TIM1_TRGO2 决定 | 快环：读取三相电流、执行 FOC |
| TIM6 中断 | 1kHz | 慢任务节拍、遥测节拍、状态机计时 |
| ADC2 DMA | 1kHz 触发转换 | 更新 VBUS / NTC raw |
| 主循环 while(1) | 尽可能快 | 后台任务、串口解析、状态机慢处理 |

快环中不要做：

```text
printf
复杂浮点打印
长时间 SPI 阻塞读取
HAL_Delay
```

---

## 7. 保护逻辑必须先有

最小保护：

| 保护 | 来源 | 动作 |
|---|---|---|
| 过压 | ADC2 VBUS | 关 PWM / 关 N_SLEEP |
| 欠压 | ADC2 VBUS | 禁止启动 |
| 过温 | ADC2 NTC | 降额或停机 |
| N_FAULT | GPIO 输入 | 立即停机 |
| ADC 异常 | raw 越界/不更新 | 停机 |
| 编码器异常 | SPI 错误/角度不更新 | 禁止闭环 |

停机动作建议统一：

```c
void motor_fault_stop(void)
{
    pwm_output_set_center();
    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_RESET);
}
```

---

## 8. 每个阶段的验收清单

### 阶段 A：外设

```text
[ ] 串口能打印
[ ] SPI 能读编码器 ID 或角度
[ ] N_SLEEP 可控
[ ] N_FAULT 可读
```

### 阶段 B：PWM

```text
[ ] TIM1 6 路 PWM 输出
[ ] PWM 20kHz
[ ] 互补输出正确
[ ] 死区约 500ns
```

### 阶段 C：ADC

```text
[ ] ADC1 注入组三路采样正常
[ ] 零电流 raw 稳定
[ ] ADC2 DMA 两路更新正常
[ ] VBUS / NTC 换算合理
```

### 阶段 D：开环

```text
[ ] SVPWM duty 连续变化
[ ] 低压限流下电机能转
[ ] 方向可控
```

### 阶段 E：闭环

```text
[ ] 编码器角度连续
[ ] 电角度方向正确
[ ] 对齐 offset 正常
[ ] iq_ref 小电流能产生稳定转矩
[ ] 速度闭环能低速稳定运行
```

---

## 9. 推荐调试顺序

非常重要：每次只改一个变量。

```text
1. 不接功率，验证 MCU 外设。
2. 不接电机，验证 PWM 波形。
3. 不开 N_SLEEP，验证 ADC 偏置和慢变量。
4. 低压限流，验证开环 SVPWM。
5. 接编码器，只看角度，不闭环。
6. 做电角度对齐。
7. 小电流闭环。
8. 小速度闭环。
9. 再逐步加电压、电流、速度。
```

不要直接上母线高压。

---

## 10. 你当前工程下一步建议

你已经基本完成 CubeMX 的核心外设配置。下一步建议按这个顺序写代码：

1. 确认生成代码里有：

```c
MX_TIM1_Init();
MX_ADC1_Init();
MX_ADC2_Init();
MX_TIM6_Init();
MX_DMA_Init();
```

2. 新建或补充 `App` 层：

```text
App/pwm_output.c
App/current_sample.c
App/slow_adc.c
App/foc_core.c
App/app_main.c
```

3. 先写最小启动链：

```text
ADC2 DMA + TIM6
TIM1 50% PWM
ADC1 Injected IT
串口打印 raw 值
```

4. 等 raw 值和 PWM 都正确后，再写 SVPWM。

---

## 11. 最小可运行目标

第一版不要追求完整闭环，只要做到：

```text
上电
  -> 串口打印
  -> ADC2 DMA 采 VBUS/NTC
  -> TIM1 输出 20kHz 三相互补 PWM
  -> ADC1 每次 TIM1_TRGO2 触发采三相电流
  -> 串口能打印 ia/ib/ic raw 和 vbus raw
```

这个目标完成后，项目骨架就立住了。

然后再继续：

```text
SVPWM 开环
  -> 编码器角度
  -> 角度对齐
  -> 电流环
  -> 速度环
  -> 位置环
```

