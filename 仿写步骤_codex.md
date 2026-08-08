# FOC 电机控制项目仿写步骤（Codex 版）

这份文档不是“猜测版”，而是按当前工程源码整理的复原手册。目标很简单：让你按顺序抄，能少走弯路，少漏功能，最后尽量做到和原项目同构。

---

## 1. 项目定位

这是一个基于 `STM32G431 + MP6539 + MA600A` 的 FOC 电机驱动项目，源码里已经具备这些能力：

- 三相互补 PWM 驱动
- 三相电流同步采样
- 母线电压与 NTC 慢变量采样
- MA600A 编码器角度读取
- 10kHz 快环控制
- 1kHz 慢任务
- 零偏校准
- 编码器对齐
- 开环启动
- 电流环 / 速度环 / 位置环
- 故障检测与保护
- DWT 耗时统计
- UART 调试与 VOFA 波形发送

工程里还有一个 `motor_state.c` 状态机模块，但当前主流程实际跑的是 `program.c` 里的控制链。也就是说：

- **真正在线上的主逻辑**：`program_run_speed_current_control()`
- **独立状态机模块**：`motor_state.c`，目前更像备用/遗留结构

---

## 2. 总体架构

```text
main.c
  ├─ HAL_Init()
  ├─ SystemClock_Config()
  ├─ MX_GPIO_Init()
  ├─ MX_DMA_Init()
  ├─ MX_ADC1_Init()
  ├─ MX_ADC2_Init()
  ├─ MX_FDCAN1_Init()      // 目前仅初始化，未接业务
  ├─ MX_SPI1_Init()
  ├─ MX_TIM1_Init()
  ├─ MX_USART1_UART_Init()
  ├─ MX_TIM6_Init()
  ├─ program_init()
  └─ while(1) program_task()
```

### 两条执行链

```text
10kHz 快环
ADC1 注入完成中断
  ├─ 读取 IA/IB/IC
  ├─ 零偏校准
  ├─ 触发 MA600A 角度读取
  ├─ 更新编码器速度观测器
  ├─ 更新当前控制角
  ├─ 更新电流反馈 id/iq
  └─ 执行故障/对齐/速度环/电流环/电压输出

1kHz 慢环
TIM6 周期中断 + program_task()
  ├─ 读取 ADC2 DMA 的 VBUS / NTC
  ├─ 母线电压滤波
  ├─ 刷新故障标志
  └─ 按周期发送 VOFA 数据
```

---

## 3. 外设与引脚

### TIM1

- 用途：三相互补 PWM
- 模式：中心对齐
- ARR：4249
- 重复计数：3
- 死区：85
- TRGO2：Update，触发 ADC1 注入采样

### TIM6

- 用途：1kHz 慢任务时基
- PSC：169
- ARR：999
- TRGO：Update，触发 ADC2 常规转换

### ADC1

- 用途：三相电流采样
- 模式：注入组 3 通道
- 触发：TIM1_TRGO2
- 通道：PA0 / PA2 / PA3

### ADC2 + DMA1_Channel1

- 用途：母线电压 + NTC
- 模式：常规组 2 通道
- 触发：TIM6_TRGO
- DMA：循环模式，半字对齐
- 通道：PB2 / PB11

### SPI1

- 用途：MA600A 编码器
- 模式：主机
- 数据位宽：16bit
- CPOL/CPHA：High / 2Edge
- NSS：软件控制
- 片选：PA4

### USART1 + DMA1_Channel2

- 用途：调试串口 / VOFA
- 波特率：115200
- 数据：8N1
- TX：DMA

### GPIO

- PA4：ENC_CS
- PB14：N_SLEEP，默认低，控制驱动休眠/使能
- PB15：N_FAULT，输入上拉，驱动故障反馈

### FDCAN1

- 工程中已初始化
- 当前未接入实际业务逻辑
- 复原时要明确标注为“预留”

---

## 4. 中断优先级

| 中断 | 优先级 | 作用 |
|---|---:|---|
| ADC1_2_IRQn | 0 | 快环核心，电流采样与控制 |
| SPI1_IRQn | 1 | 编码器通信 |
| TIM6_DAC_IRQn | 2 | 1kHz 时基 |
| DMA1_Channel1_IRQn | 3 | ADC2 DMA |
| DMA1_Channel2_IRQn | 3 | USART1 TX DMA |
| USART1_IRQn | 4 | 串口接收 |

---

## 5. 源码文件结构

```text
App/
  program.c / program.h     主控制逻辑、调度、回调、遥测
  foc_core.c / foc_core.h   Clarke / Park / 反Park / SVPWM
  ma600a.c / ma600a.h       MA600A 编码器驱动
  filter.c / filter.h       一阶低通滤波
  drv_pid.c / drv_pid.h     Q15 PI 控制器
  cli_uart.c / cli_uart.h   串口调试 / VOFA
  motor_state.c / motor_state.h  独立状态机模块
  motor_params.h            电机参数与角度换算

Core/
  Src/main.c
  Src/adc.c
  Src/spi.c
  Src/tim.c
  Src/usart.c
  Src/dma.c
  Src/gpio.c
  Src/stm32g4xx_it.c
```

---

## 6. 主流程说明

### 6.1 上电初始化顺序

`program_init()` 实际做的事情：

1. 关闭驱动，使 `N_SLEEP` 拉低
2. 初始化遥测结构
3. 关闭 debug PWM
4. 对 ADC1 / ADC2 做硬件校准
5. 初始化 `foc_core`
6. 初始化 `motor_state`
7. 设置默认速度、位置、电流参数
8. 初始化位置环 / 编码器观测器 / 对齐状态
9. 初始化 VBUS 低通滤波器
10. 初始化 UART
11. 初始化 MA600A
12. 初始化 DWT 周期计数器
13. 预读一次 MA600A 角度
14. 启动 ADC2 DMA + TIM6
15. 启动 ADC1 注入链

### 6.2 快环入口

`HAL_ADCEx_InjectedConvCpltCallback()` -> `program_adc_injected_conv_cplt_callback()`

这里是整个项目最关键的地方，按顺序做：

1. 读 ADC1 三相电流
2. 零偏校准累计
3. 调用 `ma600a_read_angle()`
4. 更新编码器角度、速度、位置观测
5. 更新开环控制角
6. 计算当前控制角
7. 更新电流反馈 id/iq
8. 执行快环控制主流程
9. 统计 DWT 耗时

### 6.3 慢环入口

`HAL_TIM_PeriodElapsedCallback()` -> `program_tim_period_elapsed_callback()`  
`program_task()` 读取 `g_tim6_tick_ms`，每 1ms 执行一次：

1. 读取 ADC2 DMA 值
2. 更新 VBUS 滤波值
3. 刷新故障标志
4. 周期性发送 VOFA 波形

---

## 7. `program.c` 功能拆解

### 7.1 关键全局对象

| 对象 | 作用 |
|---|---|
| `g_program_telemetry` | 全部遥测数据 |
| `g_program_debug_pwm_test` | debug PWM 测试参数 |
| `g_motor` | 控制参数、状态、PI 积分、参考值 |
| `g_foc` | FOC 数学对象和占空比输出 |
| `g_ma600a` | 编码器对象 |

### 7.2 初始化相关

| 函数 | 作用 |
|---|---|
| `program_init()` | 总初始化入口 |
| `program_init_telemetry()` | 清空遥测并写入默认值 |
| `program_init_cycle_counter()` | 打开 DWT 并计算快环周期 |
| `program_start_adc2_dma_chain()` | 启动 ADC2 + TIM6 链 |
| `program_start_adc1_injected_chain()` | 启动 TIM1 PWM + ADC1 注入链 |
| `program_start_tim1_pwm_outputs()` | 启动 TIM1 主路和互补路 PWM |
| `program_set_power_stage_enable()` | 控制 N_SLEEP |

### 7.3 测量相关

| 函数 | 作用 |
|---|---|
| `program_update_measurements()` | 更新 VBUS / NTC |
| `program_convert_vbus_from_raw()` | ADC 码值转母线电压 |
| `program_convert_current_from_raw()` | ADC 码值转相电流 |
| `program_update_current_feedback_from_raw()` | 更新 ia/ib/ic/id/iq |
| `program_update_encoder_measurements()` | 更新角度、速度、观测状态 |
| `program_update_speed_measurement()` | 连续角速度估计 |
| `program_update_fault_flags()` | 刷新 N_FAULT 状态 |

### 7.4 控制相关

| 函数 | 作用 |
|---|---|
| `program_run_speed_current_control()` | 快环主入口 |
| `program_update_speed_loop()` | 速度环调度 |
| `program_update_position_loop()` | 位置环调度 |
| `program_run_current_loop()` | d/q 电流 PI + SVPWM |
| `program_run_voltage_mode()` | 电压开环模式 |
| `program_update_applied_current_references()` | 电流参考斜坡 |
| `program_update_speed_reference_ramp()` | 速度参考斜坡 |
| `program_limit_voltage_vector()` | 电压矢量限幅 |
| `program_run_pi_f32()` | 通用 PI，带抗饱和 |

### 7.5 编码器与对齐

| 函数 | 作用 |
|---|---|
| `program_get_encoder_rotor_mech_angle_rad()` | 编码器角转转子机械角 |
| `program_get_encoder_output_continuous_mech_angle_rad()` | 输出轴连续角 |
| `program_get_encoder_output_mech_angle_rad()` | 输出轴周期角 |
| `program_get_encoder_raw_elec_angle_rad()` | 原始电角度 |
| `program_get_encoder_aligned_elec_angle_rad()` | 对齐后的控制电角度 |
| `program_get_control_elec_angle_rad()` | 当前控制角统一入口 |
| `program_capture_encoder_alignment_sample()` | 对齐窗口采样 |
| `program_get_encoder_alignment_angle_rad()` | 计算平均对齐角 |
| `program_reset_encoder_alignment()` | 重置对齐状态 |
| `program_reset_encoder_observer()` | 重置速度观测器 |
| `program_renormalize_encoder_observer()` | 防止连续角无限增长 |
| `program_update_control_angle_open_loop_state()` | 开环角积分 |

### 7.6 状态与保护

| 函数 | 作用 |
|---|---|
| `program_is_driver_fault_active()` | 读取 N_FAULT |
| `program_reset_speed_loop()` | 速度环积分和状态复位 |
| `program_reset_position_loop()` | 位置环复位 |
| `program_reset_current_loop()` | 电流环复位 |
| `program_reset_speed_reference_ramp()` | 速度参考斜坡复位 |
| `program_handle_position_loop_mode_switch()` | 位置环模式切换处理 |
| `program_handle_current_loop_mode_switch()` | 电流环模式切换处理 |

### 7.7 遥测与调试

| 函数 | 作用 |
|---|---|
| `program_update_debug_telemetry()` | 汇总遥测快照 |
| `program_send_wave_if_needed()` | 定周期发送 VOFA |
| `program_apply_debug_pwm_test_output()` | debug PWM 输出 |
| `program_apply_center_duty()` | 安全中心占空比 |
| `program_apply_svpwm_to_tim1()` | 写 TIM1 CCR |

---

## 8. FOC 数学核心

### 8.1 `foc_core.c`

这个模块只负责数学，不碰硬件寄存器。

| 函数 | 作用 |
|---|---|
| `foc_core_init()` | 清零 FOC 对象 |
| `foc_core_reset_output()` | 输出拉回 50% 安全态 |
| `foc_core_set_bus_voltage()` | 更新母线电压 |
| `foc_core_set_electrical_angle()` | 更新电角度并缓存 sin/cos |
| `foc_core_clarke()` | Clarke 变换 |
| `foc_core_park()` | Park 变换 |
| `foc_core_inv_park()` | 反 Park |
| `foc_core_svpwm()` | SVPWM 输出占空比 |
| `foc_core_run_voltage_open_loop()` | 一步式电压开环 |

### 8.2 核心算法

- Clarke：`ia, ib -> iα, iβ`
- Park：`iα, iβ -> id, iq`
- 反 Park：`ud, uq -> vα, vβ`
- SVPWM：`vα, vβ -> duty_a, duty_b, duty_c`

---

## 9. 编码器 MA600A

### 9.1 驱动对象

`ma600a_t` 里记录了：

- SPI 句柄
- CS 端口和引脚
- 发送/接收字
- 原始角度
- 角度转弧度/角度
- 数据有效位
- 忙标志
- 连续坏点计数
- 采样次数
- 拒绝次数
- 通信错误次数

### 9.2 主要函数

| 函数 | 作用 |
|---|---|
| `ma600a_init()` | 绑定 SPI 和 CS |
| `ma600a_read_angle()` | 发起一次 SPI 读角 |
| `ma600a_spi_txrx_cplt_callback()` | SPI 完成回调 |
| `ma600a_spi_error_callback()` | SPI 错误回调 |

### 9.3 编码器逻辑特点

- 不是只读一个原始角度
- 有坏样本判定
- 有连续坏点计数
- 有通信错误计数
- 快环中不断更新角度和速度
- 对齐阶段用 `sin/cos` 平均法求电角偏置

---

## 10. 状态机模块 `motor_state.c`

这个模块单独存在，但当前主流程没直接调用它。复原时建议保留，因为它是另一种完整的状态机实现。

### 10.1 状态枚举

```text
INIT -> READY -> ALIGN -> OPEN_LOOP -> CLOSED_LOOP -> FAULT
```

### 10.2 主要函数

| 函数 | 作用 |
|---|---|
| `motor_state_init()` | 初始化状态机对象 |
| `motor_state_task()` | 状态切换和输出控制 |
| `motor_state_set_run_request()` | 设置运行请求 |
| `motor_state_set_fault()` | 设置故障 |
| `motor_state_clear_fault()` | 清除故障 |
| `motor_state_get_name()` | 状态名字 |

### 10.3 状态机行为

- READY：等待 run_request
- ALIGN：定电角锁定
- OPEN_LOOP：开环启动
- CLOSED_LOOP：闭环运行
- FAULT：故障停机

---

## 11. 滤波与 PI

### 11.1 `filter.c`

| 函数 | 作用 |
|---|---|
| `filter_lpf_f32_init()` | 初始化一阶低通 |
| `filter_lpf_f32_update()` | 更新浮点低通 |
| `filter_lpf_s32_init()` | 初始化整型低通 |
| `filter_lpf_s32_update()` | 更新整型低通 |

### 11.2 `drv_pid.c`

| 函数 | 作用 |
|---|---|
| `drv_pid_pi_init()` | 初始化 Q15 PI |
| `drv_pid_pi_reset()` | 重置 PI |
| `drv_pid_pi_step()` | 执行一步 PI |

当前主控制链主要用的是 `program_run_pi_f32()`，`drv_pid.c` 更像备用或移植保留。

---

## 12. 串口调试与 VOFA

### 12.1 `cli_uart.c`

| 函数 | 作用 |
|---|---|
| `cli_uart_init()` | 绑定 UART |
| `cli_uart_is_tx_busy()` | 查询发送忙状态 |
| `cli_uart_send_text()` | 阻塞文本发送 |
| `cli_uart_send_vofa()` | JustFloat + 尾帧 DMA 发送 |

### 12.2 关键点

- VOFA 发送不是字符串
- 数据是 float 二进制直接打包
- 末尾带 `0x00 0x00 0x80 0x7F`
- 发送 busy 时直接丢帧
- TX 完成回调里清 busy

---

## 13. 运行状态与控制链

### 13.1 快环内部逻辑

`program_run_speed_current_control()` 做的事很长，但核心是以下几段：

1. 读驱动故障
2. 如果是 debug PWM 模式，直接输出 debug 占空比
3. 如果驱动故障、零偏未完成、编码器无效，进入安全处理
4. 处理位置环 / 电流环模式切换
5. run_request 关闭时，回安全态
6. 对齐未完成时，执行对齐并计算编码器电角偏置
7. 编码器速度未就绪时，退回开环或电压模式
8. 正常闭环时，速度环生成 iq_ref 或 uq_ref
9. 电流环生成 ud/uq
10. 写入 SVPWM

### 13.2 对齐阶段

对齐过程不是简单“打一脚电压”：

- `ud = ALIGN_UD_V`
- `uq = 0`
- 维持 `ALIGN_HOLD_TICKS`
- 在最后一段窗口里累积 `sin(raw_theta)` 和 `cos(raw_theta)`
- 用 `atan2f(sum_sin, sum_cos)` 求平均对齐角
- 再算出 `encoder_elec_offset_rad`

### 13.3 速度观测

速度观测是按编码器样本窗口做差分：

- 连续角由 MA600A 角度拼接
- 窗口默认 20 个快环样本
- 低速时有量化保护
- 速度测量再过一阶低通

### 13.4 位置环

位置环是输出轴坐标，不是电机转子坐标。

特性：

- 200Hz 左右更新
- 带 hold / release 逻辑
- 有 creep 小速度补偿
- 输出的是机械速度参考
- 再乘减速比转成电机转子速度参考

### 13.5 电流环

电流环在快环里直接跑：

- `id_ref` 默认 0
- `iq_ref` 来自速度环
- `ud/uq` 通过 PI 生成
- 电压矢量限幅
- 最终调用 `foc_core_run_voltage_open_loop()`

---

## 14. 初始化和回调的实际接线

### 14.1 `main.c`

你要在 `USER CODE BEGIN 2` 里调用：

```c
program_init();
```

在 `while(1)` 里调用：

```c
program_task();
```

### 14.2 `stm32g4xx_it.c`

当前这些回调都已经接好了：

- `HAL_TIM_PeriodElapsedCallback()` -> `program_tim_period_elapsed_callback()`
- `HAL_ADC_ConvCpltCallback()` -> `program_adc_conv_cplt_callback()`
- `HAL_ADCEx_InjectedConvCpltCallback()` -> `program_adc_injected_conv_cplt_callback()`
- `HAL_SPI_TxRxCpltCallback()` -> `ma600a_spi_txrx_cplt_callback()`
- `HAL_SPI_ErrorCallback()` -> `ma600a_spi_error_callback()`
- `HAL_UART_TxCpltCallback()` -> `cli_uart.c`
- `HAL_UART_ErrorCallback()` -> `cli_uart.c`

---

## 15. 功能测试清单

下面这部分是仿写时最该照着走的。建议按顺序做，不要跳。

### 15.1 上电基础测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| 编译通过 | Keil 编译 | 无错误 |
| 系统时钟 | 观察 `SystemCoreClock` | 170MHz |
| TIM6 计数 | 观察 `g_tim6_tick_ms` | 每秒约加 1000 |
| 驱动休眠 | 上电默认 | `N_SLEEP=0` |
| PWM 关闭 | 初始状态 | TIM1 输出不驱动功率级 |

### 15.2 ADC2 慢变量测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| VBUS 原始值 | 看 `vbus_raw` | 跟母线电压相关 |
| VBUS 电压换算 | 看 `vbus` | 接近实际供电 |
| NTC 原始值 | 看 `ntc_raw` | 随温度变化 |
| DMA 正常 | 观察 DMA 缓冲 | 持续刷新 |

### 15.3 ADC1 电流采样测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| 三相原始码值 | 看 `ia_raw/ib_raw/ic_raw` | 正常更新 |
| 零偏校准 | 连续运行 1024 次快环 | `current_offset_ready=1` |
| 电流换算 | 看 `ia/ib/ic` | 归零后接近 0 |
| `id/iq` 计算 | 转动电机 | 变化合理 |

### 15.4 编码器测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| SPI 读角 | 断点看 `ma600a_read_angle()` 返回值 | 成功 |
| 角度值 | 观察 `ma600a_angle_deg/rad` | 连续变化 |
| 坏点过滤 | 模拟异常角 | `reject_count` 增加 |
| 通信错误 | 断开编码器 | `comm_error_count` 增加 |
| 连续角 | 手动转轴 | 角度无跳变 |

### 15.5 对齐测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| 对齐开始 | 置 `run_request=1` | 进入 ALIGN |
| 对齐电压 | 看 `ud_ref=ALIGN_UD_V` | 维持锁定 |
| 对齐窗口 | 观察 `g_encoder_align_counter` | 到 `ALIGN_HOLD_TICKS` |
| 对齐完成 | 观察 `encoder_align_done` | 变成 1 |
| 偏置角 | 看 `encoder_elec_offset_rad` | 得到稳定值 |

### 15.6 开环测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| 开环启动 | 初始闭环前阶段 | 电机可起转 |
| 开环角积分 | 看 `theta_open_loop` | 连续增加 |
| 开环频率 | 看 `open_loop_speed_elec` | 依据设定变化 |

### 15.7 速度环测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| 速度给定斜坡 | 修改 rpm 给定 | 平滑上升 |
| 速度测量 | 看 `speed_meas_mech_rpm` | 与实际转速接近 |
| 低速保护 | 接近 0 转速 | 不乱抖 |
| 速度 PI | 设定阶跃命令 | `iq_ref` 或 `uq_ref` 响应 |

### 15.8 位置环测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| 位置保持 | 轻微挪动轴 | 自动回位 |
| hold/release | 小角度扰动 | 先保持后释放 |
| creep | 小误差下观察 | 有微小补偿速度 |
| 位置到达 | `position_error` 很小 | 速度命令归零 |

### 15.9 电流环测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| `id=0` 控制 | 闭环运行 | d 轴接近 0 |
| `iq` 跟踪 | 改 `iq_ref` | 转矩变化明显 |
| 电压限幅 | 提高给定 | `ud/uq` 被限制 |
| 电流环切换 | 开关 `current_loop_enable` | 状态平稳 |

### 15.10 故障测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| nFAULT 拉低 | 短接 PB15 到 GND | 立即停机 |
| 故障恢复 | 松开故障并清状态 | 恢复 READY |
| 编码器失效 | 断开 SPI | 进入安全态 |
| 零偏未完成 | 启动前观察 | 不进入正常闭环 |

### 15.11 调试输出测试

| 测试项 | 方法 | 期望现象 |
|---|---|---|
| VOFA 输出 | 看串口数据 | 2ms 周期 |
| TX busy | 连续发送 | 忙时丢帧 |
| debug PWM | 置 `enable=1` | 直接输出设定占空比 |
| DWT 统计 | 看 `fast_loop_time_us` | 有值且小于周期 |

---

## 16. 建议的仿写顺序

如果你是从零抄，建议按这个顺序做，难度会低很多：

1. 先把 `main.c` 和 CubeMX 外设配对
2. 先做 `program_init()` 最小框架
3. 先把 `TIM6 + ADC2 DMA` 跑通
4. 再把 `TIM1 + ADC1 注入` 跑通
5. 再接 `MA600A`
6. 再做零偏校准
7. 再做编码器对齐
8. 再做开环
9. 再做速度环
10. 再做位置环
11. 最后补 VOFA、DWT、故障保护、debug PWM

---

## 17. 容易漏掉的点

这几个最容易漏：

- `MX_FDCAN1_Init()` 虽然在 `main.c`，但当前没接业务
- 主控制入口不是 `motor_state_task()`，而是 `program_run_speed_current_control()`
- `program_task()` 只是 1kHz 慢任务
- `program_adc_injected_conv_cplt_callback()` 才是 10kHz 快环入口
- 编码器对齐不是单点读取，而是窗口平均
- 速度观测不是直接差分，是窗口统计 + 低通 + 量化保护
- 位置环有 hold / release / creep
- `cli_uart_send_vofa()` 不是文本协议
- `N_SLEEP` 默认低，启动时必须主动拉高

---

## 18. 最后给你的复原目标

如果你要把这个项目完整仿出来，最少要保证以下 5 个层面都在：

1. **硬件层**：PWM、ADC1、ADC2、SPI1、TIM6、UART、GPIO、DMA
2. **控制层**：对齐、开环、速度环、电流环、位置环
3. **保护层**：nFAULT、零偏未完成、编码器异常、DWT overrun
4. **调试层**：VOFA、文本串口、debug PWM、遥测快照
5. **工程层**：main 初始化顺序、回调接线、模块文件结构

把这五层都补齐，才算真的接近“复原”。

---

## 19. 关键功能代码骨架

这一节不是完整源码全文，而是把最关键的功能函数按工程真实逻辑整理成可直接仿写的骨架。你可以按这个顺序把代码补回去。

### 19.1 `Core/Src/main.c`

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_FDCAN1_Init();      /* 当前仅初始化，业务未启用 */
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_TIM6_Init();

    program_init();

    while (1) {
        program_task();
    }
}
```

### 19.2 `App/program.h`

```c
typedef struct
{
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint16_t vbus_raw;
    uint16_t ntc_raw;
    uint16_t ma600a_angle_raw;
    uint16_t ia_offset_raw;
    uint16_t ib_offset_raw;
    uint16_t ic_offset_raw;
    uint16_t current_offset_sample_count;
    uint8_t  current_offset_ready;
    uint8_t  pwm_enable_cmd;
    uint8_t  power_stage_enabled;
    uint8_t  control_state;
    uint8_t  encoder_align_done;
    uint8_t  speed_loop_ready;
    uint8_t  current_loop_enable;
    uint8_t  position_loop_enable;
    uint8_t  control_angle_open_loop_enable;
    uint8_t  driver_fault_active;
    uint8_t  ma600a_angle_valid;
    uint8_t  ma600a_consecutive_bad_count;
    uint8_t  fast_loop_overrun;
    float ia;
    float ib;
    float ic;
    float id;
    float iq;
    float theta_elec;
    float duty_a;
    float duty_b;
    float duty_c;
    float vbus;
    float ma600a_angle_deg;
    float ma600a_angle_rad;
    float speed_ref_mech_rpm;
    float speed_ref_mech_applied_rpm;
    float speed_meas_mech_rpm;
    float position_ref_mech_deg;
    float position_meas_mech_deg;
    float position_error_mech_deg;
    float fast_loop_time_us;
    float fast_loop_period_us;
} program_telemetry_t;
```

### 19.3 `App/program.c` 总入口

```c
void program_init(void)
{
    program_set_power_stage_enable(0U);
    program_init_telemetry();

    g_program_debug_pwm_test.enable = 0U;
    g_program_debug_pwm_test.duty_a = 0.30f;
    g_program_debug_pwm_test.duty_b = 0.40f;
    g_program_debug_pwm_test.duty_c = 0.60f;

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    foc_core_init(&g_foc);
    foc_core_set_bus_voltage(&g_foc, 48.0f);

    motor_state_init(&g_motor);
    g_motor.state = MOTOR_STATE_READY;
    g_motor.current_loop_enable = 1U;
    g_motor.position_loop_enable = 0U;
    g_motor.speed_kp = 0.0015f;
    g_motor.speed_ki = 0.015f;
    g_motor.position_kp = 3.0f;
    g_motor.iq_limit = 12.0f;
    g_motor.current_kp = 2.5761f;
    g_motor.current_ki = 4555.31f;

    filter_lpf_f32_init(&g_vbus_lpf, 0.1f, 48.0f);
    cli_uart_init(&huart1);
    ma600a_init(&g_ma600a, &hspi1, ENC_CS_GPIO_Port, ENC_CS_Pin);
    program_init_cycle_counter();

    (void)ma600a_read_angle(&g_ma600a);

    program_update_fault_flags();
    program_update_debug_telemetry();
    program_start_adc2_dma_chain();
    program_start_adc1_injected_chain();
}
```

### 19.4 慢任务

```c
void program_task(void)
{
    uint32_t now_ms = g_tim6_tick_ms;
    if (now_ms == g_last_slow_task_tick_ms) {
        return;
    }

    g_last_slow_task_tick_ms = now_ms;
    program_update_measurements();
    program_update_fault_flags();
    program_send_wave_if_needed(now_ms);
}
```

### 19.5 10kHz 快环入口

```c
void program_adc_injected_conv_cplt_callback(ADC_HandleTypeDef *hadc)
{
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint32_t fast_loop_start_cycles;

    if ((hadc == 0) || (hadc->Instance != ADC1)) {
        return;
    }

    fast_loop_start_cycles = (g_dwt_cycle_counter_ready != 0U) ? DWT->CYCCNT : 0U;

    ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    ic_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    g_program_telemetry.ia_raw = ia_raw;
    g_program_telemetry.ib_raw = ib_raw;
    g_program_telemetry.ic_raw = ic_raw;

    if (g_program_telemetry.current_offset_ready == 0U) {
        g_ia_offset_sum += ia_raw;
        g_ib_offset_sum += ib_raw;
        g_ic_offset_sum += ic_raw;
        g_program_telemetry.current_offset_sample_count++;
        if (g_program_telemetry.current_offset_sample_count >= 1024U) {
            g_program_telemetry.ia_offset_raw = (uint16_t)(g_ia_offset_sum / 1024U);
            g_program_telemetry.ib_offset_raw = (uint16_t)(g_ib_offset_sum / 1024U);
            g_program_telemetry.ic_offset_raw = (uint16_t)(g_ic_offset_sum / 1024U);
            g_program_telemetry.current_offset_ready = 1U;
        }
    }

    (void)ma600a_read_angle(&g_ma600a);
    program_update_encoder_measurements();
    program_update_control_angle_open_loop_state();

    program_update_current_feedback_from_raw(ia_raw, ib_raw, ic_raw,
                                             g_encoder_align_done ? program_get_control_elec_angle_rad() : 0.0f);
    program_run_speed_current_control();

    if (g_dwt_cycle_counter_ready != 0U) {
        program_update_fast_loop_timing(DWT->CYCCNT - fast_loop_start_cycles);
    }
}
```

### 19.6 编码器读取

```c
uint8_t ma600a_read_angle(ma600a_t *sensor)
{
    if ((sensor == 0) || (sensor->transfer_busy != 0U)) {
        return 0U;
    }

    sensor->tx_word = 0x0000U;
    sensor->transfer_busy = 1U;
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_IT(sensor->hspi,
                                   (uint8_t *)&sensor->tx_word,
                                   (uint8_t *)&sensor->rx_word,
                                   1U) != HAL_OK) {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        sensor->transfer_busy = 0U;
        sensor->comm_error_count++;
        return 0U;
    }

    return 1U;
}
```

### 19.7 `VOFA` 发送

```c
uint8_t cli_uart_send_vofa(const float *values, uint8_t count)
{
    uint32_t payload_len = (uint32_t)count * (uint32_t)sizeof(float);
    uint32_t total_len = payload_len + 4U;

    if ((g_cli_uart == 0) || (values == 0) || (count == 0U)) {
        return 0U;
    }
    if (g_cli_uart_tx_busy != 0U) {
        return 0U;
    }
    if (total_len > CLI_UART_TX_BUF_SIZE) {
        return 0U;
    }

    memcpy(g_cli_uart_tx_buf, values, payload_len);
    memcpy(&g_cli_uart_tx_buf[payload_len], g_cli_uart_vofa_tail, 4U);
    g_cli_uart_tx_busy = 1U;

    if (HAL_UART_Transmit_DMA(g_cli_uart, g_cli_uart_tx_buf, (uint16_t)total_len) != HAL_OK) {
        g_cli_uart_tx_busy = 0U;
        return 0U;
    }

    return 1U;
}
```

### 19.8 FOC 数学核心

```c
void foc_core_clarke(float ia, float ib, foc_alpha_beta_t *out)
{
    out->alpha = ia;
    out->beta = (ia + 2.0f * ib) * 0.57735026919f;
}

void foc_core_park(const foc_alpha_beta_t *ab, float sin_theta, float cos_theta, foc_dq_t *out)
{
    out->d = ab->alpha * cos_theta + ab->beta * sin_theta;
    out->q = -ab->alpha * sin_theta + ab->beta * cos_theta;
}

void foc_core_inv_park(const foc_dq_t *dq, float sin_theta, float cos_theta, foc_alpha_beta_t *out)
{
    out->alpha = dq->d * cos_theta - dq->q * sin_theta;
    out->beta  = dq->d * sin_theta + dq->q * cos_theta;
}
```

### 19.9 电压矢量限幅

```c
static void program_limit_voltage_vector(float *ud_ref, float *uq_ref, float v_limit)
{
    float v_mag = sqrtf((*ud_ref * *ud_ref) + (*uq_ref * *uq_ref));
    if ((v_mag > v_limit) && (v_mag > 0.0f)) {
        float v_scale = v_limit / v_mag;
        *ud_ref *= v_scale;
        *uq_ref *= v_scale;
    }
}
```

### 19.10 电流环主计算

```c
static void program_run_current_loop(float theta_cmd)
{
    float v_limit = program_get_voltage_limit_v();
    g_motor.voltage_limit = v_limit;

    program_update_applied_current_references();

    g_motor.ud_ref = program_run_pi_f32(g_id_ref_applied_a,
                                        g_program_telemetry.id,
                                        g_motor.current_kp,
                                        g_motor.current_ki,
                                        PROGRAM_FAST_LOOP_DT_S,
                                        &g_motor.id_integral_v,
                                        -v_limit,
                                        v_limit);

    g_motor.uq_ref = program_run_pi_f32(g_iq_ref_applied_a,
                                        g_program_telemetry.iq,
                                        g_motor.current_kp,
                                        g_motor.current_ki,
                                        PROGRAM_FAST_LOOP_DT_S,
                                        &g_motor.iq_integral_v,
                                        -v_limit,
                                        v_limit);

    program_limit_voltage_vector(&g_motor.ud_ref, &g_motor.uq_ref, v_limit);
    foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, theta_cmd, g_program_telemetry.vbus);
}
```

### 19.11 故障和安全态

```c
if ((program_is_driver_fault_active() != 0U) ||
    (g_program_telemetry.current_offset_ready == 0U) ||
    (g_program_telemetry.ma600a_angle_valid == 0U)) {
    program_set_power_stage_enable(0U);
    foc_core_reset_output(&g_foc);
    foc_core_set_electrical_angle(&g_foc, 0.0f);
    program_apply_svpwm_to_tim1(&g_foc.duty);
    program_reset_speed_loop();
    program_reset_position_loop();
    program_reset_current_loop();
    program_reset_speed_reference_ramp();
    program_reset_encoder_observer();
    program_reset_encoder_alignment();
    return;
}
```

### 19.12 位置环和速度环入口

```c
static void program_update_speed_loop(void)
{
    if (g_motor.position_loop_enable == 0U) {
        program_reset_position_loop();
    }

    program_update_speed_reference_ramp();

    if (g_encoder_speed_ready == 0U) {
        g_motor.iq_ref = 0.0f;
        g_motor.uq_ref = 0.0f;
        return;
    }

    if (g_speed_loop_update_pending == 0U) {
        return;
    }

    g_speed_loop_update_pending = 0U;
    /* 速度 PI 结果根据 current_loop_enable 选择进入 iq_ref 或 uq_ref */
}
```

---

## 20. 你下一步怎么用这份文档

建议你直接按下面顺序补代码：

1. 先把 `main.c` 和 `program_init()` 抄出来
2. 把 `program_task()` 和 `program_adc_injected_conv_cplt_callback()` 跑通
3. 把 `ma600a_read_angle()` 和回调接好
4. 把 `cli_uart_send_vofa()` 加进去
5. 再补 `foc_core.c`
6. 最后补速度环、位置环、故障保护和 debug PWM

这样最省时间。
