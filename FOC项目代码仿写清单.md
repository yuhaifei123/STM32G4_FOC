# FOC 项目代码仿写清单

本文补充 `FOC项目仿写还原路线.md` 中缺少的“对应代码”。目标是让你按文件一点点写，先还原最小可运行骨架，再继续写 SVPWM 和 FOC。

当前工程已有：

```text
Core/App/program.c
Core/App/program.h
Core/Src/adc.c
Core/Src/tim.c
Core/Src/dma.c
Core/Src/main.c
```

先不要急着写完整闭环。第一阶段只做：

```text
ADC2 DMA + TIM6
TIM1 六路互补 PWM
ADC1 Injected 三相电流采样
串口/断点查看 raw 数据
```

---

## 1. main.c 需要怎么接 App 层

CubeMX 生成初始化后，在 `main.c` 里加入：

```c
#include "program.h"
```

初始化顺序建议：

```c
MX_GPIO_Init();
MX_DMA_Init();
MX_TIM1_Init();
MX_ADC1_Init();
MX_ADC2_Init();
MX_TIM6_Init();
MX_USART1_UART_Init();
MX_SPI1_Init();

Program_Init();
```

主循环里调用：

```c
while (1)
{
    Program_Task();
}
```

注意：函数名大小写要完全一致。建议统一叫：

```c
Program_Init();
Program_Task();
```

---

## 2. program.h 最小版本

当前 `program.h` 里有两个命名问题：

```text
extern volatile program_debug_pwm_t program_debug_pwm;
void program_Task(void);
```

但 `program.c` 里是：

```text
volatile program_debug_pwm_t g_debug_pwm;
void Program_Task(void)
```

建议统一成下面这样：

```c
#ifndef __PROGRAM_H
#define __PROGRAM_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"

typedef struct {
    uint8_t enable;
    float duty_a;
    float duty_b;
    float duty_c;
} program_debug_pwm_t;

extern volatile program_debug_pwm_t g_debug_pwm;

void Program_Init(void);
void Program_Task(void);

void program_start_adc2_dma_chain(void);
void program_start_adc1_injected_chain(void);
void program_start_pwm_outputs(void);
void program_set_pwm_duty(float duty_a, float duty_b, float duty_c);

#endif
```

---

## 3. program.c 全局变量

先写这些全局变量：

```c
#include "program.h"
#include <math.h>

#define ADC2_DMA_LEN        2U
#define ADC_REF_V           3.30f
#define ADC_FULL_SCALE      4095.0f

#define VBUS_R_UP           240000.0f
#define VBUS_R_DOWN         10000.0f

#define CURRENT_GAIN        20.0f
#define CURRENT_RSENSE      0.01f

static volatile uint16_t g_adc2_dma_buf[ADC2_DMA_LEN];
static volatile uint32_t g_tim6_tick_ms = 0U;

static volatile uint16_t g_ia_raw = 0U;
static volatile uint16_t g_ib_raw = 0U;
static volatile uint16_t g_ic_raw = 0U;

static float g_ia_offset = 2048.0f;
static float g_ib_offset = 2048.0f;
static float g_ic_offset = 2048.0f;

volatile program_debug_pwm_t g_debug_pwm = {
    .enable = 0U,
    .duty_a = 0.5f,
    .duty_b = 0.5f,
    .duty_c = 0.5f,
};
```

变量含义：

| 变量 | 作用 |
|---|---|
| `g_adc2_dma_buf[0]` | ADC2 Rank1，通常是 VBUS |
| `g_adc2_dma_buf[1]` | ADC2 Rank2，通常是 NTC |
| `g_tim6_tick_ms` | TIM6 1ms 计数 |
| `g_ia_raw/g_ib_raw/g_ic_raw` | ADC1 三相电流 raw |
| `g_ia_offset/...` | 零电流偏置 |
| `g_debug_pwm` | 调试占空比 |

---

## 4. Program_Init 对应代码

第一版初始化只做三件事：

```text
功率级保持关闭
ADC 校准
启动 ADC2/TIM6、TIM1/ADC1
```

代码：

```c
void Program_Init(void)
{
    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_RESET);

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    program_start_adc2_dma_chain();
    program_start_pwm_outputs();
    program_start_adc1_injected_chain();
}
```

重要说明：

```text
N_SLEEP 通常是低电平睡眠、高电平使能。
所以初期调试建议先保持 RESET，不要一上电就打开功率级。
```

如果确认要使能功率级，再写：

```c
HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_SET);
```

---

## 5. 启动 ADC2 DMA + TIM6

ADC2 用来采 VBUS 和 NTC，TIM6 每 1ms 触发一次。

```c
void program_start_adc2_dma_chain(void)
{
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_adc2_dma_buf, ADC2_DMA_LEN) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}
```

对应 CubeMX：

```text
ADC2 Regular
Rank1 = Channel 12
Rank2 = Channel 14
External Trigger = TIM6_TRGO
DMA = Circular
TIM6 TRGO = Update Event
```

---

## 6. TIM6 回调

TIM6 中断每 1ms 进入一次。

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        g_tim6_tick_ms++;
    }
}
```

用途：

```text
慢任务计时
串口遥测节拍
状态机计时
ADC2 采样触发节奏确认
```

---

## 7. 启动 TIM1 六路互补 PWM

这个函数负责：

```text
先写 50% 占空比
再启动 CH1/2/3
再启动 CH1N/2N/3N
```

```c
void program_start_pwm_outputs(void)
{
    program_set_pwm_duty(0.5f, 0.5f, 0.5f);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}
```

写占空比函数：

```c
static float program_clamp_f32(float x, float min_v, float max_v)
{
    if (x < min_v) {
        return min_v;
    }
    if (x > max_v) {
        return max_v;
    }
    return x;
}

void program_set_pwm_duty(float duty_a, float duty_b, float duty_c)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    duty_a = program_clamp_f32(duty_a, 0.0f, 1.0f);
    duty_b = program_clamp_f32(duty_b, 0.0f, 1.0f);
    duty_c = program_clamp_f32(duty_c, 0.0f, 1.0f);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(duty_a * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(duty_b * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(duty_c * (float)arr));
}
```

验证：

```text
示波器测 PA8/PA9/PA10 和 PC13/PB0/PB1。
频率约 20kHz。
主路和互补路相反。
上下桥之间有死区。
```

---

## 8. 启动 ADC1 Injected

ADC1 注入组由 TIM1_TRGO2 触发。

```c
void program_start_adc1_injected_chain(void)
{
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}
```

对应 CubeMX：

```text
ADC1 Injected
Rank1 = Channel 1
Rank2 = Channel 3
Rank3 = Channel 4
Trigger = TIM1_TRGO2 rising edge
Regular = Disable
DMA = Disable
```

---

## 9. ADC1 注入完成回调

每次 ADC1 被 TIM1_TRGO2 触发并完成 3 路采样后，会进入这个回调。

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    g_ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    g_ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    g_ic_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
}
```

第一阶段先只保存 raw，不要在回调里打印。

原因：

```text
ADC 回调属于快环。
printf 太慢，会干扰控制时序。
```

---

## 10. ADC raw 换算函数

ADC 码值转电压：

```c
static float program_adc_raw_to_voltage(uint16_t raw)
{
    return (float)raw * ADC_REF_V / ADC_FULL_SCALE;
}
```

VBUS 换算：

```c
static float program_get_vbus_voltage(void)
{
    float vadc = program_adc_raw_to_voltage(g_adc2_dma_buf[0]);
    return vadc * (VBUS_R_UP + VBUS_R_DOWN) / VBUS_R_DOWN;
}
```

电流换算：

```c
static float program_current_from_raw(uint16_t raw, float offset)
{
    float sense_v = ((float)raw - offset) * ADC_REF_V / ADC_FULL_SCALE;
    return sense_v / (CURRENT_GAIN * CURRENT_RSENSE);
}
```

如果 `INA240A1 + 10mR`：

```text
CURRENT_GAIN = 20
CURRENT_RSENSE = 0.01
```

---

## 11. Program_Task 第一版

主循环慢任务先这样写，用断点看变量即可。

```c
void Program_Task(void)
{
    static uint32_t last_ms = 0U;
    uint32_t now_ms = g_tim6_tick_ms;

    if (now_ms == last_ms) {
        return;
    }
    last_ms = now_ms;

    float vbus = program_get_vbus_voltage();
    float ia = program_current_from_raw(g_ia_raw, g_ia_offset);
    float ib = program_current_from_raw(g_ib_raw, g_ib_offset);
    float ic = program_current_from_raw(g_ic_raw, g_ic_offset);

    (void)vbus;
    (void)ia;
    (void)ib;
    (void)ic;

    if (g_debug_pwm.enable != 0U) {
        program_set_pwm_duty(g_debug_pwm.duty_a, g_debug_pwm.duty_b, g_debug_pwm.duty_c);
    }
}
```

调试时在这里打断点，看：

```text
vbus 是否接近真实母线电压
ia/ib/ic 零电流是否接近 0
g_ia_raw/g_ib_raw/g_ic_raw 是否稳定
```

---

## 12. 电流零点校准代码

上电后功率级不使能，采样一段时间求平均。

```c
void program_calibrate_current_offset(void)
{
    const uint32_t sample_count = 1024U;
    uint32_t sum_a = 0U;
    uint32_t sum_b = 0U;
    uint32_t sum_c = 0U;

    for (uint32_t i = 0; i < sample_count; i++) {
        sum_a += g_ia_raw;
        sum_b += g_ib_raw;
        sum_c += g_ic_raw;
        HAL_Delay(1);
    }

    g_ia_offset = (float)sum_a / (float)sample_count;
    g_ib_offset = (float)sum_b / (float)sample_count;
    g_ic_offset = (float)sum_c / (float)sample_count;
}
```

注意：

```text
这个函数不要放在 ADC 快环回调里。
第一版可以在 Program_Init 启动 ADC1 后延时调用。
```

---

## 13. SVPWM 最小代码

等 PWM 和 ADC 都验证后，再加 SVPWM。

先定义结构：

```c
typedef struct {
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_duty_t;
```

简化版 SVPWM：

```c
#define PI_F        3.14159265359f
#define TWO_PI_F    6.28318530718f
#define SQRT3_F     1.73205080757f

static float wrap_0_2pi(float x)
{
    while (x >= TWO_PI_F) {
        x -= TWO_PI_F;
    }
    while (x < 0.0f) {
        x += TWO_PI_F;
    }
    return x;
}

static void inverse_park(float vd, float vq, float theta, float *v_alpha, float *v_beta)
{
    float c = cosf(theta);
    float s = sinf(theta);

    *v_alpha = vd * c - vq * s;
    *v_beta  = vd * s + vq * c;
}

static svpwm_duty_t svpwm_from_alpha_beta(float v_alpha, float v_beta, float vbus)
{
    svpwm_duty_t out;

    float va = v_alpha;
    float vb = -0.5f * v_alpha + 0.5f * SQRT3_F * v_beta;
    float vc = -0.5f * v_alpha - 0.5f * SQRT3_F * v_beta;

    float vmax = fmaxf(va, fmaxf(vb, vc));
    float vmin = fminf(va, fminf(vb, vc));
    float vcom = -0.5f * (vmax + vmin);

    out.duty_a = 0.5f + (va + vcom) / vbus;
    out.duty_b = 0.5f + (vb + vcom) / vbus;
    out.duty_c = 0.5f + (vc + vcom) / vbus;

    out.duty_a = program_clamp_f32(out.duty_a, 0.02f, 0.98f);
    out.duty_b = program_clamp_f32(out.duty_b, 0.02f, 0.98f);
    out.duty_c = program_clamp_f32(out.duty_c, 0.02f, 0.98f);

    return out;
}
```

开环测试：

```c
static float g_open_loop_theta = 0.0f;

void program_open_loop_test_1ms(void)
{
    float vbus = program_get_vbus_voltage();
    float v_alpha;
    float v_beta;
    svpwm_duty_t duty;

    if (vbus < 6.0f) {
        return;
    }

    g_open_loop_theta = wrap_0_2pi(g_open_loop_theta + 0.02f);

    inverse_park(0.0f, 1.0f, g_open_loop_theta, &v_alpha, &v_beta);
    duty = svpwm_from_alpha_beta(v_alpha, v_beta, vbus);

    program_set_pwm_duty(duty.duty_a, duty.duty_b, duty.duty_c);
}
```

第一版只低压限流测试，不要直接上高压母线。

---

## 14. 现在当前代码需要先修的点

你当前 `program.c/.h` 至少有这些要先统一：

| 问题 | 当前情况 | 建议 |
|---|---|---|
| 函数名不一致 | `Program_Task` vs `program_Task` | 统一 `Program_Task` |
| 全局变量名不一致 | `g_debug_pwm` vs `program_debug_pwm` | 统一 `g_debug_pwm` |
| PWM 启动函数是 static | `static void program_start_pwm(void)` | 改成 `program_start_pwm_outputs` 并在 `Program_Init` 调用 |
| ADC1 没启动 | 只启动了 ADC2 DMA | 增加 `HAL_ADCEx_InjectedStart_IT` |
| N_SLEEP 逻辑要确认 | 现在写 `RESET` | 若是 nSLEEP，低电平通常是睡眠，高电平才使能 |
| HAL 返回值没检查 | 直接调用 HAL | 启动链建议失败进 `Error_Handler()` |

先把这些修完，再写 SVPWM。

---

## 15. 第一阶段完成标准

第一阶段不要看电机转不转，只看数据链通不通。

完成标准：

```text
[ ] main.c 调用了 Program_Init 和 Program_Task
[ ] ADC2 DMA buffer 持续更新
[ ] TIM6 tick 每 1ms 增加
[ ] TIM1 六路 PWM 输出
[ ] ADC1 Injected 回调能进入
[ ] ia/ib/ic raw 有值且稳定
[ ] vbus 换算基本正确
```

这些都完成后，再进入第二阶段：

```text
SVPWM 开环
编码器角度
角度对齐
电流闭环
速度闭环
```

