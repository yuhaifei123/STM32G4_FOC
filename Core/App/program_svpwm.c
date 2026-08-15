/*
 * ========================================
 *  program_svpwm.c — 编码器 / 斜坡 / 工具层
 *  负责: 速度测量、编码器角度获取与对齐、
 *        角度换算、斜坡生成、量化保护
 *  被调: program.c (HAL回调链)
 *        program_current.c (控制环)
 * ========================================
 */
/*
 * ── 编码器（角度传感器 MA600A）相关方法索引 ──
 * [角度获取]
 *   program_get_encoder_rotor_mech_angle_rad        转子机械角（原始角×方向符号）
 *   program_get_encoder_output_continuous_mech_angle_rad 输出轴连续机械角（÷减速比，可多圈）
 *   program_get_encoder_output_mech_angle_rad       输出轴单圈机械角（归一化 0~2π）
 *   program_get_encoder_raw_elec_angle_rad          原始电角度（机械角×极对数）
 *   program_get_encoder_aligned_elec_angle_rad      对齐后电角度（扣除零位偏置）
 *   program_get_control_elec_angle_rad              控制用电角度（开环/闭环选择）
 *   program_update_control_angle_open_loop_state    开环角度积分更新
 * [速度观测]
 *   program_update_speed_measurement                角度差分→滤波→量化保护→输出速度
 *   program_reset_encoder_observer                  复位测速观测器
 *   program_renormalize_encoder_observer (static)   连续角重归一化防精度丢失
 * [零位对齐]
 *   program_reset_encoder_alignment                 复位对齐结果（偏置/完成标志）
 *   program_reset_encoder_align_runtime             复位对齐累积量（sin/cos 和）
 *   program_capture_encoder_alignment_sample        对齐保持期采集 sin/cos 样本
 *   program_get_encoder_alignment_angle_rad         atan2 求平均对齐电角
 * [测速辅助]
 *   program_get_speed_quantization_rad_s            测速量化分辨率估算
 *   program_apply_speed_quantization_guard          低速量化抖动归零保护
 * [速度斜坡]
 *   program_update_speed_reference_ramp             速度给定斜坡（限加速度）
 */
#include "program_svpwm.h"
#include "program_current.h"
#include <math.h>
#include "filter.h"
#include "foc_core.h"
#include "ma600a.h"
#include "motor_params.h"

/* ── 跨文件业务状态（extern 声明见 program.h）：g_ma600a/g_motor/g_encoder/g_control ── */
/* g_ma600a 未在 program.h 中声明，此处单独 extern */
extern ma600a_t        g_ma600a;                        /* MA600A 磁编码器对象，存储角度和通信状态 */

/* ── 文件内静态状态：编码器观测器与零位对齐累积（仅本文件使用） ── */

/** 编码器测速观测器状态 */
typedef struct {
    uint32_t last_sample_counter;         /* 上一次样本计数器，用于检测新样本到达 */
    float    prev_mech_angle_rad;         /* 上一拍机械角 (rad)，用于差分计算 */
    float    continuous_mech_rad;         /* 连续累加机械角 (rad)，处理 0/360 跳变 */
    float    speed_window_start_mech_rad; /* 速度观测窗口起点机械角 (rad) */
    uint32_t speed_window_sample_count;   /* 速度观测窗口内已累积拍数 */
} encoder_observer_t;

/** 编码器测速观测器静态实例 */
static encoder_observer_t s_encoder_observer;

/** 零位对齐采样累积量 */
typedef struct {
    float    sum_sin;       /* 对齐采样窗口 sin 累加和 */
    float    sum_cos;       /* 对齐采样窗口 cos 累加和 */
    uint32_t sample_count;  /* 对齐采样窗口已采集样本数 */
} encoder_align_t;

/** 零位对齐采样累积量静态实例 */
static encoder_align_t s_encoder_align;

/////////////////////////////////////////////////
/* ── 角度工具 ── */
/**
 * @brief  把角度归一化到 [0, 2π)
 * @param  angle_rad  任意浮点角度 (rad)
 * @return 归一化后的角度，NaN/Inf 返回 0
 * @note   运行频率: 角度采样和测速处理中按需调用
 *          运行内容: 通过整圈折返保证角度始终落在单圈范围内
 */
float program_wrap_angle_0_2pi(float angle_rad)
{
    float turns;
    if (!isfinite(angle_rad)) return 0.0f;
    turns = floorf(angle_rad / MOTOR_TWO_PI);
    angle_rad -= turns * MOTOR_TWO_PI;
    if (angle_rad < 0.0f) angle_rad += MOTOR_TWO_PI;
    else if (angle_rad >= MOTOR_TWO_PI) angle_rad -= MOTOR_TWO_PI;
    return angle_rad;
}

/**
 * @brief  把角度差归一化到 [-π, π]
 * @param  angle_rad  任意角度差 (rad)
 * @return 最短角度差，NaN/Inf 返回 0
 * @note   运行频率: 位置误差和速度观测按需调用
 *          运行内容: 避免跨 0/2π 边界时出现跳变
 */
float program_wrap_delta_pm_pi(float angle_rad)
{
    if (!isfinite(angle_rad)) return 0.0f;
    return program_wrap_angle_0_2pi(angle_rad + PROGRAM_PI) - PROGRAM_PI;
}

/**
 * @brief  把角度归一化到 [0°, 360°)
 * @param  angle_deg  任意角度 (°)
 * @return 归一化后的角度，NaN/Inf 返回 0
 * @note   运行频率: 位置环和遥测更新时调用
 *          运行内容: 通过整圈折返保证角度落在 0~360° 内
 */
float program_wrap_angle_0_360_deg(float angle_deg)
{
    float turns;
    if (!isfinite(angle_deg)) return 0.0f;
    turns = floorf(angle_deg / 360.0f);
    angle_deg -= turns * 360.0f;
    if (angle_deg < 0.0f) angle_deg += 360.0f;
    else if (angle_deg >= 360.0f) angle_deg -= 360.0f;
    return angle_deg;
}

/**
 * @brief  rad/s → rpm 单位换算
 * @param  speed_rad_s  角速度 (rad/s)
 * @return 转速 (rpm)
 * @note   运行频率: 遥测和参数初始化时按需调用
 */
float program_rad_s_to_rpm(float speed_rad_s)
    { return speed_rad_s * (60.0f / MOTOR_TWO_PI); }

/**
 * @brief  rad → deg 单位换算
 * @param  angle_rad  角度 (rad)
 * @return 角度 (°)，NaN/Inf 返回 0
 * @note   运行频率: 位置环和遥测更新时按需调用
 */
float program_rad_to_deg(float angle_rad)
    { if (!isfinite(angle_rad)) return 0.0f; return angle_rad * (360.0f / MOTOR_TWO_PI); }

/**
 * @brief  deg → rad 单位换算
 * @param  angle_deg  角度 (°)
 * @return 角度 (rad)，NaN/Inf 返回 0
 * @note   运行频率: 位置环给定处理时按需调用
 */
float program_deg_to_rad(float angle_deg)
    { if (!isfinite(angle_deg)) return 0.0f; return angle_deg * (MOTOR_TWO_PI / 360.0f); }

/**
 * @brief  rpm → rad/s 单位换算
 * @param  speed_rpm  转速 (rpm)
 * @return 角速度 (rad/s)，NaN/Inf 返回 0
 * @note   运行频率: 速度给定更新和初始化时调用
 */
float program_rpm_to_rad_s(float speed_rpm)
    { 
        if (!isfinite(speed_rpm)) return 0.0f; return speed_rpm * (MOTOR_TWO_PI / 60.0f); 
    }
/////////////////////////////////////



/**
 * @brief  限制浮点量上下界 [min, max]
 * @param  value      输入值
 * @param  min_value  下限
 * @param  max_value  上限
 * @return 限幅后的值
 * @note   运行频率: 各控制环按需调用
 *          运行内容: 为电流、电压和速度中间量提供统一限幅
 */
float program_clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/* ── LPF 系数 ── */

/**
 * @brief  由截止频率和采样周期换算一阶低通滤波系数 α
 * @param  cutoff_hz  截止频率 (Hz)
 * @param  dt_s       采样周期 (s)
 * @return 滤波系数 α (0~1)
 * @note   运行频率: 滤波器初始化或更新系数时调用
 *          运行内容: α = 1 - exp(-2π·fc·dt)，参数非法时返回 1.0（无滤波）
 */
float program_lpf_alpha_from_cutoff_hz(float cutoff_hz, float dt_s)
{
    float alpha;
    if ((!isfinite(cutoff_hz)) || (!isfinite(dt_s)) || (cutoff_hz <= 0.0f) || (dt_s <= 0.0f))
        return 1.0f;
    alpha = 1.0f - expf(-MOTOR_TWO_PI * cutoff_hz * dt_s);
    return program_clamp_f32(alpha, 0.0f, 1.0f);
}

/* ── 量化保护 ── */

/**
 * @brief  估算编码器测速量化分辨率
 * @param  observer_window_samples  测速窗口样本数
 * @return 最小速度分辨率 (rad/s)
 * @note   运行频率: 速度测量更新时调用
 *          运行内容: 按编码器 LSB 和测速窗口长度换算速度量化台阶
 */
float program_get_speed_quantization_rad_s(uint32_t observer_window_samples)
{
    if (observer_window_samples == 0U) return 0.0f;
    return PROGRAM_ENCODER_LSB_RAD / (PROGRAM_FAST_LOOP_DT_S * (float)observer_window_samples);
}

/**
 * @brief  在低速附近抑制编码器量化抖动
 * @param  speed_mech_rad_s         待保护的机械角速度 (rad/s)
 * @param  observer_window_samples  测速窗口样本数
 * @return 保护后的机械角速度
 * @note   运行频率: 每次刷新速度测量后调用
 *          运行内容: 当速度指令和测速值都接近量化台阶时强制回零，
 *                    避免静止时微小抖动导致电机微振
 */
float program_apply_speed_quantization_guard(float speed_mech_rad_s,
                                             uint32_t observer_window_samples)
{
    float zero_hold_threshold_rad_s;
    if (!isfinite(speed_mech_rad_s)) return 0.0f;
    zero_hold_threshold_rad_s = PROGRAM_SPEED_MEAS_ZERO_HOLD_SCALE * program_get_speed_quantization_rad_s(observer_window_samples);
    if ((!isfinite(zero_hold_threshold_rad_s)) || (zero_hold_threshold_rad_s <= 0.0f))
        return speed_mech_rad_s;
    if (zero_hold_threshold_rad_s < PROGRAM_SPEED_MEAS_ZERO_HOLD_MIN_MECH_RAD_S)
        zero_hold_threshold_rad_s = PROGRAM_SPEED_MEAS_ZERO_HOLD_MIN_MECH_RAD_S;
    if ((fabsf(g_motor.speed_ref_mech_applied_rad_s) <= zero_hold_threshold_rad_s) &&
        (fabsf(speed_mech_rad_s) <= zero_hold_threshold_rad_s))
        return 0.0f;
    return speed_mech_rad_s;
}

/* ── 编码器角度获取 ── */

/**
 * @brief  读取编码器对应的转子机械角
 * @return 转子机械角 (rad)，已考虑编码器方向符号
 * @note   运行频率: 编码器采样和角度换算时按需调用
 */
float program_get_encoder_rotor_mech_angle_rad(void)
{
    return g_ma600a.angle_rad * MOTOR_ENCODER_DIRECTION_SIGN;
}

/**
 * @brief  获取输出轴连续机械角（无跳变，可超过单圈）
 * @return 输出轴连续机械角 (rad)
 * @note   运行频率: 位置环和调试遥测更新时调用
 *          运行内容: 优先使用连续机械角观测值，再按减速比折算回输出轴
 */
float program_get_encoder_output_continuous_mech_angle_rad(void)
{
    float rotor_mech_angle_rad;
    if (g_encoder.speed_primed != 0U)
        rotor_mech_angle_rad = s_encoder_observer.continuous_mech_rad;
    else
        rotor_mech_angle_rad = program_get_encoder_rotor_mech_angle_rad();
    if ((!isfinite(rotor_mech_angle_rad)) || (MOTOR_GEAR_RATIO <= 0.0f)) return 0.0f;
    return rotor_mech_angle_rad / MOTOR_GEAR_RATIO;
}

/**
 * @brief  获取输出轴单圈机械角（归一化到 [0, 2π)）
 * @return 输出轴单圈机械角 (rad)
 * @note   运行频率: 位置环更新时调用
 */
float program_get_encoder_output_mech_angle_rad(void)
{
    return program_wrap_angle_0_2pi(program_get_encoder_output_continuous_mech_angle_rad());
}

/**
 * @brief  获取未经零位补偿的原始电角度
 * @return 原始电角度 (rad)
 * @note   运行频率: 对齐采样和故障前角度观察时调用
 *          运行内容: 由转子机械角和极对数直接换算电角度
 */
float program_get_encoder_raw_elec_angle_rad(void)
{
    return program_wrap_angle_0_2pi(program_get_encoder_rotor_mech_angle_rad() * MOTOR_POLE_PAIRS);
}

/**
 * @brief  获取完成零位补偿后的电角度
 * @return 对齐后的控制电角度 (rad)
 * @note   运行频率: 闭环控制和遥测更新时调用
 *          运行内容: 在原始电角度基础上扣除对齐得到的电角偏置
 */
float program_get_encoder_aligned_elec_angle_rad(void)
{
    return program_wrap_angle_0_2pi(program_get_encoder_raw_elec_angle_rad() - g_encoder.elec_offset_rad);
}

/**
 * @brief  提供当前控制使用的电角度
 * @return 控制电角度 (rad)
 * @note   运行频率: 快环每次执行控制时调用
 *          运行内容: 开环模式用积分器角度，否则用编码器对齐角度
 */
float program_get_control_elec_angle_rad(void)
{
    if (g_motor.control_angle_open_loop_enable != 0U)
        return program_wrap_angle_0_2pi(g_motor.theta_open_loop);
    return program_get_encoder_aligned_elec_angle_rad();
}

/**
 * @brief  更新控制角度开环状态
 * @note   运行频率: 快环内每次控制前调用
 *          运行内容: 开环模式下对 theta_open_loop 按固定速度积分
 */
void program_update_control_angle_open_loop_state(void)
{
    if ((g_motor.control_angle_open_loop_enable == 0U) || (g_encoder.align_done == 0U)) return;
    g_motor.theta_open_loop = program_wrap_angle_0_2pi(
        g_motor.theta_open_loop + g_motor.control_angle_open_loop_speed_elec * PROGRAM_FAST_LOOP_DT_S);
}

/* ── 编码器观测器 ── */
/**
 * @brief  复位编码器测速观测器
 * @note   运行频率: 上电初始化、读角失效或重新对齐时调用
 *          运行内容: 清空连续角、测速窗口、滤波器、挂起标志和相关遥测量
 */
void program_reset_encoder_observer(void)
{
    s_encoder_observer.last_sample_counter = 0U;
    g_encoder.speed_primed = 0U;
    g_encoder.speed_ready = 0U;
    s_encoder_observer.prev_mech_angle_rad = 0.0f;
    s_encoder_observer.continuous_mech_rad = 0.0f;
    s_encoder_observer.speed_window_start_mech_rad = 0.0f;
    s_encoder_observer.speed_window_sample_count = 0U;
    g_encoder.speed_raw_mech_rad_s = 0.0f;
    g_motor.speed_meas_mech_rad_s = 0.0f;
    g_motor.speed_meas_elec_rad_s = 0.0f;
    g_motor.position_meas_mech_deg = 0.0f;
    g_motor.position_meas_mech_rad = 0.0f;
    g_encoder.speed_loop_update_pending = 0U;
    g_control.speed_loop_dt_s = PROGRAM_FAST_LOOP_DT_S * 20.0f;
    g_control.position_meas_output_continuous_rad = 0.0f;
    filter_lpf_f32_init(&g_control.position_meas_lpf,
        program_lpf_alpha_from_cutoff_hz(12.0f, 0.005f), 0.0f);
    g_control.position_meas_lpf.initialized = 0U;
    filter_lpf_f32_init(&g_control.speed_meas_lpf,
        program_lpf_alpha_from_cutoff_hz(g_motor.speed_meas_lpf_cutoff_hz, 0.002f), 0.0f);
}

/**
 * @brief  对连续机械角做周期性重归一化，防止浮点精度丢失
 * @note   运行频率: 每次编码器角度更新后调用（每个快环周期）
 *          运行内容: 连续角超过 ±32 圈时，减去整数圈锚点；
 *                    窗口起点同步偏移，保持速度差分值不变
 */
static void program_renormalize_encoder_observer(void)
{
    /** 锚定圈数（整数圈） */
    float anchor_turns;
    /** 锚定弧度 = 圈数 × 2π */
    float anchor_rad;
    /** 连续角或窗口起点出现 NaN/Inf → 复位观测器 */
    if ((!isfinite(s_encoder_observer.continuous_mech_rad)) ||
        (!isfinite(s_encoder_observer.speed_window_start_mech_rad)) ||
        (!isfinite(g_encoder.speed_raw_mech_rad_s))) {
        program_reset_encoder_observer();
        return;
    }
    /** 连续角和窗口起点均 < 32 圈 → 无需重归一化 */
    if ((fabsf(s_encoder_observer.continuous_mech_rad) < PROGRAM_ENCODER_OBSERVER_RENORM_RAD) &&
        (fabsf(s_encoder_observer.speed_window_start_mech_rad) < PROGRAM_ENCODER_OBSERVER_RENORM_RAD))
        return;
    /** 取整数圈数作为锚点 */
    anchor_turns = floorf(s_encoder_observer.continuous_mech_rad / MOTOR_TWO_PI);
    anchor_rad = anchor_turns * MOTOR_TWO_PI;
    /** 连续角和窗口起点同时减去整数圈，保持差值不变 */
    s_encoder_observer.continuous_mech_rad -= anchor_rad;
    s_encoder_observer.speed_window_start_mech_rad -= anchor_rad;
}

/**
 * @brief  基于编码器角度更新机械速度测量
 * @note   运行频率: 每次获得有效 MA600A 角度样本后调用（快环 10kHz 内）
 *          运行内容: 维护连续机械角和测速窗口，窗口满后计算原始速度
 *                    → LPF 滤波 → 量化保护，最终输出机械/电角速度
 */
void program_update_speed_measurement(void)
{
    /** 当前拍机械角 (rad)，归一化到 [0, 2π) */
    float mech_angle_rad;
    /** 本拍角差 (rad)，处理 0°/360° 跳变 */
    float mech_delta_rad;
    /** 观测窗口时间跨度 (s)，窗口满时用于计算速度 */
    float observer_dt_s;
    /** 观测窗口内的拍数（可能因丢样本而 >20） */
    uint32_t observer_window_samples;
    /** 两次有效样本间的计数器增量（>1 表示有丢样本） */
    uint32_t sample_delta_count;

    /** 编码器数据无效 → 跳过本拍 */
    if (g_ma600a.data_valid == 0U) return;
    /** 无新样本到达（counter 未变）→ 跳过 */
    if (g_ma600a.sample_counter == s_encoder_observer.last_sample_counter) return;

    /** 获取当前拍转子机械角 (rad)，归一化到 [0, 2π) */
    mech_angle_rad = program_wrap_angle_0_2pi(program_get_encoder_rotor_mech_angle_rad());
    /** 两次有效样本间的计数器增量（>1 表示有丢样本） */
    sample_delta_count = g_ma600a.sample_counter - s_encoder_observer.last_sample_counter;
    if (sample_delta_count == 0U) return;
    /** 更新最新样本计数器 */
    s_encoder_observer.last_sample_counter = g_ma600a.sample_counter;

    /* ══ 首次采样：初始化观测器基准值 ══ */
    if (g_encoder.speed_primed == 0U) {
        s_encoder_observer.prev_mech_angle_rad = mech_angle_rad;          /** 记录初始角度 */
        s_encoder_observer.continuous_mech_rad = mech_angle_rad;          /** 连续角起点 */
        g_motor.position_meas_mech_rad = program_get_encoder_output_mech_angle_rad();
        s_encoder_observer.speed_window_start_mech_rad = mech_angle_rad;  /** 窗口起点 */
        s_encoder_observer.speed_window_sample_count = 0U;                /** 窗口计数清零 */
        g_encoder.speed_raw_mech_rad_s = 0.0f;
        g_encoder.speed_primed = 1U;                             /** 首次角度已记录 */
        g_encoder.speed_ready = 0U;                              /** 速度尚未就绪 */
        g_motor.speed_meas_mech_rad_s = 0.0f;
        g_motor.speed_meas_elec_rad_s = 0.0f;
        return;
    }

    /* ══ 正常累加：计算本拍角差 → 累加到连续角 → 窗口计数递增 ══ */
    /** 本拍角差（处理 0°/360° 跳变） */
    mech_delta_rad = program_wrap_delta_pm_pi(mech_angle_rad - s_encoder_observer.prev_mech_angle_rad);
    if (!isfinite(mech_delta_rad)) { program_reset_encoder_observer(); return; }
    s_encoder_observer.prev_mech_angle_rad = mech_angle_rad;
    s_encoder_observer.continuous_mech_rad += mech_delta_rad;             /** 连续角累加 */
    s_encoder_observer.speed_window_sample_count += sample_delta_count;   /** 窗口计数累加 */
    /** 防止连续角无限增长导致浮点精度丢失 */
    program_renormalize_encoder_observer();
    /** 同步更新位置测量值 */
    g_motor.position_meas_mech_rad = program_get_encoder_output_mech_angle_rad();
    g_motor.position_meas_mech_deg = program_wrap_angle_0_360_deg(
        program_rad_to_deg(g_motor.position_meas_mech_rad));

    /** 窗口未满 20 拍 → 继续积累，不计算速度 */
    if (s_encoder_observer.speed_window_sample_count < 20U) return;

    /* ══ 窗口满：差分计算原始速度 ══ */
    observer_window_samples = s_encoder_observer.speed_window_sample_count;
    /** 窗口时间跨度 = 快环周期 × 拍数 */
    observer_dt_s = PROGRAM_FAST_LOOP_DT_S * (float)observer_window_samples;
    if ((!isfinite(observer_dt_s)) || (observer_dt_s <= 0.0f))
        { program_reset_encoder_observer(); return; }

    /** 速度 = (终点角度 - 起点角度) / 时间 */
    g_encoder.speed_raw_mech_rad_s =
        (s_encoder_observer.continuous_mech_rad - s_encoder_observer.speed_window_start_mech_rad) / observer_dt_s;
    if (!isfinite(g_encoder.speed_raw_mech_rad_s)) { program_reset_encoder_observer(); return; }

    /** 重置窗口：起点拉到当前连续角，计数清零，开启下一窗口 */
    s_encoder_observer.speed_window_start_mech_rad = s_encoder_observer.continuous_mech_rad;
    s_encoder_observer.speed_window_sample_count = 0U;
    g_control.speed_loop_dt_s = observer_dt_s;                           /** 速度环实际调用周期 */
    /** 更新 LPF 系数，匹配当前的观测周期 */
    g_control.speed_meas_lpf.alpha = program_lpf_alpha_from_cutoff_hz(
        g_motor.speed_meas_lpf_cutoff_hz, observer_dt_s);

    /* ══ 首个窗口：初始化 LPF → 速度测量就绪 ══ */
    if (g_encoder.speed_ready == 0U) {
        filter_lpf_f32_init(&g_control.speed_meas_lpf, g_control.speed_meas_lpf.alpha,
                            g_encoder.speed_raw_mech_rad_s);
        g_encoder.speed_ready = 1U;                              /** 速度测量就绪，闭环可启动 */
    }

    /** LPF 滤波原始速度 */
    g_motor.speed_meas_mech_rad_s = filter_lpf_f32_update(&g_control.speed_meas_lpf,
        g_encoder.speed_raw_mech_rad_s);
    if (!isfinite(g_motor.speed_meas_mech_rad_s)) { program_reset_encoder_observer(); return; }

    /** 低速量化保护：接近零速时强制归零，消除编码器量化噪声 */
    g_motor.speed_meas_mech_rad_s = program_apply_speed_quantization_guard(
        g_motor.speed_meas_mech_rad_s, observer_window_samples);
    /** 机械角速度 → 电角速度（× 极对数） */
    g_motor.speed_meas_elec_rad_s = g_motor.speed_meas_mech_rad_s * MOTOR_POLE_PAIRS;
    /** 通知速度环：本次窗口已产出新速度，可执行一次速度 PI */
    g_encoder.speed_loop_update_pending = 1U;
}

/* ── 编码器零位对齐 ── */

/**
 * @brief  复位编码器零位对齐结果
 * @note   运行频率: 重新对齐、停机或读角失效时调用
 *          运行内容: 清空零位偏置和对齐完成标志
 */
void program_reset_encoder_alignment(void)
{
    g_encoder.align_done = 0U;
    g_encoder.align_counter = 0U;
    g_encoder.elec_offset_rad = 0.0f;
    g_motor.align_done = 0U;
}

/**
 * @brief  复位编码器对齐过程的运行时累积量
 * @note   运行频率: 开始重新对齐或完成对齐后调用
 *          运行内容: 清空对齐计数器以及 sin/cos 平均所需的累积和
 */
void program_reset_encoder_align_runtime(void)
{
    g_encoder.align_counter = 0U;
    s_encoder_align.sum_sin = 0.0f;
    s_encoder_align.sum_cos = 0.0f;
    s_encoder_align.sample_count = 0U;
}

/**
 * @brief  在对齐保持阶段采集编码器电角度样本
 * @note   运行频率: 零位对齐保持期间的每个快环调用
 *          运行内容: 只在最后 512 拍采样窗口内累积电角度的 sin/cos，
 *                    用于后续 atan2 求平均角，降低编码器抖动影响
 */
void program_capture_encoder_alignment_sample(void)
{
    uint32_t sample_window_start_tick;
    float raw_theta_elec;
    sample_window_start_tick = PROGRAM_ALIGN_HOLD_TICKS - PROGRAM_ALIGN_SAMPLE_WINDOW_TICKS;
    if (g_encoder.align_counter < sample_window_start_tick) return;
    raw_theta_elec = program_get_encoder_raw_elec_angle_rad();
    s_encoder_align.sum_sin += sinf(raw_theta_elec);
    s_encoder_align.sum_cos += cosf(raw_theta_elec);
    s_encoder_align.sample_count++;
}

/**
 * @brief  计算编码器零位对齐得到的平均电角度
 * @return 对齐采样得到的电角度 (rad)
 * @note   运行频率: 对齐结束时调用
 *          运行内容: 基于累积的 sin/cos 用 atan2 求平均方向角，
 *                    抗噪声能力远优于直接平均角度
 */
float program_get_encoder_alignment_angle_rad(void)
{
    float raw_theta_elec;
    if (s_encoder_align.sample_count == 0U)
        return program_get_encoder_raw_elec_angle_rad();
    raw_theta_elec = atan2f(s_encoder_align.sum_sin, s_encoder_align.sum_cos);
    return program_wrap_angle_0_2pi(raw_theta_elec);
}

/* ── 速度斜坡 ── */

/**
 * @brief  按设定加速度更新实际速度给定（速度斜坡）
 * @note   运行频率: 快环内每次速度控制前调用
 *          运行内容: 非位置环模式下自动 rpm→rad/s 转换，
 *                    再以 100 rad/s? 斜率逐步逼近目标，减少转矩冲击；
 *                    同步输出电角速度和开环速度
 */
void program_update_speed_reference_ramp(void)
{
    float speed_step_rad_s, speed_delta_rad_s;
    if (g_motor.position_loop_enable == 0U)
        g_motor.speed_ref_mech_rad_s = program_rpm_to_rad_s(g_motor.speed_ref_mech_rpm);
    speed_step_rad_s = PROGRAM_SPEED_REF_RAMP_RAD_S2 * PROGRAM_FAST_LOOP_DT_S;
    speed_delta_rad_s = g_motor.speed_ref_mech_rad_s - g_motor.speed_ref_mech_applied_rad_s;
    speed_delta_rad_s = program_clamp_f32(speed_delta_rad_s, -speed_step_rad_s, speed_step_rad_s);
    g_motor.speed_ref_mech_applied_rad_s += speed_delta_rad_s;
    g_motor.speed_ref_elec_rad_s = g_motor.speed_ref_mech_applied_rad_s * MOTOR_POLE_PAIRS;
    g_motor.open_loop_speed_elec = g_motor.speed_ref_elec_rad_s;
}
