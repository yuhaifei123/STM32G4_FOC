#ifndef program_svpwm_h
#define program_svpwm_h

#include <stdint.h>

/**
 * 开环拖动 SVPWM 输出
 * 积分器生成电角度 → 反Park → Clarke反 → 三次谐波注入 → 占空比 → TIM1
 */
void program_open_loop_svpwm(void);

/**
 * 闭环拖动 FOC 主逻辑
 * ①电流换算 → ②Clarke → ③电角 → ④Park → ⑤速度PI → ⑥vbus → ⑦电流PI → ⑧限幅 → ⑨反Park+SVPWM
 * @param ia_raw       A 相 ADC 原始值
 * @param ib_raw       B 相 ADC 原始值
 * @param ia_offset    A 相零偏
 * @param ib_offset    B 相零偏
 * @param elec_offset  电角偏置
 */
void program_closed_loop_svpwm(uint16_t ia_raw, uint16_t ib_raw);

#endif // !program_svpwm_h
