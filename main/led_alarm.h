/**
 * @file led_alarm.h
 * @brief 报警灯与阈值的外部控制接口（供智能体下行命令使用）
 */

#ifndef __LED_ALARM_H__
#define __LED_ALARM_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置蓝灯手动模式
 * @param mode -1 = 自动（按重量阈值闪烁）；0 = 强制灭；1 = 强制常亮
 */
void led_alarm_set_manual(int mode);

/**
 * @brief 运行时修改超重报警阈值（克）
 */
void alarm_set_threshold_g(float grams);
float alarm_get_threshold_g(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_ALARM_H__ */
