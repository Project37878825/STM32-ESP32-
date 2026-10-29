/**
 * @file env_sensors.h
 * @brief 光敏 / 人体红外 / 蜂鸣器 纯驱动（不含任何业务逻辑）
 *
 * 业务逻辑（何时报警）在 guard.c 里。本文件只负责读电平和驱动蜂鸣器，
 * 保证守卫状态机是唯一拥有蜂鸣器的模块，不会互相抢。
 *
 * 接线（ESP32 DevKit）：
 *   光敏 DO    -> GPIO4
 *   蜂鸣器 I/O -> GPIO18
 *   PIR OUT    -> GPIO5   （GPIO5 是 strapping 脚，但仅作输入、复位采样已结束，安全）
 *   水位 AO    -> GPIO34  （仅输入、ADC1_CH6）
 *   语音模块 IO -> GPIO25 （可选，需 ENV_VOICE_ENABLE=1）
 */

#ifndef __ENV_SENSORS_H__
#define __ENV_SENSORS_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENV_LIGHT_PIN    4
#define ENV_BUZZER_PIN   18
#define ENV_PIR_PIN      5

// 水位传感器：GPIO34 = ADC1_CHANNEL_6，仅输入脚，按模拟量读取。
// 刻度为会话内自动两点标定：开机后观察到的最低 raw 记为 0%、最高 raw 记为 100%。
// 因此首次浸到最高水位、再排干一次后刻度即自动变准；无需手填任何数字。
#define ENV_WATER_PIN    34
// 低于该百分比视为"低水位"，触发低水位报警（蜂鸣+蓝灯闪）
#define ENV_WATER_PRESENT_PCT  20

// DHT11 温湿度：单线协议，GPIO19。采样率上限 1Hz，监测任务按 10s 周期读
#define ENV_DHT11_PIN    19

// DHT11 现场排查开关：
//   1 = 2s 读一次、每次都打印、失败时打印卡在哪一步 + 原始 5 字节
//   0 = 10s 读一次、数值有明显变化才打印（日常使用）
// 排查完记得改回 0，否则串口会被刷屏。
#define ENV_DHT11_DIAG   1

// 光敏模块极性：多数模块"天黑时 DO 输出高"。
// 若你的模块相反（天黑输出低），改成 0。开机日志会打印当前电平供你核对。
#define ENV_LIGHT_DARK_IS_HIGH  1

// 有源蜂鸣器高电平响；若你的模块低电平响改成 0
#define ENV_BUZZER_ON_LEVEL     1

/* ---- 语音播报模块（一线脉冲触发） ----
 * 有源蜂鸣器只能发单频"滴"声，物理上播放不了人声。要播报"警告 危险区域
 * 请立即离开"，必须外接语音模块：音频存在模块自己的 flash 里，ESP32 只负责触发。
 *
 * 当前驱动适配 IO 一线脉冲触发型（JQ8900-16P 一线模式、DY-SV17F IO 触发、
 * WT588E 等），规则是"发 N 个低电平脉冲 = 播放模块内第 N 段语音"。
 * 若你买的是 UART 串口型（DFPlayer Mini、JQ8900 串口模式），协议完全不同，
 * 把型号告诉我，我再写串口版。
 *
 * ENV_VOICE_ENABLE 保持 0 时（=没接模块）：代码照常编译，入侵报警退化为蜂鸣+蓝灯。
 * 接好模块后改成 1 即可，其余逻辑不用动。
 */
#define ENV_VOICE_ENABLE          0
#define ENV_VOICE_PIN             25   // GPIO25 空闲（HX711 占 26/27），非 strapping 脚
#define ENV_VOICE_PULSE_LOW_US    500  // 触发脉冲低电平宽度；模块手册要求不同就改这里
#define ENV_VOICE_PULSE_GAP_US    500  // 两个脉冲之间的高电平间隔
#define ENV_VOICE_IDX_WARNING     1    // 第 1 段语音 = 事先录/拷好的警告词

/**
 * @brief 配置光敏/PIR/蜂鸣器/语音触发脚的 GPIO 方向、初始化水位 ADC，
 *        并启动水位+温湿度监测任务（不起业务判断）
 * @return 0 成功
 */
int env_sensors_init(void);

/** @brief 当前是否天黑（按 ENV_LIGHT_DARK_IS_HIGH 极性换算） */
bool env_is_dark(void);

/** @brief 当前是否检测到人体移动（PIR 高电平 = 有人） */
bool env_motion(void);

/** @brief 蜂鸣器开关 */
void env_buzzer_set(bool on);

/**
 * @brief 播放语音模块内第 index 段语音（一线脉冲触发）
 * @param index 段号，从 1 开始；ENV_VOICE_IDX_WARNING 即警告词
 * @return 0 已发出触发脉冲；-1 未启用语音模块（ENV_VOICE_ENABLE=0）或 index 非法
 */
int env_voice_play(int index);

/**
 * @brief 读水位百分比（ADC 原始值换算 0-100）
 * @return 0-100；-1 读取失败
 */
int env_water_percent(void);

/**
 * @brief 读监测任务缓存的水位百分比（不做 ADC 转换，供高频调用方如守卫使用）
 * @return 0-100；-1 尚无有效读数
 */
int env_water_percent_cached(void);

/**
 * @brief 读 DHT11 温湿度（校验和通过才算成功，内部重试 3 次）
 *        采用轮询+校验和而非关中断死等：关 4ms 中断会干扰 Wi-Fi 时序，
 *        而被抢占导致的错读会被校验和抓住并重试
 * @return 0 成功；-1 三次都失败（接线/时序/被抢占）
 */
int env_read_dht11(float *temp_c, float *hum_pct);

#ifdef __cplusplus
}
#endif

#endif /* __ENV_SENSORS_H__ */
