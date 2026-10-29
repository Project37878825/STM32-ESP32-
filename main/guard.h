/**
 * @file guard.h
 * @brief 环境守卫状态机：入侵报警 + 低水位报警
 *
 * 两个报警源，任一成立即报警（蜂鸣间歇 + 蓝灯闪 + 云事件）：
 *
 * 1) 入侵（受布防条件约束）：入侵 = 布防 && 有人
 *      白天 + 有人  -> 不报警（默认仅夜间布防，GUARD_ALWAYS_ARM=1 可改全天）
 *      晚上 + 无人  -> 不报警
 *      晚上 + 有人  -> 报警，并语音播报"警告 危险区域请立即离开"（需接语音模块）
 *
 * 2) 低水位（不受布防约束，昼夜都判）：水位 < ENV_WATER_PRESENT_PCT 持续
 *    WATER_LOW_CONFIRM_MS 即报警。这是设备自身状态，不播语音。
 *
 * 工程保护（否则实际使用会误报/抖报）：
 *   - 光敏滞回：持续暗/亮 GUARD_DARK_CONFIRM_MS 才切换昼夜，避免黄昏抖动
 *   - PIR 去抖：持续有人 GUARD_MOTION_CONFIRM_MS 才算入侵，滤瞬时误触发
 *   - PIR 上电稳定期 GUARD_PIR_WARMUP_MS 内不报警（HC-SR501 上电会乱触发）
 *   - 低水位去抖：持续低于阈值 WATER_LOW_CONFIRM_MS 才报，滤水面波动
 *   - 水位未标定（env_water_percent_cached 返回 -1）时不参与判定，避免开机误报
 */

#ifndef __GUARD_H__
#define __GUARD_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 1 = 全天布防（忽略光敏）；0 = 仅夜间布防（默认）
#define GUARD_ALWAYS_ARM          0

// 判定"天黑/天亮"所需持续时长(ms)
#define GUARD_DARK_CONFIRM_MS     2000
// 判定"有人"所需 PIR 持续时长(ms)
#define GUARD_MOTION_CONFIRM_MS   1000
// 注：目前没有"人走后再延迟 N 秒才解除"的逻辑——PIR 一旦无人（去抖计数归零）
// 入侵条件立即失效，报警随即解除。原先这里有个 GUARD_RELEASE_MS 宏但从未被引用，
// 已随死代码清理删除。要加解除延时，就在 guard.c 里用 still_ms 累计后与阈值比较。
// PIR 上电稳定期(ms)，期间不报警。HC-SR501 上电头十几秒会乱触发。
#define GUARD_PIR_WARMUP_MS       5000

/**
 * @brief 启动守卫状态机任务
 *        必须在 env_sensors_init() 之后调用
 * @return 0 成功
 */
int guard_init(void);

/** @brief 当前是否处于报警状态 */
bool guard_is_alarming(void);

#ifdef __cplusplus
}
#endif

#endif /* __GUARD_H__ */
