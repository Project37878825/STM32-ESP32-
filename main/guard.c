/**
 * @file guard.c
 * @brief 夜间入侵守卫状态机
 *
 * 100ms 一个 tick，用"持续时长计数器"实现滞回与去抖，而不是直接看瞬时电平。
 * 报警输出：蜂鸣器 300ms 间歇 + 蓝灯同步闪（复用 led_alarm 的手动模式）+
 *           入侵时语音播报"警告 警告 危险区域请立即离开"（需外接语音模块）+
 *           向巴法云报警主题发一条开始/结束事件。
 * 
 * 新增：水位低于 20% 时触发低水位报警（与入侵报警独立，任一触发即报警）
 */

#include "guard.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "env_sensors.h"
#include "led_alarm.h"
#include "cloud_uplink.h"

static const char *TAG = "GUARD";

#define GUARD_TICK_MS        100
#define ALARM_TOGGLE_MS      300   // 蜂鸣/蓝灯 的间歇半周期

// 低水位去抖：持续低于阈值多久才触发报警（避免水面波动导致误报）
#define WATER_LOW_CONFIRM_MS   2000

// 人一直不走时，语音警告的重复播报间隔。
// 不重复喊会被当成背景噪音忽略，喊太密又像故障；10s 是常见警示牌的节奏。
#define VOICE_REPEAT_MS        10000

static volatile bool s_alarming = false;
// 报警原因标记（用于日志区分，实际输出都是蜂鸣+蓝灯+云事件）
static volatile bool s_alarm_water = false;
static volatile bool s_alarm_intrusion = false;

static void publish_alarm(int state)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"type\":\"alarm\",\"state\":%d}", state);
    if (cloud_uplink_publish_json(BEMFA_ALARM_TOPIC, buf) != 0) {
        ESP_LOGW(TAG, "报警事件未能上云（当前未连接）");
    }
}

static void guard_task(void *arg)
{
    uint32_t dark_ms = 0, bright_ms = 0;     // 光敏滞回计数
    uint32_t motion_ms = 0, still_ms = 0;    // PIR 去抖 / 解除计数
    uint32_t uptime_ms = 0;                  // PIR 上电稳定期
    uint32_t toggle_ms = 0;                  // 报警输出间歇计数
    uint32_t log_ms = 0;                     // 状态日志节流计数
    uint32_t water_low_ms = 0;               // 低水位持续计数
    uint32_t voice_ms = 0;                   // 语音重复播报计时
    bool night = false;
    bool alarming = false;
    bool out_on = false;

    ESP_LOGI(TAG, "守卫启动: 模式=%s, 水位报警阈值=%d%%", 
             GUARD_ALWAYS_ARM ? "全天布防" : "仅夜间布防",
             ENV_WATER_PRESENT_PCT);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(GUARD_TICK_MS));
        uptime_ms += GUARD_TICK_MS;

        /* ---- 昼夜判定（滞回） ---- */
        if (env_is_dark()) {
            dark_ms += GUARD_TICK_MS;
            bright_ms = 0;
        } else {
            bright_ms += GUARD_TICK_MS;
            dark_ms = 0;
        }
        if (dark_ms >= GUARD_DARK_CONFIRM_MS) {
            night = true;
        }
        if (bright_ms >= GUARD_DARK_CONFIRM_MS) {
            night = false;
        }

        /* ---- 布防与入侵判定 ---- */
        bool armed = (GUARD_ALWAYS_ARM == 1) || night;
        bool motion = env_motion() && (uptime_ms >= GUARD_PIR_WARMUP_MS);
        if (motion) {
            motion_ms += GUARD_TICK_MS;
            still_ms = 0;
        } else {
            still_ms += GUARD_TICK_MS;
            motion_ms = 0;
        }
        bool intrusion = armed && (motion_ms >= GUARD_MOTION_CONFIRM_MS);

        /* ---- 水位判定（新增） ---- */
        // 水位传感器返回 -1 表示标定中/读取失败，此时不触发报警
        int water_pct = env_water_percent_cached();
        bool water_low = false;
        if (water_pct >= 0) {
            water_low = (water_pct < ENV_WATER_PRESENT_PCT);
        }
        
        // 低水位去抖：持续低水位超过 WATER_LOW_CONFIRM_MS 才确认报警
        if (water_low) {
            water_low_ms += GUARD_TICK_MS;
        } else {
            water_low_ms = 0;
        }
        bool water_alarm = (water_low_ms >= WATER_LOW_CONFIRM_MS);

        /* ---- 低频状态日志：现场验证真值表与极性用 ---- */
        log_ms += GUARD_TICK_MS;
        if (log_ms >= 5000) {
            log_ms = 0;
            // pir/light 是原始电平，motion 是过了稳定期+去抖后的结果，
            // 两者对照就能区分"传感器没信号"和"逻辑在 gating"
            ESP_LOGI(TAG, "状态: night=%d armed=%d pir=%d light=%d motion=%d water=%d%% alarm=%d%s%s",
                     night ? 1 : 0, armed ? 1 : 0,
                     env_motion() ? 1 : 0, env_is_dark() ? 0 : 1,
                     (motion_ms >= GUARD_MOTION_CONFIRM_MS) ? 1 : 0,
                     water_pct,
                     alarming ? 1 : 0,
                     water_alarm ? " [低水位]" : "",
                     intrusion ? " [入侵]" : "");
        }

        /* ---- 状态迁移 ---- */
        // 任一条件触发都进入报警状态
        bool should_alarm = intrusion || water_alarm;
        
        if (!alarming && should_alarm) {
            alarming = true;
            s_alarming = true;
            s_alarm_intrusion = intrusion;
            s_alarm_water = water_alarm;
            
            if (intrusion && water_alarm) {
                ESP_LOGW(TAG, "复合报警触发（入侵+低水位）: night=%d, motion持续=%lums, 水位=%d%%",
                         night ? 1 : 0, (unsigned long)motion_ms, water_pct);
            } else if (intrusion) {
                ESP_LOGW(TAG, "入侵报警触发（night=%d, motion持续=%lums）",
                         night ? 1 : 0, (unsigned long)motion_ms);
            } else {
                ESP_LOGW(TAG, "低水位报警触发（水位=%d%%, 持续=%lums）",
                         water_pct, (unsigned long)water_low_ms);
            }

            // 只有入侵才播报语音：低水位是设备自身状态，
            // 对着空屋子喊"危险区域请立即离开"没有意义。
            if (intrusion) {
                if (env_voice_play(ENV_VOICE_IDX_WARNING) == 0) {
                    ESP_LOGW(TAG, "语音播报: 警告 警告 危险区域请立即离开");
                } else {
                    ESP_LOGW(TAG, "未接语音模块(ENV_VOICE_ENABLE=0)，本次仅蜂鸣+蓝灯");
                }
                voice_ms = 0;
            }
            publish_alarm(1);
        } else if (alarming && !should_alarm) {
            // 两个条件都解除才解除报警
            alarming = false;
            s_alarming = false;
            s_alarm_intrusion = false;
            s_alarm_water = false;
            ESP_LOGI(TAG, "报警解除（入侵解除=%d, 水位恢复=%d%%）", 
                     !intrusion, water_pct);
            publish_alarm(0);
            env_buzzer_set(false);
            led_alarm_set_manual(-1);  // 蓝灯交还给称重报警逻辑
            voice_ms = 0;
        }

        /* ---- 报警输出：蜂鸣 + 蓝灯 同步间歇 ---- */
        if (alarming) {
            toggle_ms += GUARD_TICK_MS;
            if (toggle_ms >= ALARM_TOGGLE_MS) {
                toggle_ms = 0;
                out_on = !out_on;
                env_buzzer_set(out_on);
                led_alarm_set_manual(out_on ? 1 : 0);
            }

            // 人还在（入侵条件仍成立）就周期性重播警告，直到报警解除
            if (intrusion) {
                voice_ms += GUARD_TICK_MS;
                if (voice_ms >= VOICE_REPEAT_MS) {
                    voice_ms = 0;
                    env_voice_play(ENV_VOICE_IDX_WARNING);
                }
            }
        }
    }
}

int guard_init(void)
{
    if (xTaskCreate(guard_task, "guard", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "守卫任务创建失败");
        return -1;
    }
    return 0;
}

bool guard_is_alarming(void)
{
    return s_alarming;
}
