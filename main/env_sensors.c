/**
 * @file env_sensors.c
 * @brief 光敏 / PIR / 蜂鸣器 纯驱动实现
 *
 * 只读电平和驱动蜂鸣器，不做任何"何时报警"的判断——那属于 guard.c。
 */

#include "env_sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "ENV";

static adc_oneshot_unit_handle_t s_adc1 = NULL;   // GPIO34 = ADC1_CHANNEL_6

// 水位会话内自动两点标定：观察到的最低/最高 raw 即 0%/100%
static int s_water_min = -1;
static int s_water_max = -1;
static volatile int s_water_pct_cached = -1;

// raw 跨度小于此值视为尚未标定（例如开机后还没浸过水），此时返回 -1 不报百分比，
// 避免"跨度为 0 时任何读数都算 0%" 导致开机就误报低水位
#define ENV_WATER_MIN_SPAN  200

// 定义在文件末尾（依赖 env_water_percent），此处前向声明
static void env_water_task(void *arg);

int env_sensors_init(void)
{
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << ENV_LIGHT_PIN) | (1ULL << ENV_PIR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    gpio_config_t out_conf = {
        .pin_bit_mask = 1ULL << ENV_BUZZER_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
#if ENV_VOICE_ENABLE
    out_conf.pin_bit_mask |= 1ULL << ENV_VOICE_PIN;
#endif
    gpio_config(&out_conf);
    env_buzzer_set(false);
#if ENV_VOICE_ENABLE
    gpio_set_level(ENV_VOICE_PIN, 1);   // 一线触发空闲态为高，脉冲拉低
    ESP_LOGI(TAG, "语音模块就绪: 触发脚=GPIO%d, 警告语音=第%d段",
             ENV_VOICE_PIN, ENV_VOICE_IDX_WARNING);
#endif

    // 水位：GPIO34 = ADC1_CHANNEL_6，仅输入脚，用 oneshot 单次转换读取
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&unit_cfg, &s_adc1) != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 单元创建失败");
        s_adc1 = NULL;
    } else {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,   // 0~3.3V 满量程
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_oneshot_config_channel(s_adc1, ADC_CHANNEL_6, &chan_cfg);
    }

    // 打印当前电平/水位，方便核对极性与接线
    ESP_LOGI(TAG, "环境感知就绪: 光敏=GPIO%d(电平%d) PIR=GPIO%d(电平%d) 蜂鸣=GPIO%d 水位=%d%%",
             ENV_LIGHT_PIN, gpio_get_level(ENV_LIGHT_PIN),
             ENV_PIR_PIN, gpio_get_level(ENV_PIR_PIN), ENV_BUZZER_PIN,
             env_water_percent());

    // 水位监测：变化才打印，避免刷屏
    if (xTaskCreate(env_water_task, "env_water", 2048, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "水位监测任务创建失败");
        return -1;
    }
    return 0;
}

bool env_is_dark(void)
{
    return gpio_get_level(ENV_LIGHT_PIN) == ENV_LIGHT_DARK_IS_HIGH;
}

bool env_motion(void)
{
    return gpio_get_level(ENV_PIR_PIN) == 1;
}

void env_buzzer_set(bool on)
{
    gpio_set_level(ENV_BUZZER_PIN, on ? ENV_BUZZER_ON_LEVEL : !ENV_BUZZER_ON_LEVEL);
}

// 一线脉冲触发：发 index 个低电平脉冲 = 播放模块内第 index 段语音。
// 全程阻塞 index*(LOW+GAP) 微秒（几毫秒），可以在任务里直接调。
// 未启用语音模块时返回 -1，调用方据此退化为蜂鸣+灯。
int env_voice_play(int index)
{
#if ENV_VOICE_ENABLE
    if (index <= 0) {
        return -1;
    }
    for (int i = 0; i < index; i++) {
        gpio_set_level(ENV_VOICE_PIN, 0);
        esp_rom_delay_us(ENV_VOICE_PULSE_LOW_US);
        gpio_set_level(ENV_VOICE_PIN, 1);
        esp_rom_delay_us(ENV_VOICE_PULSE_GAP_US);
    }
    return 0;
#else
    (void)index;
    return -1;
#endif
}

// 16 次平均降噪：ESP32 ADC 单次值噪声大，平均后才可用
static int water_avg_raw(void)
{
    if (s_adc1 == NULL) {
        return -1;
    }
    long sum = 0;
    int raw = 0;
    for (int i = 0; i < 16; i++) {
        if (adc_oneshot_read(s_adc1, ADC_CHANNEL_6, &raw) != ESP_OK) {
            return -1;
        }
        sum += raw;
    }
    return (int)(sum / 16);
}

int env_water_percent(void)
{
    int raw = water_avg_raw();
    if (raw < 0) {
        return -1;
    }

    // 会话内自动两点标定：观察到的最低 raw = 0%，最高 raw = 100%。
    // 浸水一次、排干一次后刻度即自动变准，无需手填任何数字。
    if (s_water_min < 0) {
        s_water_min = raw;
        s_water_max = raw;
    }
    if (raw < s_water_min) {
        s_water_min = raw;
    }
    if (raw > s_water_max) {
        s_water_max = raw;
    }

    int span = s_water_max - s_water_min;
    if (span < ENV_WATER_MIN_SPAN) {
        s_water_pct_cached = -1;   // 标定中：不报百分比，也不触发低水位报警
        return -1;
    }

    int pct = (raw - s_water_min) * 100 / span;
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    s_water_pct_cached = pct;
    return pct;
}

int env_water_percent_cached(void)
{
    return s_water_pct_cached;
}

/* ---------------- DHT11 单线协议 ---------------- */

// 等待电平变为 level，超时返回 false。轮询粒度 1us
static bool dht11_wait_level(int level, int timeout_us)
{
    int waited = 0;
    while (gpio_get_level(ENV_DHT11_PIN) == level) {
        if (waited >= timeout_us) {
            return false;
        }
        esp_rom_delay_us(1);
        waited++;
    }
    return true;
}

// 失败阶段码：接线问题和时序问题的排查方向完全不同，只返回 -1 等于把线索全丢了
enum {
    DHT11_OK             = 0,
    DHT11_NO_PULL_LOW    = 1,
    DHT11_NO_PULL_HIGH   = 2,
    DHT11_NO_DATA_START  = 3,
    DHT11_BIT_TIMEOUT    = 4,
    DHT11_BIT_STUCK_HIGH = 5,
    DHT11_CHECKSUM       = 6,
};

// 解码期间把本任务临时提到这个优先级。
// 40 位数据的解码窗口约 4ms，中途被守卫任务（优先级 4）抢占就会时序错乱、校验和失败。
// 选 10 而不是 configMAX_PRIORITIES：只要压过应用层任务就够，
// 压过 Wi-Fi(23)/定时器(22) 这些系统任务反而会破坏射频时序。
#define DHT11_READ_PRIO  10

static int s_dht_stage = DHT11_OK;   // 最后一次失败卡在哪一步
static int s_dht_bit = -1;           // 失败发生在第几位（仅 BIT_* 阶段有意义）
static int s_dht_fails = 0;          // 连续失败次数
static uint8_t s_dht_raw[5] = {0};   // 最后一次读到的原始字节（校验和错时最有用）

#if ENV_DHT11_DIAG
static const char *dht11_stage_str(int stage)
{
    switch (stage) {
    case DHT11_NO_PULL_LOW:
        return "传感器没响应(未拉低) -> 查 VCC/GND/DATA 接线，DATA 需 4.7k~10k 上拉到 3.3V";
    case DHT11_NO_PULL_HIGH:
        return "响应波形不完整(拉低后没拉高) -> 多为供电不稳或杜邦线太长";
    case DHT11_NO_DATA_START:
        return "响应结束但没进入数据位";
    case DHT11_BIT_TIMEOUT:
        return "数据位中途超时 -> 时序被干扰（抢占/中断）";
    case DHT11_BIT_STUCK_HIGH:
        return "数据位高电平超 100us 不结束 -> DATA 被外部拉住或上拉过强";
    case DHT11_CHECKSUM:
        return "40 位读完但校验和错 -> 读取被打断，或上拉太弱导致 0/1 判决偏";
    default:
        return "未知";
    }
}
#endif

// 读一次 40 位原始数据。返回 0 成功；失败时通过 stage/bit 报告卡在哪一步
static int dht11_measure(uint8_t out[5], int *stage, int *bit)
{
    for (int i = 0; i < 5; i++) {
        out[i] = 0;
    }
    *stage = DHT11_OK;
    *bit = -1;

    // 起始信号：拉低 20ms 后释放，转输入等传感器响应
    gpio_set_direction(ENV_DHT11_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENV_DHT11_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(ENV_DHT11_PIN, 1);
    gpio_set_direction(ENV_DHT11_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENV_DHT11_PIN, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(30);

    // 传感器响应：80us 低 + 80us 高，然后进入数据位
    if (!dht11_wait_level(0, 100)) { *stage = DHT11_NO_PULL_LOW;   return -1; }
    if (!dht11_wait_level(1, 100)) { *stage = DHT11_NO_PULL_HIGH;  return -1; }
    if (!dht11_wait_level(0, 100)) { *stage = DHT11_NO_DATA_START; return -1; }

    for (int i = 0; i < 40; i++) {
        // 每位以 50us 低开始，高电平时长决定 0/1（~27us=0，~70us=1）
        if (!dht11_wait_level(1, 60)) {
            *stage = DHT11_BIT_TIMEOUT;
            *bit = i;
            return -1;
        }
        int high_us = 0;
        while (gpio_get_level(ENV_DHT11_PIN) == 1) {
            if (high_us >= 100) {
                *stage = DHT11_BIT_STUCK_HIGH;
                *bit = i;
                return -1;
            }
            esp_rom_delay_us(1);
            high_us++;
        }
        out[i / 8] = (uint8_t)((out[i / 8] << 1) | (high_us > 40 ? 1 : 0));
    }
    return 0;
}

int env_read_dht11(float *temp_c, float *hum_pct)
{
    uint8_t d[5];
    UBaseType_t saved_prio = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, DHT11_READ_PRIO);

    for (int attempt = 0; attempt < 3; attempt++) {
        int stage = DHT11_OK, bit = -1;
        if (dht11_measure(d, &stage, &bit) != 0) {
            s_dht_stage = stage;
            s_dht_bit = bit;
            for (int i = 0; i < 5; i++) {
                s_dht_raw[i] = d[i];
            }
            continue;
        }
        // 校验和：前 4 字节之和 == 第 5 字节
        uint8_t sum = (uint8_t)(d[0] + d[1] + d[2] + d[3]);
        if (sum != d[4]) {
            s_dht_stage = DHT11_CHECKSUM;
            s_dht_bit = -1;
            for (int i = 0; i < 5; i++) {
                s_dht_raw[i] = d[i];
            }
            continue;  // 被抢占或干扰导致错读，重试
        }
        if (hum_pct != NULL) {
            *hum_pct = (float)d[0] + (float)d[1] / 10.0f;
        }
        if (temp_c != NULL) {
            float t = (float)(d[2] & 0x7F) + (float)d[3] / 10.0f;
            *temp_c = (d[2] & 0x80) ? -t : t;
        }
        vTaskPrioritySet(NULL, saved_prio);
        s_dht_stage = DHT11_OK;
        s_dht_fails = 0;
        return 0;
    }

    vTaskPrioritySet(NULL, saved_prio);
    s_dht_fails++;
    return -1;
}

// 水位/温湿度监测任务：水位每 2s，温湿度按 ENV_DHT11_DIAG 决定周期与打印策略
static void env_water_task(void *arg)
{
    int last_logged = -100;
#if !ENV_DHT11_DIAG
    // 只有日常模式需要"和上次比有没有变化"，排查模式每次都打印
    float last_temp = -999.0f, last_hum = -999.0f;
    int tick = 0;
#endif

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
#if !ENV_DHT11_DIAG
        tick++;
#endif

        int raw = water_avg_raw();
        int pct = env_water_percent();
        if (pct < 0) {
            // 标定中（raw 跨度还不够）：只提示一次，避免刷屏
            if (last_logged != -2) {
                ESP_LOGI(TAG, "水位: 标定中 (raw=%d)——浸水一次再排干，刻度即自动变准", raw);
                last_logged = -2;
            }
        } else {
            bool low = (pct < ENV_WATER_PRESENT_PCT);
            bool last_low = (last_logged >= 0 && last_logged < ENV_WATER_PRESENT_PCT);
            int delta = pct - last_logged;
            if (last_logged < 0 || low != last_low || delta >= 10 || delta <= -10) {
                ESP_LOGI(TAG, "水位: %d%% (raw=%d, %s)", pct, raw, low ? "低水位" : "正常");
                last_logged = pct;
            }
        }

#if ENV_DHT11_DIAG
        // 排查模式：每 2s 读一次且无条件打印。
        // 原来 10s 才读一次 + 变化门限（Δ温≥0.5C 或 Δ湿≥2%）会把哈气这种
        // 2~3 秒的瞬态整个漏掉——DHT11 的湿度响应时间 τ63% 约 15s，
        // 一次哈气本来就只能推动很小的变化，采样再稀疏就什么都看不见了。
        {
            float t = 0.0f, h = 0.0f;
            if (env_read_dht11(&t, &h) == 0) {
                ESP_LOGI(TAG, "温湿度: %.1f C / %.1f %%", (double)t, (double)h);
            } else {
                ESP_LOGW(TAG, "DHT11 失败: %s | 第%d位 | 连续失败%d次 | raw=%02X %02X %02X %02X %02X (应=%02X)",
                         dht11_stage_str(s_dht_stage), s_dht_bit, s_dht_fails,
                         s_dht_raw[0], s_dht_raw[1], s_dht_raw[2], s_dht_raw[3], s_dht_raw[4],
                         (uint8_t)(s_dht_raw[0] + s_dht_raw[1] + s_dht_raw[2] + s_dht_raw[3]));
            }
        }
#else
        // DHT11 采样率上限 1Hz，这里 10s 读一次足够；数值有明显变化才打印，避免刷屏
        if (tick % 5 == 0) {
            float t = 0.0f, h = 0.0f;
            if (env_read_dht11(&t, &h) == 0) {
                float dt = t - last_temp, dh = h - last_hum;
                if (dt < 0) dt = -dt;
                if (dh < 0) dh = -dh;
                if (last_temp < -900.0f || dt >= 0.5f || dh >= 2.0f) {
                    ESP_LOGI(TAG, "温湿度: %.1f C / %.1f %%", (double)t, (double)h);
                    last_temp = t;
                    last_hum = h;
                }
            } else {
                ESP_LOGW(TAG, "DHT11 读取失败（接线/时序/被抢占），10 秒后重试");
            }
        }
#endif
    }
}
