/**
 * @file main.c
 * @brief 系统入口：NVS 初始化、子系统启动编排、称重任务与蓝灯报警任务
 *
 * 目标板：ESP32-DevKitC 类（WROOM-32D），ESP-IDF v6.1
 *
 * 引脚分配（细节见各模块头文件）：
 *   HX711   DT=GPIO26  SCK=GPIO27   称重
 *   光敏    DO=GPIO4                昼夜判定
 *   PIR     OUT=GPIO5               人体红外
 *   蜂鸣器  I/O=GPIO18              报警输出
 *   水位    AO=GPIO34 (ADC1_CH6)    低水位报警
 *   DHT11   DATA=GPIO19             温湿度
 *   语音    IO=GPIO25               入侵语音播报（可选，默认未启用）
 *   蓝灯    GPIO2（板载）           报警指示
 *
 * 子系统开关 ENABLE_WIFI_CLOUD / ENABLE_HX711 默认为 0：
 * 日常只跑"环境守卫"，开机不连网、不读称重，启动快、日志干净。
 * 代码仍然完整保留，想用哪个把对应开关改成 1 重新编译即可。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "hx711.h"
#include "wifi_setup.h"
#include "cloud_uplink.h"
#include "led_alarm.h"
#include "env_sensors.h"
#include "guard.h"

static const char *TAG = "APP";

// HX711 设备句柄
static hx711_device_t hx711_dev = {0};

// 校准参数（根据实际传感器调整）
#define HX711_SCALE_FACTOR  420.0f  // 缩放因子，需要根据实际校准
#define HX711_TARE_TIMES    50      // 零点校准采样次数（增加到 50 次以获得更稳定的零点）
#define HX711_READ_TIMES    20      // 每次读取的采样次数（增加到 20 次）
#define HX711_READ_INTERVAL 1000    // 读取间隔（毫秒）

// 子系统开关：0 = 不启动（减少开机负担与日志），1 = 启动。
// 关掉后对应代码仍会编译进固件，只是不初始化、不建任务，想恢复改回 1 重烧即可。
// 副作用：被 #if 排除调用的静态函数会触发 defined-but-unused 警告，属预期，不影响功能。
#define ENABLE_WIFI_CLOUD  0
#define ENABLE_HX711       0

// 板载蓝灯与超重报警配置
// GPIO2 是板载蓝灯。它同时也是 strapping 引脚，但 strapping 只在复位瞬间被采样，
// 程序启动后再把它配置成输出是安全的（板载 LED 本来就是这么用的）。
#define LED_BLUE_PIN          2       // 板载蓝灯引脚
#define LED_ACTIVE_HIGH       1       // 该板蓝灯高电平点亮；若发现逻辑反了改成 0
#define LED_ON_LEVEL          (LED_ACTIVE_HIGH ? 1 : 0)
#define LED_OFF_LEVEL         (LED_ACTIVE_HIGH ? 0 : 1)
#define WEIGHT_ALARM_G        100.0f  // 超过该重量(克)蓝灯开始闪烁
#define LED_BLINK_PERIOD_MS   200     // 闪烁半周期(毫秒)

// 称重任务 -> 蓝灯任务 的共享标志。
// 单个 bool 的读写在 ESP32 上是原子的，两个任务间传一个比特不需要加锁。
static volatile bool s_weight_over_threshold = false;

// 运行时可调状态：蓝灯手动模式由守卫报警逻辑接管（见 guard.c / led_alarm.h）
static volatile float s_alarm_threshold_g = WEIGHT_ALARM_G;  // 超重阈值(克)
static volatile int s_led_manual = -1;  // -1 自动 / 0 强制灭 / 1 强制常亮

/**
 * @brief 初始化NVS存储
 */
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS初始化成功");
}

/**
 * @brief 硬件探针：定位"重量恒为 0.00"到底是驱动问题还是接线问题
 *
 * 判据来自 HX711 数据手册的三条硬件行为：
 *   A) 转换进行中 DT 输出高；转换完成 DT 拉低，并一直保持低电平直到被时钟读走。
 *   B) 一次完整读取需要 25/26/27 个 SCK 脉冲（24 位数据 + 1~3 位增益选择）。
 *   C) 脉冲数给够之后，HX711 立即开始下一次转换，DT 会重新变高。
 *
 * 关键推论：如果 SCK 根本没送到 HX711，芯片完成第一次转换后 DT 就永远停在低电平，
 *           于是 wait_ready() 每次都秒过、24 位全读成 0 —— 正好就是当前的症状。
 */
static void hardware_probe(void)
{
    ESP_LOGI(TAG, "============ 硬件探针 ============");

    /* ---- 探针 1：SCK 引脚本身能不能翻转 ---- */
    // GPIO_MODE_OUTPUT 内部会调 gpio_input_disable()，输入缓冲被关掉，
    // 此时 gpio_get_level() 读不到焊盘真实电平。要自检必须临时切成 INPUT_OUTPUT。
    gpio_set_direction(HX711_SCK_PIN, GPIO_MODE_INPUT_OUTPUT);

    gpio_set_level(HX711_SCK_PIN, 1);
    esp_rom_delay_us(2);
    int sck_hi = gpio_get_level(HX711_SCK_PIN);

    gpio_set_level(HX711_SCK_PIN, 0);
    esp_rom_delay_us(2);
    int sck_lo = gpio_get_level(HX711_SCK_PIN);

    gpio_set_direction(HX711_SCK_PIN, GPIO_MODE_OUTPUT);

    if (sck_hi == 1 && sck_lo == 0) {
        ESP_LOGI(TAG, "探针1 SCK(GPIO%d) 回读 高=1 低=0 -> GPIO 输出正常", HX711_SCK_PIN);
    } else {
        ESP_LOGE(TAG, "探针1 SCK(GPIO%d) 回读 高=%d 低=%d -> 引脚被外部拉住或已损坏",
                 HX711_SCK_PIN, sck_hi, sck_lo);
    }

    /* ---- 探针 2：DT 是被 HX711 驱动，还是悬空 ---- */
    // 手法：先禁掉上下拉读一次，再打开内部上拉(~45kΩ)读一次。
    // HX711 的 DT 是推挽输出，真在驱动低电平时，45kΩ 上拉绝对抬不动它。
    gpio_config_t dt_conf = {
        .pin_bit_mask = 1ULL << HX711_DT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dt_conf);
    vTaskDelay(pdMS_TO_TICKS(10));
    int dt_nopull = gpio_get_level(HX711_DT_PIN);

    dt_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&dt_conf);
    vTaskDelay(pdMS_TO_TICKS(10));
    int dt_pullup = gpio_get_level(HX711_DT_PIN);

    // 恢复成驱动实际使用的配置（禁上拉），避免影响后面的读取
    dt_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&dt_conf);

    ESP_LOGI(TAG, "探针2 DT(GPIO%d) 无上拉=%d 开内部上拉=%d",
             HX711_DT_PIN, dt_nopull, dt_pullup);
    if (dt_nopull == 0 && dt_pullup == 1) {
        ESP_LOGE(TAG, "       -> DT 悬空！内部上拉就能把它抬到高电平，");
        ESP_LOGE(TAG, "          说明 HX711 没在驱动这根线：杜邦线没插好 / 芯片没供电 / 接错脚");
    } else if (dt_nopull == 0 && dt_pullup == 0) {
        ESP_LOGI(TAG, "       -> DT 被外部实打实拉低（HX711 在驱动，或该线短到了 GND）");
    } else {
        ESP_LOGI(TAG, "       -> DT 当前为高，HX711 正在转换中（正常状态之一）");
    }

    /* ---- 探针 3：手动打 25 个脉冲，看 HX711 是否响应 ---- */
    int ready_wait = 0;
    while (gpio_get_level(HX711_DT_PIN) != 0 && ready_wait < 500) {
        vTaskDelay(pdMS_TO_TICKS(1));
        ready_wait++;
    }
    ESP_LOGI(TAG, "探针3 等待 DT 变低耗时 %d ms（>=500 说明 DT 一直是高，从未就绪）", ready_wait);

    uint32_t bits = 0;
    int high_bits = 0;
    for (int i = 0; i < 25; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        int b = gpio_get_level(HX711_DT_PIN);
        if (b) {
            high_bits++;
        }
        bits = (bits << 1) | (uint32_t)b;
        gpio_set_level(HX711_SCK_PIN, 0);
        esp_rom_delay_us(1);
    }

    ESP_LOGI(TAG, "探针3 25 脉冲读回 0x%06lX，高电平位 %d 个",
             (unsigned long)(bits & 0xFFFFFF), high_bits);

    // 判据 C：脉冲给够后 HX711 会立刻开始下一次转换，DT 应重新变高
    vTaskDelay(pdMS_TO_TICKS(2));
    int dt_after = gpio_get_level(HX711_DT_PIN);
    if (dt_after == 1) {
        ESP_LOGI(TAG, "探针3 打完后 DT=1 -> HX711 收到了时钟，通信链路是通的");
    } else {
        ESP_LOGE(TAG, "探针3 打完后 DT=0 -> SCK 没送达 HX711（查 SCK 线 / 芯片供电）");
    }

    /* ---- 探针 4：连续读 5 次原始值，看是否恒定 ---- */
    for (int i = 0; i < 5; i++) {
        int32_t raw;
        if (hx711_read_raw(&hx711_dev, &raw) == 0) {
            ESP_LOGI(TAG, "探针4 第%d次 raw = %ld (0x%06lX)",
                     i + 1, (long)raw, (unsigned long)(raw & 0xFFFFFF));
        } else {
            ESP_LOGE(TAG, "探针4 第%d次读取失败：DT 一直为高，等不到就绪信号", i + 1);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "==================================");
}

/**
 * @brief 主任务：读取并打印重量数据
 */
static void weight_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "开始读取重量数据...");
    
    while (1) {
        int32_t raw_value = 0;
        
        if (hx711_read_average(&hx711_dev, &raw_value, HX711_READ_TIMES) == 0) {
            // 换算：(原始值 - 零点偏移) / 缩放因子
            int32_t offset = hx711_get_offset(&hx711_dev);
            float scale = hx711_get_scale(&hx711_dev);
            int32_t diff = raw_value - offset;
            float weight_grams = (float)diff / scale;
            float weight_kg = weight_grams / 1000.0f;
            
            // raw 是判断硬件是否真的在工作的关键证据，必须一起打印
            printf("\n========== 称重数据 ==========\n");
            printf("原始值：%ld (offset=%ld, 差值=%ld, scale=%.2f)\n",
                   (long)raw_value, (long)offset, (long)diff, scale);
            printf("重量：%.2f g (克)\n", weight_grams);
            printf("重量：%.3f kg (千克)\n", weight_kg);
            printf("==============================\n\n");
            
            ESP_LOGI(TAG, "raw=%ld 差值=%ld | 重量：%.2f g | %.3f kg | 云:%s",
                     (long)raw_value, (long)diff, weight_grams, weight_kg,
                     cloud_uplink_is_connected() ? "已连" : "未连");
            
            // 更新超重标志，蓝灯任务据此决定闪不闪（阈值运行时可调）
            s_weight_over_threshold = (weight_grams >= s_alarm_threshold_g);
            
            // 上报到云。没连上时返回 -1，本地功能不受影响
            if (cloud_uplink_publish_weight(weight_grams, raw_value) != 0) {
                ESP_LOGW(TAG, "本次数据未上云（MQTT 未连接）");
            }
            
            // 空载时差值应接近 0；远超阈值说明被碰、超载或接线松动
            if (abs(diff) > (int32_t)(scale * 50)) {
                ESP_LOGW(TAG, "警告：差值异常大 (%ld)，请检查传感器是否被碰或超载", (long)diff);
            }
        } else {
            ESP_LOGE(TAG, "读取重量失败");
        }
        
        // 等待一段时间再进行下一次读取
        vTaskDelay(pdMS_TO_TICKS(HX711_READ_INTERVAL));
    }
}

/**
 * @brief 蓝灯报警任务：重量超阈值时闪烁，否则熄灭
 *
 * 独立成任务是为了让闪烁频率(200ms)不受称重任务 1 秒读取间隔的限制。
 * 与称重任务之间只通过 s_weight_over_threshold 这一个 volatile bool 通信。
 */
static void led_alarm_task(void *pvParameters)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = 1ULL << LED_BLUE_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_BLUE_PIN, LED_OFF_LEVEL);  // 初始熄灭
    
    bool lit = false;
    while (1) {
        int manual = s_led_manual;
        if (manual == 1) {
            gpio_set_level(LED_BLUE_PIN, LED_ON_LEVEL);       // 智能体强制常亮
        } else if (manual == 0) {
            gpio_set_level(LED_BLUE_PIN, LED_OFF_LEVEL);      // 智能体强制灭
        } else if (s_weight_over_threshold) {
            lit = !lit;  // 自动模式且超阈值：翻转电平形成闪烁
            gpio_set_level(LED_BLUE_PIN, lit ? LED_ON_LEVEL : LED_OFF_LEVEL);
        } else {
            lit = false;  // 自动模式且未超阈值：保持熄灭
            gpio_set_level(LED_BLUE_PIN, LED_OFF_LEVEL);
        }
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_PERIOD_MS));
    }
}

/* ---- led_alarm.h 的实现：供智能体下行命令在运行时控制 ---- */

void led_alarm_set_manual(int mode)
{
    s_led_manual = mode;  // 静默 setter：guard 每 300ms 调一次，不能打日志
}

void alarm_set_threshold_g(float grams)
{
    if (grams > 0.0f) {
        s_alarm_threshold_g = grams;
        ESP_LOGI(TAG, "超重阈值改为 %.1f g", (double)grams);
    }
}

float alarm_get_threshold_g(void)
{
    return s_alarm_threshold_g;
}

/**
 * @brief 主函数：按依赖顺序启动各子系统
 *
 * 顺序有讲究：NVS 必须最先（Wi-Fi 凭据存在里面）；env_sensors_init 必须在
 * guard_init 之前（守卫只调驱动，不负责配引脚）；Wi-Fi 与上云放在最前面启动
 * 是因为它们非阻塞，连网可以和后面的传感器预热并行进行。
 */
void app_main(void)
{
    ESP_LOGI(TAG, "============ ESP32 多传感环境守卫系统 ============");
    ESP_LOGI(TAG, "入侵报警 / 低水位报警 / 温湿度 / 称重(可选) / 上云(可选)");
    ESP_LOGI(TAG, "子系统开关: Wi-Fi+云=%s  称重HX711=%s  语音播报=%s",
             ENABLE_WIFI_CLOUD ? "开" : "关",
             ENABLE_HX711 ? "开" : "关",
             ENV_VOICE_ENABLE ? "开" : "关");
    ESP_LOGI(TAG, "=================================================");
    
    // 初始化NVS
    init_nvs();
    
#if ENABLE_WIFI_CLOUD
    // 先起 Wi-Fi（含配网逻辑），再起 MQTT 上云。两者都非阻塞
    wifi_setup_init();
    cloud_uplink_init();
#endif
    env_sensors_init();  // 光敏/PIR/蜂鸣器/水位ADC/DHT11/语音触发脚 + 水位温湿度监测任务
    guard_init();        // 报警状态机（入侵 = 布防 && 有人；低水位 = 水位 < 阈值）
    
#if ENABLE_HX711
    // 初始化HX711
    ESP_LOGI(TAG, "正在初始化HX711...");
    ESP_LOGI(TAG, "DT引脚: GPIO%d, SCK引脚: GPIO%d", HX711_DT_PIN, HX711_SCK_PIN);
    
    if (hx711_init(&hx711_dev, HX711_DT_PIN, HX711_SCK_PIN, HX711_GAIN_128) != 0) {
        ESP_LOGE(TAG, "HX711初始化失败！请检查硬件连接");
        ESP_LOGE(TAG, "请确认：");
        ESP_LOGE(TAG, "  1. HX711的VCC连接到3.3V或5V");
        ESP_LOGE(TAG, "  2. HX711的GND连接到GND");
        ESP_LOGE(TAG, "  3. HX711的DT连接到GPIO%d", HX711_DT_PIN);
        ESP_LOGE(TAG, "  4. HX711的SCK连接到GPIO%d", HX711_SCK_PIN);
        ESP_LOGE(TAG, "  5. 称重传感器正确连接到HX711");
        return;
    }
    
    ESP_LOGI(TAG, "HX711初始化成功！");
    
    // 先跑硬件探针，再校准。
    // 顺序很重要：如果硬件其实是坏的，tare 会把这个恒定的故障值当成零点记下来，
    // 之后 (raw - offset) 永远等于 0，故障就被彻底掩盖了。
    hardware_probe();

    // 缩放因子需要按实际传感器标定：放已知重量（如 100g 砝码），
    // scale = (raw - offset) / 实际克数。当前值是占位的，称重前必须先标。
    hx711_set_scale(&hx711_dev, HX711_SCALE_FACTOR);
    
    // 上电预热：HX711 和应变片通电后头几秒零点会漂移。
    // 如果立刻 tare，会把漂移中的值当成零点记下来，之后读数就整体偏。
    ESP_LOGI(TAG, "传感器预热 3 秒，请保持空载、不要触碰...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 简单零点校准
    ESP_LOGI(TAG, "进行简单零点校准...");
    if (hx711_tare(&hx711_dev, HX711_TARE_TIMES) == 0) {
        ESP_LOGI(TAG, "零点校准成功");
    } else {
        ESP_LOGW(TAG, "零点校准失败，使用默认偏移值: 0");
    }
    
    // 创建重量读取任务
    BaseType_t ret = xTaskCreate(
        weight_read_task,       // 任务函数
        "weight_read_task",     // 任务名称
        4096,                   // 栈大小
        NULL,                   // 参数
        5,                      // 优先级
        NULL                    // 任务句柄
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建读取任务失败");
        return;
    }
#endif  // ENABLE_HX711
    
    // 蓝灯任务保留：守卫报警期间要接管蓝灯闪烁
    BaseType_t ret_led = xTaskCreate(
        led_alarm_task,         // 任务函数
        "led_alarm_task",       // 任务名称
        2048,                   // 栈大小
        NULL,                   // 参数
        4,                      // 优先级（低于称重任务）
        NULL                    // 任务句柄
    );
    
    if (ret_led != pdPASS) {
        ESP_LOGE(TAG, "创建蓝灯任务失败");
        return;
    }
    
    ESP_LOGI(TAG, "系统启动完成");
    
    // 主循环可以执行其他任务
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 延时10秒
        
        // 可以在这里添加其他逻辑，比如：
        // - 检查是否需要重新校准
        // - 处理用户输入
        // - 发送数据到云端等
    }
}
