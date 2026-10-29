/**
 * @file hx711.c
 * @brief HX711称重传感器ADC驱动实现
 */

#include "hx711.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "HX711";

// HX711 的时序靠软件打拍，临界区需要一把自旋锁来保护
static portMUX_TYPE s_hx711_mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief 内部函数：发送时钟脉冲并读取数据位
 */
static int hx711_read_cycle(hx711_device_t *dev)
{
    // 拉高时钟线
    gpio_set_level(dev->sck_pin, 1);
    
    // 短暂延时（>0.1us）
    esp_rom_delay_us(1);
    
    // 读取数据位
    int value = gpio_get_level(dev->dt_pin);
    
    // 拉低时钟线
    gpio_set_level(dev->sck_pin, 0);
    
    // 短暂延时（>0.1us）
    esp_rom_delay_us(1);
    
    return value;
}

int hx711_init(hx711_device_t *dev, int dt_pin, int sck_pin, hx711_gain_t gain)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "设备指针为空");
        return -1;
    }
    
    // 初始化设备结构体
    dev->dt_pin = dt_pin;
    dev->sck_pin = sck_pin;
    dev->gain = gain;
    dev->offset = 0;
    dev->scale = 1.0f;
    
    // 配置SCK引脚为输出模式
    gpio_config_t io_conf_sck = {
        .pin_bit_mask = (1ULL << sck_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf_sck);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置SCK引脚失败: %s", esp_err_to_name(ret));
        return -1;
    }
    
    // 配置DT引脚为输入模式
    gpio_config_t io_conf_dt = {
        .pin_bit_mask = (1ULL << dt_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    ret = gpio_config(&io_conf_dt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置DT引脚失败: %s", esp_err_to_name(ret));
        return -1;
    }
    
    // 初始化时钟线为低电平
    gpio_set_level(sck_pin, 0);
    
    // 等待HX711准备好
    ESP_LOGI(TAG, "等待HX711准备...");
    if (hx711_wait_ready(dev, 1000) != 0) {
        ESP_LOGE(TAG, "HX711未准备好");
        return -1;
    }
    
    // 执行一次空读，设置正确的增益
    int32_t dummy;
    hx711_read_raw(dev, &dummy);
    
    ESP_LOGI(TAG, "HX711初始化成功 - DT:%d, SCK:%d, 增益:%d", dt_pin, sck_pin, gain);
    return 0;
}

bool hx711_is_ready(hx711_device_t *dev)
{
    if (dev == NULL) {
        return false;
    }
    
    // HX711准备好时，DT引脚为低电平
    return gpio_get_level(dev->dt_pin) == 0;
}

int hx711_wait_ready(hx711_device_t *dev, uint32_t timeout_ms)
{
    if (dev == NULL) {
        return -1;
    }
    
    uint32_t start_time = xTaskGetTickCount();
    
    while (!hx711_is_ready(dev)) {
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "等待HX711超时");
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return 0;
}

int hx711_read_raw(hx711_device_t *dev, int32_t *data)
{
    if (dev == NULL || data == NULL) {
        return -1;
    }
    
    // 等待数据准备好
    if (hx711_wait_ready(dev, 1000) != 0) {
        return -1;
    }
    
    // 禁用中断（保证时序准确性）
    taskENTER_CRITICAL(&s_hx711_mux);
    
    // 读取24位数据
    int32_t value = 0;
    for (int i = 0; i < 24; i++) {
        value = (value << 1) | hx711_read_cycle(dev);
    }
    
    // 根据增益设置发送额外的时钟脉冲
    int extra_pulses = (int)dev->gain;
    for (int i = 0; i < extra_pulses; i++) {
        hx711_read_cycle(dev);
    }
    
    // 恢复中断
    taskEXIT_CRITICAL(&s_hx711_mux);
    
    // 处理24位有符号数
    if (value & 0x800000) {
        value |= (int32_t)0xFF000000;  // 符号扩展
    }
    
    *data = value;
    return 0;
}

// 中位数滤波的样本上限。24 位 ADC 会出现偶发毛刺（单点跳变数万计），
// 算术平均会被一个离群点带偏数千计，中位数则完全不受影响。
#define HX711_MAX_SAMPLES 32

int hx711_read_average(hx711_device_t *dev, int32_t *data, int times)
{
    if (dev == NULL || data == NULL || times <= 0) {
        return -1;
    }
    if (times > HX711_MAX_SAMPLES) {
        times = HX711_MAX_SAMPLES;
    }
    
    int32_t samples[HX711_MAX_SAMPLES];
    for (int i = 0; i < times; i++) {
        if (hx711_read_raw(dev, &samples[i]) != 0) {
            ESP_LOGE(TAG, "第%d次读取失败", i + 1);
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 短暂延时
    }
    
    // 插入排序后取中位数
    for (int i = 1; i < times; i++) {
        int32_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }
    
    *data = samples[times / 2];
    return 0;
}

void hx711_set_offset(hx711_device_t *dev, int32_t offset)
{
    if (dev != NULL) {
        dev->offset = offset;
        ESP_LOGI(TAG, "设置偏移值: %ld", (long)offset);
    }
}

int32_t hx711_get_offset(hx711_device_t *dev)
{
    if (dev == NULL) {
        return 0;
    }
    return dev->offset;
}

int hx711_tare(hx711_device_t *dev, int times)
{
    if (dev == NULL || times <= 0) {
        return -1;
    }
    if (times > 100) {
        times = 100;  // values[] 只有 100 格，防栈越界
    }
    
    ESP_LOGI(TAG, "正在校准零点（请确保空载）...");
    ESP_LOGI(TAG, "采样次数：%d", times);
    
    // 先丢弃前几次可能不稳定的读数
    int32_t warmup_values[3];
    ESP_LOGI(TAG, "预热采集（丢弃这些数据）...");
    for (int i = 0; i < 3; i++) {
        if (hx711_read_raw(dev, &warmup_values[i]) != 0) {
            ESP_LOGE(TAG, "预热采集失败");
            return -1;
        }
        ESP_LOGI(TAG, "预热读数 %d: raw=%ld", i+1, (long)warmup_values[i]);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // 正式采集
    int64_t sum = 0;
    int32_t value;
    int32_t values[100];  // 保存所有读数用于分析
    int valid_count = 0;
    
    for (int i = 0; i < times; i++) {
        if (hx711_read_raw(dev, &value) != 0) {
            ESP_LOGE(TAG, "零点校准读取失败");
            return -1;
        }
        
        sum += value;
        values[valid_count++] = value;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // 计算平均偏移
    dev->offset = (int32_t)(sum / times);
    ESP_LOGI(TAG, "零点校准完成，偏移值：%ld", (long)dev->offset);
    
    // 分析数据稳定性
    if (valid_count >= 2) {
        int32_t max_val = values[0];
        int32_t min_val = values[0];
        
        for (int i = 1; i < valid_count; i++) {
            if (values[i] > max_val) max_val = values[i];
            if (values[i] < min_val) min_val = values[i];
        }
        
        int32_t range = max_val - min_val;
        // 用克做阈值才有物理意义：同样的计数跨度，scale 不同代表的重量不同
        float sc = dev->scale > 0.0f ? dev->scale : 1.0f;
        float span_grams = (float)range / sc;
        ESP_LOGI(TAG, "零点数据范围：%ld ~ %ld，波动范围：%ld 计 (≈%.2f g)",
                 (long)min_val, (long)max_val, (long)range, (double)span_grams);
        
        // 判断稳定性（按克分档）
        if (span_grams < 0.5f) {
            ESP_LOGI(TAG, "✓ 零点数据非常稳定（≈%.2f g）", (double)span_grams);
        } else if (span_grams < 2.0f) {
            ESP_LOGW(TAG, "⚠ 零点数据有轻微波动（≈%.2f g），建议检查环境", (double)span_grams);
        } else if (span_grams < 5.0f) {
            ESP_LOGW(TAG, "⚠ 零点数据波动较大（≈%.2f g），请确保传感器稳定", (double)span_grams);
        } else {
            ESP_LOGE(TAG, "✗ 零点数据极不稳定（≈%.2f g），请检查硬件连接或重新校准", (double)span_grams);
        }
    }
    
    return 0;
}

void hx711_set_scale(hx711_device_t *dev, float scale)
{
    if (dev != NULL) {
        dev->scale = scale;
        ESP_LOGI(TAG, "设置缩放因子: %.2f", scale);
    }
}

float hx711_get_scale(hx711_device_t *dev)
{
    if (dev == NULL) {
        return 1.0f;
    }
    return dev->scale;
}

int hx711_get_weight(hx711_device_t *dev, float *weight, int times)
{
    if (dev == NULL || weight == NULL) {
        return -1;
    }
    
    int32_t raw_value;
    
    if (hx711_read_average(dev, &raw_value, times) != 0) {
        return -1;
    }
    
    // 计算实际重量：(原始值 - 偏移) / 缩放因子
    *weight = (float)(raw_value - dev->offset) / dev->scale;
    
    return 0;
}

void hx711_power_down(hx711_device_t *dev)
{
    if (dev != NULL) {
        // SCK引脚拉高并保持高电平超过60us进入掉电模式
        gpio_set_level(dev->sck_pin, 1);
        esp_rom_delay_us(70);
        ESP_LOGI(TAG, "HX711进入掉电模式");
    }
}

void hx711_power_up(hx711_device_t *dev)
{
    if (dev != NULL) {
        // SCK引脚拉低退出掉电模式
        gpio_set_level(dev->sck_pin, 0);
        esp_rom_delay_us(100);  // 等待稳定
        
        // 重新初始化设备
        gpio_set_level(dev->sck_pin, 0);
        
        // 执行一次空读以应用增益设置
        int32_t dummy;
        hx711_read_raw(dev, &dummy);
        
        ESP_LOGI(TAG, "HX711从掉电模式唤醒");
    }
}
