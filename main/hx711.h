/**
 * @file hx711.h
 * @brief HX711称重传感器ADC驱动
 * 
 * HX711是一款24位高精度ADC，专为称重传感器设计
 * 硬件连接：
 * - DT (数据引脚) -> ESP32 GPIO26
 * - SCK (时钟引脚) -> ESP32 GPIO27
 */

#ifndef __HX711_H__
#define __HX711_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// HX711引脚配置
//
// 目标板：ESP32-DevKitC 类（WROOM-32D，无 PSRAM），GPIO 基本都引出来了。
// 选 26/27 的原因（都是踩过的坑）：
//   GPIO6-11         -> 模组内部 SPI flash，绝对不能碰
//   GPIO0/2/5/12/15  -> strapping 引脚，复位瞬间的电平决定启动行为。
//                       其中 GPIO12 被拉高会把 flash 供电设成 1.8V，
//                       烧录直接报 "Failed to communicate with the flash chip"。
//                       而 HX711 的 DT 在转换期间恒为高电平，接 GPIO12 必踩此坑。
//   GPIO34-39        -> 仅输入、无内部上下拉（本项目水位传感器接 GPIO34，只读模拟量）
//   GPIO2            -> 板载蓝灯（本项目用作报警指示灯）
//   GPIO4/5/18/19/25 -> 已分给光敏 / PIR / 蜂鸣器 / DHT11 / 语音触发
// 26/27 是普通 IO，无上述限制且支持内部上下拉，因此分给 HX711。
#define HX711_DT_PIN    26  // 数据输出引脚
#define HX711_SCK_PIN   27  // 时钟输入引脚

// HX711增益配置
typedef enum {
    HX711_GAIN_128 = 1,  // 通道A，增益128
    HX711_GAIN_32 = 2,   // 通道B，增益32
    HX711_GAIN_64 = 3    // 通道A，增益64
} hx711_gain_t;

// HX711设备结构体
typedef struct {
    int dt_pin;          // 数据引脚
    int sck_pin;         // 时钟引脚
    hx711_gain_t gain;   // 增益设置
    int32_t offset;      // 零点偏移值
    float scale;         // 缩放因子（用于转换为实际重量单位）
} hx711_device_t;

/**
 * @brief 初始化HX711设备
 * @param dev 设备结构体指针
 * @param dt_pin 数据引脚GPIO编号
 * @param sck_pin 时钟引脚GPIO编号
 * @param gain 增益设置
 * @return 0成功，-1失败
 */
int hx711_init(hx711_device_t *dev, int dt_pin, int sck_pin, hx711_gain_t gain);

/**
 * @brief 检查HX711是否准备好
 * @param dev 设备结构体指针
 * @return true准备好，false未准备好
 */
bool hx711_is_ready(hx711_device_t *dev);

/**
 * @brief 等待HX711准备好
 * @param dev 设备结构体指针
 * @param timeout_ms 超时时间（毫秒）
 * @return 0成功，-1超时
 */
int hx711_wait_ready(hx711_device_t *dev, uint32_t timeout_ms);

/**
 * @brief 读取原始ADC值
 * @param dev 设备结构体指针
 * @param data 输出数据指针
 * @return 0成功，-1失败
 */
int hx711_read_raw(hx711_device_t *dev, int32_t *data);

/**
 * @brief 读取平均值（多次采样取平均）
 * @param dev 设备结构体指针
 * @param data 输出数据指针
 * @param times 采样次数（建议10-20）
 * @return 0成功，-1失败
 */
int hx711_read_average(hx711_device_t *dev, int32_t *data, int times);

/**
 * @brief 设置零点偏移
 * @param dev 设备结构体指针
 * @param offset 偏移值
 */
void hx711_set_offset(hx711_device_t *dev, int32_t offset);

/**
 * @brief 获取当前偏移值
 * @param dev 设备结构体指针
 * @return 偏移值
 */
int32_t hx711_get_offset(hx711_device_t *dev);

/**
 * @brief 自动校准零点（需要空载）
 * @param dev 设备结构体指针
 * @param times 采样次数
 * @return 0成功，-1失败
 */
int hx711_tare(hx711_device_t *dev, int times);

/**
 * @brief 设置缩放因子
 * @param dev 设备结构体指针
 * @param scale 缩放因子
 */
void hx711_set_scale(hx711_device_t *dev, float scale);

/**
 * @brief 获取缩放因子
 * @param dev 设备结构体指针
 * @return 缩放因子
 */
float hx711_get_scale(hx711_device_t *dev);

/**
 * @brief 读取重量值（单位由scale决定）
 * @param dev 设备结构体指针
 * @param weight 输出重量值指针
 * @param times 采样次数
 * @return 0成功，-1失败
 */
int hx711_get_weight(hx711_device_t *dev, float *weight, int times);

/**
 * @brief 关闭HX711（进入掉电模式）
 * @param dev 设备结构体指针
 */
void hx711_power_down(hx711_device_t *dev);

/**
 * @brief 唤醒HX711（从掉电模式恢复）
 * @param dev 设备结构体指针
 */
void hx711_power_up(hx711_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __HX711_H__ */
