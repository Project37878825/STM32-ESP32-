/**
 * @file wifi_setup.h
 * @brief Wi-Fi 连接 + SoftAP 网页配网
 *
 * 凭据优先级：NVS 里保存的 > 下面的出厂默认宏。
 * 行为：
 *  - NVS 有凭据            -> 以 STA 模式连接，断线自动重连
 *  - NVS 无但有出厂默认宏   -> 用默认宏以 STA 连接
 *  - 两者都没有            -> 直接开热点进配网模式
 *  - STA 连续失败 5 次      -> 自动追加热点（APSTA），手机可直接改配置，无需重启
 *  - 开机按住 BOOT(GPIO0)  -> 清除 NVS 凭据并强制进配网模式
 *
 * 配网方法：手机连热点 WIFI_AP_SSID，浏览器打开 http://192.168.4.1 填家里 Wi-Fi。
 * 注意：ESP32 只支持 2.4GHz Wi-Fi，5GHz 网络永远连不上。
 */

#ifndef __WIFI_SETUP_H__
#define __WIFI_SETUP_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 配网热点的名字和密码（手机连这个）
#define WIFI_AP_SSID      "HX711-Setup"
#define WIFI_AP_PASSWORD  "12345678"

// 出厂默认凭据：仅在本机 NVS 还没存过配置时使用。
// 本仓库公开，这里已清空——留空时首次开机直接进配网模式（开热点，手机连上填 Wi-Fi），
// 这也是推荐用法：凭据存在设备自己的 NVS 里，不进代码库。
// 若你想跳过配网直接连自家 Wi-Fi，把下面两行填上再编译（别提交到公开仓库）。
#define WIFI_DEFAULT_SSID     ""
#define WIFI_DEFAULT_PASSWORD ""

// STA 连续断开多少次后自动开启配网热点
#define WIFI_AP_FALLBACK_FAILS 5

/**
 * @brief 初始化网络栈与 Wi-Fi，按上述策略进入 STA 或配网模式（非阻塞）
 *        必须在 nvs_flash_init() 之后调用
 * @return 0 成功
 */
int wifi_setup_init(void);

/**
 * @brief 当前 STA 是否已拿到 IP
 */
bool wifi_setup_has_ip(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_SETUP_H__ */
