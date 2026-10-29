/**
 * @file cloud_uplink.h
 * @brief 称重数据实时上云（Wi-Fi + MQTT -> 巴法云）
 *
 * 只负责"把数据发出去"这一件事：
 *   - 连 Wi-Fi（STA 模式，断线自动重连）
 *   - 连巴法云 MQTT broker，断线自动重连
 *   - 提供 cloud_uplink_publish_weight() 供称重任务调用
 *
 * 展示层（曲线/面板）由巴法云的小程序或网页控制台提供，本项目不写前端。
 */

#ifndef __CLOUD_UPLINK_H__
#define __CLOUD_UPLINK_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 改成你自己的两个值 ================= */

// 巴法云控制台 -> 个人中心 -> 私有密钥（一串字母数字）
// 本仓库公开，此处已清空：克隆后填你自己的私钥再编译。
// 私钥等于设备的身份凭证，泄露后任何人都能冒充你的设备、读写你的主题。
// 留空时 cloud_uplink_init() 会打印提示并跳过连云，本地功能不受影响。
#define BEMFA_PRIVATE_KEY    ""

// 巴法云控制台 -> 主题列表 -> 新建主题，名字自己起，例如 weight001
#define BEMFA_TOPIC          "传感器"

// 下行命令主题：智能体/手机往这里发 JSON 指令。
// 用 ASCII 名字，避免中文主题名在平台侧的兼容问题。
#define BEMFA_CMD_TOPIC      "weightcmd"

// 命令响应主题：设备把执行结果发到这里。
// 必须与 CMD 主题分开——设备订阅着 CMD，若响应也发回 CMD，
// 设备会收到自己的响应并误当命令，形成无限循环。
#define BEMFA_RESP_TOPIC     "weightresp"

// 报警事件主题：守卫模块上报入侵开始/结束。需在巴法云控制台单独创建。
#define BEMFA_ALARM_TOPIC    "weightalarm"

/* Wi-Fi 的 SSID/密码不在这里配置，见 wifi_setup.h（NVS + 配网网页 + 出厂默认宏） */

/* ===================================================== */

// 收到下行命令时的回调。注意：回调运行在 mqtt 事件任务里，必须快速返回，
// 重活要自己转交给别的任务（典型做法：回调只把消息塞进队列，另建任务消费）。
typedef void (*cloud_cmd_handler_t)(const char *payload, int len);

/**
 * @brief 注册下行命令回调（订阅 BEMFA_CMD_TOPIC 后，收到消息即调用）
 */
void cloud_uplink_set_command_handler(cloud_cmd_handler_t fn);

/**
 * @brief 创建 MQTT 客户端，并挂接"Wi-Fi 拿到 IP 后自动启动"的逻辑（非阻塞）
 *        必须在 wifi_setup_init() 之后调用
 * @return 0 成功
 */
int cloud_uplink_init(void);

/**
 * @brief 上报一次重量数据（JSON：{"weight":克,"raw":原始值}）
 * @return 0 已发出；-1 当前未连上云（数据被丢弃，不影响本地功能）
 */
int cloud_uplink_publish_weight(float grams, int32_t raw);

/**
 * @brief 向任意主题发一条 JSON（用于命令响应等）
 * @return 0 已发出；-1 未连上云
 */
int cloud_uplink_publish_json(const char *topic, const char *json);

/**
 * @brief 当前 MQTT 是否已连上云
 */
bool cloud_uplink_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* __CLOUD_UPLINK_H__ */
