/**
 * @file cloud_uplink.c
 * @brief MQTT 上云实现（Wi-Fi 的连接/配网由 wifi_setup 负责）
 *
 * 设计要点：
 *  - 全异步。init 只创建 MQTT 客户端并挂接"拿到 IP 后自动启动"，不阻塞 app_main。
 *  - 没拿到 IP 之前不启动 MQTT，避免在配网模式下刷 DNS 失败日志。
 *  - 上云失败绝不影响本地功能：publish 在未连接时直接返回 -1，称重和报警灯照跑。
 */

#include "cloud_uplink.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"   // IP_EVENT / IP_EVENT_STA_GOT_IP 声明在这里
#include "mqtt_client.h"

static const char *TAG = "CLOUD";

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static volatile bool s_mqtt_connected = false;
static bool s_mqtt_started = false;
static cloud_cmd_handler_t s_cmd_handler = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "MQTT 已连接巴法云，开始上云");
        // 回环自检：订阅自己发布的主题。broker 若真收下了消息会转发回来，
        // 串口能看到回环 = 云端已收到；看不到 = 被静默丢弃（主题未创建/名字不合法）
        esp_mqtt_client_subscribe(s_mqtt_client, BEMFA_TOPIC, 1);
        // 下行命令通道
        esp_mqtt_client_subscribe(s_mqtt_client, BEMFA_CMD_TOPIC, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT 断开，esp-mqtt 会自动重连");
        break;
    case MQTT_EVENT_DATA: {
        esp_mqtt_event_t *e = (esp_mqtt_event_t *)event_data;
        if (e->topic_len == (int)strlen(BEMFA_CMD_TOPIC) &&
            strncmp(e->topic, BEMFA_CMD_TOPIC, (size_t)e->topic_len) == 0) {
            // 命令：只转交，重活由 agent 任务做，绝不阻塞 mqtt 事件任务
            if (s_cmd_handler != NULL) {
                s_cmd_handler(e->data, e->data_len);
            }
        } else {
            char buf[96];
            int n = e->data_len < (int)sizeof(buf) - 1 ? e->data_len : (int)sizeof(buf) - 1;
            memcpy(buf, e->data, n);
            buf[n] = '\0';
            ESP_LOGI(TAG, "回环收到: topic=%.*s payload=%s", e->topic_len, e->topic, buf);
        }
        break;
    }
    default:
        break;
    }
}

// Wi-Fi 拿到 IP 后才启动 MQTT：配网模式下没有 IP，就不该去解析域名
static void ip_event_handler(void *handler_args, esp_event_base_t base,
                             int32_t event_id, void *event_data)
{
    if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP && !s_mqtt_started) {
        s_mqtt_started = true;
        ESP_LOGI(TAG, "Wi-Fi 已拿到 IP，启动 MQTT");
        esp_mqtt_client_start(s_mqtt_client);
    }
}

int cloud_uplink_init(void)
{
    // 公开仓库里私钥被清空了。没填就直接跳过，不去建客户端：
    // 否则 mqtt 会拿着空 clientId 反复重连失败，日志全是噪音。
    if (BEMFA_PRIVATE_KEY[0] == '\0') {
        ESP_LOGW(TAG, "未配置巴法云私钥（cloud_uplink.h 的 BEMFA_PRIVATE_KEY 为空），跳过连云。"
                      "本地功能不受影响；要上云请填入自己的私钥后重新编译。");
        return -1;
    }

    // 巴法云 MQTT：clientId 填私有密钥，用户名密码留空
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://bemfa.com:9501",
        .credentials.client_id = BEMFA_PRIVATE_KEY,
        .session.keepalive = 60,
    };
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT 客户端创建失败");
        return -1;
    }
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // 事件循环由 wifi_setup_init() 创建，这里只挂一个 IP 事件
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, NULL));

    ESP_LOGI(TAG, "云上行已就绪，拿到 IP 后自动连接，主题=%s", BEMFA_TOPIC);
    return 0;
}

int cloud_uplink_publish_json(const char *topic, const char *json)
{
    if (s_mqtt_client == NULL || !s_mqtt_connected || topic == NULL || json == NULL) {
        return -1;  // 没连上就丢弃，不阻塞调用方
    }
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json, (int)strlen(json), 1, 0);
    return (msg_id < 0) ? -1 : 0;
}

int cloud_uplink_publish_weight(float grams, int32_t raw)
{
    char payload[64];
    int len = snprintf(payload, sizeof(payload),
                       "{\"weight\":%.2f,\"raw\":%ld}", (double)grams, (long)raw);
    if (len <= 0 || len >= (int)sizeof(payload)) {
        return -1;
    }
    return cloud_uplink_publish_json(BEMFA_TOPIC, payload);
}

void cloud_uplink_set_command_handler(cloud_cmd_handler_t fn)
{
    s_cmd_handler = fn;
}

bool cloud_uplink_is_connected(void)
{
    return s_mqtt_connected;
}
