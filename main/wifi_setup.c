/**
 * @file wifi_setup.c
 * @brief Wi-Fi STA 连接 + SoftAP 网页配网实现
 *
 * 凭据存 NVS（namespace "wificfg"），不写死在代码里。
 * 开机按住 BOOT(GPIO0) 可清除旧凭据强制进配网模式。
 */

#include "wifi_setup.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "cJSON.h"

static const char *TAG = "WIFI_SETUP";

#define WIFI_NVS_NAMESPACE "wificfg"
#define NVS_KEY_SSID       "ssid"
#define NVS_KEY_PASS       "pass"
#define BOOT_BUTTON_PIN    0   // 板载 BOOT 键

static volatile bool s_got_ip = false;
static bool s_ap_started = false;
static int s_fail_count = 0;   // STA 连续断开次数
static httpd_handle_t s_httpd = NULL;

/* ---------------- NVS 凭据 ---------------- */

static bool load_creds(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz)
{
    nvs_handle_t h;
    if (nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t n = ssid_sz;
    esp_err_t e1 = nvs_get_str(h, NVS_KEY_SSID, ssid, &n);
    n = pass_sz;
    esp_err_t e2 = nvs_get_str(h, NVS_KEY_PASS, pass, &n);
    nvs_close(h);
    return (e1 == ESP_OK && e2 == ESP_OK && ssid[0] != '\0');
}

static void save_creds(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_str(h, NVS_KEY_SSID, ssid);
    nvs_set_str(h, NVS_KEY_PASS, pass);
    nvs_commit(h);
    nvs_close(h);
}

static void erase_creds(void)
{
    nvs_handle_t h;
    if (nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* ---------------- 配网网页 ---------------- */

// 页面里统一用单引号，避免在 C 字符串里转义双引号
static const char *s_config_html =
    "<html><head><meta charset=utf-8><title>HX711 配网</title></head><body>"
    "<h3>HX711 配网</h3>"
    "<input id=s placeholder='Wi-Fi 名称 (SSID)' style='width:230px'><br><br>"
    "<input id=p placeholder='Wi-Fi 密码' type=password style='width:230px'><br><br>"
    "<button id=b onclick='go()'>保存并重启</button>"
    "<div id=m style='margin-top:10px'></div>"
    "<script>function go(){b.disabled=true;m.innerText='提交中…';"
    "fetch('/save',{method:'POST',body:JSON.stringify({ssid:s.value,pass:p.value})})"
    ".then(function(r){return r.json();})"
    ".then(function(j){m.innerText=j.ok?"
    "'已保存，设备重启中…热点会消失，请回家里 Wi-Fi 看串口日志':"
    "'保存失败';b.disabled=false;})"
    ".catch(function(e){m.innerText='没收到正常响应：'+e+"
    "'（设备很可能已保存并重启，热点消失即属正常）';});}"
    "</script></body></html>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, s_config_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// 独立任务里重启：让 save_handler 先干净返回，httpd 有机会把响应收尾
static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "现在重启");
    esp_restart();
}

static esp_err_t save_handler(httpd_req_t *req)
{
    char body[256];
    int total = req->content_len;
    ESP_LOGI(TAG, "收到 /save 请求，body 长度=%d", total);
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body too large");
        return ESP_OK;
    }

    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_OK;
        }
        received += r;
    }
    body[received] = '\0';

    cJSON *json = cJSON_Parse(body);
    if (json == NULL) {
        ESP_LOGE(TAG, "JSON 解析失败，原文=%s", body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_OK;
    }
    cJSON *jssid = cJSON_GetObjectItemCaseSensitive(json, "ssid");
    cJSON *jpass = cJSON_GetObjectItemCaseSensitive(json, "pass");
    if (!cJSON_IsString(jssid) || !cJSON_IsString(jpass) || jssid->valuestring[0] == '\0') {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing ssid or pass");
        return ESP_OK;
    }

    // 只打印 SSID，密码不落日志
    ESP_LOGI(TAG, "保存 Wi-Fi 配置: SSID=%s", jssid->valuestring);
    save_creds(jssid->valuestring, jpass->valuestring);
    cJSON_Delete(json);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");

    xTaskCreate(restart_task, "rst", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static void start_config_ap(bool keep_sta)
{
    if (s_ap_started) {
        return;
    }
    s_ap_started = true;

    wifi_config_t ap_cfg = { 0 };
    snprintf((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "%s", WIFI_AP_SSID);
    ap_cfg.ap.ssid_len = (uint8_t)strlen(WIFI_AP_SSID);
    snprintf((char *)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s", WIFI_AP_PASSWORD);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    // keep_sta 时保留 STA 继续重连，手机改完配置保存后仍会重启生效
    esp_wifi_set_mode(keep_sta ? WIFI_MODE_APSTA : WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&s_httpd, &cfg) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
        httpd_uri_t save = { .uri = "/save", .method = HTTP_POST, .handler = save_handler };
        httpd_register_uri_handler(s_httpd, &root);
        httpd_register_uri_handler(s_httpd, &save);
    } else {
        ESP_LOGE(TAG, "配网网页服务启动失败");
    }

    ESP_LOGI(TAG, "=========== 配网模式 ===========");
    ESP_LOGI(TAG, "1. 手机连 Wi-Fi: %s  密码: %s", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "2. 浏览器打开 http://192.168.4.1");
    ESP_LOGI(TAG, "3. 填家里 Wi-Fi 保存，设备会自动重启");
    ESP_LOGI(TAG, "================================");
}

/* ---------------- 事件 ---------------- */

static const char *disconnect_reason_str(uint8_t reason)
{
    switch (reason) {
    case 15:  return "4次握手超时（多半是密码错）";
    case 201: return "找不到该 SSID（名字错 / 路由器是 5GHz / 距离太远）";
    case 204: return "认证失败（密码错）";
    default:  return "其它原因";
    }
}

static void wifi_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA 启动，开始连接...");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "已关联 AP，等待 DHCP 分配 IP...");
            s_fail_count = 0;
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)event_data;
            s_got_ip = false;
            s_fail_count++;
            ESP_LOGW(TAG, "STA 断开 reason=%d (%s)，第 %d 次重试",
                     e->reason, disconnect_reason_str(e->reason), s_fail_count);
            if (s_fail_count >= WIFI_AP_FALLBACK_FAILS && !s_ap_started) {
                ESP_LOGW(TAG, "连续失败 %d 次，自动开启配网热点，手机可连上来改配置",
                         s_fail_count);
                start_config_ap(true);  // APSTA：STA 继续重连，同时提供配网页
            }
            // ESP32 单射频：APSTA 下 STA 被锁在热点信道，可能永远扫不到其它信道的
            // 路由器（表现为 reason=201 无限循环）。再失败若干次就切回 STA-only
            // 恢复全信道扫描。注意不用 esp_restart：重启会清零守卫的 PIR 稳定期等
            // uptime 状态，导致守卫永远无法布防。
            if (s_ap_started && s_fail_count >= WIFI_AP_FALLBACK_FAILS + 5) {
                ESP_LOGW(TAG, "热点开启后仍连续失败 %d 次（疑似信道锁定），关闭热点回 STA-only",
                         s_fail_count);
                esp_wifi_set_mode(WIFI_MODE_STA);
                s_ap_started = false;
                s_fail_count = 0;  // 允许之后再失败 5 次重新开热点
            }
            esp_wifi_connect();
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_got_ip = true;
        s_fail_count = 0;
        ESP_LOGI(TAG, "Wi-Fi 已拿到 IP，链路就绪");
    }
}

/* ---------------- 入口 ---------------- */

int wifi_setup_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();  // 配网模式才真正用到

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // 开机按住 BOOT 键 -> 清除旧凭据，强制进配网模式
    bool force_config = false;
    {
        gpio_config_t btn = {
            .pin_bit_mask = 1ULL << BOOT_BUTTON_PIN,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&btn);
        force_config = (gpio_get_level(BOOT_BUTTON_PIN) == 0);
        gpio_reset_pin(BOOT_BUTTON_PIN);  // 读完还回去，别占着 strapping 脚
    }

    if (force_config) {
        erase_creds();
        ESP_LOGW(TAG, "检测到 BOOT 键按下，已清除旧 Wi-Fi 配置");
    }

    char ssid[33] = { 0 };
    char pass[65] = { 0 };
    bool have_creds = !force_config && load_creds(ssid, sizeof(ssid), pass, sizeof(pass));
    const char *cred_src = "NVS";

    // NVS 没存过 -> 回落到出厂默认宏；默认宏也留空 -> 直接进配网模式
    if (!have_creds && WIFI_DEFAULT_SSID[0] != '\0') {
        snprintf(ssid, sizeof(ssid), "%s", WIFI_DEFAULT_SSID);
        snprintf(pass, sizeof(pass), "%s", WIFI_DEFAULT_PASSWORD);
        have_creds = true;
        cred_src = "出厂默认宏";
    }

    if (have_creds) {
        wifi_config_t sta_cfg = { 0 };
        strncpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid) - 1);
        strncpy((char *)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password) - 1);
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        esp_wifi_start();
        ESP_LOGI(TAG, "使用 Wi-Fi（来源:%s）: %s", cred_src, ssid);
    } else {
        esp_wifi_start();
        start_config_ap(false);
    }

    return 0;
}

bool wifi_setup_has_ip(void)
{
    return s_got_ip;
}
