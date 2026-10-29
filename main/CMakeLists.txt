# 注册main组件并包含源文件
# 注意：mqtt 与 cJSON 都不写进 REQUIRES，它们由 main/idf_component.yml 的托管依赖自动引入
idf_component_register(SRCS "cloud_uplink.c" "wifi_setup.c" "env_sensors.c" "guard.c" "main.c" "hx711.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_gpio nvs_flash esp_wifi esp_netif esp_event
                             esp_http_server esp_adc)
