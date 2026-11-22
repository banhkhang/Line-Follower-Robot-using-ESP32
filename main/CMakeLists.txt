idf_component_register(
    SRCS
        "main.c"
        "../boot/ota.c"
        "../components/application/app.c"
        "../components/driver/wifi_ap.c"
        "../components/driver/hx711.c"
        "../components/service/loadcell_service.c"

    INCLUDE_DIRS
        "."
        "../boot"
        "../components/application"
        "../components/driver/include"
        "../components/service/include"

    REQUIRES
        esp_driver_gpio
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        nvs_flash
        app_update
    PRIV_REQUIRES
        esp_rom
)