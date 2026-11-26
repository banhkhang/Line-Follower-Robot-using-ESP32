idf_component_register(
    SRCS
        "main.c"
        "../boot/ota.c"
        "../components/application/app.c"

        "../components/service/loadcell_service.c"
        "../components/service/sensor_service.c"

        "../components/driver/hx711.c"
        "../components/driver/tcrt5000.c"
        "../components/driver/telemetry.c"
        "../components/driver/wifi_ap.c"
        "../components/driver/usr_led.c"
        # "../components/driver/motor.c"   # uncomment once motor.c is ready
        
    INCLUDE_DIRS
        "."
        "../boot"

        "../components/application"
        "../components/service/include"
        "../components/driver/include"

    REQUIRES
        esp_driver_gpio
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        esp_adc
        nvs_flash
        app_update
        esp_timer
    PRIV_REQUIRES esp_rom
)