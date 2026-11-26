idf_component_register(
    SRCS
        "main.c"
        "../boot/ota.c"
        "../components/application/app.c"
<<<<<<< HEAD
        "../components/driver/wifi_ap.c"
=======

        "../components/service/loadcell_service.c"
        "../components/service/vel_ctrl.c"
        "../components/service/drive_service.c"
        "../components/service/sensor_service.c"

        "../components/driver/hx711.c"
        "../components/driver/tcrt5000.c"
        "../components/driver/motor.c"
        "../components/driver/encoder.c"
        "../components/driver/telemetry.c"
        "../components/driver/wifi_ap.c"
        "../components/driver/usr_led.c"
        # "../components/driver/motor.c"   # uncomment once motor.c is ready
        
>>>>>>> b3bd95d5874f98f3172a4e590f05b1fae8eedb51
    INCLUDE_DIRS
        "."
        "../boot"
        "../components/application"
        "../components/driver/include"
    REQUIRES
        esp_driver_gpio
<<<<<<< HEAD
        nvs_flash
=======
        esp_driver_ledc
        esp_driver_gptimer
        esp_timer
>>>>>>> b3bd95d5874f98f3172a4e590f05b1fae8eedb51
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
<<<<<<< HEAD
        app_update
        esp_timer
        driver
        lwip
=======
        esp_adc
        nvs_flash
        app_update
        esp_timer
>>>>>>> b3bd95d5874f98f3172a4e590f05b1fae8eedb51
    PRIV_REQUIRES esp_rom
)
