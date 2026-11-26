#include <stdbool.h>
#include <string.h>
#include "wifi_ap.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static bool s_base_ready = false;
static bool s_radio_on   = false;

esp_err_t wifi_ap_init_base(void)
{
    if (s_base_ready) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    esp_netif_create_default_wifi_ap();
    s_base_ready = true;
    return ESP_OK;
}

esp_err_t wifi_ap_on(const char *ssid, const char *pass)
{
    if (s_radio_on) {
        wifi_ap_off();
    }

    esp_err_t err = wifi_ap_init_base();
    if (err != ESP_OK) return err;

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) return err;

    wifi_config_t ap = { 0 };
    strncpy((char *)ap.ap.ssid, ssid, sizeof(ap.ap.ssid) - 1);
    strncpy((char *)ap.ap.password, pass, sizeof(ap.ap.password) - 1);
    ap.ap.ssid_len       = (uint8_t)strlen(ssid);
    ap.ap.channel        = 1;
    ap.ap.max_connection = 2;
    ap.ap.authmode       = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (err == ESP_OK) err = esp_wifi_start();

    if (err != ESP_OK) {
        esp_wifi_deinit();
        return err;
    }

    /* Disable Wi-Fi power-save to keep UDP latency and jitter low */
    esp_wifi_set_ps(WIFI_PS_NONE);

    s_radio_on = true;
    return ESP_OK;
}

void wifi_ap_off(void)
{
    if (!s_radio_on) {
        return;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    s_radio_on = false;
}

bool wifi_ap_is_on(void)
{
    return s_radio_on;
}

bool wifi_ap_has_station(void)
{
    if (!s_radio_on) {
        return false;
    }
    wifi_sta_list_t list;
    memset(&list, 0, sizeof(list));
    return (esp_wifi_ap_get_sta_list(&list) == ESP_OK && list.num > 0);
}
