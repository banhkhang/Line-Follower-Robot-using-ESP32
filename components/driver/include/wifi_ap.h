#pragma once

#include "esp_err.h"

esp_err_t wifi_ap_init_base(void);
esp_err_t wifi_ap_on(const char *ssid, const char *pass);
void wifi_ap_off(void);