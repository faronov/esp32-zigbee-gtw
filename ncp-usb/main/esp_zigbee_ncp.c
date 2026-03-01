/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zigbee NCP — USB standalone variant
 *
 * Same as the UART NCP but communicates over the built-in USB Serial/JTAG
 * interface.  Plug the ESP32-H2 directly into a PC and use with zigbee2mqtt,
 * ZHA, or any software that speaks the Espressif NCP serial protocol.
 */

#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_zb_ncp.h"
#include "esp_zigbee_core.h"
#include "test/esp_zigbee_test_utils.h"

static const char *TAG = "ESP_NCP_USB";

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_ncp_init(NCP_HOST_CONNECTION_MODE_USB));

    /* Xiaomi-compatible security settings — must be set before NCP start */
    esp_zb_secur_link_key_exchange_required_set(false);
    esp_zb_secur_ic_only_enable(false);
    esp_zb_secur_network_min_join_lqi_set(0);
    esp_zb_secur_tcpol_set_allow_tc_rejoins(1);
    esp_zb_secur_set_unsecure_tc_rejoin_enabled(true);

    /* Set long ED aging timeout so sleepy devices (Xiaomi sensors) stay in
     * the child table even when they don't poll for hours. */
    esp_zb_nwk_set_ed_timeout(ESP_ZB_ED_AGING_TIMEOUT_16384MIN);

    ESP_LOGI(TAG, "USB NCP: Security + ED timeout configured for Xiaomi compatibility");

    ESP_ERROR_CHECK(esp_ncp_start());
}
