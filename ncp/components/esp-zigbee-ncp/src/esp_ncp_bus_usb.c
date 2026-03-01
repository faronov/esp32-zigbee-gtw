/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "esp_log.h"
#include "driver/usb_serial_jtag.h"

#include "esp_ncp_bus.h"
#include "esp_ncp_frame.h"
#include "esp_ncp_main.h"
#include "slip.h"

static const char *TAG = "ESP_NCP_BUS_USB";

static esp_ncp_bus_t *s_ncp_bus;

static esp_err_t ncp_bus_read_hdl(void *buffer, uint16_t size)
{
    return ESP_FAIL;
}

static esp_err_t ncp_bus_write_hdl(void *buffer, uint16_t size)
{
    int written = 0;
    int remaining = size;
    uint8_t *buf = (uint8_t *)buffer;

    /* USB Serial/JTAG may not accept all bytes at once if the TX FIFO is full.
     * Loop until everything is sent or we time out. */
    while (remaining > 0) {
        int ret = usb_serial_jtag_write_bytes(buf + written, remaining, pdMS_TO_TICKS(100));
        if (ret < 0) {
            return ESP_FAIL;
        }
        if (ret == 0) {
            /* Host not reading — yield and retry once */
            vTaskDelay(pdMS_TO_TICKS(1));
            ret = usb_serial_jtag_write_bytes(buf + written, remaining, pdMS_TO_TICKS(500));
            if (ret <= 0) {
                ESP_LOGW(TAG, "USB write stalled, %d/%d bytes sent", written, size);
                return ESP_FAIL;
            }
        }
        written += ret;
        remaining -= ret;
    }

    return ESP_OK;
}

static esp_err_t ncp_bus_deinit_hdl(void)
{
    return usb_serial_jtag_driver_uninstall();
}

static esp_err_t ncp_bus_init_hdl(uint8_t transport)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = NCP_BUS_BUF_SIZE * 2,
        .tx_buffer_size = NCP_BUS_BUF_SIZE * 2,
    };

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial/JTAG driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "USB Serial/JTAG bus initialized");
    return ESP_OK;
}

/*
 * USB bus task — polls usb_serial_jtag_read_bytes() and scans for SLIP END
 * markers to delineate frames.  Unlike UART, there is no hardware pattern
 * detection, so we accumulate bytes into a frame buffer and push complete
 * SLIP frames to the stream buffer.
 */
static void esp_ncp_bus_task(void *pvParameter)
{
    esp_ncp_bus_t *bus = (esp_ncp_bus_t *)pvParameter;
    bus->state = BUS_INIT_START;

    uint8_t *rxbuf = (uint8_t *)malloc(NCP_BUS_BUF_SIZE);
    uint8_t *frame = (uint8_t *)malloc(NCP_BUS_BUF_SIZE);
    if (!rxbuf || !frame) {
        ESP_LOGE(TAG, "bus task: malloc failed");
        free(rxbuf);
        free(frame);
        vTaskDelete(NULL);
        return;
    }

    esp_ncp_ctx_t ncp_event = {
        .event = NCP_EVENT_OUTPUT,
    };

    int frame_pos = 0;

    while (bus->state == BUS_INIT_START) {
        int len = usb_serial_jtag_read_bytes(rxbuf, NCP_BUS_BUF_SIZE, pdMS_TO_TICKS(20));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; i++) {
            if (rxbuf[i] == SLIP_END) {
                if (frame_pos > 0) {
                    /* Include the trailing END marker in the frame data */
                    if (frame_pos < NCP_BUS_BUF_SIZE) {
                        frame[frame_pos++] = SLIP_END;
                    }
                    ncp_event.size = frame_pos;
                    xStreamBufferSend(bus->output_buf, frame, frame_pos, pdMS_TO_TICKS(100));
                    esp_ncp_send_event(&ncp_event);
                    frame_pos = 0;
                }
                /* else: leading END byte — skip */
            } else {
                if (frame_pos < NCP_BUS_BUF_SIZE) {
                    frame[frame_pos++] = rxbuf[i];
                } else {
                    ESP_LOGW(TAG, "SLIP frame overflow, discarding");
                    frame_pos = 0;
                }
            }
        }
    }

    free(rxbuf);
    free(frame);
    vTaskDelete(NULL);
}

esp_err_t esp_ncp_bus_input(const void *buffer, uint16_t len)
{
    if (buffer == NULL) {
        return ESP_FAIL;
    }

    esp_ncp_bus_t *bus = s_ncp_bus;
    esp_ncp_ctx_t ncp_event = {
        .event = NCP_EVENT_INPUT,
    };
    int count = NCP_BUS_RINGBUF_TIMEOUT_MS / 10;
    size_t ret_size = 0;

    if (bus->input_buf == NULL) {
        return ESP_FAIL;
    }

    while (xStreamBufferSpacesAvailable(bus->input_buf) < len) {
        if (count == 0) break;
        vTaskDelay(pdMS_TO_TICKS(10));
        count--;
    }

    if (count == 0) {
        ESP_LOGE(TAG, "input_buf not enough");
        return ESP_FAIL;
    }

    xSemaphoreTake(bus->input_sem, portMAX_DELAY);
    ret_size = xStreamBufferSend(bus->input_buf, buffer, len, 0);
    xSemaphoreGive(bus->input_sem);
    if (ret_size != len) {
        ESP_LOGE(TAG, "input_buf send error: size %d expect %d", ret_size, len);
        return ESP_FAIL;
    } else {
        ncp_event.size = len;
        return esp_ncp_send_event(&ncp_event);
    }
}

esp_err_t esp_ncp_bus_output(const void *buffer, uint16_t len)
{
    return esp_ncp_frame_output(buffer, len);
}

esp_err_t esp_ncp_bus_init(esp_ncp_bus_t **bus)
{
    esp_ncp_bus_t *bus_handle = calloc(1, sizeof(esp_ncp_bus_t));

    if (!bus_handle) {
        return ESP_ERR_NO_MEM;
    }

    bus_handle->input_buf = xStreamBufferCreate(NCP_BUS_RINGBUF_SIZE, 1);
    if (bus_handle->input_buf == NULL) {
        ESP_LOGE(TAG, "Input buffer create error");
        esp_ncp_bus_deinit(bus_handle);
        return ESP_ERR_NO_MEM;
    }

    bus_handle->output_buf = xStreamBufferCreate(NCP_BUS_RINGBUF_SIZE, 1);
    if (bus_handle->output_buf == NULL) {
        ESP_LOGE(TAG, "Out buffer create error");
        esp_ncp_bus_deinit(bus_handle);
        return ESP_ERR_NO_MEM;
    }

    bus_handle->input_sem = xSemaphoreCreateMutex();
    if (bus_handle->input_sem == NULL) {
        ESP_LOGE(TAG, "Input semaphore create error");
        esp_ncp_bus_deinit(bus_handle);
        return ESP_ERR_NO_MEM;
    }

    bus_handle->init = ncp_bus_init_hdl;
    bus_handle->deinit = ncp_bus_deinit_hdl;
    bus_handle->read = ncp_bus_read_hdl;
    bus_handle->write = ncp_bus_write_hdl;

    *bus = bus_handle;
    s_ncp_bus = bus_handle;

    return ESP_OK;
}

esp_err_t esp_ncp_bus_start(esp_ncp_bus_t *bus)
{
    if (!bus) {
        ESP_LOGE(TAG, "Invalid handle when start bus");
        return ESP_ERR_INVALID_ARG;
    }

    if (bus->state == BUS_INIT_START) {
        ESP_LOGE(TAG, "Invalid state %d when start bus", bus->state);
        return ESP_FAIL;
    }

    return (xTaskCreate(esp_ncp_bus_task, "esp_ncp_bus_task", NCP_BUS_TASK_STACK, bus, NCP_BUS_TASK_PRIORITY, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

esp_err_t esp_ncp_bus_stop(esp_ncp_bus_t *bus)
{
    if (!bus) {
        ESP_LOGE(TAG, "Invalid handle when stop bus");
        return ESP_ERR_INVALID_ARG;
    }

    if (bus->state != BUS_INIT_START) {
        ESP_LOGE(TAG, "Invalid state %d when stop bus", bus->state);
        return ESP_FAIL;
    }

    bus->state = BUS_INIT_STOP;

    return ESP_OK;
}

esp_err_t esp_ncp_bus_deinit(esp_ncp_bus_t *bus)
{
    if (!bus) {
        ESP_LOGE(TAG, "Invalid handle when deinit");
        return ESP_ERR_INVALID_ARG;
    }

    if (bus->output_buf) {
        vStreamBufferDelete(bus->output_buf);
        bus->output_buf = NULL;
    }

    if (bus->input_buf) {
        vStreamBufferDelete(bus->input_buf);
        bus->input_buf = NULL;
    }

    if (bus->input_sem) {
        vSemaphoreDelete(bus->input_sem);
        bus->input_sem = NULL;
    }

    free(bus);
    s_ncp_bus = NULL;

    return ESP_OK;
}
