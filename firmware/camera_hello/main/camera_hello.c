#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"   // pdMS_TO_TICKS
#include "freertos/task.h"       // vTaskDelay
#include "esp_err.h"             // ESP_ERROR_CHECK
#include "esp_camera.h"
#include "mbedtls/base64.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#define CAM_PIN_PWDN    -1   // not connected
#define CAM_PIN_RESET   -1   // not connected
#define CAM_PIN_XCLK    10
#define CAM_PIN_SIOD    40   // SCCB data  (SDA)
#define CAM_PIN_SIOC    39   // SCCB clock (SCL)
#define CAM_PIN_D7      48   // a.k.a. Y9
#define CAM_PIN_D6      11   // Y8
#define CAM_PIN_D5      12   // Y7
#define CAM_PIN_D4      14   // Y6
#define CAM_PIN_D3      16   // Y5
#define CAM_PIN_D2      18   // Y4
#define CAM_PIN_D1      17   // Y3
#define CAM_PIN_D0      15   // Y2
#define CAM_PIN_VSYNC   38
#define CAM_PIN_HREF    47
#define CAM_PIN_PCLK    13

static camera_config_t cam_cfg = {
    .pin_pwdn = CAM_PIN_PWDN,   .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD, .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6, .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2, .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,          // 20 MHz camera clock
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,    // sensor compresses on-chip, so frames are small
    .frame_size   = FRAMESIZE_VGA,     // 640x480
    .jpeg_quality = 16,                // 0-63, lower = better quality / bigger
    .jpeg_buffer_size = 128 * 1024,    // default is w*h/5 (60 KB at VGA)
    .fb_count     = 2,                 // 2 buffers: camera fills one while we send the other
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST // continuous capture
};

static const char *TAG = "cam";

#define FRAME_PERIOD_MS 0     // 0 = unbounded (as fast as camera + USB allow); 100 = 10 fps

static int SKIPPED_FRAMES = 10;

// ---- Bulk USB output -------------------------------------------------------
#define TX_STAGING_SIZE 4096
static uint8_t tx_staging[TX_STAGING_SIZE];
static size_t tx_used;

static void tx_flush(void) {
    if (tx_used > 0) {
        usb_serial_jtag_write_bytes(tx_staging, tx_used, portMAX_DELAY);
        tx_used = 0;
    }
}

static void tx_reserve(size_t n) {
    if (tx_used + n > TX_STAGING_SIZE) tx_flush();
}

static void send_frame_base64(const camera_fb_t *fb) {
    tx_reserve(48);
    tx_used += snprintf((char *)tx_staging + tx_used, TX_STAGING_SIZE - tx_used,
                        "===IMG_START %zu===\n", fb->len);

    for (size_t off = 0; off < fb->len; off += 57) {
        size_t chunk_len = fb->len - off;          // bytes left
        if (chunk_len > 57) chunk_len = 57;        // at most 57 per line

        tx_reserve(78);                            // 76 chars + '\n' + mbedtls' NUL
        size_t olen;
        int ret = mbedtls_base64_encode(tx_staging + tx_used, TX_STAGING_SIZE - tx_used, &olen,
                                        fb->buf + off, chunk_len);
        if (ret != 0) {
            ESP_LOGE(TAG, "base64 failed: %d", ret);
            break;
        }
        tx_used += olen;
        tx_staging[tx_used++] = '\n';             // overwrites the NUL
    }

    tx_reserve(16);
    memcpy(tx_staging + tx_used, "===IMG_END===\n", 14);
    tx_used += 14;
    tx_flush();
}

void app_main(void) {
    // Route stdout through the interrupt-driven USB driver instead of the default
    // polling path: writes block on a ring buffer (other tasks, incl. IDLE, can run
    // meanwhile, so no watchdog trips in the unbounded loop) and throughput is higher.
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_cfg.tx_buffer_size = 16 * 1024;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    usb_serial_jtag_vfs_use_driver();

    ESP_ERROR_CHECK(esp_camera_init(&cam_cfg));
    
    while (SKIPPED_FRAMES--) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGE(TAG, "camera returned NULL fb");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        esp_camera_fb_return(fb);
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        int64_t t0 = esp_timer_get_time();
        camera_fb_t* fb = esp_camera_fb_get();
        int64_t t1 = esp_timer_get_time();
        if (fb == NULL) {
            ESP_LOGE(TAG, "camera returned NULL fb");
            vTaskDelay(pdMS_TO_TICKS(500));
            last_wake = xTaskGetTickCount();   // restart the schedule after the pause
            continue;
        }

        send_frame_base64(fb);
        int64_t t2 = esp_timer_get_time();
        ESP_LOGI(TAG, "len=%zu get=%lld us send=%lld us", fb->len, t1 - t0, t2 - t1);
        esp_camera_fb_return(fb);

        // Wake at fixed period marks regardless of how long capture+send took.
        // Returns pdFALSE (without sleeping) if we already overran the period.
        if (FRAME_PERIOD_MS > 0 &&
            xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRAME_PERIOD_MS)) == pdFALSE) {
            ESP_LOGW(TAG, "missed %d ms frame deadline", FRAME_PERIOD_MS);
        }
    }

}
