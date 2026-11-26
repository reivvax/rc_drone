/*
 * camera_wifi: live MJPEG stream from the XIAO ESP32S3 Sense over Wi-Fi.
 *
 * Endpoints (see docs/runbooks/03-wifi-camera-stream.md):
 *   port 80  GET /         minimal viewer page (stream + status + controls)
 *            GET /capture  one JPEG
 *            GET /status   JSON: fps, frame size, RSSI, memory, ...
 *            GET /control?framesize=vga&quality=12   change settings at runtime
 *   port 81  GET /stream   multipart/x-mixed-replace MJPEG stream
 *
 * Every stream part carries X-Frame-Id and X-Timestamp-Us headers (capture time on
 * the camera's clock, us since boot), so a client such as the future onboard
 * computer can detect dropped frames and measure frame age.
 *
 * The stream runs on its own HTTP server (port 81) because esp_http_server handles
 * one request at a time per server, and /stream never returns while a client watches.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "mdns.h"
#include "driver/gpio.h"
#include "esp_camera.h"

static const char *TAG = "camwifi";

// Kconfig bools that are "n" are not defined at all, so map them to 0/1 here.
#ifdef CONFIG_CAM_HMIRROR
#define CAM_HMIRROR 1
#else
#define CAM_HMIRROR 0
#endif
#ifdef CONFIG_CAM_VFLIP
#define CAM_VFLIP 1
#else
#define CAM_VFLIP 0
#endif

/* ------------------------------------------------------------------------- */
/* Board: XIAO ESP32S3 Sense                                                  */
/* ------------------------------------------------------------------------- */

#define LED_GPIO        GPIO_NUM_21   // user LED, active low: on = Wi-Fi up
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    10
#define CAM_PIN_SIOD    40
#define CAM_PIN_SIOC    39
#define CAM_PIN_D7      48
#define CAM_PIN_D6      11
#define CAM_PIN_D5      12
#define CAM_PIN_D4      14
#define CAM_PIN_D3      16
#define CAM_PIN_D2      18
#define CAM_PIN_D1      17
#define CAM_PIN_D0      15
#define CAM_PIN_VSYNC   38
#define CAM_PIN_HREF    47
#define CAM_PIN_PCLK    13

#if CONFIG_CAM_FRAMESIZE_QVGA
#define INITIAL_FRAMESIZE FRAMESIZE_QVGA
#elif CONFIG_CAM_FRAMESIZE_SVGA
#define INITIAL_FRAMESIZE FRAMESIZE_SVGA
#elif CONFIG_CAM_FRAMESIZE_HD
#define INITIAL_FRAMESIZE FRAMESIZE_HD
#else
#define INITIAL_FRAMESIZE FRAMESIZE_VGA
#endif

/* ------------------------------------------------------------------------- */
/* Shared state                                                               */
/* ------------------------------------------------------------------------- */

// Written by the stream task, read by /status. Plain fields are fine for
// monitoring numbers: a torn read just shows a slightly stale value.
// Window metrics are recomputed once per second over the frames of that second.
static struct {
    volatile float    fps;
    volatile uint32_t last_frame_bytes;
    volatile uint32_t frames_sent;
    volatile int      stream_clients;
    // per-second window, to locate the bottleneck:
    volatile uint32_t get_us_avg;      // waiting for the camera   (~frame period => sensor-bound)
    volatile uint32_t send_us_avg;     // pushing into TCP/Wi-Fi   (~frame period => network-bound)
    volatile uint32_t send_us_max;     // worst send: Wi-Fi retries / TCP retransmission stalls
    volatile uint32_t frame_bytes_avg;
    volatile float    throughput_kBps; // payload KB/s actually delivered
} s_stats;

static volatile bool s_wifi_up;

/* ------------------------------------------------------------------------- */
/* Camera                                                                     */
/* ------------------------------------------------------------------------- */

static esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn = CAM_PIN_PWDN,     .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD, .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6, .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2, .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        // Initialise at the LARGEST size so buffers fit any size chosen later via
        // /control, then switch down below. JPEG buffers default to w*h/5 at this
        // size (~384 KB each), which is plenty and lives in PSRAM.
        .frame_size   = FRAMESIZE_UXGA,
        .jpeg_quality = CONFIG_CAM_JPEG_QUALITY,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, INITIAL_FRAMESIZE);
    // Orientation is done by the sensor itself (free), so every client gets a
    // correctly oriented image. Can be changed live via /control.
    s->set_hmirror(s, CAM_HMIRROR);
    s->set_vflip(s, CAM_VFLIP);
    ESP_LOGI(TAG, "camera ready, sensor PID 0x%x, hmirror=%d vflip=%d",
             s->id.PID, s->status.hmirror, s->status.vflip);
    return ESP_OK;
}

static const struct { const char *name; framesize_t size; } FRAMESIZES[] = {
    {"qqvga", FRAMESIZE_QQVGA},  // 160x120
    {"qvga",  FRAMESIZE_QVGA},   // 320x240
    {"vga",   FRAMESIZE_VGA},    // 640x480
    {"svga",  FRAMESIZE_SVGA},   // 800x600
    {"xga",   FRAMESIZE_XGA},    // 1024x768
    {"hd",    FRAMESIZE_HD},     // 1280x720
    {"sxga",  FRAMESIZE_SXGA},   // 1280x1024
    {"uxga",  FRAMESIZE_UXGA},   // 1600x1200
};

static const char *framesize_name(framesize_t size)
{
    for (size_t i = 0; i < sizeof FRAMESIZES / sizeof FRAMESIZES[0]; i++) {
        if (FRAMESIZES[i].size == size) return FRAMESIZES[i].name;
    }
    return "other";
}

/* ------------------------------------------------------------------------- */
/* HTTP: stream server (port 81)                                              */
/* ------------------------------------------------------------------------- */

#define PART_BOUNDARY "xiaoframe"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART_FMT =
    "Content-Type: image/jpeg\r\n"
    "Content-Length: %u\r\n"
    "X-Frame-Id: %lu\r\n"
    "X-Timestamp-Us: %lld\r\n"
    "\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    s_stats.stream_clients++;
    ESP_LOGI(TAG, "stream client connected");

    char part[160];
    uint32_t frame_id = 0;
    uint32_t window_frames = 0;
    uint64_t window_get_us = 0, window_send_us = 0, window_bytes = 0;
    uint32_t window_send_max = 0;
    int64_t window_start = esp_timer_get_time();
    esp_err_t res = ESP_OK;

    while (res == ESP_OK) {
        int64_t t_get = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        int64_t t_got = esp_timer_get_time();
        if (fb == NULL) {
            ESP_LOGW(TAG, "camera returned no frame");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        int64_t ts_us = (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
        int hlen = snprintf(part, sizeof part, STREAM_PART_FMT,
                            (unsigned)fb->len, (unsigned long)frame_id, (long long)ts_us);

        // Each send blocks until lwIP has taken the data. A slow client therefore
        // slows this loop, and GRAB_LATEST makes the next fb_get skip stale frames
        // instead of building a backlog: latency stays low, fps drops.
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        int64_t t_sent = esp_timer_get_time();
        size_t frame_len = fb->len;
        s_stats.last_frame_bytes = frame_len;
        esp_camera_fb_return(fb);

        if (res == ESP_OK) {
            frame_id++;
            s_stats.frames_sent++;
            uint32_t send_us = (uint32_t)(t_sent - t_got);
            window_frames++;
            window_get_us += (uint64_t)(t_got - t_get);
            window_send_us += send_us;
            window_bytes += frame_len;
            if (send_us > window_send_max) window_send_max = send_us;

            int64_t now = esp_timer_get_time();
            int64_t elapsed = now - window_start;
            if (elapsed >= 1000000) {  // publish once per second
                s_stats.fps = window_frames * 1e6f / (float)elapsed;
                s_stats.get_us_avg = (uint32_t)(window_get_us / window_frames);
                s_stats.send_us_avg = (uint32_t)(window_send_us / window_frames);
                s_stats.send_us_max = window_send_max;
                s_stats.frame_bytes_avg = (uint32_t)(window_bytes / window_frames);
                s_stats.throughput_kBps = window_bytes * 1e6f / 1024.0f / (float)elapsed;
                window_frames = 0;
                window_get_us = window_send_us = window_bytes = 0;
                window_send_max = 0;
                window_start = now;
            }
        }
    }

    s_stats.stream_clients--;
    s_stats.fps = 0;
    s_stats.throughput_kBps = 0;
    ESP_LOGI(TAG, "stream client disconnected after %lu frames", (unsigned long)frame_id);
    return ESP_FAIL;  // tells httpd to close the socket
}

/* ------------------------------------------------------------------------- */
/* HTTP: control server (port 80)                                             */
/* ------------------------------------------------------------------------- */

static const char INDEX_HTML[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>XIAO camera</title>"
    "<style>body{font-family:sans-serif;background:#111;color:#eee;margin:16px}"
    "img{max-width:100%;background:#000}select,button{font-size:1em;margin:4px}"
    "pre{color:#8f8}</style></head><body>"
    "<h3>XIAO ESP32S3 camera</h3>"
    "<img id='s'><br>"
    "<label>size <select id='fs'>"
    "<option>qqvga</option><option>qvga</option><option selected>vga</option>"
    "<option>svga</option><option>xga</option><option>hd</option>"
    "<option>sxga</option><option>uxga</option></select></label>"
    "<label>quality <select id='q'>"
    "<option>8</option><option>10</option><option selected>12</option>"
    "<option>15</option><option>20</option><option>30</option></select></label>"
    "<label><input type='checkbox' id='hm'>mirror</label>"
    "<label><input type='checkbox' id='vf'>flip</label>"
    "<button onclick='apply()'>apply</button><pre id='st'></pre>"
    "<script>"
    "document.getElementById('s').src='http://'+location.hostname+':81/stream';"
    "function apply(){fetch('/control?framesize='+fs.value+'&quality='+q.value"
    "+'&hmirror='+(hm.checked?1:0)+'&vflip='+(vf.checked?1:0));}"
    "let init=false;"
    "setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{"
    "if(!init){hm.checked=j.hmirror;vf.checked=j.vflip;init=true;}"
    "st.textContent=JSON.stringify(j,null,1);}),1000);"
    "</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, sizeof INDEX_HTML - 1);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera returned no frame");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    sensor_t *s = esp_camera_sensor_get();
    int rssi = 0;
#if CONFIG_CAM_WIFI_MODE_STA
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
#endif
    char json[640];
    int len = snprintf(json, sizeof json,
        "{\"framesize\":\"%s\",\"quality\":%d,\"hmirror\":%d,\"vflip\":%d,"
        "\"fps\":%.1f,\"last_frame_bytes\":%lu,"
        "\"frame_bytes_avg\":%lu,\"throughput_kBps\":%.1f,"
        "\"get_us_avg\":%lu,\"send_us_avg\":%lu,\"send_us_max\":%lu,"
        "\"frames_sent\":%lu,\"stream_clients\":%d,\"rssi_dbm\":%d,"
        "\"free_internal\":%u,\"free_psram\":%u,\"uptime_s\":%lld}",
        framesize_name(s->status.framesize), s->status.quality,
        s->status.hmirror, s->status.vflip, (double)s_stats.fps,
        (unsigned long)s_stats.last_frame_bytes,
        (unsigned long)s_stats.frame_bytes_avg, (double)s_stats.throughput_kBps,
        (unsigned long)s_stats.get_us_avg, (unsigned long)s_stats.send_us_avg,
        (unsigned long)s_stats.send_us_max,
        (unsigned long)s_stats.frames_sent, s_stats.stream_clients, rssi,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        (long long)(esp_timer_get_time() / 1000000));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, len);
}

// GET /control?framesize=<name>&quality=<4..63>&hmirror=<0|1>&vflip=<0|1>
// (every parameter optional)
static esp_err_t control_handler(httpd_req_t *req)
{
    char query[128], val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof query) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing query");
        return ESP_FAIL;
    }
    sensor_t *s = esp_camera_sensor_get();

    if (httpd_query_key_value(query, "framesize", val, sizeof val) == ESP_OK) {
        bool found = false;
        for (size_t i = 0; i < sizeof FRAMESIZES / sizeof FRAMESIZES[0]; i++) {
            if (strcasecmp(val, FRAMESIZES[i].name) == 0) {
                s->set_framesize(s, FRAMESIZES[i].size);
                found = true;
                break;
            }
        }
        if (!found) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown framesize");
            return ESP_FAIL;
        }
    }
    if (httpd_query_key_value(query, "quality", val, sizeof val) == ESP_OK) {
        int q = atoi(val);
        if (q < 4 || q > 63) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "quality must be 4..63");
            return ESP_FAIL;
        }
        s->set_quality(s, q);
    }
    if (httpd_query_key_value(query, "hmirror", val, sizeof val) == ESP_OK) {
        s->set_hmirror(s, atoi(val) ? 1 : 0);
    }
    if (httpd_query_key_value(query, "vflip", val, sizeof val) == ESP_OK) {
        s->set_vflip(s, atoi(val) ? 1 : 0);
    }
    ESP_LOGI(TAG, "control: %s", query);
    return status_handler(req);  // reply with the new state
}

static void start_http_servers(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    httpd_handle_t ctrl = NULL;
    ESP_ERROR_CHECK(httpd_start(&ctrl, &cfg));
    const httpd_uri_t ctrl_uris[] = {
        {.uri = "/",        .method = HTTP_GET, .handler = index_handler},
        {.uri = "/capture", .method = HTTP_GET, .handler = capture_handler},
        {.uri = "/status",  .method = HTTP_GET, .handler = status_handler},
        {.uri = "/control", .method = HTTP_GET, .handler = control_handler},
    };
    for (size_t i = 0; i < sizeof ctrl_uris / sizeof ctrl_uris[0]; i++) {
        httpd_register_uri_handler(ctrl, &ctrl_uris[i]);
    }

    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port = 32769;
    scfg.stack_size = 6144;
    // If a client vanishes without closing (e.g. Wi-Fi drop), give up on a send
    // after 5 s so the slot frees up for the next client.
    scfg.send_wait_timeout = 5;
    httpd_handle_t stream = NULL;
    ESP_ERROR_CHECK(httpd_start(&stream, &scfg));
    const httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
    httpd_register_uri_handler(stream, &stream_uri);

    ESP_LOGI(TAG, "HTTP servers up: control on :80, stream on :81");
}

/* ------------------------------------------------------------------------- */
/* Wi-Fi                                                                      */
/* ------------------------------------------------------------------------- */

static void set_led(bool on)
{
    gpio_set_level(LED_GPIO, on ? 0 : 1);  // active low
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = data;
        s_wifi_up = false;
        set_led(false);
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason %d), reconnecting...", ev->reason);
        vTaskDelay(pdMS_TO_TICKS(1000));  // don't hammer the access point
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = data;
        s_wifi_up = true;
        set_led(true);
        ESP_LOGI(TAG, "connected, open http://" IPSTR "/  or  http://%s.local/",
                 IP2STR(&ev->ip_info.ip), CONFIG_CAM_HOSTNAME);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "a client joined the access point");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "a client left the access point");
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wcfg = {0};
#if CONFIG_CAM_WIFI_MODE_STA
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    strlcpy((char *)wcfg.sta.ssid, CONFIG_CAM_WIFI_SSID, sizeof wcfg.sta.ssid);
    strlcpy((char *)wcfg.sta.password, CONFIG_CAM_WIFI_PASSWORD, sizeof wcfg.sta.password);
    wcfg.sta.threshold.authmode = strlen(CONFIG_CAM_WIFI_PASSWORD) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    if (strlen(CONFIG_CAM_WIFI_SSID) == 0) {
        ESP_LOGE(TAG, "no SSID set: run `idf.py menuconfig` -> Camera Wi-Fi Stream");
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_LOGI(TAG, "joining \"%s\"...", CONFIG_CAM_WIFI_SSID);
#else
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    strlcpy((char *)wcfg.ap.ssid, CONFIG_CAM_AP_SSID, sizeof wcfg.ap.ssid);
    wcfg.ap.ssid_len = strlen(CONFIG_CAM_AP_SSID);
    strlcpy((char *)wcfg.ap.password, CONFIG_CAM_AP_PASSWORD, sizeof wcfg.ap.password);
    wcfg.ap.authmode = strlen(CONFIG_CAM_AP_PASSWORD) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wcfg.ap.channel = CONFIG_CAM_AP_CHANNEL;
    wcfg.ap.max_connection = 2;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
#endif
    esp_netif_set_hostname(netif, CONFIG_CAM_HOSTNAME);  // shows up in the router's client list
    ESP_ERROR_CHECK(esp_wifi_start());

    // Default power save (modem sleep) wakes the radio only every beacon interval,
    // adding tens of ms of latency and jitter. Keep the radio awake for streaming.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_CAM_WIFI_MODE_AP
    s_wifi_up = true;
    set_led(true);
    ESP_LOGI(TAG, "access point \"%s\" up; join it and open http://192.168.4.1/", CONFIG_CAM_AP_SSID);
#endif
}

static void mdns_start(void)
{
    if (mdns_init() != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed; use the IP address instead");
        return;
    }
    mdns_hostname_set(CONFIG_CAM_HOSTNAME);
    mdns_instance_name_set("XIAO ESP32S3 camera");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

/* ------------------------------------------------------------------------- */

void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    set_led(false);

    // Wi-Fi stores calibration data in NVS (a key-value area in flash).
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(camera_init());
    wifi_init();
    mdns_start();
    start_http_servers();
    // app_main may return: Wi-Fi, HTTP and camera all run in their own tasks.
}
