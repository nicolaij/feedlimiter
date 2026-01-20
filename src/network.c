#include "main.h"

#include "esp_partition.h"
#include <esp_ota_ops.h>

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <esp_http_server.h>

#include "esp_spiffs.h"

#include <arpa/inet.h>

#include "driver/dac_oneshot.h"

uint8_t mac[6];
extern esp_ip4_addr_t pdp_ip;
extern char net_status_current[32];

#define CLIENT_WIFI_SSID "ap1"
#define CLIENT_WIFI_PASS ""

#define AP_WIFI_SSID "Piloramka"
#define AP_WIFI_PASS "123123123"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

int32_t timezone = 3;

static const char *TAGW = "wifi";
static const char *TAGH = "httpd";

static int s_retry_num = 0;

#define TRANSFER_SIZE (CONFIG_LWIP_TCP_MSS - 14) // Correction for Chunked transfer encoding
static char network_buf[CONFIG_LWIP_TCP_MSS];
size_t buf_len;

static int64_t timeout_begin;

extern dac_oneshot_handle_t chan1_handle;
extern dac_oneshot_handle_t chan2_handle;

void reset_sleep_timeout()
{
    timeout_begin = esp_timer_get_time();
    // ESP_LOGV(TAGW, "Timeout reset");
    // xEventGroupSetBits(status_event_group, WIFI_ACTIVE);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        reset_sleep_timeout();
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 1)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAGW, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAGW, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAGW, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        reset_sleep_timeout();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        reset_sleep_timeout();
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAGW, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAGW, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(uint8_t channel, uint8_t ssid_hidden)
{
    ESP_LOGD("wifi_init_softap", "Start");
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_WIFI_SSID,
            .ssid_len = strlen(AP_WIFI_SSID),
            .ssid_hidden = ssid_hidden,
            .channel = channel,
            .password = AP_WIFI_PASS,
            .max_connection = 3,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };

    if (strlen(AP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config.ap.ssid_len = snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s%.0f", AP_WIFI_SSID, get_menu_val_by_id("idn"));

    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_AP, WIFI_BW_HT20)); // иначе не работает 11 канал
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44)); // влияние на измерения АЦП незначительные

    int8_t power = 0;
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));

    ESP_LOGI(TAGW, "wifi_init_softap finished. SSID:%s password:%s channel:%d power:%d", AP_WIFI_SSID, AP_WIFI_PASS, wifi_config.ap.channel, power);
}

int wifi_init_sta(void)
{

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CLIENT_WIFI_SSID,
            .password = CLIENT_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        }};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAGW, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAGW, "connected to ap SSID:%s password:%s", CLIENT_WIFI_SSID, CLIENT_WIFI_PASS);
        return 1;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAGW, "Failed to connect to SSID:%s, password:%s", CLIENT_WIFI_SSID, CLIENT_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAGW, "UNEXPECTED EVENT");
    }

    return 0;
}

static esp_err_t menu_get_handler(httpd_req_t *req)
{
    int l = 0;

    const char http1[] = "<!DOCTYPE html>\n\
    <html><head>\
    <meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\" />\
    <meta name=\"viewport\" content=\"width=device-width\" />\
    <title>Настройки</title>\
    <meta charset=\"utf-8\"></head><body>\
    <form name=\"settings\" action=\"\" method=\"post\">\
    <div id=\"settingscontent\">";

    const char http99[] = "</div><input type=\"submit\" value=\"Submit\" /></form><br>";

    const char http100[] = "<div>"
                           "<button onclick=\"sendMessage(0)\">DEBUG! DAC = 0%</button>"
                           "<button onclick=\"sendMessage(127)\">DEBUG! DAC = 50%</button>"
                           "<button onclick=\"sendMessage(255)\">DEBUG! DAC = 100%</button>"
                           "</div>"
                           "<script>const socket = new WebSocket('ws://192.168.4.1/ws');"
                           "function sendMessage(value) {if (socket.readyState === WebSocket.OPEN) {socket.send(\"DAC=\" + value.toString())};}"
                           "</script>"
                           "</body></html>";

    httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
    httpd_resp_send_chunk(req, http1, sizeof(http1));

    do
    {
        l = get_menu_html(network_buf);
        if (l > 0)
            httpd_resp_send_chunk(req, network_buf, l);
    } while (l > 0);

    httpd_resp_send_chunk(req, http99, sizeof(http99));
    httpd_resp_send_chunk(req, http100, sizeof(http100));
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t menu_post_handler(httpd_req_t *req)
{
    int remaining = req->content_len;
    int try = 3;
    while (remaining > 0 && try > 0)
    {
        /* Read the data for the request */
        int ret = 0;
        if ((ret = httpd_req_recv(req, network_buf, MIN(remaining, sizeof(network_buf)))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry receiving if timeout occurred */
                vTaskDelay(100 / portTICK_PERIOD_MS);
                try--;
                continue;
            }
            return ESP_FAIL;
        }

        remaining -= ret;

        /* Log data received */
        // ESP_LOGI(TAGH, "=========== RECEIVED DATA ==========");
        // ESP_LOGI(TAGH, "%.*s", ret, buf);
        // ESP_LOGI(TAGH, "====================================");
    }

    network_buf[req->content_len] = '\0';

    char *s = network_buf;
    char name[16];
    int mac2 = 0;
    uint8_t addr[6];
    while (s && s < (network_buf + req->content_len))
    {
        char *e = strchr(s, '=');
        *e = '\0';
        strncpy(name, s, sizeof(name));
        float v = 0;
        if (strncmp(name, "ip", 2) == 0)
        {
            int parsed = sscanf((const char *)e + 1, "%hhu.%hhu.%hhu.%hhu", &addr[0], &addr[1], &addr[2], &addr[3]);
            if (parsed == 4)
            {
                v = (addr[0] << 0) | (addr[1] << 8) | (addr[2] << 16) | (addr[3] << 24);
            }
        }
        else if (strncmp(name, "MAC1", 4) == 0)
        {
            //&MAC1=00%3A00%3A00%3A00%3A00%3A51&MAC2=81
            int parsed = sscanf((const char *)e + 1,
                                "%hhx%%3A%hhx%%3A%hhx%%3A%hhx%%3A%hhx%%3A%hhx",
                                &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);

            if (parsed == 6)
            {
                v = (addr[0] << 16) | (addr[1] << 8) | addr[2];
                mac2 = (addr[3] << 16) | (addr[4] << 8) | addr[5];
            }
            else
            {
                mac2 = 0;
            }
        }
        else if (strncmp(name, "MAC2", 4) == 0)
        {
            v = mac2;
        }
        else
        {
            v = atof(e + 1);
        }

        set_menu_val_by_id(name, v);

        s = strchr(e + 1, '&');
        if (s)
            s = s + 1;
    }

    if (xHandleWifi)
        xTaskNotify(xHandleWifi, NOTYFY_WIFI_REBOOT, eSetValueWithOverwrite);

    // End response
    return menu_get_handler(req);
}

#define ESP_IMAGE_HEADER_MAGIC 0xE9 /*!< The magic word for the esp_image_header_t structure. */

/*
 * Handle OTA file upload
 */
esp_err_t update_get_handler(httpd_req_t *req)
{
    /*py -c "import sys; a=bytearray(); [a:=a+l.encode('utf-8') for l in sys.stdin]; print(','.join(map(str, a)))" < update.html*/
    char content[] = {60, 33, 68, 79, 67, 84, 89, 80, 69, 32, 104, 116, 109, 108, 62, 10, 60, 104, 116, 109, 108, 62, 10, 32, 32, 60, 104, 101, 97, 100, 62, 10, 32, 32, 32, 32, 60, 109, 101, 116, 97, 32, 104, 116, 116, 112, 45, 101, 113, 117, 105, 118, 61, 34, 99, 111, 110, 116, 101, 110, 116, 45, 116, 121, 112, 101, 34, 32, 99, 111, 110, 116, 101, 110, 116, 61, 34, 116, 101, 120, 116, 47, 104, 116, 109, 108, 59, 32, 99, 104, 97, 114, 115, 101, 116, 61, 117, 116, 102, 45, 56, 34, 32, 47, 62, 10, 9, 60, 109, 101, 116, 97, 32, 110, 97, 109, 101, 61, 34, 118, 105, 101, 119, 112, 111, 114, 116, 34, 32, 99, 111, 110, 116, 101, 110, 116, 61, 34, 119, 105, 100, 116, 104, 61, 100, 101, 118, 105, 99, 101, 45, 119, 105, 100, 116, 104, 34, 62, 10, 32, 32, 32, 32, 60, 116, 105, 116, 108, 101, 62, 69, 83, 80, 51, 50, 32, 79, 84, 65, 32, 85, 112, 100, 97, 116, 101, 60, 47, 116, 105, 116, 108, 101, 62, 10, 32, 32, 32, 32, 60, 115, 99, 114, 105, 112, 116, 62, 10, 32, 32, 32, 32, 32, 32, 102, 117, 110, 99, 116, 105, 111, 110, 32, 115, 116, 97, 114, 116, 85, 112, 108, 111, 97, 100, 40, 41, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 118, 97, 114, 32, 111, 116, 97, 102, 105, 108, 101, 32, 61, 32, 100, 111, 99, 117, 109, 101, 110, 116, 46, 103, 101, 116, 69, 108, 101, 109, 101, 110, 116, 66, 121, 73, 100, 40, 34, 111, 116, 97, 102, 105, 108, 101, 34, 41, 46, 102, 105, 108, 101, 115, 59, 10, 10, 32, 32, 32, 32, 32, 32, 32, 32, 105, 102, 32, 40, 111, 116, 97, 102, 105, 108, 101, 46, 108, 101, 110, 103, 116, 104, 32, 61, 61, 32, 48, 41, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 97, 108, 101, 114, 116, 40, 34, 78, 111, 32, 102, 105, 108, 101, 32, 115, 101, 108, 101, 99, 116, 101, 100, 33, 34, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 125, 32, 101, 108, 115, 101, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 100, 111, 99, 117, 109, 101, 110, 116, 46, 103, 101, 116, 69, 108, 101, 109, 101, 110, 116, 66, 121, 73, 100, 40, 34, 111, 116, 97, 102, 105, 108, 101, 34, 41, 46, 100, 105, 115, 97, 98, 108, 101, 100, 32, 61, 32, 116, 114, 117, 101, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 100, 111, 99, 117, 109, 101, 110, 116, 46, 103, 101, 116, 69, 108, 101, 109, 101, 110, 116, 66, 121, 73, 100, 40, 34, 117, 112, 108, 111, 97, 100, 34, 41, 46, 100, 105, 115, 97, 98, 108, 101, 100, 32, 61, 32, 116, 114, 117, 101, 59, 10, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 118, 97, 114, 32, 102, 105, 108, 101, 32, 61, 32, 111, 116, 97, 102, 105, 108, 101, 91, 48, 93, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 118, 97, 114, 32, 120, 104, 114, 32, 61, 32, 110, 101, 119, 32, 88, 77, 76, 72, 116, 116, 112, 82, 101, 113, 117, 101, 115, 116, 40, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 120, 104, 114, 46, 111, 110, 114, 101, 97, 100, 121, 115, 116, 97, 116, 101, 99, 104, 97, 110, 103, 101, 32, 61, 32, 102, 117, 110, 99, 116, 105, 111, 110, 32, 40, 41, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 105, 102, 32, 40, 120, 104, 114, 46, 114, 101, 97, 100, 121, 83, 116, 97, 116, 101, 32, 61, 61, 32, 52, 41, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 105, 102, 32, 40, 120, 104, 114, 46, 115, 116, 97, 116, 117, 115, 32, 61, 61, 32, 50, 48, 48, 41, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 100, 111, 99, 117, 109, 101, 110, 116, 46, 111, 112, 101, 110, 40, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 100, 111, 99, 117, 109, 101, 110, 116, 46, 119, 114, 105, 116, 101, 40, 120, 104, 114, 46, 114, 101, 115, 112, 111, 110, 115, 101, 84, 101, 120, 116, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 100, 111, 99, 117, 109, 101, 110, 116, 46, 99, 108, 111, 115, 101, 40, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 125, 32, 101, 108, 115, 101, 32, 105, 102, 32, 40, 120, 104, 114, 46, 115, 116, 97, 116, 117, 115, 32, 61, 61, 32, 48, 41, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 97, 108, 101, 114, 116, 40, 34, 83, 101, 114, 118, 101, 114, 32, 99, 108, 111, 115, 101, 100, 32, 116, 104, 101, 32, 99, 111, 110, 110, 101, 99, 116, 105, 111, 110, 32, 97, 98, 114, 117, 112, 116, 108, 121, 33, 34, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 108, 111, 99, 97, 116, 105, 111, 110, 46, 114, 101, 108, 111, 97, 100, 40, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 125, 32, 101, 108, 115, 101, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 97, 108, 101, 114, 116, 40, 120, 104, 114, 46, 115, 116, 97, 116, 117, 115, 32, 43, 32, 34, 32, 69, 114, 114, 111, 114, 33, 92, 110, 34, 32, 43, 32, 120, 104, 114, 46, 114, 101, 115, 112, 111, 110, 115, 101, 84, 101, 120, 116, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 108, 111, 99, 97, 116, 105, 111, 110, 46, 114, 101, 108, 111, 97, 100, 40, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 125, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 125, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 125, 59, 10, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 120, 104, 114, 46, 117, 112, 108, 111, 97, 100, 46, 111, 110, 112, 114, 111, 103, 114, 101, 115, 115, 32, 61, 32, 102, 117, 110, 99, 116, 105, 111, 110, 32, 40, 101, 41, 32, 123, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 118, 97, 114, 32, 112, 114, 111, 103, 114, 101, 115, 115, 32, 61, 32, 100, 111, 99, 117, 109, 101, 110, 116, 46, 103, 101, 116, 69, 108, 101, 109, 101, 110, 116, 66, 121, 73, 100, 40, 34, 112, 114, 111, 103, 114, 101, 115, 115, 34, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 112, 114, 111, 103, 114, 101, 115, 115, 46, 116, 101, 120, 116, 67, 111, 110, 116, 101, 110, 116, 32, 61, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 34, 80, 114, 111, 103, 114, 101, 115, 115, 58, 32, 34, 32, 43, 32, 40, 40, 101, 46, 108, 111, 97, 100, 101, 100, 32, 47, 32, 101, 46, 116, 111, 116, 97, 108, 41, 32, 42, 32, 49, 48, 48, 41, 46, 116, 111, 70, 105, 120, 101, 100, 40, 48, 41, 32, 43, 32, 34, 37, 34, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 125, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 120, 104, 114, 46, 111, 112, 101, 110, 40, 34, 80, 79, 83, 84, 34, 44, 32, 34, 47, 117, 112, 100, 97, 116, 101, 34, 44, 32, 116, 114, 117, 101, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 120, 104, 114, 46, 115, 101, 110, 100, 40, 102, 105, 108, 101, 41, 59, 10, 32, 32, 32, 32, 32, 32, 32, 32, 125, 10, 32, 32, 32, 32, 32, 32, 125, 10, 32, 32, 32, 32, 60, 47, 115, 99, 114, 105, 112, 116, 62, 10, 32, 32, 60, 47, 104, 101, 97, 100, 62, 10, 32, 32, 60, 98, 111, 100, 121, 62, 10, 32, 32, 32, 32, 60, 104, 49, 62, 69, 83, 80, 51, 50, 32, 79, 84, 65, 32, 70, 105, 114, 109, 119, 97, 114, 101, 32, 85, 112, 100, 97, 116, 101, 60, 47, 104, 49, 62, 10, 32, 32, 32, 32, 60, 100, 105, 118, 62, 10, 32, 32, 32, 32, 32, 32, 60, 108, 97, 98, 101, 108, 32, 102, 111, 114, 61, 34, 111, 116, 97, 102, 105, 108, 101, 34, 62, 70, 105, 114, 109, 119, 97, 114, 101, 32, 111, 114, 32, 83, 80, 73, 70, 70, 83, 32, 102, 105, 108, 101, 58, 60, 47, 108, 97, 98, 101, 108, 62, 10, 32, 32, 32, 32, 32, 32, 60, 105, 110, 112, 117, 116, 32, 116, 121, 112, 101, 61, 34, 102, 105, 108, 101, 34, 32, 105, 100, 61, 34, 111, 116, 97, 102, 105, 108, 101, 34, 32, 110, 97, 109, 101, 61, 34, 111, 116, 97, 102, 105, 108, 101, 34, 32, 47, 62, 10, 32, 32, 32, 32, 60, 47, 100, 105, 118, 62, 10, 32, 32, 32, 32, 60, 100, 105, 118, 62, 10, 32, 32, 32, 32, 32, 32, 60, 98, 117, 116, 116, 111, 110, 32, 105, 100, 61, 34, 117, 112, 108, 111, 97, 100, 34, 32, 116, 121, 112, 101, 61, 34, 98, 117, 116, 116, 111, 110, 34, 32, 111, 110, 99, 108, 105, 99, 107, 61, 34, 115, 116, 97, 114, 116, 85, 112, 108, 111, 97, 100, 40, 41, 34, 62, 85, 112, 108, 111, 97, 100, 60, 47, 98, 117, 116, 116, 111, 110, 62, 10, 32, 32, 32, 32, 60, 47, 100, 105, 118, 62, 10, 32, 32, 32, 32, 60, 100, 105, 118, 32, 105, 100, 61, 34, 112, 114, 111, 103, 114, 101, 115, 115, 34, 62, 60, 47, 100, 105, 118, 62, 10, 32, 32, 60, 47, 98, 111, 100, 121, 62, 10, 60, 47, 104, 116, 109, 108, 62, 10};
    return httpd_resp_sendstr(req, content);
}

esp_err_t update_post_handler(httpd_req_t *req)
{
    int file_id = -1;
    esp_ota_handle_t ota_handle;
    int remaining = req->content_len;

    reset_sleep_timeout();

    /* Пишем в next_ota и прошивку и spiffs.bin*/
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
    int try = 3;
    while (remaining > 0 && try > 0)
    {
        int recv_len = httpd_req_recv(req, network_buf, MIN(remaining, sizeof(network_buf)));

        // Timeout Error: Just retry
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            try--;
            continue;

            // Serious Error: Abort OTA
        }
        else if (recv_len <= 0)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
            return ESP_FAIL;
        }

        if (file_id == -1) // first data block
        {
            file_id = network_buf[0];

            if (file_id == ESP_IMAGE_HEADER_MAGIC)
            {
                ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));
            }
            else if (remaining == 0x50000) // SPIFFS Image
            {
                file_id = remaining;
                ESP_ERROR_CHECK(esp_partition_erase_range(ota_partition, 0, 0x50000));
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File type Error");
                return ESP_FAIL;
            }
        }

        // firmware.bin
        if (file_id == ESP_IMAGE_HEADER_MAGIC)
        {
            // Successful Upload: Flash firmware chunk
            if (esp_ota_write(ota_handle, (const void *)network_buf, recv_len) != ESP_OK)
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write Error");
                return ESP_FAIL;
            }
        }
        else if (file_id == 0x50000) // spiffs.bin
        {
            ESP_ERROR_CHECK(esp_partition_write(ota_partition, (req->content_len - remaining), (const void *)network_buf, recv_len));
        }

        vTaskDelay(1);
        remaining -= recv_len;
    }

    if (file_id == ESP_IMAGE_HEADER_MAGIC)
    {
        // Validate and switch to new OTA image and reboot
        if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
            return ESP_FAIL;
        }

        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");
        ESP_LOGW(TAGH, "Firmware update complete, rebooting now!");
        if (xHandleWifi)
            xTaskNotify(xHandleWifi, NOTYFY_WIFI_REBOOT, eSetValueWithOverwrite);
    }
    else if (file_id == 0x50000)
    {
        const esp_partition_t *storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
        ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, 0, 0x50000));

        remaining = req->content_len;
        int recv_len = sizeof(network_buf);
        while (remaining > 0)
        {
            recv_len = MIN(remaining, sizeof(network_buf));
            if (esp_partition_read(ota_partition, (req->content_len - remaining), (void *)network_buf, recv_len) == ESP_OK)
            {
                ESP_ERROR_CHECK(esp_partition_write(storage_partition, (req->content_len - remaining), (const void *)network_buf, recv_len));
                vTaskDelay(1);
            }
            remaining -= recv_len;
        }

        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_sendstr(req, "SPIFFS update complete, rebooting now!\n");
        ESP_LOGW(TAGH, "SPIFFS update complete, rebooting now!");

        if (xHandleWifi)
            xTaskNotify(xHandleWifi, NOTYFY_WIFI_REBOOT, eSetValueWithOverwrite);
    }

    return ESP_OK;
}

char *get_datetime(time_t ttime)
{
    static char datetime[24];
    struct tm *localtm = localtime(&ttime);
    strftime(datetime, sizeof(datetime), "%Y-%m-%d %T", localtm);
    return datetime;
}

httpd_handle_t ws_hd;
int ws_fd[5] = {0, 0, 0, 0, 0};

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAGH, "WS handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAGH, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAGH, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAGH, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAGH, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAGH, "Got packet with message: %s", ws_pkt.payload);
    }

    ESP_LOGI(TAGH, "ws_handler: httpd_handle_t=%p, sockfd=%d, client_info:%d", req->handle, httpd_req_to_sockfd(req), httpd_ws_get_fd_info(req->handle, httpd_req_to_sockfd(req)));

    if (strncmp("DAC=255", (const char *)ws_pkt.payload, 7) == 0)
    {
        run_stage = 100;
    }
    if (strncmp("DAC=127", (const char *)ws_pkt.payload, 7) == 0)
    {
        run_stage = 101;
    }
    if (strncmp("DAC=0", (const char *)ws_pkt.payload, 5) == 0)
    {
        run_stage = 102;
    }

    free(buf);
    return ret;
}

static const httpd_uri_t main_page = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = menu_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t menu_post = {
    .uri = "/",
    .method = HTTP_POST,
    .handler = menu_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t update_get = {
    .uri = "/update",
    .method = HTTP_GET,
    .handler = update_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t update_post = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = update_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL,
    .is_websocket = true};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // config.max_open_sockets = 5;
    config.stack_size = 1024 * 5;
    // config.lru_purge_enable = true;
    // config.send_wait_timeout = 30;
    // config.recv_wait_timeout = 30;
    // config.task_priority = 6;
    // config.close_fn = ws_close_fn;
    config.max_uri_handlers = 16;

    // Start the httpd server
    ESP_LOGI(TAGH, "Starting server on port: '%d'", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAGH, "Registering URI handlers");
        if (httpd_register_uri_handler(server, &main_page) == ESP_OK &&
            httpd_register_uri_handler(server, &menu_post) == ESP_OK &&
            httpd_register_uri_handler(server, &update_get) == ESP_OK &&
            httpd_register_uri_handler(server, &update_post) == ESP_OK &&
            httpd_register_uri_handler(server, &ws) == ESP_OK)
            return server;
    }

    ESP_LOGE(TAGH, "Error starting server!");
    return NULL;
}

void wifi_task(void *arg)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    ESP_LOGI("mac AP", MACSTR, MAC2STR(mac));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    uint32_t ulNotifiedValue;

    while (1)
    {
        /* Ожидание оповещения безконечно, для запуска WiFi. */
        xTaskNotifyWait(pdFALSE,          /* Не очищать биты на входе. */
                        ULONG_MAX,        // ULONG_MAX, /* Очистка всех бит на выходе. */
                        &ulNotifiedValue, /* Сохраняет значение оповещения. */
                        portMAX_DELAY);

        // ESP_LOGD("WIFI ulNotifiedValue", "0x%lX", ulNotifiedValue);

        if ((ulNotifiedValue & NOTYFY_WIFI) != 0)
        {
            wifi_init_softap(1, 0); // WiFi

            /* Start the server for the first time */
            start_webserver();

            /* Mark current app as valid */
            const esp_partition_t *partition = esp_ota_get_running_partition();
            // printf("Currently running partition: %s\r\n", partition->label);
            ESP_LOGI(TAGW, "Currently running partition: %s", partition->label);

            esp_ota_img_states_t ota_state;
            if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK)
            {
                if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
                {
                    esp_ota_mark_app_valid_cancel_rollback();
                }
            }
            reset_sleep_timeout();
        }
        else if ((ulNotifiedValue & NOTYFY_WIFI_ESPNOW) != 0)
        {
            wifi_init_softap(1, 1);
        }

        // WIFI loop
        if (ulNotifiedValue & (NOTYFY_WIFI | NOTYFY_WIFI_ESPNOW))
        {
            while (1)
            {
                xTaskNotifyWait(pdFALSE,          /* очищать биты на входе. */
                                ULONG_MAX,        // ULONG_MAX, /* Очистка всех бит на выходе. */
                                &ulNotifiedValue, /* Сохраняет значение оповещения. */
                                pdMS_TO_TICKS(100));

                if ((ulNotifiedValue & NOTYFY_WIFI_STOP) != 0)
                {
                    esp_wifi_stop();
                    esp_wifi_deinit();
                    break;
                }
                else if ((ulNotifiedValue & NOTYFY_WIFI_REBOOT) != 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_wifi_stop();
                    esp_wifi_deinit();
                    esp_restart();
                }

                if (esp_timer_get_time() - timeout_begin > (get_menu_val_by_id("waitwifi") * 60LL * 1000000LL))
                {
                    if (xHandleWifi)
                        xTaskNotify(xHandleWifi, NOTYFY_WIFI_REBOOT, eSetValueWithOverwrite);
                }
            }
        }
    }
}
