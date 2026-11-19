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
#include "driver/gpio.h"

uint8_t mac[6];
extern esp_ip4_addr_t pdp_ip;
extern char net_status_current[32];

#define AP_WIFI_SSID "Piloramka"
#define AP_WIFI_PASS "123123123"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1


static const char *TAGW = "wifi";
static const char *TAGH = "httpd";

#define WS_MAX_PAYLOAD 256

static esp_netif_t *s_ap_netif = NULL;
static esp_event_handler_instance_t s_wifi_event_instance = NULL;
static httpd_handle_t s_httpd = NULL;

static int s_retry_num = 0;

static char network_buf[CONFIG_LWIP_TCP_MSS];

static int64_t timeout_begin;

void reset_wifi_timeout()
{
    timeout_begin = esp_timer_get_time();
    // ESP_LOGV(TAGW, "Timeout reset");
    // xEventGroupSetBits(status_event_group, WIFI_ACTIVE);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        reset_wifi_timeout();
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
        ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAGW, "got ip:" IPSTR, IP2STR(&ip_event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        reset_wifi_timeout();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        reset_wifi_timeout();
        wifi_event_ap_staconnected_t *ap_connected_event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAGW, "station " MACSTR " join, AID=%d", MAC2STR(ap_connected_event->mac), ap_connected_event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *ap_disconnected_event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAGW, "station " MACSTR " leave, AID=%d", MAC2STR(ap_disconnected_event->mac), ap_disconnected_event->aid);
    }
}

esp_err_t wifi_init_softap(uint8_t channel, uint8_t ssid_hidden)
{
    ESP_LOGD("wifi_init_softap", "Start");
    if (!s_ap_netif)
        s_ap_netif = esp_netif_create_default_wifi_ap();
    if (!s_ap_netif)
    {
        ESP_LOGE(TAGW, "Failed to create default AP netif");
        return ESP_ERR_NO_MEM;
    }
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &s_wifi_event_instance));
    wifi_config_t ap_config = {
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
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ap_config.ap.ssid_len = snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s%.0f", AP_WIFI_SSID, get_menu_val_by_id("idn"));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20)); // иначе не работает 11 канал
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44)); // влияние на измерения АЦП незначительные
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(20));

    int8_t power = 0;
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));

    ESP_LOGI(TAGW, "wifi_init_softap finished. SSID:%s password:%s channel:%d power:%d", AP_WIFI_SSID, AP_WIFI_PASS, ap_config.ap.channel, power);
    return ESP_OK;
}

static esp_err_t menu_get_handler(httpd_req_t *req)
{
    int chunk_length = 0;

    const char html_head[] = "<!DOCTYPE html>\n"
                         "<html><head>"
                         "<meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\" />"
                         "<meta name=\"viewport\" content=\"width=device-width\" />"
                         "<title>Настройки</title>"
                         "<meta charset=\"utf-8\"></head><body>";

    const char html_form_start[] = "<form name=\"settings\" action=\"\" method=\"post\">"
                         "<div id=\"settingscontent\">";

    const char html_form_end[] = "</div><input type=\"submit\" value=\"Submit\" /></form><br><textarea id=\"txt\" rows=\"10\" cols=\"100\" readonly></textarea>";

    const char html_debug[] = "<div>"
                         "<button onclick=\"sendMessage('DAC=0')\">DEBUG! DAC = 0%</button>&nbsp;&nbsp;&nbsp;"
                         "<button onclick=\"sendMessage('DAC=127')\">DEBUG! DAC = 50%</button>&nbsp;&nbsp;&nbsp;"
                         "<button onclick=\"sendMessage('DAC=255')\">DEBUG! DAC = 100%</button>"
                         "</div><div>"
                         "<button onclick=\"sendMessage('Current=10')\">DEBUG! Current = 10A</button>&nbsp;&nbsp;&nbsp;"
                         "<button onclick=\"sendMessage('Current=50')\">DEBUG! Current = 50A</button>"
                         "</div>"
                         "<br><div style=\"display: flex; gap: 10px;\">"
                         "</div>"
                         "<script>\nconst socket = new WebSocket(\"ws://\" + location.host + \"/ws\");\n"
                         "const textarea = document.getElementById(\"txt\");"
                         "socket.onmessage = function (event) {"
                         "textarea.value += event.data + \"\\n\";"
                         "textarea.scrollTop = textarea.scrollHeight;"
                         "};"
                         "function sendMessage(value) {if (socket.readyState === WebSocket.OPEN) {socket.send(value.toString())};}"
                         "</script>"
                         "</body></html>";

    httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
    httpd_resp_send_chunk(req, html_head, sizeof(html_head) - 1);

    char mac_line[128];
    int mac_line_length = sprintf(mac_line, "<b>MAC address:</b> " MACSTR "<br>", MAC2STR(mac));
    httpd_resp_send_chunk(req, mac_line, mac_line_length);

    httpd_resp_send_chunk(req, html_form_start, sizeof(html_form_start) - 1);
    reset_menu_html();
    do
    {
        chunk_length = build_menu_html(network_buf);
        if (chunk_length > 0)
            httpd_resp_send_chunk(req, network_buf, chunk_length);
    } while (chunk_length > 0);

    httpd_resp_send_chunk(req, html_form_end, sizeof(html_form_end) - 1);
    httpd_resp_send_chunk(req, html_debug, sizeof(html_debug) - 1);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t menu_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= (int)sizeof(network_buf))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid form size");
        return ESP_FAIL;
    }

    int remaining_bytes = req->content_len;
    int received_bytes = 0;
    int retries = 3;
    while (remaining_bytes > 0 && retries > 0)
    {
        int space_left = (int)sizeof(network_buf) - 1 - received_bytes;
        if (space_left <= 0)
            break;

        /* Read the data for the request */
        int recv_len = httpd_req_recv(req, network_buf + received_bytes, MIN(remaining_bytes, space_left));
        if (recv_len <= 0)
        {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* Retry receiving if timeout occurred */
                vTaskDelay(100 / portTICK_PERIOD_MS);
                retries--;
                continue;
            }
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Form receive error");
            return ESP_FAIL;
        }

        received_bytes += recv_len;
        remaining_bytes -= recv_len;
    }

    if (remaining_bytes != 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete form");
        return ESP_FAIL;
    }

    network_buf[received_bytes] = '\0';

    char *cursor = network_buf;
    char field_name[16];
    int mac_low = 0;
    bool mac_valid = true;
    uint8_t mac_bytes[6];
    while (cursor && *cursor)
    {
        char *equals = strchr(cursor, '=');
        if (!equals)
        {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Malformed form");
            return ESP_FAIL;
        }

        *equals = '\0';
        int field_name_length = (int)(equals - cursor);
        if (field_name_length <= 0 || field_name_length >= (int)sizeof(field_name))
        {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid field name");
            return ESP_FAIL;
        }
        snprintf(field_name, sizeof(field_name), "%.*s", field_name_length, cursor);

        float field_value = 0;
        bool field_valid = true;
        if (strcmp(field_name, "ipaddr") == 0)
        {
            int parsed_count = sscanf((const char *)equals + 1, "%hhu.%hhu.%hhu.%hhu", &mac_bytes[0], &mac_bytes[1], &mac_bytes[2], &mac_bytes[3]);
            if (parsed_count == 4)
                field_value = (mac_bytes[0] << 0) | (mac_bytes[1] << 8) | (mac_bytes[2] << 16) | (mac_bytes[3] << 24);
        }
        else if (strcmp(field_name, "MAC1") == 0)
        {
            //&MAC1=00%3A00%3A00%3A00%3A00%3A51&MAC2=81
            int parsed_count = sscanf((const char *)equals + 1,
                                "%hhx%%3A%hhx%%3A%hhx%%3A%hhx%%3A%hhx%%3A%hhx",
                                &mac_bytes[0], &mac_bytes[1], &mac_bytes[2], &mac_bytes[3], &mac_bytes[4], &mac_bytes[5]);

            if (parsed_count == 6)
            {
                field_value = (mac_bytes[0] << 16) | (mac_bytes[1] << 8) | mac_bytes[2];
                mac_low = (mac_bytes[3] << 16) | (mac_bytes[4] << 8) | mac_bytes[5];
            }
            else
            {
                field_valid = false;
                mac_valid = false;
            }
        }
        else if (strcmp(field_name, "MAC2") == 0)
        {
            if (mac_valid)
                field_value = mac_low;
            else
                field_valid = false;
        }
        else
        {
            field_value = atof(equals + 1);
        }

        if (field_valid)
            set_menu_val_by_id(field_name, field_value);

        cursor = strchr(equals + 1, '&');
        if (cursor)
            cursor = cursor + 1;
    }

    if (xHandleWifi)
        xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);

    // End response
    return menu_get_handler(req);
}

#define ESP_IMAGE_HEADER_MAGIC 0xE9 /*!< The magic word for the esp_image_header_t structure. */

/*
 * Handle OTA file upload
 */
esp_err_t ota_get_handler(httpd_req_t *req)
{
    static const char update_page[] =
        "<!DOCTYPE html>\n"
        "<html>\n"
        "  <head>\n"
        "    <meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\" />\n"
        "\t<meta name=\"viewport\" content=\"width=device-width\">\n"
        "    <title>ESP32 OTA Update</title>\n"
        "    <script>\n"
        "      function startUpload() {\n"
        "        var otafile = document.getElementById(\"otafile\").files;\n"
        "\n"
        "        if (otafile.length == 0) {\n"
        "          alert(\"No file selected!\");\n"
        "        } else {\n"
        "          document.getElementById(\"otafile\").disabled = true;\n"
        "          document.getElementById(\"upload\").disabled = true;\n"
        "\n"
        "          var file = otafile[0];\n"
        "          var xhr = new XMLHttpRequest();\n"
        "          xhr.onreadystatechange = function () {\n"
        "            if (xhr.readyState == 4) {\n"
        "              if (xhr.status == 200) {\n"
        "                document.open();\n"
        "                document.write(xhr.responseText);\n"
        "                document.close();\n"
        "              } else if (xhr.status == 0) {\n"
        "                alert(\"Server closed the connection abruptly!\");\n"
        "                location.reload();\n"
        "              } else {\n"
        "                alert(xhr.status + \" Error!\\n\" + xhr.responseText);\n"
        "                location.reload();\n"
        "              }\n"
        "            }\n"
        "          };\n"
        "\n"
        "          xhr.upload.onprogress = function (e) {\n"
        "            var progress = document.getElementById(\"progress\");\n"
        "            progress.textContent =\n"
        "              \"Progress: \" + ((e.loaded / e.total) * 100).toFixed(0) + \"%\";\n"
        "          };\n"
        "          xhr.open(\"POST\", \"/update\", true);\n"
        "          xhr.send(file);\n"
        "        }\n"
        "      }\n"
        "    </script>\n"
        "  </head>\n"
        "  <body>\n"
        "    <h1>ESP32 OTA Firmware Update</h1>\n"
        "    <div>\n"
        "      <label for=\"otafile\">Firmware or SPIFFS file:</label>\n"
        "      <input type=\"file\" id=\"otafile\" name=\"otafile\" />\n"
        "    </div>\n"
        "    <div>\n"
        "      <button id=\"upload\" type=\"button\" onclick=\"startUpload()\">Upload</button>\n"
        "    </div>\n"
        "    <div id=\"progress\"></div>\n"
        "  </body>\n"
        "</html>\n";
    return httpd_resp_send(req, update_page, sizeof(update_page) - 1);
}

esp_err_t ota_post_handler(httpd_req_t *req)
{
    int upload_type = -1;
    esp_ota_handle_t ota_handle;
    int remaining_bytes = req->content_len;

    reset_wifi_timeout();

    /* Пишем в next_ota и прошивку и spiffs.bin*/
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
    if (!ota_partition)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA partition not found");
        return ESP_FAIL;
    }
    int retries = 3;
    while (remaining_bytes > 0 && retries > 0)
    {
        int recv_len = httpd_req_recv(req, network_buf, MIN(remaining_bytes, sizeof(network_buf)));

        // Timeout Error: Just retry
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            retries--;
            continue;

            // Serious Error: Abort OTA
        }
        else if (recv_len <= 0)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
            return ESP_FAIL;
        }

        if (upload_type == -1) // first data block
        {
            upload_type = (uint8_t)network_buf[0];

            if (upload_type == ESP_IMAGE_HEADER_MAGIC)
            {
                ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));
            }
            else if (remaining_bytes == 0x50000) // SPIFFS Image
            {
                upload_type = remaining_bytes;
                ESP_ERROR_CHECK(esp_partition_erase_range(ota_partition, 0, 0x50000));
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File type Error");
                return ESP_FAIL;
            }
        }

        // firmware.bin
        if (upload_type == ESP_IMAGE_HEADER_MAGIC)
        {
            // Successful Upload: Flash firmware chunk
            if (esp_ota_write(ota_handle, (const void *)network_buf, recv_len) != ESP_OK)
            {
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write Error");
                return ESP_FAIL;
            }
        }
        else if (upload_type == 0x50000) // spiffs.bin
        {
            ESP_ERROR_CHECK(esp_partition_write(ota_partition, (req->content_len - remaining_bytes), (const void *)network_buf, recv_len));
        }

        vTaskDelay(1);
        remaining_bytes -= recv_len;
    }

    if (remaining_bytes != 0)
    {
        if (upload_type == ESP_IMAGE_HEADER_MAGIC)
            esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Incomplete upload");
        return ESP_FAIL;
    }

    if (upload_type == -1)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty upload");
        return ESP_FAIL;
    }

    if (upload_type == ESP_IMAGE_HEADER_MAGIC)
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
            xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);
    }
    else if (upload_type == 0x50000)
    {
        const esp_partition_t *storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
        if (!storage_partition)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Storage partition not found");
            return ESP_FAIL;
        }
        ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, 0, 0x50000));

        remaining_bytes = req->content_len;
        int recv_len = sizeof(network_buf);
        while (remaining_bytes > 0)
        {
            recv_len = MIN(remaining_bytes, sizeof(network_buf));
            esp_err_t err = esp_partition_read(ota_partition, (req->content_len - remaining_bytes), (void *)network_buf, recv_len);
            if (err != ESP_OK)
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash read Error");
                return ESP_FAIL;
            }
            ESP_ERROR_CHECK(esp_partition_write(storage_partition, (req->content_len - remaining_bytes), (const void *)network_buf, recv_len));
            vTaskDelay(1);
            remaining_bytes -= recv_len;
        }

        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_sendstr(req, "SPIFFS update complete, rebooting now!\n");
        ESP_LOGW(TAGH, "SPIFFS update complete, rebooting now!");

        if (xHandleWifi)
            xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);
    }

    return ESP_OK;
}

struct t_async_resp_arg
{
    httpd_handle_t hd;
    int fd;
} async_resp_arg;

typedef struct
{
    httpd_handle_t hd;
    int fd;
    httpd_ws_frame_t frame;
    uint8_t payload[256];
} ws_work_t;

static ws_work_t ws_work;
static volatile bool ws_work_busy = false;

static void ws_send_work(void *arg)
{
    ws_work_t *work = (ws_work_t *)arg;
    httpd_ws_send_frame_async(work->hd, work->fd, &work->frame);
    ws_work_busy = false;
}

esp_err_t ws_send_text(const char *payload, const int payload_length)
{
    if (payload_length <= 0 || payload_length >= (int)sizeof(ws_work.payload))
        return ESP_ERR_INVALID_SIZE;

    if (!async_resp_arg.hd || ws_work_busy)
        return ESP_ERR_INVALID_STATE;

    ws_work.hd = async_resp_arg.hd;
    ws_work.fd = async_resp_arg.fd;
    memcpy(ws_work.payload, payload, payload_length);
    ws_work.frame.len = payload_length;
    ws_work.frame.payload = ws_work.payload;
    ws_work.frame.type = HTTPD_WS_TYPE_TEXT;

    ws_work_busy = true;
    esp_err_t err = httpd_queue_work(ws_work.hd, ws_send_work, &ws_work);
    if (err != ESP_OK)
    {
        ws_work_busy = false;
        if (err == ESP_ERR_INVALID_ARG)
            async_resp_arg.hd = 0;
    }

    return err;
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    async_resp_arg.fd = httpd_req_to_sockfd(req);
    async_resp_arg.hd = req->handle;

    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAGH, "WS handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_frame;
    uint8_t *payload_buffer = NULL;
    memset(&ws_frame, 0, sizeof(httpd_ws_frame_t));
    ws_frame.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t err = httpd_ws_recv_frame(req, &ws_frame, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAGH, "httpd_ws_recv_frame failed to get frame len with %d", err);
        return err;
    }
    ESP_LOGI(TAGH, "frame len is %d", ws_frame.len);
    if (ws_frame.len > WS_MAX_PAYLOAD)
    {
        ESP_LOGW(TAGH, "WebSocket frame too large: %d", ws_frame.len);
        return ESP_ERR_INVALID_SIZE;
    }
    if (ws_frame.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        payload_buffer = calloc(1, ws_frame.len + 1);
        if (payload_buffer == NULL)
        {
            ESP_LOGE(TAGH, "Failed to calloc memory for payload_buffer");
            return ESP_ERR_NO_MEM;
        }
        ws_frame.payload = payload_buffer;
        /* Set max_len = ws_pkt.len to get the frame payload */
        err = httpd_ws_recv_frame(req, &ws_frame, ws_frame.len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAGH, "httpd_ws_recv_frame failed with %d", err);
            free(payload_buffer);
            return err;
        }
        ESP_LOGI(TAGH, "Got packet with message: %s", ws_frame.payload);
    }

    ESP_LOGI(TAGH, "websocket_handler: httpd_handle_t=%p, sockfd=%d, client_info:%d", req->handle, httpd_req_to_sockfd(req), httpd_ws_get_fd_info(req->handle, httpd_req_to_sockfd(req)));

    if (ws_frame.len == 0 || ws_frame.payload == NULL)
    {
        free(payload_buffer);
        return ESP_OK;
    }

    if (!ws_frame.final || ws_frame.type != HTTPD_WS_TYPE_TEXT)
    {
        free(payload_buffer);
        return ESP_OK;
    }

    if (strncmp("DAC=255", (const char *)ws_frame.payload, 7) == 0)
    {
        run_stage_set(100);
    }
    if (strncmp("DAC=127", (const char *)ws_frame.payload, 7) == 0)
    {
        run_stage_set(101);
    }
    if (strncmp("DAC=0", (const char *)ws_frame.payload, 5) == 0)
    {
        run_stage_set(102);
    }
    if (strncmp("Current=10", (const char *)ws_frame.payload, 10) == 0)
    {
        run_stage_set(110);
    }
    if (strncmp("Current=50", (const char *)ws_frame.payload, 10) == 0)
    {
        run_stage_set(111);
    }

    free(payload_buffer);
    return err;
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
    .handler = ota_get_handler,
    .user_ctx = NULL};

static const httpd_uri_t update_post = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = ota_post_handler,
    .user_ctx = NULL};

static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = websocket_handler,
    .user_ctx = NULL,
    .is_websocket = true};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    reset_menu_html();
    ws_work_busy = false;
    async_resp_arg.hd = NULL;
    async_resp_arg.fd = -1;
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    // config.max_open_sockets = 5;
    server_config.stack_size = 1024 * 5;
    // config.lru_purge_enable = true;
    // config.send_wait_timeout = 30;
    // config.recv_wait_timeout = 30;
    // config.task_priority = 6;
    // config.close_fn = ws_close_fn;
    server_config.max_uri_handlers = 16;

    // Start the httpd server
    ESP_LOGI(TAGH, "Starting server on port: '%d'", server_config.server_port);

    if (httpd_start(&server, &server_config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAGH, "Registering URI handlers");
        if (httpd_register_uri_handler(server, &main_page) == ESP_OK &&
            httpd_register_uri_handler(server, &menu_post) == ESP_OK &&
            httpd_register_uri_handler(server, &update_get) == ESP_OK &&
            httpd_register_uri_handler(server, &update_post) == ESP_OK &&
            httpd_register_uri_handler(server, &ws) == ESP_OK)
        {
            s_httpd = server;
            return server;
        }
        httpd_stop(server);
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

    uint32_t notified_value;

    while (1)
    {
        /* Ожидание оповещения безконечно, для запуска WiFi. */
        xTaskNotifyWait(pdTRUE,           /* очищать биты на входе. */
                        ULONG_MAX,        // ULONG_MAX, /* Очистка всех бит на выходе. */
                        &notified_value, /* Сохраняет значение оповещения. */
                        portMAX_DELAY);

        // ESP_LOGD("WIFI ulNotifiedValue", "0x%lX", ulNotifiedValue);

        if ((notified_value & REBOOT_NOW) != 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
        if ((notified_value & NOTYFY_WIFI_STOP) != 0)
            continue;

        esp_err_t wifi_err;
        if ((notified_value & NOTYFY_WIFI_ESPNOW) != 0)
        {
            wifi_err = wifi_init_softap(1, 1);
        }
        else
        {
            wifi_err = wifi_init_softap(1, 0); // WiFi
        }

        if (wifi_err == ESP_OK)
        {
            if (xWifiReadySemaphore)
                xSemaphoreGive(xWifiReadySemaphore); // ESP-NOW may initialize now
        }
        else
        {
            ESP_LOGE(TAGW, "WiFi softAP start failed: %s", esp_err_to_name(wifi_err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Start the server for the first time */
        if (s_httpd) // предыдущий экземпляр мог остаться после STOP
        {
            httpd_stop(s_httpd);
            s_httpd = NULL;
        }
        s_httpd = start_webserver();

        /* Mark current app as valid */
        const esp_partition_t *running_partition = esp_ota_get_running_partition();
        // printf("Currently running partition: %s\r\n", partition->label);
        ESP_LOGI(TAGW, "Currently running partition: %s", running_partition->label);

        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running_partition, &ota_state) == ESP_OK)
        {
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
            {
                esp_ota_mark_app_valid_cancel_rollback();
            }
        }
        reset_wifi_timeout();

        // WIFI loop
        while (1)
        {
            xTaskNotifyWait(pdTRUE,           /* очищать биты на входе. */
                            ULONG_MAX,        // ULONG_MAX, /* Очистка всех бит на выходе. */
                            &notified_value, /* Сохраняет значение оповещения. */
                            portMAX_DELAY);   // 300 / portTICK_PERIOD_MS);

            if ((notified_value & NOTYFY_WIFI_STOP) != 0)
            {
                ws_work_busy = false;
                async_resp_arg.hd = NULL;
                async_resp_arg.fd = -1;
                if (s_httpd)
                {
                    httpd_stop(s_httpd);
                    s_httpd = NULL;
                }
                esp_wifi_stop();
                esp_wifi_deinit();
                if (s_wifi_event_instance)
                {
                    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_event_instance);
                    s_wifi_event_instance = NULL;
                }
                if (s_ap_netif)
                {
                    esp_netif_destroy_default_wifi(s_ap_netif);
                    s_ap_netif = NULL;
                }
                break;
            }
            else if ((notified_value & NOTYFY_WIFI_SWITCH) != 0)
            {
                wifi_config_t ap_config;
                if (esp_wifi_get_config(WIFI_IF_AP, &ap_config) == ESP_OK)
                {
                    if (ap_config.ap.ssid_hidden == 1)
                    {
                        ap_config.ap.ssid_hidden = 0;
                        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
                    }
                }
            }
            else if ((notified_value & REBOOT_NOW) != 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_wifi_stop();
                esp_wifi_deinit();
                esp_restart();
            }
            /*
                            if (esp_timer_get_time() - timeout_begin > (get_menu_val_by_id("waitwifi") * 60LL * 1000000LL))
                            {
                                if (xHandleWifi)
                                    xTaskNotify(xHandleWifi, REBOOT_NOW, eSetValueWithOverwrite);
                            }
            */
        }
    }
}
