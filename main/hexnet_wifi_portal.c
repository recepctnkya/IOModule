#include "hexnet_wifi_portal.h"
#include "hexnet_debug.h"
#include "hexnet_wifi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dns_server.h"
#define LOG_LOCAL_LEVEL HEXNET_LOG_LEVEL_WIFI
#include "hexnet_log.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"
#include "nvs.h"

static const char *TAG = "WiFiPortal";
#define PORTAL_NVS_NAMESPACE "hexnet"
#define PORTAL_NVS_KEY_PROV "wifi_prov"

#define PORTAL_SCAN_MAX_AP   24
#define PORTAL_HTML_MAX      12288
#define PORTAL_SCAN_CACHE_MS 20000
#define PORTAL_CONNECT_FAIL_RESTART_MS 90000

static httpd_handle_t s_httpd;
static dns_server_handle_t s_dns;
static char s_ap_ssid[32];
static bool s_portal_active;
static bool s_scan_running;
static bool s_bringup_scheduled;

static wifi_ap_record_t s_scan_cache[PORTAL_SCAN_MAX_AP];
static uint16_t s_scan_cache_count;
static int64_t s_scan_cache_ms;

static const char HTML_STYLE[] =
    "<style>"
    "*{box-sizing:border-box}"
    "body{margin:0;min-height:100vh;font-family:Segoe UI,Roboto,Arial,sans-serif;"
    "background:linear-gradient(160deg,#e8f4f8 0%,#f5f7fa 45%,#eef1f5 100%);"
    "display:flex;align-items:center;justify-content:center;padding:16px}"
    ".card{width:100%;max-width:400px;background:#fff;border-radius:18px;"
    "padding:28px 24px 22px;box-shadow:0 12px 40px rgba(15,45,75,.12)}"
    ".brand{margin-bottom:6px;text-align:center;background:#000;border-radius:12px;"
    "padding:14px 16px}"
    ".brand img{width:100%;max-width:240px;height:auto;display:block;margin:0 auto}"
    ".brand-sub{text-align:center;font-size:11px;font-weight:700;color:#1a3a52;"
    "letter-spacing:1.4px;margin:0 0 20px}"
    "h1{font-size:22px;margin:0 0 6px;color:#1a2b3c;font-weight:700}"
    ".lead{color:#6b7c8f;font-size:14px;margin:0 0 22px}"
    "label{display:block;font-size:13px;font-weight:600;color:#3d4f61;margin:0 0 6px}"
    ".field{position:relative;margin-bottom:16px}"
    "select,input{width:100%;padding:12px 14px 12px 36px;font-size:15px;border:1px solid #d5dee8;"
    "border-radius:10px;background:#f9fbfd;color:#1a2b3c;appearance:none}"
    "select{padding-right:36px}"
    ".ico{position:absolute;left:12px;top:50%;transform:translateY(-50%);opacity:.45;font-size:14px}"
    ".eye{position:absolute;right:10px;top:50%;transform:translateY(-50%);border:0;background:0;"
    "font-size:18px;cursor:pointer;opacity:.5;padding:4px 8px}"
    ".btn{width:100%;margin-top:8px;padding:14px;border:0;border-radius:10px;"
    "background:linear-gradient(90deg,#0e9ab0,#12b3c7);color:#fff;font-size:15px;font-weight:700;"
    "letter-spacing:.8px;cursor:pointer}"
    ".info{margin-top:18px;padding:12px 14px;background:#f0f4f8;border-radius:10px;"
    "font-size:12px;color:#5a6b7d;line-height:1.45}"
    ".hint{font-size:12px;color:#0e9ab0;text-align:center;margin-top:12px}"
    "a{color:#0e9ab0;text-decoration:none}"
    "</style>";

extern const uint8_t _binary_hexnet_logo_png_start[] asm("_binary_hexnet_logo_png_start");
extern const uint8_t _binary_hexnet_logo_png_end[] asm("_binary_hexnet_logo_png_end");

static void build_ap_ssid(void);
static void portal_configure_ap_netif(void);
static void portal_ap_bringup_task(void *arg);
static esp_err_t http_404_to_root(httpd_req_t *req, httpd_err_code_t err);

static bool is_factory_partition(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    return (running != NULL && strcmp(running->label, "factory") == 0);
}

bool hexnet_wifi_portal_is_factory_boot(void)
{
    return is_factory_partition();
}

bool hexnet_wifi_portal_is_active(void)
{
    return s_portal_active;
}

static bool is_provisioned_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(PORTAL_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    uint8_t prov = 0;
    esp_err_t err = nvs_get_u8(handle, PORTAL_NVS_KEY_PROV, &prov);
    nvs_close(handle);
    return (err == ESP_OK && prov != 0);
}

bool hexnet_wifi_portal_is_provisioned(void)
{
    return is_provisioned_nvs();
}

void hexnet_wifi_portal_mark_provisioned(void)
{
    nvs_handle_t handle;
    if (nvs_open(PORTAL_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    (void)nvs_set_u8(handle, PORTAL_NVS_KEY_PROV, 1);
    (void)nvs_commit(handle);
    nvs_close(handle);
}

static void build_ap_ssid(void)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) == ESP_OK) {
        snprintf(s_ap_ssid, sizeof(s_ap_ssid), "Hexnet-IO-%02X%02X", mac[4], mac[5]);
    } else {
        snprintf(s_ap_ssid, sizeof(s_ap_ssid), "Hexnet-IO-Setup");
    }
}

const char *hexnet_wifi_portal_get_ap_ssid(void)
{
    if (s_ap_ssid[0] == '\0') {
        build_ap_ssid();
    }
    return s_ap_ssid;
}

static int cmp_ap_rssi_desc(const void *a, const void *b)
{
    const wifi_ap_record_t *ap_a = (const wifi_ap_record_t *)a;
    const wifi_ap_record_t *ap_b = (const wifi_ap_record_t *)b;
    return (int)ap_b->rssi - (int)ap_a->rssi;
}

static uint16_t dedupe_scan_results(wifi_ap_record_t *list, uint16_t count)
{
    uint16_t out = 0;
    for (uint16_t i = 0; i < count; i++) {
        bool dup = false;
        for (uint16_t j = 0; j < out; j++) {
            if (strcmp((const char *)list[i].ssid, (const char *)list[j].ssid) == 0) {
                dup = true;
                break;
            }
        }
        if (!dup && list[i].ssid[0] != '\0') {
            if (out != i) {
                list[out] = list[i];
            }
            out++;
        }
    }
    return out;
}

static void load_scan_results_from_driver(void)
{
    uint16_t count = PORTAL_SCAN_MAX_AP;
    if (esp_wifi_scan_get_ap_records(&count, s_scan_cache) != ESP_OK) {
        s_scan_cache_count = 0;
        return;
    }
    qsort(s_scan_cache, count, sizeof(wifi_ap_record_t), cmp_ap_rssi_desc);
    s_scan_cache_count = dedupe_scan_results(s_scan_cache, count);
    s_scan_cache_ms = esp_timer_get_time() / 1000LL;
    ESP_LOGI(TAG, "Scan: %u networks", (unsigned)s_scan_cache_count);
}

void hexnet_wifi_portal_on_scan_done(void)
{
    s_scan_running = false;
    load_scan_results_from_driver();
}

void hexnet_wifi_portal_request_scan(void)
{
    if (s_scan_running) {
        return;
    }
    wifi_scan_config_t scan_cfg = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, false);
    if (err == ESP_OK) {
        s_scan_running = true;
    } else {
        ESP_LOGW(TAG, "scan_start: %s", esp_err_to_name(err));
    }
}

static size_t html_escape(const char *src, char *dst, size_t dst_len)
{
    if (!src || !dst || dst_len == 0) {
        return 0;
    }
    size_t w = 0;
    for (const char *p = src; *p && w + 1 < dst_len; p++) {
        const char *rep = NULL;
        char single[2] = {*p, '\0'};
        switch (*p) {
        case '&':
            rep = "&amp;";
            break;
        case '<':
            rep = "&lt;";
            break;
        case '>':
            rep = "&gt;";
            break;
        case '"':
            rep = "&quot;";
            break;
        default:
            rep = single;
            break;
        }
        size_t rl = strlen(rep);
        if (w + rl >= dst_len) {
            break;
        }
        memcpy(dst + w, rep, rl);
        w += rl;
    }
    dst[w] = '\0';
    return w;
}

static void parse_form_field(const char *body, const char *key, char *out, size_t out_len)
{
    if (!body || !key || !out || out_len == 0) {
        return;
    }
    out[0] = '\0';
    char pattern[24];
    snprintf(pattern, sizeof(pattern), "%s=", key);
    const char *p = strstr(body, pattern);
    if (!p) {
        return;
    }
    p += strlen(pattern);
    size_t i = 0;
    while (*p && *p != '&' && i + 1 < out_len) {
        if (*p == '+') {
            out[i++] = ' ';
        } else if (*p == '%' && p[1] && p[2]) {
            unsigned v = 0;
            if (sscanf(p + 1, "%2x", &v) == 1) {
                out[i++] = (char)v;
            }
            p += 2;
        } else {
            out[i++] = *p;
        }
        p++;
    }
    out[i] = '\0';
}

static esp_err_t send_setup_page(httpd_req_t *req)
{
    int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (s_scan_cache_count == 0 || (s_scan_cache_ms > 0 && (now_ms - s_scan_cache_ms) > PORTAL_SCAN_CACHE_MS)) {
        hexnet_wifi_portal_request_scan();
    }

    char *html = (char *)malloc(PORTAL_HTML_MAX);
    if (!html) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
        return ESP_FAIL;
    }

    size_t pos = 0;
    pos += (size_t)snprintf(
        html + pos,
        PORTAL_HTML_MAX - pos,
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>Hexnet Wi-Fi</title>%s",
        HTML_STYLE);
    if (s_scan_cache_count == 0 && s_scan_running) {
        pos += (size_t)snprintf(html + pos, PORTAL_HTML_MAX - pos, "<meta http-equiv=\"refresh\" content=\"2\">");
    }
    pos += (size_t)snprintf(
        html + pos,
        PORTAL_HTML_MAX - pos,
        "<script>function togglePwd(){var p=document.getElementById('pwd');"
        "p.type=p.type==='password'?'text':'password';}</script></head><body>"
        "<div class=\"card\"><div class=\"brand\">"
        "<img src=\"/logo.png\" alt=\"HEXNET\"></div>"
        "<p class=\"brand-sub\">IO MODULE</p>"
        "<h1>Wi-Fi Kurulumu</h1>"
        "<p class=\"lead\">Cihazınızı yönlendiricinize bağlayın</p>"
        "<form method=\"POST\" action=\"/save\">"
        "<label>Wi-Fi Ağı (SSID)</label>"
        "<div class=\"field\"><span class=\"ico\">&#128246;</span>"
        "<select name=\"net\" id=\"ssid_sel\" required>");

    if (s_scan_cache_count == 0) {
        pos += (size_t)snprintf(
            html + pos,
            PORTAL_HTML_MAX - pos,
            "<option value=\"\" disabled selected>Ag taranıyor...</option>");
    } else {
        char esc[96];
        for (uint16_t i = 0; i < s_scan_cache_count && pos + 256 < PORTAL_HTML_MAX; i++) {
            html_escape((const char *)s_scan_cache[i].ssid, esc, sizeof(esc));
            pos += (size_t)snprintf(
                html + pos,
                PORTAL_HTML_MAX - pos,
                "<option value=\"%u\">%s</option>",
                (unsigned)i,
                esc);
        }
    }

    pos += (size_t)snprintf(
        html + pos,
        PORTAL_HTML_MAX - pos,
        "</select></div>"
        "<label>Şifre</label>"
        "<div class=\"field\"><span class=\"ico\">&#128274;</span>"
        "<input type=\"password\" name=\"password\" id=\"pwd\" maxlength=\"64\" "
        "placeholder=\"Wi-Fi şifrenizi girin\" autocomplete=\"off\">"
        "<button type=\"button\" class=\"eye\" onclick=\"togglePwd()\" aria-label=\"Göster\">&#128065;</button>"
        "</div>"
        "<button type=\"submit\" class=\"btn\">&#128246; BAĞLAN</button>"
        "</form>"
        "<div class=\"info\">&#9432; Bağlantı sağlanamazsa cihaz otomatik olarak yeniden başlatılacaktır.</div>"
        "<p class=\"hint\"><a href=\"/\">Listeyi yenile</a> &middot; http://" HEXNET_PORTAL_IP "</p>"
        "</div></body></html>");

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    esp_err_t ret = httpd_resp_send(req, html, pos);
    free(html);
    return ret;
}

static void portal_configure_ap_netif(void)
{
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap == NULL) {
        ESP_LOGE(TAG, "WIFI_AP_DEF netif missing");
        return;
    }

    esp_netif_ip_info_t ip = {0};
    IP4_ADDR(&ip.ip, 192, 168, 1, 1);
    IP4_ADDR(&ip.gw, 192, 168, 1, 1);
    IP4_ADDR(&ip.netmask, 255, 255, 255, 0);

    esp_err_t err = esp_netif_dhcps_stop(ap);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(TAG, "dhcps_stop: %s", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap, &ip));

    esp_netif_dns_info_t dns = {0};
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ip.ip.addr;
    ESP_ERROR_CHECK(esp_netif_set_dns_info(ap, ESP_NETIF_DNS_MAIN, &dns));

    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap));
    ESP_LOGI(TAG, "AP netif ready: http://" HEXNET_PORTAL_IP);
}

static void portal_ap_bringup_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(400));
    portal_configure_ap_netif();
    vTaskDelay(pdMS_TO_TICKS(100));
    hexnet_wifi_portal_start_http();
    if (s_dns == NULL) {
        dns_server_config_t dns_cfg = DNS_SERVER_CONFIG_SINGLE("*", "WIFI_AP_DEF");
        s_dns = start_dns_server(&dns_cfg);
        if (s_dns == NULL) {
            ESP_LOGW(TAG, "DNS server failed (use http://" HEXNET_PORTAL_IP ")");
        }
    }
    hexnet_wifi_portal_request_scan();
    s_bringup_scheduled = false;
    vTaskDelete(NULL);
}

void hexnet_wifi_portal_on_ap_started(void)
{
    if (s_bringup_scheduled || s_httpd != NULL) {
        return;
    }
    s_bringup_scheduled = true;
    xTaskCreate(portal_ap_bringup_task, "portal_ap", 8192, NULL, 5, NULL);
}

void hexnet_wifi_portal_shutdown(void)
{
    if (s_httpd != NULL) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
        ESP_LOGI(TAG, "HTTP portal stopped");
    }
    if (s_dns != NULL) {
        stop_dns_server(s_dns);
        s_dns = NULL;
        ESP_LOGI(TAG, "DNS portal stopped");
    }

    s_portal_active = false;
    s_bringup_scheduled = false;
    s_scan_running = false;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_mode STA: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Setup AP off; STA-only (internet + MQTT)");
        hexnet_debug_log("Kurulum portali kapandi; STA modu");
    }

    if (hexnet_wifi_has_saved_credentials()) {
        esp_wifi_connect();
    }
}

void hexnet_wifi_portal_open(void)
{
    if (s_portal_active && s_httpd != NULL) {
        return;
    }

    wifi_ap_record_t sta_ap = {0};
    if (esp_wifi_sta_get_ap_info(&sta_ap) == ESP_OK) {
        ESP_LOGW(TAG, "portal_open: STA hala bagli, once disconnect gerekli");
        return;
    }

    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap == NULL) {
        esp_netif_create_default_wifi_ap();
    }

    wifi_mode_t mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&mode) != ESP_OK || mode != WIFI_MODE_APSTA) {
        esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "set_mode APSTA: %s", esp_err_to_name(err));
            return;
        }
    }

    hexnet_wifi_portal_enable_ap();
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    if (s_httpd == NULL && !s_bringup_scheduled) {
        s_bringup_scheduled = true;
        xTaskCreate(portal_ap_bringup_task, "portal_ap", 8192, NULL, 5, NULL);
    }

    hexnet_debug_log("Kurulum portali acildi -> http://%s (AP: %s)",
                     HEXNET_PORTAL_IP,
                     hexnet_wifi_portal_get_ap_ssid());
}

static esp_err_t http_404_to_root(httpd_req_t *req, httpd_err_code_t err)
{
    if (err != HTTPD_404_NOT_FOUND) {
        return ESP_FAIL;
    }
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t logo_get_handler(httpd_req_t *req)
{
    const size_t len = (size_t)(_binary_hexnet_logo_png_end - _binary_hexnet_logo_png_start);
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    return httpd_resp_send(req, (const char *)_binary_hexnet_logo_png_start, len);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    return send_setup_page(req);
}

static void connect_fail_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(PORTAL_CONNECT_FAIL_RESTART_MS));
    if (!hexnet_wifi_is_connected()) {
        hexnet_debug_log("Portal baglantisi zaman asimi; kurulum sayfasi acik kalacak");
        hexnet_wifi_portal_open();
    }
    vTaskDelete(NULL);
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    char body[384] = {0};
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
        return ESP_FAIL;
    }
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, body + received, total - received);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }
        received += r;
    }
    body[received] = '\0';

    char net_idx[8] = {0};
    char password[65] = {0};
    parse_form_field(body, "net", net_idx, sizeof(net_idx));
    parse_form_field(body, "password", password, sizeof(password));

    unsigned idx = (unsigned)atoi(net_idx);
    if (idx >= s_scan_cache_count || s_scan_cache[idx].ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid network");
        return ESP_FAIL;
    }

    const char *ssid = (const char *)s_scan_cache[idx].ssid;

    wifi_config_t sta_cfg = {0};
    strlcpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, password, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode =
        (strlen(password) > 0) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_config: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_config");
        return ESP_FAIL;
    }

    hexnet_wifi_portal_mark_provisioned();
    hexnet_debug_log("Wi-Fi flash'a kaydedildi: SSID=\"%s\"", ssid);

    esp_wifi_disconnect();
    err = esp_wifi_connect();
    ESP_LOGI(TAG, "Connecting to \"%s\" (%s)", ssid, esp_err_to_name(err));
    hexnet_debug_log("Baglaniliyor: \"%s\"", ssid);

    xTaskCreate(connect_fail_restart_task, "portal_rst", 2048, NULL, 5, NULL);

    char esc[96];
    html_escape(ssid, esc, sizeof(esc));
    char resp[1280];
    int n = snprintf(
        resp,
        sizeof(resp),
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<style>body{font-family:sans-serif;background:#eef4f8;padding:24px}"
        ".card{max-width:400px;margin:auto;background:#fff;padding:24px;border-radius:16px}"
        "h1{color:#0e9ab0}.ok h1{color:#1a8f4c}</style></head><body>"
        "<div class=\"card\" id=\"card\"><h1 id=\"title\">Baglaniyor</h1>"
        "<p id=\"msg\"><b>%s</b> agina baglaniliyor...</p></div>"
        "<script>"
        "function done(){"
        "document.getElementById('card').className='card ok';"
        "document.getElementById('title').textContent='Ba\\u015far\\u0131l\\u0131';"
        "document.getElementById('msg').textContent="
        "'Ba\\u011flant\\u0131 kuruldu. Wi-Fi ba\\u011flant\\u0131s\\u0131 ba\\u015flad\\u0131.';"
        "setTimeout(function(){window.close();},1200);"
        "setTimeout(function(){location.replace('about:blank');},1800);"
        "}"
        "function poll(){fetch('/status').then(function(r){return r.json();})"
        ".then(function(d){if(d.connected)done();else setTimeout(poll,1000);})"
        ".catch(function(){setTimeout(poll,1500);});}"
        "setTimeout(poll,800);"
        "</script></body></html>",
        esc);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, resp, (size_t)n);
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    const bool connected = hexnet_wifi_is_connected() && hexnet_wifi_is_internet_ok();
    char json[48];
    int n = snprintf(json, sizeof(json), "{\"connected\":%s}", connected ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, (size_t)n);
}

static httpd_uri_t s_uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
};

static httpd_uri_t s_uri_status = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_get_handler,
};

static httpd_uri_t s_uri_logo = {
    .uri = "/logo.png",
    .method = HTTP_GET,
    .handler = logo_get_handler,
};

static httpd_uri_t s_uri_save = {
    .uri = "/save",
    .method = HTTP_POST,
    .handler = save_post_handler,
};

void hexnet_wifi_portal_start_http(void)
{
    if (s_httpd != NULL) {
        return;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = 10240;
    cfg.max_open_sockets = 7;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;

    esp_err_t err = httpd_start(&s_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        return;
    }

    httpd_register_uri_handler(s_httpd, &s_uri_root);
    httpd_register_uri_handler(s_httpd, &s_uri_status);
    httpd_register_uri_handler(s_httpd, &s_uri_logo);
    httpd_register_uri_handler(s_httpd, &s_uri_save);
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, http_404_to_root);
    ESP_LOGI(TAG, "HTTP server on http://" HEXNET_PORTAL_IP "/ (heap=%lu)", (unsigned long)esp_get_free_heap_size());
}

void hexnet_wifi_portal_enable_ap(void)
{
    build_ap_ssid();

    wifi_config_t ap_cfg = {0};
    strlcpy((char *)ap_cfg.ap.ssid, s_ap_ssid, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len = (uint8_t)strlen(s_ap_ssid);
    ap_cfg.ap.channel = 6;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    ap_cfg.ap.ssid_hidden = 0;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AP config: %s", esp_err_to_name(err));
        return;
    }

    s_portal_active = true;
    ESP_LOGI(TAG, "SoftAP \"%s\" -> http://" HEXNET_PORTAL_IP, s_ap_ssid);
}
