/*
 * WiFi Provisioning via Captive Portal
 *
 * On first boot (or after credentials are erased) the ESP32 starts as an
 * Access Point ("ESP32-Setup").  A DNS server running on UDP/53 redirects
 * every hostname to 192.168.4.1 so that phones/laptops automatically open
 * the captive-portal page.  The HTTP server serves a small config page at
 * every URL and exposes two extra endpoints:
 *   GET  /scan  – returns a JSON array of visible SSIDs
 *   POST /save  – body: ssid=<name>&pass=<pw>  – saves & connects
 */

#include "wifi_prov_mgr.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define TAG             "wifi_prov"
#define AP_SSID         "ESP32-Setup"
#define AP_PASS         ""           /* open network – easier for users */
#define AP_CHANNEL      1
#define AP_MAX_CONN     4
#define AP_IP           "192.168.4.1"

#define NVS_NAMESPACE   "wifi_creds"
#define NVS_KEY_SSID    "ssid"
#define NVS_KEY_PASS    "pass"

#define STA_MAX_RETRY   3

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* ------------------------------------------------------------------ */
/*  Globals                                                            */
/* ------------------------------------------------------------------ */

static EventGroupHandle_t  s_wifi_eg;
static int                 s_retry_num = 0;
static wifi_prov_state_t   s_prov_state = WIFI_PROV_STATE_IDLE;

wifi_prov_state_t wifi_prov_get_state(void) { return s_prov_state; }
static bool               s_dns_run   = false;
static httpd_handle_t     s_httpd     = NULL;

/* ------------------------------------------------------------------ */
/*  NVS helpers                                                        */
/* ------------------------------------------------------------------ */

static esp_err_t nvsRead(char *ssid_out, size_t ssid_len,
                         char *pass_out, size_t pass_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    err = nvs_get_str(h, NVS_KEY_SSID, ssid_out, &ssid_len);
    if (err == ESP_OK)
        err = nvs_get_str(h, NVS_KEY_PASS, pass_out, &pass_len);

    nvs_close(h);
    return err;
}

static esp_err_t nvsWrite(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_str(h, NVS_KEY_SSID, ssid);
    if (err == ESP_OK) err = nvs_set_str(h, NVS_KEY_PASS, pass);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void wifi_prov_erase_credentials(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
}

/* ------------------------------------------------------------------ */
/*  WiFi event handler                                                 */
/* ------------------------------------------------------------------ */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < STA_MAX_RETRY) {
            s_retry_num++;
            ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d", s_retry_num, STA_MAX_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "WiFi connect failed after %d retries", STA_MAX_RETRY);
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry_num = 0;
        s_prov_state = WIFI_PROV_STATE_CONNECTED;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Client connected to AP");
        s_prov_state = WIFI_PROV_STATE_AP_CLIENT;

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Client disconnected from AP");
        /* Only revert if we haven't connected to the router yet */
        if (s_prov_state != WIFI_PROV_STATE_CONNECTED) {
            s_prov_state = WIFI_PROV_STATE_DISCONNECTED;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  STA-only connection (credentials already known)                   */
/* ------------------------------------------------------------------ */

static esp_err_t sta_connect(const char *ssid, const char *pass)
{
    s_retry_num = 0;
    xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid,     ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(30000));
    if (bits & WIFI_CONNECTED_BIT) return ESP_OK;

    ESP_LOGE(TAG, "sta_connect: failed for SSID \"%s\"", ssid);
    esp_wifi_stop();
    return ESP_FAIL;
}

/* ------------------------------------------------------------------ */
/*  DNS server (redirects everything to AP_IP)                        */
/* ------------------------------------------------------------------ */

static void dns_server_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS socket create failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "DNS bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DNS server listening on UDP/53");

    uint8_t buf[512];
    while (s_dns_run) {
        struct sockaddr_in client;
        socklen_t client_len = sizeof(client);
        int len = recvfrom(sock, buf, sizeof(buf) - 20, 0,
                           (struct sockaddr *)&client, &client_len);
        if (len < 12) continue;

        /* Build minimal DNS A-record response pointing to AP_IP */
        uint8_t resp[512];
        memcpy(resp, buf, len);

        resp[2] = 0x81; /* QR=1, AA=1                */
        resp[3] = 0x80; /* RA=1, RCODE=0 (no error)  */
        /* keep QDCOUNT as-is */
        resp[6] = 0x00; /* ANCOUNT high               */
        resp[7] = 0x01; /* ANCOUNT low  = 1           */
        resp[8] = 0x00; resp[9] = 0x00;  /* NSCOUNT */
        resp[10]= 0x00; resp[11]= 0x00;  /* ARCOUNT */

        int rlen = len;
        resp[rlen++] = 0xC0; /* pointer to question name */
        resp[rlen++] = 0x0C;
        resp[rlen++] = 0x00; resp[rlen++] = 0x01; /* type A  */
        resp[rlen++] = 0x00; resp[rlen++] = 0x01; /* class IN*/
        resp[rlen++] = 0x00; resp[rlen++] = 0x00;
        resp[rlen++] = 0x00; resp[rlen++] = 0x3C; /* TTL 60s */
        resp[rlen++] = 0x00; resp[rlen++] = 0x04; /* RDLEN   */
        /* IP 192.168.4.1 */
        resp[rlen++] = 192; resp[rlen++] = 168;
        resp[rlen++] = 4;   resp[rlen++] = 1;

        sendto(sock, resp, rlen, 0,
               (struct sockaddr *)&client, client_len);
    }

    close(sock);
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  HTML page (embedded)                                               */
/* ------------------------------------------------------------------ */

static const char PROV_HTML[] =
"<!DOCTYPE html>"
"<html lang='en'><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>HEXNET TECHNOLOGY – WiFi Configuration</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{"
"  font-family:'Segoe UI',Arial,sans-serif;"
"  background:linear-gradient(160deg,#040e22 0%,#071a3e 60%,#0a2352 100%);"
"  min-height:100vh;display:flex;flex-direction:column;"
"  align-items:center;padding:24px 16px 40px;"
"}"
".hdr{text-align:center;padding:28px 0 20px;width:100%;max-width:440px}"
".brand{"
"  font-size:11px;letter-spacing:5px;color:#4fc3f7;"
"  text-transform:uppercase;font-weight:700;margin-bottom:10px"
"}"
".divider{"
"  width:40px;height:2px;"
"  background:linear-gradient(90deg,transparent,#1565c0,transparent);"
"  margin:8px auto"
"}"
"h1{font-size:26px;font-weight:800;color:#fff;letter-spacing:2px;margin:6px 0 4px}"
".sub{font-size:11px;letter-spacing:3px;color:#90caf9;text-transform:uppercase}"
".card{"
"  background:rgba(255,255,255,0.04);"
"  border:1px solid rgba(79,195,247,0.18);"
"  border-radius:14px;padding:30px 26px;"
"  width:100%;max-width:440px;"
"  box-shadow:0 8px 32px rgba(0,0,0,0.4)"
"}"
"label{"
"  display:block;font-size:11px;font-weight:700;"
"  color:#4fc3f7;letter-spacing:1.5px;"
"  text-transform:uppercase;margin:20px 0 7px"
"}"
"select,input[type=password],input[type=text]{"
"  width:100%;padding:12px 14px;"
"  background:rgba(4,14,34,0.8);"
"  border:1px solid rgba(79,195,247,0.25);"
"  border-radius:8px;color:#e3f2fd;"
"  font-size:14px;outline:none;"
"  transition:border-color .2s,box-shadow .2s"
"}"
"select:focus,input:focus{"
"  border-color:#4fc3f7;"
"  box-shadow:0 0 0 3px rgba(79,195,247,0.12)"
"}"
"select option{background:#071a3e}"
".btn{"
"  width:100%;padding:13px;border:none;"
"  border-radius:8px;font-size:14px;"
"  font-weight:700;cursor:pointer;"
"  letter-spacing:1px;transition:all .2s;margin-top:12px"
"}"
".btn-connect{"
"  background:linear-gradient(90deg,#1565c0 0%,#0288d1 100%);"
"  color:#fff;margin-top:24px;"
"  box-shadow:0 4px 15px rgba(21,101,192,0.4)"
"}"
".btn-connect:hover{background:linear-gradient(90deg,#1976d2,#039be5);"
"  box-shadow:0 4px 20px rgba(21,101,192,0.6)}"
".btn-scan{"
"  background:rgba(79,195,247,0.08);"
"  border:1px solid rgba(79,195,247,0.3);"
"  color:#4fc3f7;font-size:13px"
"}"
".btn-scan:hover{background:rgba(79,195,247,0.16)}"
"#msg{"
"  margin-top:18px;padding:13px;border-radius:8px;"
"  display:none;font-size:13px;text-align:center;line-height:1.5"
"}"
".ok{background:rgba(0,200,83,0.1);border:1px solid rgba(0,200,83,0.35);color:#69f0ae}"
".err{background:rgba(229,57,53,0.1);border:1px solid rgba(229,57,53,0.35);color:#ff8a80}"
".footer{"
"  margin-top:28px;font-size:10px;"
"  color:rgba(144,202,249,0.3);letter-spacing:2px;text-align:center"
"}"
"</style></head>"
"<body>"
"<div class='hdr'>"
"  <div class='brand'>&#9632; HEXNET TECHNOLOGY &#9632;</div>"
"  <div class='divider'></div>"
"  <h1>VANGO</h1>"
"  <div class='sub'>WiFi Configuration</div>"
"</div>"
"<div class='card'>"
"  <form id='f'>"
"    <label>Available Networks</label>"
"    <select id='ssid' name='ssid'>"
"      <option value=''>&#8212; tap Scan to search &#8212;</option>"
"    </select>"
"    <button type='button' class='btn btn-scan' onclick='doScan()'>&#128269; Scan for Networks</button>"
"    <label>Or Enter SSID Manually</label>"
"    <input type='text' id='manual' placeholder='Network name'>"
"    <label>Password</label>"
"    <input type='password' name='pass' id='pass' placeholder='Leave blank for open networks'>"
"    <button type='submit' class='btn btn-connect'>&#10003;&ensp;Connect</button>"
"  </form>"
"  <div id='msg'></div>"
"</div>"
"<div class='footer'>HEXNET TECHNOLOGY &copy; VANGO SYSTEM</div>"
"<script>"
"function doScan(){"
"  var b=document.querySelector('.btn-scan');"
"  b.textContent='Scanning...';"
"  fetch('/scan').then(function(r){return r.json();}).then(function(nets){"
"    var s=document.getElementById('ssid');"
"    s.innerHTML='<option value=\"\">&#8212; select network &#8212;</option>';"
"    nets.forEach(function(n){"
"      var o=document.createElement('option');"
"      o.value=n.ssid;"
"      o.textContent=n.ssid+' ('+n.rssi+' dBm)';"
"      s.appendChild(o);"
"    });"
"    b.textContent='&#128269; Scan Again';"
"  }).catch(function(){b.textContent='Scan failed – retry';});"
"}"
"document.getElementById('f').onsubmit=function(e){"
"  e.preventDefault();"
"  var ssid=document.getElementById('ssid').value||document.getElementById('manual').value;"
"  var pass=document.getElementById('pass').value;"
"  if(!ssid){alert('Please select or enter a network name');return;}"
"  var m=document.getElementById('msg');"
"  m.className='ok';m.style.display='block';"
"  m.textContent='Connecting, please wait up to 30 seconds...';"
"  var fd=new URLSearchParams();"
"  fd.append('ssid',ssid);fd.append('pass',pass);"
"  fetch('/save',{method:'POST',body:fd})"
"  .then(function(r){return r.text();})"
"  .then(function(t){m.textContent=t;})"
"  .catch(function(){m.className='err';m.textContent='Request failed – check your connection';});"
"};"
"</script>"
"</body></html>";

/* ------------------------------------------------------------------ */
/*  HTTP handlers                                                      */
/* ------------------------------------------------------------------ */

/* Serve config page for any GET request */
static esp_err_t http_root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PROV_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* /scan – returns JSON array [{ssid,rssi}, ...] */
static esp_err_t http_scan_handler(httpd_req_t *req)
{
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL, .bssid = NULL,
        .channel = 0, .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    esp_wifi_scan_start(&scan_cfg, true); /* blocking */

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count > 20) ap_count = 20;

    wifi_ap_record_t *ap_list = calloc(ap_count, sizeof(wifi_ap_record_t));
    if (!ap_list) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    esp_wifi_scan_get_ap_records(&ap_count, ap_list);

    char *buf = malloc(ap_count * 80 + 4);
    if (!buf) { free(ap_list); httpd_resp_send_500(req); return ESP_FAIL; }

    int pos = 0;
    pos += sprintf(buf + pos, "[");
    for (int i = 0; i < ap_count; i++) {
        char ssid_esc[66] = {0};
        /* simple escaping – replace " with ' */
        int si = 0;
        for (int j = 0; j < (int)strlen((char *)ap_list[i].ssid) && si < 63; j++) {
            char c = ap_list[i].ssid[j];
            ssid_esc[si++] = (c == '"') ? '\'' : c;
        }
        pos += sprintf(buf + pos, "%s{\"ssid\":\"%s\",\"rssi\":%d}",
                       i ? "," : "", ssid_esc, ap_list[i].rssi);
    }
    pos += sprintf(buf + pos, "]");

    free(ap_list);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, pos);
    free(buf);
    return ESP_OK;
}

/* Simple URL-encoded body parser – extracts value of `key` into out (max outlen) */
static bool parse_form_field(const char *body, const char *key,
                             char *out, size_t outlen)
{
    size_t klen = strlen(key);
    const char *p = body;
    while (*p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            p += klen + 1;
            size_t i = 0;
            while (*p && *p != '&' && i < outlen - 1) {
                if (*p == '+') {
                    out[i++] = ' ';
                    p++;
                } else if (*p == '%' && p[1] && p[2]) {
                    char hex[3] = { p[1], p[2], 0 };
                    out[i++] = (char)strtol(hex, NULL, 16);
                    p += 3;
                } else {
                    out[i++] = *p++;
                }
            }
            out[i] = '\0';
            return true;
        }
        while (*p && *p != '&') p++;
        if (*p == '&') p++;
    }
    return false;
}

/* Shared state: new credentials submitted via POST /save */
static char s_new_ssid[64];
static char s_new_pass[64];
static volatile bool s_creds_received = false;

/* POST /save  body: ssid=<x>&pass=<y> */
static esp_err_t http_save_handler(httpd_req_t *req)
{
    char body[256] = {0};
    int  len = req->content_len;
    if (len <= 0 || len >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad request");
        return ESP_FAIL;
    }
    int r = httpd_req_recv(req, body, len);
    if (r <= 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Read error");
        return ESP_FAIL;
    }
    body[r] = '\0';

    char ssid[64] = {0};
    char pass[64] = {0};
    if (!parse_form_field(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing SSID");
        return ESP_FAIL;
    }
    parse_form_field(body, "pass", pass, sizeof(pass));

    ESP_LOGI(TAG, "Received credentials for SSID: %s", ssid);

    /* Try to connect before saving */
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(500));

    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid,     ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    s_retry_num = 0;
    xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    esp_wifi_start();
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(30000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Could not connect – check SSID/password and try again.");
        return ESP_FAIL;
    }

    /* Connection succeeded – persist and signal main task */
    nvsWrite(ssid, pass);
    strlcpy(s_new_ssid, ssid, sizeof(s_new_ssid));
    strlcpy(s_new_pass, pass, sizeof(s_new_pass));
    s_creds_received = true;

    const char *ok_msg = "Connected! The device will now switch to normal operation.";
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, ok_msg, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* Catch-all handler: redirect captive-portal probes to the config page */
static esp_err_t http_catchall_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" AP_IP "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  HTTP server                                                        */
/* ------------------------------------------------------------------ */

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;
    cfg.uri_match_fn     = httpd_uri_match_wildcard;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    static const httpd_uri_t uri_root = {
        .uri = "/", .method = HTTP_GET, .handler = http_root_handler
    };
    static const httpd_uri_t uri_scan = {
        .uri = "/scan", .method = HTTP_GET, .handler = http_scan_handler
    };
    static const httpd_uri_t uri_save = {
        .uri = "/save", .method = HTTP_POST, .handler = http_save_handler
    };
    static const httpd_uri_t uri_any = {
        .uri = "/*", .method = HTTP_GET, .handler = http_catchall_handler
    };

    httpd_register_uri_handler(s_httpd, &uri_root);
    httpd_register_uri_handler(s_httpd, &uri_scan);
    httpd_register_uri_handler(s_httpd, &uri_save);
    httpd_register_uri_handler(s_httpd, &uri_any);

    ESP_LOGI(TAG, "HTTP server started on port 80");
}

/* ------------------------------------------------------------------ */
/*  AP + captive portal provisioning                                   */
/* ------------------------------------------------------------------ */

static void start_provisioning_ap(void)
{
    /* Configure AP */
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid            = AP_SSID,
            .ssid_len        = strlen(AP_SSID),
            .channel         = AP_CHANNEL,
            .password        = AP_PASS,
            .max_connection  = AP_MAX_CONN,
            .authmode        = (strlen(AP_PASS) == 0) ? WIFI_AUTH_OPEN
                                                       : WIFI_AUTH_WPA2_PSK,
        },
    };

    /* APSTA so we can also scan for networks from the config page */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started: SSID=\"%s\"  IP=%s", AP_SSID, AP_IP);

    /* DNS server */
    s_dns_run = true;
    xTaskCreate(dns_server_task, "dns_srv", 4096, NULL, 5, NULL);

    /* HTTP server */
    start_http_server();

    ESP_LOGI(TAG, "Captive portal ready – connect to \"%s\" and open a browser", AP_SSID);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

esp_err_t wifi_prov_init(void)
{
    s_wifi_eg = xEventGroupCreate();

    /* Ensure NVS is initialised (safe to call if already done) */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    /* --- Try stored credentials first --- */
    char ssid[64] = {0};
    char pass[64] = {0};
    if (nvsRead(ssid, sizeof(ssid), pass, sizeof(pass)) == ESP_OK && ssid[0] != '\0') {
        ESP_LOGI(TAG, "Found stored credentials for SSID: %s", ssid);
        if (sta_connect(ssid, pass) == ESP_OK) {
            ESP_LOGI(TAG, "Connected using stored credentials");
            return ESP_OK;
        }
        /* Credentials exist but connection failed */
        s_prov_state = WIFI_PROV_STATE_DISCONNECTED;
        ESP_LOGW(TAG, "Stored credentials failed – starting provisioning AP");
        esp_wifi_stop();
    } else {
        /* No credentials at all – fresh provisioning */
        s_prov_state = WIFI_PROV_STATE_DISCONNECTED;
        ESP_LOGI(TAG, "No stored credentials – starting provisioning AP");
    }

    /* Reset retry counter so the /save handler gets a clean slate */
    s_retry_num = 0;
    xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    /* --- Start captive portal and wait for user to provision --- */
    start_provisioning_ap();

    while (!s_creds_received) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Shut down provisioning infrastructure */
    s_dns_run = false;
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }

    /* Switch to pure STA mode now that we are connected */
    esp_wifi_set_mode(WIFI_MODE_STA);

    ESP_LOGI(TAG, "Provisioning complete – WiFi connected as STA");
    return ESP_OK;
}
