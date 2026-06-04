#pragma once

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

// ─── Config — fill these in ───────────────────────────────────────────
// #define WIFI_SSID       "S24+ de Agustin"
// #define WIFI_PASSWORD   "12345678"
//#define WIFI_SSID       "IZZI-0F89-5G"
//#define WIFI_PASSWORD   "3Q9F0CVQGMRO"
 #define WIFI_SSID       "S21"
 #define WIFI_PASSWORD   "wifi4040"
//#define WIFI_SSID       "IPhone"
//#define WIFI_PASSWORD   "ness12345"
#define SERVER_IP       "192.168.124.153"  // laptop IP
#define SERVER_PORT     5006            // laptop listens on this
#define LISTEN_PORT     5005            // ESP32 listens on this

static const char* NET_TAG = "Network";

// ─── Command struct ───────────────────────────────────────────────────
struct RobotCommand {
    char cmd[16];      // "move_joint","move_tcp","jog","estop","home"
    float j1, j2, j3, j4;
    float x,  y,  z,  alpha;
    char  axis[8];
    int   direction;
    bool  active;
    float step;
};

// ─── Queues ───────────────────────────────────────────────────────────
static QueueHandle_t s_cmdQueue = nullptr;  // internal — use networkInit() to get it
static int           udpSock   = -1;
static struct sockaddr_in serverAddr;

// ─── WiFi ─────────────────────────────────────────────────────────────
static void wifiEventHandler(void* arg, esp_event_base_t base,
                              int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(NET_TAG, "WiFi disconnected — reconnecting...");
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* e = (ip_event_got_ip_t*)data;
        ESP_LOGI(NET_TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
    }
}

void wifiInit()
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,  wifiEventHandler, nullptr);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifiEventHandler, nullptr);

    wifi_config_t wifiCfg = {};
    strncpy((char*)wifiCfg.sta.ssid,     WIFI_SSID,     sizeof(wifiCfg.sta.ssid));
    strncpy((char*)wifiCfg.sta.password, WIFI_PASSWORD, sizeof(wifiCfg.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifiCfg);
    esp_wifi_start();
    esp_wifi_connect();
    ESP_LOGI(NET_TAG, "Connecting to %s...", WIFI_SSID);
}

// ─── UDP setup ────────────────────────────────────────────────────────
void udpInit()
{
    udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    // bind to listen port
    struct sockaddr_in localAddr = {};
    localAddr.sin_family      = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port        = htons(LISTEN_PORT);
    bind(udpSock, (struct sockaddr*)&localAddr, sizeof(localAddr));

    // server address (laptop)
    serverAddr.sin_family      = AF_INET;
    serverAddr.sin_port        = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr);

    ESP_LOGI(NET_TAG, "UDP ready — listening on :%d, server at %s:%d",
             LISTEN_PORT, SERVER_IP, SERVER_PORT);
}

// ─── Send state to laptop ─────────────────────────────────────────────
void sendState(float j1, float j2, float j3, float j4,
               float j1v=0, float j2v=0, float j3v=0, float j4v=0)
{
    if (udpSock < 0) return;
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"j1\":%.2f,\"j2\":%.2f,\"j3\":%.2f,\"j4\":%.2f,"
             "\"j1v\":%.2f,\"j2v\":%.2f,\"j3v\":%.2f,\"j4v\":%.2f}",
             j1, j2, j3, j4, j1v, j2v, j3v, j4v);
    sendto(udpSock, buf, strlen(buf), 0,
           (struct sockaddr*)&serverAddr, sizeof(serverAddr));
}

// ─── Simple JSON parser helpers ───────────────────────────────────────
static float jsonFloat(const char* json, const char* key, float def = 0.0f)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char* p = strstr(json, search);
    if (!p) return def;
    p += strlen(search);
    while (*p == ' ') p++;
    return strtof(p, nullptr);
}

static void jsonStr(const char* json, const char* key, char* out, size_t len)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char* p = strstr(json, search);
    if (!p) { out[0] = 0; return; }
    p += strlen(search);
    size_t i = 0;
    while (*p && *p != '"' && i < len - 1)
        out[i++] = *p++;
    out[i] = 0;
}

// ─── Receive task ─────────────────────────────────────────────────────
void udpReceiveTask(void* arg)
{
    char buf[512];
    struct sockaddr_in src;
    socklen_t srcLen = sizeof(src);

    while (true)
    {
        int len = recvfrom(udpSock, buf, sizeof(buf) - 1, 0,
                           (struct sockaddr*)&src, &srcLen);
        if (len <= 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        buf[len] = '\0';

        ESP_LOGD(NET_TAG, "Received: %s", buf);

        RobotCommand cmd = {};
        jsonStr(buf, "cmd", cmd.cmd, sizeof(cmd.cmd));
        cmd.j1        = jsonFloat(buf, "j1");
        cmd.j2        = jsonFloat(buf, "j2");
        cmd.j3        = jsonFloat(buf, "j3");
        cmd.j4        = jsonFloat(buf, "j4");
        cmd.x         = jsonFloat(buf, "x");
        cmd.y         = jsonFloat(buf, "y");
        cmd.z         = jsonFloat(buf, "z");
        cmd.alpha     = jsonFloat(buf, "alpha");
        cmd.step      = jsonFloat(buf, "step", 5.0f);
        cmd.direction = (int)jsonFloat(buf, "direction", 1.0f);
        cmd.active    = jsonFloat(buf, "active") > 0.5f;
        jsonStr(buf, "axis", cmd.axis, sizeof(cmd.axis));

        if (s_cmdQueue)
            xQueueSend(s_cmdQueue, &cmd, 0);
    }
}

// ─── Init everything and start receive task ───────────────────────────
QueueHandle_t networkInit()
{
    s_cmdQueue = xQueueCreate(16, sizeof(RobotCommand));
    wifiInit();

    // wait until IP is assigned — check every 500ms up to 30 seconds
    for (int i = 0; i < 60; i++)
    {
        esp_netif_ip_info_t ip;
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0)
            break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    udpInit();
    xTaskCreate(udpReceiveTask, "udp_rx", 4096, nullptr, 5, nullptr);
    return s_cmdQueue;
}