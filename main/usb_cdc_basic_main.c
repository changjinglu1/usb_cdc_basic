/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "iot_usbh_cdc.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "sys/socket.h"
#ifdef CONFIG_ESP32_S3_USB_OTG
#include "bsp/esp-bsp.h"
#endif

static const char *TAG = "cdc_basic_demo";
static const char *TAG_WIFI = "wifi_config";
static const char *TAG_NVS = "nvs_config";
static const char *TAG_TCP = "TCP_Client";
static esp_netif_t *sta_netif = NULL;         // 全局变量，只创建一次
static EventGroupHandle_t s_wifi_event_group; // 用于 WiFi 连接状态,用于同步“是否已获取 IP”

/* ringbuffer size */
#define IN_RINGBUF_SIZE (1024 * 1)
#define OUT_RINGBUF_SIZE (1024 * 1)

/* enable interface num */
#define EXAMPLE_BULK_ITF_NUM 1        // 设备端口数量，默认1个
#define WIFI_CONNECTED_BIT BIT0       // WiFi 已连接
#define STORAGE_NAMESPACE "wifi_info" // NVS 存储空间名称
#define KEY_SSID "ssid"
#define KEY_PASSWD "passwd"
#define SERVER_IP "192.168.196.86" // 替换为 PC 的 IP 地址
#define SERVER_PORT 1234           // 替换为 PC 的端口号

/* choose if use user endpoint descriptors */
#define EXAMPLE_CONFIG_USER_EP_DESC

static void wifi_apply_config_and_connect(const wifi_config_t *wifi_cfg);
static esp_err_t store_wifi_information(const char *ssid, const char *passwd);

static void usb_receive_task(void *param)
{
    usbh_cdc_handle_t *handle = (usbh_cdc_handle_t *)param;
    uint8_t buf[IN_RINGBUF_SIZE + 1]; // +1 用于 '\0'
    size_t data_len;

    while (1)
    {
        for (size_t i = 0; i < EXAMPLE_BULK_ITF_NUM; i++)
        {
            // 取可读长度
            usbh_cdc_get_rx_buffer_size(handle[i], &data_len);
            if (data_len > 0)
            {
                // 限制到缓冲区大小，避免越界
                size_t to_read = data_len;
                if (to_read > IN_RINGBUF_SIZE)
                {
                    to_read = IN_RINGBUF_SIZE;
                }

                // 读取并 NUL 终止
                usbh_cdc_read_bytes(handle[i], buf, &to_read, pdMS_TO_TICKS(100));
                buf[to_read] = '\0';

                // 安全打印
                ESP_LOGI(TAG, "Device %d Receive len=%d: %.*s",
                         i, (int)to_read, (int)to_read, buf);

                char *ssid = strtok((char *)buf, "#");
                char *passwd = strtok(NULL, "#");

                if (ssid && passwd)
                {
                    wifi_config_t current_cfg = {0};
                    esp_wifi_get_config(WIFI_IF_STA, &current_cfg);

                    bool ssid_changed = strncmp((char *)current_cfg.sta.ssid, ssid, sizeof(current_cfg.sta.ssid)) != 0;
                    bool passwd_changed = strncmp((char *)current_cfg.sta.password, passwd, sizeof(current_cfg.sta.password)) != 0;

                    if (ssid_changed || passwd_changed)
                    {
                        ESP_LOGI(TAG_WIFI, "New credentials received, updating NVS and reconnecting...");
                        esp_err_t err = store_wifi_information(ssid, passwd); // 存储到 NVS

                        if (err == ESP_OK)
                        {
                            wifi_config_t cfg = {.sta = {{0}}};
                            strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
                            strncpy((char *)cfg.sta.password, passwd, sizeof(cfg.sta.password) - 1);

                            wifi_apply_config_and_connect(&cfg);
                        }

                        else
                        {
                            ESP_LOGE(TAG_WIFI, "Failed to store Wi-Fi credentials to NVS");
                        }
                    }
                    else
                    {
                        ESP_LOGI(TAG_WIFI, "Received credentials match current config, skipping reconnect.");
                    }
                }
            }

            // 让出 CPU，避免看门狗报警
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void tcp_client_task(void *pvParameters)
{
    const TickType_t wait_per_try = pdMS_TO_TICKS(10000);    // 每次等待 Wi-Fi 连接的超时
    const TickType_t retry_delay_min = pdMS_TO_TICKS(2000);  // 最小重试延时
    const TickType_t retry_delay_max = pdMS_TO_TICKS(30000); // 最大重试延时
    TickType_t retry_delay = retry_delay_min;

    char rx_buffer[128];
    const char tx_buffer[] = "Hello from ESP32S3";

    while (1)
    {
        // 1) 等待 Wi‑Fi 连接，未连接则指数退避延时重试
        EventBits_t bits = xEventGroupWaitBits(
            s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE, // 不清除位
            pdTRUE,  // 所有位均需满足
            wait_per_try);

        if (!(bits & WIFI_CONNECTED_BIT))
        {
            ESP_LOGW(TAG_TCP, "Wi-Fi not connected, retrying...");
            vTaskDelay(retry_delay);
            // 指数退避
            if (retry_delay < retry_delay_max)
            {
                TickType_t next = retry_delay * 2;
                retry_delay = (next > retry_delay_max) ? retry_delay_max : next;
            }
            continue; // 回到 for(;;) 继续等待 Wi‑Fi
        }
        // 一旦连上，重置退避
        retry_delay = retry_delay_min;

        // 2) 解析服务器地址
        struct sockaddr_in dest_addr = {0};
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(SERVER_PORT);
        dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
        if (dest_addr.sin_addr.s_addr == INADDR_NONE)
        {
            ESP_LOGE(TAG_TCP, "Invalid SERVER_IP: %s", SERVER_IP);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // 3) 建立 socket
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0)
        {
            ESP_LOGE(TAG_TCP, "Unable to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // 让 recv 有超时，防止无尽阻塞
        struct timeval tv = {.tv_sec = 5, .tv_usec = 0};
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        ESP_LOGI(TAG_TCP, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);

        if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
        {
            ESP_LOGE(TAG_TCP, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue; // 回到等待 Wi‑Fi/重连流程（此时 Wi‑Fi 仍可能在线）
        }

        ESP_LOGI(TAG_TCP, "Successfully connected");

        // 4) 会话循环：若 Wi‑Fi 掉线或对端关闭，跳出并重试
        while (1)
        {
            // 若 Wi‑Fi 掉线，主动退出会话循环
            if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT))
            {
                ESP_LOGW(TAG_TCP, "Wi-Fi lost, closing socket and waiting to reconnect");
                break;
            }

            int err = send(sock, tx_buffer, strlen(tx_buffer), 0);
            if (err < 0)
            {
                ESP_LOGE(TAG_TCP, "Error occurred during sending: errno %d", errno);
                break;
            }

            ESP_LOGI(TAG_TCP, "Message sent");

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0)
            {
                ESP_LOGE(TAG_TCP, "recv failed or timed out: errno %d", errno);
                // 这里可以选择继续循环（心跳场景）或断开重连
                break;
            }
            else if (len == 0)
            {
                ESP_LOGI(TAG_TCP, "Connection closed by peer");
                break;
            }
            else
            {
                rx_buffer[len] = 0;
                ESP_LOGI(TAG_TCP, "Received: %s", rx_buffer);
            }

            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        close(sock);
        // 回到 for(;;) 顶部：若 Wi‑Fi 已断，等待；若仍在线，直接尝试重连服务器
    }

    // 一般不退出；如需退出可在外部发任务通知，并在此处 vTaskDelete(NULL);
}

static void usb_connect_callback(usbh_cdc_handle_t cdc_handle, void *user_data)
{
    usbh_cdc_desc_print(cdc_handle);
    TaskHandle_t task_hdl = (TaskHandle_t)user_data;
    vTaskResume(task_hdl);
    ESP_LOGI(TAG, "Device Connected!");
}

static void usb_disconnect_callback(usbh_cdc_handle_t cdc_handle, void *user_data)
{
    ESP_LOGW(TAG, "Device Disconnected!");
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, // wifi 事件回调
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        ESP_LOGI(TAG_WIFI, "WIFI_EVENT: event_id=%ld", event_id);
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG_WIFI, "WIFI_EVENT_STA_START: WiFi STA 启动");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG_WIFI, "WIFI_EVENT_STA_CONNECTED: 已连接到 AP");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            wifi_event_sta_disconnected_t *d = event_data;
            ESP_LOGW(TAG_WIFI, "断开，reason=%d", d->reason);
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        default:
            ESP_LOGI(TAG_WIFI, "其他 WIFI_EVENT: event_id=%ld", event_id);
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        if (event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t *info = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG_WIFI, "Got IP:" IPSTR, IP2STR(&info->ip_info.ip));
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        else
        {
            ESP_LOGI(TAG_WIFI, "IP_EVENT id=%ld", event_id);
        }
    }
}

static void init_wifi_sta(void) // 初始化 WiFi STA 模块
{
    // 1) 创建事件组
    s_wifi_event_group = xEventGroupCreate();

    // 2) 底层初始化
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3) 注册事件回调
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));

    // 4) 创建默认 STA 网络接口
    sta_netif = esp_netif_create_default_wifi_sta();

    // 5) 初始化 Wi-Fi 驱动
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    // 6) 设置模式并启动
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "WiFi STA initialized");
}

static void wifi_apply_config_and_connect(const wifi_config_t *wifi_cfg) // 动态应用 SSID/密码并连接
{
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifi_cfg));
    ESP_LOGI(TAG_WIFI, "New WiFi config applied: SSID=%s", wifi_cfg->sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_connect());

    // 阻塞等待 GOT_IP（超时 10s）,完成 DHCP
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdTRUE,  // 读取后自动清除
                                           pdFALSE, // 不必等待所有位
                                           pdMS_TO_TICKS(10000));
    if (!(bits & WIFI_CONNECTED_BIT))
    {
        ESP_LOGW(TAG_WIFI, "Failed to get IP within 10s");
    }
}

// 将 SSID/密码写入 NVS
static esp_err_t store_wifi_information(const char *ssid, const char *passwd)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle); // 打开 NVS 存储空间
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_WIFI, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_set_str(handle, KEY_SSID, ssid);
    if (ret == ESP_OK)
    {
        ret = nvs_set_str(handle, KEY_PASSWD, passwd);
    }
    if (ret == ESP_OK)
    {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_LOGI(TAG_WIFI, "Stored SSID/Passwd to NVS");
    return ret;
}

// 从 NVS 读取 SSID/密码到缓冲区，缓冲区需足够大
static esp_err_t load_wifi_information(char *ssid, size_t ssid_len,
                                       char *passwd, size_t passwd_len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG_WIFI, "No stored Wi-Fi information, error: %s", esp_err_to_name(ret));
        return ret;
    }
    // 读取 SSID
    ret = nvs_get_str(handle, KEY_SSID, ssid, &ssid_len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_WIFI, "Failed to get SSID: %s", esp_err_to_name(ret));
    }
    // 读取密码
    if (ret == ESP_OK)
    {
        ret = nvs_get_str(handle, KEY_PASSWD, passwd, &passwd_len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG_WIFI, "Failed to get Passwd: %s", esp_err_to_name(ret));
        }
    }
    nvs_close(handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_WIFI, "Loaded SSID/Passwd from NVS");
    }
    return ret;
}

void app_main(void)
{
#ifdef CONFIG_ESP32_S3_USB_OTG
    bsp_usb_mode_select_host();
    bsp_usb_host_power_mode(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
#endif

    // 1) 初始化 NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    init_wifi_sta();
    // 尝试从 NVS 中加载上次配置
    char ssid[33] = {0};
    char passwd[65] = {0};
    if (load_wifi_information(ssid, sizeof(ssid), passwd, sizeof(passwd)) == ESP_OK)
    {
        wifi_config_t cfg = {0};
        strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
        strncpy((char *)cfg.sta.password, passwd, sizeof(cfg.sta.password) - 1);
        wifi_apply_config_and_connect(&cfg);
    }
    else
    {
        ESP_LOGW(TAG_WIFI, "No saved Wi-Fi config found");
    }
    /* install usbh cdc driver with skip_init_usb_host_driver */
    usbh_cdc_driver_config_t config = {
        .task_stack_size = 1024 * 4,
        .task_priority = 5,
        .task_coreid = 0,
        .skip_init_usb_host_driver = false,
    };
    /* install USB host CDC driver */
    usbh_cdc_driver_install(&config);

    usbh_cdc_handle_t handle[EXAMPLE_BULK_ITF_NUM] = {};

    usbh_cdc_device_config_t dev_config = {
        .vid = 0,
        .pid = 0,
        .itf_num = 0,
        .rx_buffer_size = IN_RINGBUF_SIZE,
        .tx_buffer_size = OUT_RINGBUF_SIZE,
        .cbs = {
            .connect = usb_connect_callback,
            .disconnect = usb_disconnect_callback,
            .user_data = xTaskGetCurrentTaskHandle(),
        },
    };

    usbh_cdc_create(&dev_config, &handle[0]);

#if (EXAMPLE_BULK_ITF_NUM > 1)
    dev_config.itf_num = 3;
    memset(&dev_config.cbs, 0, sizeof(dev_config.cbs));
    ESP_LOGI(TAG, "Open interface num: %d with first USB CDC Device", dev_config.itf_num);
    usbh_cdc_create(&dev_config, &handle[1]);
#endif
    /*!< Wait for the USB device to be connected */
    vTaskSuspend(NULL);

    /* Create a task for USB data processing */
    xTaskCreate(usb_receive_task, "usb_rx", 4096, (void *)handle, 2, NULL);
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

    /* Repeatedly sent AT through USB */
    static const uint8_t raw_frame[] = {0xA5, 0xB1, 0x02, 0x00, 0x00};

    static const size_t raw_len = sizeof(raw_frame);
    usbh_cdc_write_bytes(handle[0], raw_frame, raw_len, pdMS_TO_TICKS(100));
    //  usbh_cdc_write_bytes(handle[0], raw_frame, raw_len, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Send itf0 len=%d", raw_len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, raw_frame, raw_len, ESP_LOG_INFO);
    while (1)
    {

#if (EXAMPLE_BULK_ITF_NUM > 1)
        len = strlen(buff);
        usbh_cdc_write_bytes(handle[1], (uint8_t *)buff, len, pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Send itf1 len=%d: %s", len, buff);
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
