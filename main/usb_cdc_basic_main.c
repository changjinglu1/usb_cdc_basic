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
#include "driver/gpio.h"
#include "driver/ledc.h"
#ifdef CONFIG_ESP32_S3_USB_OTG
#include "bsp/esp-bsp.h"
#endif

static const char *TAG = "cdc_basic_demo";
static const char *TAG_WIFI = "wifi_config";
static const char *TAG_NVS = "nvs_config";
static const char *TAG_TCP = "TCP_Client";
static const char *TAG_FAN = "FAN_CTRL";
static esp_netif_t *sta_netif = NULL;         // 全局变量，只创建一次
static EventGroupHandle_t s_wifi_event_group; // 用于 WiFi 连接状态,用于同步“是否已获取 IP”
static EventGroupHandle_t s_net_event_group;  // 用于网络连接状态,方便LED反映

/* ringbuffer size */
#define IN_RINGBUF_SIZE (1024 * 1)
#define OUT_RINGBUF_SIZE (1024 * 1)

/* enable interface num */
#define EXAMPLE_BULK_ITF_NUM 1        // 设备端口数量，默认1个
#define WIFI_CONNECTED_BIT BIT0       // WiFi 已连接
#define TCP_CONNECTED_BIT BIT1        // TCP 已连接
#define LED_GPIO GPIO_NUM_10          // LED GPIO 引脚
#define STORAGE_NAMESPACE "wifi_info" // NVS 存储空间名称
#define KEY_SSID "ssid"
#define KEY_PASSWD "passwd"
#define SERVER_IP "192.168.50.1" // 替换为 PC 的 IP 地址
#define SERVER_PORT 1231         // 替换为 PC 的端口号
#define FRAME_HEADER0 0xAA
#define FRAME_HEADER1 0x66
#define MAX_BUF_SIZE 512

#define GPIO_FAN_PWR (GPIO_NUM_37) // 高侧开关/使能输入（5V 侧逻辑）
#define GPIO_FAN_PWM (GPIO_NUM_36) // 接到外部 NMOS 栅极或电平转换后的 PWM
// === PWM 参数（可按需改）===
#define PWM_FREQ_HZ (25000) // 先用 25 kHz；若异响或不转，试 1000/500 Hz
#define PWM_RES_BITS (10)   // 10位分辨率：0..1023
#define PWM_ACTIVE_LOW (1)  // 1=主动低（常见开漏规范）；0=主动高
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0

// ==== fans运行时状态 ====
static bool s_fan_power_on = false;
static uint8_t s_duty_percent = 0; // 0..100
// 状态枚举
typedef enum
{
    NET_STATE_DISCONNECTED = 0, // AP & TCP 都未连
    NET_STATE_AP_ONLY,          // 只连上 AP
    NET_STATE_AP_TCP,           // AP & TCP 都已连
    NET_STATE_MAX
} net_state_t;

// 三种状态下的LED on/off 时长（毫秒），初始值可随意设
static uint32_t s_led_on_ms[NET_STATE_MAX] = {200, 800, 1000};
static uint32_t s_led_off_ms[NET_STATE_MAX] = {200, 800, 0};

/* choose if use user endpoint descriptors */
#define EXAMPLE_CONFIG_USER_EP_DESC

static void wifi_apply_config_and_connect(const wifi_config_t *wifi_cfg);
static esp_err_t store_wifi_information(const char *ssid, const char *passwd);
static void process_ringbuf(uint8_t *ringbuf, size_t *p_len, int sockfd);
static esp_err_t apply_pwm_percent(uint8_t percent);
static void set_fan_power(bool on);

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
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, // 等待 Wi-Fi 连接
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(10000)); // 最多等 10 秒

    uint8_t ringbuf[MAX_BUF_SIZE];
    size_t ringbuf_len = 0;
    static const uint8_t tx_frame[] = {0xAA, 0x66, 0x02, 0x80, 0x01, 0x81};

    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG_TCP, "Unable to create socket: errno %d", errno);
        return;
    }

    ESP_LOGI(TAG_TCP, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)); // 连接到服务器
    if (err != 0)
    {
        ESP_LOGE(TAG_TCP, "Socket unable to connect: errno %d", errno);
        close(sock);
        return;
    }

    ESP_LOGI(TAG_TCP, "Successfully connected");
    xEventGroupSetBits(s_net_event_group, TCP_CONNECTED_BIT);

    while (1)
    {
        uint8_t tmp[128];
        int len = recv(sock, tmp, sizeof(tmp), 0);
        if (len <= 0)
        {
            ESP_LOGE(TAG_TCP, "recv failed len=%d, errno=%d", len, errno);
            break;
        }

        ESP_LOGI(TAG_TCP, "Raw TCP Data Received (len=%d):", len);
        ESP_LOG_BUFFER_HEXDUMP(TAG_TCP, tmp, len, ESP_LOG_INFO);

        // 追加到环形缓存
        if (ringbuf_len + len <= sizeof(ringbuf))
        {
            memcpy(ringbuf + ringbuf_len, tmp, len);
            ringbuf_len += len;
        }
        else
        {
            // 缓存溢出，丢弃所有数据
            ESP_LOGW(TAG_TCP, "Ring buffer overflow, discarding");
            ringbuf_len = 0;
        }

        // 解析并应答完整帧
        process_ringbuf(ringbuf, &ringbuf_len, sock);
    }

    close(sock);
    xEventGroupClearBits(s_net_event_group, TCP_CONNECTED_BIT);
    vTaskDelete(NULL);
}

static void process_ringbuf(uint8_t *ringbuf, size_t *p_len, int sockfd)
{
    size_t idx = 0;

    // 报文最小长度：2(header)+1(len)+0(payload)+1(checksum)=4
    while (*p_len >= 4)
    {
        // 1) 查找帧头
        if (ringbuf[idx] != FRAME_HEADER0 || ringbuf[idx + 1] != FRAME_HEADER1)
        {
            // 丢弃第一个字节，继续在下一个位置寻找
            memmove(ringbuf, ringbuf + 1, --(*p_len));
            continue;
        }

        // 2) 读取长度
        uint8_t payload_len = ringbuf[idx + 2];
        size_t frame_len = 2 + 1 + payload_len + 1; // hdr(2)+len(1)+payload(p_len)+chk(1)

        // 3) 如果不够一个完整帧，则等待后续数据
        if (*p_len < frame_len)
        {
            break;
        }

        // 4) 校验和验证
        uint8_t sum = 0;
        for (size_t i = 0; i < payload_len; i++)
        {
            sum += ringbuf[3 + i];
        }
        uint8_t recv_chk = ringbuf[3 + payload_len];
        if (sum == recv_chk)
        {
            // 5) 有效帧，构造并发送应答
            // 仅针对 payload_len == 2 的情况示例
            if (payload_len == 2)
            {
                uint8_t *payload = &ringbuf[3];
                uint8_t cmd = payload[0];  // 第一个字节，Command
                uint8_t data = payload[1]; // 第二个字节，Data

                // 只对 cmd==0x00 && data==0x00 的帧进行回复
                if (cmd == 0x00 && data == 0x00)
                {
                    // 固定填充回复 payload
                    uint8_t resp_payload[2] = {0x80, 0x01};
                    // 计算校验和
                    uint8_t resp_chk = resp_payload[0] + resp_payload[1];

                    // 打包完整应答帧
                    uint8_t resp_frame[2 + 1 + 2 + 1];
                    size_t off = 0;
                    resp_frame[off++] = FRAME_HEADER0;
                    resp_frame[off++] = FRAME_HEADER1;
                    resp_frame[off++] = 2; // length
                    resp_frame[off++] = resp_payload[0];
                    resp_frame[off++] = resp_payload[1];
                    resp_frame[off++] = resp_chk;

                    // 发送应答
                    send(sockfd, resp_frame, off, 0);
                    ESP_LOGI(TAG_TCP, "Raw TCP Data Send (len=%d):", off);
                    ESP_LOG_BUFFER_HEXDUMP(TAG_TCP, resp_frame, off, ESP_LOG_INFO);
                }
                else if (cmd == 0x01)
                {
                    if (data > 100)
                    {
                        data = 100;
                    }
                    // 初始状态：断电，占空比 0%
                    apply_pwm_percent(0);
                    set_fan_power(false);

                    set_fan_power(true);
                    apply_pwm_percent(data); // 设置风扇 PWM 占空比
                    ESP_LOGI(TAG_TCP, "Set fan power ON, duty=%u%%", data);
                }
                else
                {
                    // 不是 00 00，可以在这里处理其他命令
                    ESP_LOGI(TAG_TCP, "Received other CMD/DATA: %02X %02X", cmd, data);
                }
            }
            // 6) 丢弃已处理帧
            size_t remain = *p_len - frame_len;
            if (remain > 0)
            {
                memmove(ringbuf, ringbuf + frame_len, remain);
            }
            *p_len = remain;
        }
    }
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
            xEventGroupSetBits(s_net_event_group, WIFI_CONNECTED_BIT);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            wifi_event_sta_disconnected_t *d = event_data;
            ESP_LOGW(TAG_WIFI, "断开，reason=%d", d->reason);
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_net_event_group, WIFI_CONNECTED_BIT);
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
    s_net_event_group = xEventGroupCreate();
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

void set_led_blink_time(net_state_t state, uint32_t on_ms, uint32_t off_ms) // 动态调整某个状态下的 LED 闪烁参数
{
    if (state < NET_STATE_MAX)
    {
        s_led_on_ms[state] = on_ms;
        s_led_off_ms[state] = off_ms;
    }
}

static net_state_t get_net_state(void) // 计算当前网络状态
{
    EventBits_t bits = xEventGroupGetBits(s_net_event_group);
    bool wifi_ok = (bits & WIFI_CONNECTED_BIT);
    bool tcp_ok = (bits & TCP_CONNECTED_BIT);

    if (!wifi_ok)
    {
        return NET_STATE_DISCONNECTED;
    }
    else if (wifi_ok && !tcp_ok)
    {
        return NET_STATE_AP_ONLY;
    }
    else
    {
        return NET_STATE_AP_TCP;
    }
}

static void led_blink_task(void *pv)
{
    // 配置 LED GPIO
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
        net_state_t state = get_net_state();
        uint32_t on_ms = s_led_on_ms[state];
        uint32_t off_ms = s_led_off_ms[state];

        // 打开 LED
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));

        // 关闭 LED
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

static inline uint32_t percent_to_duty(uint8_t percent) // 将百分比转换为 PWM 占空比
{
    if (percent > 100)
        percent = 100;
    uint32_t max_duty = (1u << PWM_RES_BITS) - 1u; // 10位 -> 1023
    return (uint32_t)((percent * max_duty) / 100u);
}

static void set_fan_power(bool on) // 设置风扇电源状态
{
    s_fan_power_on = on;
    // 先把PWM置0再断电，或先上电再给PWM，避免毛刺
    if (!on)
    {
        apply_pwm_percent(0);
        gpio_set_level(GPIO_FAN_PWR, 0); // 断电
        ESP_LOGI(TAG_FAN, "Power OFF");
    }
    else
    {
        gpio_set_level(GPIO_FAN_PWR, 1); // 上电
        // 上电后恢复上次占空比（若为0则静止）
        apply_pwm_percent(s_duty_percent);
        ESP_LOGI(TAG_FAN, "Power ON, duty=%u%%", s_duty_percent);
    }
}

static esp_err_t apply_pwm_percent(uint8_t percent) // 应用 PWM 占空比
{
    s_duty_percent = percent;
    uint32_t duty = percent_to_duty(percent);
    esp_err_t err;

    err = ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    if (err != ESP_OK)
        return err;
    err = ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    return err;
}

static void fan_gpio_init(void)
{
    // 电源控制脚
    gpio_config_t pwr = {
        .pin_bit_mask = (1ULL << GPIO_FAN_PWR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&pwr);
    gpio_set_level(GPIO_FAN_PWR, 0); // 默认断电
}

static void ledc_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num = GPIO_FAN_PWM,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

void app_main(void)
{
#ifdef CONFIG_ESP32_S3_USB_OTG
    bsp_usb_mode_select_host();
    bsp_usb_host_power_mode(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
#endif

    fan_gpio_init();
    ledc_init();
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
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, tskIDLE_PRIORITY, NULL);

    /* Repeatedly sent AT through USB */
    static const uint8_t raw_frame[] = {0xA5, 0xB1, 0x02, 0x00, 0x00};

    static const size_t raw_len = sizeof(raw_frame);
    usbh_cdc_write_bytes(handle[0], raw_frame, raw_len, pdMS_TO_TICKS(100));
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
