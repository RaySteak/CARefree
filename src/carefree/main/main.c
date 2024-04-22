#define INCLUDE_vTaskSuspend 1
#define CAMERA_MODEL_AI_THINKER

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h" //esp_init funtions esp_err_t
#include "esp_wifi.h"   //esp_wifi_init functions and wifi operations
#include "esp_event.h"  //for wifi event
#include "mqtt_client.h"
#include "esp_camera.h"
#include "esp_http_client.h"
#include "img_converters.h"

// CAMERA DEFINES

// Left hand side pins
#define CAM_PIN_SCL 41
#define CAM_PIN_VS 36
#define CAM_PIN_PLK 35
#define CAM_PIN_D7 34
#define CAM_PIN_D5 33
#define CAM_PIN_D3 47
#define CAM_PIN_D1 48
#define CAM_PIN_RET 26
// #define CAM_PIN_RET -1
// Right hand side pins
#define CAM_PIN_SDA 42
#define CAM_PIN_HS 1
#define CAM_PIN_XLK 2
#define CAM_PIN_D6 3
#define CAM_PIN_D4 4
#define CAM_PIN_D2 5
#define CAM_PIN_D0 6
#define CAM_PIN_PWDN 7
// #define CAM_PIN_PWDN -1

#define XCLK_FREQ_HZ 20000000
// #define PIXFORMAT PIXFORMAT_RGB565
#define PIXFORMAT PIXFORMAT_GRAYSCALE
#define FRAMESIZE FRAMESIZE_QVGA
// #define FRAMESIZE FRAMESIZE_VGA
#define JPG_MAX_SIZE 15 * 1024
#define JPG_QUALITY 50

// WIFI DEFINES

#define WIFI_MAX_RETRIES 5

// HTTP DEFINES

#define HTTP_HOST "192.168.251.26" // Server domain/address and port here
#define HTTP_PORT 5000

// ML DEFINES
#define FEATS_LEN 768

// TAGS

const char *APP_TAG = "CARefree";
const char *WIFI_TAG = "WiFi";
const char *HTTP_TAG = "HTTP";
const char *CAM_TAG = "Camera";

// CAMERA GLOBALS

typedef struct _jpg_encode_cb_params
{
    char *img;
    size_t img_len;
} jpg_encode_cb_params;

// WIFI GLOBALS

const char *ssid = "OnePlus Nord"; // WiFi credentials here
const char *pass = "ayylmao123";
bool wifi_connected = false;

// ML GLOBALS
QueueHandle_t feats_queue = NULL;

// WIFI FUNCTIONS

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static int retry_num = 0;

    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(WIFI_TAG, "CONNECTING...");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(WIFI_TAG, "CONNECTED!");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(WIFI_TAG, "LOST CONNECTION");
        wifi_connected = false;
        if (retry_num >= WIFI_MAX_RETRIES)
            break;
        ESP_LOGI(WIFI_TAG, "RETRYING TO CONENCT...");
        esp_wifi_connect();
        retry_num++;
        break;
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(WIFI_TAG, "GOT IP");
        wifi_connected = true;
    }
}

void wifi_init()
{
    // Wi-Fi config phase
    esp_netif_init();
    esp_event_loop_create_default();     // event loop
    esp_netif_create_default_wifi_sta(); // WiFi station
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",
        }};
    strcpy((char *)wifi_configuration.sta.ssid, ssid);
    strcpy((char *)wifi_configuration.sta.password, pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // Wi-Fi Start Phase
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    // Wi-Fi Connect Phase
    esp_wifi_connect();
    ESP_LOGI(WIFI_TAG, "initialization finished");
}

// HTTP FUNCTIONS

esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    static size_t cur_copied;
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGE(HTTP_TAG, "ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(HTTP_TAG, "CONNECTED");
        cur_copied = 0;
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(HTTP_TAG, "HEADER SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(HTTP_TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(HTTP_TAG, "DATA RECEIVED, len=%d", evt->data_len);
        memcpy(evt->user_data + cur_copied, evt->data, evt->data_len);
        cur_copied += evt->data_len;
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(HTTP_TAG, "FINISHED");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(HTTP_TAG, "DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGI(HTTP_TAG, "REDIRECT");
        esp_http_client_set_redirection(evt->client);
        break;
    }
    return ESP_OK;
}

void http_post(char *host, int port, char *path, char *data, size_t data_len, const char *content_type, char *response)
{
    ESP_LOGW(HTTP_TAG, "POSTING DATA OF SIZE %u", data_len);
    esp_http_client_config_t config = {
        .host = host,
        .port = port,
        .path = path,
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .user_data = response,
        // TODO: look into keeping the connection alive to save the redundancy of the TCP handhsake,
        // although it might not be needed when duty cycle sleep is incorporated
        .keep_alive_enable = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_post_field(client, data, data_len);
    esp_http_client_perform(client);

    esp_http_client_cleanup(client);
}

// CAMERA FUNCTIONS

int camera_init(pixformat_t pixformat, framesize_t framesize)
{
    esp_rom_gpio_pad_select_gpio(CAM_PIN_PWDN);
    gpio_set_direction(CAM_PIN_PWDN, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_PWDN, 0);

    esp_rom_gpio_pad_select_gpio(CAM_PIN_RET);
    gpio_set_direction(CAM_PIN_RET, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_PWDN, 1);

    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .xclk_freq_hz = XCLK_FREQ_HZ,
        .pixel_format = pixformat,
        .frame_size = framesize,
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_LATEST, // See difference from CAMERA_GRAB_WHEN_EMPTY
        .fb_count = 1,
        .pin_sccb_scl = CAM_PIN_SCL,
        .pin_vsync = CAM_PIN_VS,
        .pin_pclk = CAM_PIN_PLK,
        .pin_d7 = CAM_PIN_D7,
        .pin_d5 = CAM_PIN_D5,
        .pin_d3 = CAM_PIN_D3,
        .pin_d1 = CAM_PIN_D1,
        .pin_reset = CAM_PIN_RET,
        .pin_sccb_sda = CAM_PIN_SDA,
        .pin_href = CAM_PIN_HS,
        .pin_xclk = CAM_PIN_XLK,
        .pin_d6 = CAM_PIN_D6,
        .pin_d4 = CAM_PIN_D4,
        .pin_d2 = CAM_PIN_D2,
        .pin_d0 = CAM_PIN_D0,
        .pin_pwdn = CAM_PIN_PWDN};

    // initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
        return -1;
    ESP_LOGI(CAM_TAG, "initialization finished");
    return 0;
}

size_t jpg_write_buf_cb(void *arg, size_t index, const void *data, size_t len)
{
    // TODO: Implement realloc of jpg buffer here instead of static buffer with hardcoded size;
    // whenever there is a need for more space, realloc. Warning, this might cause fragmentation as well.
    jpg_encode_cb_params *params = (jpg_encode_cb_params *)arg;
    params->img_len += len;
    memcpy(params->img + index, data, len);
    return len;
}

int camera_capture_image(void)
{
    static char jpg_img[JPG_MAX_SIZE];
    static char http_response[FEATS_LEN * sizeof(float)];
    // Capture an image
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
        return -1;

    // Process image
    ESP_LOGI(CAM_TAG, "Image captured height=%d, width=%d, length=%d", fb->height, fb->width, fb->len);
    if (wifi_connected)
    {
        const char *content_type = "application/jpeg";
        jpg_encode_cb_params params = {.img = jpg_img, .img_len = 0};
        // Convert image to JPEG
        frame2jpg_cb(fb, JPG_QUALITY, jpg_write_buf_cb, &params);
        // Send image to server
        http_post(HTTP_HOST, HTTP_PORT, "/embed", jpg_img, params.img_len, content_type, http_response);
        xQueueSend(feats_queue, http_response, portMAX_DELAY);
    }
    // Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return 0;
}

// ML FUNCTIONS
void train_task(void *arg)
{
    static float feats[FEATS_LEN];

    while (1)
    {
        xQueueReceive(feats_queue, feats, portMAX_DELAY);
        ESP_LOGW(HTTP_TAG, "5th value is %f", feats[5]);
    }
}

void app_main(void)
{
    int ret;

    // Get free heap memory
    ESP_LOGI(APP_TAG, "Free heap memory: %lu bytes", esp_get_free_heap_size());

    // WiFi
    nvs_flash_init();
    wifi_init();

    // Camera
    ret = camera_init(PIXFORMAT, FRAMESIZE);

    if (ret != 0)
    {
        ESP_LOGE("init_camera", "Camera Init Failed");
        return;
    }

    // ML
    feats_queue = xQueueCreate(1, FEATS_LEN * sizeof(float));

    // TODO: look into how much is used to set stack size (especially for camera task)
    xTaskCreate(train_task, "train_task", configMINIMAL_STACK_SIZE + 1000, NULL, 1, NULL);

    while (1)
    {
        ret = camera_capture_image();
        if (ret != 0)
        {
            ESP_LOGE("capture_image", "Capture Image Failed");
            return;
        }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}