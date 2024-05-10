#define INCLUDE_vTaskSuspend 1
#define CAMERA_MODEL_AI_THINKER

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "esp_camera.h"
#include "esp_http_client.h"
#include "img_converters.h"
#include "human_face_detect_msr01.hpp"
#include "math.h"
#include "dl_tool.hpp"
#include "esp_random.h"
#include "esp_crt_bundle.h"

// DEBUG DEFINES
#define DEBUG_SHOW_IMAGE 0
#define DEBUG_SHOW_FACE 1
#define DEBUG_PRINT_LOSS 1

// CONFIG DEFINES
#define USE_FACE_DETECTION 0
#define TWO_LAYER_NN 1
#define HTTPS 0
// Beware, async works with HTTPS only. Also, right now it doesn't work at all because it returns EAGAIN only (TODO: fix that)
#define ASYNC_HTTP 0

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

// Be careful when changing this, as it may make certain resolutions not work
// Use 16MHz for QQVGA, 20MHz for everything else.
// Setting it to 16MHz uses the weird EDMA mode that allocates double the space
// plus 16 bytes instead of the needed 19.2KB for one buffer. This seems to have
// to do with the fact that it can't do grayscale at this resolution (with
// RGB 565 it allocates the same amount). Still weird that if not in EDMA
// (freq is set to anything but 16MHz), it allocates exactly the correct amount.
// Still, the ~20KB waste is still better than the ~60KB waste and inconvenience
// of QVGA
// Datasheet for the camera says this should be between 10MHz and 48MHz, typical being 24MHz
#define XCLK_FREQ_HZ 16000000
#if USE_FACE_DETECTION
#define PIXFORMAT PIXFORMAT_RGB565
#define FRAMESIZE FRAMESIZE_QVGA
#else
#define PIXFORMAT PIXFORMAT_GRAYSCALE
#define FRAMESIZE FRAMESIZE_QQVGA
#endif
#define JPG_MAX_SIZE 5 * 1024
#define JPG_QUALITY 12

// WIFI DEFINES

#define WIFI_MAX_RETRIES 5

// HTTP DEFINES

#define HTTP_HOST "192.168.24.26" // Server domain/address and port here
#define HTTP_PORT 5000

// MQTT DEFINES
// Beware, with QOS 1 and 2, the messages are stored in the outbox which uses up more RAM
#define MQTT_QOS 0

// ML DEFINES

// feature extraction
#if USE_FACE_DETECTION
#define POOL_KERNEL_SIZE 4
#define FACE_SIZE 64
#define HOG_PATCH_SIZE 16
#define HOG_NUM_BINS 5
#define FEATS_LEN 80 // dependent on image face size and HOG parameters
#else
#define POOL_KERNEL_SIZE 2
#define FACE_SIZE 64
#define HOG_PATCH_SIZE 8
#define HOG_NUM_BINS 9
#define FEATS_LEN 576 // dependent on image face size and HOG parameters
#endif
// training
typedef enum _activation
{
    RELU,
    SIGMOID,
    TANH
} activation;

#if TWO_LAYER_NN
#define HIDDEN_LAYER_SIZE 50
#endif
#define NUM_CLASSES 7
#define LEARNING_RATE 0.01f
#define ACTIVATION RELU

#define EPOCHS_PER_ROUND 10

// TAGS

const char *const APP_TAG = "CARefree";
const char *const WIFI_TAG = "WiFi";
const char *const HTTP_TAG = "HTTP";
const char *const MQTT_TAG = "MQTT";
const char *const CAM_TAG = "Camera";
const char *const ML_TAG = "Train";

// CAMERA GLOBALS

typedef struct _jpg_encode_cb_params
{
    char *img;
    size_t img_len;
} jpg_encode_cb_params;

// WIFI GLOBALS

const char *const ssid = "OnePlus Nord"; // WiFi credentials here
const char *const pass = "ayylmao123";
bool wifi_connected = false;

// HTTP GLOBALS

typedef struct _http_response
{
    char *data;
    bool completed_successfully;
    SemaphoreHandle_t done_sem;
} http_response;

const bool send_jpeg_encoded = false;

// MQTT GLOBALS

esp_mqtt_client_handle_t client;
bool mqtt_connected = 0;

extern const char *mqtt_root_ca_cert;
extern const char *mqtt_cert;
extern const char *mqtt_privkey;
extern const char *mqtt_aws_uri;
extern const char *mqtt_client_id;

const char *const mqtt_post_topic = "post_weights";
const char *const mqtt_get_topic = "get_weights";

// ML GLOBALS
QueueHandle_t feats_queue = NULL;
#if TWO_LAYER_NN
static float model[HIDDEN_LAYER_SIZE * (FEATS_LEN + NUM_CLASSES + 1) + NUM_CLASSES] = {0};
#else
static float model[NUM_CLASSES * (FEATS_LEN + 1)] = {0};
#endif
SemaphoreHandle_t model_lock = NULL;
SemaphoreHandle_t weights_received = NULL;

// WIFI FUNCTIONS

void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
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
    wifi_config_t wifi_configuration;
    memset(&wifi_configuration, 0, sizeof(wifi_configuration));
    strcpy((char *)wifi_configuration.sta.ssid, ssid);
    strcpy((char *)wifi_configuration.sta.password, pass);
    esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifi_configuration);
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
    http_response *response = (http_response *)evt->user_data;
    switch (evt->event_id)
    {
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
        if (!response)
            break;
        memcpy(response->data + cur_copied, evt->data, evt->data_len);
        cur_copied += evt->data_len;
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(HTTP_TAG, "FINISHED");
        if (!response)
            break;
        response->completed_successfully = true;
        if (!response->done_sem)
            break;
        xSemaphoreGive(response->done_sem);
        break;
    case HTTP_EVENT_ERROR:
        ESP_LOGE(HTTP_TAG, "ERROR");
        if (!response || !response->done_sem)
            break;
        xSemaphoreGive(response->done_sem);
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

esp_http_client_handle_t http_post(const char *host, int port, const char *path, char *data, size_t data_len, const char *content_type, http_response *response, bool async)
{
    ESP_LOGW(HTTP_TAG, "POSTING DATA OF SIZE %u", data_len);
    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));
    config.host = host;
    config.port = port;
    config.path = path;
    config.method = HTTP_METHOD_POST;
    config.event_handler = http_event_handler;
    config.user_data = response;
    config.is_async = async;
#if HTTPS
    config.transport_type = HTTP_TRANSPORT_OVER_SSL;
    config.auth_type = HTTP_AUTH_TYPE_BASIC;
    config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
    // TODO: look into keeping the connection alive to save the redundancy of the TCP handhsake,
    // although it might not be needed when duty cycle sleep is incorporated
    config.keep_alive_enable = false;

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_post_field(client, data, data_len);
    int ret = esp_http_client_perform(client);
    if (ret != ESP_OK)
        ESP_LOGE(HTTP_TAG, "ERROR IN HTTP PERFORM");
    if (async)
        return client;
    esp_http_client_cleanup(client);
    return NULL;
}

// MQTT

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "CONNECTED");
        esp_mqtt_client_subscribe(client, mqtt_get_topic, MQTT_QOS);
        mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGE(MQTT_TAG, "DISCONNECTED");
        mqtt_connected = false;
        // To make sure train task doesn't get stuck waiting for weights
        xSemaphoreGive(weights_received);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "DATA RECEIVED");
        xSemaphoreTake(model_lock, portMAX_DELAY);
        memcpy(model + event->current_data_offset, event->data, event->data_len);
        if (event->current_data_offset + event->data_len == event->total_data_len)
        {
            ESP_LOGI(MQTT_TAG, "MODEL FULLY RECEIVED");
            xSemaphoreGive(weights_received);
        }
        xSemaphoreGive(model_lock);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(MQTT_TAG, "ERROR");
        break;
    default:
        break;
    }
}

static int mqtt_initialize(void)
{
    esp_mqtt_client_config_t mqtt_cfg;
    memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
    mqtt_cfg.broker.address.uri = mqtt_aws_uri;
    mqtt_cfg.broker.verification.certificate = mqtt_root_ca_cert;
    mqtt_cfg.credentials.authentication.certificate = mqtt_cert;
    mqtt_cfg.credentials.authentication.key = mqtt_privkey;
    mqtt_cfg.credentials.client_id = mqtt_client_id;
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    int err = esp_mqtt_client_start(client);
    if (err != ESP_OK)
        ESP_LOGE(MQTT_TAG, "MQTT START ERROR");
    return err;
}

// CAMERA FUNCTIONS

int camera_init(pixformat_t pixformat, framesize_t framesize)
{
    esp_rom_gpio_pad_select_gpio((uint32_t)CAM_PIN_PWDN);
    gpio_set_direction((gpio_num_t)CAM_PIN_PWDN, (gpio_mode_t)GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CAM_PIN_PWDN, (uint32_t)0);

    if (CAM_PIN_RET != -1)
    {
        esp_rom_gpio_pad_select_gpio((uint32_t)CAM_PIN_RET);
        gpio_set_direction((gpio_num_t)CAM_PIN_RET, (gpio_mode_t)GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)CAM_PIN_RET, (uint32_t)1);
    }

    camera_config_t config;
    memset(&config, 0, sizeof(config));
    config.ledc_channel = (ledc_channel_t)LEDC_CHANNEL_0;
    config.ledc_timer = (ledc_timer_t)LEDC_TIMER_0;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = (pixformat_t)pixformat;
    config.frame_size = (framesize_t)framesize;
    config.fb_location = (camera_fb_location_t)CAMERA_FB_IN_DRAM;
    config.grab_mode = (camera_grab_mode_t)CAMERA_GRAB_LATEST;
    config.fb_count = (size_t)1;
    config.pin_sccb_scl = CAM_PIN_SCL;
    config.pin_vsync = CAM_PIN_VS;
    config.pin_pclk = CAM_PIN_PLK;
    config.pin_d7 = CAM_PIN_D7;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_reset = CAM_PIN_RET;
    config.pin_sccb_sda = CAM_PIN_SDA;
    config.pin_href = CAM_PIN_HS;
    config.pin_xclk = CAM_PIN_XLK;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d0 = CAM_PIN_D0;
    config.pin_pwdn = CAM_PIN_PWDN;

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

// TODO: why does this not work with average pooling but works when picking the first value in the kernel??????
void *avg_pool_rgb565(uint16_t *in, int height, int width, int kernel_size, bool convert_to_grayscale)
{
    int out_height = height / kernel_size;
    int out_width = width / kernel_size;
    void *out = NULL;
    size_t to_alloc = convert_to_grayscale ? out_height * out_width : out_height * out_width * sizeof(uint16_t);

    out = malloc(to_alloc);
    if (!out)
    {
        ESP_LOGE(CAM_TAG, "Failed to allocate %lu bytes of memory for pooled image", to_alloc);
        return NULL;
    }
    for (int i = 0; i < height; i += kernel_size)
    {
        for (int j = 0; j < width; j += kernel_size)
        {
            float sum[3] = {0.f, 0.f, 0.f};
            for (int bi = 0; bi < kernel_size; bi++)
            {
                for (int bj = 0; bj < kernel_size; bj++)
                {
                    uint16_t pixel = in[(i + bi) * width + j + bj];
                    sum[0] += (pixel >> 11);
                    sum[1] += ((pixel >> 5) & 0b111111);
                    sum[2] += (pixel & 0b11111);
                    // break;
                }
                // break;
            }
            uint16_t r = (uint16_t)roundf(sum[0] / (kernel_size * kernel_size));
            uint16_t g = (uint16_t)roundf(sum[1] / (kernel_size * kernel_size));
            uint16_t b = (uint16_t)roundf(sum[2] / (kernel_size * kernel_size));
            // uint16_t r = (uint16_t)sum[0];
            // uint16_t g = (uint16_t)sum[1];
            // uint16_t b = (uint16_t)sum[2];

            if (convert_to_grayscale)
            {
                uint8_t colorimetric_grasycale = (uint8_t)roundf(255.f * ((0.2126f * r / 31.f) + (0.7152f * g / 63.f) + (0.0722f * b / 31.f)));
                ((uint8_t *)out)[(i / kernel_size) * out_width + (j / kernel_size)] = colorimetric_grasycale;
            }
            else
            {
                ((uint16_t *)out)[(i / kernel_size) * out_width + (j / kernel_size)] = (r << 11) | (g << 5) | b;
            }
        }
    }
    return out;
}

void *avg_pool_grayscale(uint8_t *in, int height, int width, int kernel_size)
{
    int out_height = height / kernel_size;
    int out_width = width / kernel_size;
    uint8_t *out = NULL;
    size_t to_alloc = out_width * out_height;

    out = (uint8_t *)malloc(to_alloc);
    if (!out)
    {
        ESP_LOGE(CAM_TAG, "Failed to allocate %lu bytes of memory for pooled image", to_alloc);
        return NULL;
    }
    for (int i = 0; i < height; i += kernel_size)
    {
        for (int j = 0; j < width; j += kernel_size)
        {
            float sum = 0.f;
            for (int bi = 0; bi < kernel_size; bi++)
            {
                for (int bj = 0; bj < kernel_size; bj++)
                    sum += in[(i + bi) * width + j + bj];
            }
            out[(i / kernel_size) * out_width + (j / kernel_size)] = sum / (kernel_size * kernel_size);
        }
    }
    return out;
}

uint8_t *resize_bilinear_grayscale(uint8_t *in, int height, int width, int out_height, int out_width, dl::detect::result_t *face_pos)
{
    uint8_t *out = (uint8_t *)malloc(out_height * out_width);
    if (!out)
    {
        ESP_LOGE(CAM_TAG, "Failed to allocate %lu bytes of memory for resized image", out_height * out_width);
        return NULL;
    }

    for (int i = 0; i < out_height; i++)
    {
        float in_i;
        if (face_pos)
            in_i = i / ((float)out_height) * (face_pos->box[3] - face_pos->box[1]) + face_pos->box[1];
        else
            in_i = i / ((float)out_height) * height;
        int in_i_1 = (int)floorf(in_i);
        int in_i_2 = (int)ceilf(in_i);
        in_i_2 = in_i_1 == in_i_2 ? in_i_2 + 1 : in_i_2;
        for (int j = 0; j < out_width; j++)
        {
            float in_j;
            if (face_pos)
                in_j = j / ((float)out_width) * (face_pos->box[2] - face_pos->box[0]) + face_pos->box[0];
            else
                in_j = j / ((float)out_width) * width;
            int in_j_1 = (int)floorf(in_j);
            int in_j_2 = (int)ceilf(in_j);
            in_j_2 = in_j_1 == in_j_2 ? in_j_2 + 1 : in_j_2;

            float upper_mid = in[in_i_1 * width + in_j_1] * (in_j_2 - in_j) + in[in_i_1 * width + in_j_2] * (in_j - in_j_1);
            float lower_mid = in[in_i_2 * width + in_j_1] * (in_j_2 - in_j) + in[in_i_2 * width + in_j_2] * (in_j - in_j_1);
            float mid = upper_mid * (in_i_2 - in_i) + lower_mid * (in_i - in_i_1);
            out[i * out_width + j] = (uint8_t)mid;
        }
    }
    return out;
}

void *extract_hog_features(uint8_t *in, int height, int width)
{
    const int patch_size = HOG_PATCH_SIZE;
    const int num_bins = HOG_NUM_BINS;
    int delta = 180 / num_bins;
    int num_elements = (height / patch_size) * (width / patch_size) * num_bins;

    void *feature_vec = calloc(1 + num_elements * sizeof(float), 1); // 1 byte space for the class label
    if (!feature_vec)
    {
        ESP_LOGE(CAM_TAG, "Failed to allocate %lu bytes of memory for HOG", 1 + num_elements * sizeof(float));
        return NULL;
    }
    float *hog = (float *)((uint8_t *)feature_vec + 1);

    for (int i = 0; i < height; i += patch_size)
    {
        for (int j = 0; j < width; j += patch_size)
        {
            int bins_start = ((i / patch_size) * (width / patch_size) + (j / patch_size)) * num_bins;
            for (int bi = 0; bi < patch_size; bi++)
            {
                for (int bj = 0; bj < patch_size; bj++)
                {
                    int pos = (i + bi) * width + j + bj;
                    int pos_diff1, pos_diff2;
                    int dx, dy;

                    pos_diff1 = j == 0 ? in[pos] : in[pos - 1];
                    pos_diff2 = j == width - 1 ? in[pos] : in[pos + 1];
                    dx = in[pos_diff2] - in[pos_diff1];

                    pos_diff1 = i == 0 ? in[pos] : in[pos - width];
                    pos_diff2 = i == height - 1 ? in[pos] : in[pos + width];
                    dy = in[pos_diff1] - in[pos_diff2];

                    float angle = atan2f(dy, dx);               // returns result in [-PI, PI]
                    angle = angle < 0.f ? angle + M_PI : angle; // constrain angle to [0, PI]
                    int grad = (int)roundf(angle / M_PI * 180.f);
                    int mag = (int)roundf(sqrtf(dx * dx + dy * dy));
                    int bin_angle = (grad / delta) * delta;
                    bin_angle = bin_angle == 180 ? bin_angle - delta : bin_angle;
                    int contrib1 = (int)roundf(((bin_angle + delta - grad) / (float)delta) * mag);
                    int bin1_pos = bins_start + bin_angle / delta;
                    hog[bin1_pos] += contrib1;
                    if (bin_angle / delta + 1 < num_bins)
                        hog[bin1_pos + 1] += mag - contrib1;
                }
            }
            // Unlike standard HOG, we don't perform conflation of neighboring blocks here,
            // as it increases feature vector size by a factor of at least 2
            // Instead, normalization is done on each block
            float sqr_norm = 0.f;
            for (int k = 0; k < num_bins; k++)
                sqr_norm += hog[bins_start + k] * hog[bins_start + k];
            for (int k = 0; k < num_bins; k++)
                hog[bins_start + k] /= sqrtf(sqr_norm + 1e-6); // Done to avoid 0 division
        }
    }
    return feature_vec;
}

int camera_capture_image(HumanFaceDetectMSR01 *s1)
{
    // static char http_response[FEATS_LEN * sizeof(float)];
    // Capture an image
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
        return -1;

    // Process image
    // ESP_LOGI(CAM_TAG, "Image captured height=%d, width=%d, length=%d", fb->height, fb->width, fb->len);
    if (wifi_connected)
    {
#if USE_FACE_DETECTION
        // It doesn't work to use the pooled image for HumanFaceDetectMSR01 inference, because of this error:
        // assert failed: dl::Tensor<T>& dl::Tensor<T>::set_shape(std::vector<int>) [with T = short int] dl_variable.cpp:20 (shape[i] >= 0)
        // TODO: maybe look into it but it seems to be a bug with the library (maybe try different pooling shapes or something
        // but even with 60x60 and QQVGA it seems to complain, seems to not like small input sizes, as aspect ratio doesn't seem to matter)
        // ESP_LOGW(CAM_TAG, "FREE HEAP MEMORY BEFORE INFER: %lu", esp_get_free_heap_size());
        std::list<dl::detect::result_t> candidates = s1->infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
#endif

#if DEBUG_SHOW_IMAGE
#if USE_FACE_DETECTION
        if (candidates.size())
        {
            int32_t rect[4] = {candidates.front().box[0], candidates.front().box[1], candidates.front().box[2], candidates.front().box[3]};
            http_post(HTTP_HOST, HTTP_PORT, "/debug/set_rect", (char *)rect, 4 * sizeof(uint32_t), "application/octet-stream", NULL, false);
        }
#endif
        if (send_jpeg_encoded) // This makes the program run out of memory on next img, beware
        {
            char *jpg_img = (char *)malloc(JPG_MAX_SIZE);
            jpg_encode_cb_params params = {.img = jpg_img, .img_len = 0};
            // Convert image to JPEG
            frame2jpg_cb(fb, JPG_QUALITY, jpg_write_buf_cb, &params);
            // Send image to server
            http_post(HTTP_HOST, HTTP_PORT, "/debug/set_image", jpg_img, params.img_len, "application/jpeg", NULL, false);
            free(jpg_img);
        }
        else
        {
            // Send image to server
            http_post(HTTP_HOST, HTTP_PORT, "/debug/set_image", (char *)fb->buf, fb->len, "application/octet-stream", NULL, false);
        }
#endif

#if ASYNC_HTTP
        SemaphoreHandle_t done_sem = xSemaphoreCreateBinary();
#else
        SemaphoreHandle_t done_sem = NULL;
#endif

        uint8_t class_label;
        http_response response = {.data = (char *)&class_label, .completed_successfully = false, .done_sem = done_sem};

#if USE_FACE_DETECTION
        if (!candidates.size())
        {
            // TODO: turn this into goto (disable unused label error for that)
            esp_camera_fb_return(fb);
            return 0;
        }

        dl::detect::result_t &face = candidates.front();
        uint8_t *pooled = (uint8_t *)avg_pool_rgb565((uint16_t *)fb->buf, fb->height, fb->width, POOL_KERNEL_SIZE, true);
        int height = fb->height / POOL_KERNEL_SIZE;
        int width = fb->width / POOL_KERNEL_SIZE;
        // http_post(HTTP_HOST, HTTP_PORT, "/debug/set_image", (char *)pooled, height * width, "application/octet-stream", NULL, false);
        face.box[0] = face.box[0] < 0 ? 0 : face.box[0] / POOL_KERNEL_SIZE;
        face.box[1] = face.box[1] < 0 ? 0 : face.box[1] / POOL_KERNEL_SIZE;
        face.box[2] = face.box[2] > fb->width ? width : (int)(ceilf(face.box[2] / (float)POOL_KERNEL_SIZE));
        face.box[3] = face.box[3] > fb->height ? height : (int)(ceilf(face.box[3] / (float)POOL_KERNEL_SIZE));

        // Send image to server to get target ground truth class
        esp_http_client_handle_t client = http_post(HTTP_HOST, HTTP_PORT, "/predict", (char *)pooled, height * width, "application/octet-stream", &response, ASYNC_HTTP);

        uint8_t *resized = resize_bilinear_grayscale(pooled, height, width, FACE_SIZE, FACE_SIZE, &face);
        free(pooled);
#else
        // Downsize image before sending
        uint8_t *pooled = (uint8_t *)avg_pool_grayscale((uint8_t *)fb->buf, fb->height, fb->width, POOL_KERNEL_SIZE);
        int height = fb->height / POOL_KERNEL_SIZE;
        int width = fb->width / POOL_KERNEL_SIZE;
        // Send image to server to get target ground truth class
        esp_http_client_handle_t client = http_post(HTTP_HOST, HTTP_PORT, "/predict", (char *)pooled, height * width, "application/octet-stream", &response, ASYNC_HTTP);

        uint8_t *resized = resize_bilinear_grayscale(pooled, height, width, FACE_SIZE, FACE_SIZE, NULL);
        free(pooled);
#endif
#if DEBUG_SHOW_FACE
        http_post(HTTP_HOST, HTTP_PORT, "/debug/set_image", (char *)resized, FACE_SIZE * FACE_SIZE, "application/octet-stream", NULL, false);
#endif
        void *feature_vec = extract_hog_features(resized, FACE_SIZE, FACE_SIZE);
        free(resized);
        // Wait for response to complete if async
#if ASYNC_HTTP
        xSemaphoreTake(done_sem, portMAX_DELAY);
        vSemaphoreDelete(done_sem);
#endif
        if (client)
            esp_http_client_cleanup(client);
        if (response.completed_successfully)
        {
            *((uint8_t *)feature_vec) = class_label;
            xQueueSend(feats_queue, feature_vec, portMAX_DELAY);
        }

        free(feature_vec);
    }
    // Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return 0;
}

// ML FUNCTIONS

float uniform_rand(float min, float max)
{
    return min + (max - min) * (esp_random() / (float)UINT32_MAX);
}

void linear_forward(float *feats, float *w, float *b, int in_size, int out_size, float *out)
{
    for (int j = 0; j < out_size; j++)
        out[j] = b[j];
    for (int i = 0; i < in_size; i++)
    {
        for (int j = 0; j < out_size; j++)
            out[j] += feats[j] * w[i * out_size + j];
    }
}

void activation_forward(float *in, int size, activation act)
{
    for (int i = 0; i < size; i++)
    {
        switch (act)
        {
        case RELU:
            in[i] = in[i] < 0.f ? 0.f : in[i];
            break;
        case SIGMOID:
            in[i] = 1.f / (1.f + expf(-in[i]));
            break;
        case TANH:
            in[i] = tanhf(in[i]);
            break;
        }
    }
}

void activation_backward(float *grads, float *output, int size, activation act)
{
    for (int i = 0; i < size; i++)
    {
        switch (act)
        {
        case RELU:
            grads[i] = output[i] != 0.f ? grads[i] : 0.f;
            break;
        case SIGMOID:
            grads[i] = grads[i] * (1 - output[i]) * output[i];
            break;
        case TANH:
            grads[i] = grads[i] * (1 - output[i] * output[i]);
            break;
        }
    }
}

float cross_entropy_with_logits(float *in, int target)
{
    float softmax_denom_sum = 0.f;
    for (int j = 0; j < NUM_CLASSES; j++)
        softmax_denom_sum += expf(in[j]);
    return logf(softmax_denom_sum) - in[target];
}

float *backprop_cross_entropy(float *out, int target)
{
    float *grads = (float *)malloc(FEATS_LEN * sizeof(float));
    float softmax = 0.f;
    // Numerically stable softmax
    float out_max = out[0];
    for (int j = 1; j < NUM_CLASSES; j++)
        out_max = out[j] > out_max ? out[j] : out_max;
    for (int j = 0; j < NUM_CLASSES; j++)
        softmax += expf(out[j] - out_max);
    softmax = expf(out[target] - out_max) / softmax;

    for (int j = 0; j < NUM_CLASSES; j++)
        grads[j] = softmax - (j == target ? 1.f : 0.f);

    return grads;
}

// TODO: sometimes it seems to diverge to inf and then NaN, maybe it's just the learning rate or the activation function
void train_task(void *arg)
{
    static char feature_vec[1 + FEATS_LEN * sizeof(float)];
    float *feats = (float *)(feature_vec + 1);
#if TWO_LAYER_NN
    float *const w1 = model;
    float *const b1 = w1 + HIDDEN_LAYER_SIZE * FEATS_LEN;
    float *const w2 = b1 + HIDDEN_LAYER_SIZE;
    float *const b2 = w2 + HIDDEN_LAYER_SIZE * NUM_CLASSES;
    float *const weights[] = {w1, w2};
    float *const biases[] = {b1, b2};
    const int layer_sizes[] = {FEATS_LEN, HIDDEN_LAYER_SIZE, NUM_CLASSES};
    const int num_layers = 2;
#else
    float *const w = model;
    float *const b = w + FEATS_LEN * NUM_CLASSES;
    float *const weights[] = {w};
    float *const biases[] = {b};
    const int layer_sizes[] = {FEATS_LEN, NUM_CLASSES};
    const int num_layers = 1;
#endif
    static float out[NUM_CLASSES];

    for (int l = 0; l < num_layers; l++)
    {
        for (int i = 0; i < layer_sizes[l] * layer_sizes[l + 1]; i++)
        {
            // Uniform Glorot initialization
            float x = sqrtf(6.f / (layer_sizes[l] + layer_sizes[l + 1]));
            weights[l][i] = uniform_rand(-x, x);
        }
    }

    for (int step = 0;; step++)
    {
        xQueueReceive(feats_queue, feature_vec, portMAX_DELAY);
        uint8_t target = *((uint8_t *)feature_vec);
        ESP_LOGW(ML_TAG, "Received target is %d", (int)target);

        // Lock model before updating
        xSemaphoreTake(model_lock, portMAX_DELAY);

        // Forward pass
#if TWO_LAYER_NN
        float *hidden = (float *)malloc(HIDDEN_LAYER_SIZE * sizeof(float));
        linear_forward(feats, w1, b1, FEATS_LEN, HIDDEN_LAYER_SIZE, hidden);
        activation_forward(hidden, HIDDEN_LAYER_SIZE, ACTIVATION);
        linear_forward(hidden, w2, b2, HIDDEN_LAYER_SIZE, NUM_CLASSES, out);
#else
        linear_forward(feats, w, b, FEATS_LEN, NUM_CLASSES, out);
#endif
#if DEBUG_PRINT_LOSS
        float loss = cross_entropy_with_logits(out, target);
        ESP_LOGI(ML_TAG, "Step %d, Loss=%f", step, loss);
#endif
        // Backprop and gradient descent
        // Computing the output loss or logits is optional, the formula just needs the logits from the last layer
        float *grads_out = backprop_cross_entropy(out, target);
        // The gradients for the weights are one vector outer product away. We can apply backpropagation directly without
        // storing them, thus saving memory. Even with a second layer, we can first apply gradient descent on the current layer
        // while also computing the gradients for the input of the layer and then continue with backpropagation on the next layer.
#if TWO_LAYER_NN
        // updating gradients of second layer and computing gradients for hidden layer (also known as first layer)
        float *grads_first = (float *)malloc(HIDDEN_LAYER_SIZE * sizeof(float));
        for (int i = 0; i < HIDDEN_LAYER_SIZE; i++)
        {
            grads_first[i] = 0.f;
            for (int j = 0; j < NUM_CLASSES; j++)
            {
                // make sure to compute hidden layer gradients before gradient update
                grads_first[i] += w2[i * NUM_CLASSES + j] * grads_out[j];
                w2[i * NUM_CLASSES + j] -= LEARNING_RATE * hidden[i] * grads_out[j];
            }
        }
        for (int j = 0; j < NUM_CLASSES; j++)
            b2[j] -= LEARNING_RATE * grads_out[j];
        free(grads_out);
        // computing gradients through hidden layer activation
        activation_backward(grads_first, hidden, HIDDEN_LAYER_SIZE, ACTIVATION);
        free(hidden);
#else
        // The gradients of first layer and of the output layer are one and the same
        float *grads_first = grads_out;
#endif
        // Updating gradients for first layer
        for (int i = 0; i < layer_sizes[0]; i++)
        {
            for (int j = 0; j < layer_sizes[1]; j++)
                weights[0][i * layer_sizes[1] + j] -= LEARNING_RATE * feats[i] * grads_first[j];
        }
        for (int j = 0; j < NUM_CLASSES; j++)
            biases[0][j] -= LEARNING_RATE * grads_first[j];
        free(grads_first);
        // Unlock model after updating is done
        xSemaphoreTake(model_lock, portMAX_DELAY);

        if ((step + 1) % EPOCHS_PER_ROUND)
            continue;
        // Send weights through MQTT
        int msg_id = esp_mqtt_client_publish(client, mqtt_post_topic, (const char *)model, sizeof(model), MQTT_QOS, 0);
        ESP_LOGI(MQTT_TAG, "Published model, msg_id=%d", msg_id);
        // Wait for aggregate weights to come back
        if (msg_id != -1 && mqtt_connected)
        {
            xSemaphoreTake(weights_received, portMAX_DELAY);
            ESP_LOGI(ML_TAG, "Weights received");
        }
    }
}

extern "C" void app_main(void)
{
    int ret;

    weights_received = xSemaphoreCreateBinary();
    model_lock = xSemaphoreCreateMutex();

    // Get free heap memory
    ESP_LOGI(APP_TAG, "Free heap memory: %lu bytes", esp_get_free_heap_size());

    // WiFi
    nvs_flash_init();
    wifi_init();

    // MQTT
    mqtt_initialize();

    // Camera
    ret = camera_init(PIXFORMAT, FRAMESIZE);
    if (ret != 0)
    {
        ESP_LOGE("init_camera", "Camera Init Failed");
        return;
    }

    // ML
    HumanFaceDetectMSR01 s1(0.1F, 0.5F, 1, 0.2F); // TODO: make these defines and find the sweet spot

    feats_queue = xQueueCreate(1, 1 + FEATS_LEN * sizeof(float));

    // TODO: look into how much is used to set stack size accodringly (especially for camera task)
    xTaskCreate(train_task, "train_task", configMINIMAL_STACK_SIZE + 2000, NULL, 1, NULL);

    // TODO: move this to task and delete main task
    while (1)
    {
        ret = camera_capture_image(&s1);
        if (ret != 0)
        {
            ESP_LOGE("capture_image", "Capture Image Failed");
            return;
        }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}