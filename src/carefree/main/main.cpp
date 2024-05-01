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
// #include "human_face_detect_mnp01.hpp"
#include "dl_tool.hpp"

// DEBUG DEFINES
#define DEBUG_SHOW_IMAGE 0
#define DEBUG_SHOW_FACE 0

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
#define PIXFORMAT PIXFORMAT_RGB565
// #define PIXFORMAT PIXFORMAT_GRAYSCALE
#define FRAMESIZE FRAMESIZE_QVGA
// #define FRAMESIZE FRAMESIZE_VGA
#define JPG_MAX_SIZE 5 * 1024
#define JPG_QUALITY 12

// WIFI DEFINES

#define WIFI_MAX_RETRIES 5

// HTTP DEFINES

#define HTTP_HOST "192.168.24.26" // Server domain/address and port here
#define HTTP_PORT 5000

// ML DEFINES

#define POOL_KERNEL_SIZE 4
#define FACE_SIZE 64
#define FEATS_LEN 576 // dependent on image face size and HoG parameters

// TAGS

const char *APP_TAG = "CARefree";
const char *WIFI_TAG = "WiFi";
const char *HTTP_TAG = "HTTP";
const char *CAM_TAG = "Camera";
const char *ML_TAG = "Train";

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

// HTTP GLOBALS
const bool send_jpeg_encoded = false;

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
        if (!evt->user_data)
            break;
        memcpy(((char *)evt->user_data) + cur_copied, evt->data, evt->data_len);
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

void http_post(const char *host, int port, const char *path, char *data, size_t data_len, const char *content_type, char *response)
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
    // TODO: look into keeping the connection alive to save the redundancy of the TCP handhsake,
    // although it might not be needed when duty cycle sleep is incorporated
    config.keep_alive_enable = false;

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_post_field(client, data, data_len);
    esp_http_client_perform(client);

    esp_http_client_cleanup(client);
}

// CAMERA FUNCTIONS

int camera_init(pixformat_t pixformat, framesize_t framesize)
{
    esp_rom_gpio_pad_select_gpio((uint32_t)CAM_PIN_PWDN);
    gpio_set_direction((gpio_num_t)CAM_PIN_PWDN, (gpio_mode_t)GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CAM_PIN_PWDN, (uint32_t)0);

    esp_rom_gpio_pad_select_gpio((uint32_t)CAM_PIN_RET);
    gpio_set_direction((gpio_num_t)CAM_PIN_RET, (gpio_mode_t)GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CAM_PIN_PWDN, (uint32_t)1);

    camera_config_t config;
    memset(&config, 0, sizeof(config));
    config.ledc_channel = (ledc_channel_t)LEDC_CHANNEL_0;
    config.ledc_timer = (ledc_timer_t)LEDC_TIMER_0;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = (pixformat_t)pixformat;
    config.frame_size = (framesize_t)framesize;
    config.fb_location = (camera_fb_location_t)CAMERA_FB_IN_DRAM;
    config.grab_mode = (camera_grab_mode_t)CAMERA_GRAB_LATEST; // See difference from CAMERA_GRAB_WHEN_EMPTY
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
                }
            }
            uint16_t r = (uint16_t)(sum[0] / (kernel_size * kernel_size));
            uint16_t g = (uint16_t)(sum[1] / (kernel_size * kernel_size));
            uint16_t b = (uint16_t)(sum[2] / (kernel_size * kernel_size));

            if (convert_to_grayscale)
                ((uint8_t *)out)[(i / kernel_size) * out_width + (j / kernel_size)] = ((r << 3) + (g << 2) + (b << 3)) / 3;
            else
                ((uint16_t *)out)[(i / kernel_size) * out_width + (j / kernel_size)] = (r << 11) | (g << 5) | b;
        }
    }
    return out;
}

uint8_t *resize_bilinear_grayscale(uint8_t *in, int height, int width, int out_height, int out_width, dl::detect::result_t &face_pos)
{
    uint8_t *out = (uint8_t *)malloc(out_height * out_width);

    for (int i = 0; i < out_height; i++)
    {
        float in_i = i / ((float)out_height) * (face_pos.box[3] - face_pos.box[1]) + face_pos.box[1];
        int in_i_1 = (int)floorf(in_i);
        int in_i_2 = (int)ceilf(in_i);
        in_i_2 = in_i_1 == in_i_2 ? in_i_2 + 1 : in_i_2;
        for (int j = 0; j < out_width; j++)
        {
            float in_j = j / ((float)out_width) * (face_pos.box[2] - face_pos.box[0]) + face_pos.box[0];
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

float *extract_hog_features(uint8_t *in, int height, int width)
{
    const int patch_size = 8;
    const int num_bins = 9;
    int delta = 180 / num_bins;

    float *hog = (float *)malloc((height / patch_size) * (width / patch_size) * num_bins * sizeof(float));

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

                    int grad = (int)roundf(fabsf(atan2f(dy, dx)) / M_PI * 180.f); // TODO: is this correct?
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
            // Unlike standard HoG, we don't perform conflation of neighboring blocks here (7 * 7 * 36 would be too large of a feature vector)
            // Instead, normalization is done on each block
            float sqr_norm = 0.f;
            for (int k = 0; k < num_bins; k++)
                sqr_norm += hog[bins_start + k] * hog[bins_start + k];
            for (int k = 0; k < num_bins; k++)
                hog[bins_start + k] /= sqrtf(sqr_norm + 1e-6); // Done to avoid 0 division
        }
    }
    return hog;
}

int camera_capture_image(HumanFaceDetectMSR01 *s1)
{
    // static char http_response[FEATS_LEN * sizeof(float)];
    // Capture an image
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
        return -1;

    // Process image
    ESP_LOGI(CAM_TAG, "Image captured height=%d, width=%d, length=%d", fb->height, fb->width, fb->len);
    if (wifi_connected)
    {
        // It doesn't work to use the pooled image for HumanFaceDetectMSR01 infering, because of this error:
        // assert failed: dl::Tensor<T>& dl::Tensor<T>::set_shape(std::vector<int>) [with T = short int] dl_variable.cpp:20 (shape[i] >= 0)
        // TODO: maybe look into it but it seems to be a bug with the library (maybe try different pooling shapes or something
        // but even with square 60x60 it seems to complain, maybe it only wants QVGA but it should have a resize thingy anyway)
        ESP_LOGW(CAM_TAG, "FREE HEAP MEMORY BEFORE INFER: %lu", esp_get_free_heap_size());
        std::list<dl::detect::result_t> candidates = s1->infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});

#if DEBUG_SHOW_IMAGE
        if (candidates.size())
        {
            int32_t rect[4] = {candidates.front().box[0], candidates.front().box[1], candidates.front().box[2], candidates.front().box[3]};
            http_post(HTTP_HOST, HTTP_PORT, "/debug/set_rect", (char *)rect, 4 * sizeof(uint32_t), "application/octet-stream", NULL);
        }
        if (send_jpeg_encoded) // This makes the program run out of memory on next img, beware
        {
            char *jpg_img = (char *)malloc(JPG_MAX_SIZE);
            jpg_encode_cb_params params = {.img = jpg_img, .img_len = 0};
            // Convert image to JPEG
            frame2jpg_cb(fb, JPG_QUALITY, jpg_write_buf_cb, &params);
            // Send image to server
            http_post(HTTP_HOST, HTTP_PORT, "/debug/set_image", jpg_img, params.img_len, "application/jpeg", NULL);
            free(jpg_img);
        }
        else
        {
            // Send image to server
            http_post(HTTP_HOST, HTTP_PORT, "/debug/set_image", (char *)fb->buf, fb->len, "application/octet-stream", NULL);
        }
#endif
        if (candidates.size())
        {
            dl::detect::result_t &face = candidates.front();
            uint8_t *pooled = (uint8_t *)avg_pool_rgb565((uint16_t *)fb->buf, fb->height, fb->width, POOL_KERNEL_SIZE, true);
            int height = fb->height / POOL_KERNEL_SIZE;
            int width = fb->width / POOL_KERNEL_SIZE;
            face.box[0] = face.box[0] < 0 ? 0 : face.box[0] / POOL_KERNEL_SIZE;
            face.box[1] = face.box[1] < 0 ? 0 : face.box[1] / POOL_KERNEL_SIZE;
            face.box[2] = face.box[2] > fb->width ? width : (int)(ceilf(face.box[2] / (float)POOL_KERNEL_SIZE));
            face.box[3] = face.box[3] > fb->height ? height : (int)(ceilf(face.box[3] / (float)POOL_KERNEL_SIZE));
            uint8_t *resized = resize_bilinear_grayscale(pooled, height, width, 64, 64, face);
            free(pooled);
#if DEBUG_SHOW_FACE
            http_post(HTTP_HOST, HTTP_PORT, "/debug/set_image", (char *)resized, FACE_SIZE * FACE_SIZE, "application/octet-stream", NULL);
#endif
            float *hog = extract_hog_features(resized, FACE_SIZE, FACE_SIZE);
            free(resized);
            xQueueSend(feats_queue, hog, portMAX_DELAY);
            free(hog);
        }
    }
    // Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return 0;
}

// ML FUNCTIONS

void train_task(void *arg)
{
    static float feats[FEATS_LEN];
    static float w[FEATS_LEN * 7];
    static float b[FEATS_LEN];

    while (1)
    {
        xQueueReceive(feats_queue, feats, portMAX_DELAY);
        // Uncomment to see whether program actually works with the set weights and biases without running out of memory
        // If uncommented, the optimizing compiler just removes the definitions for the unused vectors
        // for (int i = 0; i < FEATS_LEN; i++)
        // {
        //     w[i] = feats[i] + 1.f;
        //     b[i] = feats[i] + 2.f;
        // }
        // TODO: train a classifier, preferably a 1-layer NN
        ESP_LOGW(HTTP_TAG, "5th value of features is %f", feats[5]);
    }
}

extern "C" void app_main(void)
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
    HumanFaceDetectMSR01 s1(0.1F, 0.5F, 1, 0.2F);

    feats_queue = xQueueCreate(1, FEATS_LEN * sizeof(float));

    // TODO: look into how much is used to set stack size accodringly (especially for camera task)
    xTaskCreate(train_task, "train_task", configMINIMAL_STACK_SIZE + 1000, NULL, 1, NULL);

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