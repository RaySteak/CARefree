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
#include "esp_psram.h"

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

// #define PIXFORMAT PIXFORMAT_RG  B565
#define PIXFORMAT PIXFORMAT_GRAYSCALE

// #define FRAMESIZE FRAMESIZE_QVGA
#define FRAMESIZE FRAMESIZE_VGA

int init_camera(pixformat_t pixformat, framesize_t framesize)
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
        .xclk_freq_hz = 20000000,
        .pixel_format = pixformat,
        .frame_size = framesize,
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_count = 1,
        .sccb_i2c_port = 1,
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
    return 0;
}

int capture_image(void)
{
    // Capture an image
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
        return -1;

    // Process image here
    ESP_LOGI("capture_image", "Image captured height=%d, width=%d", fb->height, fb->width);

    // Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return 0;
}

void app_main(void)
{
    int ret;

    // Get free heap memory
    ESP_LOGI("app_main", "Free heap memory: %lu bytes", esp_get_free_heap_size());

    ret = init_camera(PIXFORMAT, FRAMESIZE);
    if (ret != 0)
    {
        ESP_LOGE("init_camera", "Camera Init Failed");
        return;
    }

    while (1)
    {
        ret = capture_image();
        if (ret != 0)
        {
            ESP_LOGE("capture_image", "Capture Image Failed");
            return;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}