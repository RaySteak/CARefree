#define INCLUDE_vTaskSuspend 1

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

#define WINDOW_WIDTH 5

const char *APP_NAME = "WIFI_AWS_MQTT";

const char *ssid = "FASTWEB-ksA45e";
const char *pass = "mh9ms5Ghuq";

// const char *ssid = "OnePlus Nord";
// const char *pass = "ayylmao123";

QueueHandle_t queue;
SemaphoreHandle_t xSemaphoreWifiConnected;

TaskHandle_t myTaskHandle1 = NULL;
TaskHandle_t myTaskHandle2 = NULL;

esp_mqtt_client_handle_t client;
int mqtt_connected = 0;

// !!!!!!!!
// mosquitto_pub -i basicPubSub --cafile root-CA.crt --key ESP32.private.key --cert ESP32.cert.pem -h aighnm5nrap92-ats.iot.us-east-1.amazonaws.com -t 'sdk/test/python' -m 'IT WORKS'
//

const char *aws_uri = "mqtts://aighnm5nrap92-ats.iot.us-east-1.amazonaws.com";
const char *client_id = "basicPubSub";
char *topic = "sdk/test/python";

const char *root_ca_cert = "-----BEGIN CERTIFICATE-----\n"
                           "MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
                           "ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
                           "b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
                           "MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
                           "b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
                           "ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
                           "9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
                           "IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
                           "VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
                           "93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
                           "jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
                           "AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
                           "A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
                           "U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
                           "N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
                           "o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
                           "5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
                           "rqXRfboQnoZsG4q5WTP468SQvvG5\n"
                           "-----END CERTIFICATE-----\n";

const char *cert = "-----BEGIN CERTIFICATE-----\n"
                   "MIIDWjCCAkKgAwIBAgIVAOXqKXkfp2lFEe0B9CulVFZ/c2J6MA0GCSqGSIb3DQEB\n"
                   "CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t\n"
                   "IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNDA0MDMyMzI1\n"
                   "MjlaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh\n"
                   "dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCxb+R/NkbYFfmaA83r\n"
                   "AQvQTeeQOmLpP/ZZ9OqtjqQ1WowTdkbqSO8f3ciZRoAfXVcqJCnEmA3KTN3H8DO0\n"
                   "H6ubmFlarc+zZ1HCV18dpZcJdi0EKXeJMgQRI30f7Ugd3NgQA1rlNwhPoxUWtHom\n"
                   "daChR7yNOib7bzsdqD7w1S8NjV9xTEsrnFi8rmJd3EgdqSmUSWLbNn0LfFriQVpg\n"
                   "ZNGzj005tQURRHY3+u9PDn6zvENE28P2LdY50435CZYhq58uUoaisNmw+CLk9yRG\n"
                   "ystpdAtx1Ct5eVFK9WYQAuFerHG/t0t5gLMaAydxdbEla24U3Jv1lRq2cuEirUZz\n"
                   "7orDAgMBAAGjYDBeMB8GA1UdIwQYMBaAFOTzWL3HFlSkTuyrYEWYfEN11403MB0G\n"
                   "A1UdDgQWBBQz574HXitbXUke48DxmRD95AohoTAMBgNVHRMBAf8EAjAAMA4GA1Ud\n"
                   "DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEARFcJ6Qn8vQ8piPRL5oqSOem2\n"
                   "X7KnrXN8oEvLwhHVGaCbeyTUw2GsqxDQavn+ByMdSD6YcUc8dJ9sRJ0CTaZykpPE\n"
                   "k+V6btzote+SytZD9tZjY5PyRrpCWUuoxff2TFEMl4ZWgw5FFN6lq0DFad5hmQZr\n"
                   "PCFPiHcN0yuI+3uB7i/kZPmOXGZnBphgVa/ZjTUARzyZ1B+QfrUGluDf1SrW7Mzg\n"
                   "wfORiCZBN1q3U0aQinOT5rPDTtnKWclBbZWioCrSB/Qiw47PylVDYhhayQ1l0GfS\n"
                   "zmu7jFFEM9efNo+HqAwL8UAfZ61vBbkEeHHWYXrUYfM2h3UAVKazuC03ji9Ieg==\n"
                   "-----END CERTIFICATE-----\n";

const char *privkey = "-----BEGIN RSA PRIVATE KEY-----\n"
                      "MIIEowIBAAKCAQEAsW/kfzZG2BX5mgPN6wEL0E3nkDpi6T/2WfTqrY6kNVqME3ZG\n"
                      "6kjvH93ImUaAH11XKiQpxJgNykzdx/AztB+rm5hZWq3Ps2dRwldfHaWXCXYtBCl3\n"
                      "iTIEESN9H+1IHdzYEANa5TcIT6MVFrR6JnWgoUe8jTom+287Hag+8NUvDY1fcUxL\n"
                      "K5xYvK5iXdxIHakplEli2zZ9C3xa4kFaYGTRs49NObUFEUR2N/rvTw5+s7xDRNvD\n"
                      "9i3WOdON+QmWIaufLlKGorDZsPgi5PckRsrLaXQLcdQreXlRSvVmEALhXqxxv7dL\n"
                      "eYCzGgMncXWxJWtuFNyb9ZUatnLhIq1Gc+6KwwIDAQABAoIBAC/SR1QdzcQpUAc5\n"
                      "ZUMkGgn/JRzTD5truLqP7rvuCMW0bLJGG2DDXWdKP+lHffp2Dqr16ifpDzZptxBO\n"
                      "5bBzJK4fxl1RIPxqKTfmkmUMHwPWzNUlU+kvCLeRyARV0Cz0O2s9bD8AsQW9n05q\n"
                      "P82jwDIXgnLRrHXxiY45rRT9b9wHvdEVOSq9O28J+tQNMjcwK0d0Fm4lZ64X32Kp\n"
                      "ysP+jrOPlwmiUFPWmoL+8xZeKkPuxDiBio7GXaiuSvJTQ+AL9H6pf3apGM3VN0iP\n"
                      "xvUrDqvCZw0dAvitc+R9pJy5JrQeNc4osIoWQ0UMbzO2mDQRJnoRvK297iViJE4V\n"
                      "thJ/AskCgYEA6KtW3z3ob+ICBFdDNZBOQLje7MVVEGF8z2PAktF3RRSNXliVPEls\n"
                      "RDJJz5+jQOLOojKE/tN04rRKWIzsPsbzHUF1H+trnbTYsLUXmE+03EXOejyYirbs\n"
                      "QeENGG62McDGxtmpjEyecexrCaGO9hPhSbDhQoyC7I/aquG6LXS7tBUCgYEAwzq7\n"
                      "mmU9b8VcnG9FKlDpTVxq+GEaxy8G17yxQRrB2weWxgEXhgVinlLzUJFExC9ENim/\n"
                      "4GS6MQCwRjYfe75D2vq00UGAhVLHlZVipejS46JOFjU/Gu5JVoUeXN876ScB8hXy\n"
                      "ojhsbmEJQ5+rbJru/ruqNBFrZ4MEE98lj97CwXcCgYBX+VEjn/jfNQmmMoiLHEE6\n"
                      "X6yKwiJjyziyAfmyidkbGaZKYhwra2ewcYNPKgFEa33N/mQjJjkv7xOdX8uEAR2z\n"
                      "lih/JTjPrK2yQwbk5F5yW0KzHW5gJMI4NqwI7POe5FFEptBg6CeQ09+1CI1Rpjqs\n"
                      "7mB+FTr7z3jS2/7VaSn/8QKBgQC+LVuM5mw3khlKJjZJJ3JtMsbdV7J/L7dJtv0w\n"
                      "f4Ex/S2JiozAuEQ+GG3/INPNDII1BOxuCZE6pQISEy0OHOT6ztgPM/Cfb2kOtOvY\n"
                      "1HmC15ublJj5ggAc8SYHsEorMB18Fpc6IR63U31zy3lxHWgM6wC9Ie7vcbgqJ2ju\n"
                      "wkATtwKBgCF+8+0GXEdJ8zo7iyqZeKyVYTHYRcd59sbxFDR0QRwzHm5ZqULLurkk\n"
                      "/+2OEYp/aBlab2rZvCza1Q2e+XhNwlxatPc0fnH6dsxDAnKkn2rkbAMhUOb8rHZB\n"
                      "w6QhzXuwoRRH8mQIOGiuUEiEzRn70NQpk3coiCrWAfp/2gsao/X7\n"
                      "-----END RSA PRIVATE KEY-----\n";

// WiFi
static void
wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    static int retry_num = 0;

    if (event_id == WIFI_EVENT_STA_START)
    {
        printf("WIFI CONNECTING....\n");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        printf("WiFi CONNECTED\n");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi lost connection\n");
        if (retry_num < 5)
        {
            esp_wifi_connect();
            retry_num++;
            printf("Retrying to Connect...\n");
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        xSemaphoreGive(xSemaphoreWifiConnected);
        printf("Wifi got IP...\n\n");
    }
}

void wifi_connection()
{
    //                          s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_netif_init();
    esp_event_loop_create_default();     // event loop                    s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station                      s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); //
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",
        }};
    strcpy((char *)wifi_configuration.sta.ssid, ssid);
    strcpy((char *)wifi_configuration.sta.password, pass);
    // esp_log_write(ESP_LOG_INFO, "Kconfig", "SSID=%s, PASS=%s", ssid, pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
    printf("wifi_init_softap finished. SSID:%s  password:%s", ssid, pass);
}

// MQTT

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;      // here esp_mqtt_event_handle_t is a struct which receieves struct event from mqtt app start funtion
    esp_mqtt_client_handle_t client = event->client; // making obj client of struct esp_mqtt_client_handle_t and assigning it the receieved event client
    if (event->event_id == MQTT_EVENT_CONNECTED)
    {
        mqtt_connected = 1;
        ESP_LOGI(APP_NAME, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, topic, 0); // in mqtt we require a topic to subscribe and client is from event client and 0 is quality of service it can be 1 or 2
        ESP_LOGI(APP_NAME, "sent subscribe successful");
    }
    else if (event->event_id == MQTT_EVENT_DATA)
    {
        ESP_LOGI(APP_NAME, "MQTT_EVENT_DATA");
        char json[100];
        memcpy(json, event->data, event->data_len);
        json[event->data_len] = 0;
        printf("Received data from MQTT: %s", json);
    }
    else if (event->event_id == MQTT_EVENT_DISCONNECTED)
    {
        mqtt_connected = 0;
        ESP_LOGI(APP_NAME, "MQTT_EVENT_DISCONNECTED"); // if disconnected
    }
    else if (event->event_id == MQTT_EVENT_SUBSCRIBED)
    {
        ESP_LOGI(APP_NAME, "MQTT_EVENT_SUBSCRIBED");
    }
    else if (event->event_id == MQTT_EVENT_UNSUBSCRIBED) // when subscribed
    {
        ESP_LOGI(APP_NAME, "MQTT_EVENT_UNSUBSCRIBED");
    }
    else if (event->event_id == MQTT_EVENT_DATA) // when unsubscribed
    {
        ESP_LOGI(APP_NAME, "MQTT_EVENT_DATA");
    }
    else if (event->event_id == MQTT_EVENT_ERROR) // when any error
    {
        ESP_LOGI(APP_NAME, "MQTT_EVENT_ERROR");
    }
}

static void mqtt_initialize(void)
{ /*Depending on your website or cloud there could be more parameters in mqtt_cfg.*/
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = aws_uri, // Uniform Resource Identifier includes path,protocol
            .verification = {
                .certificate = root_ca_cert, // described above event handler
            }},
        .credentials = {.authentication = {.certificate = cert, .key = privkey}, .client_id = client_id}
        // .username = /*your username*/,                                 // your username
        // .password = /*your adafruit password*/,                        // your adafruit io password
    };
    client = esp_mqtt_client_init(&mqtt_cfg);                                           // sending struct as a parameter in init client function
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL); // registering event handler
    esp_mqtt_client_start(client);                                                      // starting the process
}

// TASKS

void Producer_Task(void *arg)
{
    while (1)
    {
        int value = rand() % 100;
        xQueueSend(queue, &value, 0);
        ESP_LOGI(APP_NAME, "Sent data to queue: %d", value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Consumer_Task(void *arg)
{
    int value;
    int cur_width = 0;
    int cur_pos = 0;
    int sum = 0;

    int buffer[WINDOW_WIDTH];
    while (1)
    {
        mqtt_connected = 1;
        if (mqtt_connected)
        {
            if (xQueueReceive(queue, &value, 5))
            {
                ESP_LOGI(APP_NAME, "Received data from queue: %d", value);

                buffer[cur_pos] = value;
                cur_pos = (cur_pos + 1) % WINDOW_WIDTH;
                sum += value;

                if (cur_width < WINDOW_WIDTH)
                    cur_width++;
                else
                    sum -= buffer[cur_pos];

                float average = (float)sum / cur_width;

                char json[100];
                sprintf(json, "{\"average\": %f}", average);
                int msg_id = esp_mqtt_client_publish(client, "sdk/test/python", json, 0, 1, 0);

                ESP_LOGI(APP_NAME, "Published data, msg_id=%d", msg_id);

                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void app_main(void)
{
    xSemaphoreWifiConnected = xSemaphoreCreateBinary();
    // WiFi
    nvs_flash_init();
    wifi_connection();
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    // MQTT
    xSemaphoreTake(xSemaphoreWifiConnected, portMAX_DELAY);
    mqtt_initialize();
    // Queue
    queue = xQueueCreate(5, sizeof(int));

    if (queue == 0)
        printf("Error in creating the queue\n");

    xTaskCreate(Producer_Task, "Producer", 4096, NULL, 10, &myTaskHandle1);
    xTaskCreate(Consumer_Task, "Consumer", 4096, NULL, 10, &myTaskHandle2);
}