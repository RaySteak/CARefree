// Snatched and modified from https://github.com/imxieyi/esp32-i2c-adxl345
// ADXL345 datasheet for reference:
// https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"
#include "driver/gpio.h"
#include "adxl345.h"
#include "i2c.h"

#define A_TO_READ (6)

static const char *TAG = "adxl345";

static uint8_t ACC;

// Write val to address register on ACC
void writeTo(uint8_t DEVICE, uint8_t address, uint8_t val)
{
    if (!i2c_slave_write_with_reg(DEVICE, address, val))
        ESP_LOGE(TAG, "I2C write error");
}

// Read num bytes starting from address register on ACC in to buff array
void readFrom(uint8_t DEVICE, uint8_t address, uint8_t num, uint8_t buff[])
{
    if (!i2c_slave_read(DEVICE, address, buff, num))
        ESP_LOGE(TAG, "I2C read error");
}

void adxl345_init(uint8_t i2c_address, uint8_t scl_pin, uint8_t sda_pin)
{
    ACC = i2c_address;
    // Turning on ADXL345
    i2c_init(scl_pin, sda_pin);
    // Set mode to measure
    writeTo(ACC, 0x2D, 1 << 3);
    // Set mode to 13 bit full resolution, +/-16g at 3.9mg/LSB
    writeTo(ACC, 0x31, 0x0B);
    // Set data rate to 1001, 50Hz, low power mode
    // Low power mode only works for rates 0111 to 1100 (12.5-400Hz)
    writeTo(ACC, 0x2C, 0b1001);
}

void adxl345_get_data(int16_t *result)
{
    uint8_t regAddress = 0x32;
    uint8_t buff[A_TO_READ];
    readFrom(ACC, regAddress, A_TO_READ, buff);
    result[0] = (((int)buff[1]) << 8) | buff[0];
    result[1] = (((int)buff[3]) << 8) | buff[2];
    result[2] = (((int)buff[5]) << 8) | buff[4];
}

void adxl345_set_interrupt(adxl345_int_type type, int int_pin, bool enable)
{
    const uint8_t reg_int_enable = 0x2E;
    const uint8_t reg_int_map = 0x2F;
    uint8_t val;

    if (int_pin != 1 && int_pin != 2)
    {
        ESP_LOGE(TAG, "Invalid interrupt pin");
        return;
    }

    // readFrom(ACC, reg_int_map, 1, &val);
    // ESP_LOGE(TAG, "REG INT MAP val: %hhu", val);
    val = 0;
    if (int_pin == 1)
        val &= ~(1 << type);
    else
        val |= 1 << type;
    writeTo(ACC, reg_int_map, val);

    // readFrom(ACC, reg_int_enable, 1, &val);
    // ESP_LOGE(TAG, "REG INT ENABLE val: %hhu", val);
    val = 0;
    if (enable)
        val |= 1 << type;
    else
        val &= ~(1 << type);
    writeTo(ACC, reg_int_enable, val);
}

void adxl345_set_activity_threshold(uint8_t threshold, bool x_axis, bool y_axis, bool z_axis)
{
    const uint8_t reg_activity_threshold = 0x24;
    const uint8_t reg_act_inact_ctl = 0x27;
    uint8_t val;

    writeTo(ACC, reg_activity_threshold, threshold);

    // readFrom(ACC, reg_act_inact_ctl, 1, &val);
    // ESP_LOGE(TAG, "REG ACT INACT CTL val: %hhu", val);
    val = 0;
    if (x_axis)
        val |= 1 << ADXL345_ACT_X;
    else
        val &= ~(1 << ADXL345_ACT_X);
    if (y_axis)
        val |= 1 << ADXL345_ACT_Y;
    else
        val &= ~(1 << ADXL345_ACT_Y);
    if (z_axis)
        val |= 1 << ADXL345_ACT_Z;
    else
        val &= ~(1 << ADXL345_ACT_Z);
    ESP_LOGE(TAG, "Writing %hhu to reg_act_inact_ctl", val);
    writeTo(ACC, reg_act_inact_ctl, val);
}