// Using modified code from https://gist.github.com/mws-rmain/2ba434cd2a3f32d6d343c1c60fbd65c8
// ADXL345 datasheet for reference:
// https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "adxl345.h"
#include "driver/i2c.h"
#include <string.h>

#define I2C_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
// I2C common protocol defines
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

#define I2C_GLITCH_IGNORE_CNT 7
#define I2C_CLK_FREQ 100000
#define A_TO_READ 6

static const char *TAG = "adxl345";

static i2c_port_t adxl345_i2c_port;
static uint8_t adxl345_i2c_addr;

static int timeout;

static void i2c_read_from_reg(uint8_t i2c_reg, uint8_t *data_rd, size_t size)
{
    if (size == 0)
        return;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, (adxl345_i2c_addr << 1), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, (adxl345_i2c_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(adxl345_i2c_port, cmd, timeout / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Read error");

    i2c_cmd_link_delete(cmd);
}

static void i2c_write_to_reg(uint8_t i2c_reg, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, (adxl345_i2c_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(adxl345_i2c_port, cmd, timeout / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Write error");

    i2c_cmd_link_delete(cmd);
}

static void i2c_write_byte_to_reg(uint8_t i2c_reg, uint8_t byte)
{
    char byte_str[9];
    for (int i = 0; i < 8; i++)
        byte_str[7 - i] = (byte & (1 << i)) ? '1' : '0';
    byte_str[8] = 0;
    ESP_LOGE(TAG, "Writing byte 0b%s to reg 0x%02X", byte_str, i2c_reg);

    i2c_write_to_reg(i2c_reg, &byte, 1);
}

int adxl345_init(uint8_t i2c_address, uint8_t scl_pin, uint8_t sda_pin, int i2c_port, int timeout_ms)
{
    esp_err_t ret;
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_CLK_FREQ;
    i2c_param_config(i2c_port, &conf);

    ret = i2c_driver_install(i2c_port, conf.mode, I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK && ret != ESP_FAIL) // for some reason it returns fail even when it correctly initializes
    {
        ESP_LOGE(TAG, "Failed to initiate I2C");
        return -1;
    }

    adxl345_i2c_port = i2c_port;
    adxl345_i2c_addr = i2c_address;
    timeout = timeout_ms;

    // Set mode to measure
    i2c_write_byte_to_reg(0x2D, 1 << 3);
    // Set mode to 13 bit full resolution, +/-16g at 3.9mg/LSB
    i2c_write_byte_to_reg(0x31, 0x0B);
    // Set data rate to 1001, 50Hz, normal mode
    // Low power mode only works for rates 0111 to 1100 (12.5-400Hz)
    i2c_write_byte_to_reg(0x2C, 0b1001);
    return 0;
}

void adxl345_get_data(int16_t *result)
{
    uint8_t regAddress = 0x32;
    uint8_t buff[A_TO_READ];
    i2c_read_from_reg(regAddress, buff, A_TO_READ);
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

    // i2c_read_from_reg(reg_int_map, &val, 1);
    // ESP_LOGE(TAG, "REG INT MAP val: %hhu", val);
    val = 0;
    if (int_pin == 1)
        val &= ~(1 << type);
    else
        val |= 1 << type;
    i2c_write_byte_to_reg(reg_int_map, val);

    // i2c_read_from_reg(reg_int_enable, &val, 1);
    // ESP_LOGE(TAG, "REG INT ENABLE val: %hhu", val);
    val = 0;
    if (enable)
        val |= 1 << type;
    else
        val &= ~(1 << type);
    i2c_write_byte_to_reg(reg_int_enable, val);
}

void adxl345_set_activity_threshold(uint8_t threshold, bool x_axis, bool y_axis, bool z_axis)
{
    const uint8_t reg_activity_threshold = 0x24;
    const uint8_t reg_act_inact_ctl = 0x27;
    uint8_t val;

    i2c_write_byte_to_reg(reg_activity_threshold, threshold);

    // i2c_read_from_reg(reg_act_inact_ctl, &val, 1);
    // ESP_LOGE(TAG, "REG ACT INACT CTL val: %hhu", val);
    val = 0;
    // val |= 1 << ADXL345_ACT_ACDC;
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
    i2c_write_byte_to_reg(reg_act_inact_ctl, val);
}