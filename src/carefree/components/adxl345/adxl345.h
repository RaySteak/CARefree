// Snatched and modified from https://github.com/imxieyi/esp32-i2c-adxl345
#ifndef __ADXL345_H__
#define __ADXL345_H__
#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum _adxl345_int_type
    {
        ADXL345_INT_OVERRUN = 0,
        ADXL345_INT_WATERMARK,
        ADXL345_INT_FREEFALL,
        ADXL345_INT_INACTIVITY,
        ADXL345_INT_ACTIVITY,
        ADXL345_INT_DOUBLE_TAP,
        ADXL345_INT_SINGLE_TAP,
        ADXL345_INT_DATA_READY
    } adxl345_int_type;

    typedef enum _adxl345_act_inact_ctl
    {
        ADXL345_INACT_Z = 0,
        ADXL345_INACT_Y,
        ADXL345_INACT_X,
        ADXL345_INACT_ACDC,
        ADXL345_ACT_Z,
        ADXL345_ACT_Y,
        ADXL345_ACT_X,
        ADXL345_ACT_ACDC
    } adxl345_act_inact_ctl;

    int adxl345_init(uint8_t i2c_address, uint8_t scl_pin, uint8_t sda_pin, int i2c_port, int timeout_ms);
    void adxl345_get_data(int16_t *result);
    void adxl345_set_interrupt(adxl345_int_type type, int int_pin, bool enable);
    void adxl345_set_activity_threshold(uint8_t threshold, bool x_axis, bool y_axis, bool z_axis);

#ifdef __cplusplus
}
#endif

#endif