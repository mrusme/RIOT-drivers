/*
 * Implementation based on original implementation by Bosch Sensortec GmbH
 * Copyright (C) 2019 Marius <marius@twostairs.com>
 *
 * Copyright of reference implementation:
 * Copyright (C) 2016 - 2017 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 */

/**
 * @ingroup     drivers_bmp388
 * @{
 *
 * @file
 * @brief       Device driver implementation for the BMP388 precision pressure sensor.
 *
 * @author      Marius <marius@twostairs.com>
 *
 * @}
 */

#include <math.h>

#include "bmp388.h"

#include "xtimer.h"
#include "periph_conf.h"
#include "periph/i2c.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define DEV_I2C      (dev->params.i2c_dev)
#define DEV_ADDR     (dev->params.i2c_addr)

int bmp388_init(bmp388_t *dev, const bmp388_params_t *params) {
    int retval;
    uint8_t id;
    uint16_t settings_sel = 0;

    dev->params = *params;
    dev->params.settings.temp_en = BMP3_ENABLE;
    dev->params.settings.press_en = BMP3_ENABLE;
    dev->params.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_8X;
    dev->params.settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    dev->params.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_CHIP_ID_ADDR, &id, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    if(id != 0x50) {
        return -BMP388_ERR_NODEV;
    }

    retval = bmp388_soft_reset(dev);
    if(retval != BMP388_OK) {
        return retval;
    }

    retval = bmp388_get_calib_data(dev);
    if(retval != BMP388_OK) {
        return retval;
    }

    settings_sel |= BMP3_TEMP_EN_SEL;
    settings_sel |= BMP3_TEMP_OS_SEL;
    settings_sel |= BMP3_PRESS_EN_SEL;
    settings_sel |= BMP3_PRESS_OS_SEL;
    settings_sel |= BMP3_IIR_FILTER_SEL;
    settings_sel |= BMP3_ODR_SEL;

    // set interrupt to data ready
    //settings_sel |= BMP3_DRDY_EN_SEL | BMP3_LEVEL_SEL | BMP3_LATCH_SEL;

    retval = bmp388_set_sensor_settings(dev, settings_sel);
    if(retval != BMP388_OK) {
        return retval;
    }

    dev->params.settings.op_mode = BMP3_FORCED_MODE;
    retval = bmp388_set_op_mode(dev);
    if(retval != BMP388_OK) {
        return retval;
    }

    return BMP388_OK;
}

int bmp388_soft_reset(bmp388_t *dev) {
    uint8_t cmd_rdy_status;
    uint8_t cmd_err_status;

    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_SENS_STATUS_REG_ADDR, &cmd_rdy_status, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    if(!(cmd_rdy_status & BMP3_CMD_RDY)) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NOTRDY;
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, BMP3_CMD_ADDR, BMP388_CMD_SOFT_RESET, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_CMD;
    }

    xtimer_usleep((2U * US_PER_MS));

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_ERR_REG_ADDR, &cmd_err_status, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    if((cmd_err_status & BMP3_CMD_ERR)) {
        return -BMP388_ERR_CMD;
    }

    return BMP388_OK;
}

int bmp388_get_calib_data(bmp388_t *dev) {
    uint8_t calib_data[BMP3_CALIB_DATA_LEN] = {0};

    i2c_acquire(DEV_I2C);

    if(i2c_read_regs(DEV_I2C, DEV_ADDR, BMP3_CALIB_DATA_ADDR, &calib_data, BMP3_CALIB_DATA_LEN, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    bmp388_parse_calib_data(dev, calib_data);

    return BMP388_OK;
}

bool are_settings_changed(uint32_t sub_settings, uint32_t desired_settings) {
    bool settings_changed = false;

    if (sub_settings & desired_settings) {
        settings_changed = true;
    } else {
        settings_changed = false;
    }

    return settings_changed;
}

int bmp388_set_sensor_settings(bmp388_t *dev, uint32_t desired_settings) {
    int retval = 0;

    if(are_settings_changed(POWER_CNTL, desired_settings)) {
        retval = bmp388_set_pwr_ctrl_settings(dev, desired_settings);
    }

    if(are_settings_changed(ODR_FILTER, desired_settings) && (!retval)) {
        retval = bmp388_set_odr_filter_settings(dev, desired_settings);
    }

    if(are_settings_changed(INT_CTRL, desired_settings) && (!retval)) {
        retval = bmp388_set_int_ctrl_settings(dev, desired_settings);
    }

    if(are_settings_changed(ADV_SETT, desired_settings) && (!retval)) {
        retval = bmp388_set_advance_settings(dev, desired_settings);
    }

    return retval;
}

int bmp388_set_pwr_ctrl_settings(bmp388_t *dev, uint32_t desired_settings) {
    uint8_t reg_data;

    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, &reg_data, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    if(desired_settings & BMP3_TEMP_EN_SEL) {
        reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, BMP3_ENABLE);
    } else {
        reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, BMP3_DISABLE);
    }

    if(desired_settings & BMP3_PRESS_EN_SEL) {
        reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_PRESS_EN, BMP3_ENABLE);
    } else {
        reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_PRESS_EN, BMP3_DISABLE);
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, reg_data, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_CMD;
    }

    i2c_release(DEV_I2C);

    return BMP388_OK;
}

int bmp388_set_odr_filter_settings(bmp388_t *dev, uint32_t desired_settings) {
    uint8_t reg_addr[3] = {0};
    uint8_t reg_data[4];
    uint8_t len = 0;

    i2c_acquire(DEV_I2C);

    if(i2c_read_regs(DEV_I2C, DEV_ADDR, BMP3_OSR_ADDR, &reg_data, 4, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    if(are_settings_changed((BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL), desired_settings)) {
        fill_osr_data(dev, desired_settings, reg_addr, reg_data, &len);
    }

    if(are_settings_changed(BMP3_ODR_SEL, desired_settings)) {
        fill_odr_data(dev, reg_addr, reg_data, &len);
    }

    if(are_settings_changed(BMP3_IIR_FILTER_SEL, desired_settings)) {
        fill_filter_data(dev, reg_addr, reg_data, &len);
    }

    if(dev->params.settings.op_mode == BMP3_NORMAL_MODE) {
        if(validate_osr_and_odr_settings(dev) != BMP388_OK) {
            i2c_release(DEV_I2C);
            return -BMP388_ERR_CMD;
        }

        for(uint8_t i = 0; i <= len; i++) {
            if(i2c_write_reg(DEV_I2C, DEV_ADDR, reg_addr[i], reg_data[i], 0) != 0) {
                i2c_release(DEV_I2C);
                return -BMP388_ERR_CMD;
            }
        }
    }

    i2c_release(DEV_I2C);

    return BMP388_OK;
}

int bmp388_set_int_ctrl_settings(const bmp388_t *dev, uint32_t desired_settings) {
    uint8_t reg_data;
    struct bmp3_int_ctrl_settings int_settings;

    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_INT_CTRL_ADDR, &reg_data, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    int_settings = dev->params.settings.int_settings;

    if(desired_settings & BMP3_OUTPUT_MODE_SEL) {
        reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_INT_OUTPUT_MODE, int_settings.output_mode);
    }

    if(desired_settings & BMP3_LEVEL_SEL) {
        reg_data = BMP3_SET_BITS(reg_data, BMP3_INT_LEVEL, int_settings.level);
    }

    if(desired_settings & BMP3_LATCH_SEL) {
        reg_data = BMP3_SET_BITS(reg_data, BMP3_INT_LATCH, int_settings.latch);
    }

    if(desired_settings & BMP3_DRDY_EN_SEL) {
        reg_data = BMP3_SET_BITS(reg_data, BMP3_INT_DRDY_EN, int_settings.drdy_en);
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, BMP3_INT_CTRL_ADDR, reg_data, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_CMD;
    }

    i2c_release(DEV_I2C);

    return BMP388_OK;
}

int bmp388_set_advance_settings(const bmp388_t *dev, uint32_t desired_settings) {
    uint8_t reg_data;
    struct bmp3_adv_settings adv_settings = dev->params.settings.adv_settings;

    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_IF_CONF_ADDR, &reg_data, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    if(desired_settings & BMP3_I2C_WDT_EN_SEL) {
        /* Set the i2c watch dog enable bits */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_I2C_WDT_EN, adv_settings.i2c_wdt_en);
    }

    if(desired_settings & BMP3_I2C_WDT_SEL_SEL) {
        /* Set the i2c watch dog select bits */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_I2C_WDT_SEL, adv_settings.i2c_wdt_sel);
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, BMP3_IF_CONF_ADDR, reg_data, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_CMD;
    }

    i2c_release(DEV_I2C);

    return BMP388_OK;
}

void fill_osr_data(bmp388_t *dev, uint32_t settings, uint8_t *addr, uint8_t *reg_data, uint8_t *len) {
    struct bmp3_odr_filter_settings osr_settings = dev->params.settings.odr_filter;

    if (settings & (BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL)) {
        /* Pressure over sampling settings check */
        if (settings & BMP3_PRESS_OS_SEL) {
            /* Set the pressure over sampling settings in the
              register variable */
            reg_data[*len] = BMP3_SET_BITS_POS_0(reg_data[0], BMP3_PRESS_OS, osr_settings.press_os);
        }
        /* Temperature over sampling settings check */
        if (settings & BMP3_TEMP_OS_SEL) {
            /* Set the temperature over sampling settings in the
               register variable */
            reg_data[*len] = BMP3_SET_BITS(reg_data[0], BMP3_TEMP_OS, osr_settings.temp_os);
        }
        /* 0x1C is the register address of over sampling register */
        addr[*len] = BMP3_OSR_ADDR;
        (*len)++;
    }

    return;
}

void fill_odr_data(bmp388_t *dev, uint8_t *addr, uint8_t *reg_data, uint8_t *len) {
    struct bmp3_odr_filter_settings *osr_settings = &dev->params.settings.odr_filter;

    /* Limit the ODR to 0.001525879 Hz*/
    if (osr_settings->odr > BMP3_ODR_0_001_HZ)
        osr_settings->odr = BMP3_ODR_0_001_HZ;
    /* Set the odr settings in the register variable */
    reg_data[*len] = BMP3_SET_BITS_POS_0(reg_data[1], BMP3_ODR, osr_settings->odr);
    /* 0x1D is the register address of output data rate register */
    addr[*len] = 0x1D;
    (*len)++;

    return;
}

void fill_filter_data(bmp388_t *dev, uint8_t *addr, uint8_t *reg_data, uint8_t *len) {
    struct bmp3_odr_filter_settings osr_settings = dev->params.settings.odr_filter;

       /* Set the iir settings in the register variable */
    reg_data[*len] = BMP3_SET_BITS(reg_data[3], BMP3_IIR_FILTER, osr_settings.iir_filter);
       /* 0x1F is the register address of iir filter register */
    addr[*len] = 0x1F;
    (*len)++;

    return;
}

int8_t validate_osr_and_odr_settings(const bmp388_t *dev) {
    int8_t rslt;
    uint16_t meas_t = 0;
    /* Odr values in milli secs  */
    uint32_t odr[18] = {5, 10, 20, 40, 80, 160, 320, 640, 1280, 2560, 5120, 10240,
            20480, 40960, 81920, 163840, 327680, 655360};

    if (dev->params.settings.press_en) {
        /* Calculate the pressure measurement duration */
        meas_t = calculate_press_meas_time(dev);
    }
    if (dev->params.settings.temp_en) {
        /* Calculate the temperature measurement duration */
        meas_t += calculate_temp_meas_time(dev);
    }
    rslt = verify_meas_time_and_odr_duration(meas_t, odr[dev->params.settings.odr_filter.odr]);

    return rslt;
}

int8_t verify_meas_time_and_odr_duration(uint16_t meas_t, uint32_t odr_duration) {
    int8_t rslt;

    if (meas_t < odr_duration) {
        /* If measurement duration is less than odr duration
           then osr and odr settings are fine */
        rslt = BMP3_OK;
    } else {
        /* Osr and odr settings are not proper */
        rslt = BMP3_E_INVALID_ODR_OSR_SETTINGS;
    }

    return rslt;
}

uint16_t calculate_press_meas_time(const bmp388_t *dev) {
    uint16_t press_meas_t;
    struct bmp3_odr_filter_settings odr_filter = dev->params.settings.odr_filter;
#ifdef BMP3_DOUBLE_PRECISION_COMPENSATION
    double base = 2.0;
    double partial_out;
#else
    uint8_t base = 2;
    uint32_t partial_out;
#endif /* BMP3_DOUBLE_PRECISION_COMPENSATION */

    partial_out = bmp3_pow(base, odr_filter.press_os);
    press_meas_t = (uint16_t)(BMP3_PRESS_SETTLE_TIME + partial_out * BMP3_ADC_CONV_TIME);
    /* convert into mill seconds */
    press_meas_t = press_meas_t / 1000;

    return press_meas_t;
}

uint16_t calculate_temp_meas_time(const bmp388_t *dev) {
    uint16_t temp_meas_t;
    struct bmp3_odr_filter_settings odr_filter = dev->params.settings.odr_filter;
#ifdef BMP3_DOUBLE_PRECISION_COMPENSATION
    float base = 2.0;
    float partial_out;
#else
    uint8_t base = 2;
    uint32_t partial_out;
#endif /* BMP3_DOUBLE_PRECISION_COMPENSATION */

    partial_out = bmp3_pow(base, odr_filter.temp_os);
    temp_meas_t = (uint16_t)(BMP3_TEMP_SETTLE_TIME + partial_out * BMP3_ADC_CONV_TIME);
    /* convert into mill seconds */
    temp_meas_t = temp_meas_t / 1000;

    return temp_meas_t;
}

void parse_odr_filter_settings(const uint8_t *reg_data, struct bmp3_odr_filter_settings *settings) {
    uint8_t index = 0;

    /* Odr and filter settings index starts from one (0x1C register) */
    settings->press_os = BMP3_GET_BITS_POS_0(reg_data[index], BMP3_PRESS_OS);
    settings->temp_os = BMP3_GET_BITS(reg_data[index], BMP3_TEMP_OS);

    /* Move index to 0x1D register */
    index++;
    settings->odr = BMP3_GET_BITS_POS_0(reg_data[index], BMP3_ODR);

    /* Move index to 0x1F register */
    index = index + 2;
    settings->iir_filter = BMP3_GET_BITS(reg_data[index], BMP3_IIR_FILTER);

    return;
}

int get_odr_filter_settings(bmp388_t *dev) {
    uint8_t reg_data[4];

    i2c_acquire(DEV_I2C);

    if(i2c_read_regs(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, reg_data, 4, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    parse_odr_filter_settings(reg_data, &dev->params.settings.odr_filter);

    return BMP388_OK;
}

int validate_normal_mode_settings(bmp388_t *dev) {
    int retval;

    retval = get_odr_filter_settings(dev);

    if(retval == BMP388_OK) {
        retval = validate_osr_and_odr_settings(dev);
    }

    return retval;
}

int set_normal_mode(bmp388_t *dev) {
    int retval;
    uint8_t conf_err_status;

    retval = validate_normal_mode_settings(dev);
    if(retval != BMP388_OK) {
        return retval;
    }

    /* If osr and odr settings are proper then write the power mode */
    retval = write_power_mode(dev);
    if(retval != BMP388_OK) {
        return retval;
    }

    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_ERR_REG_ADDR, &conf_err_status, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    i2c_release(DEV_I2C);
    /* check for configuration error */
    /* Read the configuration error status */
    if(conf_err_status & BMP3_CONF_ERR) {
        /* Osr and odr configuration is
           not proper */
        return -BMP388_ERR_CONFIG;
    }

    return BMP388_OK;
}

int write_power_mode(const bmp388_t *dev) {
    uint8_t op_mode = dev->params.settings.op_mode;
    /* Temporary variable to store the value read from op-mode register */
    uint8_t op_mode_reg_val;


    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, &op_mode_reg_val, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, op_mode);

    /* Write the power mode in the register */
    if(i2c_write_reg(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, op_mode_reg_val, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_CMD;
    }

    i2c_release(DEV_I2C);

    return BMP388_OK;
}

int put_device_to_sleep(const bmp388_t *dev) {
    /* Temporary variable to store the value read from op-mode register */
    uint8_t op_mode_reg_val;


    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, &op_mode_reg_val, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    op_mode_reg_val = op_mode_reg_val & (~(BMP3_OP_MODE_MSK));

    /* Write the power mode in the register */
    if(i2c_write_reg(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, op_mode_reg_val, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_CMD;
    }

    i2c_release(DEV_I2C);

    return BMP388_OK;
}

int bmp388_set_op_mode(bmp388_t *dev) {
    int retval = 0;
    uint8_t last_set_mode;
    uint8_t curr_mode = dev->params.settings.op_mode;

    if(bmp388_get_op_mode(dev, &last_set_mode) != BMP388_OK) {
        return -BMP388_ERR_NODEV;
    }

    /* If the sensor is not in sleep mode put the device to sleep
       mode */
    if(last_set_mode != BMP3_SLEEP_MODE) {
        /* Device should be put to sleep before transiting to
           forced mode or normal mode */
        retval = put_device_to_sleep(dev);
        /* Give some time for device to go into sleep mode */
        xtimer_usleep((5U * US_PER_MS));
    }
    /* Set the power mode */
    if (curr_mode == BMP3_NORMAL_MODE) {
        /* Set normal mode and validate
           necessary settings */
        retval = set_normal_mode(dev);
    } else if (curr_mode == BMP3_FORCED_MODE) {
        /* Set forced mode */
        retval = write_power_mode(dev);
    }

    return retval;
}

int bmp388_get_op_mode(const bmp388_t *dev, uint8_t *op_mode) {
    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, BMP3_PWR_CTRL_ADDR, op_mode, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    *op_mode = BMP3_GET_BITS(*op_mode, BMP3_OP_MODE);

    return BMP388_OK;
}

int bmp388_get_sensor_data(bmp388_t *dev, struct bmp3_data *comp_data) {
    uint8_t sensor_comp = 0;
    uint8_t reg_data[BMP3_P_T_DATA_LEN] = {0};
    struct bmp3_uncomp_data uncomp_data = {0};

    sensor_comp |= BMP3_TEMP;
    sensor_comp |= BMP3_PRESS;

    i2c_acquire(DEV_I2C);

    if(i2c_read_regs(DEV_I2C, DEV_ADDR, BMP3_DATA_ADDR, &reg_data, BMP3_P_T_DATA_LEN, 0) != 0) {
        i2c_release(DEV_I2C);
        return -BMP388_ERR_NODEV;
    }

    parse_sensor_data(reg_data, &uncomp_data);
    compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->params.calib_data);

    i2c_release(DEV_I2C);

    return BMP388_OK;
}

void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data) {
    /* Temporary variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_xlsb = (uint32_t)reg_data[0];
    data_lsb = (uint32_t)reg_data[1] << 8;
    data_msb = (uint32_t)reg_data[2] << 16;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_xlsb = (uint32_t)reg_data[3];
    data_lsb = (uint32_t)reg_data[4] << 8;
    data_msb = (uint32_t)reg_data[5] << 16;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    return;
}

int8_t compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data, struct bmp3_data *comp_data, struct bmp3_calib_data *calib_data) {
    int8_t rslt = BMP3_OK;

    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL)) {
        /* If pressure or temperature component is selected */
        if (sensor_comp & (BMP3_PRESS | BMP3_TEMP)) {
            /* Compensate the temperature data */
            comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        }
        if (sensor_comp & BMP3_PRESS) {
            /* Compensate the pressure data */
            comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        }
    } else {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}


#ifdef BMP3_DOUBLE_PRECISION_COMPENSATION
void bmp388_parse_calib_data(bmp388_t *dev, const uint8_t *reg_data) {
    /* Temporary variable to store the aligned trim data */
    struct bmp3_reg_calib_data *reg_calib_data = &dev->params.calib_data.reg_calib_data;
    struct bmp3_quantized_calib_data *quantized_calib_data = &dev->params.calib_data.quantized_calib_data;

    /* Temporary variable */
    double temp_var;

    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    quantized_calib_data->par_t1 = ((double)reg_calib_data->par_t1 / temp_var);

    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
    temp_var = 1073741824.0f;
    quantized_calib_data->par_t2 = ((double)reg_calib_data->par_t2 / temp_var);

    reg_calib_data->par_t3 = (int8_t)reg_data[4];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_t3 = ((double)reg_calib_data->par_t3 / temp_var);

    reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
    temp_var = 1048576.0f;
    quantized_calib_data->par_p1 = ((double)(reg_calib_data->par_p1 - (16384)) / temp_var);

    reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
    temp_var = 536870912.0f;
    quantized_calib_data->par_p2 = ((double)(reg_calib_data->par_p2 - (16384)) / temp_var);

    reg_calib_data->par_p3 = (int8_t)reg_data[9];
    temp_var = 4294967296.0f;
    quantized_calib_data->par_p3 = ((double)reg_calib_data->par_p3 / temp_var);

    reg_calib_data->par_p4 = (int8_t)reg_data[10];
    temp_var = 137438953472.0f;
    quantized_calib_data->par_p4 = ((double)reg_calib_data->par_p4 / temp_var);

    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantized_calib_data->par_p5 = ((double)reg_calib_data->par_p5 / temp_var);

    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14],  reg_data[13]);
    temp_var = 64.0f;
    quantized_calib_data->par_p6 = ((double)reg_calib_data->par_p6 / temp_var);

    reg_calib_data->par_p7 = (int8_t)reg_data[15];
    temp_var = 256.0f;
    quantized_calib_data->par_p7 = ((double)reg_calib_data->par_p7 / temp_var);

    reg_calib_data->par_p8 = (int8_t)reg_data[16];
    temp_var = 32768.0f;
    quantized_calib_data->par_p8 = ((double)reg_calib_data->par_p8 / temp_var);

    reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p9 = ((double)reg_calib_data->par_p9 / temp_var);

    reg_calib_data->par_p10 = (int8_t)reg_data[19];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p10 = ((double)reg_calib_data->par_p10 / temp_var);

    reg_calib_data->par_p11 = (int8_t)reg_data[20];
    temp_var = 36893488147419103232.0f;
    quantized_calib_data->par_p11 = ((double)reg_calib_data->par_p11 / temp_var);

    return;
}

double compensate_temperature(const struct bmp3_uncomp_data *uncomp_data, struct bmp3_calib_data *calib_data)
{
    uint32_t uncomp_temp = uncomp_data->temperature;
    double partial_data1;
    double partial_data2;

    partial_data1 = (double)(uncomp_temp - calib_data->quantized_calib_data.par_t1);
    partial_data2 = (double)(partial_data1 * calib_data->quantized_calib_data.par_t2);
    /* Update the compensated temperature in calib structure since this is
       needed for pressure calculation */
    calib_data->quantized_calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1)
                            * calib_data->quantized_calib_data.par_t3;

    /* Returns compensated temperature */
    return calib_data->quantized_calib_data.t_lin;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
double compensate_pressure(const struct bmp3_uncomp_data *uncomp_data, const struct bmp3_calib_data *calib_data) {
    const struct bmp3_quantized_calib_data *quantized_calib_data = &calib_data->quantized_calib_data;
    /* Variable to store the compensated pressure */
    double comp_press;
    /* Temporary variables used for compensation */
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    partial_data1 = quantized_calib_data->par_p6 * quantized_calib_data->t_lin;
    partial_data2 = quantized_calib_data->par_p7 * bmp3_pow(quantized_calib_data->t_lin, 2);
    partial_data3 = quantized_calib_data->par_p8 * bmp3_pow(quantized_calib_data->t_lin, 3);
    partial_out1 = quantized_calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = quantized_calib_data->par_p2 * quantized_calib_data->t_lin;
    partial_data2 = quantized_calib_data->par_p3 * bmp3_pow(quantized_calib_data->t_lin, 2);
    partial_data3 = quantized_calib_data->par_p4 * bmp3_pow(quantized_calib_data->t_lin, 3);
    partial_out2 = uncomp_data->pressure *
            (quantized_calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = bmp3_pow((double)uncomp_data->pressure, 2);
    partial_data2 = quantized_calib_data->par_p9 + quantized_calib_data->par_p10 * quantized_calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + bmp3_pow((double)uncomp_data->pressure, 3) * quantized_calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}

double bmp3_pow(double base, uint8_t power) {
    double pow_output = 1;

    while (power != 0) {
        pow_output = base * pow_output;
        power--;
    }

    return pow_output;
}

#else // ELSE ELSE ELSE ELSE ELSE ELSE ELSE ELSE ELSE ELSE ELSE

void bmp388_parse_calib_data(bmp388_t *dev, const uint8_t *reg_data) {
    struct bmp3_reg_calib_data *reg_calib_data = &dev->params.calib_data.reg_calib_data;

    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
    reg_calib_data->par_t3 = (int8_t)reg_data[4];
    reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
    reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
    reg_calib_data->par_p3 = (int8_t)reg_data[9];
    reg_calib_data->par_p4 = (int8_t)reg_data[10];
    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14],  reg_data[13]);
    reg_calib_data->par_p7 = (int8_t)reg_data[15];
    reg_calib_data->par_p8 = (int8_t)reg_data[16];
    reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
    reg_calib_data->par_p10 = (int8_t)reg_data[19];
    reg_calib_data->par_p11 = (int8_t)reg_data[20];

    return;
}

int64_t compensate_temperature(const struct bmp3_uncomp_data *uncomp_data, struct bmp3_calib_data *calib_data) {
    uint64_t partial_data1;
    uint64_t partial_data2;
    uint64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t comp_temp;

    partial_data1 = uncomp_data->temperature - (256 * calib_data->reg_calib_data.par_t1);
    partial_data2 = calib_data->reg_calib_data.par_t2 * partial_data1;
    partial_data3 = partial_data1 * partial_data1;
    partial_data4 = (int64_t)partial_data3 * calib_data->reg_calib_data.par_t3;
    partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
    partial_data6 = partial_data5 / 4294967296;
    /* Store t_lin in dev. structure for pressure calculation */
    calib_data->reg_calib_data.t_lin = partial_data6;
    comp_temp = (int64_t)((partial_data6 * 25)  / 16384);

    return comp_temp;
}

uint64_t compensate_pressure(const struct bmp3_uncomp_data *uncomp_data, const struct bmp3_calib_data *calib_data) {
    const struct bmp3_reg_calib_data *reg_calib_data = &calib_data->reg_calib_data;
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    partial_data1 = reg_calib_data->t_lin * reg_calib_data->t_lin;
    partial_data2 = partial_data1 / 64;
    partial_data3 = (partial_data2 * reg_calib_data->t_lin) / 256;
    partial_data4 = (reg_calib_data->par_p8 * partial_data3) / 32;
    partial_data5 = (reg_calib_data->par_p7 * partial_data1) * 16;
    partial_data6 = (reg_calib_data->par_p6 * reg_calib_data->t_lin) * 4194304;
    offset = (reg_calib_data->par_p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;

    partial_data2 = (reg_calib_data->par_p4 * partial_data3) / 32;
    partial_data4 = (reg_calib_data->par_p3 * partial_data1) * 4;
    partial_data5 = (reg_calib_data->par_p2 - 16384) * reg_calib_data->t_lin * 2097152;
    sensitivity = ((reg_calib_data->par_p1 - 16384) * 70368744177664) + partial_data2 + partial_data4
            + partial_data5;

    partial_data1 = (sensitivity / 16777216) * uncomp_data->pressure;
    partial_data2 = reg_calib_data->par_p10 * reg_calib_data->t_lin;
    partial_data3 = partial_data2 + (65536 * reg_calib_data->par_p9);
    partial_data4 = (partial_data3 * uncomp_data->pressure) / 8192;
    partial_data5 = (partial_data4 * uncomp_data->pressure) / 512;
    partial_data6 = (int64_t)((uint64_t)uncomp_data->pressure * (uint64_t)uncomp_data->pressure);
    partial_data2 = (reg_calib_data->par_p11 * partial_data6) / 65536;
    partial_data3 = (partial_data2 * uncomp_data->pressure) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

    return comp_press;
}

uint32_t bmp3_pow(uint8_t base, uint8_t power) {
    uint32_t pow_output = 1;

    while (power != 0) {
        pow_output = base * pow_output;
        power--;
    }

    return pow_output;
}

#endif
