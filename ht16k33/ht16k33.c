/*
 * Copyright (C) 2019 Marius <marius@twostairs.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ht16k33
 * @{
 *
 * @file
 * @brief       Device driver implementation for the HT16K33.
 *
 * @author      Marius <marius@twostairs.com>
 *
 * @}
 */

#include <math.h>

#include "ht16k33.h"

#include "xtimer.h"
#include "periph_conf.h"
#include "periph/i2c.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define DEV_I2C      (dev->params.i2c_dev)
#define DEV_ADDR     (dev->params.i2c_addr)

int ht16k33_init(ht16k33_t *dev, const ht16k33_params_t *params) {
    dev->params = *params;

    int turnon = ht16k33_on(dev);
    if(turnon != HT16K33_OK) {
        return turnon;
    }

    i2c_acquire(DEV_I2C);

    if(i2c_write_byte(DEV_I2C, DEV_ADDR, (HT16K33_CMD_BRIGHTNESS | 15), 0) < 0) {
        i2c_release(DEV_I2C);
        return -HT16K33_ERROR_BUS;
    }

    if(i2c_write_byte(DEV_I2C, DEV_ADDR, (HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1)), 0) < 0) {
        i2c_release(DEV_I2C);
        return -HT16K33_ERROR_BUS;
    }

    i2c_release(DEV_I2C);

    return HT16K33_OK;
}


int ht16k33_display_decimal(ht16k33_t *dev, uint16_t number) {
    uint16_t numbuff = number;
    uint8_t posval;
    int_fast8_t numlen, i;
    uint16_t buff[5] = {
        (0x3F),
        (0x3F),
        (0x00),
        (0x3F),
        (0x3F)
    };

    if(number > 9999) {
        return -HT16K33_ERR_NOSPACE;
    }

    numlen = floor(log10(fabs(number))) + 1;

    for(i = 4; i >= (4 - numlen); i--) {
        if(i == 2) {
            continue;
        }

        posval = numbuff % 10;
        buff[i] = numbertable[posval];
        numbuff /= 10;
    }

    return ht16k33_display_raw(dev, buff);
}

int ht16k33_display_raw(ht16k33_t *dev, const uint16_t *data) {
    uint8_t buff[10] = {
        (data[0] & 0xFF), (data[0] >> 8),
        (data[1] & 0xFF), (data[1] >> 8),
        (data[2] & 0xFF), (data[2] >> 8),
        (data[3] & 0xFF), (data[3] >> 8),
        (data[4] & 0xFF), (data[4] >> 8)
    };

    i2c_acquire(DEV_I2C);

    if(i2c_write_regs(DEV_I2C, DEV_ADDR, 0x00, &buff, 10, 0) < 0) {
        i2c_release(DEV_I2C);
        return -HT16K33_ERROR_BUS;
    }

    i2c_release(DEV_I2C);

    return HT16K33_OK;
}

int ht16k33_on(ht16k33_t *dev) {
    i2c_acquire(DEV_I2C);

    if(i2c_write_byte(DEV_I2C, DEV_ADDR, 0x21, 0) < 0) {
        i2c_release(DEV_I2C);
        return -HT16K33_ERROR_BUS;
    }

    i2c_release(DEV_I2C);
    return HT16K33_OK;
}

int ht16k33_off(ht16k33_t *dev) {
    i2c_acquire(DEV_I2C);

    if(i2c_write_byte(DEV_I2C, DEV_ADDR, 0x20, 0) < 0) {
        i2c_release(DEV_I2C);
        return -HT16K33_ERROR_BUS;
    }

    i2c_release(DEV_I2C);
    return HT16K33_OK;
}
