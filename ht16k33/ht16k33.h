/*
 * Copyright (C) 2019 Marius <marius@twostairs.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ht16k33 HT16K33
 * @ingroup     drivers_ht16k33
 * @brief       Device driver interface for the HT16K33
 * @{
 *
 * @file
 * @brief       Device driver interface for the HT16K33
 *
 * @author      Marius <marius@twostairs.com>
 *
 * @}
 */

#ifndef HT16K33_H
#define HT16K33_H

#include <stdio.h>

#include "periph/i2c.h"

#define HT16K33_ADDR (0x70)

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

#ifndef HT16K33_PARAM_I2C_DEV
#define HT16K33_PARAM_I2C_DEV         I2C_DEV(0)
#endif
#ifndef HT16K33_PARAM_I2C_ADDR
#define HT16K33_PARAM_I2C_ADDR        HT16K33_ADDR
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Device initialization parameters
 */
typedef struct {
    i2c_t i2c_dev;                              /**< I2C device which is used */
    uint8_t i2c_addr;                           /**< I2C address */
} ht16k33_params_t;

/**
 * @brief   Device descriptor for the HT16K33
 */
typedef struct {
    ht16k33_params_t      params;               /**< Device initialization parameters */
} ht16k33_t;

/**
 * @brief   Status and error return codes
 */
enum {
    HT16K33_OK = 0,                              /**< everything was fine */
    HT16K33_ERR_NODEV,                           /**< did not detect device */
    HT16K33_ERROR_BUS,                            /**< error on the bus */
    HT16K33_ERR_NOSPACE,                         /**< not enough space on display */
};

static const uint8_t numbertable[] = {
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
    0x77, /* a */
    0x7C, /* b */
    0x39, /* C */
    0x5E, /* d */
    0x79, /* E */
    0x71, /* F */
};

int ht16k33_init(ht16k33_t *dev, const ht16k33_params_t *params);
int ht16k33_display_decimal(ht16k33_t *dev, uint16_t number);
int ht16k33_display_raw(ht16k33_t *dev, const uint16_t *data);
int ht16k33_on(ht16k33_t *dev);
int ht16k33_off(ht16k33_t *dev);

#ifdef __cplusplus
}
#endif

#endif
