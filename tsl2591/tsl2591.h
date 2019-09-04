/*
 * Copyright (C) 2019 Marius <marius@twostairs.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_tsl2591 TSL2591 very-high sensitivity light-to-digital converter
 * @ingroup     drivers_tsl2591
 * @brief       Device driver interface for the TSL2591 very-high sensitivity light-to-digital converter.
 * @{
 *
 * @file
 * @brief       Device driver interface for the TSL2591 very-high sensitivity light-to-digital converter.
 *
 * @author      Marius <marius@twostairs.com>
 *
 * @}
 */

#ifndef TSL2591_H
#define TSL2591_H

#include <stdio.h>
#include "xtimer.h"
#include "periph/i2c.h"

#define TSL2591_VISIBLE           (2)       ///< (channel 0) - (channel 1)
#define TSL2591_INFRARED          (1)       ///< channel 1
#define TSL2591_FULLSPECTRUM      (0)       ///< channel 0

#define TSL2591_ADDR (0x29)

#define TSL2591_COMMAND_BIT       (0xA0)    ///< 1010 0000: bits 7 and 5 for 'command normal'

#define TSL2591_CLEAR_INT         (0xE7)
///! Special Function Command for "Interrupt set - forces an interrupt"
#define TSL2591_TEST_INT          (0xE4)


#define TSL2591_WORD_BIT          (0x20)    ///< 1 = read/write word (rather than byte)
#define TSL2591_BLOCK_BIT         (0x10)    ///< 1 = using block read/write

#define TSL2591_ENABLE_POWEROFF   (0x00)    ///< Flag for ENABLE register to disable
#define TSL2591_ENABLE_POWERON    (0x01)    ///< Flag for ENABLE register to enable
#define TSL2591_ENABLE_AEN        (0x02)    ///< ALS Enable. This field activates ALS function. Writing a one activates the ALS. Writing a zero disables the ALS.
#define TSL2591_ENABLE_AIEN       (0x10)    ///< ALS Interrupt Enable. When asserted permits ALS interrupts to be generated, subject to the persist filter.
#define TSL2591_ENABLE_NPIEN      (0x80)    ///< No Persist Interrupt Enable. When asserted NP Threshold conditions will generate an interrupt, bypassing the persist filter

#define TSL2591_LUX_DF            (408.0F) ///< Lux cooefficient
#define TSL2591_LUX_COEFB         (1.64F)  ///< CH0 coefficient
#define TSL2591_LUX_COEFC         (0.59F)  ///< CH1 coefficient A
#define TSL2591_LUX_COEFD         (0.86F)  ///< CH2 coefficient B

#ifndef TSL2591_PARAM_I2C_DEV
#define TSL2591_PARAM_I2C_DEV         I2C_DEV(0)
#endif
#ifndef TSL2591_PARAM_I2C_ADDR
#define TSL2591_PARAM_I2C_ADDR        TSL2591_ADDR
#endif

#ifdef __cplusplus
extern "C" {
#endif

/// TSL2591 Register map
enum
{
    TSL2591_REGISTER_ENABLE             = 0x00, // Enable register
    TSL2591_REGISTER_CONTROL            = 0x01, // Control register
    TSL2591_REGISTER_THRESHOLD_AILTL    = 0x04, // ALS low threshold lower byte
    TSL2591_REGISTER_THRESHOLD_AILTH    = 0x05, // ALS low threshold upper byte
    TSL2591_REGISTER_THRESHOLD_AIHTL    = 0x06, // ALS high threshold lower byte
    TSL2591_REGISTER_THRESHOLD_AIHTH    = 0x07, // ALS high threshold upper byte
    TSL2591_REGISTER_THRESHOLD_NPAILTL  = 0x08, // No Persist ALS low threshold lower byte
    TSL2591_REGISTER_THRESHOLD_NPAILTH  = 0x09, // No Persist ALS low threshold higher byte
    TSL2591_REGISTER_THRESHOLD_NPAIHTL  = 0x0A, // No Persist ALS high threshold lower byte
    TSL2591_REGISTER_THRESHOLD_NPAIHTH  = 0x0B, // No Persist ALS high threshold higher byte
    TSL2591_REGISTER_PERSIST_FILTER     = 0x0C, // Interrupt persistence filter
    TSL2591_REGISTER_PACKAGE_PID        = 0x11, // Package Identification
    TSL2591_REGISTER_DEVICE_ID          = 0x12, // Device Identification
    TSL2591_REGISTER_DEVICE_STATUS      = 0x13, // Internal Status
    TSL2591_REGISTER_CHAN0_LOW          = 0x14, // Channel 0 data, low byte
    TSL2591_REGISTER_CHAN0_HIGH         = 0x15, // Channel 0 data, high byte
    TSL2591_REGISTER_CHAN1_LOW          = 0x16, // Channel 1 data, low byte
    TSL2591_REGISTER_CHAN1_HIGH         = 0x17, // Channel 1 data, high byte
};

/// Enumeration for the sensor integration timing
typedef enum
{
    TSL2591_INTEGRATIONTIME_100MS     = 0x00,  // 100 millis
    TSL2591_INTEGRATIONTIME_200MS     = 0x01,  // 200 millis
    TSL2591_INTEGRATIONTIME_300MS     = 0x02,  // 300 millis
    TSL2591_INTEGRATIONTIME_400MS     = 0x03,  // 400 millis
    TSL2591_INTEGRATIONTIME_500MS     = 0x04,  // 500 millis
    TSL2591_INTEGRATIONTIME_600MS     = 0x05,  // 600 millis
}
tsl2591_integration_time_t;

/// Enumeration for the persistance filter (for interrupts)
typedef enum
{
    //  bit 7:4: 0
    TSL2591_PERSIST_EVERY             = 0x00, // Every ALS cycle generates an interrupt
    TSL2591_PERSIST_ANY               = 0x01, // Any value outside of threshold range
    TSL2591_PERSIST_2                 = 0x02, // 2 consecutive values out of range
    TSL2591_PERSIST_3                 = 0x03, // 3 consecutive values out of range
    TSL2591_PERSIST_5                 = 0x04, // 5 consecutive values out of range
    TSL2591_PERSIST_10                = 0x05, // 10 consecutive values out of range
    TSL2591_PERSIST_15                = 0x06, // 15 consecutive values out of range
    TSL2591_PERSIST_20                = 0x07, // 20 consecutive values out of range
    TSL2591_PERSIST_25                = 0x08, // 25 consecutive values out of range
    TSL2591_PERSIST_30                = 0x09, // 30 consecutive values out of range
    TSL2591_PERSIST_35                = 0x0A, // 35 consecutive values out of range
    TSL2591_PERSIST_40                = 0x0B, // 40 consecutive values out of range
    TSL2591_PERSIST_45                = 0x0C, // 45 consecutive values out of range
    TSL2591_PERSIST_50                = 0x0D, // 50 consecutive values out of range
    TSL2591_PERSIST_55                = 0x0E, // 55 consecutive values out of range
    TSL2591_PERSIST_60                = 0x0F, // 60 consecutive values out of range
}
tsl2591_persist_t;

/// Enumeration for the sensor gain
typedef enum
{
    TSL2591_GAIN_LOW                  = 0x00,    /// low gain (1x)
    TSL2591_GAIN_MED                  = 0x10,    /// medium gain (25x)
    TSL2591_GAIN_HIGH                 = 0x20,    /// medium gain (428x)
    TSL2591_GAIN_MAX                  = 0x30,    /// max gain (9876x)
}
tsl2591_gain_t;


/**
 * @brief   Device initialization parameters
 */
typedef struct {
    i2c_t i2c_dev;                              /**< I2C device which is used */
    uint8_t i2c_addr;                           /**< I2C address */
    bool initialized;                           /**< Initialization status */
    bool enabled;                               /**< Enabled status */
    tsl2591_gain_t gain;                        /**< Gain */
    tsl2591_integration_time_t integration;        /**< Integration time */
} tsl2591_params_t;

/**
 * @brief   Device descriptor for the TSL2591
 */
typedef struct {
    tsl2591_params_t      params;               /**< Device initialization parameters */
} tsl2591_t;

/**
 * @brief   Status and error return codes
 */
enum {
    TSL2591_OK = 0,                              /**< everything was fine */
    TSL2591_ERR_NODEV,                           /**< did not detect device */
    TSL2591_ERR_NOTINIT,                         /**< error when sensor was not initialized correctly */
    TSL2591_ERR_ALRDYINIT,                       /**< error when sensor was already initialized */
    TSL2591_ERR_ENABLE,                          /**< error when sensor could not be enabled */
    TSL2591_ERR_REGINT,                          /**< error registering interrupt */
    TSL2591_ERR_CLRINT,                          /**< error clearing interrupt */
    TSL2591_ERR_STATUS,                          /**< error regarding sensor status */
};

int tsl2591_init(tsl2591_t *dev, const tsl2591_params_t *params);
int tsl2591_enable(tsl2591_t *dev);
int tsl2591_disable(tsl2591_t *dev);
int tsl2591_set_gain(tsl2591_t *dev, tsl2591_gain_t gain);
tsl2591_gain_t tsl2591_get_gain(tsl2591_t *dev);
int tsl2591_set_integration_time(tsl2591_t *dev, tsl2591_integration_time_t integration);
tsl2591_integration_time_t tsl2591_get_integration_time(tsl2591_t *dev);
float tsl2591_calculate_lux(tsl2591_t *dev, uint16_t ch0, uint16_t ch1);
uint32_t tsl2591_get_full_luminosity(tsl2591_t *dev);
uint16_t tsl2591_get_luminosity(tsl2591_t *dev, uint8_t channel);
int tsl2591_register_interrupt(tsl2591_t *dev, uint16_t lowerThreshold, uint16_t upperThreshold, tsl2591_persist_t persist /*= TSL2591_PERSIST_ANY */);
int tsl2591_clear_interrupt(tsl2591_t *dev);
int tsl2591_test_interrupt(tsl2591_t *dev);
int tsl2591_get_status(tsl2591_t *dev);

#ifdef __cplusplus
}
#endif

#endif
