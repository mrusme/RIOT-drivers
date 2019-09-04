/*
 * Copyright (C) 2019 Marius <marius@twostairs.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_tsl2591
 * @{
 *
 * @file
 * @brief       Device driver implementation for the TSL2591 very-high sensitivity light-to-digital converter.
 *
 * @author      Marius <marius@twostairs.com>
 *
 * @}
 */

#include <math.h>

#include "tsl2591.h"

#include "xtimer.h"
#include "periph_conf.h"
#include "periph/i2c.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define DEV_I2C      (dev->params.i2c_dev)
#define DEV_ADDR     (dev->params.i2c_addr)

int tsl2591_init(tsl2591_t *dev, const tsl2591_params_t *params) {
    dev->params = *params;
    dev->params.initialized = false;
    dev->params.enabled = false;

    if(dev->params.initialized) {
        return -TSL2591_ERR_ALRDYINIT;
    }

    i2c_acquire(DEV_I2C);

    uint8_t id;
    if(i2c_read_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_ID), &id, 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    if(id != 0x50) {
        return -TSL2591_ERR_NODEV;
    }

    dev->params.initialized = true;

    int retval;

    retval = tsl2591_enable(dev);
    if(retval != TSL2591_OK) {
        return retval;
    }

    // retval = tsl2591_set_integration_time(dev, dev->params.integration);
    // if(retval != TSL2591_OK) {
    //     return retval;
    // }

    // retval = tsl2591_set_gain(dev, dev->params.gain);
    // if(retval != TSL2591_OK) {
    //     return retval;
    // }

    return TSL2591_OK;
}

int tsl2591_enable(tsl2591_t *dev) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    if(dev->params.enabled == true) {
        return TSL2591_OK;
    }

    i2c_acquire(DEV_I2C);

    // if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE), (TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN | TSL2591_ENABLE_NPIEN), 0) != 0) {
    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE), (0x13), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_ENABLE;
    }

    i2c_release(DEV_I2C);

    dev->params.enabled = true;

    return TSL2591_OK;
}

int tsl2591_disable(tsl2591_t *dev) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    if(dev->params.enabled == false) {
        return TSL2591_OK;
    }

    i2c_acquire(DEV_I2C);

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE), (TSL2591_ENABLE_POWEROFF), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_ENABLE;
    }

    i2c_release(DEV_I2C);

    dev->params.enabled = false;

    return TSL2591_OK;
}

int tsl2591_set_gain(tsl2591_t *dev, tsl2591_gain_t gain) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    // tsl2591_enable(dev);
    dev->params.gain = gain;

    i2c_acquire(DEV_I2C);

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL), (dev->params.integration | dev->params.gain), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_ENABLE;
    }

    i2c_release(DEV_I2C);

    // tsl2591_disable(dev);

    return TSL2591_OK;
}

tsl2591_gain_t tsl2591_get_gain(tsl2591_t *dev) {
    return dev->params.gain;
}

int tsl2591_set_integration_time(tsl2591_t *dev, tsl2591_integration_time_t integration) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    // tsl2591_enable(dev);
    dev->params.integration = integration;

    i2c_acquire(DEV_I2C);

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL), (dev->params.integration | dev->params.gain), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_ENABLE;
    }

    i2c_release(DEV_I2C);

    // tsl2591_disable(dev);

    return TSL2591_OK;
}

tsl2591_integration_time_t tsl2591_get_integration_time(tsl2591_t *dev) {
    return dev->params.integration;
}

float tsl2591_calculate_lux(tsl2591_t *dev, uint16_t ch0, uint16_t ch1) {
    float    atime = 0.0, again = 0.0;
    float    cpl = 0.0, lux = 0.0;

    // Check for overflow conditions first
    if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF))
    {
        // Signal an overflow
        return -1;
    }

    // Note: This algorithm is based on preliminary coefficients
    // provided by AMS and may need to be updated in the future

    atime = (float)(dev->params.integration + 1) * 100;

    // switch (dev->params.integration)
    // {
    //     case TSL2591_INTEGRATIONTIME_100MS :
    //         atime = 100.0F;
    //         break;
    //     case TSL2591_INTEGRATIONTIME_200MS :
    //         atime = 200.0F;
    //         break;
    //     case TSL2591_INTEGRATIONTIME_300MS :
    //         atime = 300.0F;
    //         break;
    //     case TSL2591_INTEGRATIONTIME_400MS :
    //         atime = 400.0F;
    //         break;
    //     case TSL2591_INTEGRATIONTIME_500MS :
    //         atime = 500.0F;
    //         break;
    //     case TSL2591_INTEGRATIONTIME_600MS :
    //         atime = 600.0F;
    //         break;
    //     default: // 100ms
    //         atime = 100.0F;
    //         break;
    // }

    switch (dev->params.gain)
    {
        case TSL2591_GAIN_LOW :
            again = 1.0F;
            break;
        case TSL2591_GAIN_MED :
            again = 25.0F;
            break;
        case TSL2591_GAIN_HIGH :
            again = 428.0F;
            break;
        case TSL2591_GAIN_MAX :
            again = 9876.0F;
            break;
        default:
            again = 1.0F;
            break;
    }

    // cpl = (ATIME * AGAIN) / DF
    cpl = (atime * again) / TSL2591_LUX_DF;

    // Original lux calculation (for reference sake)
    //lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
    //lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD * (float)ch1 ) ) / cpl;
    //lux = lux1 > lux2 ? lux1 : lux2;

    // Alternate lux calculation 1
    // See: https://github.com/adafruit/Adafruit_TSL2591_Library/issues/14
    lux = ( ((float)ch0 - (float)ch1 )) * (1.0F - ((float)ch1/(float)ch0) ) / cpl;

    // Alternate lux calculation 2
    //lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;

    // Signal I2C had no errors
    return lux;
}

uint32_t tsl2591_get_full_luminosity(tsl2591_t *dev) {
    uint32_t x;
    uint16_t y;
    uint8_t chan0[2];
    uint8_t chan1[2];

    if(!dev->params.initialized) {
        return 0;
    }

    // tsl2591_enable(dev);

    for (uint8_t d=0; d <= dev->params.integration; d++)
    {
        xtimer_usleep((200LU * US_PER_MS));
    }

    i2c_acquire(DEV_I2C);

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW), &chan0[0], 0) != 0) {
        i2c_release(DEV_I2C);
        return 0;
    }

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_HIGH), &chan0[1], 0) != 0) {
        i2c_release(DEV_I2C);
        return 0;
    }

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW), &chan1[0], 0) != 0) {
        i2c_release(DEV_I2C);
        return 0;
    }

    if(i2c_read_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_HIGH), &chan1[1], 0) != 0) {
        i2c_release(DEV_I2C);
        return 0;
    }

    i2c_release(DEV_I2C);

    // tsl2591_disable(dev);

    y = ((uint16_t)chan0[0] & 0xff) | ((uint16_t)chan0[1] << 8);
    x = ((uint16_t)chan1[0] & 0xff) | ((uint16_t)chan1[1] << 8);

    x <<= 16;
    x |= y;

    return x;
}

uint16_t tsl2591_get_luminosity(tsl2591_t *dev, uint8_t channel) {
    uint32_t x = tsl2591_get_full_luminosity(dev);

    if (channel == TSL2591_FULLSPECTRUM)
    {
        // Reads two byte value from channel 0 (visible + infrared)
        return (x & 0xFFFF);
    }
    else if (channel == TSL2591_INFRARED)
    {
        // Reads two byte value from channel 1 (infrared)
        return (x >> 16);
    }
    else if (channel == TSL2591_VISIBLE)
    {
        // Reads all and subtracts out just the visible!
        return ( (x & 0xFFFF) - (x >> 16));
    }

    // unknown channel!
    return 0;
}

/************************************************************************/
/*!
        @brief  Set up the interrupt to go off when light level is outside the lower/upper range.
        @param  lowerThreshold Raw light data reading level that is the lower value threshold for interrupt
        @param  upperThreshold Raw light data reading level that is the higher value threshold for interrupt
        @param  persist How many counts we must be outside range for interrupt to fire, default is any single value
*/
/**************************************************************************/
int tsl2591_register_interrupt(tsl2591_t *dev, uint16_t lowerThreshold, uint16_t upperThreshold, tsl2591_persist_t persist) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    // tsl2591_enable(dev);

    i2c_acquire(DEV_I2C);

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_PERSIST_FILTER), persist, 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_REGINT;
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTL), lowByte(lowerThreshold), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_REGINT;
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTH), highByte(lowerThreshold), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_REGINT;
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTL), lowByte(upperThreshold), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_REGINT;
    }

    if(i2c_write_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTH), highByte(upperThreshold), 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_REGINT;
    }

    i2c_release(DEV_I2C);

    // tsl2591_disable(dev);

    return TSL2591_OK;
}

int tsl2591_clear_interrupt(tsl2591_t *dev) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    // tsl2591_enable(dev);

    i2c_acquire(DEV_I2C);

    if(i2c_write_byte(DEV_I2C, DEV_ADDR, TSL2591_CLEAR_INT, 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_CLRINT;
    }

    i2c_release(DEV_I2C);

    // tsl2591_disable(dev);

    return TSL2591_OK;
}

int tsl2591_test_interrupt(tsl2591_t *dev) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    // tsl2591_enable(dev);

    i2c_acquire(DEV_I2C);

    if(i2c_write_byte(DEV_I2C, DEV_ADDR, TSL2591_TEST_INT, 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_CLRINT;
    }

    i2c_release(DEV_I2C);

    // tsl2591_disable(dev);

    return TSL2591_OK;
}

int tsl2591_get_status(tsl2591_t *dev) {
    if(!dev->params.initialized) {
        return -TSL2591_ERR_NOTINIT;
    }

    // tsl2591_enable(dev);

    i2c_acquire(DEV_I2C);

    uint8_t x;
    if(i2c_read_reg(DEV_I2C, DEV_ADDR, (TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_STATUS), &x, 0) != 0) {
        i2c_release(DEV_I2C);
        return -TSL2591_ERR_STATUS;
    }

    i2c_release(DEV_I2C);

    // tsl2591_disable(dev);

    return x;
}
