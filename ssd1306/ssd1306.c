/*
 * Copyright (C) 2019 Marius <marius@twostairs.com>
 * Copyright (c) 2012, Adafruit Industries
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ssd1306
 * @{
 *
 * @file
 * @brief       Device driver implementation for the SSD1306 OLED display.
 * This driver is a re-implementation of the Adafruit SSD1306 driver and
 * GFX Library, combining both and adapting all code to the RIOT driver
 * standards. Large parts of the logic code are copy & paste from Adafruit's
 * libraries.
 *
 * @author      Marius <marius@twostairs.com>
 *
 * @}
 */

#include <math.h>

#define SSD1306_128_32
#include "ssd1306.h"

#include "xtimer.h"
#include "periph_conf.h"
#include "periph/i2c.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }

#define DEV_I2C      (dev->params.i2c_dev)
#define DEV_ADDR     (dev->params.i2c_addr)

uint8_t Buffer_CMD[] = {SSD1306_CMD, 0x00};
uint8_t Buffer_DATA[129] = {SSD1306_DATA, };

static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

int ssd1306_init(ssd1306_t *dev, const ssd1306_params_t *params) {
    int retval = 0;

    dev->params = *params;
    dev->params.cursor_x = 0;
    dev->params.cursor_y = 0;

    retval = ssd1306_command(dev, SSD1306_DISPLAYOFF);                    // 0xAE
    if(retval != SSD1306_OK) {
        return retval;
    }
    retval = ssd1306_command(dev, SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    if(retval != SSD1306_OK) {
        return retval;
    }
    retval = ssd1306_command(dev, 0x80);                                  // the suggested ratio 0x80
    if(retval != SSD1306_OK) {
        return retval;
    }


    retval = ssd1306_command(dev, SSD1306_SETMULTIPLEX);                  // 0xA8
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, dev->params.screen_height - 1);
    if(retval != SSD1306_OK) {
        return retval;
    }


    retval = ssd1306_command(dev, SSD1306_SETDISPLAYOFFSET);              // 0xD3
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0x0);                                   // no offset
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_SETSTARTLINE | 0x0);            // line #0
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_CHARGEPUMP);                    // 0x8D
    if(retval != SSD1306_OK) {
        return retval;
    }

    if (dev->params.vccstate == SSD1306_EXTERNALVCC) {
        retval = ssd1306_command(dev, 0x10);
        if(retval != SSD1306_OK) {
            return retval;
        }

    } else {
        retval = ssd1306_command(dev, 0x14);
        if(retval != SSD1306_OK) {
            return retval;
        }

    }

    retval = ssd1306_command(dev, SSD1306_MEMORYMODE);                    // 0x20
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0x00);                                  // 0x0 act like ks0108
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_SEGREMAP | 0x1);
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_COMSCANDEC);
    if(retval != SSD1306_OK) {
        return retval;
    }


#if defined SSD1306_128_32
    retval = ssd1306_command(dev, SSD1306_SETCOMPINS);                    // 0xDA
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0x02);
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_SETCONTRAST);                   // 0x81
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0x8F);
    if(retval != SSD1306_OK) {
        return retval;
    }


#elif defined SSD1306_128_64
    retval = ssd1306_command(dev, SSD1306_SETCOMPINS);                    // 0xDA
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0x12);
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_SETCONTRAST);                   // 0x81
    if(retval != SSD1306_OK) {
        return retval;
    }

    if (dev->params.vccstate == SSD1306_EXTERNALVCC) {
        retval = ssd1306_command(dev, 0x9F);
        if(retval != SSD1306_OK) {
            return retval;
        }
    } else {
        retval = ssd1306_command(dev, 0xCF);
        if(retval != SSD1306_OK) {
            return retval;
        }
    }

#elif defined SSD1306_96_16
    retval = ssd1306_command(dev, SSD1306_SETCOMPINS);                    // 0xDA
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0x2);   //ada x12
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_SETCONTRAST);                   // 0x81
    if(retval != SSD1306_OK) {
        return retval;
    }

    if (dev->params.vccstate == SSD1306_EXTERNALVCC) {
        retval = ssd1306_command(dev, 0x10);
        if(retval != SSD1306_OK) {
            return retval;
        }
    } else {
        retval = ssd1306_command(dev, 0xAF);
        if(retval != SSD1306_OK) {
            return retval;
        }
    }

#endif

    retval = ssd1306_command(dev, SSD1306_SETPRECHARGE);                  // 0xd9
    if(retval != SSD1306_OK) {
        return retval;
    }

    if (dev->params.vccstate == SSD1306_EXTERNALVCC) {
        retval = ssd1306_command(dev, 0x22);
        if(retval != SSD1306_OK) {
            return retval;
        }
    } else {
        retval = ssd1306_command(dev, 0xF1);
        if(retval != SSD1306_OK) {
            return retval;
        }
    }
    retval = ssd1306_command(dev, SSD1306_SETVCOMDETECT);                 // 0xDB
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0x40);
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_NORMALDISPLAY);                 // 0xA6
    if(retval != SSD1306_OK) {
        return retval;
    }


    retval = ssd1306_command(dev, SSD1306_DEACTIVATE_SCROLL);
    if(retval != SSD1306_OK) {
        return retval;
    }


    retval = ssd1306_command(dev, SSD1306_DISPLAYON);//--turn on oled panel
    if(retval != SSD1306_OK) {
        return retval;
    }


    return SSD1306_OK;
}

int ssd1306_display(ssd1306_t *dev) {
    int retval = 0;

    retval = ssd1306_command(dev, SSD1306_COLUMNADDR);
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0);   // Column start address (0 = reset)
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, dev->params.screen_width-1); // Column end address (127 = reset)
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, SSD1306_PAGEADDR);
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, 0); // Page start address (0 = reset)
    if(retval != SSD1306_OK) {
        return retval;
    }

    if(dev->params.screen_height == 64) {
        retval = ssd1306_command(dev, 7); // Page end address
        if(retval != SSD1306_OK) {
            return retval;
        }
    } else if(dev->params.screen_height == 32) {
        retval = ssd1306_command(dev, 3); // Page end address
        if(retval != SSD1306_OK) {
            return retval;
        }
    } else if(dev->params.screen_height == 16) {
        retval = ssd1306_command(dev, 1); // Page end address
        if(retval != SSD1306_OK) {
            return retval;
        }
    }

    for (uint16_t i=0; i<(dev->params.screen_width*dev->params.screen_height/8); i += 128) {
        retval = ssd1306_data(dev, i);
        if(retval != SSD1306_OK) {
            return retval;
        }
    }

    return SSD1306_OK;
}

int ssd1306_clear_display(ssd1306_t *dev) {
  memset(buffer, 0, (dev->params.screen_width*dev->params.screen_height/8));
  dev->params.cursor_x = 0;
  dev->params.cursor_y = 0;
  return SSD1306_OK;
}

int ssd1306_invert_display(ssd1306_t *dev, bool i) {
  if(i) {
    return ssd1306_command(dev, SSD1306_INVERTDISPLAY);
  } else {
    return ssd1306_command(dev, SSD1306_NORMALDISPLAY);
  }
}

int ssd1306_dim(ssd1306_t *dev, bool dim) {
    uint8_t contrast;
    int retval = 0;

    if (dim) {
        contrast = 0; // Dimmed display
    } else {
        if(dev->params.vccstate == SSD1306_EXTERNALVCC) {
            contrast = 0x9F;
        } else {
            contrast = 0xCF;
        }
    }

    // the range of contrast to too small to be really useful
    // it is useful to dim the display
    retval = ssd1306_command(dev, SSD1306_SETCONTRAST);
    if(retval != SSD1306_OK) {
        return retval;
    }

    retval = ssd1306_command(dev, contrast);
    if(retval != SSD1306_OK) {
        return retval;
    }

    return SSD1306_OK;
}

int ssd1306_draw_pixel(ssd1306_t *dev, int16_t x, int16_t y, uint16_t color) {
    if ((x < 0) || (x >= dev->params.screen_width) || (y < 0) || (y >= dev->params.screen_height))
    return -1; // TODO: Error code

    // TODO: getRotation()
    switch(dev->params.screen_rotation) {
        case 1:
            ssd1306_swap(x, y);
            x = dev->params.screen_width - x - 1;
            break;
        case 2:
            x = dev->params.screen_width - x - 1;
            y = dev->params.screen_height - y - 1;
            break;
        case 3:
            ssd1306_swap(x, y);
            y = dev->params.screen_height - y - 1;
            break;
    }

    // x is which column
    switch(color)
    {
      case WHITE:   buffer[x+ (y/8)*dev->params.screen_width] |=  (1 << (y&7)); break;
      case BLACK:   buffer[x+ (y/8)*dev->params.screen_width] &= ~(1 << (y&7)); break;
      case INVERSE: buffer[x+ (y/8)*dev->params.screen_width] ^=  (1 << (y&7)); break;
    }

    return SSD1306_OK;
}

int ssd1306_draw_fast_h_line(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, uint16_t color) {
    bool bSwap = false;
    switch(dev->params.screen_rotation) {
        case 0:
            // 0 degree rotation, do nothing
            break;
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x
            bSwap = true;
            ssd1306_swap(x, y);
            x = dev->params.screen_width - x - 1;
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for height.
            x = dev->params.screen_width - x - 1;
            y = dev->params.screen_height - y - 1;
            x -= (w-1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
            bSwap = true;
            ssd1306_swap(x, y);
            y = dev->params.screen_height - y - 1;
            y -= (w-1);
            break;
    }

    if(bSwap) {
        return ssd1306_draw_fast_v_line_internal(dev, x, y, w, color);
    } else {
        return ssd1306_draw_fast_h_line_internal(dev, x, y, w, color);
    }
}

int ssd1306_draw_fast_h_line_internal(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, uint16_t color) {
    // Do bounds/limit checks
    if(y < 0 || y >= dev->params.screen_height) { return -SSD1306_ERR_RES; }

    // make sure we don't try to draw below 0
    if(x < 0) {
        w += x;
        x = 0;
    }

    // make sure we don't go off the edge of the display
    if( (x + w) > dev->params.screen_width) {
        w = (dev->params.screen_width - x);
    }

    // if our width is now negative, punt
    if(w <= 0) { return -SSD1306_ERR_GEO; }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y/8) * dev->params.screen_width);
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y&7);

    switch (color) {
        case WHITE:   while(w--) { *pBuf++ |= mask; }; break;
        case BLACK:   mask = ~mask;   while(w--) { *pBuf++ &= mask; }; break;
        case INVERSE: while(w--) { *pBuf++ ^= mask; }; break;
    }

    return SSD1306_OK;
}

int ssd1306_draw_fast_v_line(ssd1306_t *dev, int16_t x, int16_t y, int16_t h, uint16_t color) {
    bool bSwap = false;
    switch(dev->params.screen_rotation) {
        case 0:
            break;
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
            bSwap = true;
            ssd1306_swap(x, y);
            x = dev->params.screen_width - x - 1;
            x -= (h-1);
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for height.
            x = dev->params.screen_width - x - 1;
            y = dev->params.screen_height - y - 1;
            y -= (h-1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation, then invert y
            bSwap = true;
            ssd1306_swap(x, y);
            y = dev->params.screen_height - y - 1;
            break;
    }

    if(bSwap) {
        return ssd1306_draw_fast_h_line_internal(dev, x, y, h, color);
    } else {
        return ssd1306_draw_fast_v_line_internal(dev, x, y, h, color);
    }
}

int ssd1306_draw_fast_v_line_internal(ssd1306_t *dev, int16_t x, int16_t __y, int16_t __h, uint16_t color) {

    // do nothing if we're off the left or right side of the screen
    if(x < 0 || x >= dev->params.screen_width) { return -SSD1306_ERR_RES; }

    // make sure we don't try to draw below 0
    if(__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;

    }

    // make sure we don't go past the height of the display
    if( (__y + __h) > dev->params.screen_height) {
        __h = (dev->params.screen_height - __y);
    }

    // if our height is now negative, punt
    if(__h <= 0) {
        return -SSD1306_ERR_GEO;
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;


    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y/8) * dev->params.screen_width);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y&7);
    if(mod) {
        // mask off the high n bits we want to set
        mod = 8-mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if( h < mod) {
            mask &= (0XFF >> (mod-h));
        }

        switch (color) {
            case WHITE:   *pBuf |=  mask;  break;
            case BLACK:   *pBuf &= ~mask;  break;
            case INVERSE: *pBuf ^=  mask;  break;
        }

        // fast exit if we're done here!
        if(h<mod) { return SSD1306_OK; }

        h -= mod;

        pBuf += dev->params.screen_width;
    }


    // write solid bytes while we can - effectively doing 8 rows at a time
    if(h >= 8) {
        if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
            do  {
            *pBuf=~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += dev->params.screen_width;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            } while(h >= 8);
            }
        else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do  {
                // write our value in
            *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += dev->params.screen_width;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            } while(h >= 8);
            }
        }

    // now do the final partial byte, if necessary
    if(h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color) {
            case WHITE:   *pBuf |=  mask;  break;
            case BLACK:   *pBuf &= ~mask;  break;
            case INVERSE: *pBuf ^=  mask;  break;
        }
    }

    return SSD1306_OK;
}

void ssd1306_set_cursor(ssd1306_t *dev, int16_t x, int16_t y) {
    dev->params.cursor_x = x;
    dev->params.cursor_y = y;
}

int ssd1306_memset(uint8_t buffer[], uint16_t start_address, uint16_t destination_address) {
    uint16_t x;

    for(x = start_address; x < destination_address; x++) {
        buffer[x] = 0x00;
    }

    return SSD1306_OK;
}

int ssd1306_command(ssd1306_t *dev, uint8_t c) {
    Buffer_CMD[1] = c;

    i2c_acquire(DEV_I2C);

    if(i2c_write_bytes(DEV_I2C, DEV_ADDR, Buffer_CMD, 2, 0) != 0) {
        i2c_release(DEV_I2C);
        return -SSD1306_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    return SSD1306_OK;
}

int ssd1306_data(ssd1306_t *dev, uint16_t a) {
    uint8_t i;

    for(i = 0; i < 128 ; i++) {
        Buffer_DATA[i+1] = buffer[a+i];
    }

    i2c_acquire(DEV_I2C);

    if(i2c_write_bytes(DEV_I2C, DEV_ADDR, Buffer_DATA, 129, 0) != 0) {
        i2c_release(DEV_I2C);
        return -SSD1306_ERR_NODEV;
    }

    i2c_release(DEV_I2C);

    return SSD1306_OK;
}

// ----------------- Higher level functions ------------------------------------

void ssd1306_draw_circle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ssd1306_draw_pixel(dev, x0  , y0+r, color);
    ssd1306_draw_pixel(dev, x0  , y0-r, color);
    ssd1306_draw_pixel(dev, x0+r, y0  , color);
    ssd1306_draw_pixel(dev, x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ssd1306_draw_pixel(dev, x0 + x, y0 + y, color);
        ssd1306_draw_pixel(dev, x0 - x, y0 + y, color);
        ssd1306_draw_pixel(dev, x0 + x, y0 - y, color);
        ssd1306_draw_pixel(dev, x0 - x, y0 - y, color);
        ssd1306_draw_pixel(dev, x0 + y, y0 + x, color);
        ssd1306_draw_pixel(dev, x0 - y, y0 + x, color);
        ssd1306_draw_pixel(dev, x0 + y, y0 - x, color);
        ssd1306_draw_pixel(dev, x0 - y, y0 - x, color);
    }
}

void ssd1306_draw_circle_helper(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color) {
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            ssd1306_draw_pixel(dev, x0 + x, y0 + y, color);
            ssd1306_draw_pixel(dev, x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            ssd1306_draw_pixel(dev, x0 + x, y0 - y, color);
            ssd1306_draw_pixel(dev, x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            ssd1306_draw_pixel(dev, x0 - y, y0 + x, color);
            ssd1306_draw_pixel(dev, x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            ssd1306_draw_pixel(dev, x0 - y, y0 - x, color);
            ssd1306_draw_pixel(dev, x0 - x, y0 - y, color);
        }
    }
}

void ssd1306_fill_circle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    ssd1306_draw_fast_v_line(dev, x0, y0-r, 2*r+1, color);
    ssd1306_fill_circle_helper(dev, x0, y0, r, 3, 0, color);
}

void ssd1306_fill_circle_helper(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color) {
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;

        if (cornername & 0x1) {
            ssd1306_draw_fast_v_line(dev, x0+x, y0-y, 2*y+1+delta, color);
            ssd1306_draw_fast_v_line(dev, x0+y, y0-x, 2*x+1+delta, color);
        }
        if (cornername & 0x2) {
            ssd1306_draw_fast_v_line(dev, x0-x, y0-y, 2*y+1+delta, color);
            ssd1306_draw_fast_v_line(dev, x0-y, y0-x, 2*x+1+delta, color);
        }
    }
}

void ssd1306_draw_line(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        ssd1306_swap(x0, y0);
        ssd1306_swap(x1, y1);
    }

    if (x0 > x1) {
        ssd1306_swap(x0, x1);
        ssd1306_swap(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            ssd1306_draw_pixel(dev, y0, x0, color);
        } else {
            ssd1306_draw_pixel(dev, x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void ssd1306_draw_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    ssd1306_draw_fast_h_line(dev, x, y, w, color);
    ssd1306_draw_fast_h_line(dev, x, y+h-1, w, color);
    ssd1306_draw_fast_v_line(dev, x, y, h, color);
    ssd1306_draw_fast_v_line(dev, x+w-1, y, h, color);
}

void ssd1306_fill_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    for (int16_t i=x; i<x+w; i++) {
        ssd1306_draw_fast_v_line(dev, i, y, h, color);
    }
}

void ssd1306_fill_screen(ssd1306_t *dev, uint16_t color) {
    ssd1306_fill_rect(dev, 0, 0, dev->params.screen_width, dev->params.screen_height, color);
}

void ssd1306_draw_round_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
    ssd1306_draw_fast_h_line(dev, x+r  , y    , w-2*r, color); // Top
    ssd1306_draw_fast_h_line(dev, x+r  , y+h-1, w-2*r, color); // Bottom
    ssd1306_draw_fast_v_line(dev, x    , y+r  , h-2*r, color); // Left
    ssd1306_draw_fast_v_line(dev, x+w-1, y+r  , h-2*r, color); // Right

    ssd1306_draw_circle_helper(dev, x+r    , y+r    , r, 1, color);
    ssd1306_draw_circle_helper(dev, x+w-r-1, y+r    , r, 2, color);
    ssd1306_draw_circle_helper(dev, x+w-r-1, y+h-r-1, r, 4, color);
    ssd1306_draw_circle_helper(dev, x+r    , y+h-r-1, r, 8, color);
}

void ssd1306_fill_round_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
    ssd1306_fill_rect(dev, x+r, y, w-2*r, h, color);
    ssd1306_fill_circle_helper(dev, x+w-r-1, y+r, r, 1, h-2*r-1, color);
    ssd1306_fill_circle_helper(dev, x+r    , y+r, r, 2, h-2*r-1, color);
}

void ssd1306_draw_triangle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    ssd1306_draw_line(dev, x0, y0, x1, y1, color);
    ssd1306_draw_line(dev, x1, y1, x2, y2, color);
    ssd1306_draw_line(dev, x2, y2, x0, y0, color);
}

void ssd1306_fill_triangle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    int16_t a, b, y, last;

    if (y0 > y1) {
        ssd1306_swap(y0, y1); ssd1306_swap(x0, x1);
    }
    if (y1 > y2) {
        ssd1306_swap(y2, y1); ssd1306_swap(x2, x1);
    }
    if (y0 > y1) {
        ssd1306_swap(y0, y1); ssd1306_swap(x0, x1);
    }

    if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        ssd1306_draw_fast_h_line(dev, a, y0, b-a+1, color);
        return;
    }

    int16_t
        dx01 = x1 - x0,
        dy01 = y1 - y0,
        dx02 = x2 - x0,
        dy02 = y2 - y0,
        dx12 = x2 - x1,
        dy12 = y2 - y1;
    int32_t
        sa   = 0,
        sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) ssd1306_swap(a,b);
        ssd1306_draw_fast_h_line(dev, a, y, b-a+1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for(; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) ssd1306_swap(a,b);
        ssd1306_draw_fast_h_line(dev, a, y, b-a+1, color);
    }
}

void ssd1306_write(ssd1306_t *dev, uint8_t c, uint8_t textsize, uint16_t textcolor, uint16_t textbgcolor) {
    if (c == '\n') {
        dev->params.cursor_y += textsize*8;
        dev->params.cursor_x  = 0;
    } else if (c == '\r') {
        // skip
    } else {
        ssd1306_draw_char(dev, dev->params.cursor_x, dev->params.cursor_y, c, textcolor, textbgcolor, textsize);
        dev->params.cursor_x += textsize*6;
        if(dev->params.wrap && (dev->params.cursor_x > (dev->params.screen_width - textsize*6))) {
            dev->params.cursor_y += textsize*8;
            dev->params.cursor_x = 0;
        }
    }
}

void ssd1306_draw_char(ssd1306_t *dev, int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) {
    if((x >= dev->params.screen_width)            || // Clip right
         (y >= dev->params.screen_height)           || // Clip bottom
         ((x + 6 * size - 1) < 0) || // Clip left
         ((y + 8 * size - 1) < 0))   // Clip top
        return;

    for (int8_t i=0; i<6; i++ ) {
        uint8_t line;
        if (i == 5) {
            line = 0x0;
        } else {
            line = (*(const unsigned char *)(font+(c*5)+i));
        }

        for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(dev, x+i, y+j, color);
                else {  // big size
                    ssd1306_fill_rect(dev, x+(i*size), y+(j*size), size, size, color);
                }
            } else if (bg != color) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(dev, x+i, y+j, bg);
                else {  // big size
                    ssd1306_fill_rect(dev, x+i*size, y+j*size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
}

size_t ssd1306_printf(ssd1306_t *dev, uint8_t textsize, uint16_t textcolor, uint16_t textbgcolor, const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    char temp[128];
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);

    for(size_t i = 0; i < len; i++) {
        ssd1306_write(dev, temp[i], textsize, textcolor, textbgcolor);
    }

    return len;
}
