/*
 * Copyright (C) 2019 Marius <marius@twostairs.com>
 * Copyright (c) 2012, Adafruit Industries
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ssd1306 SSD1306 OLED display
 * @ingroup     drivers_ssd1306
 * @brief       Device driver interface for the SSD1306 OLED display.
 * @{
 *
 * @file
 * @brief       Device driver interface for the SSD1306 OLED display.
 * This driver is a re-implementation of the Adafruit SSD1306 driver and
 * GFX Library, combining both and adapting all code to the RIOT driver
 * standards. Large parts of the logic code are copy & paste from Adafruit's
 * libraries.
 *
 * @author      Marius <marius@twostairs.com>
 *
 * @}
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "xtimer.h"
#include "periph/i2c.h"

#include "glcdfont.h"

#define SSD1306_ADDR (0x3C)

#define BLACK                          0 ///< Draw 'off' pixels
#define WHITE                          1 ///< Draw 'on' pixels
#define INVERSE                        2 ///< Invert pixels

#define SSD1306_CMD   0x00
#define SSD1306_DATA  0x40

#define SSD1306_MEMORYMODE          0x20 ///< See datasheet
#define SSD1306_COLUMNADDR          0x21 ///< See datasheet
#define SSD1306_PAGEADDR            0x22 ///< See datasheet
#define SSD1306_SETCONTRAST         0x81 ///< See datasheet
#define SSD1306_CHARGEPUMP          0x8D ///< See datasheet
#define SSD1306_SEGREMAP            0xA0 ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON        0xA5 ///< Not currently used
#define SSD1306_NORMALDISPLAY       0xA6 ///< See datasheet
#define SSD1306_INVERTDISPLAY       0xA7 ///< See datasheet
#define SSD1306_SETMULTIPLEX        0xA8 ///< See datasheet
#define SSD1306_DISPLAYOFF          0xAE ///< See datasheet
#define SSD1306_DISPLAYON           0xAF ///< See datasheet
#define SSD1306_COMSCANINC          0xC0 ///< Not currently used
#define SSD1306_COMSCANDEC          0xC8 ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET    0xD3 ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5 ///< See datasheet
#define SSD1306_SETPRECHARGE        0xD9 ///< See datasheet
#define SSD1306_SETCOMPINS          0xDA ///< See datasheet
#define SSD1306_SETVCOMDETECT       0xDB ///< See datasheet

#define SSD1306_SETLOWCOLUMN        0x00 ///< Not currently used
#define SSD1306_SETHIGHCOLUMN       0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE        0x40 ///< See datasheet

#define SSD1306_EXTERNALVCC         0x01 ///< External display voltage source
#define SSD1306_SWITCHCAPVCC        0x02 ///< Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26 ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27 ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL                    0x2E ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL                      0x2F ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3 ///< Set scroll range

// Deprecated size stuff for backwards compatibility with old sketches
#if defined SSD1306_128_64
 #define SSD1306_LCDWIDTH  128 ///< DEPRECATED: width w/SSD1306_128_64 defined
 #define SSD1306_LCDHEIGHT  64 ///< DEPRECATED: height w/SSD1306_128_64 defined
#endif
#if defined SSD1306_128_32
 #define SSD1306_LCDWIDTH  128 ///< DEPRECATED: width w/SSD1306_128_32 defined
 #define SSD1306_LCDHEIGHT  32 ///< DEPRECATED: height w/SSD1306_128_32 defined
#endif
#if defined SSD1306_96_16
 #define SSD1306_LCDWIDTH   96 ///< DEPRECATED: width w/SSD1306_96_16 defined
 #define SSD1306_LCDHEIGHT  16 ///< DEPRECATED: height w/SSD1306_96_16 defined
#endif

#ifndef SSD1306_PARAM_I2C_DEV
#define SSD1306_PARAM_I2C_DEV         I2C_DEV(0)
#endif
#ifndef SSD1306_PARAM_I2C_ADDR
#define SSD1306_PARAM_I2C_ADDR        SSD1306_ADDR
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
    uint8_t screen_width;                       /**< Screen width (pixels) */
    uint8_t screen_height;                      /**< Screen height (pixels) */
    uint8_t screen_rotation;                    /**< Screen rotation (0 = 0°, 1 = 90°, 2 = 180°, 3 = 270°) */
    uint8_t vccstate;                           /**< vcc state */
    int16_t cursor_x;                           /**< cursor X */
    int16_t cursor_y;                           /**< cursor Y */
    bool wrap;                                  /**< wrap around */
} ssd1306_params_t;

/**
 * @brief   Device descriptor for the SSD1306
 */
typedef struct {
    ssd1306_params_t      params;               /**< Device initialization parameters */
} ssd1306_t;

/**
 * @brief   Status and error return codes
 */
enum {
    SSD1306_OK = 0,                              /**< everything was fine */
    SSD1306_ERR_NODEV,                           /**< did not detect device */
    SSD1306_ERR_RES,                             /**< error related to resolution */
    SSD1306_ERR_GEO,                             /**< error related to geometry */
};

int ssd1306_init(ssd1306_t *dev, const ssd1306_params_t *params);

int ssd1306_display(ssd1306_t *dev);
int ssd1306_clear_display(ssd1306_t *dev);
int ssd1306_invert_display(ssd1306_t *dev, bool i);
int ssd1306_dim(ssd1306_t *dev, bool dim);

int ssd1306_draw_pixel(ssd1306_t *dev, int16_t x, int16_t y, uint16_t color);
int ssd1306_draw_fast_h_line(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, uint16_t color);
int ssd1306_draw_fast_h_line_internal(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, uint16_t color);
int ssd1306_draw_fast_v_line(ssd1306_t *dev, int16_t x, int16_t y, int16_t h, uint16_t color);
int ssd1306_draw_fast_v_line_internal(ssd1306_t *dev, int16_t x, int16_t __y, int16_t __h, uint16_t color);

void ssd1306_set_cursor(ssd1306_t *dev, int16_t x, int16_t y);

int ssd1306_memset(uint8_t buffer[], uint16_t start_address, uint16_t destination_address);

int ssd1306_command(ssd1306_t *dev, uint8_t c);
int ssd1306_data(ssd1306_t *dev, uint16_t a);

// Higher level functions
void ssd1306_draw_circle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ssd1306_draw_circle_helper(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void ssd1306_fill_circle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ssd1306_fill_circle_helper(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void ssd1306_draw_line(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ssd1306_draw_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ssd1306_fill_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ssd1306_fill_screen(ssd1306_t *dev, uint16_t color);
void ssd1306_draw_round_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ssd1306_fill_round_rect(ssd1306_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ssd1306_draw_triangle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ssd1306_fill_triangle(ssd1306_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

void ssd1306_write(ssd1306_t *dev, uint8_t c, uint8_t textsize, uint16_t textcolor, uint16_t textbgcolor);
void ssd1306_draw_char(ssd1306_t *dev, int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
size_t ssd1306_printf(ssd1306_t *dev, uint8_t textsize, uint16_t textcolor, uint16_t textbgcolor, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
