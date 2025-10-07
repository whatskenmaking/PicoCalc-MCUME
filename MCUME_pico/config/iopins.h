#ifndef IOPINS_H
#define IOPINS_H

#include "platform_config.h"

#ifdef MCUME_REV1

// Speaker
#define AUDIO_PIN       28
// VGA
/*
2-9 RRRGGGBB
10-11 VSYNC and HSYNC
*/
#define VGA_COLORBASE   2
#define VGA_SYNCBASE    10

// TFT
#define TFT_SPIREG      spi1
#define TFT_SPIDREQ     DREQ_SPI1_TX
#define TFT_SCLK        14
#define TFT_MOSI        15
#define TFT_MISO        12
#define TFT_DC          28
#define TFT_CS          13  // 255 for LORES ST7789 (NO CS)
#define TFT_RST         255 // 255 for ILI/ST if connected to 3.3V
#define TFT_BACKLIGHT   255 // hardwired to 3.3v

// SD (see SPI0 in code!!!)
#define SD_SPIREG       spi0
#define SD_SCLK         18
#define SD_MOSI         19
#define SD_MISO         16 
#define SD_CS           17
#define SD_DETECT       255 // 22

// Analog joystick (primary) for JOY2 and 3 extra buttons
#define PIN_JOY2_A1X    26
#define PIN_JOY2_A2Y    27
#define PIN_JOY2_BTN    22
#define PIN_KEY_USER1   20
#define PIN_KEY_USER2   21 

// I2C keyboard (Not available on PICO)
/*
#define I2C_SCL_IO      15? 
#define I2C_SDA_IO      14?
*/

#endif /* end MCUME_REV1 */

#ifdef PICOCALC

// Audio (PWM)
#define AUDIO_PIN       26      // GP26 - Left audio channel  
#define AUDIO_RIGHT     27      // GP27 - Right audio channel

// VGA (if using VGA output)
#define VGA_COLORBASE   2
#define VGA_SYNCBASE    10

// TFT Display (SPI1)
#define TFT_SPIREG      spi1    // Uses SPI1
#define TFT_SPIDREQ     DREQ_SPI1_TX
#define TFT_SCLK        10      // GP10 - LCD_SCLK_PIN
#define TFT_MOSI        11      // GP11 - LCD_MOSI_PIN
#define TFT_MISO        12      // GP12 - LCD_MISO_PIN
#define TFT_DC          14      // GP14 - LCD_DC_PIN
#define TFT_CS          13      // GP13 - LCD_CS_PIN
#define TFT_RST         15      // GP15 - LCD_RST_PIN
#define TFT_BACKLIGHT   255     // No backlight control pin defined

// SD Card (SD SPI)
#define SD_SPIREG       spi0    // Uses SPI0
#define SD_SCLK         18      // GP18 - SD_SCLK_PIN
#define SD_MOSI         19      // GP19 - SD_MOSI_PIN
#define SD_MISO         16      // GP16 - SD_MISO_PIN
#define SD_CS           17      // GP17 - SD_CS_PIN
#define SD_DETECT       22      // GP22 - SD_DET_PIN

// I2C Keyboard (hardwired)
#define I2C_SCL_IO      7       // GP7 - I2C1 SCL
#define I2C_SDA_IO      6       // GP6 - I2C1 SDA
// Note: I2C instance (i2c1) will be used directly in code

// Calculator-specific pins (buttons from config.h)
#define NEXT_BUTTON     2       // GP2
#define PART_BUTTON     3       // GP3

#endif /* end PICOCALC */

#ifdef PICOZX

// Speaker
#define AUDIO_PIN       7
// VGA
/* RRGGBB
   CSYNC */
#define VGA_COLORBASE   0
#define VGA_SYNCBASE    6

#define TFT_SPIREG      spi0
#define TFT_SPIDREQ     DREQ_SPI0_TX
#define TFT_SCLK        2
#define TFT_MOSI        3
#define TFT_MISO        255 // Not required, used for DC... 
#define TFT_DC          6
#define TFT_RST         255
#define TFT_CS          5
#define TFT_BACKLIGHT   4

// Keyboard matrix 
//Cols (out)
#define KCOLOUT1        8
#define KCOLOUT2        9
#define KCOLOUT3        14
#define KCOLOUT4        15
#define KCOLOUT5        16
#define KCOLOUT6        17
#define KCOLOUT7        18
//Rows (in)
#define KROWIN1         19
#define KROWIN2         20
#define KROWIN3         21
#define KROWIN4         22
#define KROWIN5         26
#define KROWIN6         27
#define KROWIN7         28

#endif /* end PICOZX */

#ifdef PICOMPUTER

#ifdef PICOMPUTERMAX
#define TFT_RST         255
#define TFT_CS          21
#define TFT_BACKLIGHT   20

#else /* end PICOMPUTERMAX */
#define TFT_RST         21
#define TFT_CS          255
#define TFT_BACKLIGHT   20
#endif

#if defined(USE_VGA)

// Keyboard matrix 
//Cols (out)
#define KCOLOUT1        20
#define KCOLOUT2        21
#define KCOLOUT3        22
#define KCOLOUT4        26
#define KCOLOUT5        27
#define KCOLOUT6        28
//Rows (in)
#define KROWIN1         14
#define KROWIN2         15
#define KROWIN3         16
#define KROWIN4         17
#define KROWIN5         18
#define KROWIN6         19

// Speaker
#define AUDIO_PIN       9
// VGA
/* RRRGGGBB
   CSYNC */
#define VGA_COLORBASE   0
#define VGA_SYNCBASE    8

#else /* end USE_VGA (RETROVGA)*/

// TFT
#define TFT_SPIREG      spi0
#define TFT_SPIDREQ     DREQ_SPI0_TX
#define TFT_SCLK        18
#define TFT_MOSI        19
#define TFT_MISO        255 // Not required, used for DC... 
#define TFT_DC          16
#define TFT_RST         21
#define TFT_CS          5
#define TFT_BACKLIGHT   4

// Keyboard matrix 
//Cols (out)
#define KCOLOUT1        20
#define KCOLOUT2        21
#define KCOLOUT3        22
#define KCOLOUT4        26
#define KCOLOUT5        27
#define KCOLOUT6        28
//Rows (in)
#define KROWIN1         14
#define KROWIN2         15
#define KROWIN3         16
#define KROWIN4         17
#define KROWIN5         18
#define KROWIN6         19

// Speaker
#define AUDIO_PIN       9

// Digital joystick (primary) for JOY2 and 2 extra buttons
#define PIN_JOY2_1      27  // UP
#define PIN_JOY2_2      26  // DOWN
#define PIN_JOY2_3      28  // RIGHT
#define PIN_JOY2_4      22  // LEFT
#define PIN_JOY2_BTN    1
#define PIN_KEY_USER1   20
#define PIN_KEY_USER2   21 

#endif /* end USE_VGA */

#endif /* end PICOMPUTER */

// Second joystick (Not available on PICO)
//#define PIN_JOY1_BTN     2
//#define PIN_JOY1_1       14 // UP
//#define PIN_JOY1_2       7  // DOWN
//#define PIN_JOY1_3       6  // RIGHT
//#define PIN_JOY1_4       5  // LEFT

#endif /* end IOPINS_H */
