/*
	This file is part of DISPLAY library. 
  Supports VGA and TFT display

	DISPLAY library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Copyright (C) 2020 J-M Harvengt
*/

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <string.h>

#include "PICO_DSP.h"
#include "font8x8.h"
#include "include.h"

#include "lcdspi.h"

extern PICO_DSP tft;
#define RGBVAL16(r,g,b)  ( (((r>>3)&0x1f)<<11) | (((g>>2)&0x3f)<<5) | (((b>>3)&0x1f)<<0) )

static gfx_mode_t gfxmode = MODE_UNDEFINED;

/* TFT structures / constants */
#define digitalWrite(pin, val) gpio_put(pin, val)

// Default SPI clock for TFT panel. ILI9488 panels typically tolerate 62.5MHz+ on short wires.
// You can lower this if you see artifacts.
#ifndef SPICLOCK
#define SPICLOCK 62500000
#endif
#ifdef USE_VGA
#define SPI_MODE SPI_CPOL_1 
#else
#ifdef ST7789
#ifdef ST7789_POL
#define SPI_MODE SPI_CPOL_0
#else
#define SPI_MODE SPI_CPOL_1
#endif
#endif
#ifdef ILI9341
#define SPI_MODE SPI_CPOL_0
#endif
#ifdef ILI9488
#define SPI_MODE SPI_CPOL_0
#endif
#endif

#define LINES_PER_BLOCK  64
#define NR_OF_BLOCK      4

#define TFT_SWRESET    0x01
#define TFT_SLPOUT     0x11
#define TFT_INVON      0x21
#define TFT_DISPOFF    0x28
#define TFT_DISPON     0x29
#define TFT_CASET      0x2A
#define TFT_PASET      0x2B
#define TFT_RAMWR      0x2C
#define TFT_MADCTL     0x36
#define TFT_PIXFMT     0x3A
#define TFT_MADCTL_MY  0x80
#define TFT_MADCTL_MX  0x40
#define TFT_MADCTL_MV  0x20
#define TFT_MADCTL_ML  0x10
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08
#define TFT_MADCTL_MH  0x04

static void SPItransfer(uint8_t val)
{
  uint8_t dat8=val;
  spi_write_blocking(TFT_SPIREG, &dat8, 1);
}

static void SPItransfer16(uint16_t val)
{
  uint8_t dat8[2];
  dat8[0] = val>>8;
  dat8[1] = val&0xff;
  spi_write_blocking(TFT_SPIREG, dat8, 2);
}

// All configurations now use 16-bit RGB565 pixel transfers
#define SPItransferPixel(color) SPItransfer16(color)

#define DELAY_MASK     0x80

static const uint8_t init_commands[] = { 
  1+DELAY_MASK, TFT_SWRESET,  150,
  1+DELAY_MASK, TFT_SLPOUT,   255,
  2+DELAY_MASK, TFT_PIXFMT, 0x55, 10,
  2,            TFT_MADCTL, TFT_MADCTL_MV | TFT_MADCTL_BGR,
  1, TFT_INVON,
  1, TFT_DISPON,
  0
};

/* TFT structures / constants */
#define RGBVAL(r,g,b) RGBVAL16(r,g,b)

static uint16_t * blocks[NR_OF_BLOCK];
static uint16_t blocklens[NR_OF_BLOCK];
static dma_channel_config dmaconfig;
static uint dma_tx=0;
static volatile uint8_t rstop = 0;
static volatile bool cancelled = false;
static volatile uint8_t curTransfer = 0;
static uint8_t nbTransfer = 0;

// Scanline buffer for direct 16-bit RGB565 pushes (2 bytes per pixel)
static uint8_t picocalc_linebuf[320*2];

#ifdef PICOCALC
// Per-line DMA for PICOCALC - ping-pong buffers for overlapped line processing
static int picocalc_dma_ch = -1;
static volatile bool picocalc_dma_busy = false;
static uint8_t picocalc_line_buf[2][320*2]; // ping-pong buffers
static int picocalc_active_buf = 0; // which buffer is being filled (0 or 1)

static void picocalc_dma_isr() {
  uint32_t mask = 1u << picocalc_dma_ch;
  if (dma_hw->ints0 & mask) {
    dma_hw->ints0 = mask; // clear interrupt
    picocalc_dma_busy = false;
  }
}

static void picocalc_dma_init() {
  if (picocalc_dma_ch >= 0) return; // already initialized
  
  picocalc_dma_ch = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(picocalc_dma_ch);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
  channel_config_set_dreq(&cfg, TFT_SPIDREQ);
  channel_config_set_read_increment(&cfg, true);
  channel_config_set_write_increment(&cfg, false);
  
  dma_channel_configure(picocalc_dma_ch, &cfg,
                        &spi_get_hw(TFT_SPIREG)->dr, // SPI data register
                        NULL, 0, false); // read_addr and count set per transfer
  
  // Set up interrupt handler
  irq_set_exclusive_handler(DMA_IRQ_0, picocalc_dma_isr);
  dma_channel_set_irq0_enabled(picocalc_dma_ch, true);
  irq_set_enabled(DMA_IRQ_0, true);
}

// Non-blocking DMA transfer for a single scanline
static void picocalc_dma_send_line(const uint8_t *data, uint32_t byte_count) {
  picocalc_dma_busy = true;
  dma_channel_transfer_from_buffer_now(picocalc_dma_ch, data, byte_count);
}

// Blocking wait for current DMA transfer to complete
static void picocalc_dma_wait() {
  while (picocalc_dma_busy) {
    tight_loop_contents();
  }
}

// Get buffer for current line prep (ping-pong)
static uint8_t* picocalc_get_line_buffer() {
  return picocalc_line_buf[picocalc_active_buf];
}

// Send current buffer via DMA and switch to next buffer
static void picocalc_send_and_swap(uint32_t byte_count) {
  picocalc_dma_send_line(picocalc_line_buf[picocalc_active_buf], byte_count);
  picocalc_active_buf = 1 - picocalc_active_buf; // swap 0<->1
}
#endif // PICOCALC

/* VGA structures / constants */
#define R16(rgb) ((rgb>>8)&0xf8) 
#define G16(rgb) ((rgb>>3)&0xfc) 
#define B16(rgb) ((rgb<<3)&0xf8) 
#ifdef VGA222
#define VGA_RGB(r,g,b)   ( (((r>>6)&0x03)<<4) | (((g>>6)&0x03)<<2) | (((b>>6)&0x3)<<0) )
#else
#define VGA_RGB(r,g,b)   ( (((r>>5)&0x07)<<5) | (((g>>5)&0x07)<<2) | (((b>>6)&0x3)<<0) )
#endif

// 8 bits 320x240 frame buffer => 64K
static vga_pixel * visible_framebuffer = NULL;
static vga_pixel * framebuffer = NULL;
static vga_pixel * fb0 = NULL;
static vga_pixel * fb1 = NULL;

static int  fb_width;
static int  fb_height;
static int  fb_stride;

static const sVmode* vmode=NULL;
static const sVmode* volatile VgaVmodeReq = NULL; // request to reinitialize videomode, 1=only stop driver

static semaphore_t core1_initted;
static void core1_func();
static void core1_sio_irq();
static void VgaInitReql(const sVmode* vmode)
{
  if (vmode == NULL) vmode = (const sVmode*)1;
  __dmb();
  VgaVmodeReq = vmode;
  while (VgaVmodeReq != NULL) { __dmb(); }
}

static void core1_func()
{
  const sVmode* v;

  multicore_fifo_clear_irq();
  irq_set_exclusive_handler(SIO_IRQ_PROC1,core1_sio_irq);
  //irq_set_priority (SIO_IRQ_PROC1, 129);    
  irq_set_enabled(SIO_IRQ_PROC1,true);

  sem_release(&core1_initted);
 
  while (true)
  {
    __dmb();

    // initialize videomode
    v = VgaVmodeReq;
    if (v != NULL)
    {
      if ((u32)v == (u32)1) {
        //VgaTerm(); // terminate
      }
      else
        VgaInit(v,(u8*)framebuffer,320,240,320);
      __dmb();
      VgaVmodeReq = NULL;
    }
  }
}


void PICO_DSP::setArea(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2) {
  int dx=0;
  int dy=0;
#ifdef ST7789
  if (TFT_REALWIDTH == TFT_REALHEIGHT)
  {
#ifdef ROTATE_SCREEN
    if (!flipped) {
      dy += 80;    
    }
#else  
    if (flipped) {
      dx += 80;    
    }
#endif      
  }
#endif  

  digitalWrite(_dc, 0);
  SPItransfer(TFT_CASET);
  digitalWrite(_dc, 1);
  SPItransfer16(x1+dx);
  digitalWrite(_dc, 1);
  SPItransfer16(x2+dx);  
  digitalWrite(_dc, 0);
  SPItransfer(TFT_PASET);
  digitalWrite(_dc, 1);
  SPItransfer16(y1+dy);
  digitalWrite(_dc, 1);
  SPItransfer16(y2+dy);  

  digitalWrite(_dc, 0);
  SPItransfer(TFT_RAMWR);
  digitalWrite(_dc, 1);
 
  return; 
}


PICO_DSP::PICO_DSP()
{
}

gfx_error_t PICO_DSP::begin(gfx_mode_t mode)
{
  switch(mode) {
    case MODE_VGA_320x240:
      // Reset SPI if we come from TFT mode
      if (gfxmode == MODE_TFT_320x240) {
        fillScreenNoDma(RGBVAL(0x0,0x00,0x00));
        digitalWrite(_cs, 0);
        digitalWrite(_dc, 0);
        SPItransfer(TFT_DISPOFF);
        digitalWrite(_cs, 1);
        sleep_ms(20);
        digitalWrite(_cs, 0);
        digitalWrite(_cs, 1);
        if (_bkl != 0xff) {
          digitalWrite(_bkl, 0);
        }
        //spi_init(TFT_SPIREG, 0);
        //spi_deinit(TFT_SPIREG);
        //spi_set_slave(TFT_SPIREG, true);
      }  
      gfxmode = mode;
      fb_width = 320;
      fb_height = 240;      
      fb_stride = fb_width;
      /* initialize gfx buffer */
      if (fb0 == NULL) {
        void *mallocpt = malloc(fb_stride*fb_height*sizeof(vga_pixel)+4);    
        fb0 = (vga_pixel *)((void*)((intptr_t)mallocpt & ~3));
      }
      visible_framebuffer = fb0;
      framebuffer = fb0;
      for (uint i = 0; i < fb_height*fb_width; i++) {
        framebuffer[i] = VGA_RGB(rand() % 255,rand() % 255,rand() % 255);      
      }
      // create a semaphore to be posted when audio init is complete
      sem_init(&core1_initted, 0, 1);
      multicore_launch_core1(core1_func);
      vmode = Video(DEV_VGA, RES_QVGA);
      VgaInitReql(vmode);
      // wait for initialization of audio to be complete
      sem_acquire_blocking(&core1_initted);      
      break;

    case MODE_TFT_320x240:
      gfxmode = mode;
      fb_width = TFT_WIDTH;
      fb_height = TFT_HEIGHT;      
      fb_stride = fb_width;    
      
#ifdef PICOCALC
      // Use working LCD driver for PICOCALC
      lcd_init();
      return GFX_OK;
#endif
      
      _cs   = TFT_CS;
      _dc   = TFT_DC;
      _rst  = TFT_RST;
      _mosi = TFT_MOSI;
      _sclk = TFT_SCLK;
      _bkl = TFT_BACKLIGHT;
      gpio_init(_dc);
      gpio_set_dir(_dc, GPIO_OUT); 
      gpio_init(_cs);
      gpio_set_dir(_cs, GPIO_OUT);
      digitalWrite(_cs, 1);
      digitalWrite(_dc, 1);
      if (_bkl != 0xff) {
        gpio_init(_bkl);
        gpio_set_dir(_bkl, GPIO_OUT); 
        digitalWrite(_bkl, 1);
      } 

  spi_init(TFT_SPIREG, SPICLOCK);
      spi_set_format(TFT_SPIREG, 8, SPI_MODE, SPI_CPHA_0, SPI_MSB_FIRST);
      gpio_set_function(_sclk , GPIO_FUNC_SPI);
      gpio_set_function(_mosi , GPIO_FUNC_SPI);
         
      // Initialize display
      if (_rst != 0xff) {
        gpio_init(_rst);
        gpio_set_dir(_rst, GPIO_OUT);     
        digitalWrite(_rst, 1);
        sleep_ms(100);
        digitalWrite(_rst, 0);
        sleep_ms(100);
        digitalWrite(_rst, 1);
        sleep_ms(200);
      }
      const uint8_t *addr = init_commands;
      uint8_t count;
      digitalWrite(_cs, 0);
      while (count = *addr++) {
        uint8_t command = *addr++;
#ifdef ILI9341
        if ( command == TFT_INVON ) {
          // Skip TFT_INVON for ILI
        }
        else 
#endif
        {
          digitalWrite(_dc, 0); // command
          SPItransfer(command);
          uint16_t ms = count & DELAY_MASK;
          count &= ~DELAY_MASK;
          while (--count > 0) { // data
            uint8_t data = *addr++;
#ifdef ILI9341
#else
            if ( command == TFT_MADCTL ) {
              data = TFT_MADCTL_MX | TFT_MADCTL_MV |TFT_MADCTL_RGB;
            }
#endif
            digitalWrite(_dc, 1);
            SPItransfer(data);
          }
          if (ms) {
            ms = *addr++; // Read post-command delay time (ms)
            if(ms == 255) ms = 500;   // If 255, delay for 500 ms
            digitalWrite(_cs, 1);
            //SPI.endTransaction();
            sleep_ms(ms);
            //SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE));
            digitalWrite(_cs, 0);      
          }     
        }
      }
      digitalWrite(_cs, 1);
      break;
  }	


  return(GFX_OK);
}

void PICO_DSP::end()
{
}

gfx_mode_t PICO_DSP::getMode(void)
{
  return gfxmode;
}

void PICO_DSP::flipscreen(bool flip)
{
  digitalWrite(_dc, 0);
  digitalWrite(_cs, 0);
  SPItransfer(TFT_MADCTL);
  digitalWrite(_dc, 1);
  if (flip) {
    flipped=true;

#ifdef ILI9341         
    SPItransfer(TFT_MADCTL_MV | TFT_MADCTL_BGR);
#endif
#ifdef ILI9488P
    SPItransfer(TFT_MADCTL_MV | TFT_MADCTL_BGR);
#endif
#ifdef ST7789
#ifdef ROTATE_SCREEN
    SPItransfer(TFT_MADCTL_RGB);
#else
    SPItransfer(TFT_MADCTL_MY | TFT_MADCTL_MV |TFT_MADCTL_RGB);
#endif
#endif 
  }
  else {
    flipped=false;
   
#ifdef ILI9341        
    SPItransfer(TFT_MADCTL_MX | TFT_MADCTL_MY | TFT_MADCTL_MV | TFT_MADCTL_BGR);
#endif
#ifdef ST7789
#ifdef ROTATE_SCREEN
    SPItransfer(TFT_MADCTL_MX | TFT_MADCTL_MY | TFT_MADCTL_RGB);
#else
    SPItransfer(TFT_MADCTL_MX | TFT_MADCTL_MV | TFT_MADCTL_RGB);
#endif
#endif  
  }
  digitalWrite(_cs, 1);   
}


bool PICO_DSP::isflipped(void)
{
  return(flipped);
}
  

/***********************************************************************************************
    DMA functions
 ***********************************************************************************************/
static void dma_isr() { 
  irq_clear(DMA_IRQ_0);
  dma_hw->ints0 = 1u << dma_tx;
  curTransfer++;
  if (curTransfer >= nbTransfer) {
    curTransfer = 0;
  }
  if (cancelled) {
    rstop = 1;
  }
  else 
  {
    dma_channel_transfer_from_buffer_now(dma_tx, blocks[curTransfer], blocklens[curTransfer]);
  }  
}

static void setDmaStruct() {
  // Setup the control channel
  if (dma_tx == 0)  {
    dma_tx = dma_claim_unused_channel(true);
  }    
  dmaconfig = dma_channel_get_default_config(dma_tx);
  channel_config_set_transfer_data_size(&dmaconfig, DMA_SIZE_16);
  channel_config_set_dreq(&dmaconfig, TFT_SPIDREQ);  
  //channel_config_set_read_increment(&dmaconfig, true); // read incrementing
  //channel_config_set_write_increment(&dmaconfig, false); // no write incrementing

  dma_channel_configure(
      dma_tx,
      &dmaconfig,
      &spi_get_hw(TFT_SPIREG)->dr, // write address
      blocks[0],
      blocklens[0],
      false
  ); 

  irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);
  dma_channel_set_irq0_enabled(dma_tx, true);
  irq_set_enabled(DMA_IRQ_0, true);
  dma_hw->ints0 = 1u << dma_tx;  
}

void PICO_DSP::startRefresh(void) {
#ifdef PICOCALC
  // PICOCALC uses direct scanline pushes; skip DMA refresh setup
    return;
#endif
  if (gfxmode == MODE_TFT_320x240) {
    uint32_t remaining = TFT_HEIGHT*TFT_WIDTH*2;
    int i=0;
    nbTransfer = 0;
    while (remaining > 0) {
      uint16_t * fb = blocks[i];
      int32_t len = (remaining >= (LINES_PER_BLOCK*TFT_WIDTH*2)?LINES_PER_BLOCK*TFT_WIDTH*2:remaining);     
      switch (i) {
        case 0:
          if (fb == 0) fb = (uint16_t*)((int)malloc(len+64)&0xffffffe0);       
          break;
        case 1:
          if (fb == 0) fb = (uint16_t*)((int)malloc(len+64)&0xffffffe0);      
          break;
        case 2:
          if (fb == 0) fb = (uint16_t*)((int)malloc(len+64)&0xffffffe0);      
          break;
        case 3:
          if (fb == 0) fb = (uint16_t*)((int)malloc(len+64)&0xffffffe0);       
          break;
      }
      blocks[i] = fb;
      blocklens[i] = len/2;
      if (blocks[i] == 0) {
        fillScreenNoDma(RGBVAL(0xFF,0xFF,0x00));
        printf("FB allocaltion failed for block %d\n",i);
        sleep_ms(10000);    
      }
      nbTransfer++;
      remaining -= len;    
      i++;
    }                
    curTransfer = 0;  
    rstop = 0;     
    digitalWrite(_cs, 1);
    setDmaStruct();
    fillScreen(RGBVAL(0x00,0x00,0x00));
    digitalWrite(_cs, 0);

    setArea((TFT_REALWIDTH-TFT_WIDTH)/2, (TFT_REALHEIGHT-TFT_HEIGHT)/2, (TFT_REALWIDTH-TFT_WIDTH)/2 + TFT_WIDTH-1, (TFT_REALHEIGHT-TFT_HEIGHT)/2+TFT_HEIGHT-1);  
    // we switch to 16bit mode!!
    spi_set_format(TFT_SPIREG, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    dma_start_channel_mask(1u << dma_tx);    
  }
  else { 
    fillScreen(RGBVAL(0x00,0x00,0x00));   
  }
}

void PICO_DSP::stopRefresh(void) {
  if (gfxmode == MODE_TFT_320x240) {
    rstop = 1;
    unsigned long m = time_us_32()*1000;   
    cancelled = true; 
    while (!rstop)  {
      if ((time_us_32()*1000 - m) > 100) break;
      sleep_ms(100);
      asm volatile("wfi");
    };
    rstop = 0;
    sleep_ms(100);
    cancelled = false;  
    //dmatx.detachInterrupt();
    fillScreen(RGBVAL(0x00,0x00,0x00));
    digitalWrite(_cs, 1);
    // we switch back to GFX mode!!
    begin(gfxmode);
    setArea(0, 0, TFT_REALWIDTH-1, TFT_REALHEIGHT-1);    
  }
}


/***********************************************************************************************
    GFX functions
 ***********************************************************************************************/
// retrieve size of the frame buffer
int PICO_DSP::get_frame_buffer_size(int *width, int *height)
{
  if (width != nullptr) *width = fb_width;
  if (height != nullptr) *height = fb_height;
  return fb_stride;
}

void PICO_DSP::waitSync()
{
  if (gfxmode == MODE_TFT_320x240) {
  }
  else { 
    WaitVSync();
  }
}

void PICO_DSP::waitLine(int line)
{
  if (gfxmode == MODE_TFT_320x240) {
  }
  else { 
//  while (currentLine != line) {};
  }
}


/***********************************************************************************************
    GFX functions
 ***********************************************************************************************/
/*
vga_pixel * PICO_DSP::getLineBuffer(int j) {
  return (&framebuffer[j*fb_stride]);
}
*/

void PICO_DSP::fillScreen(dsp_pixel color) {
  int i,j;
  if (gfxmode == MODE_TFT_320x240) {
    for (j=0; j<TFT_HEIGHT; j++)
    {
      uint16_t * block=blocks[j>>6];
      uint16_t * dst=&block[(j&0x3F)*fb_stride];
      for (i=0; i<fb_width; i++)
      {
        *dst++ = color;
      }
    }
  }
  else {
    vga_pixel color8 = VGA_RGB(R16(color),G16(color),B16(color));
    for (j=0; j<fb_height; j++)
    {
      vga_pixel * dst=&framebuffer[j*fb_stride];
      for (i=0; i<fb_width; i++)
      {
        *dst++ = color8;
      }
    }    
  }  
}

void PICO_DSP::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, dsp_pixel color) {
  int i,j,l=y;
  if (gfxmode == MODE_TFT_320x240) {
    for (j=0; j<h; j++)
    {
      uint16_t * block=blocks[l>>6];
      uint16_t * dst=&block[(l&0x3F)*fb_stride+x];
      for (i=0; i<w; i++)
      {
        *dst++ = color;
      }
      l++;
    }
  }
  else {
    vga_pixel color8 = VGA_RGB(R16(color),G16(color),B16(color));
    for (j=0; j<h; j++)
    {
      vga_pixel * dst=&framebuffer[l*fb_stride+x];
      for (i=0; i<w; i++)
      {
        *dst++ = color8;
      }
      l++;
    }
  }  
}

void PICO_DSP::drawText(int16_t x, int16_t y, const char * text, dsp_pixel fgcolor, dsp_pixel bgcolor, bool doublesize) {
  if (gfxmode == MODE_TFT_320x240) {
    uint16_t c;
    uint16_t * block;
    uint16_t * dst;
    fgcolor = fgcolor;
    bgcolor = bgcolor;
    while ((c = *text++)) {
      const unsigned char * charpt=&font8x8[c][0];
      int l=y;
      for (int i=0;i<8;i++)
      {     
        unsigned char bits;
        if (doublesize) {
          block=blocks[l>>6];
          dst=&block[(l&0x3F)*fb_stride+x];         
          bits = *charpt;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor;
          else *dst++=bgcolor;
          l++;       
        }
        block=blocks[l>>6];
        dst=&block[(l&0x3F)*fb_stride+x]; 
        bits = *charpt++;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor;
        else *dst++=bgcolor;
        l++;
      }
      x +=8;
    }  
  }
  else {
    vga_pixel fgcolor8 = VGA_RGB(R16(fgcolor),G16(fgcolor),B16(fgcolor));
    vga_pixel bgcolor8 = VGA_RGB(R16(bgcolor),G16(bgcolor),B16(bgcolor));
    vga_pixel c;
    vga_pixel * dst;
    while ((c = *text++)) {
      const unsigned char * charpt=&font8x8[c][0];
      int l=y;
      for (int i=0;i<8;i++)
      {     
        unsigned char bits;
        if (doublesize) {
          dst=&framebuffer[l*fb_stride+x];
          bits = *charpt;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          bits = bits >> 1;     
          if (bits&0x01) *dst++=fgcolor8;
          else *dst++=bgcolor8;
          l++;
        }
        dst=&framebuffer[l*fb_stride+x]; 
        bits = *charpt++;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        bits = bits >> 1;     
        if (bits&0x01) *dst++=fgcolor8;
        else *dst++=bgcolor8;
        l++;
      }
      x +=8;
    }
  }  
}

void PICO_DSP::drawSprite(int16_t x, int16_t y, const dsp_pixel *bitmap, uint16_t arx, uint16_t ary, uint16_t arw, uint16_t arh)
{
  int bmp_offx = 0;
  int bmp_offy = 0;
  uint16_t *bmp_ptr;
  int w =*bitmap++;
  int h = *bitmap++;
  if ( (arw == 0) || (arh == 0) ) {
    // no crop window
    arx = x;
    ary = y;
    arw = w;
    arh = h;
  }
  else {
    if ( (x>(arx+arw)) || ((x+w)<arx) || (y>(ary+arh)) || ((y+h)<ary)   ) {
      return;
    }
    // crop area
    if ( (x > arx) && (x<(arx+arw)) ) { 
      arw = arw - (x-arx);
      arx = arx + (x-arx);
    } else {
      bmp_offx = arx;
    }
    if ( ((x+w) > arx) && ((x+w)<(arx+arw)) ) {
      arw -= (arx+arw-x-w);
    }  
    if ( (y > ary) && (y<(ary+arh)) ) {
      arh = arh - (y-ary);
      ary = ary + (y-ary);
    } else {
      bmp_offy = ary;
    }
    if ( ((y+h) > ary) && ((y+h)<(ary+arh)) ) {
      arh -= (ary+arh-y-h);
    }     
  }
  int l=ary;
  bitmap = bitmap + bmp_offy*w + bmp_offx;


  if (gfxmode == MODE_TFT_320x240) {
    for (int row=0;row<arh; row++)
    {
      uint16_t * block=blocks[l>>6];
      uint16_t * dst=&block[(l&0x3F)*fb_stride+arx];  
      bmp_ptr = (uint16_t*)bitmap;
      for (int col=0;col<arw; col++)
      {
          *dst++ = *bmp_ptr++;            
      } 
      bitmap += w;
      l++;
    } 
  }
  else {
    for (int row=0;row<arh; row++)
    {
      vga_pixel * dst=&framebuffer[l*fb_stride+arx];  
      bmp_ptr = (uint16_t *)bitmap;
      for (int col=0;col<arw; col++)
      {
          uint16_t pix= *bmp_ptr++;
          *dst++ = VGA_RGB(R16(pix),G16(pix),B16(pix));
      } 
      bitmap += w;
      l++;
    }     
  }  

}

void PICO_DSP::drawSprite(int16_t x, int16_t y, const dsp_pixel *bitmap) {
    drawSprite(x,y,bitmap, 0,0,0,0);
}

void PICO_DSP::writeLine(int width, int height, int y, dsp_pixel *buf) {
#ifdef PICOCALC
  if (gfxmode == MODE_TFT_320x240) {
    int drawWidth = width > 320 ? 320 : width;
    int xOffset = 0; // Assume full width; adjust if you want centering
    int panelYOffset = 0;
#ifdef ILI9488
    if (LCD_HEIGHT > fb_height) panelYOffset = (LCD_HEIGHT - fb_height) / 2; // vertical centering (e.g. 40)
#endif
    int physY = y + panelYOffset;

    // Fallback to original per-line approach to restore functionality
    for (int i = 0; i < drawWidth; i++) {
      uint16_t val = *buf++; // RGB565
      int idx = i * 2;
      picocalc_linebuf[idx]     = val >> 8;      // high byte
      picocalc_linebuf[idx + 1] = val & 0xFF;    // low byte
    }
    // Send one scanline window
    define_region_spi(xOffset, physY, xOffset + drawWidth - 1, physY, 1);
    hw_send_spi(picocalc_linebuf, drawWidth * 2);
    lcd_spi_raise_cs();

    return; // done for PICOCALC path
  }
#endif
  if (gfxmode == MODE_TFT_320x240) {
    uint16_t * block=blocks[y>>6];
    uint16_t * dst=&block[(y&0x3F)*fb_stride];
    if (width > fb_width) {
#ifdef TFT_LINEARINT   
      int delta = (width/(width-fb_width))-1;   
      int pos = delta;
      for (int i=0; i<fb_width; i++)
      {
        uint16_t val = *buf++;
        pos--;      
        if (pos == 0) {
#ifdef LINEARINT_HACK        
          val  = ((uint32_t)*buf++ + val)/2;
#else
          uint16_t val2 = *buf++;
          val = RGBVAL((R16(val)+R16(val2))/2,(G16(val)+G16(val2))/2,(B16(val)+B16(val2))/2);
 #endif        
          pos = delta;
        }
        *dst++=val;
      }
  #else
      int step = ((width << 8)/fb_width);
      int pos = 0;
      for (int i=0; i<fb_width; i++)
      {
        *dst++=buf[pos >> 8];
        pos +=step;
      }  
  #endif       
    }
    else if ((width*2) == fb_width) 
    {
      for (int i=0; i<width; i++)
      {
        *dst++=*buf;
        *dst++=*buf++;
      }       
    }
    else
    {
      if (width <= fb_width) {
        dst += (fb_width-width)/2;
      }
      for (int i=0; i<width; i++)
      {
        *dst++=*buf++;
      }       
    }    
  }
  else {
    if ( (height<fb_height) && (height > 2) ) y += (fb_height-height)/2;
    vga_pixel * dst=&framebuffer[y*fb_stride];    
    if (width > fb_width) {
      int step = ((width << 8)/fb_width);
      int pos = 0;
      for (int i=0; i<fb_width; i++)
      {
        uint16_t pix = buf[pos >> 8];
        *dst++ = VGA_RGB(R16(pix),G16(pix),B16(pix)); 
        pos +=step;
      }        
    }
    else if ((width*2) == fb_width) {
      for (int i=0; i<width; i++)
      {
        uint16_t pix = *buf++;
        vga_pixel col = VGA_RGB(R16(pix),G16(pix),B16(pix));
        *dst++= col;
        *dst++= col;
      }       
    }
    else {
      if (width <= fb_width) {
        dst += (fb_width-width)/2;
      }
      for (int i=0; i<width; i++)
      {
        uint16_t pix = *buf++;
        *dst++= VGA_RGB(R16(pix),G16(pix),B16(pix));
      }      
    }
  }  
}

void PICO_DSP::writeLinePal(int width, int height, int y, uint8_t *buf, dsp_pixel *palette) {
#ifdef PICOCALC
  if (gfxmode == MODE_TFT_320x240) {
    // Initialize DMA on first use
    static bool dma_initialized = false;
    if (!dma_initialized) {
      picocalc_dma_init();
      dma_initialized = true;
    }
    
    // Vertically center if needed within the emulated framebuffer height
    if ((height < fb_height) && (height > 2)) y += (fb_height - height) / 2;
    if (y < 0 || y >= fb_height) return;
    int drawWidth = width;
    if (drawWidth > fb_width) drawWidth = fb_width; // simple clip; could scale but not required for C64 (320)
    // Horizontal centering if source narrower than target
    int xOffset = 0;
    if (drawWidth < fb_width) xOffset = (fb_width - drawWidth)/2;
    // Center within physical 320x320 panel if height is 240 (add vertical margin)
#ifdef ILI9488
    int panelYOffset = 0;
    if (TFT_HEIGHT == 240) {
      panelYOffset = (LCD_HEIGHT - TFT_HEIGHT) / 2; // typically 40
    } else if (LCD_HEIGHT > fb_height) {
      panelYOffset = (LCD_HEIGHT - fb_height)/2;
    }
    int physY = y + panelYOffset;
#else
    int physY = y;
#endif
    
    // Get current line buffer for data preparation
    uint8_t* line_buffer = picocalc_get_line_buffer();
    
    // Convert palette indices to RGB565 (can overlap with previous DMA)
    for (int i = 0; i < drawWidth; i++) {
      uint16_t val = palette[buf[i]]; // RGB565
      int idx = i * 2;
      line_buffer[idx]     = val >> 8;
      line_buffer[idx + 1] = val & 0xFF;
    }
    
    // Wait for any previous DMA transfer to complete before setting up new window
    picocalc_dma_wait();
    
    // Set up window for this line
    define_region_spi(xOffset, physY, xOffset + drawWidth - 1, physY, 1);
    
    // Start DMA transfer and swap to next buffer (allows overlap with next line prep)
    picocalc_send_and_swap(drawWidth * 2);
    
    // Only wait for completion on the last line of the frame
    // For middle lines, the next line will wait, providing better overlap
    if (y >= height - 1) {
      picocalc_dma_wait();
    }
    lcd_spi_raise_cs();
    
    return;
  }
#endif
  if (gfxmode == MODE_TFT_320x240) {
    if ( (height<fb_height) && (height > 2) ) y += (fb_height-height)/2;
    uint16_t * block=blocks[y>>6];
    uint16_t * dst=&block[(y&0x3F)*fb_stride];
    if (width > fb_width) {
#ifdef TFT_LINEARINT    
      int delta = (width/(width-fb_width))-1;
      int pos = delta;
      for (int i=0; i<fb_width; i++)
      {
        uint16_t val = palette[*buf++];
        pos--;
        if (pos == 0) {
#ifdef LINEARINT_HACK
          val  = ((uint32_t)palette[*buf++] + val)/2;
#else
          uint16_t val2 = *buf++;
          val = RGBVAL((R16(val)+R16(val2))/2,(G16(val)+G16(val2))/2,(B16(val)+B16(val2))/2);
#endif        
          pos = delta;
        }
        *dst++=val;
      }
#else
      int step = ((width << 8)/fb_width);
      int pos = 0;
      for (int i=0; i<fb_width; i++)
      {
        *dst++=palette[buf[pos >> 8]];
        pos +=step;
      }  
#endif
    }
    else if ((width*2) == fb_width) {
      for (int i=0; i<width; i++)
      {
        *dst++=palette[*buf];
        *dst++=palette[*buf++];
      } 
    }
    else {
      if (width <= fb_width) {
        dst += (fb_width-width)/2;
      }
      for (int i=0; i<width; i++)
      {
        *dst++=palette[*buf++];
      } 
    }    
  }  
  else {
    if ( (height<fb_height) && (height > 2) ) y += (fb_height-height)/2;
    vga_pixel * dst=&framebuffer[y*fb_stride];
    if (width > fb_width) {
      int step = ((width << 8)/fb_width);
      int pos = 0;
      for (int i=0; i<fb_width; i++)
      {
        uint16_t pix = palette[buf[pos >> 8]];
        *dst++= VGA_RGB(R16(pix),G16(pix),B16(pix));
        pos +=step;
      }  
    }
    else if ((width*2) == fb_width) {
      for (int i=0; i<width; i++)
      {
        uint16_t pix = palette[*buf++];
        *dst++= VGA_RGB(R16(pix),G16(pix),B16(pix));
        *dst++= VGA_RGB(R16(pix),G16(pix),B16(pix));
      } 
    }
    else {
      if (width <= fb_width) {
        dst += (fb_width-width)/2;
      }
      for (int i=0; i<width; i++)
      {
        uint16_t pix = palette[*buf++];
        *dst++= VGA_RGB(R16(pix),G16(pix),B16(pix));
      } 
    }
  }
}

void PICO_DSP::writeScreenPal(int width, int height, int stride, uint8_t *buf, dsp_pixel *palette16) {
  uint8_t *src; 
  int i,j,y=0;
  int sy = 0;  
  int systep=(1<<8);
  int h = height;
  if (height <= ( (2*fb_height)/3)) {
    systep=(systep*height)/fb_height;
    h = fb_height;
  }  
  if (gfxmode == MODE_TFT_320x240) {
    if (width*2 <= fb_width) {
      for (j=0; j<h; j++)
      {
        uint16_t * block=blocks[y>>6];
        uint16_t * dst=&block[(y&0x3F)*fb_stride];        
        src=&buf[(sy>>8)*stride];
        for (i=0; i<width; i++)
        {
          uint16_t val = palette16[*src++];
          *dst++ = val;
          *dst++ = val;
        }
        y++;
        sy+=systep;  
      }
    }
    else if (width <= fb_width) {
      for (j=0; j<h; j++)
      {
        uint16_t * block=blocks[y>>6];
        uint16_t * dst=&block[(y&0x3F)*fb_stride+(fb_width-width)/2];        
        src=&buf[(sy>>8)*stride];
        for (i=0; i<width; i++)
        {
          uint16_t val = palette16[*src++];
          *dst++ = val;
        }
        y++;
        sy+=systep;  
      }
    }
  }       
  else { // VGA
    if (width*2 <= fb_width) {
      for (j=0; j<h; j++)
      {
        vga_pixel * dst=&framebuffer[y*fb_stride];                
        src=&buf[(sy>>8)*stride];
        for (i=0; i<width; i++)
        {
          uint16_t pix = palette16[*src++];
          *dst++ = VGA_RGB(R16(pix),G16(pix),B16(pix));
          *dst++ = VGA_RGB(R16(pix),G16(pix),B16(pix));
        }
        y++;
        sy+=systep;  
      }
    }
    else if (width <= fb_width) {
      for (j=0; j<h; j++)
      {
        vga_pixel * dst=&framebuffer[y*fb_stride+(fb_width-width)/2];                
        src=&buf[(sy>>8)*stride];
        for (i=0; i<width; i++)
        {
          uint16_t pix = palette16[*src++];
          *dst++ = VGA_RGB(R16(pix),G16(pix),B16(pix));
        }
        y++;
        sy+=systep;  
      }
    }
  }
}


/***********************************************************************************************
    No DMA functions
 ***********************************************************************************************/
void PICO_DSP::fillScreenNoDma(dsp_pixel color) {
  if (gfxmode == MODE_TFT_320x240) {
#ifdef PICOCALC
    // Use working LCD driver for PICOCALC  
    // Convert 16-bit color to 24-bit RGB for working driver
    int r = (color >> 11) & 0x1F;
    int g = (color >> 5) & 0x3F; 
    int b = color & 0x1F;
    // Scale to full 8-bit range
    r = (r * 255) / 31;
    g = (g * 255) / 63;
    b = (b * 255) / 31;
    int rgb24 = RGB(r, g, b);
    
    draw_rect_spi(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1, rgb24);
    return;
#endif
    
    digitalWrite(_cs, 0);
    setArea(0, 0, TFT_REALWIDTH-1, TFT_REALHEIGHT-1);  
    int i,j;
    for (j=0; j<TFT_REALHEIGHT; j++)
    {
      for (i=0; i<TFT_REALWIDTH; i++) {
        //digitalWrite(_dc, 1);
        SPItransferPixel(color);     
      }
    }
#ifdef ILI9341  
    digitalWrite(_dc, 0);
    SPItransfer(ILI9341_SLPOUT);
    digitalWrite(_dc, 1);
#endif
    digitalWrite(_cs, 1);
    setArea(0, 0, (TFT_REALWIDTH-1), (TFT_REALHEIGHT-1));     
  }
  else {
    fillScreen(color);
  }   
}

void PICO_DSP::drawRectNoDma(int16_t x, int16_t y, int16_t w, int16_t h, dsp_pixel color) {
  if (gfxmode == MODE_TFT_320x240) {
    digitalWrite(_cs, 0);
    setArea(x,y,x+w-1,y+h-1);
    int i;
    for (i=0; i<(w*h); i++)
    {
      SPItransferPixel(color);
    }
#ifdef ILI9341  
    digitalWrite(_dc, 0);
    SPItransfer(ILI9341_SLPOUT);
    digitalWrite(_dc, 1);
#endif
    digitalWrite(_cs, 1);
    setArea(0, 0, (TFT_REALWIDTH-1), (TFT_REALHEIGHT-1));   
  }
  else {
    drawRect(x, y, w, h, color);
  }  
}


void PICO_DSP::drawSpriteNoDma(int16_t x, int16_t y, const dsp_pixel *bitmap) {  
  drawSpriteNoDma(x,y,bitmap, 0,0,0,0);
}

void PICO_DSP::drawSpriteNoDma(int16_t x, int16_t y, const dsp_pixel *bitmap, uint16_t arx, uint16_t ary, uint16_t arw, uint16_t arh)
{
  if (gfxmode == MODE_TFT_320x240) {
    int bmp_offx = 0;
    int bmp_offy = 0;
    uint16_t *bmp_ptr;
    int w =*bitmap++;
    int h =*bitmap++;
    if ( (arw == 0) || (arh == 0) ) {
      // no crop window
      arx = x;
      ary = y;
      arw = w;
      arh = h;
    }
    else {
      if ( (x>(arx+arw)) || ((x+w)<arx) || (y>(ary+arh)) || ((y+h)<ary)   ) {
        return;
      }
      // crop area
      if ( (x > arx) && (x<(arx+arw)) ) { 
        arw = arw - (x-arx);
        arx = arx + (x-arx);
      } else {
        bmp_offx = arx;
      }
      if ( ((x+w) > arx) && ((x+w)<(arx+arw)) ) {
        arw -= (arx+arw-x-w);
      }  
      if ( (y > ary) && (y<(ary+arh)) ) {
        arh = arh - (y-ary);
        ary = ary + (y-ary);
      } else {
        bmp_offy = ary;
      }
      if ( ((y+h) > ary) && ((y+h)<(ary+arh)) ) {
        arh -= (ary+arh-y-h);
      }     
    }
    digitalWrite(_cs, 0);
    setArea(arx, ary, arx+arw-1, ary+arh-1);        
    bitmap = bitmap + bmp_offy*w + bmp_offx;
    for (int row=0;row<arh; row++)
    {
      bmp_ptr = (uint16_t*)bitmap;
      for (int col=0;col<arw; col++)
      {
          SPItransferPixel(*bmp_ptr++);             
      } 
      bitmap +=  w;
    }
#ifdef ILI9341  
    digitalWrite(_dc, 0);
    SPItransfer(ILI9341_SLPOUT);
    digitalWrite(_dc, 1);
#endif
    setArea(0, 0, TFT_REALWIDTH-1, TFT_REALHEIGHT-1);  
    digitalWrite(_cs, 1);  
  }
  else {
    drawSprite(x, y, bitmap, arx, ary, arw, arh);
  }   
}

void PICO_DSP::drawTextNoDma(int16_t x, int16_t y, const char * text, dsp_pixel fgcolor, dsp_pixel bgcolor, bool doublesize) {
  if (gfxmode == MODE_TFT_320x240) {
#ifdef PICOCALC
    // Use working LCD driver's native text functions
    // Convert colors from 16-bit to 24-bit RGB
    int fg_r = (fgcolor >> 11) & 0x1F;
    int fg_g = (fgcolor >> 5) & 0x3F; 
    int fg_b = fgcolor & 0x1F;
    fg_r = (fg_r * 255) / 31;
    fg_g = (fg_g * 255) / 63;
    fg_b = (fg_b * 255) / 31;
    int fg_rgb24 = RGB(fg_r, fg_g, fg_b);
    
    int bg_r = (bgcolor >> 11) & 0x1F;
    int bg_g = (bgcolor >> 5) & 0x3F; 
    int bg_b = bgcolor & 0x1F;
    bg_r = (bg_r * 255) / 31;
    bg_g = (bg_g * 255) / 63;
    bg_b = (bg_b * 255) / 31;
    int bg_rgb24 = RGB(bg_r, bg_g, bg_b);
    
    // Set cursor position and colors, then print the string
    lcd_set_cursor(x, y);
    lcd_set_text_colors(fg_rgb24, bg_rgb24);
    lcd_print_string((char*)text);
    return;
#endif
    
    uint16_t c;
    while ((c = *text++)) {
      const unsigned char * charpt=&font8x8[c][0];
      digitalWrite(_cs, 0);
      setArea(x,y,x+7,y+(doublesize?15:7));
      for (int i=0;i<8;i++)
      {
        unsigned char bits;
        if (doublesize) {
          bits = *charpt;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);
          bits = bits >> 1;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);
          bits = bits >> 1;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);
          bits = bits >> 1;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);
          bits = bits >> 1;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);
          bits = bits >> 1;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);
          bits = bits >> 1;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);
          bits = bits >> 1;     
          if (bits&0x01) SPItransferPixel(fgcolor);
          else SPItransferPixel(bgcolor);       
        }
        bits = *charpt++;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
        bits = bits >> 1;     
        if (bits&0x01) SPItransferPixel(fgcolor);
        else SPItransferPixel(bgcolor);
      }
      x +=8;
#ifdef ILI9341  
      digitalWrite(_dc, 0);
      SPItransfer(ILI9341_SLPOUT);
      digitalWrite(_dc, 1);
#endif
      digitalWrite(_cs, 1); 
    }

    digitalWrite(_cs, 0);
    setArea(0, 0, (TFT_REALWIDTH-1), (TFT_REALHEIGHT-1));  
    digitalWrite(_cs, 1);
  }
  else {
    drawText(x, y, text, fgcolor, bgcolor, doublesize);
  } 
}

/*******************************************************************
 Experimental PWM interrupt based sound driver !!!
*******************************************************************/
#include "hardware/irq.h"
#include "hardware/pwm.h"

static bool fillfirsthalf = true;
static uint16_t cnt = 0;
static uint16_t sampleBufferSize = 0;

static void (*fillsamples)(short * stream, int len) = nullptr;

static uint32_t * i2s_tx_buffer;
static short * i2s_tx_buffer16;

static void SOFTWARE_isr() {
  if (fillfirsthalf) {
    fillsamples((short *)i2s_tx_buffer, sampleBufferSize);
  }  
  else { 
    fillsamples((short *)&i2s_tx_buffer[sampleBufferSize/2], sampleBufferSize);
  }
}

static void AUDIO_isr() {
  pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));
  long s = i2s_tx_buffer16[cnt++]; 
  s += i2s_tx_buffer16[cnt++];
  s = s/2 + 32767; 
  pwm_set_gpio_level(AUDIO_PIN, s >> 8);
  cnt = cnt & (sampleBufferSize*2-1);

  if (cnt == 0) {
    fillfirsthalf = false;
    //irq_set_pending(RTC_IRQ+1);
    multicore_fifo_push_blocking(0);
  } 
  else if (cnt == sampleBufferSize) {
    fillfirsthalf = true;
    //irq_set_pending(RTC_IRQ+1);
    multicore_fifo_push_blocking(0);
  }
}

static void core1_sio_irq() {
  irq_clear(SIO_IRQ_PROC1);
  while(multicore_fifo_rvalid()) {
    uint16_t raw = multicore_fifo_pop_blocking();
    SOFTWARE_isr();
  } 
  multicore_fifo_clear_irq();
}

static void core1_func_tft() {
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1,core1_sio_irq);
    //irq_set_priority (SIO_IRQ_PROC1, 129);    
    irq_set_enabled(SIO_IRQ_PROC1,true);

    while (true) {
        tight_loop_contents();
    }
}

void PICO_DSP::begin_audio(int samplesize, void (*callback)(short * stream, int len))
{
  fillsamples = callback;
  i2s_tx_buffer =  (uint32_t*)malloc(samplesize*sizeof(uint32_t));

  if (i2s_tx_buffer == NULL) {
    printf("sound buffer could not be allocated!!!!!\n");
    return;  
  }
  memset((void*)i2s_tx_buffer,0, samplesize*sizeof(uint32_t));
  printf("sound buffer allocated\n");

  i2s_tx_buffer16 = (short*)i2s_tx_buffer;

  sampleBufferSize = samplesize;

  gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

  if (gfxmode == MODE_TFT_320x240) {
    multicore_launch_core1(core1_func_tft);
  }
  
  int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);
  // Setup PWM interrupt to fire when PWM cycle is complete
  pwm_clear_irq(audio_pin_slice);
  pwm_set_irq_enabled(audio_pin_slice, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, AUDIO_isr);
  irq_set_priority (PWM_IRQ_WRAP, 128);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  //irq_set_exclusive_handler(RTC_IRQ+1,SOFTWARE_isr);
  //irq_set_priority (RTC_IRQ+1, 120);
  //irq_set_enabled(RTC_IRQ+1,true);


  // Setup PWM for audio output
  pwm_config config = pwm_get_default_config();
//  pwm_config_set_clkdiv(&config, 5.5f);
  pwm_config_set_clkdiv(&config, 50.0f);
  pwm_config_set_wrap(&config, 254);
  pwm_init(audio_pin_slice, &config, true);

  pwm_set_gpio_level(AUDIO_PIN, 0);
  printf("sound initialized\n");
}
 
void PICO_DSP::end_audio()
{
  if (i2s_tx_buffer != NULL) {
    free(i2s_tx_buffer);
  }  
}


 

