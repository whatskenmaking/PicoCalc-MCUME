#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include <string.h>

extern "C" {
  #include "iopins.h"  
  #include "emuapi.h"
  #include "i2ckbd.h"
}
#include "keyboard_osd.h"
#include "c64.h"
#include "timerutil.h"

#include <stdio.h>
#include "pico_dsp.h"

extern "C" {
  #include "../display/lcdspi/lcdspi.h"
}

volatile bool vbl=true;
static bool enableKeyboard = true;

bool repeating_timer_callback(struct repeating_timer *t) {

// PicoCalc does not use a joystick, so just read the keyboard
#ifdef PICOCALC
    //uint16_t bClick = emu_DebounceLocalKeys();  // Read the joystick
    //if (bClick & MASK_KEY_USER1) enableKeyboard = !enableKeyboard;
    //if (enableKeyboard) emu_Input(bClick);       // Reads the keyboard and processes each keyboard press

    int key = emu_ReadI2CKeyboard();
    if (key == 144) { enableKeyboard = !enableKeyboard; }

    if (enableKeyboard) {
        picoCalc_Input(key);
    } else {
        if (key==97) picoCalc_Input(MASK_JOY2_UP);
        if (key==115) picoCalc_Input(MASK_JOY1_UP);
    }

#else
    uint16_t bClick = emu_DebounceLocalKeys();  // Read the joystick
    emu_Input(bClick);                          // Read and process the keyboard/joystick input
#endif 

    if (vbl) {
        vbl = false;
    } else {
        vbl = true;
    }   
    return true;
}

PICO_DSP tft;
static int skip=0;

// Frame buffer for decoupling VIC rendering from LCD output
// Store in byte-swapped format for direct DMA transfer to display
#define FB_WIDTH 320
#define FB_HEIGHT 240
static uint8_t frame_buffer[FB_WIDTH * FB_HEIGHT * 2]; // byte-swapped RGB565
static bool frame_ready = false;

// DMA infrastructure for frame buffer transfers
static int fb_dma_ch = -1;
static volatile bool fb_dma_busy = false;

// DMA interrupt handler for frame buffer transfers
static void fb_dma_isr() {
  uint32_t mask = 1u << fb_dma_ch;
  if (dma_hw->ints0 & mask) {
    dma_hw->ints0 = mask; // clear interrupt
    fb_dma_busy = false;
    lcd_spi_raise_cs(); // Close SPI transfer when DMA completes
  }
}

// Initialize DMA for frame buffer transfers
static void fb_dma_init() {
  if (fb_dma_ch >= 0) return; // already initialized
  
  fb_dma_ch = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(fb_dma_ch);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
  channel_config_set_dreq(&cfg, TFT_SPIDREQ);
  channel_config_set_read_increment(&cfg, true);
  channel_config_set_write_increment(&cfg, false);
  
  dma_channel_configure(fb_dma_ch, &cfg,
                        &spi_get_hw(TFT_SPIREG)->dr, // SPI data register
                        NULL, 0, false); // read_addr and count set per transfer
  
  // Set up interrupt handler - use DMA_IRQ_0 (SD card uses DMA_IRQ_1)
  irq_set_exclusive_handler(DMA_IRQ_0, fb_dma_isr);
  dma_channel_set_irq0_enabled(fb_dma_ch, true);
  irq_set_enabled(DMA_IRQ_0, true);
}

// Start DMA transfer of entire frame buffer
static void fb_dma_send_frame() {
  if (fb_dma_busy) return; // Previous transfer still in progress
  
  // Set up SPI window for entire frame (0,0) to (319,239)
  define_region_spi(0, 0, FB_WIDTH-1, FB_HEIGHT-1, 1);
  
  // Start DMA transfer - frame buffer is already in byte-swapped format
  fb_dma_busy = true;
  dma_channel_transfer_from_buffer_now(fb_dma_ch, frame_buffer, FB_WIDTH * FB_HEIGHT * 2);
}
static uint32_t frame_start_time = 0;

#include "hardware/clocks.h"
#include "hardware/vreg.h"

int main(void) {
//    vreg_set_voltage(VREG_VOLTAGE_1_05);
//    set_sys_clock_khz(125000, true);    
//    set_sys_clock_khz(150000, true);    
//    set_sys_clock_khz(133000, true);    
//    set_sys_clock_khz(200000, true);    
//    set_sys_clock_khz(210000, true);    
//    set_sys_clock_khz(230000, true);    
//    set_sys_clock_khz(225000, true);    
    set_sys_clock_khz(250000, true);  
    stdio_init_all();

    emu_init();
    
    // Initialize DMA for frame buffer transfers
    fb_dma_init();
    
    char * filename;
#ifdef FILEBROWSER
    while (true) {      
        if (menuActive()) {
            uint16_t bClick = emu_DebounceLocalKeys();
            int action = handleMenu(bClick);
            filename = menuSelection();   
            if (action == ACTION_RUN) {
              break;    
            }
            tft.waitSync();
        }
    }
#endif    
    emu_start();
    emu_Init(filename);
    tft.startRefresh();
    struct repeating_timer timer;
    add_repeating_timer_ms(25, repeating_timer_callback, NULL, &timer);    
    while (true) {
        //uint16_t bClick = emu_DebounceLocalKeys();
        //emu_Input(bClick);  
        emu_Step();        
    }
}

static unsigned short palette16[PALETTE_SIZE];
void emu_SetPaletteEntry(unsigned char r, unsigned char g, unsigned char b, int index)
{
    if (index<PALETTE_SIZE) {
        palette16[index]  = RGBVAL16(r,g,b);        
    }
}

void emu_DrawLinePal16(unsigned char * VBuf, int width, int height, int line) 
{
    if (skip == 0 && line >= 0 && line < FB_HEIGHT) {
        // Copy line data to frame buffer in byte-swapped format
        uint8_t* fb_line = &frame_buffer[line * FB_WIDTH * 2];
        
        // Optimized loop with reduced pointer arithmetic
        uint8_t* dst = fb_line;
        for (int x = 0; x < width && x < FB_WIDTH; x++) {
            uint16_t pixel = palette16[VBuf[x]];
            *dst++ = pixel >> 8;      // high byte first
            *dst++ = pixel & 0xFF;    // low byte
        }
    }
}

void emu_DrawLine16(unsigned short * VBuf, int width, int height, int line)
{
    if (skip == 0 && line >= 0 && line < FB_HEIGHT) {
        // Copy line data to frame buffer in byte-swapped format
        uint8_t* fb_line = &frame_buffer[line * FB_WIDTH * 2];
        int copy_width = (width < FB_WIDTH) ? width : FB_WIDTH;
        
        // Optimized byte swapping using 32-bit operations where possible
        uint32_t* dst32 = (uint32_t*)fb_line;
        uint32_t* src32 = (uint32_t*)VBuf;
        
        // Process pairs of pixels (4 bytes) at a time
        int pairs = copy_width >> 1;  // Bit shift instead of division by 2
        for (int i = 0; i < pairs; i++) {
            uint32_t two_pixels = src32[i];
            // Swap bytes within each 16-bit pixel: AABB CCDD -> BBAA DDCC
            dst32[i] = ((two_pixels & 0x00FF00FF) << 8) | ((two_pixels & 0xFF00FF00) >> 8);
        }
        
        // Handle odd pixel if width is odd
        if (copy_width & 1) {  // Bitwise AND instead of modulo
            uint16_t pixel = VBuf[copy_width - 1];
            uint8_t* last_pixel = &fb_line[(copy_width - 1) << 1];  // Bit shift for multiply by 2
            *last_pixel++ = pixel >> 8;    // high byte
            *last_pixel   = pixel & 0xFF;  // low byte
        }
    }
}

void emu_DrawVsync(void)
{
    skip += 1;
    skip &= VID_FRAME_SKIP;
    
    // Send frame buffer to LCD during VSync
    if (skip == 0) {
        fb_dma_send_frame();
    }
    
    volatile bool vb=vbl; 
    //while (vbl==vb) {};
#ifdef USE_VGA   
    //tft.waitSync();                   
#else 
    //while (vbl==vb) {};
#endif
}

/*
void emu_DrawLine8(unsigned char * VBuf, int width, int height, int line) 
{
    if (skip == 0) {
#ifdef USE_VGA                        
      tft.writeLine(width,height,line, VBuf);      
#endif      
    }
} 

void emu_DrawLine16(unsigned short * VBuf, int width, int height, int line) 
{
    if (skip == 0) {
#ifdef USE_VGA        
        tft.writeLine16(width,height,line, VBuf);
#else
        tft.writeLine(width,height,line, VBuf);
#endif        
    }
}  

void emu_DrawScreen(unsigned char * VBuf, int width, int height, int stride) 
{
    if (skip == 0) {
#ifdef USE_VGA                
        tft.writeScreen(width,height-TFT_VBUFFER_YCROP,stride, VBuf+(TFT_VBUFFER_YCROP/2)*stride, palette8);
#else
        tft.writeScreen(width,height-TFT_VBUFFER_YCROP,stride, VBuf+(TFT_VBUFFER_YCROP/2)*stride, palette16);
#endif
    }
}

int emu_FrameSkip(void)
{
    return skip;
}

void * emu_LineBuffer(int line)
{
    return (void*)tft.getLineBuffer(line);    
}
*/



#ifdef HAS_SND
#include "AudioPlaySystem.h"
AudioPlaySystem mymixer;

void emu_sndInit() {
  tft.begin_audio(256, mymixer.snd_Mixer);
  mymixer.start();    
}

void emu_sndPlaySound(int chan, int volume, int freq)
{
  if (chan < 6) {
    mymixer.sound(chan, freq, volume); 
  }
}

void emu_sndPlayBuzz(int size, int val) {
  mymixer.buzz(size,val); 
}

#endif

