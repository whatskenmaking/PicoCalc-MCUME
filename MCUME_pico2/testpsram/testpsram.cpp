#include "pico.h"
#include "pico/stdlib.h"
#include <stdio.h>
extern "C" {
  #include "iopins.h"  
  #include "emuapi.h"  
}
#include "keyboard_osd.h"
#include "pico_dsp.h"

#define BLUE       RGBVAL16(0, 0, 170)
#define RED        RGBVAL16(255, 0, 0)
#define LIGHT_BLUE RGBVAL16(0, 136, 255)

//#define TEST 8
//#define TEST 16
#define TEST 32
//#define TEST 64

PICO_DSP tft;
static int fb_width, fb_height;


#include "hardware/clocks.h"
#include "hardware/vreg.h"

static const char * digits = "0123456789ABCDEF";
static int hk = 0;
static int prevhk = 0;
static int col=0;
static int row=0;

#ifdef HAS_PSRAM

#ifdef OLD_PSRAM
#include "psram_t.h"
PSRAM_T psram = PSRAM_T(PSRAM_CS, PSRAM_MOSI, PSRAM_SCLK, PSRAM_MISO);
#else
#include "psram_spi.h"
#endif

#endif


void emu_Input(uint16_t bClick) {
    hk = emu_ReadI2CKeyboard();
}

bool repeating_timer_callback(struct repeating_timer *t) {
    uint16_t bClick = emu_DebounceLocalKeys();
    emu_Input(bClick);    
    return true;
}


int main(void) {
//    vreg_set_voltage(VREG_VOLTAGE_1_05);
//    vreg_set_voltage(VREG_VOLTAGE_3_00);



//    set_sys_clock_khz(25000, true);    
//    set_sys_clock_khz(133000, true);    

//    set_sys_clock_khz(150000, true);    
//    set_sys_clock_khz(133000, true);    
//    set_sys_clock_khz(200000, true);    
//    set_sys_clock_khz(210000, true);    
////    set_sys_clock_khz(230000, true);    
//    set_sys_clock_khz(225000, true);    
//    set_sys_clock_khz(250000, true);  

//    set_sys_clock_khz(133000, true);    

    // Overclock!
    set_sys_clock_khz(280000, true);
//    set_sys_clock_khz(260000, true); // for PSRAM tolerance 230000-270000
    *((uint32_t *)(0x40010000+0x58)) = 2 << 16; //CLK_HSTX_DIV = 2 << 16; // HSTX clock/2

    emu_init();

#ifdef FILEBROWSER
    while (true) {    
        if (menuActive()) {
            uint16_t bClick = emu_DebounceLocalKeys();
            int action = handleMenu(bClick);
            char * filename = menuSelection();   
            if (action == ACTION_RUN) {
              break;  
            }  
            tft.waitSync();
        }
    }    
#endif
    emu_start();
	tft.startRefresh();

#ifdef HAS_PSRAM
#ifdef OLD_PSRAM
    psram.begin();
    psram.qspi_mode();
#else
    psram_init();
#endif    
#endif

    struct repeating_timer timer;
    add_repeating_timer_ms(20, repeating_timer_callback, NULL, &timer);      

	tft.fillScreen(LIGHT_BLUE);
	tft.get_frame_buffer_size(&fb_width, &fb_height);
	tft.drawRect((fb_width-320)/2,(fb_height-200)/2, 320,200, BLUE);



    char bufc[4] = {32,32,32,0};
	uint sys_clk = clock_get_hz(clk_sys)/1000000;
	uint r1 = sys_clk/100;
	uint r = sys_clk - r1*100;
	uint r2 = r/10;
	r = sys_clk - r1*100 - r2*10;
	uint r3 = r;
	bufc[0] = digits[r1];
	bufc[1] = digits[r2];
	bufc[2] = digits[r3];
	tft.drawText(8,8,bufc,BLUE,LIGHT_BLUE,false);

#ifdef HAS_PSRAM
    uint32_t addr = 0; //0xFFFF3F;
#if TEST == 8
    uint8_t val = 0; //0x3F;
#elif TEST == 16
    uint16_t val = 0; //0x3F;
#elif TEST == 32
    uint32_t val = 0; //0x3F;
#elif TEST == 64
    uint64_t val = 0; //0x3F;
#endif   
    char valpt[13];
#endif

    while (true) {
#ifdef HAS_PSRAM

#if TEST == 8
        valpt[0]=digits[(val>>4)&0xf];
        valpt[1]=digits[val&0xf];
        valpt[2]=0;
#ifdef OLD_PSRAM
        //psram.qspi_mode();
        psram.pswrite(addr,val);
        //psram.spi_mode();
        uint8_t rval = psram.psread(addr);
#else
        psram_write8(addr,val);
        uint8_t rval = psram_read8(addr);
#endif
#elif TEST == 16
        valpt[0]=digits[(val>>12)&0xf];
        valpt[1]=digits[(val>>8)&0xf];
        valpt[2]=digits[(val>>4)&0xf];
        valpt[3]=digits[val&0xf];
        valpt[4]=0;
#ifdef OLD_PSRAM
        //psram.qspi_mode();
        psram.pswrite_w(addr,val);
        //psram.spi_mode();
        uint16_t rval = psram.psread_w(addr);
#else
        psram_write16(addr,val);
        uint16_t rval = psram_read16(addr);
#endif
#elif TEST == 32
        valpt[0]=digits[(val>>28)&0xf];
        valpt[1]=digits[(val>>24)&0xf];
        valpt[2]=digits[(val>>20)&0xf];
        valpt[3]=digits[(val>>16)&0xf];
        valpt[4]=digits[(val>>12)&0xf];
        valpt[5]=digits[(val>>8)&0xf];
        valpt[6]=digits[(val>>4)&0xf];
        valpt[7]=digits[val&0xf];
        valpt[8]=0;
#ifdef OLD_PSRAM
#else
        psram_write32(addr,val);
        uint32_t rval = psram_read32(addr);
#endif
#elif TEST == 64
        valpt[0]=digits[(val>>60)&0xf];
        valpt[1]=digits[(val>>56)&0xf];
        valpt[2]=digits[(val>>52)&0xf];
        valpt[3]=digits[(val>>48)&0xf];
        valpt[4]=digits[(val>>44)&0xf];
        valpt[5]=digits[(val>>40)&0xf];
        valpt[6]=digits[(val>>36)&0xf];
        valpt[7]=digits[(val>>32)&0xf];
        valpt[8]=digits[(val>>28)&0xf];
        valpt[9]=digits[(val>>24)&0xf];
        valpt[10]=digits[(val>>20)&0xf];
        valpt[11]=digits[(val>>16)&0xf];
        valpt[12]=digits[(val>>12)&0xf];
        valpt[13]=digits[(val>>8)&0xf];
        valpt[14]=digits[(val>>4)&0xf];
        valpt[15]=digits[val&0xf];
        valpt[16]=0;
#ifdef OLD_PSRAM
#else
        psram_write64(addr,val);
        uint64_t rval = psram_read64(addr);
#endif 
#endif       
        if ( rval == val ) {
          tft.drawText((addr&0xF)*16,(3+((addr>>4)&0xF))*8,valpt,BLUE,LIGHT_BLUE,false);  
        }
        else {
          tft.fillScreen(RED);
          tft.drawRect((fb_width-320)/2,(fb_height-200)/2, 320,200, BLUE);
#if TEST == 8
          valpt[0]=digits[(rval>>4)&0xF];
          valpt[1]=digits[rval&0xf];
#elif TEST == 16
          valpt[0]=digits[(rval>>12)&0xf];
          valpt[1]=digits[(rval>>8)&0xf];
          valpt[2]=digits[(rval>>4)&0xf];
          valpt[3]=digits[rval&0xf];
#elif TEST == 32
          valpt[0]=digits[(rval>>28)&0xf];
          valpt[1]=digits[(rval>>24)&0xf];
          valpt[2]=digits[(rval>>20)&0xf];
          valpt[3]=digits[(rval>>16)&0xf];
          valpt[4]=digits[(rval>>12)&0xf];
          valpt[5]=digits[(rval>>8)&0xf];
          valpt[6]=digits[(rval>>4)&0xf];
          valpt[7]=digits[rval&0xf];
#elif TEST == 64
          valpt[0]=digits[(rval>>60)&0xf];
          valpt[1]=digits[(rval>>56)&0xf];
          valpt[2]=digits[(rval>>52)&0xf];
          valpt[3]=digits[(rval>>48)&0xf];
          valpt[4]=digits[(rval>>44)&0xf];
          valpt[5]=digits[(rval>>40)&0xf];
          valpt[6]=digits[(rval>>36)&0xf];
          valpt[7]=digits[(rval>>32)&0xf];
          valpt[8]=digits[(rval>>28)&0xf];
          valpt[9]=digits[(rval>>24)&0xf];
          valpt[10]=digits[(rval>>20)&0xf];
          valpt[11]=digits[(rval>>16)&0xf];
          valpt[12]=digits[(rval>>12)&0xf];
          valpt[13]=digits[(rval>>8)&0xf];
          valpt[14]=digits[(rval>>4)&0xf];
          valpt[15]=digits[rval&0xf];
#endif

          tft.drawText((addr&0xF)*16,(3+((addr>>4)&0xF))*8,valpt,RED,LIGHT_BLUE,false);  
          //while (true) {}
        } 

#if TEST == 8       
        addr = (addr+1) & 0xFFFFFF;
        val = val+1;
        if ((addr& 0xFF)==0) val++;    
#elif TEST == 16
        addr = (addr+2) & 0xFFFFFF;
        val = val+1;
#elif TEST == 32
        addr = (addr+4) & 0xFFFFFF;
        val = val+1;
#elif TEST == 64
        addr = (addr+8) & 0xFFFFFF;
        val = val+1;
#endif



#endif        
        uint16_t bClick = emu_GetPad();
        char buf[5] = {0,0,0,0,0};
        buf[0] = digits[(bClick>>12)&0xf];
        buf[1] = digits[(bClick>>8)&0xf];
        buf[2] = digits[(bClick>>4)&0xf];
        buf[3] = digits[bClick&0xf];
        tft.drawText(4*8,0,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),false);
        buf[3] = 0;
        int key = emu_ReadI2CKeyboard();
        buf[0] = digits[(key>>8)&0xf];
        buf[1] = digits[(key>>4)&0xf];
        buf[2] = digits[key&0xf];        
        tft.drawText(4*8,8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),false);

        buf[2] = 0;
        key = emu_ReadI2CKeyboard2(0);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(9*8+0*24,0*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(1);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(9*8+1*24,0*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(2);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(9*8+2*24,0*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(3);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(9*8+3*24,0*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(4);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(9*8+4*24,0*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(5);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(9*8+5*24,0*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
#ifdef PICOZX        
        key = emu_ReadI2CKeyboard2(6);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(9*8+6*24,0*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
#endif
        if ( (hk != 0) && (hk < 128) ) {
            buf[0] = (char)(hk&0xff);  
            buf[1] = 0;
            tft.drawText(col*8,(row+3)*8,buf,LIGHT_BLUE,BLUE,false);
            col += 1;
            if (col >= 40) {
                col=0;
                row += 1;
                if (row >= 25) {
                    row=0;  
                }
            }
            if (hk != prevhk) {
                sleep_ms(200);
            }
            else {
                sleep_ms(100);
            }
        }
        prevhk = hk;
        //sleep_ms(2);
    }
}

void emu_KeyboardOnDown(int keymodifer, int key) {
}

void emu_KeyboardOnUp(int keymodifer, int key) {
}




