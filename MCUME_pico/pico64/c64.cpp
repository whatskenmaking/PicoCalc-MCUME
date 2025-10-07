#include "pico.h"
#include "pico/stdlib.h"
#include <stdio.h>

extern "C" {
#include "emuapi.h"
#include "platform_config.h"
}

#include "Teensy64.h"
#include <string.h>

#ifdef HAS_SND
#include "reSID.h"
AudioPlaySID playSID;
#endif

using namespace std;

#ifndef PICOMPUTER
/*
static bool oskbActive=false;
*/
#endif

/* IRAM_ATTR */
static void oneRasterLine(void) {
  static unsigned short lc = 1;

  while (true) {

    cpu.lineStartTime = fbmicros(); //get_ccount();
    cpu.lineCycles = cpu.lineCyclesAbs = 0;

    if (!cpu.exactTiming) {
      vic_do();
    } else {
      vic_do_simple();
    }
    
    if (--lc == 0) {
      lc = LINEFREQ / 10; // 10Hz
      cia1_checkRTCAlarm();
      cia2_checkRTCAlarm();
    }

    //Switch "ExactTiming" Mode off after a while:
    if (!cpu.exactTiming) break;
    if ( (fbmicros() - cpu.exactTimingStartTime)*1000 >= EXACTTIMINGDURATION ) {
      cpu_disableExactTiming();
      break;
    }
  };

}

const uint32_t ascii2scan[] = {
 //0 1 2 3 4 5 6 7 8 9 A B C D E F
   0,0,0,0,0,0,0,0,0,0,0,0,0,0x28,0,0, // return
 //     17:down                                                     29:right
   0x00,0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4f,0x00,0x00,
   //sp  !       "     #     $      %      &      '     (        )   *    +    ,    -    .    / 
   0x2c,0x201e,0x201f,0x2020,0x2021,0x2022,0x2023,0x2024,0x2025,0x2026,0x55,0x57,0x36,0x56,0x37,0x54,
   //0  1    2    3    4    5    6    7    8    9    :    ;    <      =    >      ?
   0x27,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x33,0x34,0x2036,0x32,0x2037,0x2054,
   //@    A    B    C    D    E    F    G    H    I    J    K    L    M    N    O
   47,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,
   //P  Q    R    S    T    U    V    W    X    Y    Z    [      \     ]     ^    _  
   0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x2026,0x31,0x2027,0x00,0x00,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, // ' a b c d e f g h i j k l m n o
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x49,0, // p q r s t u v w x y z { | } ~ DEL
 //? ?                  133:f1   f2   f3   f4   f5   f6   f7   f8 
   75,78,0x00,0x00,0x00,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x40,0x41,0x00,0x00,0x00,  // 128-143
 //     145:up                                                      157:left
   0x00,0x2051,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x204f,0x00,0x00   // 144-159
};

// we also use USB matrix for the moment
static const uint8_t keymatrixmap[2][256] = {
  //Rows:
  // 0    1     2     3    4     5     6      7     8      9     A     B     C     D     E     F
  { 0x00, 0x00, 0x00, 0x00, 0x02, 0x08, 0x04, 0x04, 0x02, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x20, //0x00
    0x10, 0x10, 0x10, 0x20, 0x80, 0x04, 0x02, 0x04, 0x08, 0x08, 0x02, 0x04, 0x08, 0x02, 0x80, 0x80, //0x10
    0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x01, 0x80, 0x01, 0x00, 0x80, 0x00, 0x00, 0x20, //0x20
    0x00, 0x00, 0x40, 0x20, 0x40, 0x00, 0x20, 0x20, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, //0x30
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x00, 0x00, 0x80, 0x01, //0x40
    0x00, 0x01, 0x00, 0x00, 0x40, 0x40, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x50
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x60
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x70
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x80
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x90
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xA0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xB0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xC0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xD0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xE0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x40, 0x02
  }, //0xF0
  //Columns:
  // 0    1     2     3    4     5     6      7     8      9     A     B     C     D     E     F
  { 0x00, 0x00, 0x00, 0x00, 0x04, 0x10, 0x10, 0x04, 0x40, 0x20, 0x04, 0x20, 0x02, 0x04, 0x20, 0x04, //0x00
    0x10, 0x80, 0x40, 0x02, 0x40, 0x02, 0x20, 0x40, 0x40, 0x80, 0x02, 0x80, 0x02, 0x10, 0x01, 0x08, //0x10
    0x01, 0x08, 0x01, 0x08, 0x01, 0x08, 0x01, 0x08, 0x02, 0x80, 0x01, 0x00, 0x10, 0x00, 0x00, 0x40, //0x20
    0x00, 0x00, 0x20, 0x20, 0x04, 0x00, 0x80, 0x10, 0x00, 0x00, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, //0x30
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x08, 0x40, 0x00, 0x00, 0x02, 0x04, //0x40
    0x00, 0x80, 0x00, 0x00, 0x80, 0x02, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x50
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x60
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x70
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x80
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x90
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xA0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xB0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xC0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xD0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0xE0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x04, 0x10, 0x80
  }
}; //0xF0 

struct {
  union {
    uint32_t kv;
    struct {
      uint8_t ke,   // Modifier Keys - SHIFT, CTRL, ALT... 
              kdummy,
              k,    // First pressed key
              k2;   // Second pressed key
    };
  };
  uint32_t lastkv;
  uint8_t shiftLock;
} kbdData = {0, 0, 0};




static void setKey(uint32_t k, bool pressed) {
  if (pressed) {                    // If the key is pressed
    kbdData.kv = (k << 16);         // Store the key in the high word (upper 16-bits)
    kbdData.ke = kbdData.k2;        // Set the 2nd key pressed as a modifier key
    kbdData.k2 = 0;                 // Clear the 2nd key pressed (k2) 
  }
  else                              // If the key is released
  {
    kbdData.kv = 0;                 // Clear the key value (kv) 
  }
}

static void pushStringToTextEntry(char * text) {
    char c;
    while ((c = *text++)) {
        setKey(ascii2scan[c], true); 
        sleep_ms(20);
        setKey(ascii2scan[c], false); 
        sleep_ms(20);
    }
}


uint8_t cia1PORTA(void) {

  uint8_t v;

  v = ~cpu.cia1.R[0x02] | (cpu.cia1.R[0x00] & cpu.cia1.R[0x02]);
  int keys = emu_GetPad();
#ifndef PICOMPUTER
  /*
  if (oskbActive) keys = 0;
  */
#endif  
  if (!cpu.swapJoysticks) {
    if (keys & MASK_JOY2_BTN) v &= 0xEF;
    if (keys & MASK_JOY2_UP) v &= 0xFE;
    if (keys & MASK_JOY2_DOWN) v &= 0xFD;
    if (keys & MASK_JOY2_RIGHT) v &= 0xFB;
    if (keys & MASK_JOY2_LEFT) v &= 0xF7;
  } else {
    if (keys & MASK_JOY1_BTN) v &= 0xEF;
    if (keys & MASK_JOY1_UP) v &= 0xFE;
    if (keys & MASK_JOY1_DOWN) v &= 0xFD;
    if (keys & MASK_JOY1_RIGHT) v &= 0xFB;
    if (keys & MASK_JOY1_LEFT) v &= 0xF7;
  }	


  if (!kbdData.kv) return v; //Keine Taste gedrückt

  uint8_t filter = ~cpu.cia1.R[0x01] & cpu.cia1.R[0x03];
  
  if (kbdData.k) {
    if ( keymatrixmap[1][kbdData.k] & filter)  v &= ~keymatrixmap[0][kbdData.k];
  }

  if (kbdData.ke) {
    if (kbdData.ke & 0x02) { //Shift-links
      if ( keymatrixmap[1][0xff] & filter) v &= ~keymatrixmap[0][0xff];
    }
    if (kbdData.ke & 0x20) { //Shift-rechts
      if ( keymatrixmap[1][0xfe] & filter) v &= ~keymatrixmap[0][0xfe];
    }
    if (kbdData.ke & 0x11) { //Control
      if ( keymatrixmap[1][0xfd] & filter) v &= ~keymatrixmap[0][0xfd];
    }
    if (kbdData.ke & 0x88) { //Windows (=> Commodore)
      if ( keymatrixmap[1][0xfc] & filter) v &= ~keymatrixmap[0][0xfc];
    }
  }
 
  return v;
}


uint8_t cia1PORTB(void) {

  uint8_t v;

  v = ~cpu.cia1.R[0x03] | (cpu.cia1.R[0x00] & cpu.cia1.R[0x02]) ;
  int keys = emu_GetPad();
#ifndef PICOMPUTER
  /*
  if (oskbActive) keys = 0;
  */
#endif  
  if (!cpu.swapJoysticks) {
    if (keys & MASK_JOY1_BTN) v &= 0xEF;
    if (keys & MASK_JOY1_UP) v &= 0xFE;
    if (keys & MASK_JOY1_DOWN) v &= 0xFD;
    if (keys & MASK_JOY1_RIGHT) v &= 0xFB;
    if (keys & MASK_JOY1_LEFT) v &= 0xF7;
  } else {
    if (keys & MASK_JOY2_BTN) v &= 0xEF;
    if (keys & MASK_JOY2_UP) v &= 0xFE;
    if (keys & MASK_JOY2_DOWN) v &= 0xFD;
    if (keys & MASK_JOY2_RIGHT) v &= 0xFB;
    if (keys & MASK_JOY2_LEFT) v &= 0xF7;
  }

  if (!kbdData.kv) return v; //Keine Taste gedrückt

  uint8_t filter = ~cpu.cia1.R[0x00] & cpu.cia1.R[0x02];

  if (kbdData.k) {
    if ( keymatrixmap[0][kbdData.k] & filter) v &= ~keymatrixmap[1][kbdData.k];
  }

  if (kbdData.ke) {
    if (kbdData.ke & 0x02) { //Shift-links
      if ( keymatrixmap[0][0xff] & filter) v &= ~keymatrixmap[1][0xff];
    }
    if (kbdData.ke & 0x20) { //Shift-rechts
      if ( keymatrixmap[0][0xfe] & filter) v &= ~keymatrixmap[1][0xfe];
    }
    if (kbdData.ke & 0x11) { //Control
      if ( keymatrixmap[0][0xfd] & filter) v &= ~keymatrixmap[1][0xfd];
    }
    if (kbdData.ke & 0x88) { //Windows (=> Commodore)
      if ( keymatrixmap[0][0xfc] & filter) v &= ~keymatrixmap[1][0xfc];
    }
  }

  return v;
}


void c64_Init(void)
{
  disableEventResponder();
  resetPLA();
  resetCia1();
  resetCia2();
  resetVic();
  cpu_reset();
#ifdef HAS_SND  
  playSID.begin();  
  emu_sndInit();
#endif  
}


void c64_Step(void)
{
	oneRasterLine();
}

void c64_Start(char * filename)
{
}


static uint8_t nbkeys = 0;  // Number of keys to process
static uint8_t kcnt = 0;
static bool toggle = true;

static char * textseq;
static char * textload = "LOAD\"\"\r\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tRUN\r";
static char textkey[1];

static bool res=false;
static bool firsttime=true;
static int  loadtimeout=100; //100*20ms;

#ifndef PICOMPUTER
/*
#define OSKB_YPOS (240-16)

static char * oskbtext1 = "FFFFFFFF  RD\"$ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static char * oskbtext2 = "12345678  TL  ,.*     0123456789        ";
static int oskbXPos = 10;
static int oskbYPos = 0;

#define OSKB_TEXT RGBVAL16(0, 0, 170)
#define OSKB_BG   RGBVAL16(255, 255, 255)
#define OSKB_HL   RGBVAL16(255, 255, 0)

int emu_oskbActive(void) {
  return (oskbActive?1:0);
}

void emu_DrawVsync(void)
{
    char sel[2]={0,0};
    if (oskbActive) {
      int fbwidth;
      tft.get_frame_buffer_size(&fbwidth,NULL);       
      tft.drawText((fbwidth-320)/2,OSKB_YPOS,oskbtext1,OSKB_TEXT,OSKB_BG,false);
      tft.drawText((fbwidth-320)/2,OSKB_YPOS+8,oskbtext2,OSKB_TEXT,OSKB_BG,false);
      sel[0]=(oskbYPos==0)?oskbtext1[oskbXPos]:oskbtext2[oskbXPos];
      tft.drawText((fbwidth-320)/2+oskbXPos*8,OSKB_YPOS+8*oskbYPos,sel,OSKB_TEXT,OSKB_HL,false);
    }
    //skip += 1;
    //skip &= VID_FRAME_SKIP;
#ifdef USE_VGA
    tft.waitSync(); 
#endif    
}
*/
#endif

//#define DEBUG 1

#ifdef DEBUG
static const char * digits = "0123456789ABCDEF";
static char buf[5] = {0,0,0,0,0};
#endif

void picoCalc_Input(int btn) {
    static bool alreadyProcessed = false;
    if ( (btn != 0) && (alreadyProcessed == false) ) { 
      setKey(ascii2scan[btn],true);                 // Get the ASCII code and set the key as pressed
        alreadyProcessed = true;                                  // We've now registered a key press
    }
    else if (btn == 0){                          // No key is pressed
      setKey(ascii2scan[btn],false);                // Release the key
      alreadyProcessed = false;                                 // Reset the key press registered flag
    }         
}

/* Handle input from the keyboard:
  1. If there are keys to process in the text sequence, process them one at a time
  2. If there are no keys to process, check if USER1 is pressed to start the LOAD sequence
  3. If load sequence isn't being run, read the keyboard
*/
void c64_Input(int bClick) {
#ifdef DEBUG
        /*
        buf[2] = 0;
        int key = emu_ReadI2CKeyboard2(0);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(4*8,16*4,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(1);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(4*8,16*5,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(2);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(4*8,16*6,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(3);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(4*8,16*7,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(4);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(4*8,16*8,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        key = emu_ReadI2CKeyboard2(5);
        buf[0] = digits[(key>>4)&0xf];
        buf[1] = digits[key&0xf];
        tft.drawText(4*8,16*9,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
        */
#endif

#ifndef PICOMPUTER
/*  
  int fbwidth;  
  if (oskbActive) {
    if (bClick & MASK_JOY2_BTN) {    
      if (oskbXPos == 10) textkey[0] = 13;
      else if (oskbXPos == 11) textkey[0] = 157;
      else if (oskbXPos < 8) textkey[0] = 0x85+oskbXPos;
      else textkey[0] = (oskbYPos == 0)?oskbtext1[oskbXPos]:oskbtext2[oskbXPos];
      textseq = textkey;
      nbkeys = 1;   
      kcnt = 0;
      oskbActive = false;
      tft.get_frame_buffer_size(&fbwidth,NULL);       
      tft.drawRect(0,OSKB_YPOS,fbwidth,16,RGBVAL16(0, 0, 0));
    }
    if (bClick & MASK_JOY2_RIGHT) if (oskbXPos != 0) oskbXPos--;
    if (bClick & MASK_JOY2_LEFT) if (oskbXPos != 39)  oskbXPos++;
    if (bClick & MASK_JOY2_UP) oskbYPos = 0;
    if (bClick & MASK_JOY2_DOWN) oskbYPos = 1;
  }
*/  
#endif
  if (nbkeys == 0) {            // If there are no keys to auto-process

#ifndef PICOMPUTER
/*    
    if (bClick & MASK_KEY_USER2) {
      if (!oskbActive) {
        oskbActive = true;
      }
      else {
        oskbActive = false;
        tft.get_frame_buffer_size(&fbwidth,NULL);       
        tft.drawRect(0,OSKB_YPOS,fbwidth,16,RGBVAL16(0, 0, 0));
      }       
    } 
    else
*/    
#endif
    if (loadtimeout > 0) {      // Wait for a time period before loading the LOAD sequence
      loadtimeout--; 
    }
    if ( (bClick & MASK_KEY_USER1) && !(emu_GetPad() & MASK_OSKB) ) {     // Only run the load sequence if USER1 is pressed (and OSKB not active)
      if (loadtimeout == 0) {                                             // Make sure we've waited for the load timeout
        if (firsttime) {                                                  // Only do this the first time USER1 is pressed
          firsttime = false;                                              // We've now loaded it once, so don't do it again
          textseq = textload;                                             // Set the text sequence to the LOAD sequence
          nbkeys = strlen(textseq);                                       // Number of keys to process
          kcnt=0;                                                         // Reset key counter
        }
        else {                                                            // If USER1 is pressed again, just swap the joysticks
          cpu.swapJoysticks = !cpu.swapJoysticks;                         // Swap joysticks when USER1 is
        }        
      }
    } 
    else                                       // If USER1 isn't pressed, read the keyboard
    {
      int hk = emu_ReadI2CKeyboard();             // Read the I2C keyboard
      if ( (hk != 0) && (res == false) ) {        // If a key is pressed and we haven't already registered a key press
#ifdef DEBUG        
        buf[3] = 0;
        buf[0] = digits[(hk>>8)&0xf];
        buf[1] = digits[(hk>>4)&0xf];
        buf[2] = digits[hk&0xf];        
        tft.drawText(0,0,buf,RGBVAL16(0x00,0x00,0x00),RGBVAL16(0xFF,0xFF,0xFF),true);
#endif
        setKey(ascii2scan[hk],true);                 // Get the ASCII code and set the key as pressed
        res = true;                                  // We've now registered a key press
      } 
      else if (hk == 0){                          // No key is pressed
        setKey(ascii2scan[hk],false);                // Release the key
        res = false;                                 // Reset the key press registered flag
      }        
    }
  }
  else {                                            // If there are keys to autoprocess, this will automatically process them      
    char k = textseq[kcnt];                            // Get the next key to process 
    if (k != '\t') setKey(ascii2scan[k],toggle);       // Set or clear the key (if not a tab)
    if (!toggle) {                                     // If we just cleared a key
      kcnt++;                                             // Move to the next key
      nbkeys--;                                           // Decrement # of keys to process
      toggle = true;                                      // Next time we will set the key
    }
    else {                                             // If we just set a key
      toggle = false;                                     // Next time we will clear the key
    }
  }
}

void emu_KeyboardOnDown(int keymodifer, int key) {
}

void emu_KeyboardOnUp(int keymodifer, int key) {
}

#ifdef HAS_SND      
void  SND_Process( void * stream, int len )
{
    playSID.update(stream, len);
}
#endif  
