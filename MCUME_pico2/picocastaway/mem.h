#include "platform_config.h"

#ifndef HAS_PSRAM
#define ALL_IN_RAM  1
#endif

#ifdef ALL_IN_RAM

#define MEMSIZE  0x00040000L /* default memsize 256 Kb split RAM */
#define RAM1BASE 0x00000000L
#define RAM1SIZE 0x00020000L
#define RAM2BASE RAM1SIZE
extern uint8    *mem1base;

#else

#define MEMSIZE  0x00100000L /* default memsize 1024 Kb split PSRAM+RAM */
#define RAM1BASE 0x00000000L
//#define RAM1SIZE 0x000B0000L
#define RAM1SIZE 0x00100000L
#define RAM2BASE RAM1SIZE

#endif
extern uint8    *mem2base;
extern uint8	*rombase;

/*
 * Read/Write memory macros - little endian
 */
#ifdef ALL_IN_RAM

#define ReadB(address) *(uint8*)((uint32)(address+mem1base)^1)
#define WriteB(address,value) *(uint8*)((uint32)(address+mem1base)^1)=value
#define ReadW(addr) *(uint16*)(addr+mem1base)
#define WriteW(addr,value) *(uint16*)(addr+mem1base)=value 
#define ReadL(address) ((*(uint16*)(address+mem1base))<<16)|(*(uint16*)(address+mem1base+2))
#define WriteL(address,value) WriteW(address + 2, value); WriteW(address, value>> 16)
//#define ReadSL(addr) (*(uint16*)(addr+mem1base))|((*(uint16*)(addr+mem1base+2))<<16)

#else

extern unsigned char ram_readb(int address);
extern void ram_writeb(int address, unsigned char val);
extern unsigned short ram_readw(int address);
extern void ram_writew(int address, unsigned short val);
extern unsigned long ram_readl(int address);
extern void ram_writel(int address, unsigned long val);


// Assumed all memory is in PSRAM else use code below !!!!
// WriteBB is olny used to copy tos image because it is already pre swapped
#define WriteBB(address,value) ram_writeb(address,value) 

#define ReadB(address) ram_readb(address^1)
#define WriteB(address,value) ram_writeb(address^1,value)
#define ReadW(address) ram_readw(address)
#define WriteW(address,value) ram_writew(address,value)
#define ReadL(address) ram_readl(address)
#define WriteL(address,value) ram_writel(address,value)
//#define ReadL(address) ((ReadW(address)<<16)|(ReadW(address+2)))
//#define WriteL(address,value) WriteW(address + 2, value); WriteW(address, value>> 16)

/*
#define ReadBB(address) (address<RAM1SIZE?ram_readb(address):*(address-RAM1SIZE+mem2base))
#define WriteBB(address,value) (address<RAM1SIZE?ram_writeb(address,value):(void)(*(address-RAM1SIZE+mem2base)=value)) 
#define ReadB(address) (address<RAM1SIZE?ram_readb(address^1):*(uint8*)((uint32)(address-RAM1SIZE+mem2base)^1))
#define WriteB(address,value) (address<RAM1SIZE?ram_writeb(address^1,value):(void)(*(uint8*)((uint32)(address-RAM1SIZE+mem2base)^1)=value)) 
#define ReadW(address) (address<RAM1SIZE?ram_readw(address):*(uint16*)(address-RAM1SIZE+mem2base))
#define WriteW(address,value) (address<RAM1SIZE?ram_writew(address,value):(void)(*(uint16*)(address-RAM1SIZE+mem2base)=value))  
#define ReadL(address) (address<RAM1SIZE?ram_readl(address):((*(uint16*)(address-RAM1SIZE+mem2base))<<16)|(*(uint16*)(address-RAM1SIZE+mem2base+2))) 
#define WriteL(address,value) (address<RAM1SIZE?ram_writel(address,value):(void)(*(uint32*)(address-RAM1SIZE+mem2base)=value))
*/

//#define ReadSL(address) (address<RAM1SIZE?(ram_readw(address))|((ram_readw(address+2))<<16):(*(uint16*)(address-RAM1SIZE+mem2base))|((*(uint16*)(address-RAM1SIZE+mem2base+2))<<16)) 
//#define ReadL(address) (address<RAM1SIZE?(ram_readw(address)<<16)|(ram_readw(address+2)):((*(uint16*)(address-RAM1SIZE+mem2base))<<16)|(*(uint16*)(address-RAM1SIZE+mem2base+2))) 
//#define WriteL(address,value) WriteW(address + 2, value); WriteW(address, value>> 16)

#endif


int MemInit(void);
void MemQuit(void);
void MemClean(void);
void MemReInit(void);

extern unsigned short int TosVersion; 
extern short TosCountry;
void TOS_FixRom(uint8 *TosAddress);
