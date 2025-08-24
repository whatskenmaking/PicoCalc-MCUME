/******************************************************************************

based on rp2040-psram:
- add QSPI mode support
- adapted for RP2350

Copyright © 2023 Ian Scott

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************/

#pragma once
#include "iopins.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/dma.h"

#include "psram_spi.pio.h"

#ifdef __cplusplus
extern "C" {
#endif


#define QSPI 1

//#define WAIT_AFTER_WRITE 1


#define WRITE_CMD_QSPI 0x38u
#define WRITE_CMD_SPI 0x02u
#define READ_CMD_QSPI 0xebu
#define READ_CMD_SPI 0x0bu

typedef struct psram_spi_inst {
    PIO pio;
    int sm;
    int write_dma_chan;
    dma_channel_config write_dma_chan_config;
    int read_dma_chan;
    dma_channel_config read_dma_chan_config;
    bool init_done;
} psram_spi_inst_t;

static psram_spi_inst_t psram_spi;


__force_inline static void __time_critical_func(pio_spi_cmd_dma_blocking)(const uint8_t command) 
{
  psram_spi_inst_t * spi = &psram_spi;
  uint8_t wrbuffer[10];
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
  dma_channel_wait_for_finish_blocking(spi->read_dma_chan);
  int k=0;
#ifdef QSPI
  wrbuffer[k++]=8; 
  wrbuffer[k++]=0;
  uint8_t d=0;
  if (command & 0b10000000) d |= 0b00010000;
  if (command & 0b01000000) d |= 0b00000001;
  wrbuffer[k++] = d;
  d=0;
  if (command & 0b00100000) d |= 0b00010000;
  if (command & 0b00010000) d |= 0b00000001;
  wrbuffer[k++] = d;
  d=0;
  if (command & 0b00001000) d |= 0b00010000;
  if (command & 0b00000100) d |= 0b00000001;
  wrbuffer[k++] = d;
  d=0;
  if (command & 0b00000010) d |= 0b00010000;
  if (command & 0b00000001) d |= 0b00000001;
  wrbuffer[k++] = d;
#else
  wrbuffer[k++]=8;
  wrbuffer[k++]=0;
  wrbuffer[k++]=command;
#endif        
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, wrbuffer, k);
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
}


static uint8_t write8_command[] = {
#ifdef QSPI
    40/4,           // 40/4 nibbles write
#else
    40,             // 40 bits write
#endif
    0,              // 0 bits read
#ifdef QSPI
    WRITE_CMD_QSPI, // Write command
#else
    WRITE_CMD_SPI,  // Write command
#endif
    0, 0, 0,        // Address
    0               // 8 bits data
};

__force_inline static void __time_critical_func(psram_write8)(uint32_t addr, uint8_t val) {
  psram_spi_inst_t * spi = &psram_spi;
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
  //dma_channel_wait_for_finish_blocking(spi->read_dma_chan);
  write8_command[3] = addr >> 16;
  write8_command[4] = addr >> 8;
  write8_command[5] = addr;
  write8_command[6] = val;
  //pio_spi_write_dma_blocking(write8_command, sizeof(write8_command));
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, write8_command, sizeof(write8_command));
#ifdef WAIT_AFTER_WRITE
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
#endif
};



static uint8_t write16_command[] = {
#ifdef QSPI
    48/4,           // 48/4 nibbles write
#else
    48,             // 48 bits write
#endif
    0,              // 0 bits read  
#ifdef QSPI
    WRITE_CMD_QSPI, // Write command
#else
    WRITE_CMD_SPI,  // Write command
#endif
    0, 0, 0,        // Address
    0, 0            // 16 bits data
};

__force_inline static void __time_critical_func(psram_write16)(uint32_t addr, uint16_t val) {
  psram_spi_inst_t * spi = &psram_spi;
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
  //dma_channel_wait_for_finish_blocking(spi->read_dma_chan);
  write16_command[3] = addr >> 16;
  write16_command[4] = addr >> 8;
  write16_command[5] = addr;
  write16_command[6] = val;
  write16_command[7] = val >> 8;
  //pio_spi_write_dma_blocking(write16_command, sizeof(write16_command));
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, write16_command, sizeof(write16_command));
#ifdef WAIT_AFTER_WRITE
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
#endif
};


static uint8_t write32_command[] = {
#ifdef QSPI
    64/4,           // 64/4 nibbles write
#else
    64,             // 64 bits write
#endif
    0,              // 0 bits read  
#ifdef QSPI
    WRITE_CMD_QSPI, // Write command
#else
    WRITE_CMD_SPI,  // Write command
#endif
    0, 0, 0,        // Address
    0, 0, 0, 0      // 32 bits data
};

__force_inline static void __time_critical_func(psram_write32)(uint32_t addr, uint32_t val) {
  psram_spi_inst_t * spi = &psram_spi;
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
  //dma_channel_wait_for_finish_blocking(spi->read_dma_chan);
  write32_command[3] = addr >> 16;
  write32_command[4] = addr >> 8;
  write32_command[5] = addr;
  write32_command[6] = val;
  write32_command[7] = val >> 8;
  write32_command[8] = val >> 16;
  write32_command[9] = val >> 24;
  //pio_spi_write_dma_blocking(write32_command, sizeof(write32_command));
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, write32_command, sizeof(write32_command));
#ifdef WAIT_AFTER_WRITE
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
#endif
};

static uint8_t write64_command[] = {
#ifdef QSPI
    96/4,           // 96/4 nibbles write
#else
    96,             // 96 bits write
#endif
    0,              // 0 bits read  
#ifdef QSPI
    WRITE_CMD_QSPI, // Write command
#else
    WRITE_CMD_SPI,  // Write command
#endif
    0, 0, 0,        // Address
    0, 0, 0, 0,     // 64 bits data
    0, 0, 0, 0    
};

__force_inline static void __time_critical_func(psram_write64)(uint32_t addr, uint64_t val) {
  psram_spi_inst_t * spi = &psram_spi;
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
  //dma_channel_wait_for_finish_blocking(spi->read_dma_chan);
  write64_command[3] = addr >> 16;
  write64_command[4] = addr >> 8;
  write64_command[5] = addr;
  write64_command[6] = val;
  write64_command[7] = val >> 8;
  write64_command[8] = val >> 16;
  write64_command[9] = val >> 24;
  write64_command[10] = val >> 32;
  write64_command[11] = val >> 40;
  write64_command[12] = val >> 48;
  write64_command[13] = val >> 56;
  //pio_spi_write_dma_blocking(write32_command, sizeof(write32_command));
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, write64_command, sizeof(write64_command));
#ifdef WAIT_AFTER_WRITE
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
#endif
};

static uint8_t writen_command[] = {
    0,               // x bits or nibbles write
    0,               // 0 bits read  
#ifdef QSPI
    WRITE_CMD_QSPI,  // Write command
#else
    WRITE_CMD_SPI,   // Write command
#endif
    0, 0, 0,         // Address
    0, 0, 0, 0       // Up to 32 bits data
};

__force_inline static void __time_critical_func(psram_writen)(uint32_t addr, uint8_t* src, const size_t count) {
  psram_spi_inst_t * spi = &psram_spi;
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
  //dma_channel_wait_for_finish_blocking(spi->read_dma_chan);
#ifdef QSPI
  writen_command[0] = (4+count)*2;
#else
  writen_command[0] = (4+count)*8;
#endif
  writen_command[3] = addr >> 16;
  writen_command[4] = addr >> 8;
  writen_command[5] = addr;
  //pio_spi_write_dma_blocking(spi, writen_command, 6);
  //pio_spi_write_dma_blocking(spi, src, count);
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, writen_command, 6);
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);  
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, src, count);
#ifdef WAIT_AFTER_WRITE
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
#endif
};




#define psram_read8(addr) ({uint8_t retval=0; psram_readn(addr,(unsigned char*)&retval,1); retval;})
#define psram_read16(addr) ({uint16_t retval=0; psram_readn(addr,(unsigned char*)&retval,2); retval;})
#define psram_read32(addr) ({uint32_t retval=0; psram_readn(addr,(unsigned char*)&retval,4); retval;})
#define psram_read64(addr) ({uint64_t retval=0; psram_readn(addr,(unsigned char*)&retval,8); retval;})

static uint8_t readn_command[] = {
#ifdef QSPI
    (40)/4,         // 40 bits write as nibbles
    0,              // n bits read as nibbles
    READ_CMD_QSPI,  // Fast read command
#else
    40,             // 40 bits write
    0,              // n bits read
    READ_CMD_SPI,   // Fast read command
#endif    
    0, 0, 0,        // Address
    0xff            // 8 delay cycles or 2 as nibbles
};

__force_inline static void __time_critical_func(psram_readn)(uint32_t addr, uint8_t* dst, const size_t count) {
  psram_spi_inst_t * spi = &psram_spi;
#ifdef QSPI
  readn_command[1] = (count * 8)/4;
#else
  readn_command[1] = count * 8;
#endif
  readn_command[3] = addr >> 16;
  readn_command[4] = addr >> 8;
  readn_command[5] = addr;
  //pio_spi_write_read_dma_blocking(spi, readn_command, sizeof(readn_command), dst, count);
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
#ifndef WAIT_AFTER_WRITE
  dma_channel_wait_for_finish_blocking(spi->read_dma_chan); // cannot be removed...
#endif
  dma_channel_transfer_from_buffer_now(spi->write_dma_chan, readn_command, sizeof(readn_command));
  dma_channel_wait_for_finish_blocking(spi->write_dma_chan);
  dma_channel_transfer_to_buffer_now(spi->read_dma_chan, dst, count);
  dma_channel_wait_for_finish_blocking(spi->read_dma_chan);
};

__force_inline static void psram_init(void)
{
  psram_spi_inst_t * spi = &psram_spi;
  spi->pio = pio2;
  spi->sm = 0;
#ifdef QSPI
  pio_qspi_psram_cs_init(spi->init_done, spi->pio, spi->sm, pio_add_program(spi->pio,&qspi_psram_fudge_program), 8 /*n_bits*/, 1.0, PSRAM_CS, PSRAM_SIO0);
#else
  pio_spi_psram_cs_init(spi->init_done, spi->pio, spi->sm, pio_add_program(spi->pio, &spi_psram_fudge_program), 8 /*n_bits*/, 1.0, PSRAM_CS, PSRAM_MOSI, PSRAM_MISO);
#endif
  if (!spi->init_done) {
    spi->init_done = true;
    spi->write_dma_chan = PSR_DMA_CHANNEL;
    spi->read_dma_chan = PSR_DMA_CHANNEL+1;
    // Write DMA channel setup
    dma_channel_abort( spi->write_dma_chan);
    spi->write_dma_chan_config = dma_channel_get_default_config(spi->write_dma_chan);
    channel_config_set_transfer_data_size(&spi->write_dma_chan_config, DMA_SIZE_8);
    channel_config_set_read_increment(&spi->write_dma_chan_config, true);
    channel_config_set_write_increment(&spi->write_dma_chan_config, false);
    channel_config_set_dreq(&spi->write_dma_chan_config, pio_get_dreq(spi->pio, spi->sm, true));
    dma_channel_set_write_addr(spi->write_dma_chan, &spi->pio->txf[spi->sm], false);
    dma_channel_set_config(spi->write_dma_chan, &spi->write_dma_chan_config, false);
    // Read DMA channel setup
    dma_channel_abort( spi->read_dma_chan);
    spi->read_dma_chan_config = dma_channel_get_default_config(spi->read_dma_chan);
    channel_config_set_transfer_data_size(&spi->read_dma_chan_config, DMA_SIZE_8);
    channel_config_set_read_increment(&spi->read_dma_chan_config, false);
    channel_config_set_write_increment(&spi->read_dma_chan_config, true);
    channel_config_set_dreq(&spi->read_dma_chan_config, pio_get_dreq(spi->pio, spi->sm, false));
    dma_channel_set_read_addr(spi->read_dma_chan, &spi->pio->rxf[spi->sm], false);
    dma_channel_set_config(spi->read_dma_chan, &spi->read_dma_chan_config, false);
  }   
  // Reset PSRAM
  pio_spi_cmd_dma_blocking(0x66u); // Reset enable command
  busy_wait_us(50);
  pio_spi_cmd_dma_blocking(0x99u); // Reset command
  busy_wait_us(100);
  //pio_spi_cmd_dma_blocking(0xc0u); // Wrap boundary toggle  
#ifdef QSPI
  pio_spi_cmd_dma_blocking(0x35u); // Enter quad command
#endif  
}

#ifdef __cplusplus
}
#endif
