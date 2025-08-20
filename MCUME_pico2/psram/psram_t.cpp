#include "iopins.h"

#ifdef HAS_PSRAM
/*
  PSRAM driver for IPS6404
*/

#include "psram_t.h"

#include <cstdio>

#include <pico/stdlib.h>

#include "hardware/spi.h"
#include "psram_spi.h"

#ifdef PSCACHE 
Page PSRAM_T::pages[MAX_PAGES];
uint8_t PSRAM_T::nbPages=0;
int8_t PSRAM_T::top=0;
int8_t PSRAM_T::last=0;
#endif


static psram_spi_inst_t psram_spi;
static psram_spi_inst_t psram_qspi;
static bool qspi=false;

void psram_spi_init_clkdiv(psram_spi_inst_t * spi, PIO pio, int sm, float clkdiv, bool fudge, bool qspi) {
    spi->pio = pio;
    spi->sm = sm;
/*
    gpio_set_drive_strength(PSRAM_CS, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_drive_strength(PSRAM_SCLK, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_drive_strength(PSRAM_SIO0, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_drive_strength(PSRAM_SIO0+1, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_drive_strength(PSRAM_SIO0+2, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_drive_strength(PSRAM_SIO0+3, GPIO_DRIVE_STRENGTH_4MA);
*/

/*
     gpio_set_slew_rate(PSRAM_CS, GPIO_SLEW_RATE_FAST); 
     gpio_set_slew_rate(PSRAM_SCLK, GPIO_SLEW_RATE_FAST); 
     gpio_set_slew_rate(PSRAM_SIO0, GPIO_SLEW_RATE_FAST); 
     gpio_set_slew_rate(PSRAM_SIO0+1, GPIO_SLEW_RATE_FAST); 
     gpio_set_slew_rate(PSRAM_SIO0+2, GPIO_SLEW_RATE_FAST); 
     gpio_set_slew_rate(PSRAM_SIO0+3, GPIO_SLEW_RATE_FAST); 
*/


    if (!qspi) {
        spi->qspi=false;
        pio_spi_psram_cs_init(spi->init_done, spi->pio, spi->sm, pio_add_program(spi->pio, &spi_psram_fudge_program), 8 /*n_bits*/, clkdiv, fudge, PSRAM_CS, PSRAM_MOSI, PSRAM_MISO);
        spi->write_dma_chan = PSR_DMA_CHANNEL;
        spi->read_dma_chan = PSR_DMA_CHANNEL+1;
    }
    else {
        spi->qspi=true;
        pio_qspi_psram_cs_init(spi->init_done, spi->pio, spi->sm, pio_add_program(spi->pio,&qspi_psram_program), 8 /*n_bits*/, clkdiv, PSRAM_CS, PSRAM_SIO0);
        spi->write_dma_chan = PSR_DMA_CHANNEL+2;
        spi->read_dma_chan = PSR_DMA_CHANNEL+3;
    }
    if (!spi->init_done) {

      spi->init_done = true;
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
};


PSRAM_T::PSRAM_T(uint8_t cs, uint8_t mosi, uint8_t sclk, uint8_t miso)
{
#ifdef PSCACHE 
  nbPages = 0;
  top = 0;
  last = 0;
#endif

}

void PSRAM_T::begin(void)
{
#ifdef QSPIONLY
  psram_spi_init_clkdiv(&psram_qspi, pio2, 0, 1.0, true, true);
  qspi = true;
  reset();
  pio_spi_cmd_dma_blocking(&psram_qspi, 0x35u); // enter quad command
#else
#ifndef SPIONLY  
  psram_spi_init_clkdiv(&psram_qspi, pio1, 0, 1.0, true, true);
#endif
  psram_spi_init_clkdiv(&psram_spi, pio2, 0, 1.0, true, false);
  reset();
#endif  
}


void PSRAM_T::reset(void) 
{
  if (qspi)
    pio_spi_cmd_dma_blocking(&psram_qspi, 0x66u); // Reset enable command
  else
    pio_spi_cmd_dma_blocking(&psram_spi, 0x66u); // Reset enable command
  busy_wait_us(50);
  if (qspi)
    pio_spi_cmd_dma_blocking(&psram_qspi, 0x99u); // Reset command
  else
    pio_spi_cmd_dma_blocking(&psram_spi, 0x99u); // Reset command
  busy_wait_us(100);
  //if (qspi)
  //  pio_spi_cmd_dma_blocking(&psram_qspi, 0xc0u); // wrap boundary toggle
  //else
  //  pio_spi_cmd_dma_blocking(&psram_spi, 0xc0u); // wrap boundary toggle
}

void PSRAM_T::spi_mode(void) 
{
#if defined(SPIONLY) || defined(QSPIONLY)
#else  
    pio_spi_cmd_dma_blocking(&psram_qspi, 0xf5u); // exit quad command
    psram_spi_init_clkdiv(&psram_spi, pio2, 0, 1.0, true, false);
    //busy_wait_us(100); 
    qspi = false;
#endif
}

void PSRAM_T::qspi_mode(void) 
{
#if defined(SPIONLY) || defined(QSPIONLY)
#else
    if (qspi)
      pio_spi_cmd_dma_blocking(&psram_qspi, 0x35u); // enter quad command
    else
      pio_spi_cmd_dma_blocking(&psram_spi, 0x35u); // enter quad command
    psram_spi_init_clkdiv(&psram_qspi, pio1, 0, 1.0, true, true);
    //busy_wait_us(100);
    qspi = true;
#endif 
}

uint8_t PSRAM_T::psram_read(uint32_t addr) 
{
  if (qspi)
    return psram_read8(&psram_qspi, addr);
  else
    return psram_read8(&psram_spi, addr);
}

uint16_t PSRAM_T::psram_read_w(uint32_t addr) 
{
  if (qspi) {
    //uint16_t a = psram_read8(&psram_qspi, addr+1) << 8;
    //a |= psram_read8(&psram_qspi, addr);
    //return a;
    return psram_read16(&psram_qspi, addr);
  }
  else
    return psram_read16(&psram_spi, addr);
}


void PSRAM_T::psram_read_n(uint32_t addr, uint8_t * dstpt, int n) 
{
  if (qspi)
    return psram_readn(&psram_qspi, addr, dstpt, n); 
  else
    return psram_readn(&psram_spi, addr, dstpt, n);
}


void PSRAM_T::psram_write(uint32_t addr, uint8_t val) 
{
  if (qspi)
    psram_write8(&psram_qspi, addr, val); 
  else
    psram_write8(&psram_spi, addr, val); 
}

void PSRAM_T::psram_write_w(uint32_t addr, uint16_t val) 
{
  if (qspi)
    psram_write16(&psram_qspi, addr, val); 
  else
    psram_write16(&psram_spi, addr, val); 
}

//void PSRAM_T::psram_write_n(uint32_t addr, uint8_t * val, int n) 
//{
//  psram_writen(&psram_spi, addr, val, n);  
//}





void PSRAM_T::pswrite(uint32_t addr, uint8_t val) 
{
  psram_write(addr, val);
#ifdef PSCACHE  
  uint32_t curPage=addr&(~(PAGE_SIZE-1));
  for (int i=0; i<nbPages; i++) {
    if (pages[i].pageid == curPage) {
      pages[i].page[addr&(PAGE_SIZE-1)] = val;
      break;
    }
  }  
#endif   
}

uint8_t PSRAM_T::psread(uint32_t addr) 
{
#ifdef PSCACHE  
  uint32_t curPage=addr&(~(PAGE_SIZE-1));
  uint32_t offs = addr&(PAGE_SIZE-1);

  for (int i=0; i<nbPages; i++) {
    if (pages[i].pageid == curPage) {
      if ( (pages[i].prev != i) && (pages[i].next != i) ) {
        pages[pages[i].prev].next =  pages[i].next;
        pages[pages[i].next].prev = pages[i].prev;     
      }
      else if (pages[i].next != i) {
        pages[pages[i].next].prev = i;
      }
      else if (pages[i].prev != i) {
        pages[pages[i].prev].next =  pages[i].prev;
        last = pages[i].prev;        
      }
       // last page accessed to top
      pages[i].prev = i; //-1;
      pages[i].next = top;      
      pages[top].prev = i;
      top = i;
      return pages[top].page[offs];
    }
  }
  if (nbPages<MAX_PAGES) 
  {
    // add at top
    pages[nbPages].pageid = curPage;           
    pages[nbPages].prev = nbPages; //-1;
    pages[nbPages].next = top;     
    pages[top].prev = nbPages;
    top = nbPages;
    nbPages++;
  }
  else {
      // replace last and move to top
      int n = pages[last].prev;
      pages[n].next = n; //-1;
      pages[last].pageid = curPage;
      pages[last].prev = last; //-1;
      pages[last].next = top;      
      pages[top].prev = last;
      top = last;
      last = n;
  }
  //emu_printi(curPage);
  psram_read_n(curPage,&(pages[top].page[0]),PAGE_SIZE);   
  return pages[top].page[offs];
#else
  return psram_read(addr);
#endif
}

uint16_t PSRAM_T::psread_w(uint32_t addr) 
{
#ifdef PSCACHE
  uint32_t curPage=addr&(~(PAGE_SIZE-1));
  uint32_t offs = addr&(PAGE_SIZE-1);

  for (int i=0; i<nbPages; i++) {
    if (pages[i].pageid == curPage) {
      if ( (pages[i].prev != i) && (pages[i].next != i) ) {
        pages[pages[i].prev].next =  pages[i].next;
        pages[pages[i].next].prev = pages[i].prev;     
      }
      else if (pages[i].next != i) {
        pages[pages[i].next].prev = i;
      }
      else if (pages[i].prev != i) {
        pages[pages[i].prev].next =  pages[i].prev;
        last = pages[i].prev;        
      }
       // last page accessed to top
      pages[i].prev = i; //-1;
      pages[i].next = top;      
      pages[top].prev = i;
      top = i;     
      return (pages[top].page[offs+1]<<8) + pages[top].page[offs];
    }
  }
  if (nbPages<MAX_PAGES) 
  {
    // add at top
    pages[nbPages].pageid = curPage;           
    pages[nbPages].prev = nbPages; //-1;
    pages[nbPages].next = top;     
    pages[top].prev = nbPages;
    top = nbPages;
    nbPages++;
  }
  else {
      // replace last and move to top
      int n = pages[last].prev;
      pages[n].next = n; //-1;
      pages[last].pageid = curPage;
      pages[last].prev = last; //-1;
      pages[last].next = top;      
      pages[top].prev = last;
      top = last;
      last = n;
  }
  //emu_printi(curPage);
  psram_read_n(curPage,&(pages[top].page[0]),PAGE_SIZE);   
  return (pages[top].page[offs+1]<<8) + pages[top].page[offs];
#else
  return psram_read_w(addr);
#endif  
}

void PSRAM_T::pswrite_w(uint32_t addr, uint16_t val)
{
  psram_write_w(addr, val);
#ifdef PSCACHE 
  uint32_t curPage=addr&(~(PAGE_SIZE-1));
  for (int i=0; i<nbPages; i++) {
    if (pages[i].pageid == curPage) {
      pages[i].page[addr&(PAGE_SIZE-1)] = val&0xff;
      pages[i].page[(addr+1)&(PAGE_SIZE-1)] = val>>8 ;
      break;
    }
  }
#endif
}
#endif
