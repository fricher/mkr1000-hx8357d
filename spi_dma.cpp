#include "spi_dma.h"
#include <SPI.h>

// DMA   12 channels
typedef struct {
    uint16_t btctrl;
    uint16_t btcnt;
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t descaddr;
} dmacdescriptor;

static volatile dmacdescriptor wrb[12] __attribute__((aligned(16)));
static dmacdescriptor descriptor_section[12] __attribute__((aligned(16)));

static volatile uint32_t dmadone;

Sercom *sercom = (Sercom *)SERCOM1; // SPI SERCOM

void DMAC_Handler()
{
    // interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
    uint8_t active_channel;

    // disable irqs ?
    __disable_irq();
    active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk;             // get channel number
    DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
    dmadone = DMAC->CHINTFLAG.reg;
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL;             // clear
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
    __enable_irq();
}

void spi_dma_init()
{
    memset((void *)descriptor_section, 0, 12 * sizeof(dmacdescriptor));
    memset((void *)wrb, 0, 12 * sizeof(dmacdescriptor));

    // probably on by default
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
    PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
    NVIC_EnableIRQ(DMAC_IRQn);

    DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;       // CTRL.DMAENABLE must be disabled to write to BASEADDR and WRBADDR
    DMAC->BASEADDR.reg = (uint32_t)descriptor_section;             // Descriptor base memory address
    DMAC->WRBADDR.reg = (uint32_t)wrb;             // Write-back memory base address
    DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);             // Enable DMA + Enable all priority levels

    dmadone = 0;
}

void spi_dma_write(void *data, uint16_t n, uint32_t chan)
{
    dmadone = 0;
    DMAC->CHID.reg = DMAC_CHID_ID(chan);

    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
    DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
    DMAC->SWTRIGCTRL.reg &= ~DMAC_SWTRIGCTRL_SWTRIG(1 << chan);
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(SERCOM1_DMAC_ID_TX) | DMAC_CHCTRLB_LVL(0);
    DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;

    descriptor_section[chan].btcnt = n;
    descriptor_section[chan].srcaddr = (uint32_t)data + n * sizeof(uint8_t);
    descriptor_section[chan].dstaddr = (uint32_t)&sercom->SPI.DATA.reg;
    descriptor_section[chan].descaddr = 0;
    descriptor_section[chan].btctrl = DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_VALID;

    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

uint32_t spi_dma_done()
{
    return dmadone;
}
