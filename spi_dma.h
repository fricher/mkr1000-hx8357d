#ifndef _SPI_DMA_H_
#define _SPI_DMA_H_

#include <Arduino.h>

void spi_dma_init();
void spi_dma_write ( void *data, uint16_t n, uint32_t chan );
uint32_t spi_dma_done();

#endif
