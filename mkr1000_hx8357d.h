#ifndef _ISIR_MKR1000_HX8357D_H_
#define _ISIR_MKR1000_HX8357D_H_

#include "hx8357d_defs.h"

#include <Arduino.h>
#include <stdint.h>

class HX8357D
{
public:
    HX8357D();
    void begin ( uint8_t cspin, uint8_t dcpin );

    void fillScreen ( uint16_t color );
    void fillRect ( int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color );

    void pushPixelsDMA ( uint8_t* buf, uint16_t sz );
    bool waitForDMA();

    void setAddrWindow ( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 );

    uint32_t width() {
        return HX8357_TFTWIDTH;
    }
    uint32_t height() {
        return HX8357_TFTHEIGHT;
    }

    static uint16_t rgbTo565 ( uint8_t r, uint8_t g, uint8_t b );

private:
    void writeCommand ( uint8_t c );
    void writeData ( uint8_t c );
    uint8_t readData ( uint8_t reg );

    volatile RwReg* _csport;
    uint32_t _cspinmask;

    volatile RwReg* _dcport;
    uint32_t _dcpinmask;
};

#endif
