/***************************************************
  Based on the HX8357 library from Adafruit :

  This is our library for the Adafruit HX8357D Breakout
  ----> http://www.adafruit.com/products/2050
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "mkr1000_hx8357d.h"
#include "spi_dma.h"
#include <SPI.h>

HX8357D::HX8357D()
{
}

void HX8357D::begin(uint8_t cspin, uint8_t dcpin)
{
    SPI.begin();

    pinMode(dcpin, OUTPUT);
    pinMode(cspin, OUTPUT);

    _csport = portOutputRegister(digitalPinToPort(cspin));
    _cspinmask = digitalPinToBitMask(cspin);

    _dcport = portOutputRegister(digitalPinToPort(dcpin));
    _dcpinmask = digitalPinToBitMask(dcpin);

    spi_dma_init();

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    writeCommand(HX8357_SWRESET);

// setextc
    writeCommand(HX8357D_SETC);
    writeData(0xFF);
    writeData(0x83);
    writeData(0x57);
    delay(300);
// setRGB which also enables SDO
    writeCommand(HX8357_SETRGB);
    writeData(0x80);                                                                                                                                     //enable SDO pin!
//    writeData(0x00);  //disable SDO pin!
    writeData(0x0);
    writeData(0x06);
    writeData(0x06);

    writeCommand(HX8357D_SETCOM);
    writeData(0x25);                                                                                                                                     // -1.52V

    writeCommand(HX8357_SETOSC);
    writeData(0x68);                                                                                                                                     // Normal mode 70Hz, Idle mode 55 Hz

    writeCommand(HX8357_SETPANEL);                                                                                                                                     //Set Panel
    writeData(0x05);                                                                                                                                     // BGR, Gate direction swapped

    writeCommand(HX8357_SETPWR1);
    writeData(0x00);                                                                                                                                     // Not deep standby
    writeData(0x15);                                                                                                                                     //BT
    writeData(0x1C);                                                                                                                                     //VSPR
    writeData(0x1C);                                                                                                                                     //VSNR
    writeData(0x83);                                                                                                                                     //AP
    writeData(0xAA);                                                                                                                                     //FS

    writeCommand(HX8357D_SETSTBA);
    writeData(0x50);                                                                                                                                     //OPON normal
    writeData(0x50);                                                                                                                                     //OPON idle
    writeData(0x01);                                                                                                                                     //STBA
    writeData(0x3C);                                                                                                                                     //STBA
    writeData(0x1E);                                                                                                                                     //STBA
    writeData(0x08);                                                                                                                                     //GEN

    writeCommand(HX8357D_SETCYC);
    writeData(0x02);                                                                                                                                     //NW 0x02
    writeData(0x40);                                                                                                                                     //RTN
    writeData(0x00);                                                                                                                                     //DIV
    writeData(0x2A);                                                                                                                                     //DUM
    writeData(0x2A);                                                                                                                                     //DUM
    writeData(0x0D);                                                                                                                                     //GDON
    writeData(0x78);                                                                                                                                     //GDOFF

    writeCommand(HX8357D_SETGAMMA);
    writeData(0x02);
    writeData(0x0A);
    writeData(0x11);
    writeData(0x1d);
    writeData(0x23);
    writeData(0x35);
    writeData(0x41);
    writeData(0x4b);
    writeData(0x4b);
    writeData(0x42);
    writeData(0x3A);
    writeData(0x27);
    writeData(0x1B);
    writeData(0x08);
    writeData(0x09);
    writeData(0x03);
    writeData(0x02);
    writeData(0x0A);
    writeData(0x11);
    writeData(0x1d);
    writeData(0x23);
    writeData(0x35);
    writeData(0x41);
    writeData(0x4b);
    writeData(0x4b);
    writeData(0x42);
    writeData(0x3A);
    writeData(0x27);
    writeData(0x1B);
    writeData(0x08);
    writeData(0x09);
    writeData(0x03);
    writeData(0x00);
    writeData(0x01);

    writeCommand(HX8357_COLMOD);
    writeData(0x55);                                                                                                                                     // 16 bit

    writeCommand(HX8357_MADCTL);
    writeData(0xC0);

    writeCommand(HX8357_TEON);                                                                                                                                     // TE off
    writeData(0x00);

    writeCommand(HX8357_TEARLINE);                                                                                                                                     // tear line
    writeData(0x00);
    writeData(0x02);

    writeCommand(HX8357_SLPOUT);                                                                                                                                     //Exit Sleep
    delay(150);

    writeCommand(HX8357_DISPON);                                                                                                                                     // display on
    delay(50);

    uint8_t ret = readData(HX8357_RDPOWMODE);
    Serial.print("Display Power Mode: 0x");
    Serial.println(ret, HEX);

    ret = readData(HX8357_RDMADCTL);
    Serial.print("MADCTL Mode: 0x");
    Serial.println(ret, HEX);

    ret = readData(HX8357_RDCOLMOD);
    Serial.print("Pixel Format: 0x");
    Serial.println(ret, HEX);

    ret = readData(HX8357_RDDIM);
    Serial.print("Image Format: 0x");
    Serial.println(ret, HEX);

    ret = readData(HX8357_RDDSDR);
    Serial.print("Self Diagnostic: 0x");
    Serial.println(ret, HEX);

    SPI.endTransaction();
}

void HX8357D::writeCommand(uint8_t c)
{

    *_csport &= ~_cspinmask;
    *_dcport &=  ~_dcpinmask;

    SPI.transfer(c);

    *_csport |= _cspinmask;
}

void HX8357D::writeData(uint8_t c)
{
    *_csport &= ~_cspinmask;
    *_dcport |=  _dcpinmask;

    SPI.transfer(c);

    *_csport |= _cspinmask;
}

uint8_t HX8357D::readData(uint8_t reg)
{
    *_csport &= ~_cspinmask;
    *_dcport &= ~_dcpinmask;

    SPI.transfer(reg);

    *_dcport |= _dcpinmask;

    uint8_t ret = SPI.transfer(0);

    *_csport |= _cspinmask;

    return ret;
}

void HX8357D::fillScreen(uint16_t color)
{
    fillRect(0, 0, HX8357_TFTWIDTH, HX8357_TFTHEIGHT, color);
}

void HX8357D::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{

    if ((x >= HX8357_TFTWIDTH) || (y >= HX8357_TFTHEIGHT)) {
        return;
    }

    if ((x + w - 1) >= HX8357_TFTWIDTH) {
        w = HX8357_TFTWIDTH  - x;
    }

    if ((y + h - 1) >= HX8357_TFTHEIGHT) {
        h = HX8357_TFTHEIGHT - y;
    }

    setAddrWindow(x, y, x + w - 1, y + h - 1);

    uint8_t pix_buf[12800];
    for (int i = 0; i < 6400; i++) {
        pix_buf[2 * i] = (color >> 8);
        pix_buf[2 * i + 1] = color & 0xff;
    }

    SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
    *_dcport |=  _dcpinmask;
    *_csport &= ~_cspinmask;

    //spi_dma_init();
    for (int i = 0; i < 24; ++i) {
        spi_dma_write(pix_buf, 12800, 0);
        waitForDMA();
    }

    *_csport |= _cspinmask;
    SPI.endTransaction();
}

void HX8357D::pushPixelsDMA(uint8_t *buf, uint16_t sz)
{
    SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
    *_dcport |=  _dcpinmask;
    *_csport &= ~_cspinmask;

    //spi_dma_init();
    spi_dma_write(buf, sz, 0);
    waitForDMA();

    *_csport |= _cspinmask;
    SPI.endTransaction();
}

bool HX8357D::waitForDMA()
{
    while (!spi_dma_done()) ;
}

void HX8357D::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    writeCommand(HX8357_CASET);                                                                                                                         // Column addr set
    writeData(x0 >> 8);
    writeData(x0 & 0xFF);                                                                                                                             // XSTART
    writeData(x1 >> 8);
    writeData(x1 & 0xFF);                                                                                                                             // XEND

    writeCommand(HX8357_PASET);                                                                                                                         // Row addr set
    writeData(y0 >> 8);
    writeData(y0);                                                                                                                             // YSTART
    writeData(y1 >> 8);
    writeData(y1);                                                                                                                             // YEND

    writeCommand(HX8357_RAMWR);                                                                                                                         // write to RAM
    SPI.endTransaction();
}

uint16_t HX8357D::rgbTo565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3);
}
