# mkr1000-hx8357d

This repository contains several methods designed to push data very quickly (via DMA transfers to the SPI bus) to a HX8357D screen from Adafruit.
It is based on the Adafruit library :
```
https://github.com/adafruit/Adafruit_HX8357_Library
```
But only implements fillRect and fillScreen as of now. The DMA part is widely inspired from this repo :
```
https://github.com/manitou48/ZERO
```