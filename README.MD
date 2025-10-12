# Arduino Vehicle Datalogger

This code, along with some shields, turns your Arduino Uno R4 or R3 into a combination CAN bus and Analog data logger.

### ⚠️ Warning ⚠️
Ensure that the voltage of analog inputs does not exceed the limits of the hardware you are using!

## Configuration
### shield.h
This is where settings based on hardware are configured.

Check that chip select pins, status LEDs, input buttons, etc. are assigned appropriatly.

### options.h
This is where settings based on data collection are configured.

Up to six CAN messages can be filtered for, or no filter can be set and all CAN messages recorded.
Up to four analog data channels can be recorded. Pins A4 and A5 are used for I2C communication, so it is not possible to also record data on these pins.
Pacing for analog data can be configured, to only record analog data every nth CAN message recieved.

## Data Processing

Data saved is in a binary format, following the structure defined in `include/tCAN.h`.
For simplicity, analog signals are saved the same as CAN messages, with their message ID being equal to the value of the pin. 

There is an example script, `datapostprocessing/bintolog.py`, that converts binary files into human readable text logs of all message data.

## Shield Modification

Ensure that there are no overlapping chip select pins
I moved the chip select pin of the SD card on the RTC board to D6, cutting the trace, and using the provided breakouts.

Ensure that devices have appropriate pullups/downs
I wired 4.7kΩ pullup resistors to the SDA and SCL lines of the DS1307's I2C bus, using the provided breakouts on the RTC board.

## Reccomended Shields

### CAN bus

Any MCP2515 based shield can be made to work, but here are some suggestions:

Image + Link | Description | Price | Tested
:--: | ---- | ---: | :--:
[<img src="https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/3/13262-01.jpg" width="200">](https://www.sparkfun.com/can-bus-shield.html) | Shield with CAN connectivity, built in LEDs, and a joystick for input.<br>What I used to build this. | $39.95 | ✅
[<img src="https://productimages.microcenter.com/655014_464875_01_front_zoom.jpg" width="200">](https://www.microcenter.com/product/655014/inland-ks0411-can-bus-shield) | Much more affordable, but lacks extra peripherals. | $6.99 | 

### RTC

Any RTC chip should work, but here are some suggestions:

Image + Link | Description | Price | Tested
:--: | ---- | ---: | :--:
[<img src="https://m.media-amazon.com/images/I/61A8iYOE2JL._SX522_.jpg" width="200">](https://www.amazon.com/HiLetgo-Logging-Recorder-Logger-Arduino/dp/B00PI6TQWO) | A clone of an older Adafruit design, has an SD card port which I find preferable to microSD.<br>Also has plenty of space to add LEDs or buttons.<br>What I used to build this. | $7.99 | ✅
[<img src="https://cdn-shop.adafruit.com/230x173/1141-12.jpg" width="200">](https://www.adafruit.com/product/1141) | Uses PCF8523 instead of DS1307. An updated version of the board the one above was based on. | $13.95 | 
