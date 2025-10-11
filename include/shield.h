#ifndef SHIELD_H
#define SHIELD_H

#include <Arduino.h>
#include <RTClib.h>

// CAN Shield
#define CANCS 10          // CAN chip select pin
#define CAN0_INT 2        // CAN interrupt pin

// Datalogger Shield
RTC_DS1307 rtc;           // Set the type of RTC chip being used
#define DS1307_ADR 0x68   // Set the I2C of the RTC chip
#define SDCS 6            // Chip select pin of SD/microSD card being used to store data

// Peripherals
#define WRITE_LED 7       // pin for write LED
#define STATUS_LED 8      // pin for status LED
#define RECORD_BUTTON A3  // pin for record start button

void shieldInit();
void lightUpdate(int light, int value);
void i2cBusRecovery();


#endif // SHIELD_H