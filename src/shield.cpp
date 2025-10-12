
// Sparkfun CAN Shield
// Adafruit Datalogger shield for rtc

#include <Arduino.h>

#include <RTClib.h>

#include "shield.h"

void shieldInit() {
#if defined(ARDUINO_AVR_UNO)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
#else
  pinMode(A0, INPUT_PULLDOWN);
  pinMode(A1, INPUT_PULLDOWN);
  pinMode(A2, INPUT_PULLDOWN);
  pinMode(A3, INPUT_PULLDOWN);
#endif // ARDUINO_AVR_UNO

  pinMode(RECORD_BUTTON,INPUT_PULLUP);
  digitalWrite(RECORD_BUTTON, HIGH);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  pinMode(WRITE_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(WRITE_LED, LOW);
  digitalWrite(STATUS_LED, LOW);
}

void lightUpdate(int light, int value) {
  if (value > 0) {
    digitalWrite(light, HIGH);
  } else if (value == 0) {
    digitalWrite(light, LOW);
  } else if (value == -1) {
    digitalWrite(light, !digitalRead(light));
  }
}

void i2cBusRecovery() {

  Serial.println("Resetting I2C state");

  pinMode(SDA, OUTPUT);           // set as outputs for bitbang
  pinMode(SCL, OUTPUT);

  digitalWrite(SDA, HIGH);        // SDA HIGH means no data
  for (int i = 0; i < 9; i++) {   // iter clock 9 times, byte + ACK
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }
  digitalWrite(SDA, LOW);         // set low
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);        // low to high --> STOP

  pinMode(SDA, INPUT_PULLUP);     // reset for I2C
  pinMode(SCL, INPUT_PULLUP);

  Wire.beginTransmission(DS1307_ADR); // DS1307 I2C address
  Wire.write(0x00);               // Seconds register
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADR, 1);
  if (Wire.available()) {
    uint8_t seconds = Wire.read();
    
    if (seconds & 0b10000000) {   // if clock halt is 1
      Serial.println("clock halted, turning on ...");

      // Clear CH bit
      seconds &= 0b01111111;

      Wire.beginTransmission(DS1307_ADR);
      Wire.write(0x00);           // seconds register
      Wire.write(seconds);        // updated value with CH bit cleared
      Wire.endTransmission();
    }
  }
}