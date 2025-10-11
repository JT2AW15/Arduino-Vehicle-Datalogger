#ifndef TCAN_H
#define TCAN_H

#include <Arduino.h>

struct tCAN {
  unsigned long rxID;
  bool rtr;
  bool ext;
  uint8_t dlc;
  uint8_t rxBuf[8];
  unsigned long time;
} ;

#endif // TCAN_H