#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define USE_LONG_FILE_NAMES 1
#include <SdFat.h>
#include <RTClib.h>
#include <mcp_can.h>
//#include <mcp_can_dfs.h>

#include <shield.h>
#include <tCAN.h>
#include <options.h>

SdFs sd;
FsFile dataFile;
MCP_CAN CAN0(CANCS);

char datestring[25];
char binfilename[(sizeof(datestring) + sizeof(FILEPREFIX) + sizeof(EXT_BIN) + 3)];
uint8_t writestate = 0;

uint16_t writepos;
uint16_t readpos;
uint16_t bufitemcount = 0;
uint8_t canmsgcount = 0;
tCAN filebuffer[F_BUFFSIZE];

tCAN outputdata;

bool rtc_init();

bool query_buffer();
bool enqueue(const tCAN data);
bool dequeue(tCAN &outData);
void dump_buffer();

void setup() {
  Serial.begin(115200);
  Serial.println("CANalogger");
  Serial.println();

  shieldInit();
  
  while (!rtc_init());

  pinMode(SDCS, OUTPUT);
  if (!sd.begin(SDCS)) {
    Serial.println("SD card not found");
    lightUpdate(STATUS_LED, 1);
    while(1);
  }

#if defined(READCAN)
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
  }
  else {
    Serial.println("MCP2515 init failure");
    lightUpdate(STATUS_LED, 1);
    while(1);
  }

#if defined(READALLCAN)
    Serial.println("Reading all CAN Messages");
#endif // READALLCAN
#if !defined(READALLCAN)
  Serial.println("Setting Masks and Filters");
  #if defined(READPID)
  CAN0.init_Mask(0,0x7FF0000);
  CAN0.init_Filt(0,PID_REPLY<<16);
  CAN0.init_Filt(1,CAN_MSG0<<16);

  CAN0.init_Mask(1,0x7FF0000);
  CAN0.init_Filt(2,PID_REPLY<<16);
  CAN0.init_Filt(3,CAN_MSG1<<16);
  CAN0.init_Filt(4,CAN_MSG2<<16);
  CAN0.init_Filt(5,CAN_MSG3<<16);
  #endif // READPID
  #if !defined(READPID)
  CAN0.init_Mask(0,0x7FF0000);
  CAN0.init_Filt(0,CAN_MSG0<<16);
  CAN0.init_Filt(1,CAN_MSG1<<16);

  CAN0.init_Mask(1,0x7FF0000);
  CAN0.init_Filt(2,CAN_MSG2<<16);
  CAN0.init_Filt(3,CAN_MSG3<<16);
  CAN0.init_Filt(4,CAN_MSG4<<16);
  CAN0.init_Filt(5,CAN_MSG5<<16);
  #endif // READPID
  #if defined(READPIDREPLYONLY)
  CAN0.init_Mask(0,0x7FF0000);
  CAN0.init_Filt(0,PID_REPLY<<16);
  CAN0.init_Filt(1,PID_REPLY<<16);

  CAN0.init_Mask(1,0x7FF0000);
  CAN0.init_Filt(2,PID_REPLY<<16);
  CAN0.init_Filt(3,PID_REPLY<<16);
  CAN0.init_Filt(4,PID_REPLY<<16);
  CAN0.init_Filt(5,PID_REPLY<<16);
  #endif // READPIDREPLYONLY
#endif // READALLCAN
  CAN0.setMode(MCP_LISTENONLY);
#endif // READCAN

#if defined(READANA)
  Serial.println("Reading Analog pin 0");
#endif // READANA

  Serial.println("Listening ...\n");
}

void loop() {

  // write can message to buffer
#if defined(READCAN)
  if (!digitalRead(CAN0_INT) && writestate == 1) {
    unsigned long rxID;
    byte dlc;
    byte rxBuf[8];

    CAN0.readMsgBuf(&rxID, &dlc, rxBuf);

    tCAN msg;

    msg.rxID = rxID;
    msg.rtr = (msg.rxID & 0x40000000) == 0x40000000;
    msg.ext = (msg.rxID & 0x80000000) == 0x80000000;
    msg.dlc = dlc;
    for (int i = 0; i <= msg.dlc; i++) {
      msg.rxBuf[i] = rxBuf[i];
    }
    msg.time = millis();

    if (!enqueue(msg)) {
      dump_buffer();
      if(!enqueue(msg)) {
        Serial.print("Buffer err");
        writestate = 2;
      }
    }

    canmsgcount++;

  #if defined(SERIALLOGGING)
    char *msgString;
    if((rxID & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Ext ID: 0x%.8lX  DLC: %1d  Data:", (rxID & 0x1FFFFFFF), dlc);
    else
      sprintf(msgString, "    ID: 0x%.3lX       DLC: %1d  Data:", rxID, dlc);
  
    Serial.print(msgString);
    if((rxID & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " RTR");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<dlc; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
    Serial.println();
  #endif // SERIALLOGGING
  }
#endif // READCAN
  
  // when allowed, write analogs to buffer
#if defined(READANA) 
  if (query_buffer() && writestate == 1) {
    tCAN anadat;
    int analogval;

    anadat.rxID = A0;
    anadat.rtr = false;
    anadat.ext = false;
    anadat.dlc = 2;
    analogval = analogRead(A0);
    anadat.rxBuf[0] = analogval >> 8;
    anadat.rxBuf[1] = analogval;
    for (size_t i = 2; i < 8; i++) {
      anadat.rxBuf[i] = 0;
    }
    anadat.time = millis();
    
    if (!enqueue(anadat)) {
      dump_buffer();
      if(!enqueue(anadat)) {
        Serial.print("Buffer err");
        writestate = 2;
      }
    }

  #if defined(SERIALLOGGING)
    Serial.print("time");
    Serial.print(anadat.time);
    Serial.print(",");
    Serial.print(anadat.rxID,HEX);
    Serial.print(",");
    Serial.print(anadat.rxBuf[0],HEX);
  #endif // SERIALLOGGING
  } else {
    ;
  }
#endif // READANA

  // when buffer full, write msgs from buffer to SD card
  if (writestate == 2) {
#if !defined(ARDUINO_AVR_UNO)
    Serial.println("Saving " + String(binfilename));
#endif // ARDUINO_AVR_UNO
    dump_buffer();
    // close binary
    writestate = 0;

    dataFile.truncate();
#if !defined(ARDUINO_AVR_UNO)
    Serial.println("Filesize:" + String((long)dataFile.fileSize()) + "b");
#endif // ARDUINO_AVR_UNO
    dataFile.close();
#if !defined(ARDUINO_AVR_UNO)
    Serial.println(String(binfilename) + " closed");
#endif // ARDUINO_AVR_UNO

    readpos = 0;
    writepos = 0;
    canmsgcount = 0;
    lightUpdate(STATUS_LED, writestate);
    lightUpdate(WRITE_LED, writestate);
    Serial.println();
  }

  // data record button
  if (digitalRead(RECORD_BUTTON) == 0) {
    if (writestate == 0){
      // write to binary file
      writestate = 1;
      DateTime now = rtc.now();

      // set filename
      snprintf(datestring, sizeof(datestring), "%04u-%02u-%02u_%02u%02u%02u_%06lu", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), millis());
      snprintf(binfilename, sizeof(binfilename), "%s_%s.%s", FILEPREFIX, datestring, EXT_BIN);

      dataFile = sd.open(binfilename, O_CREAT | O_WRITE | O_READ);
#if !defined(ARDUINO_AVR_UNO)
      if (!dataFile) {
        Serial.println("Error opening file " + String(binfilename) + " for writing");
        writestate = 0;
      } else {
        if (!dataFile.preAllocate(FILE_SIZE)){
          Serial.print("Error pre-allocating space for " + String(binfilename));
        }
        Serial.println("Writing to " + String(binfilename));
      }
#else
      if (!dataFile) {
        writestate = 0;
      } else {
        if (!dataFile.preAllocate(FILE_SIZE)){
        }
      }
#endif // ARDUINO_AVR_UNO
    } else if (writestate == 1) {
      // finish saving binary from buffer
      writestate = 2;
    }
    
    lightUpdate(STATUS_LED, writestate);
    delay(BUT_DELAY);
  }

}  

bool rtc_init() {
  Wire.begin();

  if (! rtc.begin()) {
    lightUpdate(STATUS_LED, 1);
    delay(500);
    i2cBusRecovery();
    delay(500);
    return false;
  }

#if !defined(ARDUINO_AVR_UNO)
  DateTime now = rtc.now();

  Serial.println("RTC current date and time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  Serial.println();
#endif // ARDUINO_AVR_UNO
  
  lightUpdate(STATUS_LED, 0);

  return true;
}

bool query_buffer() {
#if !defined(ANA_RATE)
  return true;
#endif // ANA_RATE

#if defined(READANA)
  #if defined(ANA_RATE)
  if (!(canmsgcount % ANA_RATE)) {
    canmsgcount++;
    return true;
  }
  #endif // ANA_RATE
#endif // READANA
  return false;
}

bool enqueue(const tCAN data) {
  if (bufitemcount < F_BUFFSIZE) {
    filebuffer[writepos] = data;
    writepos++;
    writepos %= F_BUFFSIZE;  // Move tail to the next position
    bufitemcount++;
    return true;
  } else {
    return false;
  }
}

bool dequeue(tCAN &outData) {
  if (bufitemcount == 0) {
    return false;
  } else {
    outData = filebuffer[readpos];
    readpos++;
    readpos %= F_BUFFSIZE;  // Move head to the next position
    bufitemcount--;
    return true;
  }
}

void dump_buffer() {
  while (dequeue(outputdata)) {
    dataFile.write(reinterpret_cast<const uint8_t*>(&outputdata),sizeof(tCAN));
  }
  dataFile.flush();
  lightUpdate(WRITE_LED, -1);
}