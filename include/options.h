#ifndef OPTIONS_H
#define OPTIONS_H


// Options

// CAN message IDs
const uint32_t PID_REPLY    = 0x07E8;
const uint32_t CAN_MSG0     = 0x0140;
const uint32_t CAN_MSG1     = 0x0140;
const uint32_t CAN_MSG2     = 0x0140;
const uint32_t CAN_MSG3     = 0x0140;
const uint32_t CAN_MSG4     = 0x0140;
const uint32_t CAN_MSG5     = 0x0140;

// Logged Data
#define READANA               // enables reading analog data
#define ANACHNLS 1            // number of analog channels to record data on. MAX 4
#define ANA_RATE 2            // how many CAN messages to record before recording every analog signal
#define READCAN               // enables reading CAN messages
//#define READPID             // filter accepts PID_REPLY and CAN_MSG0 to CAN_MSG4
//#define READPIDREPLYONLY    // filter only accepts PID_REPLY message
//#define READALLCAN          // disables CAN message filters

// Saving Files
#define FILEPREFIX "MYCARDATA"
#define EXT_BIN "bin"

// Debugging
//#define SERIALLOGGING       // Prints CAN and analog to serial. May cause messages to be missed


// Feel Options
#define BUT_DELAY   500       // button update delay in ms
#define FILE_SIZE   0x02FFFFFF// Preallocated storage space for file (bytes)
#define F_BUFFSIZE  128       // Size of message buffer in number of messages. Ideal size is a function of the size of tCAN, and 512 (SD write buffer)



#if defined(ARDUINO_AVR_UNO)
#define F_BUFFSIZE  25        // Uno R3 hardware limits force a smaller buffer size
#endif // ARDUINO_AVR_UNO


#endif	// OPTIONS_H
