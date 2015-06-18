#ifndef CONFIG_H
#define CONFIG_H

#include <SPI.h>

// #define DEBUG
// #define OUTPUT_TEMPERATURE
// #define OUTPUT_RAW_GYRO
// #define OUTPUT_READABLE_ROLLPITCHYAW

// #define CALI_THRO
// #define OUTPUT_READABLE_EULER
// #define OUTPUT_TEAPOT
#define RC_OUTPUT
#define EULER_OUTPUT
#define THRROTLE_OUTPUT
#define GYRO_OUTPUT
/**
 * Serial.print definitions for debug output
 */
#ifdef DEBUG
#define DEBUG_PRINT(x)       Serial.print(x)
#define DEBUG_PRINTF(x, y)   Serial.print(x, y)
#define DEBUG_PRINTLN(x)     Serial.println(x)
#define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTF(x, y)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNF(x, y)
#endif


extern byte teapotPacket[14];


const int blueled = 25;
const int yellowled = 26;
const int redled = 27;

/**
 * mpu6050.ino
 * CS - to digital pin 53  (SS pin)
 * SDI - to digital pin 51 (MOSI pin)
 * CLK - to digital pin 52 (SCK pin) *
 */
const int ChipSelPin1 = 53;
const int ChipSelPin2 = 40;

void spi_writeReg(int ChipSelPin, unsigned char reg, unsigned char data);
unsigned char spi_readReg(int ChipSelPin, unsigned char reg);
void spi_SetBits(int ChipSelPin, unsigned char reg, unsigned char mask);
void spi_ClrBits(int ChipSelPin, unsigned char reg, unsigned char mask);
void spi_readBytes(int ChipSelPin, byte reg, unsigned int length, byte *data);

unsigned int getFIFOCount(int ChipSelPin);
unsigned char dmpInitialize();
extern boolean dmpReady;     // set true if DMP initialization was successful
extern unsigned int packetSize; // number of unique bytes of data written by the DMP each time (FIFO can hold multiples of 42-bytes)
extern unsigned int fifoCount;       // count of all bytes currently in FIFO
extern byte fifoBuffer[64];          // FIFO storage buffer (in fact only 42 used...) // But in datasheet, fifo has 1024bytes and FIFO count may be large...

// Remote Control 
void rc_setup();
int rc_adjust();
void rc_get();
extern long roll, pitch, throttle, yaw, ch5;

#endif
