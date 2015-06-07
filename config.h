#ifndef CONFIG_H
#define CONFIG_H

#include <SPI.h>

#define DEBUG
// #define OUTPUT_TEMPERATURE
#define OUTPUT_READABLE_EULER
// #define OUTPUT_TEAPOT
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



/**
 * CS - to digital pin 53  (SS pin)
 * SDI - to digital pin 51 (MOSI pin)
 * CLK - to digital pin 52 (SCK pin) *
 */
const int blueled = 25;
const int yellowled = 26;
const int redled = 27;
const int ChipSelPin1 = 53;
const int ChipSelPin2 = 40;

void spi_writeReg(int ChipSelPin, unsigned char reg, unsigned char data);
unsigned char spi_readReg(int ChipSelPin, unsigned char reg);
void spi_SetBits(int ChipSelPin, unsigned char reg, unsigned char mask);
void spi_ClrBits(int ChipSelPin, unsigned char reg, unsigned char mask);
void spi_readBytes(int ChipSelPin, byte reg, unsigned int length, byte *data);

unsigned int getFIFOCount(int ChipSelPin);
unsigned char dmpInitialize();

// Remote Control 
void rc_setup();
void rc_get();
extern long roll, pitch, throttle, yaw;

#endif