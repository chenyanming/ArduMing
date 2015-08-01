#ifndef CONFIG_H
#define CONFIG_H

#include <SPI.h>
#include "PID_v1.h"

// #define DEBUG
// #define OUTPUT_TEMPERATURE
// #define OUTPUT_RAW_GYRO
// #define OUTPUT_READABLE_ROLLPITCHYAW

// #define CALI_THRO
// #define OUTPUT_READABLE_EULER
// #define OUTPUT_TEAPOT
// #define RC_OUTPUT
// #define EULER_OUTPUT
// #define THROTTLE_OUTPUT
// #define GYRO_OUTPUT
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

/**
 * mpu6050
 * CS - to digital pin 53  (SS pin)
 * SDI - to digital pin 51 (MOSI pin)
 * CLK - to digital pin 52 (SCK pin) *
 */
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

extern float rpy_pit, rpy_rol, rpy_yaw, kal_pit, kal_rol, kal_yaw, GyroX, GyroY, GyroZ;
extern int AcceX, AcceY, AcceZ;
extern float Ax, Ay, Az;

// Remote Control 
void rc_setup();
int rc_adjust();
void rc_get();
extern float roll, pitch, throttle, yaw, ch5, ch6, max_yaw;
extern boolean on_ch5;

// Motor
void motor_setup();
void motor_adjust();
void motor_output();
extern int throttle1;
extern int throttle2;
extern int throttle3;
extern int throttle4;
extern float pitch_angle_pid_output;
extern float roll_angle_pid_output;
extern float yaw_angle_pid_output;
extern PID pitch_angle;
extern PID roll_angle;
extern PID yaw_angle;

// MS5611
extern void ms5611_setup();
extern void ms5611_convert();
extern void ms5611_convert_ready();
extern void ms5611_get();
extern void ms5611_adjust();
extern float kal_alt;
extern unsigned char D1_timer;
extern unsigned char D2_timer;
unsigned long D1_Pres, D2_Temp; // store pressure and temperature value
extern boolean turn_ready;
extern boolean D1_ready;
extern boolean D2_ready;
extern float _Temperature, _Pressure, _altitude;
// extern float _ground_temperature, _ground_pressure;

#endif
