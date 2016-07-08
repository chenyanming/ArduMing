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
extern int AcceX_L, AcceY_L, AcceZ_L;
extern float AcceW_W, AcceX_W, AcceY_W, AcceZ_W;
extern float Vel_Z;

// Remote Control 
void rc_setup();
int rc_adjust();
void rc_get();
extern float roll, pitch, throttle, yaw, ch5, ch6, max_yaw;
extern boolean on_ch5, off_ch5;
extern unsigned int ch5_count;

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
extern float height_baro_pid_output;

extern PID pitch_angle;
extern PID roll_angle;
extern PID yaw_angle;
extern PID height_baro;
// extern PID height_sonar;
// extern PID height_sonar_2;

extern float pitch_p, pitch_i, pitch_d, roll_p, roll_i, roll_d, yaw_p, yaw_i, yaw_d;
extern float height_sonar_p, height_sonar_d;/*PID Sonar*/

void motor_slave_output();

// MS5611
extern void ms5611_setup();
extern void ms5611_convert();
extern void ms5611_convert_ready();
extern void ms5611_get();
extern void ms5611_adjust();
extern void ms5611_alt_average();
extern float baro_altitude;
// extern volatile unsigned char D1_timer;
// extern volatile unsigned char D2_timer;
extern volatile unsigned char convert_timer;
extern boolean convert_finish;
unsigned long D1_Pres, D2_Temp; // store pressure and temperature value
// extern boolean turn_ready;
// extern boolean D1_ready;
// extern boolean D2_ready;
extern boolean convert_ready;
extern float _Temperature, _Pressure, _altitude;
// extern float _ground_temperature, _ground_pressure;
extern float average_altitude;

// HMC5883L
boolean hmc_setup();
void hmc_get();
extern int mx, my, mz, mx_r, my_r;
extern float _heading;

// sonar
void sonar_setup();
void sonar_get();
void sonar_mode();
extern float _sonar_altitude;
extern float _sonar_mode_altitude;
extern float kal_sonar;
extern float height_sonar_pid_output;
extern float lock_sonar_altitude;
extern float Vel_sonar;
extern int sonar_error;
extern float SonarPID;

// mavlink
void mavlink_get();
extern int groundMsgReady;
extern float m_x, m_y, m_z, m_yaw, m_x_change, m_y_change, m_z_change, m_ck;

// mavlink and motor
extern float x_p, x_d;
extern float y_p, y_d;
extern PID pid_x;
extern PID pid_y;




#endif
