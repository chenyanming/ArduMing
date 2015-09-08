//FL : CW
//FR : CCW
//BL : CCW
//BR : CW


/*
**    Hong Kong Polytechnic Unversity 2014-2015 Academic Year
**            Undergraduate Final Year Project
**                 Revised by King Ho(handsome man)
**    By David (Li Shunxing), Email: lishunxing@live.hk
**
**    Edited on "ArduPilot-Arduino-1.0.3-windows" IDE.
**    This program is arget to run on Ardupilot (apm2.6) control board which will drive a quadcopter
*/



/*libraries for Ardupilot's hardware operation and some ready equations to control the flying*/
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <PID.h>
#include <AP_RangeFinder.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_Declination.h>
#include <AP_HAL_Empty.h>
#include <AP_Compass.h> // Compass Library
#include <AP_GPS.h>   //GPS Library
#include <stdlib.h>

#define GRAVITY -10

float yaw_current_location = 0;    //for speed mode of yaw control only
int motorsOff = 0;


/*Motors is connected(indirectly) on output channels*/
#define MOTOR_FR   0    // Front right motors, output channel 1
#define MOTOR_BL   1    // back left motors, output channel 2
#define MOTOR_FL   2    // Front left motors, output channel 3  
#define MOTOR_BR   3    // back right motors, output channel 4

//Specify the hardware is apm2.x, the "hal" variable name cannot be changed, it is used inside "AP_HAL_MAIN()"
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  //create an aliase for AP_HAL_AVR_APM2, when we using other version of apm board we can simply change "AP_HAL_AVR_APM2".

//IMU(accelerometer and gyroscopes)
AP_InertialSensor_MPU6000 imu;

// filter for sonar
ModeFilterInt16_Size5 mode_filter(2);
//sonar
AP_RangeFinder_MaxsonarXL *sonar;


//for barometer
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
static uint32_t timer;

//for compass
AP_Compass_HMC5843 compass;
uint32_t timer_compass;


//for GPS
AP_GPS_UBLOX gps;
#define T6 1000000
#define T7 10000000
uint32_t timer_gps;
uint32_t timer_serial;


extern int AutoZcontrol;
extern int AutoXYcontrol;

void setup() {

  /*uartC is used to communicate with the Master*/
  hal.uartC->begin(115200, 128, 128);

  //  LED_Test();
  IMU_Init();

  //Compass_Init();

  Output_Init();

  GPS_Init();

  Sonar_Init();



  Baro_Init();



  PID_Init();

  //   Position_Init();

}

void loop() {

  read_input();

  IMU_Update();
  // Compass_Update();
  //  PosXY_Update();

  Sonar_update();
  //  Baro_Update();

  //  GPS_Update();




  if (AutoZcontrol == 1)
  {



    Z_PositionPID();

    //  XY_PositionPID();

  }

  PID_Caliburation();

  Matlab_print();



}
//The AP_HAL_MAIN macro expands to a main function (either an "int main (void)" * or "int main (int argc, const char * argv[])", depending on platform)
AP_HAL_MAIN();

