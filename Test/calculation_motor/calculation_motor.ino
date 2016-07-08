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

/*Motors is connected(indirectly) on output channels*/
#define MOTOR_FR   0    // Front right motors, output channel 1
#define MOTOR_BL   1    // back left motors, output channel 2
#define MOTOR_FL   2    // Front left motors, output channel 3  
#define MOTOR_BR   3    // back right motors, output channel 4

#define GRAVITY -10

//Specify the hardware is apm2.x, the "hal" variable name cannot be changed, it is used inside "AP_HAL_MAIN()"
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  //create an aliase for AP_HAL_AVR_APM2, when we using other version of apm board we can simply change "AP_HAL_AVR_APM2". 

//IMU(accelerometer and gyroscopes)
AP_InertialSensor_MPU6000 imu;

// filter for sonar
ModeFilterInt16_Size5 mode_filter(2);
//sonar
AP_RangeFinder_MaxsonarXL *sonar;






void setup() 
{

    hal.console->println("motor calculation start");
    
    /*enable output channels for motors*/
  hal.rcout->set_freq(0xF, 490);  //specify indicated output channels with output frequench 490Hz.
  hal.rcout->enable_mask(0xFF);   //enable output channels, output channels 1-4 should be connected to motors. 
  
  
     
  	hal.rcout->write(MOTOR_FL, 2000);
  	hal.rcout->write(MOTOR_BL, 2000);
  	hal.rcout->write(MOTOR_FR, 2000);
  	hal.rcout->write(MOTOR_BR, 2000);
  
        hal.console->println("\nWait 4 seconds to output low throttle");
        hal.scheduler->delay(4000);
  
        hal.rcout->write(MOTOR_FL, 1000);
  	hal.rcout->write(MOTOR_BL, 1000);
  	hal.rcout->write(MOTOR_FR, 1000);
  	hal.rcout->write(MOTOR_BR, 1000);
        
        hal.console->println("finished");
  
}



void loop()
{

          hal.console->println("finished");

}

//The AP_HAL_MAIN macro expands to a main function (either an "int main (void)" * or "int main (int argc, const char * argv[])", depending on platform)
AP_HAL_MAIN();  

