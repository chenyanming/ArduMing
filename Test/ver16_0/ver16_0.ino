
/*
**    Hong Kong Polytechnic Unversity 2014-2015 Academic Year
**            Undergraduate Final Year Project
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

/*Motors is connected(indirectly) on output channels*/
#define MOTOR_FR   0    // Front right motors, output channel 1
#define MOTOR_BL   1    // back left motors, output channel 2
#define MOTOR_FL   2    // Front left motors, output channel 3
#define MOTOR_BR   3    // back right motors, output channel 4

//Specify the hardware is apm2.x, the "hal" variable name cannot be changed, it is used inside "AP_HAL_MAIN()"
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  //create an aliase for AP_HAL_AVR_APM2, when we using other version of apm board we can simply change "AP_HAL_AVR_APM2".

//IMU(accelerometer and gyroscopes)
AP_InertialSensor_MPU6000 imu;

float imu_yaw_adjust, imu_roll_adjust, imu_pitch_adjust;
float gyro_pitch_adjust = 2.15, gyro_roll_adjust = -2.05;  //gyro_yaw_adjust,

PID  PID_ROLL_SPEED, PID_PITCH_SPEED, PID_YAW_SPEED;
PID  PID_ROLL_ANGLE, PID_PITCH_ANGLE, PID_YAW_ANGLE;

float yaw_current_location = 0;    //for speed mode of yaw control only
int motorsOff = 0;


//This function will return a mapping value(in range out_min-out_min) for the input x(in range in_min-in_max)
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//for speed mode of yaw only
float yaw_degree_adjust(float degree)
{
  if (degree < -180)
    return degree + 360;
  else if (degree > 180)
    return degree - 360;
  else
    return degree;
}

// reset all PID integrators to zero
void reset_integrators()
{
  PID_ROLL_SPEED.reset_I();  PID_PITCH_SPEED.reset_I();  PID_YAW_SPEED.reset_I();
  PID_ROLL_ANGLE.reset_I();   PID_PITCH_ANGLE.reset_I();   PID_YAW_ANGLE.reset_I();
}

void setup()
{
  /*enable 3DR wireless module on uartB, with 57600 baudrate, 128 read buffer, 128 transmition buffer*/
  hal.uartB->begin(57600, 128, 128);
  /*uartC is used to communicate with the Master*/
  hal.uartC->begin(9600, 128, 128);

  //hal.console->printf_P(PSTR("Version 2.6 Kp 0.1 Kd 0.07 + no imu fix\r\n"));
  /*enable IMU(accelerometer and gyroscopes) with Digital Motion Processing Unit*/
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, 1);
  imu.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ, NULL);
  hal.scheduler->suspend_timer_procs();
  imu.dmp_init();
  imu.push_gyro_offsets_to_dmp();
  hal.scheduler->resume_timer_procs();


  hal.console->println("\nWait 15 seconds to adjust IMU");
  hal.scheduler->delay(15000);
  while (imu.num_samples_available() == 0);  //wait untill received new IMU data
  imu.update();
  float imu_roll, imu_pitch, imu_yaw;
  imu.quaternion.to_euler(&imu_roll, &imu_pitch, &imu_yaw);
  imu_yaw_adjust  = -(ToDeg(imu_yaw));
  imu_pitch_adjust = -(ToDeg(imu_pitch));
  imu_roll_adjust = -(ToDeg(imu_roll));
  hal.console->printf("IMU adjustment done, adjust factors: yaw %f pitch %f roll %f \n", imu_yaw_adjust, imu_pitch_adjust, imu_roll_adjust);


  /*enable output channels for motors*/
  hal.rcout->set_freq(0xF, 490);  //specify indicated output channels with output frequench 490Hz.
  hal.rcout->enable_mask(0xFF);   //enable output channels, output channels 1-4 should be connected to motors.

  PID_PITCH_SPEED.kP(0.7);
  PID_PITCH_SPEED.kI(1);
  PID_PITCH_SPEED.imax(50);
  PID_PITCH_SPEED.kD(0.02);

  PID_ROLL_SPEED.kP(0.7);
  PID_ROLL_SPEED.kI(1);
  PID_ROLL_SPEED.imax(50);
  PID_ROLL_SPEED.kD(0.02);

  PID_YAW_SPEED.kP(2.0);
  PID_YAW_SPEED.kI(1);
  PID_YAW_SPEED.imax(50);
  PID_YAW_SPEED.kD(0.02);

  PID_PITCH_ANGLE.kP(3);
  PID_ROLL_ANGLE.kP(3);
  PID_YAW_ANGLE.kP(12);
}

void loop()
{



  uint16_t inputs[8];  // store value of 8 input channels

  //Read RC(radio code) singals from 8 input channels and store in inputs array
  hal.rcin->read(inputs, 8);


  long roll, pitch, throttle, yaw, throttle1, throttle2, throttle3, throttle4;

  /*
  Transform readed RC signal to roll, pitch, throttle, yaw values
  Parameter 1 is the readed RC value in input channel X (theoretically range from 1000 to 2000 )
  Parameter 2 and 3 are actual min/max values (real range)
  Parameter 4 and 5 are wantted min/max values
  roll and pitch should range from -45 to 45 degrees.
  yaw should range from -150 to 150 degrees.
  throttle should range from 1000 to 2000.*/
  /*
  roll = map(inputs[0], 1090, 1902,
  pitch = map(inputs[1], 1090, 1907, -45, 45);
  throttle = map(inputs[2], 1014, 1929, 1000, 1800);  //2000 is the maximum of throttle, set a lower value to remain some room for PID controls
  yaw = map(inputs[3], 1083, 1907, -180, 180);
  */

  roll = map(inputs[3], 1083, 1907, -45, 45);
  pitch = map(inputs[1], 1090, 1907, -45, 45);
  throttle = map(inputs[2], 1014, 1929, 1000, 1800);  //2000 is the maximum of throttle, set a lower value to remain some room for PID controls
  yaw = map(inputs[0], 1090, 1911, -180, 180);

  //fixing small errors of signals from remote controller
  if (abs(roll) <= 3)
    roll = 0;
  if (abs(pitch) <= 3)
    pitch = 0;
  if (abs(yaw) <= 3)
    yaw = 0;

  /*get new IMU data and transform it to degree*/
  while (imu.num_samples_available() == 0);  //wait untill received new IMU data
  imu.update();
  float imu_roll, imu_pitch, imu_yaw;
  imu.quaternion.to_euler(&imu_roll, &imu_pitch, &imu_yaw);
  imu_yaw = ToDeg(imu_yaw);
  imu_pitch = ToDeg(imu_pitch) + imu_pitch_adjust;
  imu_roll = ToDeg(imu_roll) + imu_roll_adjust;



  //get gyroscopes data
  Vector3f gyroData = imu.get_gyro();
  float gyroYaw = ToDeg(gyroData.z), gyroPitch = ToDeg(gyroData.y) + gyro_pitch_adjust, gyroRoll = ToDeg(gyroData.x) + gyro_roll_adjust;

  /*
  static int count = 0;
  count++;
  if(count == 20)
  {
    hal.console->printf_P(PSTR("IMU YAW %4.1f PIT %4.1f ROLL %4.1f\r\n"), imu_yaw, imu_pitch, imu_roll);
    hal.console->printf_P(PSTR("remote YAW %ld PIT %ld ROLL %ld TRO %ld\r\n"), yaw, pitch, roll, throttle);
    count=0;
    }*/
  //hal.console->printf_P(PSTR("IMU YAW %4.1f PIT %4.1f ROLL %4.1f\r\n"), imu_yaw, imu_pitch, imu_roll);

  if (throttle > 1030)
  {
    //when motors is on do
    /*
    // PID to adjust the rotating speed on roll/yaw/pitch
    long pitch_pid_output =  (long) constrain(PID_PITCH_SPEED.get_pid(gyroPitch - pitch, 1), - 500, 500);
    long roll_pid_output =  (long) constrain(PID_ROLL_SPEED.get_pid(gyroRoll - roll, 1), -500, 500);
    long yaw_pid_output =  (long) constrain(PID_YAW_SPEED.get_pid(gyroYaw - yaw, 1), -500, 500);

    hal.rcout->write(MOTOR_FL, throttle - roll_pid_output - pitch_pid_output);
    hal.rcout->write(MOTOR_BL, throttle - roll_pid_output + pitch_pid_output);
    hal.rcout->write(MOTOR_FR, throttle + roll_pid_output - pitch_pid_output);
    hal.rcout->write(MOTOR_BR, throttle + roll_pid_output + pitch_pid_output);
    */

    if (motorsOff == 1)
    {
      motorsOff = 0;
      yaw_current_location = imu_yaw;
    }

    // PID to adjust the angle on roll/yaw/pitch
    float roll_angle_pid_output = constrain(PID_ROLL_ANGLE.get_pid((float)roll - imu_roll, 1), -250, 250);
    float pitch_angle_pid_output = constrain(PID_PITCH_ANGLE.get_pid((float)pitch - imu_pitch, 1), -250, 250);
    //float yaw_angle_pid_output = constrain(PID_YAW_ANGLE.get_pid((float)yaw - imu_yaw, 1), -360, 360);    // angle mode of yaw control

    float yaw_angle_pid_output = constrain(PID_YAW_ANGLE.get_pid(yaw_degree_adjust(yaw_current_location - imu_yaw), 1), -360, 360);      // speed mode of yaw control
    if (yaw < -8 || yaw > 8)
    {
      yaw_angle_pid_output = yaw;
      yaw_current_location = imu_yaw;
    }

    // PID to adjust the rotating speed on roll/yaw/pitch
    long pitch_pid_output =  (long) constrain(PID_PITCH_SPEED.get_pid(pitch_angle_pid_output - gyroPitch, 1), - 500, 500);
    long roll_pid_output =  (long) constrain(PID_ROLL_SPEED.get_pid(roll_angle_pid_output - gyroRoll, 1), -500, 500);
    long yaw_pid_output =  (long) constrain(PID_YAW_SPEED.get_pid(yaw_angle_pid_output - gyroYaw, 1), -500, 500);

    /*
    hal.rcout->write(MOTOR_FL, throttle + roll_pid_output + pitch_pid_output);
    hal.rcout->write(MOTOR_BL, throttle + roll_pid_output - pitch_pid_output);
    hal.rcout->write(MOTOR_FR, throttle - roll_pid_output + pitch_pid_output);
    hal.rcout->write(MOTOR_BR, throttle - roll_pid_output - pitch_pid_output);
    */
    //hal.console->printf_P(PSTR("output FL %ld BL %ld FR %ld BR %ld\r\n"), throttle + roll_pid_output + pitch_pid_output, throttle + roll_pid_output - pitch_pid_output, throttle - roll_pid_output + pitch_pid_output, throttle - roll_pid_output - pitch_pid_output);

    throttle1 = throttle + roll_pid_output + pitch_pid_output - yaw_pid_output;
    throttle2 = throttle + roll_pid_output - pitch_pid_output + yaw_pid_output;
    throttle3 = throttle - roll_pid_output - pitch_pid_output - yaw_pid_output;
    throttle4 = throttle - roll_pid_output + pitch_pid_output + yaw_pid_output;

    hal.rcout->write(MOTOR_FL, throttle1);
    hal.rcout->write(MOTOR_BL, throttle2);
    hal.rcout->write(MOTOR_BR, throttle3);
    hal.rcout->write(MOTOR_FR, throttle4);


    static int count = 0;
    count++;
    if(count == 20)
    {
      hal.console->printf_P(PSTR("IMU YAW %4.1f PIT %4.1f ROLL %4.1f\r\n"), imu_yaw, imu_pitch, imu_roll);
      // hal.console->printf_P(PSTR("remote YAW %ld PIT %ld ROLL %ld TRO %ld\r\n"), yaw, pitch, roll, throttle);
      //hal.console->printf_P(PSTR("output FL %ld BL %ld FR %ld BR %ld\r\n"), throttle + roll_pid_output + pitch_pid_output, throttle + roll_pid_output - pitch_pid_output, throttle - roll_pid_output + pitch_pid_output, throttle - roll_pid_output - pitch_pid_output);
      // hal.console->printf_P(PSTR("output FL %ld BL %ld FR %ld BR %ld\r\n"), throttle + roll_pid_output + pitch_pid_output - yaw_pid_output, throttle + roll_pid_output - pitch_pid_output + yaw_pid_output, throttle - roll_pid_output + pitch_pid_output + yaw_pid_output, throttle - roll_pid_output - pitch_pid_output - yaw_pid_output);
      // hal.console->printf_P(PSTR("gyro YAW %4.3f PIT %4.3f ROLL %4.3f PIT ERROR %4.3f ROLL ERROR %4.3f\r\n"), gyroYaw, gyroPitch, gyroRoll, pitch_angle_pid_output - gyroPitch, roll_angle_pid_output - gyroRoll);
      // hal.console->printf_P(PSTR("i_values YAW %4.3f PIT %4.3f ROLL %4.3f\r\n"), PID_YAW_SPEED.get_integrator(), PID_PITCH_SPEED.get_integrator(), PID_ROLL_SPEED.get_integrator());
      count=0;
    }
    hal.uartB->printf_P(PSTR("%ld,%ld,%ld,%ld\n"), throttle1, throttle2, throttle3, throttle4);
    hal.uartC->printf_P(PSTR("%ld,%ld,%ld,%ld\n"), throttle1, throttle2, throttle3, throttle4);

  } else { //when motors is off do

    //make sure the motors are off
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_BR, 1000);
    hal.rcout->write(MOTOR_FR, 1000);
    hal.uartB->printf_P(PSTR("1000,1000,1000,1000\n"));
    hal.uartC->printf_P(PSTR("1000,1000,1000,1000\n"));

    motorsOff = 1;

    // reset all PID integrals parameters while not flying
    reset_integrators();

  }



}

//The AP_HAL_MAIN macro expands to a main function (either an "int main (void)" * or "int main (int argc, const char * argv[])", depending on platform)
AP_HAL_MAIN();

