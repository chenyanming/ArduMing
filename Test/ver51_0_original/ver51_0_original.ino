
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

float imu_yaw_adjust = 0, imu_roll_adjust = 0, imu_pitch_adjust = 0;
float gyro_pitch_adjust = 2.15, gyro_roll_adjust = -2.05;  //gyro_yaw_adjust,

long x_ground = 0, y_ground = 0, z_ground = 0, yaw_ground = 0;
long throttle_lock = 0, height_lock = 0, x_lock, y_lock;
int groundMsgReady = 0;
uint32_t time_lastMsgRead = 0;

int height_sonar = 0, z_change_sonar = 0;

PID  PID_ROLL_SPEED, PID_PITCH_SPEED, PID_YAW_SPEED;
PID  PID_ROLL_ANGLE, PID_PITCH_ANGLE, PID_YAW_ANGLE;
PID  PID_X, PID_Y, PID_HEIGHT;
PID  PID_X_ACCEL, PID_Y_ACCEL, PID_HEIGHT_ACCEL;

float yaw_current_location = 0;    //for speed mode of yaw control only
int motorsOff = 0;

//for 3DR wireless module
char buffer[100];
int indexBuffer = 0;


void extractMsg()
{
  long x = strtol(strtok(buffer, ","), NULL, 10);
  long y = strtol(strtok(NULL, ","), NULL, 10);
  long z = strtol(strtok(NULL, ","), NULL, 10);
  long yaw = strtol(strtok(NULL, ","), NULL, 10);
  long x_change = strtol(strtok(NULL, ","), NULL, 10);
  long y_change = strtol(strtok(NULL, ","), NULL, 10);
  long z_change = strtol(strtok(NULL, ","), NULL, 10);
  long ck = strtol(strtok(NULL, ","), NULL, 10);

  long checksum = x + y + z + yaw + x_change + y_change + z_change;
  checksum = checksum % 1000;

  //hal.console->printf_P(PSTR("time: %d "),hal.scheduler->millis());

  if (checksum == ck)
  {
    x -= x_change;
    y -= y_change;
    //z -= z_change;

    if (abs(z_change) <= 50)
      z_change_sonar = -z_change;

    if (x > 99 && x < 330)
      x_ground = x;
    else
      hal.console->printf_P(PSTR("x_ground error %ld\n"), x);

    if (y > 99 && y < 330)
      y_ground = y;
    else
      hal.console->printf_P(PSTR("y_ground error %ld\n"), y);

    if (z > -230 && z < 0)
      z_ground = z;
    else
      hal.console->printf_P(PSTR("z_ground error %ld\n"), z);

    yaw_ground = yaw;


    time_lastMsgRead = hal.scheduler->millis();
    groundMsgReady = 1;
    hal.console->printf_P(PSTR("checksum ok , x %ld y %ld z %ld yaw %ld x_change %ld y_change %ld z_change %ld\n"), x, y, z, yaw, x_change, y_change, z_change);
  }
  else
    hal.console->printf_P(PSTR("checksum fail\n"));


}

void get3DRData()
{
  int bytesAvailable = hal.uartB->available();

  while (bytesAvailable > 0)
  {
    char c = (char)hal.uartB->read();
    if (c == 'S')
      indexBuffer = 0;
    else if (c == 'E')
    {
      buffer[indexBuffer] = '\0';
      extractMsg();
    }
    else
    {
      buffer[indexBuffer] = c;
      indexBuffer++;
    }
    bytesAvailable--;
  }
}

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
  PID_X.reset_I(); PID_Y.reset_I(); PID_HEIGHT.reset_I();
}

void setup()
{

  /*enable 3DR wireless module on uartB, with 57600 baudrate, 128 read buffer, 128 transmition buffer*/
  hal.uartB->begin(57600, 128, 128);

  /*uartC is used to communicate with the Master*/
  hal.uartC->begin(9600, 128, 128);

  /*enable sonar*/
  AP_HAL::AnalogSource *analog_source = hal.analogin->channel(8);
  sonar = new AP_RangeFinder_MaxsonarXL(analog_source, &mode_filter);
  sonar->calculate_scaler(AP_RANGEFINDER_MAXSONARXL, 5);   // setup scaling for sonar

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
  //imu_yaw_adjust  = -(ToDeg(imu_yaw));
  imu_pitch_adjust = -(ToDeg(imu_pitch));
  imu_roll_adjust = -(ToDeg(imu_roll));
  hal.console->printf("IMU adjustment done, adjust factors: pitch %f roll %f \n", imu_pitch_adjust, imu_roll_adjust);

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

  PID_PITCH_ANGLE.kP(4.2);
  PID_ROLL_ANGLE.kP(4.2);
  PID_YAW_ANGLE.kP(12);

  PID_HEIGHT.kP(2);
  //PID_HEIGHT.kI(0.5);
  //PID_HEIGHT.imax(5);
  PID_HEIGHT.kD(1);

  PID_HEIGHT_ACCEL.kP(1);
  //PID_HEIGHT_ACCEL.kI(1);
  //PID_HEIGHT_ACCEL.imax(50);
  //PID_HEIGHT_ACCEL.kD(0.1);

  PID_X.kP(0.2);
  //PID_X.kI(0.1);
  //PID_X.imax(1);
  PID_X.kD(0.5);

  PID_Y.kP(0.2);
  //PID_Y.kI(0.1);
  //PID_Y.imax(1);
  PID_Y.kD(0.5);

  PID_X_ACCEL.kP(0.5);

  //PID_X_ACCEL.kD(0.1);

  PID_Y_ACCEL.kP(0.5);
  //PID_Y_ACCEL.kI(1);
  //PID_Y_ACCEL.imax(5);
  //PID_Y_ACCEL.kD(0.1);

}

void loop()
{



  uint16_t inputs[8];  // store value of 8 input channels

  //Read RC(radio code) singals from 8 input channels and store in inputs array
  hal.rcin->read(inputs, 8);
  long roll, pitch, throttle, yaw;

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
  imu_yaw = ToDeg(imu_yaw) + imu_yaw_adjust;
  imu_pitch = ToDeg(imu_pitch) + imu_pitch_adjust;
  imu_roll = ToDeg(imu_roll) + imu_roll_adjust;

  int temp_sonar_data = sonar->read();
  int height_change = abs(temp_sonar_data - height_sonar);

  if (temp_sonar_data > 19 && temp_sonar_data < 190 && height_change < 50 && abs(imu_pitch) < 27 && abs(imu_roll) < 27)
    height_sonar = temp_sonar_data;
  else
    // hal.console->printf("Sonar noise found %d\n", temp_sonar_data);



    /*
    hal.console->printf_P(PSTR("IMU YAW %4.1f PIT %4.1f ROLL %4.1f height_change %d"), imu_yaw, imu_pitch, imu_roll, height_change);
    hal.console->print("dist:");
    hal.console->print(height_sonar);
    hal.console->print("\traw:");
    hal.console->print(sonar->raw_value);
    hal.console->println();*/

    get3DRData();

  static int run_once = 1;
  static int run_once_each_on = 1;

  if (groundMsgReady)
  {
    groundMsgReady = 0;
    //imu_yaw_adjust += constrain(yaw_ground - imu_yaw, -2, 2);

    /*****************finxing imu yaw drift********************/
    //hal.console->printf_P(PSTR("ch5 %ld "), (long)inputs[4]);
    //hal.console->printf_P(PSTR("time: %d "),hal.scheduler->millis());
    //hal.console->printf_P(PSTR("groundMsgReady: x %ld y %ld z %ld yaw %ld imu_yaw %4.1f PIT %4.1f ROLL %4.1f imu_yaw_adjust %4.1f\n"), x_ground,y_ground, z_ground, yaw_ground, imu_yaw, imu_pitch, imu_roll, imu_yaw_adjust);



    float yaw_error = imu_yaw - yaw_ground;

    if (run_once)
    {
      imu_yaw_adjust -= yaw_error;
      imu_yaw = yaw_ground;
    }
    else if (yaw_error > 0 && yaw_error < 0.15)
    {
      imu_yaw_adjust -= yaw_error;
      imu_yaw = yaw_ground;
    }
    /***************************************************/

    if (run_once)
    {
      x_lock = x_ground;
      y_lock = y_ground;
    }

    run_once = 0;

  }

  if (inputs[4] < 1300)
  {
    if (run_once_each_on)
    {
      run_once_each_on = 0;
      PID_HEIGHT.reset_I();
      throttle_lock = throttle;
      //height_lock = z_ground;
      height_lock = height_sonar;
    }
  }

  if (inputs[4] >= 1300)
    run_once_each_on = 1;



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



  if (throttle > 1030)
  { //when motors is on do



    //hal.console->printf_P(PSTR("delay_msg: %d \n"), delay_msg);
    Vector3f accel = imu.get_accel();
    uint32_t delay_msg = hal.scheduler->millis() - time_lastMsgRead;

    if (run_once == 0)
    {
      float x_pid_output = -constrain(PID_X.get_pid((float)x_lock - x_ground, 1), -20, 20);
      float y_pid_output = -constrain(PID_Y.get_pid((float)y_lock - y_ground, 1), -20, 20);


      if (delay_msg > 120)
      {
        x_pid_output = 0;
        y_pid_output = 0;
        hal.console->printf_P(PSTR("lost connection\n"));
      }

      float x_accel = accel.x;
      float x_accel_pid_output = constrain(PID_X_ACCEL.get_pid((float)x_pid_output - x_accel, 1), -20, 20);
      pitch = x_accel_pid_output;

      float y_accel = -accel.y;
      float y_accel_pid_output = constrain(PID_Y_ACCEL.get_pid((float)y_pid_output - y_accel, 1), -20, 20);
      roll = y_accel_pid_output;

      //hal.console->printf_P(PSTR("z_accel %4.2f x_accel %4.2f x_accel_pid_output %4.2f pitch %ld\n"), z_accel, x_accel, x_accel_pid_output, pitch);
      //hal.console->printf_P(PSTR("x_accel %4.2f x_accel_pid_output %4.2f y_accel %4.2f y_accel_pid_output %4.2f imu_roll  %4.2f  \n"), x_accel, x_accel_pid_output, y_accel, y_accel_pid_output, imu_roll);

      //hal.console->printf_P(PSTR("height_lock %ld x_lock %ld y_lock %ld\n"), height_lock, x_lock, y_lock);

      //hal.console->printf_P(PSTR("x_lock %ld x_ground %ld x_pid_output %4.2f y_lock %ld y_ground %ld y_pid_output %4.2f\n"), x_lock, x_ground, x_pid_output, y_lock, y_ground, y_pid_output);

      //hal.console->printf_P(PSTR("x_lock %ld x_ground %ld pitch %ld y_lock %ld y_ground %ld roll %ld height_lock %ld height_sonar %d throttle %ld \n"), x_lock, x_ground, pitch, y_lock, y_ground, roll, height_lock, height_sonar, throttle);

      //hal.console->printf_P(PSTR("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f\n"), accel.x, accel.y, accel.z, accel.length());


      /***************************************************/
    }

    if (inputs[4] < 1300)
    {
      /****************height control******************/

      int desired_height = height_sonar + z_change_sonar;

      float height_pid_output = constrain(PID_HEIGHT.get_pid((float)height_lock - desired_height, 1), -50, 50);

      /*
      float height_pid_output = constrain(PID_HEIGHT.get_pid((float)height_lock - z_ground, 1), -50, 50);
      if(delay_msg > 120)
        height_pid_output = 0;*/

      //acceleration PID control
      float z_accel = accel.z;
      float height_accel_pid_output = constrain(PID_HEIGHT_ACCEL.get_pid((float)(GRAVITY + height_pid_output) - z_accel, 1), -120, 120);

      throttle = throttle_lock + height_accel_pid_output;
      //hal.console->printf_P(PSTR("z_accel %4.2f height_lock %ld height_sonar %d height_pid_output %4.2f height_accel_pid_output: %4.2f throttle %ld \n"), z_accel, height_lock, height_sonar, height_pid_output, height_accel_pid_output, throttle);
      //hal.console->printf_P(PSTR("z_accel %4.2f height_lock %ld z_ground %ld height_pid_output %4.2f height_accel_pid_output: %4.2f throttle %ld \n"), z_accel, height_lock, z_ground, height_pid_output, height_accel_pid_output, throttle);
    }

    /*
    if(motorsOff == 1)
    {
        motorsOff = 0;
        yaw_current_location = imu_yaw;
    }*/

    // PID to adjust the angle on roll/yaw/pitch
    float roll_angle_pid_output = constrain(PID_ROLL_ANGLE.get_pid((float)roll - imu_roll, 1), -250, 250);
    float pitch_angle_pid_output = constrain(PID_PITCH_ANGLE.get_pid((float)pitch - imu_pitch, 1), -250, 250);
    //float yaw_angle_pid_output = constrain(PID_YAW_ANGLE.get_pid((float)yaw - imu_yaw, 1), -360, 360);    // angle mode of yaw control

    float yaw_angle_pid_output = constrain(PID_YAW_ANGLE.get_pid(yaw_degree_adjust(yaw_current_location - imu_yaw), 1), -360, 360);      // speed mode of yaw control

    /*
    if(yaw < -8 || yaw > 8)
    {
        yaw_angle_pid_output = yaw;
        yaw_current_location = imu_yaw;
    }  */

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



    // hal.rcout->write(MOTOR_FL, throttle + roll_pid_output + pitch_pid_output - yaw_pid_output);
    // hal.rcout->write(MOTOR_BL, throttle + roll_pid_output - pitch_pid_output + yaw_pid_output);
    // hal.rcout->write(MOTOR_BR, throttle - roll_pid_output - pitch_pid_output - yaw_pid_output);
    // hal.rcout->write(MOTOR_FR, throttle - roll_pid_output + pitch_pid_output + yaw_pid_output);

    int throttle1 = throttle + pitch_pid_output + roll_pid_output - yaw_pid_output;
    int throttle2 = throttle - pitch_pid_output + roll_pid_output + yaw_pid_output;
    int throttle3 = throttle - pitch_pid_output - roll_pid_output - yaw_pid_output;
    int throttle4 = throttle + pitch_pid_output - roll_pid_output + yaw_pid_output;

    hal.rcout->write(MOTOR_FL, throttle1);
    hal.rcout->write(MOTOR_BL, throttle2);
    hal.rcout->write(MOTOR_BR, throttle3);
    hal.rcout->write(MOTOR_FR, throttle4);

    hal.console->printf_P(PSTR("IMU YAW %4.1f PIT %4.1f ROLL %4.1f\r\n"), imu_yaw, imu_pitch, imu_roll);
    hal.console->printf_P(PSTR("remote YAW %ld PIT %ld ROLL %ld TRO %ld\r\n"), yaw, pitch, roll, throttle);
    hal.uartC->printf_P(PSTR("%d,%d,%d,%d\n"), throttle1, throttle2, throttle3, throttle4);

    /*
    static int count = 0;
    count++;
    if(count == 20)
    {
      hal.console->printf_P(PSTR("IMU YAW %4.1f PIT %4.1f ROLL %4.1f\r\n"), imu_yaw, imu_pitch, imu_roll);
      hal.console->printf_P(PSTR("remote YAW %ld PIT %ld ROLL %ld TRO %ld\r\n"), yaw, pitch, roll, throttle);
      hal.console->printf_P(PSTR("output FL %ld BL %ld FR %ld BR %ld\r\n"), throttle + roll_pid_output + pitch_pid_output, throttle + roll_pid_output - pitch_pid_output, throttle - roll_pid_output + pitch_pid_output, throttle - roll_pid_output - pitch_pid_output);
      //hal.console->printf_P(PSTR("output FL %ld BL %ld FR %ld BR %ld\r\n"), throttle + roll_pid_output + pitch_pid_output - yaw_pid_output, throttle + roll_pid_output - pitch_pid_output + yaw_pid_output, throttle - roll_pid_output + pitch_pid_output + yaw_pid_output, throttle - roll_pid_output - pitch_pid_output - yaw_pid_output);
      //hal.console->printf_P(PSTR("gyro YAW %4.3f PIT %4.3f ROLL %4.3f PIT ERROR %4.3f ROLL ERROR %4.3f\r\n"), gyroYaw, gyroPitch, gyroRoll, pitch_angle_pid_output - gyroPitch, roll_angle_pid_output - gyroRoll);
      //hal.console->printf_P(PSTR("i_values YAW %4.3f PIT %4.3f ROLL %4.3f\r\n"), PID_YAW_SPEED.get_integrator(), PID_PITCH_SPEED.get_integrator(), PID_ROLL_SPEED.get_integrator());
      count=0;
    }*/


  } else
  { //when motors is off do

    //make sure the motors are off
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_BR, 1000);
    hal.rcout->write(MOTOR_FR, 1000);

    hal.uartC->printf_P(PSTR("1000,1000,1000,1000\n"));
    //motorsOff = 1;

    // reset all PID integrals parameters while not flying
    reset_integrators();

  }



}

//The AP_HAL_MAIN macro expands to a main function (either an "int main (void)" * or "int main (int argc, const char * argv[])", depending on platform)
AP_HAL_MAIN();

