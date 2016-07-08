
extern  long roll, pitch, throttle, yaw;
extern float imu_roll, imu_pitch, imu_yaw;
extern  float gyroYaw, gyroPitch, gyroRoll;
extern long throttle_lock, barothrottle_lock;
extern int height_current, height_lock;
extern int baro_current, baro_lock;
//for start point record
extern int runonce_gps;;

//for optical position
extern long PositionX, PositionY;
extern long PosX_lock, PosY_lock;
extern int AutoXYcontrol;

//for heading of copter
extern float heading;

PID  PID_ROLL_SPEED, PID_PITCH_SPEED, PID_YAW_SPEED;
PID  PID_ROLL_ANGLE, PID_PITCH_ANGLE, PID_YAW_ANGLE;
PID  PID_X, PID_Y, PID_HEIGHT;
PID  PID_X_ACCEL, PID_Y_ACCEL, PID_HEIGHT_ACCEL;



// set PID parameter
// including angle and gyro speed
//
void  PID_Init() {


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

  PID_PITCH_ANGLE.kP(4);
  PID_ROLL_ANGLE.kP(4);
  PID_YAW_ANGLE.kP(12);

  PID_HEIGHT.kP(2);
  //PID_HEIGHT.kI(0.5);
  //PID_HEIGHT.imax(5);
  PID_HEIGHT.kD(1);

  PID_HEIGHT_ACCEL.kP(1);
  //PID_HEIGHT_ACCEL.kI(1);
  //PID_HEIGHT_ACCEL.imax(50);
  //PID_HEIGHT_ACCEL.kD(0.1);

//  PID_X.kP(0.2);
//  //PID_X.kI(0.1);
//  //PID_X.imax(1);
//  PID_X.kD(0.5);

  PID_X.kP(0.5);
  PID_X.kD(0.5);
//
//  PID_Y.kP(0.2);
//  //PID_Y.kI(0.1);
//  //PID_Y.imax(1);
//  PID_Y.kD(0.5);
  PID_Y.kP(0.5);
  PID_Y.kD(0.5);
  PID_X_ACCEL.kP(9);

  //PID_X_ACCEL.kD(08.1);

  PID_Y_ACCEL.kP(9);
  //PID_Y_ACCEL.kI(1);
  //PID_Y_ACCEL.imax(5);
  //PID_Y_ACCEL.kD(0.1);



}

// reset all PID integrators to zero
void reset_integrators()
{
  PID_ROLL_SPEED.reset_I();  PID_PITCH_SPEED.reset_I();  PID_YAW_SPEED.reset_I();
  PID_ROLL_ANGLE.reset_I();   PID_PITCH_ANGLE.reset_I();   PID_YAW_ANGLE.reset_I();
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


void  XY_PositionPID() {

  Vector3f accel = imu.get_accel();

//      //Y stands for latitude, X stands for longitude. Convert gobal frame to body frame
//      float PositionX_Convert = (float)PositionY * cos(ToDeg(heading)) -  (float)PositionX * sin(ToDeg(heading));
//      float PositionY_Convert = (float)PositionY * sin(ToDeg(heading)) +  (float)PositionX * cos(ToDeg(heading));
//
//
//      float x_pid_output = constrain(PID_X.get_pid((float)( PositionX -  PosX_lock), 1), -50, 50);
//      float y_pid_output = constrain(PID_Y.get_pid((float)( PosY_lock -  PositionY), 1), -50, 50);


  //    float x_pid_output = (PosX_lock - PositionX);

  float x_accel = accel.x;


  //hal.console->printf_P(PSTR("x is %f\n"),x_accel);

  float y_accel = -accel.y ;

  //  hal.console->printf_P(PSTR("y is %f\n"),y_accel);
  //  float y_accel_pid_output = constrain(PID_Y_ACCEL.get_pid((float)y_pid_output - x_accel, 1), -50, 50);
  // float x_accel_pid_output = constrain(PID_X_ACCEL.get_pid((float)x_pid_output - y_accel, 1), -50, 50);


  //  float x_accel_pid_output = constrain(PID_X_ACCEL.get_pid((float)(- x_accel), 1), -50, 50);
  //  float y_accel_pid_output = constrain(PID_Y_ACCEL.get_pid((float)(- y_accel), 1), -50, 50);



  float x_complement_pid_output = constrain(PID_X_ACCEL.get_pid((float)(- x_accel), 1), -50, 50);
  float y_complement_pid_output = constrain(PID_Y_ACCEL.get_pid((float)(- y_accel), 1), -50, 50);
//      float x_complement_pid_output =  y_pid_output +  x_accel_pid_output;
//      float y_complement_pid_output =  x_pid_output +  y_accel_pid_output;

//      float x_complement_pid_output =  x_accel_pid_output;
//      float y_complement_pid_output =  y_accel_pid_output;


//      float x_complement_pid_output = y_pid_output;
//      float y_complement_pid_output = x_pid_output;
  pitch = x_complement_pid_output;
  //  pitch = x_accel_pid_output;



  roll = y_complement_pid_output;
  //    roll = y_accel_pid_output;


}



//lock throttle and Z position
void  Z_PositionPID() {

  Vector3f accel = imu.get_accel();

  float height_pid_output = constrain(PID_HEIGHT.get_pid((float)height_lock - height_current, 1), -50, 50);
//         float height_pid_output = constrain(PID_HEIGHT.get_pid((float)baro_lock - baro_current, 1), -50, 50);
  /*
  float height_pid_output = constrain(PID_HEIGHT.get_pid((float)height_lock - z_ground, 1), -50, 50);
  if(delay_msg > 120)
    height_pid_output = 0;*/

  //acceleration PID control
  float z_accel = accel.z;
  float height_accel_pid_output = constrain(PID_HEIGHT_ACCEL.get_pid((float)(GRAVITY + height_pid_output) - z_accel, 1), -120, 120);

  //   throttle = barothrottle_lock + height_accel_pid_output;
  throttle = throttle_lock + height_accel_pid_output;



}


void Matlab_print() {

  if ((hal.scheduler->micros() - timer_serial) > 1000000L / 10) {

    timer_serial = hal.scheduler->micros();

    hal.uartB->print("PID ");
    hal.uartB->print(height_lock);
    hal.uartB->print(" ");
    hal.uartB->print(height_current);
    hal.uartB->print(" ");
    hal.uartB->println("END");





  }





}








void PID_Caliburation() {



  if (throttle > 1030)
  { //when motors is on do

    if (motorsOff == 1)
    {
      motorsOff = 0;
      yaw_current_location = imu_yaw;
    }

    // PID to adjust the angle on roll/yaw/pitch
    float roll_angle_pid_output = constrain(PID_ROLL_ANGLE.get_pid((float)roll - imu_roll, 1), -250, 250);
    float pitch_angle_pid_output = constrain(PID_PITCH_ANGLE.get_pid((float)pitch - imu_pitch, 1), -250, 250);
    //float yaw_angle_pid_output = constrain(PID_YAW_ANGLE.get_pid((float)yaw - imu_yaw, 1), -360, 360);    // angle mode of yaw control

    //     float yaw_angle_pid_output = constrain(PID_YAW_ANGLE.get_pid(yaw_degree_adjust(yaw_current_location - imu_yaw), 1), -360, 360);      // speed mode of yaw control
    //      if(yaw < -8 || yaw > 8)
    //      {
    float yaw_angle_pid_output = yaw;
    constrain(yaw_angle_pid_output, -360, 360);
    yaw_current_location = imu_yaw;
    //      }

    // PID to adjust the rotating speed on roll/yaw/pitch
    long pitch_pid_output =  (long) constrain(PID_PITCH_SPEED.get_pid(pitch_angle_pid_output - gyroPitch, 1), - 500, 500);
    long roll_pid_output =  (long) constrain(PID_ROLL_SPEED.get_pid(roll_angle_pid_output - gyroRoll, 1), -500, 500);
    long yaw_pid_output =  (long) constrain(PID_YAW_SPEED.get_pid(yaw_angle_pid_output - gyroYaw, 1), -500, 500);


    long throttle1 = throttle + pitch_pid_output + roll_pid_output - yaw_pid_output;
    long throttle2 = throttle - pitch_pid_output + roll_pid_output + yaw_pid_output;
    long throttle3 = throttle - pitch_pid_output - roll_pid_output - yaw_pid_output;
    long throttle4 = throttle + pitch_pid_output - roll_pid_output + yaw_pid_output;

    hal.rcout->write(MOTOR_FL, throttle1);
    hal.rcout->write(MOTOR_BL, throttle2);
    hal.rcout->write(MOTOR_BR, throttle3);
    hal.rcout->write(MOTOR_FR, throttle4);

    hal.uartC->printf_P(PSTR("%ld,%ld,%ld,%ld\n"), throttle1, throttle2, throttle3, throttle4);

    // hal.rcout->write(MOTOR_FL, 1000);
    // hal.rcout->write(MOTOR_BL, 1000);
    // hal.rcout->write(MOTOR_FR, 1000);
    // hal.rcout->write(MOTOR_BR, 1000);

//    hal.rcout->write(MOTOR_FL, throttle + roll_pid_output + pitch_pid_output );
//    hal.rcout->write(MOTOR_BL, throttle + roll_pid_output - pitch_pid_output );
//    hal.rcout->write(MOTOR_FR, throttle - roll_pid_output + pitch_pid_output );
//    hal.rcout->write(MOTOR_BR, throttle - roll_pid_output - pitch_pid_output );
//////
    static int count = 0;
    count++;
    if (count == 20)
    {
////     //     hal.console->printf_P(PSTR("IMU YAW %4.1f PIT %4.1f ROLL %4.1f\r\n"), imu_yaw, imu_pitch, imu_roll);
      hal.console->printf_P(PSTR("remote YAW %ld PIT %ld ROLL %ld TRO %ld\r\n"), yaw, pitch, roll, throttle);






//////
//////          hal.console->printf_P(PSTR("gyro YAW %4.3f PIT %4.3f ROLL %4.3f PIT ERROR %4.3f ROLL ERROR %4.3f\r\n"), gyroYaw, gyroPitch, gyroRoll, pitch_angle_pid_output - gyroPitch, roll_angle_pid_output - gyroRoll);
////
//////          hal.console->printf("\n");
//////         hal.console->print(sonar->read());
////////          hal.console->printf("\n");
////////
//////          //hal.console->printf_P(PSTR("i_values YAW %4.3f PIT %4.3f ROLL %4.3f\r\n"), PID_YAW_SPEED.get_integrator(), PID_PITCH_SPEED.get_integrator(), PID_ROLL_SPEED.get_integrator());
      count = 0;
    }
//

  } else
  { //when motors is off do

    //make sure the motors are off
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_FR, 1000);
    hal.rcout->write(MOTOR_BR, 1000);
    hal.uartC->printf_P(PSTR("1000,1000,1000,1000\n"));

    motorsOff = 1;
    AutoZcontrol = 0;
    AutoXYcontrol = 0;
//       runonce_gps= 0;

    PosX_lock = 0;
    PosY_lock = 0;
    // Reset_opt();

    // reset all PID integrals parameters while not flying
    reset_integrators();

  }
}








