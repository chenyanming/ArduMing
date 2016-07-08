

float imu_yaw_adjust, imu_roll_adjust, imu_pitch_adjust;
float imu_roll, imu_pitch, imu_yaw; 
 float gyroYaw,gyroPitch,gyroRoll;
 float gyro_pitch_adjust = -2.45, gyro_roll_adjust= 2.45;  //gyro_yaw_adjust, 

void  IMU_Init(){


 //hal.console->printf_P(PSTR("Version 2.6 Kp 0.1 Kd 0.07 + no imu fix\r\n"));
  /*enable IMU(accelerometer and gyroscopes) with Digital Motion Processing Unit*/
   hal.gpio->pinMode(27, GPIO_OUTPUT);
  hal.gpio->write(27, 0);
  
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, 1);
  imu.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ, NULL);
  hal.scheduler->suspend_timer_procs(); 
  imu.dmp_init();                        
  imu.push_gyro_offsets_to_dmp();
  hal.scheduler->resume_timer_procs();  
  
  
 // hal.console->println("\nWait 15 seconds to adjust IMU");
  hal.scheduler->delay(15000);
  while (imu.num_samples_available() == 0);  //wait untill received new IMU data
  imu.update();  
  float imu_roll, imu_pitch, imu_yaw;  
  imu.quaternion.to_euler(&imu_roll, &imu_pitch, &imu_yaw);
  imu_yaw_adjust  = -(ToDeg(imu_yaw));
  imu_pitch_adjust = -(ToDeg(imu_pitch));
  imu_roll_adjust = -(ToDeg(imu_roll));
   hal.gpio->write(27, 1);
  //hal.console->printf("IMU adjustment done, adjust factors: yaw %f pitch %f roll %f \n", imu_yaw_adjust, imu_pitch_adjust, imu_roll_adjust);


}


void IMU_Update(){

      /*get new IMU data and transform it to degree*/
  while (imu.num_samples_available() == 0);  //wait untill received new IMU data
  imu.update();  
//  float imu_roll, imu_pitch, imu_yaw;  
  imu.quaternion.to_euler(&imu_roll, &imu_pitch, &imu_yaw);
  imu_yaw = ToDeg(imu_yaw);
  imu_pitch = ToDeg(imu_pitch) + imu_pitch_adjust;
  imu_roll = ToDeg(imu_roll) + imu_roll_adjust;
  
  //get gyroscopes data
  Vector3f gyroData = imu.get_gyro();
  gyroYaw = ToDeg(gyroData.z), gyroPitch = ToDeg(gyroData.y) + gyro_pitch_adjust, gyroRoll = ToDeg(gyroData.x) + gyro_roll_adjust;
  
  




}
