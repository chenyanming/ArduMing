/**
 * Revise file Servo.h to choose Timer3 first
 * Timer3: Servo.h
 * Digital Pin 12: motor 1
 * Digital Pin 11: motor 2
 * Digital Pin 8: motor 3
 * Digital Pin 7: motor 4
 */
#include <Servo.h>
#include "PID_v1.h"

Servo motor1, motor2, motor3, motor4;

extern float roll, pitch, throttle, yaw;
extern boolean euler_output;
extern boolean throttle_output;

extern float rpy_rol;
extern float rpy_pit;
extern float rpy_yaw;

extern int GyroX;
extern int GyroY;
extern int GyroZ;

int motor_adjust_count = 100;
boolean motor_adjust_bit = false;

float roll_angle_pid_output = 0;
float pitch_angle_pid_output = 0;
float yaw_angle_pid_output = 0;
float roll_pid_output = 0;
float pitch_pid_output = 0;
float yaw_pid_output = 0;
// Pid roll_angle, pitch_angle, yaw_angle, roll_gyro, pitch_gyro, yaw_gyro;

float pitch_kp = 0.05, pitch_ki = 0.3, pitch_kd = 0.0;
PID pitch_angle(&rpy_pit, &pitch_angle_pid_output, &pitch, pitch_kp, pitch_ki, pitch_kd, DIRECT);

float rpy_pit_adjust = 0;
float rpy_rol_adjust = 0;
float rpy_yaw_adjust = 0;

long throttle1 = 0;
long throttle2 = 0;
long throttle3 = 0;
long throttle4 = 0;

void motor_setup() {
	motor1.attach(12);
	motor2.attach(11);
	motor3.attach(8);
	motor4.attach(7);

	motor1.writeMicroseconds(MIN_SIGNAL);
	motor2.writeMicroseconds(MIN_SIGNAL);
	motor3.writeMicroseconds(MIN_SIGNAL);
	motor4.writeMicroseconds(MIN_SIGNAL);

	// pitch_angle.Init(1.5, 0, 0);
	// roll_angle.Init(0.01, 0, 0);
	// yaw_angle.Init(0.01, 0, 0);
	// pitch_gyro.Init(0.01, 1, 0.02);
	// roll_gyro.Init(0.01, 0, 0);
	// yaw_gyro.Init(0.01, 0, 0);
	//turn the PID on
	pitch_angle.SetMode(AUTOMATIC);
	pitch_angle.SetSampleTime(10);
	pitch_angle.SetOutputLimits(-20, 20);
}

void motor_adjust() {
	static float last_pitch;
	if (motor_adjust_count > 0) {
		motor_adjust_count--;
		rpy_pit_adjust = -rpy_pit;
		rpy_rol_adjust = -rpy_rol;
		rpy_yaw_adjust = -rpy_yaw;
		if (motor_adjust_count == 0) {
			// Serial.println("Drone adjust value. Get.");
			// rpy_pit = rpy_pit + rpy_pit_adjust;
			// rpy_rol = rpy_rol + rpy_rol_adjust;
			// rpy_yaw = rpy_yaw + rpy_yaw_adjust;
			motor_adjust_bit = true;
			digitalWrite(blueled, LOW);
			digitalWrite(yellowled, LOW);
			digitalWrite(redled, LOW);
		}
	}
	else {
		if (last_pitch != rpy_pit)
		{
			rpy_pit = rpy_pit + rpy_pit_adjust;
			rpy_rol = rpy_rol + rpy_rol_adjust;
			rpy_yaw = rpy_yaw + rpy_yaw_adjust;
		}
		last_pitch = rpy_pit;
	}
}

void motor_output() {
	if (motor_adjust_bit == true) {
#ifdef CALI_THRO
		motor1.writeMicroseconds(throttle);
		motor2.writeMicroseconds(throttle);
		motor3.writeMicroseconds(throttle);
		motor4.writeMicroseconds(throttle);
#else

		if (throttle > 1200) {

			// pitch_angle_pid_output = pitch_angle.Update(pitch - rpy_pit);
			// roll_angle_pid_output = roll_angle.Update(roll - rpy_rol);
			// yaw_angle_pid_output = yaw_angle.Update(yaw - rpy_yaw);

			// pitch_angle_pid_output = constrain(pitch_angle_pid_output, -10, 10);
			// roll_angle_pid_output = constrain(roll_angle_pid_output, -20, 20);
			// yaw_angle_pid_output = constrain(yaw_angle_pid_output, -20, 20);    // angle mode of yaw control

			// pitch_pid_output = pitch_gyro.Update(pitch - GyroX);
			// roll_pid_output = roll_gyro.Update(roll - GyroY);
			// yaw_pid_output = yaw_gyro.Update(yaw - GyroZ);

			// pitch_pid_output = constrain(pitch_pid_output, -30, 30);
			// roll_pid_output = constrain(roll_pid_output, -30, 30);
			// yaw_pid_output = constrain(yaw_pid_output, -30, 30);    // angle mode of yaw control

			// throttle1 = throttle + roll_pid_output + pitch_pid_output - yaw_pid_output;
			// throttle2 = throttle + roll_pid_output - pitch_pid_output + yaw_pid_output;
			// throttle3 = throttle - roll_pid_output - pitch_pid_output - yaw_pid_output;
			// throttle4 = throttle - roll_pid_output + pitch_pid_output + yaw_pid_output;

			// throttle1 = throttle - pitch_pid_output - yaw_pid_output;
			// throttle2 = throttle - roll_pid_output + yaw_pid_output;
			// throttle3 = throttle + pitch_pid_output - yaw_pid_output;
			// throttle4 = throttle + roll_pid_output + yaw_pid_output;

			// throttle1 = throttle - pitch_angle_pid_output - yaw_angle_pid_output;
			// throttle2 = throttle - roll_angle_pid_output + yaw_angle_pid_output;
			// throttle3 = throttle + pitch_angle_pid_output - yaw_angle_pid_output;
			// throttle4 = throttle + roll_angle_pid_output + yaw_angle_pid_output;


			pitch_angle.Compute();

			throttle1 = throttle - pitch_angle_pid_output;
			throttle2 = throttle;
			throttle3 = throttle + pitch_angle_pid_output;
			throttle4 = throttle;

			throttle1 = constrain(throttle1, 1, MAX_SINGLE);//start from non-zero to finish the calibration
			throttle2 = constrain(throttle2, 1, MAX_SINGLE);//start from non-zero to finish the calibration
			throttle3 = constrain(throttle3, 1, MAX_SINGLE);//start from non-zero to finish the calibration
			throttle4 = constrain(throttle4, 1, MAX_SINGLE);//start from non-zero to finish the calibration

			motor1.writeMicroseconds(throttle1);
			motor2.writeMicroseconds(throttle2);
			motor3.writeMicroseconds(throttle3);
			motor4.writeMicroseconds(throttle4);

		}
		else {

			//It has to output throttle, even if throttle < 20, otherwise, the motor will "bibibibibibibibibi"
			motor1.writeMicroseconds(MIN_SINGLE);
			motor2.writeMicroseconds(MIN_SINGLE);
			motor3.writeMicroseconds(MIN_SINGLE);
			motor4.writeMicroseconds(MIN_SINGLE);

		}
	}
	else {
		motor1.writeMicroseconds(MIN_SINGLE);
		motor2.writeMicroseconds(MIN_SINGLE);
		motor3.writeMicroseconds(MIN_SINGLE);
		motor4.writeMicroseconds(MIN_SINGLE);

	}
#endif



	}
}