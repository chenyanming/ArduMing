/**
 * Revise file Servo.h to choose Timer3 first
 * Timer3: Servo.h 
 * Digital Pin 12: motor 1
 * Digital Pin 11: motor 2
 * Digital Pin 8: motor 3
 * Digital Pin 7: motor 4
 */
#include <Servo.h>
#include "pid.h"

Servo motor1, motor2, motor3, motor4;

extern long roll, pitch, throttle, yaw;
extern boolean euler_output;
extern boolean throttle_output;

extern float rpy_rol;
extern float rpy_pit;
extern float rpy_yaw;

extern int GyroX;
extern int GyroY;
extern int GyroZ;

int motor_adjust_count = 1500;
boolean motor_adjust_bit = false;

float roll_angle_pid_output = 0;
float pitch_angle_pid_output = 0;
float yaw_angle_pid_output = 0;
float roll_pid_output = 0;
float pitch_pid_output = 0;
float yaw_pid_output = 0;
Pid roll_angle, pitch_angle, yaw_angle, roll_gyro, pitch_gyro, yaw_gyro;

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

	motor1.write(10);
	motor2.write(10);
	motor3.write(10);
	motor4.write(10);

	pitch_angle.Init(0.01, 0, 0);
	roll_angle.Init(0.01, 0, 0);
	yaw_angle.Init(0.01, 0, 0);
	pitch_gyro.Init(0.01, 1, 0.02);
	roll_gyro.Init(0.01, 0, 0);
	yaw_gyro.Init(0.01, 0, 0);
}

int motor_adjust() {
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
			return 1;
		}
	}
	return 0;
}

void motor_output() {
	if (motor_adjust_bit == true) {
#ifdef CALI_THRO
		motor1.write(throttle);
		motor2.write(throttle);
		motor3.write(throttle);
		motor4.write(throttle);
#else

		if (throttle > 20) {

			rpy_pit = rpy_pit + rpy_pit_adjust;
			rpy_rol = rpy_rol + rpy_rol_adjust;
			rpy_yaw = rpy_yaw + rpy_yaw_adjust;

			pitch_angle_pid_output = pitch_angle.Update(pitch - rpy_pit);
			roll_angle_pid_output = roll_angle.Update(roll - rpy_rol);
			yaw_angle_pid_output = yaw_angle.Update(yaw - rpy_yaw);

			pitch_angle_pid_output = constrain(pitch_angle_pid_output, -20, 20);
			roll_angle_pid_output = constrain(roll_angle_pid_output, -20, 20);
			yaw_angle_pid_output = constrain(yaw_angle_pid_output, -20, 20);    // angle mode of yaw control

			pitch_pid_output = pitch_gyro.Update(pitch - GyroX);
			roll_pid_output = roll_gyro.Update(roll - GyroY);
			yaw_pid_output = yaw_gyro.Update(yaw - GyroZ);

			pitch_pid_output = constrain(pitch_pid_output, -30, 30);
			roll_pid_output = constrain(roll_pid_output, -30, 30);
			yaw_pid_output = constrain(yaw_pid_output, -30, 30);    // angle mode of yaw control

			// throttle1 = throttle + roll_pid_output + pitch_pid_output - yaw_pid_output;
			// throttle2 = throttle + roll_pid_output - pitch_pid_output + yaw_pid_output;
			// throttle3 = throttle - roll_pid_output - pitch_pid_output - yaw_pid_output;
			// throttle4 = throttle - roll_pid_output + pitch_pid_output + yaw_pid_output;

			throttle1 = throttle - pitch_pid_output - yaw_pid_output;
			throttle2 = throttle - roll_pid_output + yaw_pid_output;
			throttle3 = throttle + pitch_pid_output - yaw_pid_output;
			throttle4 = throttle + roll_pid_output + yaw_pid_output;

			// throttle1 = throttle - pitch_angle_pid_output - yaw_angle_pid_output;
			// throttle2 = throttle - roll_angle_pid_output + yaw_angle_pid_output;
			// throttle3 = throttle + pitch_angle_pid_output - yaw_angle_pid_output;
			// throttle4 = throttle + roll_angle_pid_output + yaw_angle_pid_output;

			throttle1 = constrain(throttle1, 1, 90);//start from non-zero to finish the calibration
			throttle2 = constrain(throttle2, 1, 90);//start from non-zero to finish the calibration
			throttle3 = constrain(throttle3, 1, 90);//start from non-zero to finish the calibration
			throttle4 = constrain(throttle4, 1, 90);//start from non-zero to finish the calibration

			motor1.write(throttle1);
			motor2.write(throttle2);
			motor3.write(throttle3);
			motor4.write(throttle4);

#ifdef THROTTLE_OUTPUT
			if (throttle_output == true) {
				throttle_output = false;
				Serial.print("******The motor1, motor2, motor3, motor4 throttle value : "); Serial.print("\t");
				Serial.print(throttle1); Serial.print("\t");
				Serial.print(throttle2); Serial.print("\t");
				Serial.print(throttle3); Serial.print("\t");
				Serial.print(throttle4); Serial.println();
				Serial.print("******pitch_pid_output, roll_pid_output, yaw_pid_output: ");
				Serial.print(pitch_pid_output); Serial.print("\t");
				Serial.print(roll_pid_output); Serial.print("\t");
				Serial.print(yaw_pid_output); Serial.println();
			}
#endif

#ifdef EULER_OUTPUT
			if (euler_output == true) {
				euler_output = false;
				Serial.print("******The Pitch, Roll, Yaw : ");
				Serial.print(rpy_pit); Serial.print("\t");
				Serial.print(rpy_rol); Serial.print("\t");
				Serial.print(rpy_yaw); Serial.print("\t");
				Serial.print("(");
				Serial.print(rpy_pit_adjust); Serial.print("\t");
				Serial.print(rpy_rol_adjust); Serial.print("\t");
				Serial.print(rpy_yaw_adjust);
				Serial.print(")"); Serial.println();
			}
#endif

		}
		else {
			rpy_pit = rpy_pit + rpy_pit_adjust;
			rpy_rol = rpy_rol + rpy_rol_adjust;
			rpy_yaw = rpy_yaw + rpy_yaw_adjust;

			//It has to output throttle, even if throttle < 20, otherwise, the motor will "bibibibibibibibibi"
			motor1.write(10);
			motor2.write(10);
			motor3.write(10);
			motor4.write(10);

#ifdef THROTTLE_OUTPUT
			if (throttle_output == true) {
				throttle_output = false;
				Serial.print("++++++The motor1, motor2, motor3, motor4 throttle value : "); Serial.print("\t");
				Serial.print(10); Serial.print("\t");
				Serial.print(10); Serial.print("\t");
				Serial.print(10); Serial.print("\t");
				Serial.print(10); Serial.println();
			}
#endif

#ifdef EULER_OUTPUT
			if (euler_output == true) {
				euler_output = false;
				Serial.print("++++++The Pitch, Roll, Yaw : ");
				Serial.print(rpy_pit); Serial.print("\t");
				Serial.print(rpy_rol); Serial.print("\t");
				Serial.print(rpy_yaw); Serial.print("\t");
				Serial.print("(");
				Serial.print(rpy_pit_adjust); Serial.print("\t");
				Serial.print(rpy_rol_adjust); Serial.print("\t");
				Serial.print(rpy_yaw_adjust);
				Serial.print(")"); Serial.println();
			}
#endif
		}
	}
	else {
		motor1.write(10);
		motor2.write(10);
		motor3.write(10);
		motor4.write(10);

#ifdef THROTTLE_OUTPUT
		if (throttle_output == true) {
			throttle_output = false;
			Serial.print("......Getting motor1, motor2, motor3, motor4 throttle value : "); Serial.print("\t");
			Serial.print(10); Serial.print("\t");
			Serial.print(10); Serial.print("\t");
			Serial.print(10); Serial.print("\t");
			Serial.print(10); Serial.println();
		}
#endif
	}
#endif



	}