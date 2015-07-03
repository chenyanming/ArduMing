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
// #include "pid.h"

Servo motor1, motor2, motor3, motor4;

extern float roll, pitch, throttle, yaw;
extern boolean euler_output;
extern boolean throttle_output;

extern float kal_rol;
extern float kal_pit;
extern float kal_yaw;

extern float GyroX;
extern float GyroY;
extern float GyroZ;

extern const int MAX_SIGNAL;
extern const int MIN_SIGNAL;

int motor_adjust_count = 2800;
boolean motor_adjust_bit = false;

float roll_angle_pid_output = 0;
float pitch_angle_pid_output = 0;
float yaw_angle_pid_output = 0;
float roll_pid_output = 0;
float pitch_pid_output = 0;
float yaw_pid_output = 0;
// Pid roll_angle, pitch_angle, yaw_angle, roll_gyro, pitch_gyro, yaw_gyro;

// PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, 0.65, 0, 0, DIRECT);
// PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, 0.8, 0, 0, DIRECT);
// PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, 1.6, 0.04, 0, DIRECT);
// PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, 2, 0.04, 0, DIRECT);
PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, 4, 0.04, 0, DIRECT);
// PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, 5.9, 0.05, 0, DIRECT);
// PID pitch_gyrox(&GyroX, &pitch_pid_output, &pitch_angle_pid_output, 0, 0.0, 0.0, DIRECT);

float kal_pit_adjust = 0;
float kal_rol_adjust = 0;
float kal_yaw_adjust = 0;

int throttle1 = 0;
int throttle2 = 0;
int throttle3 = 0;
int throttle4 = 0;


void motor_setup() {
	motor1.attach(12);
	motor2.attach(11);
	motor3.attach(8);
	motor4.attach(7);

	motor1.writeMicroseconds(MIN_SIGNAL);
	motor2.writeMicroseconds(MIN_SIGNAL);
	motor3.writeMicroseconds(MIN_SIGNAL);
	motor4.writeMicroseconds(MIN_SIGNAL);

	// pitch_angle.Init(40, 0, 0);
	// roll_angle.Init(0.01, 0, 0);
	// yaw_angle.Init(0.01, 0, 0);
	// pitch_gyro.Init(0.01, 1, 0.02);
	// roll_gyro.Init(0.01, 0, 0);
	// yaw_gyro.Init(0.01, 0, 0);
	//turn the PID on
	pitch_angle.SetMode(AUTOMATIC);
	pitch_angle.SetSampleTime(10);
	pitch_angle.SetOutputLimits(-450, 450);
	// pitch_gyrox.SetMode(AUTOMATIC);
	// pitch_gyrox.SetSampleTime(10);
	// pitch_gyrox.SetOutputLimits(-500, 500);
}

void motor_adjust() {
	static float last_pitch;
	if (motor_adjust_count > 0) {
		motor_adjust_count--;
		kal_pit_adjust = -kal_pit;
		kal_rol_adjust = -kal_rol;
		kal_yaw_adjust = -kal_yaw;
		if (motor_adjust_count == 0) {
			// Serial.println("Drone adjust value. Get.");
			// kal_pit = kal_pit + kal_pit_adjust;
			// kal_rol = kal_rol + kal_rol_adjust;
			// kal_yaw = kal_yaw + kal_yaw_adjust;
			motor_adjust_bit = true;
			digitalWrite(blueled, LOW);
			digitalWrite(yellowled, LOW);
			digitalWrite(redled, LOW);
		}
	}
	else {
		if (last_pitch != kal_pit)
		{
			// kal_pit = kal_pit + 0.54;
			// kal_rol = kal_rol - 1.29;
			// kal_pit = kal_pit + kal_pit_adjust;
			// kal_rol = kal_rol - kal_rol_adjust;
			// kal_yaw = kal_yaw + kal_yaw_adjust;
		}
		last_pitch = kal_pit;
	}
}

void motor_output() {
	float tmp;
	if (motor_adjust_bit == true) {
#ifdef CALI_THRO
		motor1.writeMicroseconds(throttle);
		motor2.writeMicroseconds(throttle);
		motor3.writeMicroseconds(throttle);
		motor4.writeMicroseconds(throttle);
#else

		if (throttle > 1050) {

			// pitch_angle_pid_output = pitch_angle.Update(pitch - kal_pit);
			// roll_angle_pid_output = roll_angle.Update(roll - kal_rol);
			// yaw_angle_pid_output = yaw_angle.Update(yaw - kal_yaw);

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


			// Serial.println(pitch);
			// tmp = pitch - kal_pit;
			// pitch_angle_pid_output = pitch_angle.Update(tmp);
			pitch_angle.Compute();
			// tmp = pitch_angle_pid_output + 0.9*GyroX; //2, 0.04, 0.9
			// tmp = pitch_angle_pid_output + 0.8*GyroX; //1.6, 0.04, 0.8
			tmp = pitch_angle_pid_output + 2*GyroX;//4, 0.04, 2
			// tmp = pitch_angle_pid_output + 2.7*GyroX;
			// tmp = pitch_angle_pid_output;
			// pitch_gyrox.Compute();

			throttle1 = throttle - tmp;
			throttle2 = throttle;
			throttle3 = throttle + tmp;
			throttle4 = throttle;

			throttle1 = constrain(throttle1, 1, MAX_SIGNAL);//start from non-zero to finish the calibration
			throttle2 = constrain(throttle2, 1, MAX_SIGNAL);//start from non-zero to finish the calibration
			throttle3 = constrain(throttle3, 1, MAX_SIGNAL);//start from non-zero to finish the calibration
			throttle4 = constrain(throttle4, 1, MAX_SIGNAL);//start from non-zero to finish the calibration

			motor1.writeMicroseconds(throttle1);
			motor2.writeMicroseconds(throttle2);
			motor3.writeMicroseconds(throttle3);
			motor4.writeMicroseconds(throttle4);

		}
		else {

			//It has to output throttle, even if throttle < 20, otherwise, the motor will "bibibibibibibibibi"
			motor1.writeMicroseconds(MIN_SIGNAL);
			motor2.writeMicroseconds(MIN_SIGNAL);
			motor3.writeMicroseconds(MIN_SIGNAL);
			motor4.writeMicroseconds(MIN_SIGNAL);

		}
	}
	else {
		motor1.writeMicroseconds(MIN_SIGNAL);
		motor2.writeMicroseconds(MIN_SIGNAL);
		motor3.writeMicroseconds(MIN_SIGNAL);
		motor4.writeMicroseconds(MIN_SIGNAL);

	}
#endif



}