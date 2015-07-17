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

int tmp_count = 0;

Servo motor1, motor2, motor3, motor4;

extern float roll, pitch, throttle, yaw;
extern float max_yaw;
extern boolean euler_output;
extern boolean throttle_output;

extern float kal_rol;
extern float kal_pit;
extern float kal_yaw;
float first_kal_yaw, relative_yaw;

extern float GyroX;
extern float GyroY;
extern float GyroZ;

extern const int MAX_SIGNAL;
extern const int MIN_SIGNAL;

int motor_adjust_count = 2800;
boolean motor_adjust_bit = false;

float pitch_angle_pid_output = 0;
float roll_angle_pid_output = 0;
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
PID roll_angle(&kal_rol, &roll_angle_pid_output, &roll, 4, 0.05, 0, DIRECT);
PID yaw_angle(&kal_yaw, &yaw_angle_pid_output, &relative_yaw, 2, 0.05, 0, DIRECT);

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

	/**
	 * turn the PID on
	 */
	pitch_angle.SetMode(AUTOMATIC);
	pitch_angle.SetSampleTime(10);
	pitch_angle.SetOutputLimits(-450, 450);

	roll_angle.SetMode(AUTOMATIC);
	roll_angle.SetSampleTime(10);
	roll_angle.SetOutputLimits(-450, 450);

	yaw_angle.SetMode(AUTOMATIC);
	yaw_angle.SetSampleTime(10);
	yaw_angle.SetOutputLimits(-450, 450);

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
			first_kal_yaw = kal_yaw; // not move, no yaw
			relative_yaw = first_kal_yaw;
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
	if (motor_adjust_bit == true) {
#ifdef CALI_THRO
		motor1.writeMicroseconds(throttle);
		motor2.writeMicroseconds(throttle);
		motor3.writeMicroseconds(throttle);
		motor4.writeMicroseconds(throttle);
#else

		if (throttle > 1050) {
			// Serial.println(pitch);
			// tmp = pitch - kal_pit;
			// pitch_angle_pid_output = pitch_angle.Update(tmp);
			pitch_angle.Compute();
			roll_angle.Compute();

			/**
			 * We don't need to use the yaw speed.
			 */
			// if ((-20 < GyroZ) && (GyroZ < 20)) {
			// 	first_kal_yaw = kal_yaw;
			// 	relative_yaw = first_kal_yaw;
			// 	max_yaw = 0;
			// }

			relative_yaw = first_kal_yaw + max_yaw;


			yaw_angle.Compute();
			// tmp = pitch_angle_pid_output + 0.9*GyroX; //2, 0.04, 0.9
			// tmp = pitch_angle_pid_output + 0.8*GyroX; //1.6, 0.04, 0.8
			float tmp = pitch_angle_pid_output + 2 * GyroX; //4, 0.04, 2
			// tmp = pitch_angle_pid_output + 2.7*GyroX;
			// tmp = pitch_angle_pid_output;
			// pitch_gyrox.Compute();
			float tmp1 = roll_angle_pid_output + 2.1 * GyroY; //4, 0.04, 2
			float tmp2 = yaw_angle_pid_output + 1 * GyroZ; //4, 0.04, 2

			throttle1 = throttle - tmp - tmp2;
			throttle2 = throttle - tmp1 + tmp2;
			throttle3 = throttle + tmp - tmp2;
			throttle4 = throttle + tmp1 + tmp2;

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
			/*
			** When the propellers does not spin, transfering the take-off yaw (first_kal_yaw) to the setpoint (relative_yaw), so that it can perfrom PID caluculation.
			*/
			first_kal_yaw = kal_yaw; // not move, no yaw rotate
			relative_yaw = first_kal_yaw;
			/*
			** It has to output throttle, even if throttle < 1050, otherwise, the motor will "bibibibibibibibibi"
			*/
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
