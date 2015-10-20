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

extern float roll, pitch, throttle, yaw, ch6;
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

float height_baro_pid_output = 0;
float lock_average_altitude;
// Pid roll_angle, pitch_angle, yaw_angle, roll_gyro, pitch_gyro, yaw_gyro;

float height_sonar_pid_output = 0;
float height_sonar_2_pid_output = 0;
float lock_sonar_altitude;

// PID
float pitch_p = 2;
float pitch_i = 0.18;
float pitch_d = 1.5;
float roll_p = 2;
float roll_i = 0.18;
float roll_d = 1.5;
float yaw_p = 3.5;
float yaw_i = 0;
float yaw_d = 1.30;

PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, pitch_p, pitch_i, 0, DIRECT);
PID roll_angle(&kal_rol, &roll_angle_pid_output, &roll, roll_p, roll_i, 0, DIRECT);
PID yaw_angle(&kal_yaw, &yaw_angle_pid_output, &relative_yaw, yaw_p, yaw_i, 0, DIRECT);
PID height_baro(&average_altitude, &height_baro_pid_output, &lock_average_altitude, 0, 0, 0, DIRECT);
// PID height_baro(&average_altitude, &height_baro_pid_output, &throttle, 0, 0, 0, DIRECT);
PID height_sonar(&kal_sonar, &height_sonar_pid_output, &lock_sonar_altitude, 0, 0, 0, DIRECT);
PID height_sonar_2(&Az, &height_sonar_2_pid_output, &height_sonar_pid_output, 0, 0, 0, DIRECT);

float kal_pit_adjust = 0;
float kal_rol_adjust = 0;
float kal_yaw_adjust = 0;

int throttle1 = 0;
int throttle2 = 0;
int throttle3 = 0;
int throttle4 = 0;

float lock_throttle;

extern volatile unsigned int alt_hold_count;

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

	height_baro.SetMode(AUTOMATIC);
	height_baro.SetSampleTime(300);
	height_baro.SetOutputLimits(-450, 450);

	height_sonar.SetMode(AUTOMATIC);
	height_sonar.SetSampleTime(10);
	height_sonar.SetOutputLimits(-450, 450);

	height_sonar_2.SetMode(AUTOMATIC);
	height_sonar_2.SetSampleTime(10);
	height_sonar_2.SetOutputLimits(-450, 450);
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

void motor_slave_output(){
		motor1.writeMicroseconds(FL);
		motor2.writeMicroseconds(BL);
		motor3.writeMicroseconds(BR);
		motor4.writeMicroseconds(FR);
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

			relative_yaw = first_kal_yaw + max_yaw;


			yaw_angle.Compute();

			/**
			 * Plus (+) configuration
			 */
			// float tmp = pitch_angle_pid_output + 2 * GyroX; //4, 0.04, 2
			// float tmp1 = roll_angle_pid_output + 2.1 * GyroY; //4, 0.04, 2
			// float tmp2 = yaw_angle_pid_output + 1 * GyroZ; //4, 0.04, 2

			// throttle1 = throttle - tmp - tmp2;
			// throttle2 = throttle - tmp1 + tmp2;
			// throttle3 = throttle + tmp - tmp2;
			// throttle4 = throttle + tmp1 + tmp2;

			/**
			 * X configuration
			 */
			// float tmp = pitch_angle_pid_output + 0.90 * GyroX; //4, 0.04, 2
			float tmp = pitch_angle_pid_output + pitch_d * GyroX; //4, 0.04, 2
			float tmp1 = roll_angle_pid_output + roll_d * GyroY; //4, 0.04, 2
			// float tmp2 = yaw_angle_pid_output + last_ch6_d * GyroZ; //4, 0.04, 2
			float tmp2 = yaw_angle_pid_output + yaw_d * GyroZ; //4, 0.04, 2

			// if (on_ch5 == true) {
				// throttle = map(throttle, 1050, 1900, 1, 15);
				// height_baro.Compute();
				// float tmp3 = height_baro_pid_output;// + 0.90 * Az;
				// throttle1 = throttle - tmp - tmp1 - tmp2 + tmp3;
				// throttle2 = throttle + tmp - tmp1 + tmp2 + tmp3;
				// throttle3 = throttle + tmp + tmp1 - tmp2 + tmp3;
				// throttle4 = throttle - tmp + tmp1 + tmp2 + tmp3;

				// if (kal_sonar > 20) {
				// lock_sonar_altitude = kal_sonar - 1;
				// height_sonar.Compute();
				// height_sonar_2.Compute();
				// float tmp3 = height_sonar_2_pid_output;
				// throttle1 = throttle - tmp - tmp1 - tmp2 + tmp3;
				// throttle2 = throttle + tmp - tmp1 + tmp2 + tmp3;
				// throttle3 = throttle + tmp + tmp1 - tmp2 + tmp3;
				// throttle4 = throttle - tmp + tmp1 + tmp2 + tmp3;
				// }
			// }
			// else {

				throttle1 = throttle - tmp - tmp1 - tmp2;
				throttle2 = throttle + tmp - tmp1 + tmp2;
				throttle3 = throttle + tmp + tmp1 - tmp2;
				throttle4 = throttle - tmp + tmp1 + tmp2;

				// lock_average_altitude = average_altitude;
				// lock_throttle = throttle;
				// lock_sonar_altitude = kal_sonar;
			// }

			throttle1 = constrain(throttle1, 1050, MAX_SIGNAL);
			throttle2 = constrain(throttle2, 1050, MAX_SIGNAL);
			throttle3 = constrain(throttle3, 1050, MAX_SIGNAL);
			throttle4 = constrain(throttle4, 1050, MAX_SIGNAL);



			motor1.writeMicroseconds(throttle1);
			motor2.writeMicroseconds(throttle2);
			motor3.writeMicroseconds(throttle3);
			motor4.writeMicroseconds(throttle4);

		}
		else {
			throttle1 = MIN_SIGNAL;
			throttle2 = MIN_SIGNAL;
			throttle3 = MIN_SIGNAL;
			throttle4 = MIN_SIGNAL;
			/*
			** When the propellers does not spin, transfering the take-off yaw (first_kal_yaw) to the setpoint (relative_yaw), so that it can perfrom PID caluculation.
			*/
			first_kal_yaw = kal_yaw; // not move, no yaw rotate
			relative_yaw = first_kal_yaw;
			/*
			** It has to output throttle, even if throttle < 1050, otherwise, the motor will "bibibibibibibibibi"
			*/
			motor1.writeMicroseconds(throttle1);
			motor2.writeMicroseconds(throttle2);
			motor3.writeMicroseconds(throttle3);
			motor4.writeMicroseconds(throttle4);

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
