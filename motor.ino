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

int motor_adjust_count = 1500;//2800, 28s
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
float lock_sonar_altitude;

// PID
float pitch_p = 2;
// float pitch_i = 0.18;
float pitch_i = 0;
float pitch_d = 1.5;

float roll_p = 2;
// float roll_i = 0.18;
float roll_i = 0;
float roll_d = 1;

float yaw_p = 3;
float yaw_i = 0;
float yaw_d = 1.05;

float height_sonar_p = 0.25;
float height_sonar_i = 0;
float height_sonar_d = 0.33;

PID pitch_angle(&kal_pit, &pitch_angle_pid_output, &pitch, pitch_p, pitch_i, 0, DIRECT);
PID roll_angle(&kal_rol, &roll_angle_pid_output, &roll, roll_p, roll_i, 0, DIRECT);
PID yaw_angle(&kal_yaw, &yaw_angle_pid_output, &relative_yaw, yaw_p, yaw_i, 0, DIRECT);
PID height_baro(&average_altitude, &height_baro_pid_output, &lock_average_altitude, 0, 0, 0, DIRECT);
// PID height_baro(&average_altitude, &height_baro_pid_output, &throttle, 0, 0, 0, DIRECT);
// PID height_sonar(&kal_sonar, &height_sonar_pid_output, &lock_sonar_altitude, height_sonar_p, height_sonar_i, 0, DIRECT);

float kal_pit_adjust = 0;
float kal_rol_adjust = 0;
float kal_yaw_adjust = 0;

int throttle1 = 0;
int throttle2 = 0;
int throttle3 = 0;
int throttle4 = 0;

float lock_throttle;

// sonar
int sonar_error = 0;
long sonar_pid_time = 0;
float SonarPID = 0;
long sonarLastTime = 0;

extern volatile unsigned int alt_hold_count;

// Mavlink
float x_lock, x_ground, y_lock, y_ground, z_lock, z_ground, yaw_lock, yaw_ground;
float x_pid_output, y_pid_output, x_accel_pid_output, y_accel_pid_output, g_yaw_pid_output;

float x_p = 2.91;
float x_d = 0.5;

float y_p = 2.91;
float y_d = 0.5;

float g_yaw_p = 3.5;
float g_yaw_d = 1.2;

PID pid_x(&x_lock, &roll, &x_ground, x_p, 0, 0, DIRECT);
PID pid_y(&y_lock, &pitch, &y_ground, y_p, 0, 0, DIRECT);
PID pid_yaw(&yaw_lock, &g_yaw_pid_output, &yaw_ground, g_yaw_p, 0, 0, DIRECT);

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

	// height_sonar.SetMode(AUTOMATIC);
	// height_sonar.SetSampleTime(100);
	// height_sonar.SetOutputLimits(-150, 150);

	pid_x.SetMode(AUTOMATIC);
	pid_x.SetSampleTime(10);
	pid_x.SetOutputLimits(-450, 450);

	pid_y.SetMode(AUTOMATIC);
	pid_y.SetSampleTime(10);
	pid_y.SetOutputLimits(-450, 450);

	pid_yaw.SetMode(AUTOMATIC);
	pid_yaw.SetSampleTime(10);
	pid_yaw.SetOutputLimits(-200, 200);
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

void motor_slave_output() {
	motor1.writeMicroseconds(FL);
	motor2.writeMicroseconds(BL);
	motor3.writeMicroseconds(BR);
	motor4.writeMicroseconds(FR);
}

void motor_output() {
	if (motor_adjust_bit == true) {

		if (throttle > 1050) {
			pitch_angle.Compute();
			roll_angle.Compute();
			if (max_yaw == 0) {
				relative_yaw = relative_yaw;
			}
			else {
				relative_yaw = first_kal_yaw + max_yaw;
			}
			yaw_angle.Compute();

			float tmp = pitch_angle_pid_output + pitch_d * GyroX; //4, 0.04, 2
			float tmp1 = roll_angle_pid_output + roll_d * GyroY; //4, 0.04, 2
			float tmp2 = yaw_angle_pid_output + yaw_d * GyroZ; //4, 0.04, 2

			// If the propellers start to spin, save the real-time position
			if (groundMsgReady == 1) {
				groundMsgReady = 0;
				m_x -= m_x_change;
				m_y -= m_y_change;
				//z -= z_change;

				// if (abs(m_z_change) <= 50)
				// m_z_change_sonar = -m_z_change;
				// if (m_x > 99 && m_x < 330)
				x_ground = m_x;
				// if (m_y > 99 && m_y < 330)
				y_ground = m_y;
				// if (m_z > -230 && m_z < 0)
				z_ground = m_z;
				yaw_ground = m_yaw;

				pid_x.Compute();
				pid_y.Compute();

				pitch_angle.Compute();
				roll_angle.Compute();
				pid_yaw.Compute();

				tmp = pitch_angle_pid_output + y_d * GyroX; //4, 0.04, 2
				tmp1 = roll_angle_pid_output + x_d * GyroY; //4, 0.04, 2
				tmp2 = g_yaw_pid_output + g_yaw_d * GyroZ; //4, 0.04, 2

				// tmp = tmp + x_pid_output;// + x_d * Ay;
				// tmp1 = tmp1 + y_pid_output;// + x_d * Ax;
				// tmp2 = tmp2 - (g_yaw_pid_output + g_yaw_d * GyroZ);
			}


			// ch5
			unsigned long sonarTimeNow = millis();
			unsigned long sonarTimeChange = (sonarTimeNow - sonarLastTime);

			if (remoteMsgReady == true) {
				// sample time >= 100ms
				// set up the min. sample time limit
				if (sonarTimeChange >= 100) {

					// set up the max. sample time limit. It is also used as a filter.
					if (sonarTimeChange < 200) {
						// Serial.println(sonarTimeChange);

						// P
						sonar_error = lock_sonar_altitude - kal_sonar;
						sonar_error = constrain(sonar_error, -300, 300);

						// LPF
						static int sonar_error_offset = 0;
						sonar_error_offset -= sonar_error_offset / 8;
						sonar_error_offset += sonar_error;
						sonar_error = sonar_error_offset / 8;

						// Dead band
						if (abs(sonar_error) < 3) {
							sonar_error = 0;
						}

						// D
						float tmp3 = constrain(height_sonar_p * sonar_error, -150, 150);
						tmp3 -= constrain(height_sonar_d * Vel_Z, -150, 150);
						SonarPID = tmp3;
					}

					sonarLastTime = sonarTimeNow;
				}
			}
			else
			{
				lock_sonar_altitude = kal_sonar;
				// float tmp3 = 0;
				SonarPID = 0;
			}



			// motor output
			throttle1 = throttle - tmp - tmp1 - tmp2 + SonarPID;
			throttle2 = throttle + tmp - tmp1 + tmp2 + SonarPID;
			throttle3 = throttle + tmp + tmp1 - tmp2 + SonarPID;
			throttle4 = throttle - tmp + tmp1 + tmp2 + SonarPID;


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

			// Save the take-off position
			if (groundMsgReady == 1) {
				groundMsgReady = 0;
				x_lock = m_x;
				y_lock = m_y;
				z_lock = m_z;
				yaw_lock = m_yaw;
			}

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

}
