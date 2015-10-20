// #include <string.h>
// #include <stdio.h>
char serial1_buffer[100];
int indexBuffer = 0;
long time_lastMsgRead;
int groundMsgReady = 0;
float m_x, m_y, m_z, m_yaw, m_x_change, m_y_change, m_z_change, m_ck;
float x_lock, x_ground, y_lock, y_ground;
float x_pid_output, y_pid_output, x_accel_pid_output, y_accel_pid_output; 
PID pid_x(&x_lock, &x_pid_output, &x_ground, 0.2, 0, 0.5, DIRECT);
PID pid_y(&y_lock, &y_pid_output, &y_ground, 0.2, 0, 0.5, DIRECT);
PID pid_x_accel(&x_pid_output, &x_accel_pid_output, &Ax, 0.5, 0, 0, DIRECT);
PID pid_y_accel(&y_pid_output, &y_accel_pid_output, &Ay, 0.5, 0, 0, DIRECT);

int bytesAvailable = 0;

void ground_setup() {
	pid_x.SetMode(AUTOMATIC);
	pid_x.SetSampleTime(10);
	pid_x.SetOutputLimits(-450, 450);

	pid_y.SetMode(AUTOMATIC);
	pid_y.SetSampleTime(10);
	pid_y.SetOutputLimits(-450, 450);

	pid_x_accel.SetMode(AUTOMATIC);
	pid_x_accel.SetSampleTime(10);
	pid_x_accel.SetOutputLimits(-450, 450);

	pid_y_accel.SetMode(AUTOMATIC);
	pid_y_accel.SetSampleTime(10);
	pid_y_accel.SetOutputLimits(-450, 450);
}

void extractMsg() {
	long checksum;
	m_x = strtol(strtok(serial1_buffer, ","), NULL, 10);
	m_y = strtol(strtok(NULL, ","), NULL, 10);
	m_z = strtol(strtok(NULL, ","), NULL, 10);
	m_yaw = strtol(strtok(NULL, ","), NULL, 10);
	m_x_change = strtol(strtok(NULL, ","), NULL, 10);
	m_y_change = strtol(strtok(NULL, ","), NULL, 10);
	m_z_change = strtol(strtok(NULL, ","), NULL, 10);
	m_ck = strtol(strtok(NULL, ","), NULL, 10);

	checksum = m_x + m_y + m_z + m_yaw + m_x_change + m_y_change + m_z_change;
	checksum = checksum % 1000;

	//hal.console->printf_P(PSTR("time: %d "),hal.scheduler->millis());

	if (checksum == m_ck)
	{
		/*
				x -= x_change;
				y -= y_change;
				//z -= z_change;


				if (abs(z_change) <= 50)
					z_change_sonar = -z_change;

				if (x > 99 && x < 330)
					x_ground = x;
				else
					// hal.console->printf_P(PSTR("x_ground error %ld\n"), x);

				if (y > 99 && y < 330)
					y_ground = y;
				else
					// hal.console->printf_P(PSTR("y_ground error %ld\n"), y);

				if (z > -230 && z < 0)
					z_ground = z;
				else
					// hal.console->printf_P(PSTR("z_ground error %ld\n"), z);

				yaw_ground = yaw;
		*/

		// Extract one frame mavlink data done. Record the system time and set the groudMsgReady to 1.
		time_lastMsgRead = millis();
		groundMsgReady = 1;
	}
	else
		Serial.println("checksum fail");
}

void mavlink_get() {
	char c;
	bytesAvailable = Serial1.available();
	while (bytesAvailable > 0) {
		c = (char)Serial1.read();
		if (c == 'S') // 'S' stands for "Start Char"
			indexBuffer = 0;
		else if (c == 'E') { // 'E' stands for "End Char"
			serial1_buffer[indexBuffer] = '\0';
			extractMsg(); // After it recieves all chars, it starts to extract.
		}
		else {
			serial1_buffer[indexBuffer] = c; // Save the received chars to serial1_buffer.
			indexBuffer++;
		}
		bytesAvailable--;
	}
}


void ground_compute() {
	pid_x.Compute();
	pid_x_accel.Compute();
	pid_y.Compute();
	pid_y_accel.Compute();
}

