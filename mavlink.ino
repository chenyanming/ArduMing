// #include <string.h>
// #include <stdio.h>
char serial1_buffer[100];
int indexBuffer = 0;
long time_lastMsgRead;
int groundMsgReady = 0;
float m_x, m_y, m_z, m_yaw, m_x_change, m_y_change, m_z_change, m_ck;

int bytesAvailable = 0;


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
		groundMsgReady = 1;
		redled_state = !redled_state;
		digitalWrite(redled, redled_state);
		blueled_state = !blueled_state;
		digitalWrite(blueled, blueled_state);
		yellowled_state = !yellowled_state;
		digitalWrite(yellowled, yellowled_state);

		time_lastMsgRead = millis();
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
