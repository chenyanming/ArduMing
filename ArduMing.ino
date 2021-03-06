/**
 * Timer2: 1ms overflow, control output
 */
#include <SPI.h>
#include "config.h"

// spi
#define SPI0_MISO_PIN 50
#define SPI0_MOSI_PIN 51
#define SPI0_SCK_PIN  52
const int ChipSelPin1 = 53;
const int ChipSelPin2 = 40;

// led
const int blueled = 25;
const int yellowled = 26;
const int redled = 27;
boolean redled_state = true;
boolean yellowled_state = true;
boolean blueled_state = true;
unsigned int toggle = 0;  //used to keep the state of the LED

// counters in timer2
volatile unsigned int count = 0;   //used to keep count of how many interrupts were fired
volatile unsigned int mpu_time_count = 0; //The MPU running time
volatile unsigned int rc_time_count = 0;	//The RC running time
volatile unsigned int ch6_count = 0;	//The ch6 detect time
volatile unsigned int alt_average_timer = 0;
volatile unsigned int hundred_timer = 0;
volatile unsigned int alt_mode_timer = 0;

// yaw
unsigned char yaw_status = 0;
extern float first_kal_yaw, relative_yaw, max_yaw, kal_rc_yaw;
volatile unsigned int min_yaw_timer = 0;
extern int round_timer;

// ch5
boolean remoteMsgReady = false;

// ch6
unsigned int last_ch6;
unsigned int last_ch6_d = 0;

// hold altitude
volatile unsigned int alt_hold_count = 0;

// Print
unsigned long serialTime;

// Serial2
unsigned char serial2_rev_buf[50];
unsigned char rev_i = 0, rev_j = 0, comma = 0;
boolean rev_unhealthy = false;
unsigned char FL_tmp[4];
unsigned char BL_tmp[4];
unsigned char FR_tmp[4];
unsigned char BR_tmp[4];
unsigned int FL = 0, BL = 0, FR = 0, BR = 0;


/**
 * @param {N/A} TIMER2_OVF_vect [Timer2 Overflow Interrupt Vector, called every 1ms]
 */
ISR(TIMER2_OVF_vect) {
	mpu_time_count++;
	rc_time_count++;
	alt_average_timer++;
	hundred_timer++;
	alt_mode_timer++;

	/*
	 * Dectect the yaw stick value when pilot release, and when is the real 0 point which is not detected after release the yaw stick
	 */
	if (min_yaw_timer > 0) {
		min_yaw_timer--;
		if (min_yaw_timer == 0) {
			if (kal_rc_yaw == 0) {
				yaw_status = yaw_status << 0x01;
			}
			else {
				yaw_status = (yaw_status << 1) | 0x01;
			}
			if ( ((yaw_status & 0x02) == 0) && ((yaw_status & 0x01) == 0))
				max_yaw = 0;
			if ( ((yaw_status & 0x02) == 0x02) && ((yaw_status & 0x01) == 0)) {
				first_kal_yaw = kal_yaw;
				max_yaw = 0;
			}
		}

	}

	count++;               //Increments the interrupt counter
	if (count >= 500) {
		// digitalWrite(27, toggle);
		// toggle = !toggle;    //toggles the LED state
		count = 0;           //Resets the interrupt counter
	}

	ch6_count++;
	if (ch6_count >= 300) {
		// pitch_angle.SetTunings(1.84, 0.18, 0);
		// roll_angle.SetTunings(1.84, 0.18, 0);
		// yaw_angle.SetTunings(1 + last_ch6_p, 0, 0);
		// last_ch6_p = ch6;
		// yaw_angle.SetTunings(2.5, 0, 0);

		// height_baro.SetTunings(ch6, 0, 0);
		// height_sonar.SetTunings(2, 0, 1);
		// height_sonar_2.SetTunings(0.5, 0, 0.5);

		// x_p = 2 + ch6;
		// y_p = 2 + ch6;
		// pid_x.SetTunings(x_p, 0, 0);
		// pid_y.SetTunings(y_p, 0, 0);
		// x_d = 0.5 + ch6;
		// y_d = 0.5 + ch6;


		if (abs(ch6 - last_ch6) >= 0.01) {
			// roll_d = ch6;
			// height_sonar_d = ch6;
			// height_sonar.SetTunings(height_sonar_p, 0, height_sonar_d);

		}
		last_ch6 = ch6;

		ch6_count = 0;
	}

	/*
	 * Dectect the D1 and D2 adc conversation end time
	 */
	// if (D2_timer > 0) {
	// 	D2_timer--;
	// 	if (D2_timer == 5)
	// 		D2_ready = true;
	// 	if (D2_timer == 0)
	// 		turn_ready = false;
	// }

	// if (D1_timer > 0) {
	// 	D1_timer--;
	// 	if (D1_timer == 5)
	// 		D1_ready = true;
	// 	if (D1_timer == 0)
	// 		turn_ready = true;
	// }

	if (convert_timer > 0) {
		convert_timer--;
		if (convert_timer == 8) {
			convert_finish = true;
		}
	}

	/**
	 * Hover
	 */
	// if (alt_hold_count > 0) {
	// 	alt_hold_count--;
	// 	if (alt_hold_count == 0) {
	// 	}
	// }

	TCNT2 = 130;           //Reset Timer to 130 out of 255, 255-130 = 125 counts = 125*8us = 1ms
	TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};


void setup()
{
	Serial.begin(57600);	// Set the baud rate to 115200
	Serial1.begin(57600);    // 3DR
	Serial2.begin(115200);	// Used to communicate with the Slave

	rc_setup();
	motor_setup();
	// pinMode(led, OUTPUT);	// Set built-in LED to OUPUT mode
	pinMode(blueled, OUTPUT);	// Set built-in LED to OUPUT mode
	pinMode(yellowled, OUTPUT);	// Set built-in LED to OUPUT mode
	pinMode(redled, OUTPUT);	// Set built-in LED to OUPUT mode

	digitalWrite(blueled, HIGH);
	digitalWrite(yellowled, HIGH);
	digitalWrite(redled, HIGH);

	/**
	 * Setup Timer2 to fire every 1ms
	 */
	TCCR2B = 0x00;        //Disbale Timer2 while we set it up
	TCNT2  = 130;         //Reset Timer Count to 130 out of 255
	TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
	TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
	TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prmotoraler set to 128, 16MHz/128 = 125KHz = 8us per count

	// while (Serial.available() > 0) Serial.read(); // Flush serial buffer to clean up remnants from previous run
	// Serial.println("################ Start ################");

	pinMode(ChipSelPin2, OUTPUT);
	digitalWrite(ChipSelPin2, HIGH);

	// pinMode(SPI0_MISO_PIN, INPUT);
	// pinMode(SPI0_MOSI_PIN, OUTPUT);
	// pinMode(SPI0_SCK_PIN, OUTPUT);

	pinMode(ChipSelPin1, OUTPUT);  //--- Configure the chip select pin as output ---//
	digitalWrite(ChipSelPin1, HIGH);

	/**
	 * SPI Settings
	 */
	// Serial.println("Initializing SPI Protocol...");
	SPI.begin();  // start the SPI library
	// SPCR = _BV(SPE) | _BV(MSTR);
	SPI.setClockDivider(SPI_CLOCK_DIV16); // Arduino Mega2560 board runs on 16 MHz: 16 MHz / SPI_CLOCK_DIV16 = 1 MHz
	// 1 MHz is the maximum SPI clock frequency according to the MPU-6000 Product Specification
	SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
	SPI.setDataMode(SPI_MODE3); // latched on rising edge, transitioned on falling edge, active low  ****why****
	// Serial.println("SPI Protocol initializing done.");
	// Serial.println("################# 1. #################");
	delay(100);

	/**
	 * DMP initialization
	 */
	// write & verify dmpMemory, dmpConfig and dmpUpdates into the DMP, and make all kinds of other settings
	// !!! this is the main routine to make the DMP work, and get the quaternion out of the FIFO !!!
	// Serial.println("Initializing Digital Motion Processor (DMP)...");
	unsigned char devStatus; // return status after each device operation (0 = success, !0 = error)
	devStatus = dmpInitialize();
	// Serial.println(devStatus);
	// Serial.println("OK END initial");

	// make sure it worked: dmpInitialize() returns a 0 in devStatus if so
	if (devStatus == 0)	{
		digitalWrite(blueled, LOW); // shows end of write DMP configuration

		// now that it's ready, turn on the DMP
		// Serial.print("Enabling DMP... ");
		// SPIwriteBit(0x6A, 7, true, ChipSelPin1);
		spi_SetBits(ChipSelPin1, 0x6A, (1 << 7)); // USER_CTRL_DMP_EN

		// Serial.println("done.");

		// enable Arduino interrupt detection, this will execute dmpDataReady whenever there is an interrupt,
		// independing on what this sketch is doing at that moment
		// http://arduino.cc/en/Reference/AttachInterrupt
		// Serial.print("Enabling interrupt detection... ");
		// attachInterrupt(interrupt, function, mode) specifies a function to call when an external interrupt occurs
		// Arduino Mega2560 INT6 / PE6 (input) connected to MPU-6000 INT pin 12 (output)
		attachInterrupt(6, dmpDataReady, RISING); // the 0 points correctly to INT6 / PE6
		// -> if there is an interrupt from MPU-6000 to ATMEGA2560, boolean mpuInterrupt will be made true
		byte mpuIntStatus = spi_readReg(ChipSelPin1, 0x3A); // by reading INT_STATUS register, all interrupts are cleared

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;
		// Serial.println("DMP ready! Waiting for first data from MPU-6000...");
	}
	else // have to check if this is still functional
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print("DMP Initialization failed (code ");
		Serial.print(devStatus);
		Serial.println(")");
	}

	// MS5611 setup
	ms5611_setup();

	// HMC5883L setup
	boolean hmcStatus;
	hmcStatus = hmc_setup();
	if (hmcStatus == false) {
		// Serial.print("HMC Initialization failed (code ");
		// Serial.print(hmcStatus);
		// Serial.println(")");
	}

	// Sonar
	sonar_setup();

	// Serial.println("############# LOOP... ##############");
	mpu_time_count = 0;
	rc_time_count = 0;
} // End of Setup

void loop() {
	// long dump, dump1;
	// dump = micros();
	int rev, incomingByte;

	if (rev = Serial.available()) {
		incomingByte = Serial.read();
		// if (incomingByte == 'r') {
		// 	redled_state = !redled_state;
		// 	digitalWrite(redled, redled_state);
		// }
		// else if (incomingByte == 'b') {
		// 	blueled_state = !blueled_state;
		// 	digitalWrite(blueled, blueled_state);
		// }
		// else if (incomingByte == 'y') {
		// 	yellowled_state = !yellowled_state;
		// 	digitalWrite(yellowled, yellowled_state);
		// }

		if (incomingByte == '9') {
			pitch_p = pitch_p + 0.01;
			pitch_angle.SetTunings(pitch_p, 0, 0);
		}
		else if (incomingByte == '0') {
			pitch_p = pitch_p - 0.01;
			pitch_angle.SetTunings(pitch_p, 0, 0);
		}
		else if (incomingByte == '-') {
			pitch_d += 0.01;
		}
		else if (incomingByte == '=') {
			pitch_d -= 0.01;
		}
		else if (incomingByte == 'o') {
			height_sonar_p += 0.01;
		}
		else if (incomingByte == 'p') {
			height_sonar_p -= 0.01;
		}
		else if (incomingByte == '[') {
			height_sonar_d += 0.01;
		}
		else if (incomingByte == ']') {
			height_sonar_d -= 0.01;
		}
		else if (incomingByte == 'k') {
			roll_p = roll_p + 0.01;
			roll_angle.SetTunings(roll_p, 0, 0);
		}
		else if (incomingByte == 'l') {
			roll_p = roll_p - 0.01;
			roll_angle.SetTunings(roll_p, 0, 0);
		}
		else if (incomingByte == ';') {
			roll_d += 0.01;
		}
		else if (incomingByte == '\'') {
			roll_d -= 0.01;
		}
		// else if (incomingByte == ';') {
		// 	pitch_speed_i = pitch_speed_i + 0.01;
		// 	pitch_speed.SetTunings(pitch_speed_p, pitch_speed_i, pitch_speed_d);
		// }
		// else if (incomingByte == '\'') {
		// 	pitch_speed_i = pitch_speed_i - 0.01;
		// 	pitch_speed.SetTunings(pitch_speed_p, pitch_speed_i, pitch_speed_d);
		// }
		// else if (incomingByte == '.') {
		// 	pitch_speed_d = pitch_speed_d + 0.01;
		// 	pitch_speed.SetTunings(pitch_speed_p, pitch_speed_i, pitch_speed_d);
		// }
		// else if (incomingByte == '/') {
		// 	pitch_speed_d = pitch_speed_d - 0.01;
		// 	pitch_speed.SetTunings(pitch_speed_p, pitch_speed_i, pitch_speed_d);
		// }

		// if (incomingByte == '-') {
		// 	roll_p = roll_p + 0.01;
		// 	roll_angle.SetTunings(roll_p, 0, 0);
		// }
		// else if (incomingByte == '=') {
		// 	roll_p = roll_p - 0.01;
		// 	roll_angle.SetTunings(roll_p, 0, 0);
		// 	}

	}

	while (Serial2.available() > 0) {
		serial2_rev_buf[rev_i] = Serial2.read();
		// if (serial2_rev_buf[rev_i] == ',') {
		// 	rev_j = 0;
		// 	comma++;
		// }
		// else if (serial2_rev_buf[rev_i] == '\n') {
		if (serial2_rev_buf[rev_i] == '\n') {
			// for (int j = 0; j < rev_i; j++) {
			// 	Serial.write(serial2_rev_buf[j]);
			// }
			// Serial.println();

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (serial2_rev_buf[5 * i + j] < '0' || serial2_rev_buf[5 * i + j] > '9') {
						rev_unhealthy = true;
					}
				}
			}
			if (rev_unhealthy == false) {
				FL = (serial2_rev_buf[0] - '0') * 1000 + (serial2_rev_buf[1] - '0') * 100 + (serial2_rev_buf[2] - '0') * 10 + (serial2_rev_buf[3] - '0') * 1;
				BL = (serial2_rev_buf[5] - '0') * 1000 + (serial2_rev_buf[6] - '0') * 100 + (serial2_rev_buf[7] - '0') * 10 + (serial2_rev_buf[8] - '0') * 1;
				BR = (serial2_rev_buf[10] - '0') * 1000 + (serial2_rev_buf[11] - '0') * 100 + (serial2_rev_buf[12] - '0') * 10 + (serial2_rev_buf[13] - '0') * 1;
				FR = (serial2_rev_buf[15] - '0') * 1000 + (serial2_rev_buf[16] - '0') * 100 + (serial2_rev_buf[17] - '0') * 10 + (serial2_rev_buf[18] - '0') * 1;
				// Serial.print(FL);
				// Serial.print(' ');
				// Serial.print(BL);
				// Serial.print(' ');
				// Serial.print(FR);
				// Serial.print(' ');
				// Serial.println(BR);
			}
			else {
				rev_unhealthy = false;
				FL = FL;
				BL = BL;
				BR = BR;
				FR = FR;
			}


			rev_i = 0;
			// rev_j = 0;
			// comma = 0;
			break;
		}
		// else if ((serial2_rev_buf[rev_i] <= '9') && (serial2_rev_buf[rev_i] >= '0')) {
		// 	switch (comma) {
		// 	case 0:
		// 		FL_tmp[rev_j] = serial2_rev_buf[rev_i];
		// 		if (rev_j == 3)
		// 			FL = (FL_tmp[0] - '0') * 1000 + (FL_tmp[1] - '0') * 100 + (FL_tmp[2] - '0') * 10 + (FL_tmp[3] - '0') * 1;
		// 		rev_j++;
		// 		break;
		// 	case 1:
		// 		BL_tmp[rev_j] = serial2_rev_buf[rev_i];
		// 		if (rev_j == 3)
		// 			BL = (BL_tmp[0] - '0') * 1000 + (BL_tmp[1] - '0') * 100 + (BL_tmp[2] - '0') * 10 + (BL_tmp[3] - '0') * 1;
		// 		rev_j++;
		// 		break;
		// 	case 2:
		// 		FR_tmp[rev_j] = serial2_rev_buf[rev_i];
		// 		if (rev_j == 3)
		// 			FR = (FR_tmp[0] - '0') * 1000 + (FR_tmp[1] - '0') * 100 + (FR_tmp[2] - '0') * 10 + (FR_tmp[3] - '0') * 1;
		// 		rev_j++;
		// 		break;
		// 	case 3:
		// 		BR_tmp[rev_j] = serial2_rev_buf[rev_i];
		// 		if (rev_j == 3)
		// 			BR = (BR_tmp[0] - '0') * 1000 + (BR_tmp[1] - '0') * 100 + (BR_tmp[2] - '0') * 10 + (BR_tmp[3] - '0') * 1;
		// 		rev_j++;
		// 		break;
		// 	default:
		// 		break;
		// 	}
		// }
		// else if (serial2_rev_buf[rev_i] < '0' || serial2_rev_buf[rev_i] > '9') {
		// 	rev_i = 0;
		// 	break;
		// }
		// else {
		// comma = 0;
		// rev_i = 0;
		// rev_j = 0;
		// break;
		// }
		if (rev_i == 49)
			rev_i = 0;
		rev_i++;
	}


	// if DMP initialization during setup failed, don't try to do anything
	if (!dmpReady)
	{
		return;
	}

	mpu_get(); // Take nearly 2.4ms to run, once there is an interrupt
	// hmc_get();

	rc_get(); // Take nearly 0.7ms to run, once calibration is done

	/**
	 * MS5611 convert D1 and D2 in turn.
	 * Each conversation need 20 mS which is determinded by convert_timer
	 */
	ms5611_convert();
	/**
	 * When MS5611 calibration is done, ms5611_adjust_bit will be set, and start to calculate the temp, press and alt.
	 */
	ms5611_get();



	mavlink_get();

	if (alt_average_timer >= 30) {
		alt_average_timer = 0;
		ms5611_alt_average();
	}

	if (alt_mode_timer >= 50) {
		alt_mode_timer = 0;
	}


	if (rc_time_count >= 10) {
		rc_time_count = 0;
		sonar_get();
		// sonar_mode();
		motor_adjust();
		rc_adjust();
		ms5611_adjust();
		// if ((on_ch5 == true) && (ch5_count == 0))
		// 	motor_output();
		// else if ((off_ch5 == true) && (ch5_count == 0))
		// 	motor_output();
		// 	lock_sonar_altitude = kal_sonar;
		// else if ((off_ch5 == true) && (ch5_count == 5)) {
		// 	motor_slave_output();
		// }
		// else {
		// 	motor_output();
		// }
		motor_output();
		if (on_ch5 == true) {
			remoteMsgReady = true;
		}
		else if (off_ch5 == true) {
			remoteMsgReady = false;
		}
		// Serial_throttle();
	}

	if (hundred_timer >= 100) {
		hundred_timer = 0;
		// SerialReceive();
		// SerialSend_pit();
		// SerialSend_rol();
		// SerialSend_yaw();
		// Serial_rc();
		// Serial_gyro();
		// Serial_accel();
		// Serial_alt();
		// Serial_att();
		// Serial_heading();
		//Serial.println('y');
		// Serial_slave();
		// Serial_master();
		// Serial_mavlink();
		// Serial_mavlink_test();
	}

	//send-receive with matlab if it's time
	// if (millis() > serialTime)
	// {
	// SerialReceive();
	// SerialSend_pit();
	// SerialSend_rol();
	// SerialSend_yaw();
	// Serial_rc();
	// Serial_gyro();
	// Serial_accel();
	// Serial_alt();
	// Serial_heading();
	// Serial2.println("testing...");
	// serialTime += 100;
	// }

	// dump1 = micros();
	// Serial.println(dump1 - dump);
} // End of loop()

void Serial_rc() {

	Serial.print("Getting the remote control value : ");
	Serial.print(pitch); Serial.print('\t');
	Serial.print(roll); Serial.print('\t');
	Serial.print(yaw); Serial.print('\t');
	Serial.print(throttle); Serial.print('\t');
	Serial.print(ch5); Serial.print('\t');
	Serial.println(ch6, 6); //Floats have only 6-7 decimal digits of precision. On  the Arduino, double is the same size as float.
}

void Serial_gyro() {
	Serial.print("PID ");
	Serial.print(GyroX);
	Serial.print(" ");
	Serial.print(GyroY);
	Serial.print(" ");
	Serial.print(GyroZ);
	Serial.print(" ");
	Serial.println("END");
}

void Serial_pitch() {
	Serial.print(pitch); Serial.print("\t");
	Serial.print(rpy_pit); Serial.print("\t");
	Serial.println(kal_pit);
}

void SerialSend_pit()
{
	Serial.print("PID ");
	Serial.print(pitch);
	Serial.print(" ");
	Serial.print(kal_pit);
	Serial.print(" ");
	Serial.print(pitch_p);
	Serial.print(" ");
	Serial.print(pitch_d);
	Serial.print(" ");
	// Serial.print(pitch_speed_p);
	// Serial.print(" ");
	// Serial.print(pitch_speed_i);
	// Serial.print(" ");
	// Serial.print(pitch_speed_d);
	// Serial.print(" ");
	Serial.print(pitch_angle_pid_output);
	Serial.print(" ");
	// Serial.print(pitch_speed_pid_output);
	// Serial.print(" ");
	// Serial.print(float(throttle3) / 1000);
	// Serial.print(" ");
	// Serial.print(GyroX);
	// Serial.print(" ");
	// Serial.print(rpy_yaw);
	// Serial.print(" ");
	// Serial.print(ch6);
	// Serial.print(" ");
	// Serial.print(pitch_angle.GetKi());
	// Serial.print(" ");
	// Serial.print(pitch_angle.GetKd());
	// Serial.print(" ");
	Serial.println("END");
	// if (pitch_angle.GetMode() == AUTOMATIC) Serial.print("Automatic");
	// else Serial.print("Manual");
	// Serial.print(" ");
	// if (pitch_angle.GetDirection() == DIRECT) Serial.println("Direct");
	// else Serial.println("Reverse");
}

void SerialSend_rol()
{
	Serial.print("PID ");
	Serial.print(roll);
	Serial.print(" ");
	Serial.print(kal_rol);
	Serial.print(" ");
	Serial.print(roll_p);
	Serial.print(" ");
	Serial.print(ch6);
	Serial.print(" ");
	// Serial.print(roll_angle_pid_output);
	// Serial.print(" ");
	// Serial.print(float(throttle2) / 1000);
	// Serial.print(" ");
	// Serial.print(float(throttle4) / 1000);
	// Serial.print(" ");
	// Serial.print(GyroY);
	// Serial.print(" ");
	// Serial.print(rpy_yaw);
	// Serial.print(" ");
	// Serial.print(roll_angle.GetKp());
	// Serial.print(" ");
	// Serial.print(roll_angle.GetKi());
	// Serial.print(" ");
	// Serial.print(roll_angle.GetKd());
	// Serial.print(" ");
	Serial.println("END");
}

void SerialSend_yaw()
{
	Serial.print("PID ");
	Serial.print(kal_yaw);
	Serial.print(" ");
	Serial.print(relative_yaw);
	Serial.print(" ");
	Serial.print(max_yaw);
	Serial.print(" ");
	Serial.print(round_timer);
	Serial.print(" ");
	// Serial.print(yaw);
	// Serial.print(" ");
	// Serial.print(GyroZ);
	// Serial.print(" ");
	// Serial.print(rpy_yaw);
	// Serial.print(" ");
	// Serial.print(yaw_angle.GetKp());
	// Serial.print(" ");
	// Serial.print(yaw_angle.GetKi());
	// Serial.print(" ");
	// Serial.print(yaw_angle.GetKd());
	// Serial.print(" ");
	Serial.println("END");
}


void Serial_accel()
{
	// float Az_G = (kal_rol*Ax + kal_pit*Ay + kal_yaw*Az)/sqrt(Ax*Ax + Ay*Ay + Az*Az) - sqrt(Ax*Ax + Ay*Ay + Az*Az);
	Serial.print("Accel ");
	// Serial.print(AcceZ_W); Serial.print(" ");
	// Serial.print(AcceZ_W); Serial.print(" ");
	// Serial.print(AcceZ_W); Serial.print(" ");
	// Serial.print(AcceZ_W); Serial.print(" ");
	// Serial.print(AcceZ_W); Serial.print(" ");
	// Serial.print(AcceZ_W); Serial.print(" ");
	// Serial.print(AcceX_L);
	// Serial.print(" ");
	// Serial.print(AcceY_L);
	// Serial.print(" ");
	// Serial.print(AcceZ_L);
	// Serial.print(" ");
	// Serial.print(AcceX_W);
	// Serial.print(" ");
	// Serial.print(AcceY_W);
	// Serial.print(" ");
	Serial.print(pitch);
	Serial.print(" ");
	Serial.print(roll);
	Serial.print(" ");
	Serial.print(yaw);
	Serial.print(" ");
	Serial.print(throttle);
	Serial.print(" ");
	Serial.print(ch5);
	Serial.print(" ");
	Serial.print(ch6);
	Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_Z); Serial.print(" ");
	// Serial.print(Vel_Z); Serial.print(" ");
	// Serial.print(Vel_Z); Serial.print(" ");
	// Serial.print(Vel_Z); Serial.print(" ");
	// Serial.print(Vel_Z); Serial.print(" ");
	// Serial.print(Vel_Z); Serial.print(" ");
	// Serial.print(pitch_p); Serial.print(" ");
	// Serial.print(pitch_d); Serial.print(" ");
	// Serial.print(roll_p); Serial.print(" ");
	// Serial.print(roll_d); Serial.print(" ");
	// Serial.print(height_sonar_p); Serial.print(" ");
	// Serial.print(height_sonar_d); Serial.print(" ");
	// Serial.print(SonarPID); Serial.print(' ');
	// Serial.print(SonarPID); Serial.print(' ');
	// Serial.print(SonarPID); Serial.print(' ');
	// Serial.print(SonarPID); Serial.print(' ');
	// Serial.print(SonarPID); Serial.print(' ');
	// Serial.print(SonarPID); Serial.print(' ');
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	// Serial.print(Vel_sonar); Serial.print(" ");
	Serial.println("END");
}

void Serial_alt() {
	Serial.print("ALT "); Serial.print(' ');
	// Serial.print(throttle); Serial.print(' ');
	// Serial.print(ch5_count); Serial.print(' ');
	// Serial.print(ch6); Serial.print(' ');
	Serial.print(baro_altitude); Serial.print(' ');
	Serial.print(baro_altitude); Serial.print(' ');
	Serial.print(baro_altitude); Serial.print(' ');
	Serial.print(baro_altitude); Serial.print(' ');
	Serial.print(baro_altitude); Serial.print(' ');
	Serial.print(baro_altitude); Serial.print(' ');
	// Serial.print(height_sonar_p); Serial.print(' ');
	// Serial.print(lock_sonar_altitude); Serial.print(' ');
	// Serial.print(_sonar_altitude); Serial.print(' ');
	// Serial.print(_sonar_mode_altitude); Serial.print(' ');
	// Serial.print(kal_sonar); Serial.print(' ');
	// Serial.print(_sonar_mode_altitude); Serial.print(' ');
	// Serial.print(height_sonar_pid_output); Serial.print(' ');
	Serial.println("END");
}

void Serial_att() {
	Serial.print("ATT"); Serial.print(' ');
	Serial.print(kal_pit); Serial.print(' ');
	Serial.print(kal_rol); Serial.print(' ');
	Serial.print(kal_yaw); Serial.print(' ');
	Serial.println("END");

}

void Serial_heading() {
	Serial.print("HMC"); Serial.print(' ');
	Serial.print(kal_pit); Serial.print(' ');
	Serial.print(kal_rol); Serial.print(' ');
	Serial.print(kal_yaw); Serial.print(' ');
	Serial.print(_heading); Serial.print(' ');
	Serial.println("END");

}

void Serial_throttle() {
	Serial.print("throttle"); Serial.print(' ');
	Serial.print(throttle); Serial.print(' ');
	Serial.print(throttle1); Serial.print(' ');
	Serial.print(throttle2); Serial.print(' ');
	Serial.print(throttle3); Serial.print(' ');
	Serial.print(throttle4); Serial.print(' ');
	Serial.println("END");
}

void Serial_slave() {
	Serial.print("Slave"); Serial.print(' ');
	Serial.print(FL); Serial.print(' ');
	Serial.print(BL); Serial.print(' ');
	Serial.print(BR); Serial.print(' ');
	Serial.print(FR); Serial.print(' ');
	Serial.println("END");

}

void Serial_master() {
	Serial.print("Master"); Serial.print(' ');
	Serial.print(kal_pit); Serial.print(' ');
	Serial.print(kal_rol); Serial.print(' ');
	Serial.print(kal_yaw); Serial.print(' ');
	Serial.print(GyroX); Serial.print(' ');
	Serial.print(GyroY); Serial.print(' ');
	Serial.print(GyroZ); Serial.print(' ');
	Serial.println("END");

}

void Serial_mavlink() {
	Serial.print("MAVLINK ");
	Serial.print(m_x);
	Serial.print(" ");
	Serial.print(m_y);
	Serial.print(" ");
	Serial.print(m_z);
	Serial.print(" ");
	Serial.print(m_yaw);
	Serial.print(" ");
	Serial.print(m_x_change);
	Serial.print(" ");
	Serial.print(m_y_change);
	Serial.print(" ");
	Serial.print(m_z_change);
	Serial.print(" ");
	Serial.print(m_ck);
	Serial.print(" ");
	Serial.println("END");

}

void Serial_mavlink_test() {
	Serial.print("MAVLINK ");
	Serial.print(x_p);
	Serial.print(" ");
	Serial.print(x_d);
	Serial.print(" ");
	Serial.print(y_p);
	Serial.print(" ");
	Serial.print(y_d);
	Serial.print(" ");
	Serial.print(Ax);
	Serial.print(" ");
	Serial.print(Ay);
	Serial.print(" ");
	Serial.print(m_x);
	Serial.print(" ");
	Serial.print(m_y);
	Serial.print(" ");
	Serial.println("END");

}

void Serial_all() {
	Serial.print("Pitch "); Serial.print(pitch); Serial.print(' '); Serial.println(kal_pit);
	Serial.print("Roll "); Serial.print(roll); Serial.print(' '); Serial.println(kal_rol);
	Serial.print("Yaw "); Serial.print(yaw); Serial.print(' '); Serial.println(kal_yaw);
	Serial.print("Gyro_Pitch "); Serial.println(GyroX);
	Serial.print("Gyro_Roll "); Serial.println(GyroY);
	Serial.print("Gyro_Yaw "); Serial.println(GyroZ);
	Serial.print("Accel_Pitch "); Serial.print(Ay); Serial.print(' '); Serial.println(AcceY);
	Serial.print("Accel_Roll "); Serial.print(Ax); Serial.print(' '); Serial.println(AcceX);
	Serial.print("Accel_Z "); Serial.print(Az); Serial.print(' '); Serial.println(AcceZ);

}
