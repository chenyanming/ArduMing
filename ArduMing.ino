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

// yaw
unsigned char yaw_status = 0;
extern float first_kal_yaw, relative_yaw, max_yaw, kal_rc_yaw;
volatile unsigned int min_yaw_timer = 0;
extern int round_timer;

// ch6
unsigned int last_ch6_p = 0;
unsigned int last_ch6_d = 0;

// hold altitude
volatile unsigned int alt_hold_count = 0;

// Print
unsigned long serialTime;

/**
 * @param {N/A} TIMER2_OVF_vect [Timer2 Overflow Interrupt Vector, called every 1ms]
 */
ISR(TIMER2_OVF_vect) {
	mpu_time_count++;
	rc_time_count++;
	alt_average_timer++;
	hundred_timer++;

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
		pitch_angle.SetTunings(1.84, 0.18, 0);
		roll_angle.SetTunings(1.84, 0.18, 0);
		// yaw_angle.SetTunings(1 + last_ch6_p, 0, 0);
		// last_ch6_p = ch6;
		yaw_angle.SetTunings(2.5, 0, 0);

		height_baro.SetTunings(ch6, 0, 0);

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
	Serial.begin(115200);	// Set the baud rate to 115200
	Serial2.begin(57600);

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
		Serial.print("HMC Initialization failed (code ");
		Serial.print(hmcStatus);
		Serial.println(")");
	}

	// Serial.println("############# LOOP... ##############");
	mpu_time_count = 0;
	rc_time_count = 0;
} // End of Setup

void loop() {
	// long dump, dump1;
	// dump = micros();
	// int rev, incomingByte;

	// if (rev = Serial.available()) {
	// 	incomingByte = Serial.read();
	// 	if (incomingByte == 'r') {
	// 		redled_state = !redled_state;
	// 		digitalWrite(redled, redled_state);
	// 	}
	// 	else if (incomingByte == 'b') {
	// 		blueled_state = !blueled_state;
	// 		digitalWrite(blueled, blueled_state);
	// 	}
	// 	else if (incomingByte == 'y') {
	// 		yellowled_state = !yellowled_state;
	// 		digitalWrite(yellowled, yellowled_state);
	// 	}
	// }

	// if DMP initialization during setup failed, don't try to do anything
	if (!dmpReady)
	{
		return;
	}

	mpu_get(); // Take nearly 2.4ms to run, once there is an interrupt
	hmc_get();

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


	// sonar_get();

	if (alt_average_timer >= 30) {
		alt_average_timer = 0;
		ms5611_alt_average();
	}


	if (rc_time_count >= 10) {
		rc_time_count = 0;
		motor_adjust();
		rc_adjust();
		ms5611_adjust();
		motor_output();
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
		Serial_alt();
		// Serial_heading();
		// Serial2.println("testing...");
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


#if 0

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
	byte asBytes[24];    // us take the byte array
	float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array


// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float pitch
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

	// read the bytes sent from Processing
	int index = 0;
	byte Auto_Man = -1;
	byte Direct_Reverse = -1;
	while (Serial.available() && index < 26)
	{
		if (index == 0) Auto_Man = Serial.read();
		else if (index == 1) Direct_Reverse = Serial.read();
		else foo.asBytes[index - 2] = Serial.read();
		index++;
	}

	// if the information we got was in the correct format,
	// read it into the system
	if (index == 26  && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
	{
		pitch = double(foo.asFloat[0]);
		//Input=double(foo.asFloat[1]);       // * the user has the ability to send the
		//   value of "Input"  in most cases (as
		//   in this one) this is not needed.
		if (Auto_Man == 0)                    // * only change the output if we are in
		{	//   manual mode.  otherwise we'll get an
			pitch_angle_pid_output = double(foo.asFloat[2]);    //   output blip, then the controller will
		}                                     //   overwrite.

		double p, i, d;                       // * read in and set the controller tunings
		p = double(foo.asFloat[3]);           //
		i = double(foo.asFloat[4]);           //
		d = double(foo.asFloat[5]);           //
		pitch_angle.SetTunings(p, i, d);            //

		if (Auto_Man == 0) pitch_angle.SetMode(MANUAL); // * set the controller mode
		else pitch_angle.SetMode(AUTOMATIC);             //

		if (Direct_Reverse == 0) pitch_angle.SetControllerDirection(DIRECT); // * set the controller Direction
		else pitch_angle.SetControllerDirection(REVERSE);          //
	}
	Serial.flush();                         // * clear any random data from the serial buffer
}
#endif

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
	Serial.print("Raw gyro rotation ax, ay, az [value/deg/s]: "); Serial.print("\t\t");
	Serial.print(GyroX); Serial.print("\t");
	Serial.print(GyroY); Serial.print("\t");
	Serial.println(GyroZ);
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
	Serial.print(pitch_angle_pid_output);
	Serial.print(" ");
	// Serial.print(roll);
	// Serial.print(" ");
	// Serial.print(float(throttle3) / 1000);
	// Serial.print(" ");
	// Serial.print(GyroX);
	// Serial.print(" ");
	// Serial.print(rpy_yaw);
	// Serial.print(" ");
	Serial.print(ch6);
	Serial.print(" ");
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
	Serial.print(roll_angle_pid_output);
	Serial.print(" ");
	Serial.print(float(throttle2) / 1000);
	Serial.print(" ");
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
	Serial.print("PID ");
	Serial.print(AcceX);
	Serial.print(" ");
	Serial.print(AcceY);
	Serial.print(" ");
	Serial.print(AcceZ);
	Serial.print(" ");
	Serial.print(Ax);
	Serial.print(" ");
	Serial.print(Ay);
	Serial.print(" ");
	Serial.print(Az);
	Serial.print(" ");
	Serial.println("END");
}

void Serial_alt() {
	Serial.print("BARO"); Serial.print(' ');
	Serial.print(throttle1); Serial.print(' ');
	Serial.print(on_ch5); Serial.print(' ');
	Serial.print(ch6); Serial.print(' ');
	Serial.print(average_altitude); Serial.print(' ');
	// Serial.print(height_baro_pid_output); Serial.print(' ');
	Serial.print(_sonar_altitude); Serial.print(' ');
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
