/**
 * Timer2: 1ms overflow, control output
 */
#include <SPI.h>
#include "config.h"

boolean redled_state = true;
boolean yellowled_state = true;
boolean blueled_state = true;

#define SPI0_MISO_PIN 50
#define SPI0_MOSI_PIN 51
#define SPI0_SCK_PIN  52

unsigned int toggle = 0;  //used to keep the state of the LED
unsigned int count = 0;   //used to keep count of how many interrupts were fired
unsigned int mpu_time_count = 0; //The MPU running time
unsigned int rc_time_count = 0;	//The RC running time
unsigned int system_time_count = 0;	//The System running time

boolean euler_output = false;
boolean gyro_output = false;
boolean throttle_output = false;
boolean rc_output = false;

float rpy_rol = 0;
float rpy_pit = 0;
float rpy_yaw = 0;

int GyroX = 0;
int GyroY = 0;
int GyroZ = 0;

unsigned long serialTime; //this will help us know when to talk with processing
extern float pitch_angle_pid_output;
extern PID pitch_angle;

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) {
	mpu_time_count++;
	rc_time_count++;
	system_time_count++;
	count++;               //Increments the interrupt counter
	if (count >= 500) {

#ifdef EULER_OUTPUT
		euler_output = true; // output euler enable
#endif
#ifdef GYRO_OUTPUT
		gyro_output = true; // output gyro enable
#endif
#ifdef THROTTLE_OUTPUT
		throttle_output = true; // output euler enable
#endif
#ifdef RC_OUTPUT
		rc_output = true; // output remote control enable
#endif

		// digitalWrite(27, toggle);
		// toggle = !toggle;    //toggles the LED state
		count = 0;           //Resets the interrupt counter
	}


	TCNT2 = 130;           //Reset Timer to 130 out of 255, 255-130 = 125 counts = 125*8us = 1ms
	TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};


void setup()
{
	Serial.begin(115200);	// Set the baud rate to 115200

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


	// Serial.println("############# LOOP... ##############");
	system_time_count = 0; // Start to count the system time.
	mpu_time_count = 0;
	rc_time_count = 0;
} // End of Setup

void loop() {


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

	// wait for MPU interrupt or extra packet(s) available
	// INFO: if there is an interrupt send from the MPU-6000 to the ATmega328P (the "Arduino" processor),
	//       boolean variable "mpuInterrupt" will be made "true" (see explanation in void setup() )
	// // while ((mpuInterrupt == false) && (fifoCount < packetSize))
	// {
	// do nothing until mpuInterrupt = true or fifoCount >= 42
	// }

	if (mpu_time_count >= 4) {
		//By decreasing the time counter I can estimate the code run time. After it output 4ms(here) - the desired run time, we can comment the print code
		// Serial.print("The MPU updating time: ");
		// Serial.println(mpu_time_count);
		mpu_time_count = 0;
		mpu_get();
		motor_adjust();
	}

	if (rc_time_count >= 10) {
		// Serial.print("The Remote Control updating time: ");
		// Serial.println(rc_time_count);
		rc_time_count = 0;
		rc_adjust();
		rc_get();
	}
	motor_output();

	//send-receive with processing if it's time
	if (millis() > serialTime)
	{
		// SerialReceive();
		// SerialSend();
		Serial_rc();
		serialTime += 100;
	}

} // End of loop()



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

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
	Serial.print("PID ");
	Serial.print(pitch);
	Serial.print(" ");
	Serial.print(rpy_pit);
	Serial.print(" ");
	Serial.print(pitch_angle_pid_output);
	Serial.print(" ");
	Serial.print(pitch_angle.GetKp());
	Serial.print(" ");
	Serial.print(pitch_angle.GetKi());
	Serial.print(" ");
	Serial.print(pitch_angle.GetKd());
	Serial.print(" ");
	if (pitch_angle.GetMode() == AUTOMATIC) Serial.print("Automatic");
	else Serial.print("Manual");
	Serial.print(" ");
	if (pitch_angle.GetDirection() == DIRECT) Serial.println("Direct");
	else Serial.println("Reverse");
}


void Serial_rc() {

		Serial.print("Getting the remote control pitch, roll, yaw and throttle adjusting value : ");
		Serial.print(pitch); Serial.print('\t');
		Serial.print(roll); Serial.print('\t');
		Serial.print(yaw); Serial.print('\t');
		Serial.print(throttle); Serial.print('\t');
		Serial.println(ch5);
}
