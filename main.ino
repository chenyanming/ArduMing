#include <SPI.h>
#include "config.h"

// #define DEBUG_MODE 1

boolean redled_state = true;
boolean yellowled_state = true;
boolean blueled_state = true;

#define SPI0_MISO_PIN 50
#define SPI0_MOSI_PIN 51
#define SPI0_SCK_PIN  52


// INTERRUPT FROM MPU-6000 DETECTION ROUTINE
volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}





void setup()
{
	Serial.begin(115200);	// Set the baud rate to 115200
	// pinMode(led, OUTPUT);	// Set built-in LED to OUPUT mode
	pinMode(blueled, OUTPUT);	// Set built-in LED to OUPUT mode
	pinMode(yellowled, OUTPUT);	// Set built-in LED to OUPUT mode
	pinMode(redled, OUTPUT);	// Set built-in LED to OUPUT mode


	digitalWrite(blueled, HIGH);
	digitalWrite(yellowled, HIGH);
	digitalWrite(redled, HIGH);

	Serial.println("################ Start ################");

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
	Serial.println("Initializing SPI Protocol...");
	SPI.begin();  // start the SPI library
	// SPCR = _BV(SPE) | _BV(MSTR);
	SPI.setClockDivider(SPI_CLOCK_DIV16); // Arduino Mega2560 board runs on 16 MHz: 16 MHz / SPI_CLOCK_DIV16 = 1 MHz
	// 1 MHz is the maximum SPI clock frequency according to the MPU-6000 Product Specification
	SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
	SPI.setDataMode(SPI_MODE3); // latched on rising edge, transitioned on falling edge, active low  ****why****
	Serial.println("SPI Protocol initializing done.");
	Serial.println("################# 1. #################");
	delay(100);

	/**
	 * DMP initialization
	 */
	// write & verify dmpMemory, dmpConfig and dmpUpdates into the DMP, and make all kinds of other settings
	// !!! this is the main routine to make the DMP work, and get the quaternion out of the FIFO !!!
	Serial.println("Initializing Digital Motion Processor (DMP)...");
	unsigned char devStatus; // return status after each device operation (0 = success, !0 = error)
	devStatus = dmpInitialize();
	Serial.println(devStatus);
	Serial.println("OK END initial");

	// make sure it worked: dmpInitialize() returns a 0 in devStatus if so
	if (devStatus == 0)	{
		// now that it's ready, turn on the DMP
		Serial.print("Enabling DMP... ");
		// SPIwriteBit(0x6A, 7, true, ChipSelPin1); // USER_CTRL_DMP_EN
		spi_SetBits(ChipSelPin1, 0x6A, (1 << 7));


		// Serial.println("done.");

		// enable Arduino interrupt detection, this will execute dmpDataReady whenever there is an interrupt,
		// independing on what this sketch is doing at that moment
		// http://arduino.cc/en/Reference/AttachInterrupt
		Serial.print("Enabling interrupt detection... ");
		// attachInterrupt(interrupt, function, mode) specifies a function to call when an external interrupt occurs
		// Arduino Mega2560 INT6 / PE6 (input) connected to MPU-6000 INT pin 12 (output)
		attachInterrupt(6, dmpDataReady, RISING); // the 0 points correctly to INT0 / D2
		// -> if there is an interrupt from MPU-6000 to ATMEGA328, boolean mpuInterrupt will be made true
		// byte mpuIntStatus = SPIread(0x3A, ChipSelPin1); // by reading INT_STATUS register, all interrupts are cleared
		byte mpuIntStatus = spi_readReg(ChipSelPin1, 0x3A); // by reading INT_STATUS register, all interrupts are cleared
		// Serial.println("done.");

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;
		Serial.println("DMP ready! Waiting for first data from MPU-6000...");
		Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
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

	Serial.println("############# LOOP... ##############");
} // End of Setup

void loop() {
	int rev, incomingByte;
	if (rev = Serial.available()) {
		incomingByte = Serial.read();
		if (incomingByte == 'r') {
			redled_state = !redled_state;
			digitalWrite(redled, redled_state);
		}
		else if (incomingByte == 'b') {
			blueled_state = !blueled_state;
			digitalWrite(blueled, blueled_state);
		}
		else if (incomingByte == 'y') {
			yellowled_state = !yellowled_state;
			digitalWrite(yellowled, yellowled_state);
		}
	}

	// if DMP initialization during setup failed, don't try to do anything
	if (!dmpReady)
	{
		return;
	}

	// wait for MPU interrupt or extra packet(s) available
	// INFO: if there is an interrupt send from the MPU-6000 to the ATmega328P (the "Arduino" processor),
	//       boolean variable "mpuInterrupt" will be made "true" (see explanation in void setup() )
	while ((mpuInterrupt == false) && (fifoCount < packetSize))
	{
		// do nothing until mpuInterrupt = true or fifoCount >= 42
	}

	// there has just been an interrupt, so reset the interrupt flag, then get INT_STATUS byte
	mpuInterrupt = false;
	// byte mpuIntStatus = SPIread(0x3A, ChipSelPin1); // by reading INT_STATUS register, all interrupts are cleared
	byte mpuIntStatus = spi_readReg(ChipSelPin1, 0x3A); // by reading INT_STATUS register, all interrupts are cleared
	/**
	 * led : to digital pin 13, built-in LED for Arduino Mega 2560
	 * CS - to digital pin 53  (SS pin)
	 * SDI - to digital pin 51 (MOSI pin)
	 * CLK - to digital pin 52 (SCK pin) *
	 */
	const int led = 13;
	const int ChipSelPin1 = 53;

	// get current FIFO count
	fifoCount = getFIFOCount(ChipSelPin1);

	// check for FIFO overflow (this should never happen unless our code is too inefficient or DEBUG output delays code too much)
	if ((mpuIntStatus & 0x10) || fifoCount == 1008)
		// mpuIntStatus & 0x10 checks register 0x3A for FIFO_OFLOW_INT
		// the size of the FIFO buffer is 1024 bytes, but max. set to 1008 so after 24 packets of 42 bytes
		// the FIFO is reset, otherwise the last FIFO reading before reaching 1024 contains only 16 bytes
		// and can/will produces output value miscalculations
	{
		// reset so we can continue cleanly
		//SPIwriteBit(0x6A, 6, false, ChipSelPin1); // FIFO_EN = 0 = disable
		// SPIwriteBit(0x6A, 2, true, ChipSelPin1); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
		spi_SetBits(ChipSelPin1, 0x6A, (1 << 2));
		//SPIwriteBit(0x6A, 6, true, ChipSelPin1); // FIFO_EN = 1 = enable

		// digitalWrite(BLUE_LED_PIN, HIGH); // shows FIFO overflow without disturbing output with message
		DEBUG_PRINTLN("FIFO overflow! FIFO resetted to continue cleanly.");
	}

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	else if (mpuIntStatus & 0x02)
		// mpuIntStatus & 0x02 checks register 0x3A for (undocumented) DMP_INT
	{

		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = getFIFOCount(ChipSelPin1);

		// digitalWrite(BLUE_LED_PIN, LOW); // LED off again now that FIFO overflow is resolved

		// read a packet from FIFO
		// SPIreadBytes(0x74, packetSize, fifoBuffer, ChipSelPin1);
		spi_readBytes(ChipSelPin1, 0x74, packetSize, fifoBuffer);

		// verify contents of fifoBuffer before use:
// # ifdef DEBUG
// 		for (byte n = 0 ; n < packetSize; n ++)
// 		{
// 			Serial.print("\tfifoBuffer["); Serial.print(n); Serial.print("]\t: "); Serial.println(fifoBuffer[n], HEX);
// 		}
// # endif

		// track FIFO count here in case there is more than one packet (each of 42 bytes) available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount = fifoCount - packetSize;

// ============================================================================================== //
// >>>>>>>>> - from here the 42 FIFO bytes from the MPU-6000 can be used to generate output >>>>>>>>
// >>>>>>>>> - this would be the place to add your own code into                            >>>>>>>>
// >>>>>>>>> - of course all the normal MPU-6000 registers can be used here too             >>>>>>>>
// ============================================================================================== //

		// get the quaternion values from the FIFO - needed for Euler and roll/pitch/yaw angle calculations
		int raw_q_w = ((fifoBuffer[0] << 8)  + fifoBuffer[1]);  // W
		int raw_q_x = ((fifoBuffer[4] << 8)  + fifoBuffer[5]);  // X
		int raw_q_y = ((fifoBuffer[8] << 8)  + fifoBuffer[9]);  // Y
		int raw_q_z = ((fifoBuffer[12] << 8) + fifoBuffer[13]); // Z
		float q_w = raw_q_w / 16384.0f;
		float q_x = raw_q_x / 16384.0f;
		float q_y = raw_q_y / 16384.0f;
		float q_z = raw_q_z / 16384.0f;

#ifdef OUTPUT_RAW_ACCEL
		// print accelerometer values from fifoBuffer
		int AcceX = ((fifoBuffer[28] << 8) + fifoBuffer[29]);
		int AcceY = ((fifoBuffer[32] << 8) + fifoBuffer[33]);
		int AcceZ = ((fifoBuffer[36] << 8) + fifoBuffer[37]);
		Serial.print("Raw acceleration ax, ay, az [8192 = 1 g]: "); Serial.print("\t\t");
		Serial.print  (AcceX); Serial.print("\t");
		Serial.print  (AcceY); Serial.print("\t");
		Serial.println(AcceZ);
#endif

#ifdef OUTPUT_RAW_ACCEL_G
		// same as OUTPUT_RAW_ACCEL but recalculated to g-force values
		int AcceX = ((fifoBuffer[28] << 8) + fifoBuffer[29]);
		int AcceY = ((fifoBuffer[32] << 8) + fifoBuffer[33]);
		int AcceZ = ((fifoBuffer[36] << 8) + fifoBuffer[37]);
		float Ax = AcceX / 8192.0f; // calculate g-value
		float Ay = AcceY / 8192.0f; // calculate g-value
		float Az = AcceZ / 8192.0f; // calculate g-value
		Serial.print("Raw acceleration ax, ay, az [g]: "); Serial.print("\t\t\t");
		Serial.print  (Ax, 3); Serial.print("\t");
		Serial.print  (Ay, 3); Serial.print("\t");
		Serial.println(Az, 3);
#endif

#ifdef OUTPUT_RAW_ANGLES
		// print calculated angles for roll and pitch from the raw acceleration components
		// (yaw is undetermined here, this needs the use of the quaternion - see further on)
		int AcceX = ((fifoBuffer[28] << 8) + fifoBuffer[29]);
		int AcceY = ((fifoBuffer[32] << 8) + fifoBuffer[33]);
		int AcceZ = ((fifoBuffer[36] << 8) + fifoBuffer[37]);
		// atan2 outputs the value of -pi to pi (radians) - see http://en.wikipedia.org/wiki/Atan2
		// We then convert it to 0 to 2 pi and then from radians to degrees - in the end it's 0 - 360 degrees
		float ADegX = (atan2(AcceY, AcceZ) + PI) * RAD_TO_DEG;
		float ADegY = (atan2(AcceX, AcceZ) + PI) * RAD_TO_DEG;
		Serial.print("Calculated angle from raw acceleration - roll, pitch and yaw [degrees]: ");
		Serial.print(ADegX); Serial.print("\t");
		Serial.print(ADegY); Serial.print("\t");
		Serial.println("undetermined");
#endif

#ifdef OUTPUT_RAW_GYRO
		// print gyroscope values from fifoBuffer
		int GyroX = ((fifoBuffer[16] << 8) + fifoBuffer[17]);
		int GyroY = ((fifoBuffer[20] << 8) + fifoBuffer[21]);
		int GyroZ = ((fifoBuffer[24] << 8) + fifoBuffer[25]);
		Serial.print("Raw gyro rotation ax, ay, az [value/deg/s]: "); Serial.print("\t\t");
		Serial.print(GyroX); Serial.print("\t");
		Serial.print(GyroY); Serial.print("\t");
		Serial.println(GyroZ);
#endif

#ifdef OUTPUT_TEMPERATURE
		// print calculated temperature from standard registers (not available in fifoBuffer)
		// the chip temperature may be used for correction of the temperature sensitivity of the
		// accelerometer and the gyroscope - not done in this sketch
		byte Temp_Out_H = spi_readReg(ChipSelPin1, 0x41);
		byte Temp_Out_L = spi_readReg(ChipSelPin1, 0x42);
		int TemperatureRaw = Temp_Out_H << 8 | Temp_Out_L;
		float Temperature = (TemperatureRaw / 340.00) + 36.53; // formula from datasheet chapter 4.19
		Serial.print("Chip temperature for corrections [deg. Celsius]: ");
		Serial.println(Temperature, 2);
		delay(1000);
#endif

#ifdef OUTPUT_READABLE_QUATERNION
		Serial.print("Quaternion qw, qx, qy, qz [-1 to +1]: "); Serial.print("\t");
		Serial.print  (q_w); Serial.print("\t");
		Serial.print  (q_x); Serial.print("\t");
		Serial.print  (q_y); Serial.print("\t");
		Serial.println(q_z);
#endif

#ifdef OUTPUT_READABLE_EULER
		// calculate Euler angles
		// http://en.wikipedia.org/wiki/Atan2
		// http://en.wikipedia.org/wiki/Sine (-> Inverse)
		float euler_x = atan2((2 * q_y * q_z) - (2 * q_w * q_x), (2 * q_w * q_w) + (2 * q_z * q_z) - 1); // phi
		float euler_y = -asin((2 * q_x * q_z) + (2 * q_w * q_y));                                        // theta
		float euler_z = atan2((2 * q_x * q_y) - (2 * q_w * q_z), (2 * q_w * q_w) + (2 * q_x * q_x) - 1); // psi
		euler_x = euler_x * 180 / M_PI; // angle in degrees -180 to +180
		euler_y = euler_y * 180 / M_PI; // angle in degrees
		euler_z = euler_z * 180 / M_PI; // angle in degrees -180 to +180
		Serial.print("Euler angles x, y, z [degrees]: ");
		Serial.print(euler_x); Serial.print("\t");
		Serial.print(euler_y); Serial.print("\t");
		Serial.print(euler_z); Serial.println();
		delay(1000);
#endif

#ifdef OUTPUT_READABLE_ROLLPITCHYAW
		// display Euler angles in degrees

		// dmpGetGravity
		float grav_x = 2 * ((q_x * q_z) - (q_w * q_y));
		float grav_y = 2 * ((q_w * q_x) + (q_y * q_z));
		float grav_z = (q_w * q_w) - (q_x * q_x) - (q_y * q_y) + (q_z * q_z);

		// roll: (tilt left/right, about X axis)
		float rpy_rol = atan(grav_y / (sqrt((grav_x * grav_x) + (grav_z * grav_z))));
		// pitch: (nose up/down, about Y axis)
		float rpy_pit = atan(grav_x / (sqrt((grav_y * grav_y) + (grav_z * grav_z))));
		// yaw: (rotate around Z axis)
		float rpy_yaw = atan2((2 * q_x * q_y) - (2 * q_w * q_z), (2 * q_w * q_w) + (2 * q_x * q_x) - 1);

		Serial.print("Roll, pitch and yaw angles [degrees]: ");
		Serial.print(rpy_rol * 180 / M_PI); Serial.print("\t");
		Serial.print(rpy_pit * 180 / M_PI); Serial.print("\t");
		Serial.print(rpy_yaw * 180 / M_PI); Serial.println();
#endif

#ifdef OUTPUT_TEAPOT
		// display quaternion values in InvenSense Teapot demo format:
		teapotPacket[2] = fifoBuffer[0];
		teapotPacket[3] = fifoBuffer[1];
		teapotPacket[4] = fifoBuffer[4];
		teapotPacket[5] = fifoBuffer[5];
		teapotPacket[6] = fifoBuffer[8];
		teapotPacket[7] = fifoBuffer[9];
		teapotPacket[8] = fifoBuffer[12];
		teapotPacket[9] = fifoBuffer[13];
		Serial.write(teapotPacket, 14);
		teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

// ============================================================================================== //
// >>>>>>>>> - this would normally be the end of adding your own code into this sketch          >>>>
// >>>>>>>>> - end of using the 42 FIFO bytes from the MPU-6000 to generate output              >>>>
// >>>>>>>>> - after blinking the red LED, the main loop starts again (and again, and again...) >>>>
// ============================================================================================== //

	} // End of loop()
}
