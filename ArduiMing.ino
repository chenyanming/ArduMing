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

	TCNT2 = 130;           //Reset Timer to 130 out of 255, 255-130 = 125 counts = 125*8us = 1ms
	TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};


void setup()
{
	Serial.begin(115200);	// Set the baud rate to 115200

	rc_setup();
	motor_setup();


	/**
	 * Setup Timer2 to fire every 1ms
	 */
	TCCR2B = 0x00;        //Disbale Timer2 while we set it up
	TCNT2  = 130;         //Reset Timer Count to 130 out of 255
	TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
	TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
	TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prmotoraler set to 128, 16MHz/128 = 125KHz = 8us per count


} // End of Setup

void loop() {


	if (rc_time_count >= 10) {
		// Serial.print("The Remote Control updating time: ");
		// Serial.println(rc_time_count);
		rc_time_count = 0;
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



void Serial_rc() {

	Serial.print("Getting the remote control pitch, roll, yaw and throttle adjusting value : ");
	Serial.print(pitch); Serial.print('\t');
	Serial.print(roll); Serial.print('\t');
	Serial.print(yaw); Serial.print('\t');
	Serial.print(throttle); Serial.print('\t');
	Serial.println(ch5);
}
