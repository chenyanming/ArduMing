/**
 * Timer2: 1ms overflow, control output
 */
#include <SPI.h>
#include "config.h"

unsigned int rc_time_count = 0;	//The RC running time

unsigned long serialTime; //this will help us know when to talk with processing

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) {
	rc_time_count++;

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
		rc_time_count = 0;
		rc_get();
	}
	motor_output();

	if (millis() > serialTime)
	{
		Serial_rc();
		serialTime += 100;
	}

} // End of loop()



void Serial_rc() {

	Serial.print("Getting the remote control throttle value : ");
	Serial.println(throttle); 
}
