int analogPin = 0;
unsigned int _sonar_altitude = 0;
// void setup()
// {
// 	Serial.begin(115200);	// Set the baud rate to 115200

// }

void sonar_get() {
	_sonar_altitude = analogRead(analogPin);    // read the input pin
	// Serial.println(_sonar_altitude);             // debug value
	// delay(100);
}