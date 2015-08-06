int analogPin = 0;
int val = 0;
void setup()
{
	Serial.begin(115200);	// Set the baud rate to 115200

}

void loop()
{
	val = analogRead(analogPin);    // read the input pin
	Serial.println(val);             // debug value
	delay(100);
}
