#include <SPI.h>
#include "Kalman.h"

#define MS5611_ADC     0x00
#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximun resolution

unsigned long SETUP, C1, C2, C3, C4, C5, C6, CRC; //store the 8 sets of data for PROM
unsigned long D1_Pres, D2_Temp; // store pressure and temperature value
float dT, _Temperature, _Pressure;
double OFF, SENS;
float _ground_pressure = 0;
float _ground_temperature = 0;
float _last_altitude, _altitude, _det_altitude;

// Kalman
Kalman kalmanH;
uint32_t timer_h;
float kal_alt = 0;

// int state = 0;
// float global;
// float TEMP2, Aux, OFF2, SENS2; // temperature CRC

// int i = 0;

const int ChipSelPin1 = 53;
const int ChipSelPin2 = 40;

// void ms5611_setup() {
void setup()
{
	Serial.begin(115200);	// Set the baud rate to 115200

	pinMode(ChipSelPin1, OUTPUT);
	digitalWrite(ChipSelPin1, HIGH);

	pinMode(ChipSelPin2, OUTPUT);  //--- Configure the chip select pin as output ---//
	digitalWrite(ChipSelPin2, HIGH);

	/**
	 * SPI Settings
	 */
	// Serial.println("Initializing SPI Protocol...");
	SPI.begin();  // start the SPI library
	// SPCR = _BV(SPE) | _BV(MSTR);
	SPI.setClockDivider(SPI_CLOCK_DIV16); // Arduino Mega2560 board runs on 16 MHz: 16 MHz / SPI_CLOCK_DIV16 = 1 MHz
	// 1 MHz is the maximum SPI clock frequency according to the MPU-6000 Product Specification
	SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
	SPI.setDataMode(SPI_MODE3);

	delay(100);

	ms5611_reset();
	delay(10);
	ms5611_prom_read();
	delay(10);

	ms5611_calibrate();

	// Kalman init
	kalmanH.setAngle(-0.3);
	timer_h = micros();
}


void loop() {
	float temp, pressure, altitude;

	temp = ms5611_get_temperature();
	pressure = ms5611_get_pressure();
	altitude = ms5611_get_altitude();


	double dt = (double)(micros() - timer_h) / 1000000;
	timer_h = micros();
	kal_alt = kalmanH.getAngle(altitude, 1, dt);
	// if ((altitude != 0) && (state == 0))	{
	// 	state = 1;
	// 	global = altitude;
	// }
	// altitude = altitude - global;

	Serial.print("BARO"); Serial.print(' ');
	Serial.print(kal_alt); Serial.print(' ');
	Serial.print(D1_Pres); Serial.print(' ');
	Serial.print(D2_Temp); Serial.print(' ');
	Serial.println("END");

	delay(100);
}

void ms5611_reset() {
	ms5611_spi_write(ChipSelPin2, CMD_MS5611_RESET);
}
void ms5611_prom_read() {
	C1 = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_C1);
	C2 = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_C2);
	C3 = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_C3);
	C4 = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_C4);
	C5 = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_C5);
	C6 = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_C6);
	// SETUP = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_RESET);
	// CRC = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_CRC);
}


void ms5611_spi_write(int ChipSelPin, unsigned char data) {
	byte dump;
	digitalWrite(ChipSelPin, LOW);
	dump = SPI.transfer(data);
	digitalWrite(ChipSelPin, HIGH);
}


unsigned char ms5611_spi_read(int ChipSelPin, unsigned char addr) {
	digitalWrite(ChipSelPin, LOW);
	byte dump = SPI.transfer(addr);
	byte read_value = SPI.transfer(0);
	digitalWrite(ChipSelPin, HIGH);
	return read_value;

}

unsigned int ms5611_spi_read16(int ChipSelPin, unsigned char addr) {
	digitalWrite(ChipSelPin, LOW);
	byte dump = SPI.transfer(addr);
	byte read_value_H = SPI.transfer(0);
	byte read_value_L = SPI.transfer(0);
	digitalWrite(ChipSelPin, HIGH);
	unsigned int return_value = ((unsigned int)read_value_H << 8) | (read_value_L);
	return return_value;
}

long ms5611_spi_read_adc(int ChipSelPin)
{
	unsigned char dump, byteH, byteM, byteL;
	long return_value;

	digitalWrite(ChipSelPin, LOW);
	dump = SPI.transfer(0);
	byteH = SPI.transfer(0);
	byteM = SPI.transfer(0);
	byteL = SPI.transfer(0);
	digitalWrite(ChipSelPin, HIGH);

	return_value = (((long)byteH) << 16) | (((long)byteM) << 8) | (byteL);
	return (return_value);
}

float ms5611_get_temperature()
{
	ms5611_spi_write(ChipSelPin2, CMD_CONVERT_D2_OSR4096);
	delay(10);
	D2_Temp = ms5611_spi_read_adc(ChipSelPin2);
	dT = D2_Temp - (((unsigned long)C5) << 8);
	// Temperature = 2000 + dT * ((unsigned long)C6) / 8388608;
	_Temperature = (dT * C6) / 8388608;
	// callers want the temperature in 0.1C units
	return _Temperature / 10;
}


float ms5611_get_pressure()
{
	ms5611_spi_write(ChipSelPin2, CMD_CONVERT_D1_OSR4096);
	delay(10);
	D1_Pres = ms5611_spi_read_adc(ChipSelPin2);
	OFF = C2 * 65536.0f + (C4 * dT) / 128;
	SENS = C1 * 32768.0f + (C3 * dT) / 256;

	if (_Temperature < 0)
	{
		// second order temperature compensation when under 20 degrees C
		float TEMP2 = (dT * dT) / 0x80000000;
		float Aux = _Temperature * _Temperature;
		float OFF2 = 2.5f * Aux;
		float SENS2 = 1.25f * Aux;
		_Temperature = _Temperature - TEMP2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}
	_Pressure = (D1_Pres * SENS / 2097152 - OFF) / 32768;
	_Temperature = _Temperature + 2000;
	return _Pressure;
}

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void ms5611_calibrate()
{

	while (_ground_pressure == 0) {
		_ground_temperature      = ms5611_get_temperature();
		_ground_pressure         = ms5611_get_pressure();
	}

	// let the barometer settle for a full second after startup
	// the MS5611 reads quite a long way off for the first second,
	// leading to about 1m of error if we don't wait
	for (uint8_t i = 0; i < 10; i++) {
		_ground_temperature      = ms5611_get_temperature();
		_ground_pressure         = ms5611_get_pressure();
		delay(100);
	}


	// now average over 5 values for the ground pressure and
	// temperature settings
	for (uint16_t i = 0; i < 5; i++) {
		_ground_temperature = (_ground_temperature * 0.8f) + (ms5611_get_temperature() * 0.2f);
		_ground_pressure = (_ground_pressure * 0.8f) + (ms5611_get_pressure() * 0.2f);
		delay(100);
	}
}

float ms5611_get_altitude()
{
	// float tmp_float, Altitude;
	// tmp_float = (Pressure / 101325.0);
	// tmp_float = pow(tmp_float, 0.190295);
	// Altitude = 44330 * (1.0 - tmp_float);
	// return Altitude;

	float scaling, temp;

	// if (_last_altitude == _altitude) {
	// no new information
	// return _altitude;
	// }

	// this has no filtering of the pressure values, use a separate
	// filter if you want a smoothed value. The AHRS driver wants
	// unsmoothed values
	ms5611_get_temperature();
	scaling = (float)_ground_pressure / (float)ms5611_get_pressure();
	temp = ((float)_ground_temperature) + 273.15f;
	_altitude = log(scaling) * temp * 29.271267f;

	// _det_altitude = _altitude - _last_altitude;
	// _last_altitude = _altitude;

	return _altitude;
	// return _det_altitude;
}

