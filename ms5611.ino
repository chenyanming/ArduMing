#include <SPI.h>
// #include "Kalman.h"

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
extern unsigned long D1_Pres, D2_Temp; // store pressure and temperature value
float dT, _Temperature, _Pressure;
double OFF, SENS;
float _ground_pressure = 0;
float _ground_temperature = 0;
float _last_altitude, _altitude, _det_altitude;

// ms5611 adjust
unsigned int ms5611_adjust_count = 100;
unsigned char ms5611_adjust_bits = 0;
boolean ms5611_adjust_bit = false;

// ms5611 convert
// volatile unsigned char D1_timer = 0;
// volatile unsigned char D2_timer = 0;
// boolean D1_ready = false;
// boolean D2_ready = false;
// boolean turn_ready = true;
volatile unsigned char convert_timer = 0;
boolean convert_ready = false;
boolean convert_finish = false;
boolean convert_toggle = false;
boolean convert_ready_toggle = false;

// Kalman
// Kalman kalmanH;
// uint32_t timer_h;
// float kal_alt = 0;
// extern float Az;

// ms5611 average alt
unsigned char alt_average_count = 0;
float average_altitude = 0;
float tmp_altitude[10];


float baro_altitude = 0;

void ms5611_setup() {
	// Serial.begin(115200);	// Set the baud rate to 115200

	// pinMode(ChipSelPin1, OUTPUT);
	// digitalWrite(ChipSelPin1, HIGH);

	// pinMode(ChipSelPin2, OUTPUT);  //--- Configure the chip select pin as output ---//
	// digitalWrite(ChipSelPin2, HIGH);

	/**
	 * SPI Settings
	 */
	// Serial.println("Initializing SPI Protocol...");
	// SPI.begin();  // start the SPI library
	// SPCR = _BV(SPE) | _BV(MSTR);
	// SPI.setClockDivider(SPI_CLOCK_DIV128); // Arduino Mega2560 board runs on 16 MHz: 16 MHz / SPI_CLOCK_DIV16 = 1 MHz
	// 1 MHz is the maximum SPI clock frequency according to the MPU-6000 Product Specification
	// SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
	// SPI.setDataMode(SPI_MODE3);

	// delay(100);

	ms5611_reset();
	delay(10);
	ms5611_prom_read();
	delay(10);

	// Kalman init
	// kalmanH.setAngle(0);
	// timer_h = micros();
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

void ms5611_adjust()
{
	// When adjust count time ends, start to calculate the gound temp and gound press
	if (ms5611_adjust_count > 0) {
		ms5611_adjust_count--;
		if (ms5611_adjust_count == 0) {
			ms5611_adjust_bits++;
			switch (ms5611_adjust_bits) {
			case 1:
				ms5611_adjust_count = 100;
				ms5611_get_temperature();
				ms5611_get_pressure();
				_ground_temperature      = _Temperature;
				_ground_pressure         = _Pressure;
				break;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				// now average over 8 values for the ground pressure and
				// temperature settings
				ms5611_adjust_count = 100;
				ms5611_get_temperature();
				ms5611_get_pressure();
				_ground_temperature = (_ground_temperature * 0.8f) + (_Temperature * 0.2f);
				_ground_pressure = (_ground_pressure * 0.8f) + (_Pressure * 0.2f);
				break;
			case 10:
				ms5611_adjust_bit = true;
				break;
			default:
				break;
			}

		}
	}
}

float ms5611_get_temperature() {
	dT = D2_Temp - (((unsigned long)C5) << 8);
	// Temperature = 2000 + dT * ((unsigned long)C6) / 8388608;
	_Temperature = (dT * C6) / 8388608;
	// callers want the temperature in 0.1C units
	_Temperature = _Temperature / 10;
	return _Temperature;
}


float ms5611_get_pressure() {
	OFF = C2 * 65536.0f + (C4 * dT) / 128;
	SENS = C1 * 32768.0f + (C3 * dT) / 256;

	if (_Temperature < 2000)
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
	// _Temperature = _Temperature + 2000;
	return _Pressure;
}

float ms5611_get_altitude()
{
	// float tmp_float, Altitude;
	// tmp_float = (Pressure / 101325.0);
	// tmp_float = pow(tmp_float, 0.190295);
	// Altitude = 44330 * (1.0 - tmp_float);
	// return Altitude;

	float scaling, temp;


	// this has no filtering of the pressure values, use a separate
	// filter if you want a smoothed value. The AHRS driver wants
	// unsmoothed values
	// ms5611_get_temperature();
	scaling = (float)_ground_pressure / (float)_Pressure;
	temp = ((float)_ground_temperature) + 273.15f;
	_altitude = log(scaling) * temp * 29.271267f;

	// _det_altitude = _altitude - _last_altitude;
	// _last_altitude = _altitude;

	return _altitude;
	// return _det_altitude;
}

void ms5611_convert() {

	if (convert_ready == true) {
		if (convert_finish == true) { // can not use convert_timer to detect directly, because it is volatile. We may not detect it.
			convert_ready_toggle = !convert_ready_toggle;
			if (convert_ready_toggle == true)
				D2_Temp = ms5611_spi_read_adc(ChipSelPin2);
			else
				D1_Pres = ms5611_spi_read_adc(ChipSelPin2);
			convert_finish = false;
			convert_ready = false;
		}
	}
	else {
		if (convert_timer == 0) { // we can detect the zero value, because when timer decrease to 0, it can not change
			convert_toggle = !convert_toggle;
			if (convert_toggle == true)
				ms5611_spi_write(ChipSelPin2, CMD_CONVERT_D2_OSR4096);
			else
				ms5611_spi_write(ChipSelPin2, CMD_CONVERT_D1_OSR4096);
			convert_timer = 20;
			convert_ready = true;
		}
	}

}


void ms5611_get() {
	float temp, pressure, altitude;

	if (ms5611_adjust_bit == true) {

		temp = ms5611_get_temperature();
		pressure = ms5611_get_pressure();
		altitude = ms5611_get_altitude();
		// scale to cm
		baro_altitude = altitude * 100;

		// LPF
		static int altoffset = 0;
		altoffset -= altoffset / 100;
		altoffset += baro_altitude;
		baro_altitude = altoffset / 100;

		// double dt = (double)(micros() - timer_h) / 1000000;
		// timer_h = micros();
		// kal_alt = kalmanH.getAngle(altitude, Az, dt);
		// kal_alt = kalmanH.getAngle(_altitude, 1, dt);
		// if ((altitude != 0) && (state == 0))	{
		// 	state = 1;
		// 	global = altitude;
		// }
		// altitude = altitude - global;
	}

}

void ms5611_alt_average() {
	if (alt_average_count < 10) {
		tmp_altitude[alt_average_count] = _altitude;
		alt_average_count++;
	}
	if (alt_average_count == 9) {
		alt_average_count = 0;
		for (int i = 0; i < 10; i++) {
			average_altitude += tmp_altitude[i];
		}
		average_altitude /= 10;
	}
}



