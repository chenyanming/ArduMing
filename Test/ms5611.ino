#include <spi.h>

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

int i = 0;
float altitude[10];

// void ms5611_setup() {
void setup()
{
	ms5611_reset();
	delay(10);
	ms5611_prom_read();
	delay(10);
}

void loop() {

	ms5611_get_temperature(ChipSelPin2, CMD_CONVERT_D2_OSR4096);
	ms5611_get_pressure(ChipSelPin2, CMD_CONVERT_D1_OSR4096);
	altitude[i++] = get_altitude();
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
	SETUP = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_RESET);
	CRC = ms5611_spi_read16(ChipSelPin2, CMD_MS5611_PROM_CRC);
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
	return read_value;
}

void mS5611_get_temperature(int ChipSelPin, unsigned char OSR_Temp)
{
	ms5611_spi_write(ChipSelPin, CMD_CONVERT_D2_OSR4096);
	D2_Temp = ms5611_spi_read_adc();
	delay(10);
	dT = D2_Temp - (((ulong)C5) << 8);
	Temperature = 2000 + dT * ((ulong)C6) / 8388608;
}


void ms5611_get_pressure(unsigned char OSR_Pres)
{
	SPI_B0_Strobe(CMD_CONVERT_D1_OSR4096);
	D1_Pres = ms5611_spi_read_adc();
	delay(10);
	OFF = (ulong)C2 * 65536 + ((ulong)C4 * dT) / 128;
	SENS = (ulong)C1 * 32768 + ((ulong)C3 * dT) / 256;

	if (Temperature < 2000)
	{
		// second order temperature compensation when under 20 degrees C
		TEMP2 = (dT * dT) / 0x80000000;
		Aux = Temperature * Temperature;
		OFF2 = 2.5 * Aux;
		SENS2 = 1.25 * Aux;
		Temperature = Temperature - TEMP2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}
	Pressure = (D1_Pres * SENS / 2097152 - OFF) / 32768;
}

float get_altitude()
{
	float tmp_float, Altitude;
	tmp_float = (Pressure / 101325.0);
	tmp_float = pow(tmp_float, 0.190295);
	Altitude = 44330 * (1.0 - tmp_float);
	return (Altitude);
}

long ms5611_spi_read_adc()
{
	unsigned char byteH, byteM, byteL;
	long return_value;

	ms5611_spi_write(ChipSelPin2, MS5611_ADC);
	byteH = ms5611_spi_write(ChipSelPin2, 0);
	byteM = ms5611_spi_write(ChipSelPin2, 0);
	byteL = ms5611_spi_write(ChipSelPin2, 0);

	return_value = (((long)byteH) << 16) | (((long)byteM) << 8) | (byteL);
	return (return_value);
}
