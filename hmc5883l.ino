#include <Wire.h>
unsigned char HMC5883L_ADDRESS            = 0x1E; // this device only has one address
unsigned char HMC5883L_DEFAULT_ADDRESS    = 0x1E;

unsigned char HMC5883L_RA_CONFIG_A        = 0x00;
unsigned char HMC5883L_RA_CONFIG_B        = 0x01;
unsigned char HMC5883L_RA_MODE            = 0x02;
unsigned char HMC5883L_RA_DATAX_H         = 0x03;
unsigned char HMC5883L_RA_DATAX_L         = 0x04;
unsigned char HMC5883L_RA_DATAZ_H         = 0x05;
unsigned char HMC5883L_RA_DATAZ_L         = 0x06;
unsigned char HMC5883L_RA_DATAY_H         = 0x07;
unsigned char HMC5883L_RA_DATAY_L         = 0x08;
unsigned char HMC5883L_RA_STATUS          = 0x09;
unsigned char HMC5883L_RA_ID_A            = 0x0A;
unsigned char HMC5883L_RA_ID_B            = 0x0B;
unsigned char HMC5883L_RA_ID_C            = 0x0C;

unsigned char HMC5883L_CRA_AVERAGE_BIT    = 6;
unsigned char HMC5883L_CRA_AVERAGE_LENGTH = 2;
unsigned char HMC5883L_CRA_RATE_BIT       = 4;
unsigned char HMC5883L_CRA_RATE_LENGTH    = 3;
unsigned char HMC5883L_CRA_BIAS_BIT       = 1;
unsigned char HMC5883L_CRA_BIAS_LENGTH    = 2;

unsigned char HMC5883L_AVERAGING_1        = 0x00;
unsigned char HMC5883L_AVERAGING_2        = 0x01;
unsigned char HMC5883L_AVERAGING_4        = 0x02;
unsigned char HMC5883L_AVERAGING_8        = 0x03;

unsigned char HMC5883L_RATE_0P75          = 0x00;
unsigned char HMC5883L_RATE_1P5           = 0x01;
unsigned char HMC5883L_RATE_3             = 0x02;
unsigned char HMC5883L_RATE_7P5           = 0x03;
unsigned char HMC5883L_RATE_15            = 0x04;
unsigned char HMC5883L_RATE_30            = 0x05;
unsigned char HMC5883L_RATE_75            = 0x06;

unsigned char HMC5883L_BIAS_NORMAL        = 0x00;
unsigned char HMC5883L_BIAS_POSITIVE      = 0x01;
unsigned char HMC5883L_BIAS_NEGATIVE      = 0x02;

unsigned char HMC5883L_CRB_GAIN_BIT       = 7;
unsigned char HMC5883L_CRB_GAIN_LENGTH    = 3;

unsigned char HMC5883L_GAIN_1370          = 0x00;
unsigned char HMC5883L_GAIN_1090          = 0x01;
unsigned char HMC5883L_GAIN_820           = 0x02;
unsigned char HMC5883L_GAIN_660           = 0x03;
unsigned char HMC5883L_GAIN_440           = 0x04;
unsigned char HMC5883L_GAIN_390           = 0x05;
unsigned char HMC5883L_GAIN_330           = 0x06;
unsigned char HMC5883L_GAIN_220           = 0x07;

unsigned char HMC5883L_MODEREG_BIT        = 1;
unsigned char HMC5883L_MODEREG_LENGTH     = 2;

unsigned char HMC5883L_MODE_CONTINUOUS    = 0x00;
unsigned char HMC5883L_MODE_SINGLE        = 0x01;
unsigned char HMC5883L_MODE_IDLE          = 0x02;

unsigned char HMC5883L_STATUS_LOCK_BIT    = 1;
unsigned char HMC5883L_STATUS_READY_BIT   = 0;


unsigned char buffer[6];
unsigned char mode;

int mx, my, mz, mx_r, my_r;
float _heading;

boolean hmc_setup() {
// void setup() {
	Wire.begin();

	// Serial.begin(115200);

	// write CONFIG_A register
	hmc_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
	              (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
	              (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
	              (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

	// write CONFIG_B register
	hmc_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B,
	              HMC5883L_GAIN_1090 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));

	// write MODE register
	hmc_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE,
	              HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mode = HMC5883L_MODE_SINGLE;

	// Serial.println("Testing device connections...");
	// Serial.println(hmc_testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
	return hmc_testConnection();

}

void hmc_get() {
// void loop() {
	// read raw heading measurements from device
	hmc_getHeading(&mx, &my, &mz);

	// delay(100);
	// display tab-separated gyro x/y/z values
	// Serial.print("mag:\t");
	// Serial.print(mx); Serial.print("\t");
	// Serial.print(my); Serial.print("\t");
	// Serial.print(mz); Serial.print("\t");

// To calculate heading in degrees. 0 degree indicates North
	// float _heading = atan2(my, mx);
	mx_r = mx * cos(kal_pit) + my * sin(kal_pit) * sin(kal_rol) - mz * cos(kal_rol) * sin(kal_pit);
	my_r = my * cos(kal_rol) + mz * sin(kal_rol);


	_heading = atan2(my_r, mx_r);
	// _heading = atan2(my, mx);
	if (_heading < 0)
		_heading += 2 * M_PI;
	_heading = _heading * 180 / M_PI;
	// Serial.print("heading:\t");
	// Serial.println(heading * 180 / M_PI);


}

boolean hmc_testConnection() {
	if (hmc_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_A, 3, buffer, 100) == 3) {
		return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
	}
	return false;
}

void hmc_getHeading(int *x, int *y, int *z) {
	hmc_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer, 100);
	if (mode == HMC5883L_MODE_SINGLE)
		hmc_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE,
		              HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	*x = (((int)buffer[0]) << 8) | buffer[1];
	*y = (((int)buffer[4]) << 8) | buffer[5];
	*z = (((int)buffer[2]) << 8) | buffer[3];
}

void hmc_writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char data) {
	Wire.beginTransmission(devAddr);
	Wire.write((unsigned char) regAddr); // send address
	Wire.write((unsigned char) data); // send data
	Wire.endTransmission();
}

void hmc_writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data) {
	Wire.beginTransmission(devAddr);
	Wire.write((unsigned char) regAddr); // send address
	for (unsigned char i = 0; i < length; i++) {
		Wire.write((unsigned char) data[i]); // send data
	}
	Wire.endTransmission();
}

char hmc_readBytes(unsigned char devAddr, unsigned char regAddr, unsigned char length, unsigned char *data, int timeout) {

	int8_t count = 0;
	uint32_t t1 = millis();

	// Arduino v1.0.1+, Wire library
	// Adds official support for repeated start condition, yay!

	// I2C/TWI subsystem uses internal buffer that breaks with large data requests
	// so if user requests more than BUFFER_LENGTH bytes, we have to do it in
	// smaller chunks instead of all at once
	for (unsigned char k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
		Wire.beginTransmission(devAddr);
		Wire.write(regAddr);
		Wire.endTransmission();
		Wire.beginTransmission(devAddr);
		Wire.requestFrom(devAddr, (unsigned char)min(length - k, BUFFER_LENGTH));

		for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
			data[count] = Wire.read();
		}
	}

	// check for timeout
	if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout

	return count;
}
