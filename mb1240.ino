#include "Kalman.h"
int analogPin = 0;
float _sonar_altitude = 0;
float tmp_sonar_altitude[10];
float _sonar_mode_altitude = 0;
int alt_mode_count = 0;
long sonar_current_time = 0, sonar_previous_time = 0;

Kalman kalmanS;
uint32_t timerS;
float kal_sonar = 0;
float pre_kal_sonar = 0;
float Vel_sonar = 0;
void sonar_setup()
{
	// Kalman init
	kalmanS.setAngle(0);
	timerS = micros();

}

void sonar_get() {
	_sonar_altitude = analogRead(analogPin);    // read the input pin
	// delay(100);

	// Kalman Fiter
	double dtS = (double)(micros() - timerS) / 1000000;
	timerS = micros();

	kal_sonar = kalmanS.getAngle(_sonar_altitude, AcceZ_W, dtS);

	// Velocity of sonar
	sonar_current_time = millis();

	Vel_sonar = (constrain((kal_sonar - pre_kal_sonar), -20, 20) * 1000)  /  (sonar_current_time - sonar_previous_time);

	// LPF
	static int sonar_offset = 0;
	sonar_offset -= sonar_offset / 8;
	sonar_offset += Vel_sonar;
	Vel_sonar = sonar_offset / 8;

	// Dead band
	if (abs(Vel_sonar) < 3) {
		Vel_sonar = 0;
	}

	// Vel_sonar = kalmanS.getAngle(Vel_sonar, AcceZ_W, dtS);
	pre_kal_sonar = kal_sonar;
	sonar_previous_time = sonar_current_time;
}

void sonar_mode() {
	float temp = 0;
	if (alt_mode_count < 5) {
		tmp_sonar_altitude[alt_mode_count] = _sonar_altitude;
		alt_mode_count++;
	}


	if (alt_mode_count == 4) {
		alt_mode_count = 0;
		for (int i = 0; i < 5; i++) {
			for (int j = i + 1; j < 5; j++) {
				if (tmp_sonar_altitude[i] > tmp_sonar_altitude[j]) {
					temp = tmp_sonar_altitude[i];
					tmp_sonar_altitude[i] = tmp_sonar_altitude[j];
					tmp_sonar_altitude[j] = temp;
				}
			}
		}
		/*
		double dtS = (double)(micros() - timerS) / 1000000;
		timerS = micros();
		// _sonar_mode_altitude = (tmp_sonar_altitude[4] + tmp_sonar_altitude[5]) / 2;
		_sonar_mode_altitude = tmp_sonar_altitude[2];

		kal_sonar = kalmanS.getAngle(_sonar_mode_altitude, AcceZ_W, dtS);

		// for (int i = 0; i < 10; i++) {
		// 	average_altitude += tmp_altitude[i];
		// }
		// average_altitude /= 10;
		sonar_current_time = millis();
		Vel_sonar = ((kal_sonar - pre_kal_sonar)*1000)  /  (sonar_current_time - sonar_previous_time);
		// LPF
		static int sonar_offset = 0;
		sonar_offset -= sonar_offset / 8;
		sonar_offset += Vel_sonar;
		Vel_sonar = sonar_offset / 8;
		// Dead band
		if(abs(Vel_sonar) == 1){
			Vel_sonar = 0;
		}

		// Vel_sonar = kalmanS.getAngle(Vel_sonar, AcceZ_W, dtS);
		pre_kal_sonar = kal_sonar;
		sonar_previous_time = sonar_current_time;
		*/

	}
}