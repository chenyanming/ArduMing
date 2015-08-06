/**
 * Use input capture to detect and measure the Digital Pin 48
 * Timer5: Input Capture -> Digital Pin 48
 * The model of Remote Control: WFT06X-A
 */
#include "Kalman.h"
const int ROLL_MIN = 1308;
const int ROLL_MAX = 2938;
const int PITCH_MIN = 1515;
const int PITCH_MAX = 3083;
const int YAW_MIN = 1275;
const int YAW_MAX = 3060;
const int THROTTLE_MIN = 1500;
const int THROTTLE_MAX = 2900;
const int CH6_MIN = 1210;
const int CH6_MAX = 3242;

const int ROLL_MAP_MIN = -20;
const int ROLL_MAP_MAX = 20;
const int PITCH_MAP_MIN = -20;
const int PITCH_MAP_MAX = 20;
const int YAW_MAP_MIN = -45;
const int YAW_MAP_MAX = 45;

const int MAX_SIGNAL = 1900;
const int MIN_SIGNAL = 1000;

const int CH6_MAP_MIN = 0;
const int CH6_MAP_MAX = 20;

float roll, pitch, throttle, yaw, ch5, ch6;
boolean on_ch5 = false; // suppose that the ch5 is off
float max_yaw, min_yaw, yaw_bottle, last_yaw_bottle;
float yaw_tmp[5];
const int inputCapturePin = 48; // input pin fixed to internal Timer
// const int ledPin = 25;
static const int prescale = 8; // prescale factor (each tick 0.5 us @16MHz)
static const byte prescaleBits = B010; // see Table 18-1 or data sheet
// calculate time per counter tick in ns
static const long precision = (1000000 / (F_CPU / 1000)) * prescale ;
static const int numberOfEntries = 20; // the max number of pulses to measure
static const int gateSamplePeriod = 200; // the sample period in milliseconds
static volatile byte index = 0; // index to the stored readings
static volatile byte gate = 0; // 0 disables capture, 1 enables
static volatile byte save = 0; // 0 disables save captured data, 1 enables
static volatile unsigned int results[numberOfEntries]; // note this is 16 bit value
static volatile double millis_duration[numberOfEntries]; // note this is 16 bit value

static int rc_adjust_count = 1000;
static int pitch_adjust = 0;
static int roll_adjust = 0;
static int yaw_adjust = 0;
static boolean rc_adjust_bit = false;

Kalman kalman_rc_yaw;
uint32_t rc_timer;
extern volatile unsigned int min_yaw_timer;
extern boolean test_yaw_bit;
float kal_rc_yaw = 0;

int ii = 0;


#define PPM_SUM_capture

#ifdef PPM_SUM_capture
/* ICR interrupt vector */
ISR(TIMER5_CAPT_vect) {
  TCNT5 = 0; // reset the counter
  if (gate)
  {
    if (ICR5 > 20000)
      save = 1;
    if (save == 1) {
      results[index] = ICR5;
      index++;
    }
    if (index > 17) {
      index = 0;
      save = 0;
    }
  }
  TCCR5B ^= _BV(ICES5); // toggle bit to trigger on the other edge
}

#else

/**
 * Capture all pluses
 */
ISR(TIMER5_CAPT_vect) {
  TCNT5 = 0; // reset the counter
  if (gate)
  {
    if ( index != 0 || bitRead(TCCR5B , ICES5) == true) // wait for rising edge
    { // falling edge was detected
      if (index < numberOfEntries)
      {
        results[index] = ICR5; // save the input capture value
        index++;
      }
    }
  }
  TCCR5B ^= _BV(ICES5); // toggle bit to trigger on the other edge
}
#endif

void rc_setup() {
  // Serial.begin(115200);
  // pinMode(ledPin, OUTPUT);
  pinMode(inputCapturePin, INPUT); // ICP pin (digital pin 8 on Arduino) as input
  TCCR5A = 0 ; // Normal counting mode
  TCCR5B = prescaleBits ; // set prescale bits
  TCCR5B |= _BV(ICES5); // enable input capture
  bitSet(TIMSK5, ICIE5); // enable input capture interrupt for timer 1
  // Serial.println("pulses are sampled while LED is lit");
  // Serial.print( precision); // report duration of each tick in microseconds
  // Serial.println(" microseconds per tick");
  kalman_rc_yaw.setAngle(0);
  rc_timer = micros();
}

int rc_adjust() {
  int roll_tmp, pitch_tmp, yaw_tmp;
  if (rc_adjust_count > 0) {
    rc_adjust_count--;

    gate = 1; // enable sampling
    roll_tmp = map(results[2], ROLL_MIN, ROLL_MAX, ROLL_MAP_MIN, ROLL_MAP_MAX);
    pitch_tmp = map(results[4], PITCH_MIN, PITCH_MAX, PITCH_MAP_MIN, PITCH_MAP_MAX);
    yaw_tmp = map(results[8], YAW_MIN, YAW_MAX, YAW_MAP_MIN, YAW_MAP_MAX);
    pitch_adjust = -pitch_tmp;
    roll_adjust = -roll_tmp;
    yaw_adjust = -yaw_tmp;
    if (rc_adjust_count == 0) {
      // Serial.println("RC Adjust Value, Get.");
      rc_adjust_bit = true;
      return 1;
    }
  }
  return 0;
}

void rc_get() {

  if (rc_adjust_bit == true) {

    gate = 1; // enable sampling

#ifdef PPM_SUM_capture

    /*
     ** Read Remote Control Values and store
     */
    roll = map(results[2], ROLL_MIN, ROLL_MAX, ROLL_MAP_MIN, ROLL_MAP_MAX) + roll_adjust;// Although the range of roll or pitch is form -180 to 180, the drone can not reach large degrees.
    pitch = map(results[4], PITCH_MIN, PITCH_MAX, PITCH_MAP_MIN, PITCH_MAP_MAX) + pitch_adjust;
    throttle = map(results[6], THROTTLE_MIN, THROTTLE_MAX, MIN_SIGNAL, MAX_SIGNAL);
    throttle = constrain(throttle, MIN_SIGNAL, MAX_SIGNAL);
    ch5 = results[10];
    if (ch5 > 2000)
      on_ch5 = true; // ch5 is on (3222)
    else
      on_ch5 = false; // ch5 is off (1111)
    ch6 = mapfloat(results[12], CH6_MIN, CH6_MAX, CH6_MAP_MIN, CH6_MAP_MAX);
    ch6 = constrain(ch6, CH6_MAP_MIN, CH6_MAP_MAX);
    yaw = map(results[8], YAW_MIN, YAW_MAX, YAW_MAP_MIN, YAW_MAP_MAX) + yaw_adjust;

    /*
     ** Kalman Filter on Yaw Stick Value
     */
    double dt1 = (double)(micros() - rc_timer) / 1000000;
    rc_timer = micros();
    kal_rc_yaw  = kalman_rc_yaw.getAngle(yaw, 1, dt1);

    /*
     ** Filter out the unstable value, it is vital for later capture
     */
    if ((kal_rc_yaw <= 5) && (kal_rc_yaw >= -5)) {
      kal_rc_yaw = 0;
    }

    /*
     ** Capture the maximal Yaw Stick Value
     */
    yaw_bottle = kal_rc_yaw;
    if (kal_rc_yaw >= 0) {
      if (yaw_bottle > last_yaw_bottle) {
        if (yaw_bottle > max_yaw)
          max_yaw = yaw_bottle;
      }
      else {
        if (last_yaw_bottle > max_yaw)
          max_yaw = last_yaw_bottle;
      }
    }
    else {
      if (yaw_bottle < last_yaw_bottle) {
        if (yaw_bottle < max_yaw)
          max_yaw = yaw_bottle;
      }
      else {
        if (last_yaw_bottle < max_yaw)
          max_yaw = last_yaw_bottle;
      }
    }

    last_yaw_bottle = yaw_bottle;

    /**
     * start the timer
     */
    if (min_yaw_timer == 0) { // Before set the timer, make sure the counter is equal to zero!
      min_yaw_timer = 100;
    }

    /*
     ** fixing small errors of signals from remote controller
     */
    if (abs(roll) <= 3)
      roll = 0;
    if (abs(pitch) <= 3)
      pitch = 0;

  }

#else
    if (index > 0)
    {
      Serial.println("") ;
      Serial.println("Durations are: ") ;
      for ( byte i = 0; i < numberOfEntries; i++)
      {
        long duration;
        duration = results[i]; // pulse duration in nanoseconds
        if (duration > 0)
        {
          Serial.print(duration);
          Serial.print(' ');
        }
      }
      index = 0;

    }
#endif

  else {
    roll = 0;
    pitch = 0;
    throttle = MIN_SIGNAL;
    yaw = 0;

  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
