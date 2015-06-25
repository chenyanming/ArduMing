/**
 * Use input capture to detect and measure the Digital Pin 48
 * Timer5: Input Capture -> Digital Pin 48
 */
float roll, pitch, throttle, yaw, ch5;
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

static int rc_adjust_count = 100;
static int pitch_adjust = 0;
static int roll_adjust = 0;
static int yaw_adjust = 0;
static boolean rc_adjust_bit = false;


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
}

int rc_adjust() {
  int roll_tmp, pitch_tmp, yaw_tmp;
  if (rc_adjust_count > 0) {
    rc_adjust_count--;

    gate = 1; // enable sampling
    roll_tmp = map(results[2], 1240, 3060, -30, 30);
    pitch_tmp = map(results[4], 1705, 3250, -30, 30);
    yaw_tmp = map(results[8], 1275, 3060, -180, 180);

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

// this loop prints the number of pulses in the last second, showing min
// and max pulse widths
void rc_get() {

  if (rc_adjust_bit == true) {

    // digitalWrite(ledPin, LOW);
    // delay(gateSamplePeriod);
    // digitalWrite(ledPin, HIGH);
    gate = 1; // enable sampling

#ifdef PPM_SUM_capture
    // Serial.println("") ;
    // Serial.println("Durations are: ") ;
    // Although the range of roll or pitch is form -180 to 180, the drone can not reach large degrees.
    roll = map(results[2], 1240, 3060, -30, 30) + roll_adjust;
    pitch = map(results[4], 1705, 3250, -30, 30) + pitch_adjust;
    throttle = map(results[6], 1500, 2900, MIN_SIGNAL, MAX_SIGNAL);
    throttle = constrain(throttle, MIN_SIGNAL, MAX_SIGNAL);//start from non-zero to finish the calibration
    yaw = map(results[8], 1275, 3060, -180, 180) + yaw_adjust;
    ch5 = results[10];

    //fixing small errors of signals from remote controller
    if (abs(roll) <= 3)
      roll = 0;
    if (abs(pitch) <= 3)
      pitch = 0;
    if (abs(yaw) <= 3)
      yaw = 0;

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
  }
  else {
    roll = 0;
    pitch = 0;
    throttle = MIN_SIGNAL;
    yaw = 0;

  }
}