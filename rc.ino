long roll, pitch, throttle, yaw;
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
  Serial.println("pulses are sampled while LED is lit");
  Serial.print( precision); // report duration of each tick in microseconds
  Serial.println(" microseconds per tick");
}

// this loop prints the number of pulses in the last second, showing min
// and max pulse widths
void rc_get() {

  // digitalWrite(ledPin, LOW);
  // delay(gateSamplePeriod);
  // digitalWrite(ledPin, HIGH);
  gate = 1; // enable sampling

#ifdef PPM_SUM_capture
  // Serial.println("") ;
  // Serial.println("Durations are: ") ;
  roll = map(results[2], 1240, 3060, -90, 90);
  pitch = map(results[4], 1705, 3250, -180, 180);
  throttle = map(results[6], 1645, 3220, 1000, 1800);
  yaw = map(results[8], 1275, 3060, -180, 180);
  // ch5 = map(results[10], 1090, 1911, -180, 180);

  // Serial.print(pitch);
  // Serial.print(' ');
  // Serial.print(roll);
  // Serial.print(' ');
  // Serial.print(throttle);
  // Serial.print(' ');
  // Serial.print(yaw);
  // Serial.print(' ');
  // Serial.print(ch5);
  // Serial.print(' ');

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