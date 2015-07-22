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



void rc_setup() {

  pinMode(inputCapturePin, INPUT); // ICP pin (digital pin 8 on Arduino) as input
  TCCR5A = 0 ; // Normal counting mode
  TCCR5B = prescaleBits ; // set prescale bits
  TCCR5B |= _BV(ICES5); // enable input capture
  bitSet(TIMSK5, ICIE5); // enable input capture interrupt for timer 1

}



// this loop prints the number of pulses in the last second, showing min
// and max pulse widths
void rc_get() {

    gate = 1; // enable sampling

    throttle = map(results[6], 1500, 2900, MIN_SIGNAL, MAX_SIGNAL);
    throttle = constrain(throttle, MIN_SIGNAL, MAX_SIGNAL);//start from non-zero to finish the calibration


}