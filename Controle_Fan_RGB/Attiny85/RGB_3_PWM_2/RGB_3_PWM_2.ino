/* Three PWM Outputs */

// ATtiny85 outputs
#define Red PB0
#define Green PB1
#define Blue PB4

volatile uint8_t* Port[] = {&OCR0A, &OCR0B, &OCR1B};

void setup() {
  //Set colors as outputs
  DDRB |= 1 << Red;
  DDRB |= 1 << Green;
  DDRB |= 1 << Blue;
  // Configure counter/timer0 for fast PWM on PB0 and PB1
  TCCR0A = 3<<COM0A0 | 3<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 3<<CS00; // Optional; already set
  // Configure counter/timer1 for fast PWM on PB4
  GTCCR = 1<<PWM1B | 3<<COM1B0;
  TCCR1 = 3<<COM1A0 | 7<<CS10;
}

// Sets colour Red=0 Green=1 Blue=2 to specified intensity 0 (off) to 255 (max)
void SetColour (int colour, int intensity) {
  *Port[colour] = 255 - intensity;
}

void loop() {
  for (int i=-255; i <= 254; i++) {
    OCR0A = abs(i);
    OCR0B = 255-abs(i);
    OCR1B = 255-abs(i);
    delay(10);
  }
}
