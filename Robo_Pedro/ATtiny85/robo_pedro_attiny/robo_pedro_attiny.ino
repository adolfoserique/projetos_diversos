#define REC PB1             //REC is defined as PB1(pin 6)
#define PLAYE PB2           //PLAYE is defined as PB2(pin 7)
#define REC_TIME 5000       //Record time is 5s

void setup() {
  DDRB |= 1 << REC;         //Set REC as an input
  DDRB |= 1 << PLAYE;       //Set PLAYE as an input
  PORTB &= ~(1 << REC);     //Set REC LOW
  PORTB &= ~(1 << PLAYE);   //Set PLAYE LOW
  delay(500);               //Delay 0.5s
}

void loop() {
  PORTB |= (1 << REC);      //Set REC HIGH
  delay(REC_TIME + 150);    //Record time
  PORTB &= ~(1 << REC);     //Set REC LOW
  delay(100);               //Delay 0.1s
  PORTB |= (1 << PLAYE);    //Set PLAYE HIGH
  delay(100);               //Delay 0.1s
  PORTB &= ~(1 << PLAYE);   //Set PLAYE LOW
  delay(REC_TIME);          //Play recorded time
}
