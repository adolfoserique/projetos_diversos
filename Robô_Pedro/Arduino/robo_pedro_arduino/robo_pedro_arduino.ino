#define REC 5                 //REC is defined as digital pin 5
#define PLAYE 7               //PLAYE is defined as digital pin 7
#define REC_TIME 5000         //Record time is 5s

void setup() {
  pinMode(REC, OUTPUT);       //Set REC as an input
  pinMode(PLAYE, OUTPUT);     //Set PLAYE as an input
  digitalWrite(REC, LOW);     //Set REC LOW
  digitalWrite(PLAYE, LOW);   //Set PLAYE LOW
  delay(500);                 //Delay 0.5s
}

void loop() {
  digitalWrite(REC, HIGH);    //Set REC HIGH
  delay(REC_TIME);            //Record time
  digitalWrite(REC, LOW);     //Set REC LOW
  delay(200);                 //Delay 0.2s
  digitalWrite(PLAYE, HIGH);  //Set PLAYE HIGH
  delay(100);                 //Delay 0.1s
  digitalWrite(PLAYE, LOW);   //Set PLAYE LOW
  delay(REC_TIME);            //Play recorded time
}
