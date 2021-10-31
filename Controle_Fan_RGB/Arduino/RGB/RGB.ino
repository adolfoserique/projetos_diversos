#define red 1
#define green 0
#define blue 4
int t = 500;
 
void setup()
{
 
}
 
void loop()
{
  analogWrite(red, 255);
  delay(t);
  analogWrite(red, 0);
  delay(t);
  analogWrite(green, 255);
  delay(t);
  analogWrite(green, 0);
  delay(t);
  analogWrite(blue, 255);
  delay(t);
  analogWrite(blue, 0);
  delay(t);
}
