// Librarys
#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"

// Global constants
#define mean_times      2000                  // Number of values used to calculate the mean
#define time_pressed    1000                  // Time to keed button press to cancel
#define pot_read        0                     // Analog pin for the potentiometer
#define led             2                     // Led pin
#define button_select   1                     // Selection button pin
#define deb_delay       50                    // Debounce time (ms)
#define thermo_SO       5                     // MAX6675 SO pin
#define thermo_CS       3                     // MAX6675 CS pin
#define thermo_SCK      4                     // MAX6675 SCK pin
#define resistor        6                     // Resistor relay pin
#define pwm_res         8                     // PWM resolution (bits) 2^8 = 256
#define pwm_freq        30                    // PWM frequency (Hz)
#define pwm_channel     0                     // PWM channel
#define refresh_rate    200                   // Program refresh rate (ms)
#define kp              2.5                   // Proporcional error constant
#define ki              0.06                  // Integration error constant
#define kd              0.8                   // Differentiation error constant
#define tau             0.02                  // Low pass filter constant
#define pid_i_lim_max   5                     // Maximum limit of the integrator
#define pid_i_lim_min   -5                    // Minimum limit of the integrator 
#define pid_lim_max     18                    // Maximum limit of PID calculation
#define pid_lim_min     0                     // Minimum limit of PID calculation
#define SCREEN_WIDTH    128                   // OLED display width, in pixels
#define SCREEN_HEIGHT   64                    // OLED display height, in pixels
#define OLED_RESET      0                     // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS  0x3C                  //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// Global variables
float temp_select_cal = 0;                    // Temperature (Celsius) for calculations selected by user
float temp_cal = 0;                           // Temperature (Celsius) for calculations provided by sensor
float last_temp_cal = 0;                      // Last Temperature (Celsius) provided by sensor
float pid_error = 0;                          // PID error
float last_pid_error = 0;                     // Last PID error
float pid_p = 0;                              // Proporcional error
float pid_i = 0;                              // Integration error
float pid_d = 0;                              // Differentiation error
float pid_out = 0;                            // Sum of PID error parts
int temp_select_display = 0;                  // Temperature (Celsius) for display selected by user
int temp_display = 0;                         // Temperature (Celsius) for display provided by sensor
int select_mode = 0;                          // Mode selection
int led_count = 0;                            // Led counter blink
bool led_state = LOW;                         // Led state
volatile bool button_select_state = HIGH;     // selection button state
volatile unsigned long count_time = 0;        // Inicial time counter 
volatile unsigned long count_time_delta = 0;  // Delta time counter

// Functions declaration
float set_temp(int analog_port);
bool button_select_press(int button, bool button_state);
void temp_select_mode(void);
void ramp_mode(void);
void pid_mode(void);
void ARDUINO_ISR_ATTR debounce_select();

// External devices initialization
MAX6675 thermocouple(thermo_SCK, thermo_CS, thermo_SO);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Setup routine
void setup() {

  // Initialize serial communication at 115200 bits per second
  //Serial.begin(115200);

  // initialize oled display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  
  delay(100);
  display.clearDisplay();
  display.setTextSize(2);  
  display.setTextColor(WHITE,BLACK);
  display.display();
  delay(100);

  // Start interruption on button_select
  attachInterrupt(button_select, debounce_select, FALLING);

  // Set pin, frequency, bit resolution and channel for the PWM
  ledcAttachChannel(resistor, pwm_freq, pwm_res, pot_read);

  // Initialize led as an OUTPUT and LOW
  pinMode(led, OUTPUT);
  digitalWrite(led, led_state);

  //  Initialize selection button as an input and enable the internal pull-up resistor
  pinMode(button_select, INPUT_PULLUP);

  // Wait for all devices stabilize
  delay(refresh_rate * 3);

} // End of setup routine

// Loop routine
void loop(){

  switch (select_mode){

    case 0:

      temp_select_mode();
      break;

    case 1:

      ramp_mode();
      break;

    case 3:

      pid_mode();
      break;

  }

} // End of loop routine

// Interruption select button routine
void ARDUINO_ISR_ATTR debounce_select(){

  count_time = millis();

  while(count_time_delta < deb_delay){

    count_time_delta = millis();
    count_time_delta -= count_time;

  }

  button_select_state = digitalRead(button_select);
    
} // End interruption

// #1 - Temperature selection
float set_temp(int analog_port){

  float temp;
  int i;

  while(i <= mean_times){

    // Read the input on analog pin A0:
    int sensorValue = analogRead(analog_port);
    // Convert the analog reading (which goes from 0 - 4095) to a voltage (0 - 3.3V):
    float voltage = sensorValue * (3.3 / 4095);
    // Map the voltage across the temperature range using the following formula:
    // output = (input_value - input_min) * (output_max - output_min) / (input_max - input_min) + output_min
    float value_c = (voltage - 0) * (0 - 100) / (3.3 - 0) + 100;
    // Sum for mean calculation
    temp = temp + value_c;

    i++;

  }
  // Return the mean calculation
  return temp / mean_times;

} // End of function #1

// #2 - Check selection button state 
bool button_select_press(int button, bool button_state){

  count_time = 0;
  count_time_delta = 0;

  if(button_state == LOW){

    button_state = LOW;

    count_time = millis();

    while(button_state == LOW){

      button_state = digitalRead(button);
      count_time_delta = millis();
      count_time_delta -= count_time;

    }

    count_time = 0;

    button_state = digitalRead(button);

  }

  return button_state;

} // End of function #2

// #3 - Temperature selection mode
void temp_select_mode(void){

  count_time_delta = 0;

  // Read the state of the selection button with debounce
  button_select_state = button_select_press(button_select, button_select_state);

  // Set temperature from the potentiometer + correction
  temp_select_cal = set_temp(pot_read) + 0.6;

  // Temperature limits
  if (temp_select_cal > 100){

      temp_select_cal = 100;

  } 
  else if (temp_select_cal <= 0.61){

    temp_select_cal -= 0.6;

  }
  
  temp_select_display = round(temp_select_cal);

  //Clear dislpay and print text
  display.clearDisplay();  
  display.setCursor(0,0);           
  display.println("select");
  display.println("Temp:");
  display.setCursor(37,50);
  display.print(temp_select_display);
  display.print((char)247);
  display.print("C");
  
  // Display the image
  display.display();

  // Check if selection button has been pressed and switch to PID mode
  if(!button_select_state){

    // Turn on led and resistor
    digitalWrite(led, !led_state);
    ledcWrite(resistor, 0);
    led_count = 0;

    // Change mode and force button to HIGH to prevent error
    select_mode = 1;
    button_select_state = HIGH;
    count_time_delta = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Starting");
    display.println("Ramp");
    display.println("Mode...");

    // Display the image
    display.display();

    delay(refresh_rate * 10);

  }

} // End of function #3

// #4 - Ramp up mode
void ramp_mode(void){

  count_time_delta = 0;

  // Read the state of the selection button with debounce
  button_select_state = button_select_press(button_select, button_select_state);

  delay(refresh_rate);

  // Start interruption on button_select
  //attachInterrupt(button_select, isr, FALLING);

  // Get temperature from sensor
  temp_cal = thermocouple.readCelsius();
  temp_display = round(temp_cal);

  //Clear dislpay and print text
  display.clearDisplay();  
  display.setCursor(0,0);           
  display.println("Ramp Mode");
  display.print("S: ");
  display.print(temp_select_display);
  display.print((char)247);
  display.println("C");
  display.println();
  display.print("C: ");
  display.print(temp_display);
  display.print((char)247);
  display.print("C");
  
  // Display the image
  display.display();

  // Check if the target temperature has been reached and switch to PID mode.
  if(temp_cal > (temp_select_cal - 4)){

    // Turn off led and resistor
    digitalWrite(led, led_state);
    ledcWrite(resistor, 255);
    led_count = 0;

    // Change mode and force button to HIGH to prevent error
    select_mode = 3;
    button_select_state = HIGH;
    count_time_delta = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Starting");
    display.println("PID");
    display.print("Mode...");
  
    // Display the image
    display.display();

    delay(refresh_rate * 100);

  }

  // Cancel ramp up mode and back to temperature selection mode
  if(count_time_delta >= (time_pressed - deb_delay)){

    // Turn off led and resistor
    digitalWrite(led, led_state);
    ledcWrite(resistor, 255);

    // Change mode and force button to HIGH to prevent error
    select_mode = 0;
    button_select_state = HIGH;
    count_time_delta = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Canceling");
    display.println("Ramp");
    display.print("Mode...");
  
    // Display the image
    display.display();

    delay(refresh_rate * 10);

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Starting");
    display.println("selection");
    display.print("Mode...");
  
    // Display the image
    display.display();

    delay(refresh_rate * 10);

  }

} // End of function #4

// #5 - PID mode
void pid_mode(void){

  count_time_delta = 0;

  // Read the state of the selection button with debounce
  button_select_state = button_select_press(button_select, button_select_state);

  led_count++;

  delay(refresh_rate);

  // Get temperature from sensor
  temp_cal = thermocouple.readCelsius();
  temp_display = round(temp_cal);

  // Current PID error
  pid_error = temp_select_cal - temp_cal;

  // Proporcional error calculation
  pid_p = kp * pid_error;

  // Integration error calculation
  pid_i = pid_i + 0.5 * ki * refresh_rate * (pid_error + last_pid_error);

  // Anti-wind-up via integrator clamping
  if (pid_i > pid_i_lim_max){

      pid_i = pid_i_lim_max;

  } 
  else if (pid_i < pid_i_lim_min){

    pid_i = pid_i_lim_min;

  }

  // Differentiation error calculation (band-limited differentiator)
  pid_d = - (2  * kd * (temp_cal - last_temp_cal)	+ (2 * tau - refresh_rate) * pid_d) / (2 * tau + refresh_rate);

  // PID output calculation and mapping to PWM limits
  pid_out = pid_p + pid_i + pid_d;
  pid_out = (pid_out - pid_lim_min) * (0 - 255) / (pid_lim_max - pid_lim_min) + 255;
  pid_out = round(pid_out);

  // Apply PID output limits
  if (pid_out > 255){

    pid_out = 255;

  } 
  else if (pid_out < 0){

    pid_out = 0;

  }

  // Store error and measurement for later use
  last_pid_error = pid_error;
  last_temp_cal = temp_cal;

  // Set PWM for the resistor relay
  ledcWrite(resistor, pid_out);

  //Clear dislpay and print text
  display.clearDisplay();  
  display.setCursor(0,0);           
  display.println("PID Mode");
  display.print("S: ");
  display.print(temp_select_display);
  display.print((char)247);
  display.println("C");
  display.println();
  display.print("C: ");
  display.print(temp_display);
  display.print((char)247);
  display.print("C");
  
  // Display the image
  display.display();


  // Led blick routine
  if(led_count < 10){

    digitalWrite(led, led_state);

  }
  else if(led_count < 20 && led_count >= 10){

    digitalWrite(led, !led_state);

  }
  else{

    digitalWrite(led, led_state);
    led_count = 0;

  }

  // Cancel PID mode and back to temperature selection mode
  if(count_time_delta >= (time_pressed - deb_delay)){

    // Turn off led and resistor
    digitalWrite(led, led_state);
    ledcWrite(resistor, 255);
    led_count = 0;

    // Change mode and force button to HIGH to prevent error
    select_mode = 0;
    button_select_state = HIGH;
    count_time_delta = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Canceling");
    display.println("PID");
    display.print("Mode...");
  
    // Display the image
    display.display();

    delay(refresh_rate * 10);

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Starting");
    display.println("selection");
    display.print("Mode...");

    // Display the image
    display.display();

    delay(refresh_rate * 10);

  }

} // End of function #5
