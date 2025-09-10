// Librarys
#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"

// Global constants
#define button_cancel         0               // Cancel button pin
#define button_start          1               // Start button pin
#define button_cycle          2               // Cycle button pin
#define deb_delay             50              // Debounce time (ms)
#define time_pressed          1000            // Maximum time to keep the button pressed

// Global variables

// Functions declaration
bool debounce(int button, bool button_state, bool button_last_state, int debounce_delay);
bool define_state(int button, bool button_state, int pressed_time);
void ARDUINO_ISR_ATTR debounce_cancel();
void ARDUINO_ISR_ATTR debounce_start();
void ARDUINO_ISR_ATTR debounce_cycle();

// External devices initialization
volatile bool button_cancel_state = HIGH;
volatile bool button_start_state = HIGH;
volatile bool button_cycle_state = HIGH;
volatile unsigned long count_time = 0;          // Last time the output pin was toggled
volatile unsigned long count_time_delta = 0;

// Setup routine
void setup() {

  // Initialize serial communication at 115200 bits per second
  Serial.begin(115200);

  //  Initialize buttons as an input and enable the internal pull-up resistor
  pinMode(button_cancel, INPUT_PULLUP);
  pinMode(button_start, INPUT_PULLUP);
  pinMode(button_cycle, INPUT_PULLUP);

  // Start interruption on buttons
  attachInterrupt(button_cancel, debounce_cancel, FALLING);
  attachInterrupt(button_start, debounce_start, FALLING);
  attachInterrupt(button_cycle, debounce_cycle, FALLING);

} // End of setup routine


// Loop routine
void loop() {

  // Stop interruption on button_selec
  //detachInterrupt(button_selec);

  button_cancel_state = define_state(button_cancel, button_cancel_state, (time_pressed - deb_delay));

  if(count_time_delta > 0){

    Serial.print("O botao cancel ficou pressionado por ");
    Serial.print(count_time_delta + deb_delay);
    Serial.println(" ms");

    count_time_delta = 0;

    delay(200);

  }
  
  button_start_state = define_state(button_start, button_start_state, (time_pressed - deb_delay));

  if(count_time_delta > 0){

    Serial.print("O botao start ficou pressionado por ");
    Serial.print(count_time_delta + deb_delay);
    Serial.println(" ms");

    count_time_delta = 0;

    delay(200);


  }

  button_cycle_state = define_state(button_cycle, button_cycle_state, (time_pressed - deb_delay));

  if(count_time_delta > 0){

    Serial.print("O botao cycle ficou pressionado por ");
    Serial.print(count_time_delta + deb_delay);
    Serial.println(" ms");

    count_time_delta = 0;

    delay(200);

  }

  attachInterrupt(button_cancel, debounce_cancel, FALLING);
  attachInterrupt(button_start, debounce_start, FALLING);
  attachInterrupt(button_cycle, debounce_cycle, FALLING);

} // End of loop routine


// Interruption cancel button routine
void ARDUINO_ISR_ATTR debounce_cancel(){

  detachInterrupt(button_cancel);
  detachInterrupt(button_start);
  detachInterrupt(button_cycle);

  count_time_delta = 0;
  count_time = millis();

  while(count_time_delta < deb_delay){

    count_time_delta = millis();
    count_time_delta -= count_time;

  }

  button_cancel_state = digitalRead(button_cancel);

  count_time = 0;
  count_time_delta = 0;
    
} // End interruption

// Interruption start button routine
void ARDUINO_ISR_ATTR debounce_start(){

  detachInterrupt(button_cancel);
  detachInterrupt(button_start);
  detachInterrupt(button_cycle);

  count_time_delta = 0;
  count_time = millis();

  while(count_time_delta < deb_delay){

    count_time_delta = millis();
    count_time_delta -= count_time;

  }

  button_start_state = digitalRead(button_start);

  count_time = 0;
  count_time_delta = 0;
    
} // End interruption

// Interruption start button routine
void ARDUINO_ISR_ATTR debounce_cycle(){

  detachInterrupt(button_cancel);
  detachInterrupt(button_start);
  detachInterrupt(button_cycle);

  count_time_delta = 0;
  count_time = millis();

  while(count_time_delta < deb_delay){

    count_time_delta = millis();
    count_time_delta -= count_time;

  }

  button_cycle_state = digitalRead(button_cycle);

  count_time = 0;
  count_time_delta = 0;
    
} // End interruption

bool define_state(int button, bool button_state, int pressed_time){

  count_time_delta = 0;

  if(button_state == LOW){

    button_state = LOW;
    count_time = millis();

    while(!button_state){

      if(count_time_delta < pressed_time){

      detachInterrupt(button_cancel);
      detachInterrupt(button_start);
      detachInterrupt(button_cycle);

      button_state = digitalRead(button);
      count_time_delta = millis();
      count_time_delta -= count_time;

      }
      else break;

    }  

  }

  count_time = 0;

  return HIGH;

}