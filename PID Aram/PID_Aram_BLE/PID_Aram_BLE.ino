// Librarys
#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include "max6675.h"

// Global constants
#define pot_read        0                                               // Analog pin for the potentiometer
#define button_select   1                                               // Selection button pin
#define led             2                                               // Led pin
#define resistor        3                                               // Resistor relay pin
#define thermo_SCK      4                                               // SPI SCK pin
#define thermo_SO       5                                               // SPI MISO pin
#define thermo_CS       7                                               // SPI CS pin
#define mean_times      2000                                            // Number of values used to calculate the mean
#define time_pressed    100                                             // Maximum time to keep the button pressed
#define deb_delay       50                                              // Debounce time (ms)
#define offset          3                                               // Offset to correct the maximum value provided by the potentiometer and to enter PID mode
#define temp_max        100                                             // Define temperature maximum value
#define temp_min        5                                               // Define temperature minimum value
#define pwm_res         8                                               // PWM resolution (bits) 2^8 = 256
#define pwm_freq        200                                             // PWM frequency (Hz)
#define pwm_channel     0                                               // PWM channel
#define refresh_rate    200                                             // Program refresh rate (ms)
#define kp              2.5                                             // Proporcional error constant
#define ki              0.06                                            // Integration error constant
#define kd              0.8                                             // Differentiation error constant
#define tau             0.02                                            // Low pass filter constant
#define pid_i_lim_max   5                                               // Maximum limit of the integrator
#define pid_i_lim_min   -5                                              // Minimum limit of the integrator 
#define pid_lim_max     18                                              // Maximum limit of PID calculation
#define pid_lim_min     0                                               // Minimum limit of PID calculation
#define SCREEN_WIDTH    128                                             // OLED display width, in pixels
#define SCREEN_HEIGHT   64                                              // OLED display height, in pixels
#define OLED_RESET      -1                                              // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS  0x3C                                            // Address for I2C communication (pins: 8 and 9)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"   // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // See the following for generating UUIDs:
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // https://www.uuidgenerator.net/

// Global variables
float temp_select_cal = 0;                                              // Temperature (Celsius) for calculations selected by user
float temp_cal = 0;                                                     // Temperature (Celsius) for calculations provided by sensor
float last_temp_cal = 0;                                                // Last Temperature (Celsius) provided by sensor
float pid_error = 0;                                                    // PID error
float last_pid_error = 0;                                               // Last PID error
float pid_p = 0;                                                        // Proporcional error
float pid_i = 0;                                                        // Integration error
float pid_d = 0;                                                        // Differentiation error
float pid_out = 0;                                                      // Sum of PID error parts
float digital_temp_select_cal = 0;                                      // Digital value provided by BLE
int temp_select_display = 0;                                            // Temperature (Celsius) for display selected by user
int temp_display = 0;                                                   // Temperature (Celsius) for display provided by sensor
int select_mode = 0;                                                    // Mode selection
int led_count = 0;                                                      // Led counter blink
bool led_state = LOW;                                                   // Led state
bool deviceConnected = false;                                           // Check if device is connected
bool oldDeviceConnected = false;                                        // Check if old device is connected
volatile bool button_select_state = HIGH;                               // selection button state
volatile bool digital_select = LOW;                                     // Digital select button
volatile bool digital_cancel = LOW;                                     // Digital cancel button
volatile bool rep = LOW;                                                // Variable to prevent an operation from being repeated
volatile int digital_pot = 0;                                           // Digital step provided by BLE
volatile unsigned long count_time = 0;                                  // Inicial time counter 
volatile unsigned long count_time_delta = 0;                            // Delta time counter

/*
// Bitmap for Bluetooth connect icon 32x32
const unsigned char bluetooth_connect_icon [] PROGMEM = {
	
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 
	0x00, 0x01, 0xc0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x81, 0xf8, 0x00, 
	0x01, 0xc1, 0xfc, 0x00, 0x00, 0xe1, 0x9e, 0x00, 0x00, 0x71, 0x9e, 0x00, 0x00, 0x39, 0x9c, 0x00, 
	0x00, 0x1f, 0xf8, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x07, 0x07, 0xe0, 0xe0, 0x0f, 0x83, 0xc1, 0xf0, 
	0x0f, 0x83, 0xc1, 0xf0, 0x07, 0x07, 0xe0, 0xe0, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x1f, 0xf8, 0x00, 
	0x00, 0x39, 0x9c, 0x00, 0x00, 0x71, 0x9e, 0x00, 0x00, 0xe1, 0x9e, 0x00, 0x01, 0xc1, 0xfc, 0x00, 
	0x00, 0x81, 0xf8, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x01, 0xc0, 0x00, 
	0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};
*/
// Bitmap for Bluetooth connect icon 16x16
const unsigned char bluetooth_connect_icon [] PROGMEM = {
	
	0x00, 0x00, 0x01, 0x00, 0x01, 0x80, 0x01, 0xc0, 0x09, 0xe0, 0x05, 0xe0, 0x03, 0xc0, 0x11, 0x88, 
	0x11, 0x88, 0x03, 0xc0, 0x05, 0xe0, 0x09, 0xe0, 0x01, 0xc0, 0x01, 0x80, 0x01, 0x00, 0x00, 0x00

};

/*
// Bitmap for Bluetooth icon 32x32
const unsigned char bluetooth_icon [] PROGMEM = {

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 
	0x00, 0x01, 0xc0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x81, 0xf8, 0x00, 
	0x01, 0xc1, 0xbc, 0x00, 0x00, 0xe1, 0x9e, 0x00, 0x00, 0x71, 0x8e, 0x00, 0x00, 0x39, 0x9c, 0x00, 
	0x00, 0x1f, 0xf8, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x03, 0xc0, 0x00, 
	0x00, 0x03, 0xc0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x1f, 0xf8, 0x00, 
	0x00, 0x39, 0x9c, 0x00, 0x00, 0x71, 0x8e, 0x00, 0x00, 0xe1, 0x9e, 0x00, 0x01, 0xc1, 0xbc, 0x00, 
	0x00, 0x81, 0xf8, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x01, 0xc0, 0x00, 
	0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};

*/
// Bitmap for Bluetooth icon 16x16
const unsigned char bluetooth_icon [] PROGMEM = {

	0x00, 0x00, 0x01, 0x00, 0x01, 0x80, 0x01, 0xc0, 0x09, 0xe0, 0x07, 0xe0, 0x03, 0xc0, 0x01, 0x80, 
	0x01, 0x80, 0x03, 0xc0, 0x05, 0xe0, 0x09, 0xe0, 0x01, 0xc0, 0x01, 0x80, 0x01, 0x00, 0x00, 0x00

};


// Pointer definitions
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;

// Functions declaration
float set_temp(int analog_port);
bool button_select_press(int button, bool button_state, int pressed_time);
void temp_select_mode(void);
void ramp_mode(void);
void pid_mode(void);
void send_data_ble(float tx_value);

// Interruption routine
void ARDUINO_ISR_ATTR debounce_select();

// External devices initialization
MAX6675 thermocouple(thermo_SCK, thermo_CS, thermo_SO);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Classes
class MyServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer *pServer) {

    // Device connected
    deviceConnected = true;

  };

  void onDisconnect(BLEServer *pServer) {

    // Device disconnected
    deviceConnected = false;
    pServer->startAdvertising();

  }

};

class MyCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *pCharacteristic) {

    String rxValue = pCharacteristic->getValue();

    // Check for any received value
    if (rxValue.length() > 0) {

      // Saves the received value in a string
      for (int i = 0; i < rxValue.length(); i++) {

        rxValue[i];

      }

      // Converts string to int
      //int rx_value = rxValue.toInt();

      // Converts string to float
      float rx_value = rxValue.toFloat();

      if(rxValue.indexOf("S") != -1){

        // Digital select button
        digital_cancel = LOW;
        button_select_state = HIGH;
        digital_temp_select_cal = 0;

        if(select_mode == 0 && rep == HIGH){

          digital_select = HIGH;

        }

      }
      else if(rxValue.indexOf("C") != -1){

        // Digital cancel button
        digital_select = LOW;
        button_select_state = HIGH;
        digital_temp_select_cal = 0;

        if(select_mode != 0 && rep == HIGH){

          digital_cancel = HIGH;

        }

      }
      else if(rxValue.indexOf("+") != -1){

        // Digital select step up button 
        digital_select = LOW;
        digital_cancel = LOW;
        button_select_state = HIGH;
        digital_temp_select_cal = 0;

        if(select_mode == 0 && rep == HIGH && temp_select_display < temp_max){

          digital_pot++;

        }

      }
      else if(rxValue.indexOf("-") != -1){

        // Digital select step down button 
        digital_select = LOW;
        digital_cancel = LOW;
        button_select_state = HIGH;
        digital_temp_select_cal = 0;

       if(select_mode == 0 && rep == HIGH && temp_select_display > temp_min){

          digital_pot--;

        }

      }
      else if((rx_value >= temp_min) && (rx_value <= temp_max)){

        // Digital temperature selection
        digital_cancel = LOW;
        button_select_state = HIGH;

        // Set temperature value and start ramp mode
        if(select_mode == 0 && rep == HIGH){

          digital_temp_select_cal = rx_value;
          digital_select = HIGH;

        }

      }
    
    }
    
    // Prevent repetition
    rep = LOW;

  }

};

// Setup routine
void setup() {

  // Initialize serial communication at 115200 bits per second
  //Serial.begin(115200);

  // BLE setup
  // Create the BLE device (devide name)
  BLEDevice::init("PID Aram");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  // Descriptor 2902 is not required when using NimBLE as it is automatically added based on the characteristic properties
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  // initialize oled display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  
  delay(100);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {

    for (;;);

  }

  display.clearDisplay();
  display.setTextSize(2);  
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(100);

  // Start interruption on button_select
  attachInterrupt(button_select, debounce_select, FALLING);

  // Set pin, frequency, bit resolution and channel for the PWM and turn off resistor
  ledcAttachChannel(resistor, pwm_freq, pwm_res, pwm_channel);
  ledcWrite(resistor, 255);

  // Initialize led as an OUTPUT and LOW
  pinMode(led, OUTPUT);
  digitalWrite(led, led_state);

  // Initialize selection button as an input and enable the internal pull-up resistor
  pinMode(button_select, INPUT_PULLUP);

  // Wait for all devices stabilize
  delay(refresh_rate * 10);

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

  attachInterrupt(button_select, debounce_select, FALLING);

} // End of loop routine

// Interruption select button routine
void ARDUINO_ISR_ATTR debounce_select(){

  detachInterrupt(button_select);

  count_time_delta = 0;
  count_time = millis();

  while(count_time_delta < deb_delay){

    count_time_delta = millis();
    count_time_delta -= count_time;

  }

  button_select_state = digitalRead(button_select);

  count_time_delta = 0;
  count_time = 0;

  digital_select = LOW;
  digital_cancel = LOW;
  rep = LOW;
    
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

// #2 - Check selection button state and the time it was pressed 
bool button_select_press(int button, bool button_state, int pressed_time){

  count_time_delta = 0;

  if(!button_state){

    button_state = LOW;
    count_time = millis();

    while(!button_state){

      if(count_time_delta < pressed_time){

      detachInterrupt(button);

      digital_select = LOW;
      digital_cancel = LOW;

      button_state = digitalRead(button);
      
      count_time_delta = millis();
      count_time_delta -= count_time;

      }
      else break;

    }

    count_time = 0;

  }

  return HIGH;

} // End of function #2

// #3 - Temperature selection mode
void temp_select_mode(void){

  // Read the state of the selection button with debounce
  button_select_state = button_select_press(button_select, button_select_state, (time_pressed - deb_delay));

  // Check if selection button has been pressed and switch to Ramp mode
  if((count_time_delta >= (time_pressed - deb_delay)) || (digital_select == HIGH)){

    // Turn on led and resistor
    digitalWrite(led, !led_state);
    ledcWrite(resistor, 0);
    led_count = 0;

    // Check if any value was recevied via BLE
    if(digital_temp_select_cal != 0){

      temp_select_cal = digital_temp_select_cal;
      temp_select_display = round(temp_select_cal);

    }

    // Change mode and force variable values to prevent error
    select_mode = 1;
    button_select_state = HIGH;
    digital_select = LOW;
    digital_cancel = LOW;
    rep = LOW;
    count_time_delta = 0;
    digital_temp_select_cal = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Starting");
    display.println("Ramp");
    display.println("Mode...");

    // Display the image
    display.display();

    delay(refresh_rate * 10);

    return;

  }

  // Set temperature from the potentiometer + offset (offset is needed to correct the maximum value of set_temp)
  temp_select_cal = set_temp(pot_read) + offset;

  // Defines temperature limits. Limits need to be defined before deciding whether the digital button will be reset or not
  if ((temp_select_cal + digital_pot) > temp_max){

    temp_select_cal = temp_max;

  } 
  else if ((temp_select_cal + digital_pot) < temp_min){

    temp_select_cal = temp_min;

  }

  // Update the digital button value depending on the temperature limits
  // Reset digital button routine
  if ((temp_select_cal + digital_pot) > temp_max){

    digital_pot = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Digital");
    display.println("Button");
    display.print("Reset...");
    
    // Display the image
    display.display();

    delay(refresh_rate * 10);

    display.clearDisplay();
    display.setCursor(0,0);  
    display.println("Returning");
    display.println("To the");
    display.println("Original");
    display.print("Value...");

    // Display the image
    display.display();

    delay(refresh_rate * 10);

  } 
  else if ((temp_select_cal + digital_pot) < temp_min){

    digital_pot = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Digital");
    display.println("Button");
    display.print("Reset...");
    
    // Display the image
    display.display();

    delay(refresh_rate * 10);

    display.clearDisplay();
    display.setCursor(0,0);  
    display.println("Returning");
    display.println("To the");
    display.println("Original");
    display.print("Value...");

    // Display the image
    display.display();

    delay(refresh_rate * 10);

  }

  temp_select_cal += digital_pot;
  temp_select_display = round(temp_select_cal);

  // Check if BLE is connected
  if(deviceConnected){

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Select");
    display.println("Temp:");
    display.setCursor(37,50);
    display.print(temp_select_display);
    display.print((char)247);
    display.print("C");
    display.drawBitmap(112, 0, bluetooth_connect_icon, 16, 16, SSD1306_WHITE);
  
    // Display the image
    display.display();

    // Send data via BLE
    send_data_ble(temp_select_cal);

  }else{

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Select");
    display.println("Temp:");
    display.setCursor(37,50);
    display.print(temp_select_display);
    display.print((char)247);
    display.print("C");
    display.drawBitmap(112, 0, bluetooth_icon, 16, 16, SSD1306_WHITE);
  
    // Display the image
    display.display();

  }

  // Prevent repetition
  rep = HIGH;

  return;

} // End of function #3

// #4 - Ramp up mode
void ramp_mode(void){

  // Read the state of the selection button with debounce
  button_select_state = button_select_press(button_select, button_select_state, ((time_pressed * 10) - deb_delay));

  // Cancel ramp up mode and back to temperature selection mode
  if((count_time_delta >= ((time_pressed * 10) - deb_delay)) || (digital_cancel == HIGH)){

    // Turn off led and resistor
    digitalWrite(led, led_state);
    ledcWrite(resistor, 255);

    // Change mode and force variable values to prevent error
    select_mode = 0;
    button_select_state = HIGH;
    digital_select = LOW;
    digital_cancel = LOW;
    count_time_delta = 0;
    digital_temp_select_cal = 0;

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
    display.println("Selection");
    display.print("Mode...");
  
    // Display the image
    display.display();

    delay(refresh_rate * 10);

    return;

  }

  delay(refresh_rate);

  // Get temperature from sensor
  temp_cal = thermocouple.readCelsius();
  temp_display = round(temp_cal);

  // Check if the target temperature has been reached and switch to PID mode.
  if(temp_cal > (temp_select_cal - offset)){

    // Turn off led and resistor
    digitalWrite(led, led_state);
    ledcWrite(resistor, 255);
    led_count = 0;

    delay(refresh_rate * 5);

    // Change mode and force variable values to prevent error
    select_mode = 3;
    button_select_state = HIGH;
    digital_select = LOW;
    digital_cancel = LOW;
    count_time_delta = 0;
    digital_temp_select_cal = 0;

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Starting");
    display.println("PID");
    display.print("Mode...");
  
    // Display the image
    display.display();

    delay(refresh_rate * 100);

    return;

  }

  // Check if BLE is connected
  if(deviceConnected){

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
    display.drawBitmap(112, 0, bluetooth_connect_icon, 16, 16, SSD1306_WHITE);
  
    // Display the image
    display.display();

    // Send data via BLE
    send_data_ble(temp_cal);

  }
  else{

    //Clear dislpay and print text
    display.clearDisplay();  
    display.setCursor(0,0);           
    display.println("Ramp Mode");
    display.print("S: ");
    display.print(temp_select_cal);
    display.print((char)247);
    display.println("C");
    display.println();
    display.print("C: ");
    display.print(temp_display);
    display.print((char)247);
    display.print("C");
    display.drawBitmap(112, 0, bluetooth_icon, 16, 16, SSD1306_WHITE);
  
    // Display the image
    display.display();

  }

  // Prevent repetition
  rep = HIGH;

  return;

} // End of function #4

// #5 - PID mode
void pid_mode(void){

  // Read the state of the selection button with debounce
  button_select_state = button_select_press(button_select, button_select_state, ((time_pressed * 10) - deb_delay));

  // Cancel PID mode and back to temperature selection mode
  if((count_time_delta >= ((time_pressed * 10) - deb_delay)) || (digital_cancel == HIGH)){

    // Turn off led and resistor
    digitalWrite(led, led_state);
    ledcWrite(resistor, 255);
    led_count = 0;

    // Change mode and force variable values to prevent error
    select_mode = 0;
    button_select_state = HIGH;
    digital_select = LOW;
    digital_cancel = LOW;
    count_time_delta = 0;
    digital_temp_select_cal = 0;

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
    display.println("Selection");
    display.print("Mode...");

    // Display the image
    display.display();

    delay(refresh_rate * 10);

    return;

  }

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

  // Check if BLE is connected
  if(deviceConnected){

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
    display.drawBitmap(112, 0, bluetooth_connect_icon, 16, 16, SSD1306_WHITE);
  
    // Display the image
    display.display();

    // Send data via BLE
    send_data_ble(temp_cal);

  }
  else{

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
    display.drawBitmap(112, 0, bluetooth_icon, 16, 16, SSD1306_WHITE);
  
    // Display the image
    display.display();

  }


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

  // Prevent repetition
  rep = HIGH;

  return;

} // End of function #5

void send_data_ble(float tx_value){

  // Connecting to new device
  if(deviceConnected){

    char txString[6];
    String tempString = String(tx_value);
    tempString.toCharArray(txString, sizeof(txString));
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();

  }

  // Disconnecting
  if(!deviceConnected && oldDeviceConnected){

    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    oldDeviceConnected = false;

  }
  
  // Connecting to old device
  if(deviceConnected && !oldDeviceConnected){

    char txString[6];
    String tempString = String(tx_value);
    tempString.toCharArray(txString, sizeof(txString));
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();

    oldDeviceConnected = true;

  }

  return;

}