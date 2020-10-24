#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
// CCS811 class
#include "Adafruit_CCS811.h"

// Requires headers for AVR defines and ISR function
// #include <avr/io.h>
// #include <avr/interrupt.h>
#include <avr/sleep.h>
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)

#define INTERRUPT_PIN PCINT1  // This is PB1 per the schematic
#define INT_PIN PB1  

// libs for BME280
// #include <Adafruit_BME280.h>
#define TINY_BME280_I2C
#include "TinyBME280.h"
tiny::BME280 bme;

bool sensorsReadyState = false;  // state if sensors are heated


   // pin used for button to cycle in the menu
// debug pins
const uint8_t errorPin = 3;

// button logic
const uint8_t buttonPin = 4;
boolean buttonState = LOW;
boolean lastButtonState = LOW;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

uint8_t menuSelectedSensor = 0;
uint8_t oldSelectedMenu = 0;


// i2c addresses
const uint8_t lcdAddr = 0x27;
const uint8_t ccs811Addr = 0x5A;
const uint8_t bme280Addr = 0x76;
// bootup screen dot logic
const unsigned short bootingDotsdelay = 1000;
unsigned long previousbootinMillis = 0;
uint8_t bootingdots = 0;
LiquidCrystal_I2C lcd(lcdAddr, 16, 2);

// Adafruit_BME280 bme;
Adafruit_CCS811 ccs;

// total sensor types
const uint8_t amountOfSensors = 5;
double sensorOutputResults[amountOfSensors];
const char sensorNames[amountOfSensors][8] = {
  "CO2", 
  "Voc", 
  "Temp",
  "Humid",
  "Press"
};
// order has to match sensorNames
const char sensorValueUnit[amountOfSensors][4] = {
  "ppm", 
  "ppb", 
  "C",
  "%",
  "hPa"
};

// sensor polling interval
const unsigned short sensorPollingInterval = 15000;
unsigned long previousPollingMillis = 0;

// lcd variable
unsigned long previousButtonPressMillis = 0;
const unsigned short lcdTimeToSleep = 5000;

// declare before setup so calling is possible => https://community.platformio.org/t/order-of-function-declaration/4546/2
// void handleButtonPress();

void setup() {
  pinMode(errorPin, OUTPUT);
  digitalWrite(errorPin, LOW);
  if(!ccs.begin()){
    digitalWrite(errorPin, HIGH);
    while(1);
  }
  ccs.setDriveMode(CCS811_DRIVE_MODE_10SEC);
  if (!bme.beginI2C(bme280Addr)) {
		digitalWrite(errorPin, HIGH);
    while(1);
	}
  // make sensor sleep for low power
  bme.setMode(tiny::Mode::SLEEP);
  // disable ADC no need for it 
  adc_disable();
  lcd.init();
  lcd.backlight();
}

void showBootLoop(){
  lcd.setCursor(0, 0);
  lcd.backlight();  
  lcd.print("Heating sensors");
}

void button_logic() {
  boolean reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        menuSelectedSensor++;
      }
    }
  }
  lastButtonState = reading;
}

void view() {
  // This method handles all the UI stuff 
  // button changed in the last 20 millis it got pressed and we need to refresh the display
  unsigned long currentMillis = millis();

  // Serial.print("Currently selected menu");
  // Serial.println(menuSelectedSensor);
  if(oldSelectedMenu != menuSelectedSensor) {
    // change detected clear lcd for new data
    lcd.clear();
    lcd.backlight();
    // reset timer
    previousButtonPressMillis = currentMillis;
  }
  oldSelectedMenu = menuSelectedSensor;
  
  if (currentMillis - previousButtonPressMillis > lcdTimeToSleep ) {
     lcd.noBacklight();
  }

  if (menuSelectedSensor >= amountOfSensors) {
    menuSelectedSensor = 0;
    lcd.clear();
  }
  lcd.setCursor(0, 0);
  lcd.print(sensorNames[menuSelectedSensor]);
  lcd.setCursor(0, 1);
  lcd.print(sensorOutputResults[menuSelectedSensor]);
  lcd.setCursor(10, 1);
  lcd.print(sensorValueUnit[menuSelectedSensor]);
  // reset when overshooting amount of sensors
}

void sensorLogic() {
  // reading data from dht
  unsigned long currentMillis = millis();
  // this means reading out sensor values if possible or reading data that is already present (like DSM501)
  if(currentMillis - previousPollingMillis > sensorPollingInterval) {
    previousPollingMillis = currentMillis; 
    /* 
    
          reading data from ccs811 sensor 

    */
    if(ccs.available()){
      if(!ccs.readData()){
        sensorOutputResults[0] = ccs.geteCO2();
        sensorOutputResults[1] = ccs.getTVOC();
      }
      else{
        digitalWrite(errorPin, HIGH);
      }
    }
    bme.setMode(tiny::Mode::FORCED);
    // wait for mesurement
    while(bme.isMeasuring() == false) ; //Wait for sensor to start measurment
    while(bme.isMeasuring() == true) ; //Hang out while sensor completes the reading
    sensorOutputResults[2] = bme.readFixedTempC() / 100.0;
    sensorOutputResults[3] = bme.readFixedHumidity()/ 1000.0;
    sensorOutputResults[4] = bme.readFixedPressure() / 100.0;
  }

}

void loop() {
  view();
  sensorLogic();
  button_logic();
}