#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
// CCS811 class
#include "Adafruit_CCS811.h"

// Requires headers for AVR defines and ISR function
#include <avr/io.h>
#include <avr/interrupt.h>

#define INTERRUPT_PIN PCINT1  // This is PB1 per the schematic
#define INT_PIN PB1  

// libs for BME280
// #include <Adafruit_BME280.h>
#define TINY_BME280_I2C
#include "TinyBME280.h"
tiny::BME280 bme;

bool sensorsReadyState = false;  // state if sensors are heated

const unsigned short sensorHeatupTime = 8000;   // time for sensors to heat up

const uint8_t buttonPin = 2;   // pin used for button to cycle in the menu
boolean buttonState = LOW;
boolean previousButtonState = LOW;
unsigned long previousButtonPollMillis = 0;
const  uint8_t buttonInterval = 20;
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
  "°C",
  "%",
  "hPa"
};

// sensor polling interval
const unsigned short sensorPollingInterval = 15000;
unsigned long previousPollingMillis = 0;

// lcd variable
unsigned long previousButtonPressMillis = 0;
const unsigned short lcdTimeToSleep = 15000;

// declare before setup so calling is possible => https://community.platformio.org/t/order-of-function-declaration/4546/2
// void handleButtonPress();

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, CHANGE);
  GIMSK |= (1 << PCIE);   // pin change interrupt enable
	PCMSK |= (1 << PCINT4); // pin change interrupt enabled for PCINT4
	sei();                  // enable interrupts

  lcd.init();                        // Initialize I2C LCD module
  if(!ccs.begin()){
    while(1);
  }
  if (!bme.begin()) {
		while (1);
	}
}

void showBootLoop(){
  lcd.setCursor(0, 0);
  lcd.backlight();  
  lcd.print("Booting up");
  unsigned long currentMillis = millis();
  if(currentMillis - previousbootinMillis > bootingDotsdelay) {
    previousbootinMillis = currentMillis; 
     lcd.setCursor(11,0 );
    if (bootingdots == 0) {
      bootingdots++;
      lcd.print(".  ");
    } else if (bootingdots == 1) {
      bootingdots++;
      lcd.print(".. ");
    } else if (bootingdots == 2) {
      bootingdots = 0;
      lcd.print("...");
    }
  }
}

ISR(PCINT0_vect) {
  // buttonState = digitalRead(buttonPin);
  // if (buttonState == LOW) {
  //   menuSelectedSensor++;
  // }
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
    menuSelectedSensor++;
  }
}

void view() {
  // This method handles all the UI stuff 
  // button changed in the last 20 millis it got pressed and we need to refresh the display
  unsigned long currentMillis = millis();
  if(currentMillis - previousButtonPollMillis >= buttonInterval)
  {
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
  }
  
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
        while(1);
      }
    }
    
    sensorOutputResults[2] = bme.readFixedTempC() / 100.0;
    sensorOutputResults[3] = bme.readFixedHumidity()/ 100.1;
    sensorOutputResults[4] = bme.readFixedPressure() / 100.0;
  }

}

void start() {
  view();
  sensorLogic();
}

void loop() {
  unsigned long currentMillis = 0;
    if (!sensorsReadyState) {
      currentMillis = millis();
    }
    if(currentMillis > sensorHeatupTime || sensorsReadyState) {
      if (sensorsReadyState == false) {
        sensorsReadyState = true;
        lcd.clear();
      }
      start();
    } else {
      showBootLoop();
    }
}