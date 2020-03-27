#include <Arduino.h>
// communication
// wire for i2c
#include <Wire.h> 
// SPI for like yea SPI
#include <SPI.h>
// display class
#include <LiquidCrystal_I2C.h>
// CCS811 class
#include "ccs811.h"
// SD for writing data to the SD card
#include <SD.h>


bool sensorsReadyState = false;  // state if sensors are heated

const unsigned long sensorHeatupTime = 8000;   // time for sensors to heat up

const unsigned int buttonPin = 2;   // pin used for button to cycle in the menu
boolean buttonState = LOW;
boolean previousButtonState = LOW;
unsigned long previousButtonPollMillis = 0;
const unsigned long buttonInterval = 20;
unsigned int menuSelectedSensor = 0;
unsigned int oldSelectedMenu = 0;

const unsigned int mq135pin = A0;   // connection pin for MQ135 -> 1 sensor value
const unsigned int ccs811wakepin = 8;


// i2c addresses
const uint8_t lcdAddr = 0x38;
const uint8_t ccs811Addr = 0x5B;

// bootup screen dot logic
const unsigned long bootingDotsdelay = 1000;
unsigned long previousbootinMillis = 0;
unsigned int bootingdots = 0;

LiquidCrystal_I2C lcd(lcdAddr, 16, 2);
CCS811 myCCS811(ccs811wakepin, ccs811Addr); // 2 sensors CO2 and VOTC

File myFile;

char resultFileName[] = "results.csv"; 

// total sensor types
const int amountOfSensors = 3;
unsigned long sensorOutputResults[amountOfSensors];
String sensorNames[amountOfSensors] = {
  "CCS811 - CO2", 
  "CCS811 - Voc", 
  "MQ135"
};

// sensor polling interval
const unsigned long sensorPollingInterval = 15000;
unsigned long previousPollingMillis = 0;

// declare before setup so calling is possible => https://community.platformio.org/t/order-of-function-declaration/4546/2
void handleButtonPress();

void setup() {
  // setup serial for debugging
  Serial.begin(9600);

  // setup button as interrupt pin
  
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, CHANGE);
  // Setup lcd display
  lcd.init();                        // Initialize I2C LCD module
  lcd.backlight();                   // Turn backlight ON
  lcd.setCursor(0, 0);  // Go to column 0, row 0
  Serial.println("Started LCD");
  // setup sensors
  // setup CCS811 => carbon dioxide (eCO2) and metal oxide (MOX)
  if(!myCCS811.begin()){
    Serial.println("setup: CCS811 begin FAILED");
  }
   // Print CCS811 versions
  // Serial.print("setup: hardware    version: "); Serial.println(myCCS811.hardware_version(),HEX);
  // Serial.print("setup: bootloader  version: "); Serial.println(myCCS811.bootloader_version(),HEX);
  // Serial.print("setup: application version: "); Serial.println(myCCS811.application_version(),HEX);
  // Start measuring
  if(!myCCS811.start(CCS811_MODE_10SEC)){
    Serial.println("setup: CCS811 begin FAILED");
  }
  // setup SD card writer
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  } else {
    Serial.println("Started SPI");
  }
  if (!SD.exists(resultFileName)) {
    Serial.print("File does not exist create and write header");
    myFile = SD.open(resultFileName, FILE_WRITE);
    String header = "";
    for (int i = 0; i < amountOfSensors; i++)
    {
      if (i + 1 == amountOfSensors) {
        // the last time the loop will run so do not add a comma to the line
        header.concat(sensorNames[i]);
      } else {
        header.concat(sensorNames[i] + ",");
        }
    }
    myFile.println(header);
    // close the file:
    myFile.close();
  }
}



void showBootLoop(){
  lcd.setCursor(0, 0);
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

void handleButtonPress() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    menuSelectedSensor++;
  }
}

void view() {
  // This method handles all the UI stuff 
  // button changed in the last 20 millis it got pressed and we need to refresh the display
  unsigned long currentMillis = millis();
  if(currentMillis - previousButtonPollMillis >= buttonInterval)
  {
    if(oldSelectedMenu != menuSelectedSensor) {
      // change detected clear lcd for new data
      lcd.clear();
    }
    oldSelectedMenu = menuSelectedSensor;
  }
  if (menuSelectedSensor >= amountOfSensors) menuSelectedSensor = 0;
  lcd.setCursor(0, 0);
  lcd.print(sensorNames[menuSelectedSensor]);
  lcd.setCursor(0, 1);
  lcd.print(sensorOutputResults[menuSelectedSensor]);
  // reset when overshooting amount of sensors
}

void storageLogic() {
  myFile = SD.open(resultFileName, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Writing to result file ...");
    String row = "";
    for (int i =0; i < amountOfSensors; i++) {
      if (i + 1 == amountOfSensors) {
        // the last time the loop will run so do not add a comma to the line
        Serial.print("Last sensor value: ");
        Serial.println(sensorOutputResults[i]);
        row.concat(String(sensorOutputResults[i]));
      } else {
        Serial.print("Sensor value: ");
        Serial.println(sensorOutputResults[i]);
        row.concat(String(sensorOutputResults[i]) + ",");
        }
    }
    Serial.print("ROW to file: ");
    Serial.println(row);
    myFile.println(row);
    // close the file:
    myFile.close();
    Serial.print("Closed file");
  } else {
  // if the file didn't open, print an error:
    Serial.println("error opening result file");
  }
}

void sensorLogic() {
  // start readout of DSM501A sensor
  // readoutDSM501A();
  unsigned long currentMillis = millis();
  // this means reading out sensor values if possible or reading data that is already present (like DSM501)
  if(currentMillis - previousPollingMillis > sensorPollingInterval) {
    previousPollingMillis = currentMillis; 
    /* 
    
          reading data from ccs811 sensor 

    */
    uint16_t eco2, etvoc, errstat, raw;
    myCCS811.read(&eco2,&etvoc,&errstat,&raw); 
    if( errstat==CCS811_ERRSTAT_OK ) { 
      Serial.print("CCS811: ");
      Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
      Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
      // save to array
      sensorOutputResults[0] = eco2;
      sensorOutputResults[1] = etvoc;
      Serial.println();
    } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
      Serial.println("CCS811: waiting for (new) data");
    } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
      Serial.println("CCS811: I2C error");
    } else {
      Serial.print("CCS811: errstat="); Serial.print(errstat,HEX); 
      Serial.print("="); Serial.println( myCCS811.errstat_str(errstat) ); 
    }
    /* 
    
          reading data from MQ135

    */
    int val = analogRead(mq135pin);
    sensorOutputResults[2] = val;

    // save to sd card
    storageLogic();
  }
  
}

void start() {
  view();
  sensorLogic();
}

void loop() {
  // wait for 10 seconds for all sensors to get ready one time
  unsigned long currentMillis = 0;
  if (!sensorsReadyState) {
     currentMillis = millis();
  }
  if(currentMillis > sensorHeatupTime || sensorsReadyState) {
    if (sensorsReadyState == false) {
      Serial.println("Application has started");
      sensorsReadyState = true;
    }
    start();
  } else {
    showBootLoop();
  }
}
