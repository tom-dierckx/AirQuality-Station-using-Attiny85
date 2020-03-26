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

const unsigned int buttonPin = 7;   // pin used for button to cycle in the menu
boolean buttonState = LOW;
boolean previousButtonState = LOW;
unsigned long previousButtonPollMillis = 0;
const unsigned long buttonInterval = 20;
unsigned int menuSelectedSensor = 0;

const unsigned int mq135pin = A0;   // connection pin for MQ135 -> 1 sensor value
const unsigned int ccs811wakepin = 8;

// DSM501A logic
const unsigned int dsm25pin = 3; 
const unsigned int dsm10pin = 2;
float dsm25Concentration = 0; 
float dsm10Concentration = 0;
unsigned long dsm25Lowpulseoccupancy = 0;
unsigned long dsm10Lowpulseoccupancy = 0;
unsigned long  dsm25duration;
unsigned long dsm10duration;
float dsm25ratio = 0;
float dsm10ratio = 0;
unsigned long dsmstarttime;
unsigned long dsm_sampletime_ms = 30000;

uint32_t lowpulseoccupancy = 0;
float ratio = 0;

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

char resultFileName[] = "results2.csv"; 

// total sensor types
const int amountOfSensors = 5;
unsigned long sensorOutputResults[amountOfSensors];
String sensorNames[amountOfSensors] = {
  "CCS811 - CO2", 
  "CCS811 - Voc", 
  "MQ135",
  "DSM501A PM25",
  "DSM501A PM10"
};

// sensor polling interval
const unsigned long sensorPollingInterval = 60000;
unsigned long previousPollingMillis = 0;

void setup() {
  // setup serial for debugging
  Serial.begin(9600);
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
      header.concat(sensorNames[i]);
    }
    myFile.println(header);
    // close the file:
    myFile.close();
  }
}

void readoutDSM501A() {
	dsm25duration = pulseIn(dsm25pin, LOW);
  dsm10duration = pulseIn(dsm10pin, LOW);
  dsm25Lowpulseoccupancy = dsm25Lowpulseoccupancy+dsm25duration;
  dsm10Lowpulseoccupancy = dsm10Lowpulseoccupancy+dsm10duration;
  if ((millis()-dsmstarttime) > dsm_sampletime_ms)//if the sampel time == 30s
  { 
    dsm25ratio = dsm25Lowpulseoccupancy/(dsm_sampletime_ms*10.0);  // Integer percentage 0=>100
    dsm25Concentration = 1.1*pow(dsm25ratio,3)-3.8*pow(dsm25ratio,2)+520*dsm25ratio+0.62; // using spec sheet curve
    Serial.print("dsm25Concentration");
    Serial.println(dsm25Concentration);
    dsm10ratio = dsm10Lowpulseoccupancy/(dsm_sampletime_ms*10.0);  // Integer percentage 0=>100
    dsm10Concentration = 1.1*pow(dsm10ratio,3)-3.8*pow(dsm10ratio,2)+520*dsm10ratio+0.62; // using spec sheet curve
    Serial.print("dsm10Concentration");
    Serial.println(dsm10Concentration);
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

void view() {
  // This method handles all the UI stuff 
  // button control and the lcd display
  Serial.print("Selected Menu: ");
  Serial.println(menuSelectedSensor);
  unsigned long currentMillis = millis();
  buttonState = digitalRead(buttonPin);
  if(currentMillis - previousButtonPollMillis >= buttonInterval)
  {
    if (buttonState != previousButtonState) {
      if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
        menuSelectedSensor++;
        Serial.println("Button pressed");
        lcd.clear();
      } 
    }
    previousButtonPollMillis = currentMillis;
    // increment on button press
  }
  if (menuSelectedSensor >= amountOfSensors) menuSelectedSensor = 0;

  previousButtonState = buttonState;
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
        row.concat(sensorOutputResults[i]);
      } else {
        row.concat(sensorOutputResults[i] + ",");
        }
    }
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

    /*

          reading data from DSA501A

    */
   
    sensorOutputResults[3] = dsm25Concentration;
    sensorOutputResults[4] = dsm10Concentration;
    // write to sd card
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
