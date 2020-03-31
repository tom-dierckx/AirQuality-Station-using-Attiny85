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

// digital or analog pins
const unsigned int mq135pin = A0;   // connection pin for MQ135 -> 1 sensor value
const unsigned int ccs811wakepin = A2;
const unsigned int dhtpin = 5;

// i2c addresses
const uint8_t lcdAddr = 0x38;
const uint8_t ccs811Addr = 0x5B;
const uint8_t dht22Addr = 0x64;
// bootup screen dot logic
const unsigned long bootingDotsdelay = 1000;
unsigned long previousbootinMillis = 0;
unsigned int bootingdots = 0;
LiquidCrystal_I2C lcd(lcdAddr, 16, 2);
CCS811 myCCS811(ccs811wakepin, ccs811Addr); // 2 sensors CO2 and VOTC

// file logic
File myFile;
char resultFileName[] = "results.csv"; 

// dht 22 logic
const int dhtResponseSize = 4;
byte dhtData[dhtResponseSize] = { 0 };
byte dhtBytesReceived = 0;
// commands
enum {
    CMD_ID = 1,
    CMD_READ_TEMP  = 2,
    CMD_READ_HUMIDITY = 3
  };
unsigned long previousDhtHumidityPolling = 0;
unsigned long previousDhtTemperaturePolling = 0;
unsigned long dhtHumidityPollingInterval = 2000;
unsigned long dhtTemperaturePollingInterval = 3000;
float dhtTemperature;
float dhtHumidity;

// total sensor types
const int amountOfSensors = 4;
float sensorOutputResults[amountOfSensors];
char sensorNames[amountOfSensors][16] = {
  "CCS811  CO2", 
  "CCS811  Voc", 
  "DHT22  Temp",
  "DHT22  Humidity"
};
// order has to match sensorNames
char sensorValueUnit[amountOfSensors][5] = {
  "ppm", 
  "ppb", 
  "*C",
  "%"
};

// sensor polling interval
const unsigned long sensorPollingInterval = 15000;
unsigned long previousPollingMillis = 0;

// declare before setup so calling is possible => https://community.platformio.org/t/order-of-function-declaration/4546/2
void handleButtonPress();

void sendCommand (byte cmd, int responseSize)
  {
  Wire.beginTransmission (dht22Addr);
  Wire.write (cmd);
  Wire.endTransmission ();
  Wire.requestFrom (dht22Addr, dhtResponseSize);  
}

void setup() {
  // setup serial for debugging
  Serial.begin(9600);

  // setup wire for i2c to dht sensor
  Wire.begin (); 
  sendCommand (CMD_ID, 1);
  if (Wire.available ())
    {
    Serial.print ("Slave is ID: ");
    Serial.println (Wire.read (), HEX);
    }
  else
    Serial.println ("No response to ID request");
  // refresh random using analog pin 0
  // randomSeed(analogRead(0));
  // dhtHumidityPollingInterval = random(2000, 3000);
  // dhtTemperaturePollingInterval = random(2000, 3000);
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
  
  if(!myCCS811.start(CCS811_MODE_10SEC)){
    Serial.println("setup: CCS811 begin FAILED");
  }
  
  pinMode(10, OUTPUT);
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
  } else {
    Serial.println("Started SPI");
  }
  if (!SD.exists(resultFileName)) {
    Serial.print("File does not exist create and write header");
    myFile = SD.open(resultFileName, FILE_WRITE);
    // max header length is amount of sensors * 16 digits on display
    char header[amountOfSensors*16];
    for (int i = 0; i < amountOfSensors; i++)
    {
      if (i + 1 == amountOfSensors) {
        // the last time the loop will run so do not add a comma to the line
        strcat(header, sensorNames[i]);
      } else {
        strcat(header, sensorNames[i]);
        strcat(header, ",");
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
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
   if (interrupt_time - last_interrupt_time > 20)
  {
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) {
      menuSelectedSensor++;
    }
    last_interrupt_time = interrupt_time;
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
    }
    oldSelectedMenu = menuSelectedSensor;
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

float getDhtData(byte cmd){
  sendCommand(cmd, 4);
  dhtBytesReceived = Wire.available();                                   // count how many bytes received
  if (dhtBytesReceived == dhtResponseSize) {                                   // if received correct number of bytes...
      for (byte i=0; i<dhtResponseSize; i++) dhtData[i] = Wire.read();         // read and store each byte
      float result = *( (float*) dhtData);  
      return result;                                            // print the received data
  } else {                                                            // if received wrong number of bytes...
      Serial.print(F("\nRequested "));                                // print message with how many bytes received
      Serial.print(dhtResponseSize);
      Serial.print(F(" bytes, but got "));
      Serial.print(dhtBytesReceived);
      Serial.print(F(" bytes\n"));
  }
}

void collectDhtDataNotToFast() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousDhtHumidityPolling > dhtHumidityPollingInterval) {
    previousDhtHumidityPolling = currentMillis;
    dhtHumidity = getDhtData(CMD_READ_HUMIDITY);
    Serial.print("Measured humidity:");
    Serial.println(dhtHumidity);
  }
  if (currentMillis - previousDhtTemperaturePolling > dhtTemperaturePollingInterval) {
    previousDhtTemperaturePolling = currentMillis;
    dhtTemperature = getDhtData(CMD_READ_TEMP);
    Serial.print("Measured temperature:");
    Serial.println(dhtTemperature);
  }
}

void sensorLogic() {
  // start readout of DSM501A sensor
  collectDhtDataNotToFast();
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
    
    // get dht data
    sensorOutputResults[2] = dhtTemperature;
    sensorOutputResults[3] = dhtHumidity;
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
