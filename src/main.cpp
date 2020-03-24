#include <Arduino.h>
// communication
// wire for i2c
#include <Wire.h> 
// SPI for like yea SPI
#include <SPI.h>
// display class
#include <LiquidCrystal_I2C.h>
// CCS811 class
#include <SparkFunCCS811.h>
// SD for writing data to the SD card
#include <SD.h>


bool sensorsReadyState = false;  // state if sensors are heated

const unsigned long sensorHeatupTime = 10000;   // time for sensors to heat up

const unsigned int buttinpin = 7;   // pin used for button to cycle in the menu
const unsigned int mq135pin = A0;   // connection pin for MQ135

// i2c addresses
const uint8_t lcdAddr = 0x38;
const uint8_t ccs811Addr = 0x5B;

// bootup screen dot logic
const unsigned long bootingDotsdelay = 1000;
unsigned long previousbootinMillis = 0;
unsigned int bootingdots = 0;

LiquidCrystal_I2C lcd(lcdAddr, 16, 2);
CCS811 myCCS811(ccs811Addr);

File myFile;

// sensor polling interval
const unsigned long sensorPollingInterval = 4000;
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
  myCCS811.begin();
  Serial.println("Started myCCS811");
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
}

void sensorLogic() {
  // call 
  unsigned long currentMillis = millis();
  if(currentMillis - previousPollingMillis > sensorPollingInterval) {
    previousPollingMillis = currentMillis; 
    if (myCCS811.dataAvailable())
    {
      myCCS811.readAlgorithmResults();
      int tempCO2 = myCCS811.getCO2();
      int tempVOC = myCCS811.getTVOC();
      Serial.print("CO2:");
      Serial.println(tempCO2);
      Serial.print("Total Volatile Organic Compounds");
      Serial.println(tempVOC);
    }
    else if (myCCS811.checkForStatusError())
    {
      Serial.print("Status error");
    }
  }
}

void storageLogic() {

}

void start() {
  view();
  sensorLogic();
  storageLogic();
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

