#include <Arduino.h>

// wifi setup
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())
#define LED_ON      HIGH
#define LED_OFF     LOW
#include <ESP_WiFiManager.h>      
// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// SSID and PW for Config Portal
String AP_SSID = "AirQ-Station";
String AP_PASS = "start-up";

IPAddress stationIP   = IPAddress(192, 168, 2, 114);
IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);
IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);

// disable cloudflare
#define USE_ESP_WIFIMANAGER_NTP       false

// #include <MemoryFree.h>
// communication
// wire for i2c
#include <Wire.h> 
// SPI for like SPI
#include <SPI.h>
// display class
#include <LiquidCrystal_I2C.h>
// CCS811 class
#include "Adafruit_CCS811.h"
// SD for writing data to the SD card
#include <SD.h>

#include <Wire.h>

// libs for BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

bool sensorsReadyState = false;  // state if sensors are heated

const unsigned short sensorHeatupTime = 8000;   // time for sensors to heat up

const uint8_t buttonPin = 5;   // pin used for button to cycle in the menu
boolean buttonState = LOW;
boolean previousButtonState = LOW;
unsigned long previousButtonPollMillis = 0;
const  uint8_t buttonInterval = 20;
uint8_t menuSelectedSensor = 0;
uint8_t oldSelectedMenu = 0;

// digital or analog pins
const uint8_t ccs811wakepin = 5;

// i2c addresses
const uint8_t lcdAddr = 0x27;
const uint8_t ccs811Addr = 0x5A;
const uint8_t bme280Addr = 0x76;
// bootup screen dot logic
const unsigned short bootingDotsdelay = 1000;
unsigned long previousbootinMillis = 0;
unsigned int bootingdots = 0;
LiquidCrystal_I2C lcd(lcdAddr, 16, 2);

Adafruit_BME280 bme;
Adafruit_CCS811 ccs;

// for altitude calculation
#define SEALEVELPRESSURE_HPA (1025)

// file logic
File myFile;
const char resultFileName[] = "results.csv"; 
boolean sdcard_not_found = false;

// total sensor types
const uint8_t amountOfSensors = 6;
float sensorOutputResults[amountOfSensors];
const char sensorNames[amountOfSensors][14] = {
  "CO2", 
  "Voc", 
  "Temp",
  "Humidity",
  "BaroPressure",
  "Altitude"
};
// order has to match sensorNames
const char sensorValueUnit[amountOfSensors][4] = {
  "ppm", 
  "ppb", 
  "*C",
  "%",
  "hPa",
  "m"
};

// sensor polling interval
const unsigned short sensorPollingInterval = 15000;
unsigned long previousPollingMillis = 0;

// lcd variable
unsigned long previousButtonPressMillis = 0;
unsigned short lcdTimeToSleep = 15000;

TaskHandle_t AirQTaskHandle;

// declare before setup so calling is possible => https://community.platformio.org/t/order-of-function-declaration/4546/2
void handleButtonPress();
void AirQTask( void * pvParameters );

void upload_sensordata( void * pvParameters )
{
  while(true) {
    #define HEARTBEAT_INTERVAL    10000L
    // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("H");        // H means connected to WiFi
      HTTPClient http;
      http.begin("https://api.thingspeak.com/update");
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      String apiKey = "UIGRODKU4T99IGDJ";
      String httpRequestData = "api_key=" + apiKey + "&field1=" + String(random(40));   
      int httpResponseCode = http.POST(httpRequestData);
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      http.end();}
    else {
      Serial.print("F");        // F means not connected to WiFi
    }
    delay(HEARTBEAT_INTERVAL);
  }
}

void configModeCallback (ESP_WiFiManager *myESP_WiFiManager)
{
  Serial.print("Entered config mode with ");
  Serial.println("AP_SSID : " + myESP_WiFiManager->getConfigPortalSSID() + " and AP_PASS = " + myESP_WiFiManager->getConfigPortalPW());
}

void setup() {
  // setup serial for debugging
  Serial.begin(9600);
  // refresh random using analog pin 0
  // randomSeed(analogRead(0));
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
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }
  if (!bme.begin(bme280Addr)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    sdcard_not_found = true;
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


  // run on a dedicated CPU core 
  xTaskCreatePinnedToCore(
                    AirQTask,   /* Task function. */
                    "AirQTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &AirQTaskHandle,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
  delay(500); 

  Serial.println("\nStarting AutoConnectWithFeedBack");

  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager("AirQ-Station");

  //reset settings - for testing
  //ESP_wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  ESP_wifiManager.setAPCallback(configModeCallback);

  ESP_wifiManager.setDebugOutput(true);

  //set custom ip for portal
  ESP_wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 100, 1), IPAddress(192, 168, 100, 1), IPAddress(255, 255, 255, 0));

  ESP_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5+
  // ESP_wifiManager.setSTAStaticIPConfig(stationIP, gatewayIP, netMask, dns1IP, dns2IP);


  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  if (Router_SSID != "")
  {
    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println("Got stored Credentials. Timeout 120s");
  }
  else
  {
    Serial.println("No stored Credentials. No timeout");
  }

  // Get Router SSID and PASS from EEPROM, then open Config portal AP named "ESP_XXXXXX_AutoConnectAP" and PW "MyESP_XXXXXX"
  // 1) If got stored Credentials, Config portal timeout is 60s
  // 2) If no stored Credentials, stay in Config portal until get WiFi Credentials
  if (!ESP_wifiManager.autoConnect(AP_SSID.c_str(), AP_PASS.c_str()))
  {
    Serial.println("failed to connect and hit timeout");

    //reset and try again, or maybe put it to deep sleep
#ifdef ESP8266
    ESP.reset();
#else   //ESP32
    ESP.restart();
#endif
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("WiFi connected");

  xTaskCreatePinnedToCore(
                    upload_sensordata,   /* Function to implement the task */
                    "uploadSensorData", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */
 
  Serial.println("Task created...");

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

void storageLogic() {
  myFile = SD.open(resultFileName, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    String row = "";
    for (int i =0; i < amountOfSensors; i++) {
      if (i + 1 == amountOfSensors) {
        // the last time the loop will run so do not add a comma to the line
        row.concat(String(sensorOutputResults[i]));
      } else {
        row.concat(String(sensorOutputResults[i]) + ",");
        }
    }
    myFile.println(row);
    // close the file:
    myFile.close();
  } else {
  // if the file didn't open, print an error:
    Serial.println("error opening result file");
  }
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
        Serial.println("ERROR!");
        while(1);
      }
    }
    
    sensorOutputResults[2] = bme.readTemperature();
    sensorOutputResults[3] = bme.readHumidity();
    sensorOutputResults[4] = bme.readPressure() / 100.0F;
    sensorOutputResults[5] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    // save sensor values to SD Card to sd card
    if (!sdcard_not_found) {
      storageLogic();
    }
  }

}

void start() {
  view();
  sensorLogic();
}

void AirQTask( void * pvParameters ) {
  // wait for 10 seconds for all sensors to get ready one time
  for (;;){
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
}


void loop() {

}