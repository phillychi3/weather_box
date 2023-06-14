#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_BMP085.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SD_ZH03B.h>
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"
#include "env.h"

#define DHTPIN D5
#define DHTTYPE DHT11
Adafruit_BMP085 bmp;
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

SoftwareSerial ZHSerial(D3, D4);
SD_ZH03B ZH03B( ZHSerial );



static const int RXPin = D6, TXPin = D7;
TinyGPSPlus gps;
SoftwareSerial SerialGPS(RXPin, TXPin); 
static const uint32_t GPSBaud = 9600;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

void setup() {
  Serial.begin(9600);
  SerialGPS.begin(GPSBaud);
  Serial.println("------------------------------------");
  Serial.println("Weather Station 0.0.00000000001");
  Serial.println("Author: phillychi3,LZHYUAN,maxkirito");
  Serial.println("------------------------------------");
  Serial.println("Weather Station is starting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("success login to firebase");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  Serial.println("sensor starting...");
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
  Serial.println("sensor started");

  ZHSerial.begin(9600);
  ZH03B.setMode( SD_ZH03B::IU_MODE );
  Serial.println();
  Serial.println("Weather Station is ready");
}

void readSensorData() {
char printbuf1[80];
   
  if( ZH03B.readData() ) {
    Serial.print( ZH03B.getMode() == SD_ZH03B::IU_MODE ? "IU:" : "Q&A:" );  
    sprintf(printbuf1, "PM1.0, PM2.5, PM10=[%d %d %d]", ZH03B.getPM1_0(), ZH03B.getPM2_5(), ZH03B.getPM10_0() );
    Serial.println(printbuf1);
  } else {   
    Serial.println( "ZH03B Error reading stream or Check Sum Error" );
  } 
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


void loop() { 
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    if (Firebase.RTDB.setInt(&fbdo, "test/int", count)){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    count++;
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" m");


    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
    }
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
    }
    while (SerialGPS.available() > 0)
      if (gps.encode(SerialGPS.read()))
        displayInfo();
    Serial.println( "Wake up" );
    if( ZH03B.wakeup() ) Serial.println( "Woke up successfully" );
    readSensorData();
    ZH03B.setInitiativeMode();
    readSensorData();
  }
}


