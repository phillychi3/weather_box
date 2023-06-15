#include <DHT.h>
#include <ESP8266WiFi.h>
//#include <Firebase_ESP_Client.h>
#include <Adafruit_BMP085.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
//#include "env.h"

// --- -- --- Settings --- -- ---
//DHT11
const int Pin_DHT11 = D5;
//GPS
const int Pin_GPSTx = D6;
const int GPSSerialBaud = 9600;  // must 9600
//BMP085
// SCL to D1
// SDA to D2
//ZPH02
const int Pin_ZPH02Tx = D7;
const int ZHPSerialBaud = 9600;  // must 9600
//OTHER
const int SerialBaud = 115200;

// --- -- --- Real Datas --- -- ---
float RD_Temperature = 0;  // 溫度 (°C)
int RD_Pressure = 0;       // 氣壓 (Pa)
float RD_Altitude = 0;     // 高度 (m)
int RD_Humidity = 0;       // 濕度 (%)
float RD_PM25 = 0;         // PM2.5 (%)
float RD_GPSLat = 0;       // 緯度 (°)
float RD_GPSLng = 0;       // 經度 (°)

// --- -- --- Sensors --- -- ---

SoftwareSerial ZPHSerial(Pin_ZPH02Tx, 0);
bool UpdateZPH02() {
  while (ZPHSerial.available() > 8) {
    // get bits
    byte tmp[8];
    int index = 0;
    for (int i = 0; i < 8; i++) {
      tmp[i] = ZPHSerial.read();
      if (tmp[i] == 0x18) index = i;
    }

    // bit alignment
    byte data[8];
    for (int i = 0; i < 8; i++) {
      data[i] = tmp[(i + index + 7) % 8];
    }
    for (int i = 0; i < 8; i++) {
      Serial.print(data[i]);
      Serial.print(" ");
    }

    // if unknow
    if (data[0]!=0xFF || data[1]!=0x18) return false;

    RD_PM25 = data[3]+data[4]*0.01;
    return true;
  }
  return false;
}

TinyGPSPlus gps;
SoftwareSerial GPSSerial(Pin_GPSTx, 0);
bool UpdateGPS() {
  bool isUpdated = false;
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read()) && gps.location.isUpdated()) {
      RD_GPSLat = gps.location.lat();
      RD_GPSLng = gps.location.lng();
      isUpdated = true;
    }
  }
  return isUpdated;
}  //OK

DHT dht11(Pin_DHT11, DHT11);
bool UpdateDHT11() {

  float h = dht11.readHumidity();

  if (isnan(h))
    return false;

  RD_Humidity = h;
  return true;
}  //OK

Adafruit_BMP085 bmp085;
bool UpdateBMP085() {

  if (!bmp085.begin())
    return false;

  RD_Temperature = bmp085.readTemperature();
  RD_Pressure = bmp085.readPressure();
  RD_Altitude = bmp085.readAltitude();

  return true;
}  //OK

// call this to update all real data
void UpdateAllRealData() {
  UpdateGPS();
  UpdateDHT11();
  UpdateBMP085();
  UpdateZPH02();
}

// --- -- --- Main --- -- ---
void setup() {
  // Serial
  Serial.begin(SerialBaud);
  //DHT11
  dht11.begin();
  //GPS
  GPSSerial.begin(GPSSerialBaud);
  //ZHP02
  ZPHSerial.begin(ZHPSerialBaud);
}
void loop() {

  Serial.print(UpdateGPS());
  Serial.print("]");
  Serial.print(RD_GPSLat);
  Serial.print("°,");
  Serial.print(RD_GPSLng);
  Serial.print("° | ");

  Serial.print(UpdateDHT11());
  Serial.print("]");
  Serial.print(RD_Humidity);
  Serial.print("% | ");

  Serial.print(UpdateBMP085());
  Serial.print("]");
  Serial.print(RD_Temperature);
  Serial.print("°C,");
  Serial.print(RD_Pressure);
  Serial.print("Pa,");
  Serial.print(RD_Altitude);
  Serial.print("m | ");

  Serial.print(UpdateZPH02());
  Serial.print("]");
  Serial.print(RD_PM25);
  Serial.println("%");


  delay(500);
}