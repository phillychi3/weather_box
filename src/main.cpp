#include <Arduino.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_BMP085.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "env.h"
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "bdPtfI5SmpYP0qhwcw2utqZgMl6nz-mR0zUGuXNyUEiymc2qIwooTH2ihr8fpxB1ZTw5pgEKV_SjTWazJ4v5Wg=="
#define INFLUXDB_ORG "71786aeb37a484b6"
#define INFLUXDB_BUCKET "weatherbox"
#define DEVICE "ESP8266"
#define DHTPIN D5
#define DHTTYPE DHT11


#define TZ_INFO "	CST-8"
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
Point sensor("wifi_status");
bool signupOK = false;
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
int RD_Pressure = 0;     // 氣壓 (Pa)
float RD_Altitude = 0;     // 高度 (m)
int RD_Humidity = 0;       // 濕度 (%)
float RD_PM25 = 0;         // PM2.5 (%)
float RD_GPSLat = 0; // 緯度 (°)
float RD_GPSLng = 0; // 經度 (°)

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
SoftwareSerial GPSSerial(Pin_GPSTx,0);
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
}//OK

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

void setup() {
  // --- start up ---
  Serial.begin(9600);
  Serial.println();
  Serial.println("------------------------------------");
  Serial.println("Weather Station 0.0.1");
  Serial.println("Author: phillychi3,LZHYUAN,maxkirito");
  Serial.println("------------------------------------");
  Serial.println("Weather Station is starting...");
  // --- wifi ---
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

  // --- database ---
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("success login to firebase");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("sensor starting...");

  // --- sensors ---
  //DHT11
  dht11.begin();
  //GPS
  GPSSerial.begin(GPSSerialBaud);
  //ZHP02
  ZPHSerial.begin(ZHPSerialBaud);
  Serial.println("sensor started");
}


void loop() { 
  sensor.clearFields();
  UpdateAllRealData();
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    Firebase.RTDB.setFloat(&fbdo, "Temperature", RD_Temperature);
    Firebase.RTDB.setInt(&fbdo, "Pressure", RD_Pressure);
    Firebase.RTDB.setFloat(&fbdo, "Altitude", RD_Altitude);
    Firebase.RTDB.setInt(&fbdo, "Humidity", RD_Humidity);
    Firebase.RTDB.setFloat(&fbdo, "PM25", RD_PM25);
    Firebase.RTDB.setFloat(&fbdo, "GPSLat", RD_GPSLat);
    Firebase.RTDB.setFloat(&fbdo, "GPSLng", RD_GPSLng);
    Serial.println("upload to firebase");
  }
  sensor.addField("Temperature", RD_Temperature);
  sensor.addField("Pressure", RD_Pressure);
  sensor.addField("Altitude", RD_Altitude);
  sensor.addField("Humidity", RD_Humidity);
  sensor.addField("PM25", RD_PM25);
  sensor.addField("GPSLat", RD_GPSLat);
  sensor.addField("GPSLng", RD_GPSLng);
  Serial.print("Writing: ");
  Serial.println(sensor.toLineProtocol());
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  delay(2000);
}


