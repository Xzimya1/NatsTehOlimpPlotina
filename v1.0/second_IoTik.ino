#include <WiFi.h>
#include <Wire.h>
#include <BH1750.h>
#include <VL53L0X.h>
#include <MGS_FR403.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <ESP32_Servo.h>

// DHT22 настройки
#define DHTPIN 2      // Пин DATA
#define DHTTYPE DHT22 // Тип датчика DHT22/AM2302

DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;
StaticJsonDocument<200> doc;
Servo myservo;

#define PIN_SERVO = 19 // ЗАМЕНИТЬ СЕРВО

const char* ssid = "zopa";
const char* password = "00000001";
const char* serverIP = "10.142.155.22";
const int serverPort = 8888;


#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08
#define SOIL_MOISTURE    34 // пин влажности почвы !!! ИЗМЕНИТЬ ПРИ ПОДКЛЮЧЕНИИ !!!


//для датчика затопления
const float air_value    = 1587.0;
const float water_value  = 800.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;
const float porog = 70.0;

//для мотора

#define in1 4
#define in2 5
#define in3 6
#define in4 7
int dl = 1;

int ventilation_activity = 0; .// АКТИВНОСТЬ ВЕНТИЛЯЦИИ (0 - выкл | 1 - вкл)
int door_condition = 0; // СОСТОЯНИЕ ДВЕРИ (0 - открыта | 1 - закрыта)
/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL) - ДАТЧИК РАССТОЯНИЯ
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL) - ДАТЧИК ОСВЕЩЕННОСТИ
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/


void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  connectWiFi(ssid password);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  dht.begin();
  myservo.attach(PIN_SERVO);
}

void loop() {
  connectServer(serverIP, serverPort);   // Подключение к серверу
  
  // Считывание всех данных со всех портов
  float adc0 = analogRead(SOIL_MOISTURE);
  float HUM = map(adc0, air_value, water_value, moisture_0, moisture_100); // затопление
  float humidity = dht.readHumidity(); // влажность воздуха
  float temperature = dht.readTemperature(); //температура воздуха

  
  // Датчик затопления грунтовыми водами

  if (HUM >= porog){
    Serial.println("УГРОЗА ЗАТОПЛЕНИЯ") //заменить на функцию
  }
 
  // Формирование строк данных для отправки
  
  String HUM = "HUM " + String(HUM, 1) + " | ";
  String HUM_AIR = "HUM_AIR " + String(humidity, 1) + " | ";
  String TEM_AIR = "TEM_AIR " + String(temperature, 1) + " | ";
  
  String data = HUM + HUM_AIR + TEM_AIR
  Serial.println(data);

  
  // Формирование JSON FILE
  
  doc["HUM"] = HUM;
  doc["HUM_AIR"] = HUM_AIR;
  doc["TEM_AIR"] = TEM_AIR;
  doc["ventilation_activity"] = ventilation_activity;
  doc["door_condition"] = door_condition;


  // Общение с сервером

  unsigned long timeout = millis() + 500;
  String response = "";
  while (client.available() == 0 && millis() < timeout) {
    delay(10);
  }

  while (client.available()) {
    char c = client.read();
    if (c == '\n' || c == '\r') break;
    response += c;
  }

  if (response.length() > 0) {
    Serial.print("Получено : ");
    Serial.println(response);
    if (response.indexOf("ventilation_on") != -1){
      motor_activity = 1;
    } else if(response.indexOf("ventilation_off") != -1) {
      motor_activity = 0;
    } else if(response.indexOf("CLOSE_DOOR") != -1) {
      myservo.write(170);
      delay(500);
      door_condition = 1;
    } else if(response.indexOf("OPEN_DOOR") != -1) {
      myservo.write(10);
      delay(500);
      door_condition = 0;
    } else if(response.indexOf("5") != -1) {
      // ДЕЙСТВИЕ 5
    }
  }
  delay(500);
  client.println(doc);
}



bool setBusChannel(uint8_t i2c_channel)
{
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);
    Wire.endTransmission();
    return true;
  }
}

bool connectWiFi(ssid, password)
{
  WiFi.begin(ssid, password);
  Serial.print("Podklyuchenie k ");
  Serial.println(ssid);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi podklyuchen!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nOshibka Wi-Fi");
    while (1) delay(1000);
  }  
}

bool connectServer(serverIP, serverPort)
{
   if (!client.connected()) {
    Serial.println("\n Connecting to server " + String(serverIP,1) + ":" + String(serverPort, 1));
    if (!client.connect(serverIP, serverPort)) {
      Serial.println("Not connected. Whait 3 seconds...");
      delay(3000);
      return;
    }
    Serial.println("CONECTED TO " + String(serverIP,1) + ":" + String(serverPort, 1));
  } 
}
