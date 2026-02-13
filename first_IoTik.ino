#include <WiFi.h>
#include <Wire.h>
#include <BH1750.h>
#include <VL53L0X.h>
#include <MGS_FR403.h>
#include <ArduinoJson.h>
#include <ESP32_Servo.h>


VL53L0X lox;
BH1750 LightSensor;
WiFiClient client;
StaticJsonDocument<200> doc;
Servo myservo;

const char* ssid = "zopa";
const char* password = "00000001";
const char* serverIP = "10.142.155.22";
const int serverPort = 8888;


#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08
#define SOIL_MOISTURE    34 // пин влажности почвы !!! ИЗМЕНИТЬ ПРИ ПОДКЛЮЧЕНИИ !!!


#define PIN_SERVO = 19 // ЗАМЕНИТЬ СЕРВО


#define ledPin 19 // подсветка


// для счетчика
int peopleCount = 0; //кол-во людей
uint16_t lastDist = 2000;   // Последнее расстояние (мм), изначально далеко
unsigned long lastCount = 0; // Время последнего срабатывания 
int night = 0;
int piep = 0;

//для датчика затопления
const float air_value    = 1587.0;
const float water_value  = 800.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;
const float porog = 70.0;
 
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
  setBusChannel(0x07);
  conlox(lox);
  setBusChannel(0x07);
  LightSensor.begin();
  myservo.attach(PIN_SERVO);
}

void loop() {
  connectServer(serverIP, serverPort);   // Подключение к серверу
  
  // Считывание всех данных со всех портов
  
  setBusChannel(0x07);
  float val7 = lox.readRangeSingleMillimeters();
  setBusChannel(0x06);
  float val6 = LightSensor.readLightLevel();
  setBusChannel(0x05);
  float adc0 = analogRead(SOIL_MOISTURE);
  float val5 = map(adc0, air_value, water_value, moisture_0, moisture_100);
  setBusChannel(0x04);
  float val4 = lox.readRangeSingleMillimeters();
  setBusChannel(0x03);
  float val3st = lox.readRangeSingleMillimeters();
  
  // Счетчик людей
  
  if (lastDist > 50 && val7 <= 50 && millis() - lastCount > 2000) {
    peopleCount++;                       // +1 чел
    lastCount = millis();                // обновление последнего значения
    digitalWrite(ledPin, HIGH);          // Включение подсветки входа
    Serial.println(String(peopleCount)); //выводим обновление кол-ва людей
    piep = 1;
  }

  if (digitalRead(ledPin) && millis() - lastCount > 500 && night == 0) {
    digitalWrite(ledPin, LOW);           // погашение подсветки через 500мс
    piep = 0;
  }
  
  lastDist = val7;                       // Сохраняем для следующей итерации

  // Включение подсветки ночью

  if (val6 < 200) {
    night = 1;
    pinMode(ledPin, HIGH);
  } else if(val6 >= 200 && piep == 0){
    pinMode(lenPin, LOW)
  }

  // Датчик затопления грунтовыми водами

  if (val5 >= porog){
    Serial.println("УГРОЗА ЗАТОПЛЕНИЯ") //заменить на функцию
  }
 
  // Формирование строк данных для отправки
  
  String data7 = "Distant " + String(val7, 1) + " | ";
  String data6 = "illumination " + String(val6, 1) + " | ";
  String data5 = "Hum " + String(val5, 1) + " | ";
  String data4 = "val 4 " + String(val4, 1) + " | ";
  String data3 = "val 3 " + String(val3, 1) + " | ";
  String data = data7 + data6 + data5 + data4 + data3
  Serial.println(data);
  unsigned long timeout = millis() + 500;
  String response = "";
  
  // Формирование JSON FILE
  
  doc["Distant"] = data7;
  doc["illumination"] = data6;
  doc["Hum"] = data5;
  doc["4"] = data4;
  doc["3"] = data3;
  doc["lastCount"] = lastCount;

  // Общение с сервером

  
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
    if (response.indexOf("CLOSE_DOOR") != -1){
      myservo.write(170);
      delay(500);
    } else if(response.indexOf("OPEN_DOOR") != -1) {
      myservo.write(10);
      delay(500);
    } else if(response.indexOf("3") != -1) {
      // ДЕЙСТВИЕ 3
    } else if(response.indexOf("4") != -1) {
      // ДЕЙСТВИЕ 4
    } else if(response.indexOf("5") != -1) {
      // ДЕЙСТВИЕ 5
    }
  }
  delay(500);
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

bool conlox(lox){
  lox.init();
  lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);
}
