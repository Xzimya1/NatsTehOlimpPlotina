#include <WiFi.h>
#include <Wire.h>
#include <BH1750.h>
#include <VL53L0X.h>
#include <MGS_FR403.h>
#include <ArduinoJson.h>


VL53L0X lox;

WiFiClient client;
StaticJsonDocument<200> doc;

const char* ssid = "zopa";      // ← ОБЯЗАТЕЛЬНО ЗАМЕНИТЕ
const char* password = "00000001"; // ← ОБЯЗАТЕЛЬНО ЗАМЕНИТЕ
const char* serverIP = "10.142.155.22";    // ← ОБЯЗАТЕЛЬНО ЗАМЕНИТЕ на IP ВАШЕГО ПК
const int serverPort = 8888;


#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08

#define ledPin 19
int peopleCount = 0; //кол-во людей
uint16_t lastDist = 2000;   // Последнее расстояние (мм), изначально далеко
unsigned long lastCount = 0; // Время последнего срабатывания
int porog_srabat = 40; 
/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
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
}

void loop() {
  connectServer(serverIP, serverPort); // Подключение к серверу
  
  // Считывание всех данных со всех портов
  
  setBusChannel(0x07);
  float val7 = lox.readRangeSingleMillimeters();
  setBusChannel(0x06);
  float val6 = lox.readRangeSingleMillimeters();
  setBusChannel(0x05);
  float val5 = lox.readRangeSingleMillimeters();
  setBusChannel(0x04);
  float val4 = lox.readRangeSingleMillimeters();
  setBusChannel(0x03);
  float val3st = lox.readRangeSingleMillimeters();
  
  // Счетчик людей
  
  if (lastDist > 50 && val7 <= 50 && millis() - lastCount > 2000) {
    peopleCount++;                    // +1 чел
    lastCount = millis();             // обновление последнего значения
    digitalWrite(ledPin, HIGH);       // Включение подсветки входа
    Serial.println(String(peopleCount)); //выводим обновление кол-ва людей
  }

  if (digitalRead(ledPin) && millis() - lastCount > 500) {
    digitalWrite(ledPin, LOW);  // погашение подсветки через 500мс
  }
  
  lastDist = val7; // Сохраняем для следующей итерации
 
  // Формирование строк данных
  
  String data7 = "val 7 " + String(val7, 1) + " | ";
  String data6 = "val 6 " + String(val6, 1) + " | ";
  String data5 = "val 5 " + String(val5, 1) + " | ";
  String data4 = "val 4 " + String(val4, 1) + " | ";
  String data3 = "val 3 " + String(val3, 1) + " | ";
  String data = data7 + data6 + data5 + data4 + data3
  Serial.println(data);
  unsigned long timeout = millis() + 500;
  String response = "";
  
  // Формирование JSON FILE
  
  doc["7"] = data7;
  doc["6"] = data6;
  doc["5"] = data5;
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
    if (response.indexOf("1") != -1){
      // ДЕЙСТВИЕ 1
    } else if(response.indexOf("2") != -1) {
      // ДЕЙСТВИЕ 2
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
