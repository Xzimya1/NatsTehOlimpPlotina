#include <WiFi.h>
#include <Wire.h>
#include <BH1750.h>
#include <VL53L0X.h>
#include <MGS_FR403.h>
#include <ArduinoJson.h>

VL53L0X lox;
BH1750 LightSensor;
WiFiClient client;
StaticJsonDocument<200> doc;

const char* ssid = "zopa";
const char* password = "00000001"; 
const char* serverIP = "10.142.155.22";
const int serverPort = 8888;

#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08


#define ledPin 19 // подсветка
#define RedPin 1 // RED LIGHG
#define YellowPin 2 // Yellow LIGHG
#define GreenPin 3 // Green LIGHG

int alarm_status = 0; // СОСТОЯНИЕ СИГНАЛИЗАЦИИ (0 - ЗЕЛЕНЫЙ | 1 - ЖЕЛТЫЙ | 2 - КРАСНЫЙ)

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
  connectWiFi(ssid, password);  
  setBusChannel(0x07);
  conlox(lox);
  setBusChannel(0x07);
  LightSensor.begin();
  pinMode(RedPin, OUTPUT); pinMode(YellowPin, OUTPUT);
  pinMode(GreenPin, OUTPUT); pinMode(ledPin, OUTPUT);
  pinMode(RedPin, LOW); pinMode(YellowPin, LOW);
  pinMode(GreenPin, LOW); pinMode(ledPin, LOW);
}

void loop() {
  connectServer(serverIP, serverPort);   // Подключение к серверу
  
  // Считывание всех данных со всех портов
  
  setBusChannel(0x07);
  float val7 = lox.readRangeSingleMillimeters();
  setBusChannel(0x06);
  float val6 = LightSensor.readLightLevel();
  
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
    digitalWrite(ledPin, HIGH); 
  } else if(val6 >= 200 && piep == 0){
    digitalWrite(ledPin, LOW);
  }

  // Формирование строк данных для отправки

  String data7 = "Distant " + String(val7, 1) + " | ";
  String data6 = "illumination " + String(val6, 1) + " | ";
  String data = data7 + data6;  // ✅ ИСПРАВЛЕНО: добавлена точка с запятой
  Serial.println(data);

  // Формирование JSON FILE

  doc["Distant"] = round(val7 * 10) / 10.0;      
  doc["illumination"] = round(val6 * 10) / 10.0; 
  doc["lastCount"] = lastCount;
  doc["alarm_status"] = alarm_status;
  
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

  serializeJson(doc, client); //ОТПРАВКА JSON
  client.println();
  
  if (response.length() > 0) {
    Serial.print("Получено : ");
    Serial.println(response);
    if (response.indexOf("1") != -1){
      //ДЕЙСТВИЕ 1
    } if(response.indexOf("2") != -1) {
      //ДЕЙСТВИЕ 2
    } if(response.indexOf("RED") != -1) {
      RED(RedPin, YellowPin, GreenPin);
      alarm_status = 2;
    } else if(response.indexOf("YELLOW") != -1) {
      YELLOW(RedPin, YellowPin, GreenPin);
      alarm_status = 1;
    } else if(response.indexOf("GREEN") != -1) {
      GREEN(RedPin, YellowPin, GreenPin);
      alarm_status = 0;
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


void connectWiFi(const char* ssid, const char* password)
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


void connectServer(const char* serverIP, int serverPort)
{
   if (!client.connected()) {
    Serial.println("Connecting to server " + String(serverIP) + ":" + String(serverPort));  
    if (!client.connect(serverIP, serverPort)) {
      Serial.println("Not connected. Whait 3 seconds...");
      delay(3000);  
    }
    Serial.println("CONECTED TO " + String(serverIP) + ":" + String(serverPort)); 
  }  
}


void conlox(VL53L0X& lox){
  lox.init();
  lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);
  return true; 
}


void RED(const int RedPin, const int YellowPin, const int GreenPin){
  digitalWrite(RedPin, HIGH);
  digitalWrite(YellowPin, LOW);
  digitalWrite(GreenPin, LOW);
}


void YELLOW(const int RedPin, const int YellowPin, const int GreenPin){
  digitalWrite(RedPin, LOW);
  digitalWrite(YellowPin, HIGH);
  digitalWrite(GreenPin, LOW);
}

void GREEN(const int RedPin, const int YellowPin, const int GreenPin){
  digitalWrite(RedPin, LOW);
  digitalWrite(YellowPin, LOW);
  digitalWrite(GreenPin, HIGH);
}

