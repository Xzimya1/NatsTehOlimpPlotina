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

#define RADIATION_PIN 21 // ПИН дозиметра
#define RAD_DEBOUNCE_US 100

DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;
StaticJsonDocument<200> doc;
Servo myservo;

#define PIN_SERVO 19 // ПИН СЕРВЫ

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

int ventilation_activity = 0; // АКТИВНОСТЬ ВЕНТИЛЯЦИИ (0 - выкл | 1 - вкл)
int door_condition = 0; // СОСТОЯНИЕ ДВЕРИ (0 - открыта | 1 - закрыта)

const int potPin = A0; // ПИН ПОТЕНЦИОМЕТРА


// для дозиметра
unsigned long radiationTimer = 0;
volatile int radiationCounts = 0;
const unsigned long RAD_INTERVAL = 5000;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  connectWiFi(ssid, password); 
  
  pinMode(RADIATION_PIN, INPUT);
  
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  dht.begin();
  myservo.attach(PIN_SERVO);
  
  stopMotor(); // инициализация шаговика
}

void loop() {
  unsigned long now = millis();
  
  updateMotor(now); // ASINC ШАГОВИК (Я ГЕНИЙ)
  
  updateRadiation(now); // ASINC ДОЗИМЕТР
  
  connectServer(serverIP, serverPort);   // Подключение к серверу
  
  // Считывание всех данных со всех портов
  float adc0 = analogRead(SOIL_MOISTURE);
  float HUM = map(adc0, air_value, water_value, moisture_0, moisture_100); // затопление
  float humidity = dht.readHumidity(); // влажность воздуха
  float temperature = dht.readTemperature(); //температура воздуха
  int pot = analogRead(potPin);

  // Датчик затопления грунтовыми водами
  if (HUM >= porog){
    Serial.println("УГРОЗА ЗАТОПЛЕНИЯ"); //заменить на функцию
  }
  doc["Number"] = 2;
  doc["HUM"] = round(HUM * 10) / 10.0;
  doc["HUM_AIR"] = round(humidity * 10) / 10.0;
  doc["TEM_AIR"] = round(temperature * 10) / 10.0;
  doc["stocks"] = pot;
  doc["ventilation_activity"] = ventilation_activity;
  doc["door_condition"] = door_condition;
  doc["dosimeter"] = radiationCounts * 12.0;
  
  serializeJson(doc, client); // ОТПРАВКА json
  client.println();

  // Общение с сервером
  unsigned long timeout = millis() + 500;
  String response = "";

  while (client.available() == 0 && millis() < timeout) {
    updateMotor(millis());  // Шаговик работает во время ожидания
    updateRadiation(millis()); // Дозиметр работает во время ожидания
    delay(1);
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
      ventilation_activity = 1;
      startMotor();  // запуск ASINC шаговика
    } else if(response.indexOf("ventilation_off") != -1) {
      ventilation_activity = 0;
      stopMotor();   // остановка ASINC шаговика
    } if(response.indexOf("CLOSE_DOOR") != -1) {
      myservo.write(170);
      delay(500);
      door_condition = 1;
    } else if(response.indexOf("OPEN_DOOR") != -1) {
      myservo.write(10);
      delay(500);
      door_condition = 0;
    } if(response.indexOf("DESTROY_BOMB_SHELTER") != -1) {
      destroy();
    }
  }

  delay(100);

}

// ШАГОВЫЙ ДВИГАТЕЛЬ ОТДЕЛЬНАЯ ФУНКЦИЯ
enum MotorState { MOTOR_IDLE, MOTOR_STEP1, MOTOR_STEP2, MOTOR_STEP3, MOTOR_STEP4 };
MotorState motorState = MOTOR_IDLE;
unsigned long motorTimer = 0;
int motorStepCount = 0;
const int stepsPerRev = 512;
const int stepDelay = 3;

void updateMotor(unsigned long now) {
  if (ventilation_activity == 0) {
    motorState = MOTOR_IDLE;
    return;
  }
  
  if (now - motorTimer >= stepDelay) {
    switch (motorState) {
      case MOTOR_STEP1:
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
        digitalWrite(in3, LOW); digitalWrite(in4, LOW);
        motorState = MOTOR_STEP2;
        break;
      case MOTOR_STEP2:
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW); digitalWrite(in4, LOW);
        motorState = MOTOR_STEP3;
        break;
      case MOTOR_STEP3:
        digitalWrite(in1, LOW); digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
        motorState = MOTOR_STEP4;
        break;
      case MOTOR_STEP4:
        digitalWrite(in1, LOW); digitalWrite(in2, LOW);
        digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
        motorState = MOTOR_STEP1;
        motorStepCount++;
        if (motorStepCount >= stepsPerRev) {
          motorStepCount = 0;
        }
        break;
      default:
        motorState = MOTOR_STEP1;
        break;
    }
    motorTimer = now;
  }
}

void startMotor() {
  motorState = MOTOR_STEP1;
  motorTimer = millis();
  motorStepCount = 0;
}

void stopMotor() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  motorState = MOTOR_IDLE;
  motorStepCount = 0;
}

// ОТБЕЛЬНАЯ ФУНКЦИЯ ДОЗИМЕТРА

void updateRadiation(unsigned long now) {
  static unsigned long measureStart = 0;
  static bool measuring = false;
  
  if (!measuring && now - radiationTimer >= RAD_INTERVAL) {
    radiationCounts = 0;
    measureStart = now;
    measuring = true;
    return;
  }
  
  if (measuring && now - measureStart >= 5000) {
    measuring = false;
    radiationTimer = now;
    
    Serial.println("Дозиметр CPM: " + String(radiationCounts * 12.0, 1));
    return;
  }
  
  if (measuring && digitalRead(RADIATION_PIN) == HIGH) {
    radiationCounts++;
    delayMicroseconds(RAD_DEBOUNCE_US);
  }
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
      return;
    }
    Serial.println("CONECTED TO " + String(serverIP) + ":" + String(serverPort));
  } 
}

void destroy(){
  Serial.println("БОМБОУБЕЖИЩЕ УНИЧТОЖЕНО");
  client.println("БОМБОУБЕЖИЩЕ УНИЧТОЖЕНО +100 К ПОТУЖНОСТИ");
}


