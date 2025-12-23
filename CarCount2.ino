#include <BH1750.h>
#include <Wire.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <MGS_FR403.h>
#include <String.h>
MGS_FR403 Fire;
VL53L0X lox;

#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08


BH1750 LightSensor_1;


long long int n = 0;
long long int n1 = 0;
int k = 0;
int flag = 1;
int cars_min = 0;
int result_time = 0;
int g = 0;
float charge = 0;
int hit = 1;
int statr = 0;
int statr1 = 0;

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/

void setup() {

  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  setBusChannel(0x07);
  Wire.begin();
  lox.init();
  lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);
  
  setBusChannel(0x06);
  Serial.begin(115200);
  Wire.begin();
  Fire.begin();
  
  setBusChannel(0x03); // функция смены I2C-порта
  LightSensor_1.begin();
}

void loop() {
  setBusChannel(0x07);
  float dist = lox.readRangeSingleMillimeters();
  delay(5);
  if (dist < 50) {
    n += 1;
  }
  if (n > 0 && dist > 50){
    k += 1;
    n = 0;
    Serial.println(k);
  }
  
  if ((millis() / 1000 / 60) >= statr){
    statr += 1;
    Serial.println(String(k) + " Cars in min! on 1 road");
    k = 0;
  }
  


  setBusChannel(0x03);
  float light1 = LightSensor_1.readLightLevel();
  delay(5);
  if (light1 < 10) {
    n1 += 1;
  }
  if (n1 > 0 && light1 > 10){
    g += 1;
    n1 = 0;
    Serial.println(String(g) + " luxi");
  }
  
  if ((millis() / 1000 / 60) >= statr1){
    statr1 += 1;
    Serial.println(String(g) + " Cars in min! on 2 road");
    g = 0;
  }

  






  // setBusChannel(0x06);
  // Fire.get_ir_and_vis();
  // String inf = String(Fire.ir_data, 1);
  // Serial.println(inf);
  // String vis = String(Fire.ir_data, 1);
  // Serial.println(vis);
  // delay(500);
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