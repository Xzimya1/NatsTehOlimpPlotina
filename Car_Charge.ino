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


int n = 0;
int k = 0;
int flag = 1;
int cars_min = 0;
int result_time = 0;
int g = 0;
float charge = 0;
int hit = 1;

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
}

void loop() {
  setBusChannel(0x07);
  float dist = lox.readRangeSingleMillimeters();
  delay(100);
  if (dist < 50 & k == 0) {
    n += 1;
  }
  if (n > 0 && dist > 50 && k == 0){
    k += 1;
    n = 0;
    Serial.println(k);
  }
  if (k == 1){
    if (hit == 1){
      g = int(millis() / 1000);
      hit = 0;
    }
    if (int(millis() / 1000) == g + 1){
      g += 1;
      charge += 1.6666;
      }
    if (charge + 1.6666 >= 100){
      Serial.println("Charge Is Done  100%!");
      k = 0;
      hit = 1;
    } else {
      Serial.println(String(charge) + "%");
    }

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
