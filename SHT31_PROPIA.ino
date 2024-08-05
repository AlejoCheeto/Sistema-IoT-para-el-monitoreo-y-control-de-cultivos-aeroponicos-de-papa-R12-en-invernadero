#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#define sht_add 0x44

float data_validation(float data, int lw_lim, int up_lim){
  if(!isnan(data)){
    if(data > lw_lim){
      if(!(data < up_lim)){
        Serial.println("DATA VALIDATION ERROR:Data out of upper limit");
        data = 803;
      }
    }else{
      Serial.println("DATA VALIDATION ERROR:Data out of lower limit");
      data = 802;
    }
  }else{
    Serial.println("DATA VALIDATION ERROR: NaN reading of variable ");
    data = 801;
  }

  return data;
}

void SHT31_read(float* data_array,Adafruit_SHT31& sht_obj ){
  float t,h;

  t = sht_obj.readTemperature();
  h = sht_obj.readHumidity();

  t = data_validation(t,0,165);
  h = data_validation(h,0,100);

  data_array[0] = t;
  data_array[1] = h;

  Serial.print("Temp °C =");
  Serial.println(t);

  Serial.print("Hum % = ");
  Serial.println(h);
}

void SHT31_setup(Adafruit_SHT31& sht_obj){
  Serial.print("\n");
  Serial.print("SHT31 status:");
  if(!sht_obj.begin(0X44)){
    Serial.println("Impossible to reach SHT31...");
    while(1){
      delay(1);
    }
  } else{
    Serial.println("OK");
  }

  Serial.print("Heater status:");
  if(sht_obj.isHeaterEnabled()){
    Serial.println("ENABLE");
  }else{
    Serial.println("DISABLE");
  }

}

bool enableHeater = false;
uint8_t loopCnt = 0;
float data[2];
float t;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

void setup() {
  Serial.begin(9600);
  SHT31_setup(sht31);

}

void loop() {
 SHT31_read(data,sht31);
 delay(5000);
}
