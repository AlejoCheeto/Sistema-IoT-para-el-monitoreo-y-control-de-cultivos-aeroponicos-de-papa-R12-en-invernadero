#include "Wire.h"
#define address 0x23                 //I2C address 0x23
#define lux_reg 0x10                 //LUX register 0x10
uint8_t readReg(uint8_t reg, const void* pBuf, size_t size){
  if (pBuf == NULL) {
    Serial.println("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  Wire.beginTransmission(address);
  Wire.write(&reg, 1);
  if ( Wire.endTransmission() != 0) {
    return 0;
  }
  delay(20);
  Wire.requestFrom(address, (uint8_t) size);
  for (uint16_t i = 0; i < size; i++) {
    _pBuf[i] = Wire.read();
  }
  return size;
}

float data_validation(float data, int lw_lim, int up_lim){
  if(!isnan(data)){
    if(data > lw_lim){
      if(!(data < up_lim)){
        Serial.print("DATA VALIDATION ERROR:Data out of upper limit");
        data = 803;
      }
    }else{
      Serial.print("DATA VALIDATION ERROR:Data out of lower limit");
      data = 802;
    }
  }else{
    Serial.print("DATA VALIDATION ERROR: NaN reading of variable ");
    data = 801;
  }

  return data;
}

float LUX_read(const void* pBuf, size_t size){
  uint16_t data;
  float lux;

  uint8_t * _pBuf = (uint8_t *)pBuf;
  readReg(lux_reg, pBuf, size);
  data = _pBuf[0] << 8 | _pBuf[1];
  lux = (((float)data)/1.2);
  lux = data_validation(lux,1,65535);
  Serial.print("LUX lx =");
  Serial.println(lux);

  return lux;

}
void setup()
{
  Serial.begin(9600);
  Wire.begin();
}
uint8_t buf[4] = {0};
float lux;

void loop(){
  lux = LUX_read(buf,2);
  delay(3000);
}
