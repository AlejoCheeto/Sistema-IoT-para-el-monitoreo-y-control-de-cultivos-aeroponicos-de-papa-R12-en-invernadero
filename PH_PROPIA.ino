#define PH_PIN 39
float data_validation(float data, int lw_lim, int up_lim){
  if(!isnan(data)){
    if(data >= lw_lim){
      if(!(data <= up_lim)){
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

float voltage,phValue,temperature = 25;

//5V
//float acidVoltage = 1870;
//float neutralVoltage = 1420;
//3V3
float acidVoltage = 1980;
float neutralVoltage = 1560;


void setup() {
  Serial.begin(115200);
}

void loop() {
  static unsigned long timepoint = millis();
  if(millis() - timepoint>6000){
    timepoint = millis();
    voltage = analogRead(PH_PIN)/4095.0*3300;
    float slope = (6.9-4.0)/((neutralVoltage-1500)/3.0-(acidVoltage-1500)/3.0);
    float intercept = 6.9-slope*(neutralVoltage-1500)/3.0;

    phValue = slope*(voltage-1500)/3.0+intercept; //recta
    phValue = data_validation(phValue,0,14);
    Serial.print("Voltage: ");
    Serial.println(voltage,1);
    Serial.print("  pH: ");
    Serial.println(phValue,2);
  }
}
