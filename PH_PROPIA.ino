#define PH_PIN 36
float voltage,phValue,temperature = 25;

//float acidVoltage = 1870;
//float neutralVoltage = 1420;

float acidVoltage = 2850;
float neutralVoltage = 2100;

void setup() {
  Serial.begin(115200);
}

void loop() {
  static unsigned long timepoint = millis();
  if(millis() - timepoint>3000U){
    timepoint = millis();
    voltage = analogRead(PH_PIN)/4095.0*5000;
    float slope = (6.8-4.0)/((neutralVoltage-1500)/3.0-(acidVoltage-1500)/3.0);
    float intercept = 6.8-slope*(neutralVoltage-1500)/3.0;

    phValue = slope*(voltage-1500)/3.0+intercept; //recta

    Serial.print("Voltage: ");
    Serial.println(voltage,1);
    Serial.print("  pH: ");
    Serial.println(phValue,2);
  }
}
