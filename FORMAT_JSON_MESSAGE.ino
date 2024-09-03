float t_val = 15.439;
float h_val = 61.029;
float pH_val = 6.789;
float lux_val = 1524.69;


static char mydata[100];

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
}

void loop() {
  t_val = random(9,22);
  h_val = random(0,100);
  lux_val = random(450,1200);
  pH_val = random(4,6);
  sprintf(mydata,"\"deviceID\":\"Aeronodo\",\"temperatura\":\"%.2f\",\"humedad\":\"%.2f\",\"lux\":\"%.2f\",\"pH\":\"%.2f\"", t_val, h_val, lux_val, pH_val);
  Serial.println((char*)mydata);
  delay(1000); // add a delay to avoid flooding the serial port
}