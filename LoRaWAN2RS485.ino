#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Bonezegei_RS485.h>
#include <Bonezegei_SoftSerial.h>
#include <ArduinoJson.h>

// Declaración de los pines del radio SX1276
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 }
};

#define RS485_RX 3 //Pin RX asignado al MAX485 en la LoRa32
#define RS485_TX 1 //Pin TX asignado al MAX485 en la LoRa32
#define RS485_DE 34 //Pin de activación de transmisión del MAX485

// Credenciales LoRaWAN
static const u1_t PROGMEM APPEUI[8] = { 0x08, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 };
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8] = { 0x6A, 0x93, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = { 0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x77, 0x77, 0x88, 0x88 };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Creación de los objetos softSerial y RS485 de Bonezegei
Bonezegei_SoftSerial softSerial(RS485_RX,RS485_TX); //definición de los puertos seriales
Bonezegei_RS485 rs485(softSerial,RS485_DE); //definición del objeto rs485 llamando a los pines seriales

void sendRS485(const char* data) {
    // Activar modo transmisión
    digitalWrite(RS485_DE, HIGH);

    // Enviar cada carácter uno por uno
    while (*data) {
        rs485.write(*data++);
    }
    // Desactivar modo transmisión
    digitalWrite(RS485_DE, LOW);
}

void LoRaWAN_bandSelect (int channel){

  //Desactivación de todas las sub-bandas
  for (int b = 0; b < 8; b++) {
      LMIC_disableSubBand(b);
  }

  LMIC_enableChannel(channel);
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
}

void RS485_init(){
  Serial.println("Inicializando RS-485");
  softSerial.begin(9600); // Inicialización de softSerial a 9600 bauds, igual que el puerto serial
  rs485.begin(9600); // Inicialización del objeto rs485 a 9600 bauds
  Serial.println("RS-485 inicializado");

  // Configuración del pin DE como salida
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW); // Inicialmente en modo recepción
}

void processLoraData() {
  /*En caso de que los datos vengan en una trama jSON*/
  char jsonBuffer[LMIC.dataLen + 1]; 
  memcpy(jsonBuffer, LMIC.frame, LMIC.dataLen); 
  jsonBuffer[LMIC.dataLen] = '\0'; 

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonBuffer);

  if(error){
    Serial.print("Error al deserializar jSON: ");
    Serial.println(error.c_str());
    return;
  }

  float temperature = doc["temperature"];
  float humidity = doc["humidity"];
  float pH = doc["pH"];
  float light = doc["light"];

  Serial.print("Datos recibidos - Temp: ");
  Serial.print(temperature);
  Serial.print(", Hum: ");
  Serial.print(humidity);
  Serial.print(", pH: ");
  Serial.print(pH);
  Serial.print(", Luz: ");
  Serial.println(light);

  sendDataRS485(temperature, humidity, pH, light);
}

// En caso de que los datos no vengan en jSON
void processLoraDataRaw() {
  float temperature = LMIC.frame[0];
  float humidity = LMIC.frame[1];
  float pH = LMIC.frame[2];
  float light = LMIC.frame[3];

  Serial.print("Datos recibidos - Temp: ");
  Serial.print(temperature);
  Serial.print(", Hum: ");
  Serial.print(humidity);
  Serial.print(", pH: ");
  Serial.print(pH);
  Serial.print(", Luz: ");
  Serial.println(light);

  sendDataRS485(temperature, humidity, pH, light);
}

void sendDataRS485(float temperature, float humidity, float pH, float light) {
  /*Envio de datos por RS485 utilizando el objeto rs485*/
  char tempBuffer[20];
  char humBuffer[20];
  char phBuffer[20];
  char lightBuffer[20];

  dtostrf(temperature, 6, 2, tempBuffer);  
  dtostrf(humidity, 6, 2, humBuffer);
  dtostrf(pH, 6, 2, phBuffer);
  dtostrf(light, 6, 2, lightBuffer);

  char message[100];
  snprintf(message, sizeof(message), "Temp:%s,Hum:%s,pH:%s,IntLum:%s", tempBuffer, humBuffer, phBuffer, lightBuffer);

  sendRS485(message);

  Serial.println("Datos enviados por Rs-485.");
}

/*void howSendData(){
  typedef enum {Json,NoJson} STATE_
  static STATE_ state = ini;

  if(1){
    switch(state){
      case ini:
        processLoRaData();
        state = Json;
      break;
      case Json:

      break;
      case NoJson:

      break;
    }
  }
}*/

void setup() {
  Serial.begin(9600); //Config. del serial a 9600 bauds
  Serial.println("Inicializando LoRa...");

  os_init(); //Inicialización de lmic
  LMIC_reset(); //Reset del estado MAC

  LoRaWAN_bandSelect (1); //Seleccion de la banda de LoRaWAN
  RS485_init();
}

void loop() {
  os_runloop_once(); //Se ejecuta el loop LMIC

  if(LMIC.dataLen){
    processLoraData();
  }

}
