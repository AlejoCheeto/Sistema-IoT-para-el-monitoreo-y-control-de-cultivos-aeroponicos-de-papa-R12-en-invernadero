#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Bonezegei_RS485.h>
#include <Bonezegei_SoftSerial.h>
#include <ArduinoJson.h>

// Declaración de los pines 
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

/*void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i) {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i) {
                if (i != 0)
                    Serial.print("-");
                printHex2(nwkKey[i]);
            }
            Serial.println();
        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}*/

void setup() {
  Serial.begin(9600); //Config. del serial a 9600 bauds
  Serial.println("Inicializando LoRa... ");

  os_init(); //Inicialización de lmic
  LMIC_reset(); //Reset del estado MAC

  //Desactivación de todas las sub-bandas
  for (int b = 0; b < 8; b++) {
      LMIC_disableSubBand(b);
    }

  // Activación del canal a utilizar (Utilizado también por el Gateway)
  LMIC_enableChannel(1); // 903.9 MHz

  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  // Inicialización del módulo RS-485
  Serial.println("Inicializando RS-485");
  softSerial.begin(9600); // Inicialización de softSerial a 9600 bauds, igual que el puerto serial
  rs485.begin(9600); // Inicialización del objeto rs485 a 9600 bauds
  Serial.println("RS-485 inicializado");

  // Configuración del pin DE como salida
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW); // Inicialmente en modo recepción
}

void loop() {
  os_runloop_once(); //Se ejecuta el loop LMIC

  if(LMIC.dataLen){ //LMIC.dataLen revisa si hay datos recibidos por LoRaWAN

  /*En caso de que los datos no vengan en jSON usar este método de adquisición*/  
  /****************************************/
    /*float temperature = LMIC.frame[0];
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
    Serial.println(light);*/
  /****************************************/

  /*En caso de que los datos vengan en una trama jSON*/
  /****************************************/
    // Convertir datos entrantes a una cadena jSON
    char jsonBuffer[LMIC.dataLen + 1]; // Declaración del Buffer donde LMIC.dataLen es la long de los datos y +1 para agregar el carácter nulo
    memcpy(jsonBuffer, LMIC.frame, LMIC.dataLen); // Copia LMIC.dataLen desde el LMIC.frame al jsonBuffer 
    jsonBuffer[LMIC.dataLen] = '\0'; // Se agrega el carácter nulo necesario para tener una cadena C válida

    // Procesamiento de la cadena jSON
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, jsonBuffer);

    if(error){
      Serial.print("Error al deserializar jSON: ");
      Serial.println(error.c_str());
      return;
    }

    //Extracción de los datos jSON - Se debe de cambiar los nombres de acuerdo a la trama
    float temperature = doc["temperature"];
    float humidity = doc["humidity"];
    float pH = doc["pH"];
    float light = doc["light"];

    // Impresión de los datos obtenidos luego del procesamiento de la cada jSON
    Serial.print("Datos recibidos - Temp: ");
    Serial.print(temperature);
    Serial.print(", Hum: ");
    Serial.print(humidity);
    Serial.print(", pH: ");
    Serial.print(pH);
    Serial.print(", Luz: ");
    Serial.println(light);
  /****************************************/

  /*Envio de datos por RS485 utilizando el objeto rs485*/
  /****************************************/

    // Activación del modo transmisión
    //digitalWrite(RS485_DE, HIGH);

        // Convertir los valores float a cadenas de caracteres
        char tempBuffer[20];
        char humBuffer[20];
        char phBuffer[20];
        char lightBuffer[20];

        dtostrf(temperature, 6, 2, tempBuffer);  // 6 caracteres de ancho, 2 decimales
        dtostrf(humidity, 6, 2, humBuffer);
        dtostrf(pH, 6, 2, phBuffer);
        dtostrf(light, 6, 2, lightBuffer);

        // Crear el mensaje final
        char message[100];
        snprintf(message, sizeof(message), "Temp:%s,Hum:%s,pH:%s,IntLum:%s", tempBuffer, humBuffer, phBuffer, lightBuffer);

        // Enviar el mensaje completo por RS-485
        sendRS485(message);

    Serial.println("Datos enviados por Rs-485.");

    //digitalWrite(RS485_DE, LOW); // Desactivación del modo de transmisión
  /****************************************/
  }
}
