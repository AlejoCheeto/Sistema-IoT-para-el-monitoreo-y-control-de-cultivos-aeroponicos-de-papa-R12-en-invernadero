/* Ruler 1         2         3         4         5         6         7        */
/******************************************************************************/
/*                                                                            */
/*   +----+ +----+                          110924                            */
/*   ++  ++ ++******                                                          */
/*    |  |   |**  **      This code is used for a IoT monitoring node         */
/*    |  |   | *  *       for an aeroponic potato module using a TTGO Lora32  */
/*    |  |   | *  *       board.This node measures temperature, relative      */
/*    |******| *  *       humidity, pH and light intensity and that data      */
/*    |**  **| *  *       throught LoRaWAN network to a gateway.The LoRaWAN   */                                                  
/*    ++*  ** **  *       connection is is based on MCCI LoRAWAN LCIM library */
/*     +**  ***  **                                                           */
/*      +**     **        DOCUMENTED BY: Alejandro Pachón Garzón              */                                            
/*        *******                        paalejandro@javeriana.edu.co         */
/*                                                                            */
/*                        IMPLEMENTED BY: Juan Esteban Rodríguez Hernández    */
/*                                        jesteban.rodriguez@javeriana.edu.co */
/*                                        Alejandro Pachón Garzón             */
/*                                        paalejandro@javeriana.edu.co        */
/*                                                                            */
/*                        Bogota, D.C., September 26th, 2024.                 */                               
/*          Copyright (C) Department de Electronics                           */
/*                        School of Engineering                               */
/*                        Pontificia Universidad Javeriana                    */
/*                        Bogota - Colombia - South America                   */
/*                                                                            */
/******************************************************************************/
 // Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 // Copyright (c) 2018 Terry Moore, MCCI
//************************************************************************************************************
// LIBRARIES ------------------------------------------------------------------------------
#include <lmic.h>
#include <hal/hal.h>
#include <string>
//************************************************************************************************************
// CONTROL VARIABLES ----------------------------------------------------------------------------------------------------
// CONTROL RELAY
const int led = 4;
//************************************************************************************************************
// LoRaWAN LMIC LIBRARY VARIABLES ----------------------------------------------------------------------------
// * Include comments from initial implementation
//
// This EUI must be in little-endian format, so least-significant-byte first.
static const u1_t PROGMEM APPEUI[8] = { 0x07, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 }; //Gateway Indoor
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little-endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x6A, 0x93, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format.
static const u1_t PROGMEM APPKEY[16] = { 0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x77, 0x77, 0x11, 0x11 }; //Gateway Indoor
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[] = "Lorito";
static osjob_t sendjob;

// Radio Pins - lmic
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 }
};

//************************************************************************************************************
// WATCHDOG TIMER --------------------------------------------------------------------------------------------

/* Necesitas implementar esta wea */

//************************************* MCCI LoRaWAN FUNCTIONS ***********************************************
//************************************************************************************************************

/*FN****************************************************************************
*
*   void printHex2(unsigned v)
*
*   Purpose: Print the value of v in hexadecimal format, padding single-digit 
*            values with a leading zero.
*
*   Parameters:
*       v: The unsigned integer value to be printed in hexadecimal format.
*
*   Plan: Mask the higher bits of v, check for single-digit values, and print 
*         the result in hexadecimal format.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE     COMMENT
*   -----------------------------------------------------------------------
*   Sep 18/2017  T.Moore, MCCI   Initial implementation
*   Sep 11/2024  A.Pachón        Documentation
*
*******************************************************************************/
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}
/*FN****************************************************************************
*
*   void onEvent(ev_t ev)
*
*   Purpose: Handle different events related to LoRaWAN communication, such as 
*            joining, transmitting, and receiving data.
*
*   Parameters:
*       ev: The event type, which determines the action to be taken.
*
*   Plan: Use a switch statement to handle different events, printing 
*         descriptive messages to the serial console and performing 
*         necessary actions.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE     COMMENT
*   -----------------------------------------------------------------------
*   Sep 18/2017  T.Moore, MCCI   Initial implementation
*   Sep 25/2024  A.Pachón        receiveMessage and LMIC.dataLen inclusion
*   Sep 26/2024  A.Pachón        Documentation
*
*******************************************************************************/
void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
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
            Serial.println("JOIN-------------------------------------");
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
        break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println("TXCOMPLETE-----------------------------------------");
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen != 0) {
                // Datos recibidos. Extraer el número de puerto si lo hay.
                u1_t bPort = 0;
                if (LMIC.txrxFlags & TXRX_PORT)
                    bPort = LMIC.frame[LMIC.dataBeg - 1];
                // Llamar a la función proporcionada por el usuario con el puerto, el mensaje y la longitud del mensaje
                receiveMessage(bPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            }
            // Programar la próxima transmisión
            os_setCallback(&sendjob, do_send);
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("EV_RXCOMPLETE"));
            if (LMIC.dataLen != 0) {
                // Datos recibidos
                u1_t bPort = 0;
                if (LMIC.txrxFlags & TXRX_PORT)
                    bPort = LMIC.frame[LMIC.dataBeg - 1];
                // Llamar a la función proporcionada por el usuario con el puerto, el mensaje y la longitud del mensaje
                receiveMessage(bPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
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
            Serial.println((unsigned)ev);
            break;
    }
}
/*FN****************************************************************************
*
*   void do_send(osjob_t* j)
*
*   Purpose: Prepare and send an upstream data transmission at the next possible time.
*
*   Parameters:
*       j: The OS job that triggered this function.
*
*   Plan: Check if there is no current TX/RX job running, prepare the data transmission,
*         and schedule the next transmission.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE     COMMENT
*   -----------------------------------------------------------------------
*   Sep 18/2017  T.Moore, MCCI   Initial implementation
*   Sep 26/2024  A.Pachón        Documentation
*
*******************************************************************************/
void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Preparar datos para la transmisión
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        Serial.println(F("Packet queued"));
    }
}
/*FN****************************************************************************
*
*   receiveMessage(uint8_t bPort, uint8_t* message, uint8_t messageLength)
*
*   Purpose: Receives and processes a message from a given port, printing the 
*            message to the Serial Monitor. Depending on the message content, 
*            it controls the state of an LED.
*
*   Parameters:
*       bPort          : Port number where the message was received
*       message        : Pointer to the array containing the message data
*       messageLength  : Length of the received message
*
*   Plan: This function reads the received message, stores it in a `std::string`,
*         and prints it on the Serial Monitor. If the message contains the value
*         "1", it turns the LED on; otherwise, it turns the LED off.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE    COMMENT
*   -----------------------------------------------------------------------
*   Sep 26/2024  A.Pachón       Initial Implementation
*   Sep 26/2024  A.Pachón       Documentation
*
*******************************************************************************/


void receiveMessage(uint8_t bPort, uint8_t* message, uint8_t messageLength) {
    Serial.print("Received message on port ");
    Serial.println(bPort);
    Serial.print("Message: ");
    std::string receivedMessage = "";

    for (uint8_t i = 0; i < messageLength; i++) {
        Serial.print((char)message[i]);
        receivedMessage += (char)message[i];
    }
    Serial.println();

    if (receivedMessage == "1") {
        digitalWrite(led, HIGH);
        Serial.println("LED ON");
    } else {
        digitalWrite(led, LOW);
        Serial.println("LED OFF");
    }
}

//*******************************************************************************

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));
    // Inicializar LMIC
    os_init();
    // Resetear el estado de MAC
    LMIC_reset();

    pinMode(led,OUTPUT);

    // Deshabilitar todos los sub-bands
    for (int b = 0; b < 8; b++) {
        LMIC_disableSubBand(b);
    }
    // Habilitar el canal deseado
    LMIC_enableChannel(1); // 903.9 MHz

    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

    // Iniciar trabajo (envío inicia OTAA automáticamente)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
