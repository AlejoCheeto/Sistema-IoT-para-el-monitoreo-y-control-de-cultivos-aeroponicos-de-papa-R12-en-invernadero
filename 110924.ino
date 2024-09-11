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
/*      +**     **        DOCUMENTED BY: Juan Esteban Rodríguez Hernández     */                                            
/*        *******                        jesteban.rodriguez@javeriana.edu.co  */
/*                                                                            */
/*                        IMPLEMENTED BY: Juan Esteban Rodríguez Hernández    */
/*                                        jesteban.rodriguez@javeriana.edu.co */
/*                                        Alejandro Pachón Garzón             */
/*                                        palejandro@javeriana.edu.co         */
/*                                                                            */
/*                        Bogota, D.C., September 11th, 2024.                 */                               
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
#include <SPI.h>

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

#include "esp_system.h"
#include "rom/ets_sys.h"
//************************************************************************************************************
// SENSORS FMS STATE DEFINITION ------------------------------------------------------------------------------
typedef enum
{
  INI_WDT,
  SHT31_READ,
  KTD_1,
  LUX_READ,
  KTD_2,
  PH_READ,
  KTD_3,
  MEAN,
  SEND_DATA,
  KTD_4,
  HEATER,
  H_OFF,
  H_ON,
  KTD_5,
  DELAY,
  KTD_6
} sensors_states;
//************************************************************************************************************
#define address 0x23
#define lux_reg 0x10
#define PH_PIN 36
//************************************************************************************************************
// LoRaWAN LMIC LIBRARY VARIABLES ----------------------------------------------------------------------------
// * Include comments from initial implementation
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
//static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70 };
//static const u1_t PROGMEM APPEUI[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x08 };
static const u1_t PROGMEM APPEUI[8] = { 0x07, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 };
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little-endian format, see above.
//static const u1_t PROGMEM DEVEUI[8] = { 0xD3, 0x92, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
//static const u1_t PROGMEM DEVEUI[8] = { 0x6A, 0x95, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8] = { 0x6A, 0x92, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x77, 0x77, 0x11, 0x11 };
//static const u1_t PROGMEM APPKEY[16] = { 0x88, 0x88, 0x77, 0x77, 0x66, 0x66, 0x55, 0x55, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11 };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

//static uint8_t mydata[] = "\"deviceID\":\"Aeronodo\",\"temperatura\":\"88888\",\"humedad\":\"88888\",\"lux\":\"88888\",\"pH\":\"88888\"";
static uint8_t mydata[] = "\"device\":\"Aeronodo\",\"temperatura\":\"88888\",\"humedad\":\"88888\",\"lux\":\"88888\",\"pH\":\"888888\"";
//static uint8_t mydata[] = "\"aaaaaa\":\"Aeronodo\",\"temperatura\":\"88888\",\"humedad\":\"88888\",\"lux\":\"88888\",\"pH\":\"88888\"";
static osjob_t sendjob;
const int maxSize = 100;
uint8_t msg_cast[maxSize];

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 }
};
//************************************************************************************************************
// WATCHDOG TIMER --------------------------------------------------------------------------------------------
const int wdtTimeout = 50000;    //Watchdog timer timeout in ms
hw_timer_t *wdt_timer = NULL;
//************************************************************************************************************
// SENSORS FSM VARIABLES ---------------------------------------------------------------------------------------------
//*****************************************************
// ONLY MODIFY THE VARIABLES INSIDE THIS BOX
// NUMBER OF READING PER SAMPLE
const int c = 10; // Number or readings after compute an averaged sample

// DELAY TIME
const int delay_time = 2000;    // Delay time between each data measurment (ms)

// SHT30 timeouts variables
const int heater_wait_timeout_min = 2;//min 
const int heater_active_timeout_min = 1;//min
//*****************************************************
// MISC
int count = 0;                  // FSM cycles counter
unsigned long int prev_time;    // Keep the previous registered time to compare during DELAY state
int loraflag = 1;               // Flag to excute LoRaWAN connection, it´s 0 when a message is upload to the network
//HEATER VARIABLES
unsigned long int heater_time;  // Keep track of SHT30 heater active and wait timeout
// SHT30 TIMEOUTS VARIABLES
const int heater_wait_timeout_ms = heater_wait_timeout_min*60000;//ms
const int heater_active_timeout_ms =  heater_active_timeout_min*60000;//ms
//*****************************************************
// MEASURMENT VARIABLES
// SHT30 READING VARIABLES ----------------------------------------------
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // SHT object to interact with SHT30
float sht_vec[2] ;              // SHT30 reading result vector [0]:Temperature (°C) and [1]:Humidity (%RH)

// LUX READING VARIABLES ------------------------------------------------
uint8_t buf[4] = {0};           // Light intensity sensor´s reading buffer
float lux_val = -1;             // Light intensity reading variable (lux)

// PH READING VARIABLES -------------------------------------------------
// This values were obtained by a calibration process with a set of buffered pH solutions
// CHANGE THEM ONLY IF NECCESSARY !!
float ph_acid = 4.0;
float acid_voltage = 2870;
float ph_neutral = 6.8;
float neutral_voltage = 2130;    
float ph_val;                   // pH reading varaible (pH)

// VARIABLES ------------------------------------------------------------
float t_vec[c],h_vec[c],lux_vec[c],ph_vec[c]; //Array for keep the readings before average them
float t_mean = 0,h_mean = 0, lux_mean = 0, ph_mean = 0; //Resulting average for each measured variable

//************************************************************************************************************
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
*   Sep 11/2024  J.Rodríguez     Documentation
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
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 18/2017  T.Moore, MCCI   Initial implementation
*   Sep 11/2024  J.Rodríguez     Documentation
*
*******************************************************************************/
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
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
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 18/2017  T.Moore, MCCI   Initial implementation
*   Sep 10/2024  J.Rodríguez     Alive message inclusion
*   Sep 11/2024  J.Rodríguez     Documentation
*
*******************************************************************************/
void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    Serial.println("------DO SEND");
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        LMIC_sendAlive();
        loraflag = 0;
        timerWrite(wdt_timer,0);
        Serial.println((char*)mydata);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//************************************************************************************************************
//******************************************** DATA HANDLING *************************************************
//************************************************************************************************************

/*FN****************************************************************************
*
*   float data_validation(float data, int lw_lim, int up_lim)
*
*   Purpose: Validate read data is into the given range [lw_lim,up_lim].
*
*   Parameters:
*       data   : Input data to be validated
*       lw_lim : Lower limit for the data validation
*       up_lim : Upper limit for the data validation
*
*   Plan: Check if there is the data is not NaN or it´s outside of the 
*         range. Return an error code according if data is not validated
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Agu  3/2024  J.Rodríguez     Initial implementation
*   Sep 11/2024  J.Rodríguez     Documentation
*
*******************************************************************************/
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
/*FN****************************************************************************
*
*   float mean(float* vec, int size)
*
*   Purpose: Calculate the mean of an array of floats.
*
*   Parameters:
*       vec: The array of floats.
*       size: The size of the array.
*
*   Returns:
*       The calculated mean value.
*
*   Plan: Check if the array is not empty, then iterate over the array to calculate the sum,
*         and finally divide the sum by the size to get the mean.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
float mean(float* vec, int size){
 if (size == 0) {
    Serial.println("mean: Empty array");
    return 0; 
  }

  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += vec[i];
  }

  float average = sum / size;
  return average;
}
/*FN****************************************************************************
*
*   float median(float vec[], int size
*
*   Purpose: Calculate the median of an array of floats.
*
*   Parameters:
*       vec: The array of floats.
*       size: The size of the array.
*
*   Returns:
*       The calculated median value.
*
*   Plan: Check if the array is not empty, then sort the array and extract
*         it´s median value.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
float median(float vec[], int size){
  float med;
  int mid;
  if (size == 0) {
    Serial.println("mean: Empty array");
    return 0; 
  }
  // INSERTION SORT
  for(int i=1; i<size; i++){
    int key = vec[i];
    int j = i - 1;
    while(j >= 0 && vec[j]>key){
      vec[j+1] = vec[j];
      j = j-1;
    }
    vec[j+1] = key;
  }

  if(size % 2 == 0){
  //even size
    mid = int(size/2);
    med = (vec[mid]+vec[mid+1])/2;
  } else if (size % 2 != 0){
  //odd size
    mid = int(size/2);
    med = vec[mid];
  }

  return med;
}

//************************************************************************************************************
//********************************************** SHT SENSOR **************************************************
//************************************************************************************************************

/*FN****************************************************************************
*
*   void SHT31_setup(Adafruit_SHT31& sht_obj)
*
*   Purpose: Initialize the SHT31 sensor and check the heater status.
*
*   Parameters:
*       sht_obj: The Adafruit_SHT31 object to be initialized.
*
*   Returns:
*       None
*
*   Plan: Initialize the SHT31 sensor using the begin method, and 
*         then check the heater status.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
void SHT31_setup(Adafruit_SHT31& sht_obj){
  Serial.print("\n");
  Serial.print("SHT30 status:");
  if(!sht_obj.begin(0X44)){
    Serial.println("SHT30: Impossible to reach SHT31...");
    while(1){
      delay(1);
    }
  } else{
    Serial.println("OK");
  }

  Serial.print("Heater status:");
  if(sht_obj.isHeaterEnabled()){
    Serial.println("SHT30 HEATER: ENABLE");
  }else{
    Serial.println("SHT30 HEATER: DISABLE");
  }

}

/*FN****************************************************************************
*
*   void SHT31_read(float* data_array, Adafruit_SHT31& sht_obj)
*
*   Purpose: Read temperature and humidity data from the SHT31 sensor, validate 
*            the data, and store it in an array.
*
*   Parameters:
*       data_array: The array to store the temperature and humidity values.
*       sht_obj: The Adafruit_SHT31 object to read data from.
*
*   Returns:
*       None
*
*   Plan: Read temperature and humidity values, validate the data, and store 
*         it in the array.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
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

//************************************************************************************************************
//********************************************** LUX SENSOR **************************************************
//************************************************************************************************************

/*FN****************************************************************************
*
*   uint8_t readReg(uint8_t reg, const void* pBuf, size_t size)
*
*   Purpose: Read data from a specific register of an I2C lux sensor.
*
*   Parameters:
*       reg: The register address to read from.
*       pBuf: The buffer to store the read data.
*       size: The number of bytes to read.
*
*   Returns:
*       The number of bytes read.
*
*   Plan: Start I2C transmission, write register address, read data, 
*         and store it in the buffer.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   N/A          DFRobots     Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
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

/*FN****************************************************************************
*
*   float LUX_read(const void* pBuf, size_t size)
*
*   Purpose: Read data from the lux sensor, calculate the lux value
*            and validate it.
*
*   Parameters:
*       pBuf: The buffer to store the read data.
*       size: The number of bytes to read.
*
*   Returns:
*       The calculated lux value.
*
*   Plan: Read data from lux register, calculate lux value, validate it
*         and return it.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
float LUX_read(const void* pBuf, size_t size){
  uint16_t data;
  float lux;

  uint8_t * _pBuf = (uint8_t *)pBuf;
  readReg(lux_reg, pBuf, size);
  data = _pBuf[0] << 8 | _pBuf[1];
  lux = (((float)data)/1.2);
  lux = data_validation(lux,0,65535);
  Serial.print("LUX lx =");
  Serial.println(lux);

  return lux;

}

//************************************************************************************************************
//*********************************************** pH SENSOR **************************************************
//************************************************************************************************************

/*FN****************************************************************************
*
*   float PH_read(float ph_acid, float acid_voltage, float ph_neutral, float neutral_voltage)
*
*   Purpose: Read the voltage from the pH sensor, calculate the pH value 
*            using a linear calibration curve, and validate it.
*
*   Parameters:
*       ph_acid: The pH value for an acidic solution.
*       acid_voltage: The voltage value for an acidic solution.
*       ph_neutral: The pH value for a neutral solution.
*       neutral_voltage: The voltage value for a neutral solution.
*
*   Returns:
*       The calculated pH value.
*
*   Plan: Read voltage from pH sensor, calculate pH value using linear calibration 
*         curve, validate it, and return it.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
float PH_read(float ph_acid,float acid_voltage, float ph_neutral, float neutral_voltage){
  static unsigned long timepoint = millis();
  float slope, intercept, ph_value, voltage;

  voltage = analogRead(PH_PIN)/4095.0*5000;
  slope = (ph_neutral - ph_acid)/((neutral_voltage-1500)/3.0 - (acid_voltage-1500)/3.0);
  intercept = ph_neutral - slope*(neutral_voltage-1500)/3.0;

  ph_value = slope*((voltage-1500)/3.0)+intercept;
  ph_value = data_validation(ph_value,0,14);

  Serial.print("pH = ");
  Serial.println(ph_value);

  return ph_value;
}

//************************************************************************************************************
//****************************************** WATCHDOG TIMER **************************************************
//************************************************************************************************************

/*FN****************************************************************************
*
*   void resetModule()
*
*   Purpose: Reset the ESP32 module.
*
*   Plan: Print "reboot" to the console and restart the ESP32 module.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

//************************************************************************************************************
//********************************************* PRINCIPAL FSM ************************************************
//************************************************************************************************************

/*FN****************************************************************************
*
*   void FSM_sensors(Adafruit_SHT31& sht_obj,hw_timer_t* wdt_timer)
*
*   Purpose: Read, validate temperature, relative humidity, light intensity,
*             and pH values, average them and send throught LoRaWAN network
*             to a gateway
*
*   Plan: Finite state machine to read and validate the mentioned read 
          variables,average them and send then through LoRaWAN
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE  COMMENT
*   -----------------------------------------------------------------------
*   Sep 10/2024  J.Rodríguez  Initial Implementation
*   Sep 11/2024  J.Rodríguez  Documentation
*
*******************************************************************************/
void FSM_sensors(Adafruit_SHT31& sht_obj,hw_timer_t* wdt_timer){
  static sensors_states sen_st = INI_WDT;
    switch(sen_st){
      case INI_WDT:
        //Serial.println("INI_WDT");
        timerWrite(wdt_timer,0);
        heater_time = millis();
        sen_st = SHT31_READ;
        break;
      case SHT31_READ:
        //Serial.println("SHT31_READ");
        SHT31_read(sht_vec,sht_obj);
        sen_st = KTD_1;
        break;
      case KTD_1:
        //Serial.println("KTD_1");
        timerWrite(wdt_timer,0);
        sen_st = LUX_READ;
        break;
      case LUX_READ:
        //Serial.println("LUX_READ");
        // Sensor´s connection validation
        if ( Wire.requestFrom(address,2,0) != 0) {
          lux_val = LUX_read(buf,2);
        }
        else{
          lux_val = -1;
          Serial.println("LUX SENSOR: I2C device is not found...");
          Serial.println("LUX SENSOR: Check the illuminance sensor connection");
        }
        sen_st = KTD_2;
        break;
      case KTD_2:
        //Serial.println("KTD_2");
        timerWrite(wdt_timer,0);
        sen_st = PH_READ;
        break;
      case PH_READ:
        //Serial.println("PH READ");
        ph_val = PH_read(ph_acid,acid_voltage,ph_neutral,neutral_voltage);
        sen_st = KTD_3;
        break;
      case KTD_3:
        //Serial.println("KTD_3");
        timerWrite(wdt_timer,0);
        count++; // Count that a sensors reading loop was completed
        sen_st = MEAN;
        break;
      case MEAN:
        Serial.print("Loop number # : ");
        Serial.println(count);
        if(count <= c){
          // Insert the read values into the corresponding array
          t_vec[count-1] = sht_vec[0];
          h_vec[count-1] = sht_vec[1];
          lux_vec[count-1] = lux_val;
          ph_vec[count-1] = ph_val;
          sen_st = KTD_4;
        }else if (count >= c){
          // Once the FSM reach to the c_th loop it average the content of the vector and restart the count
          t_mean = mean(t_vec,c-1);
          h_mean = mean(h_vec,c-1);
          lux_mean = mean(lux_vec,c-1);
          ph_mean = mean(ph_vec,c-1); 
          count = 0;
          // Format the message to send using LoRaWAN
          sprintf((char*)mydata,"\"device\":\"Aeronodo\",\"temperatura\":\"%.2f\",\"humedad\":\"%.2f\",\"lux\":\"%.2f\",\"pH\":\"%.2f\"", t_mean, h_mean, lux_mean, ph_mean);
          sen_st = SEND_DATA;
        }
        break;
      case SEND_DATA:
        Serial.println("SEND_DATA");
        loraflag = 1;
        // Call the LoRaWAN FSM onEvent until it connects and a message is transmitted
        // See do_send function to see where loraflag is write to 0/False and thus, exit the while loop
        while (loraflag){
          os_runloop_once();
        }
        sen_st = KTD_4;
        break;
      case KTD_4:
        //Serial.println("KTD_4");
        timerWrite(wdt_timer,0);
        sen_st = HEATER;
        break;
      case HEATER:
        //Serial.println("HEATER");
        if(sht_obj.isHeaterEnabled())
        {
          sen_st = H_ON;
        } 
        else
        {
          sen_st = H_OFF;
        }
        break;
      case H_ON:
        //Serial.println("H_ON"); 
        if((millis()-heater_time)>heater_active_timeout_ms)
        {
          Serial.println("HEATER OFF -------------------------");
          sht_obj.heater(0);
          heater_time = millis();
        }
        sen_st = KTD_5;
        break;
      case H_OFF:
        //Serial.println("H_OFF");
        if((millis()-heater_time>heater_wait_timeout_ms))
        {
          Serial.println("HEATER ON -------------------------");
          sht_obj.heater(1);
          heater_time = millis();
        }
        sen_st = KTD_5;
        break;
      case KTD_5:
        //Serial.println("KTD_5");
        timerWrite(wdt_timer,0);
        prev_time = millis();
        sen_st = DELAY;
        break;
      case DELAY:
        //Serial.println("DELAY");
        if((prev_time - millis())<delay_time)
        {
          sen_st = DELAY;
        }
        else if ((millis()-prev_time)>delay_time)
        {
          sen_st = KTD_6;
        }
        break;
      case KTD_6:
        //Serial.println("KTD_6");
        timerWrite(wdt_timer,0);
        sen_st = SHT31_READ;
        break;
    }  
}

//*******************************************************************************

void setup() {

  Serial.println(F("Starting"));

  //***********************************************************
    // SERIAL COMMS INITIALIZATION
    Serial.begin(9600);
  //***********************************************************
    // ESP32 WATCHDOG TIMER INITIALIZATION
    wdt_timer = timerBegin(1000000);                // 1MHz - Timer frecuency
    timerAttachInterrupt(wdt_timer, &resetModule);  // Attach interrump to timer
    timerAlarm(wdt_timer,wdtTimeout*1000,false,0);  // Alarm value configuration in microseconds (us)
  //***********************************************************
    // SHT SENSOR INITIALIZATION
    Serial.println("SHT30 initializated...");
    SHT31_setup(sht31);
  //***********************************************************
    // I2C COMMS INITIALIZATION
    Wire.begin();
  //***********************************************************
    // MCCI LoRaWAN LMIC INITIALIZATION 
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Disabling not select sub-bands
    for (int b = 0; b < 8; b++)
    {
      LMIC_disableSubBand(b);
    }
    // Then enable the channel(s) you want to use
    LMIC_enableChannel(1); // 903.9 MHz
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  //***********************************************************
   // Start job (sending automatically starts OTAA too)
    do_send(&sendjob); 
}

void loop() {
    FSM_sensors(sht31,wdt_timer);
}
