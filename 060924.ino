/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification, and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where a DevEUI and
 * application key are configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI, and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

#include "esp_system.h"
#include "rom/ets_sys.h"

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

#define address 0x23
#define lux_reg 0x10
#define PH_PIN 36

unsigned long int heater_time;
unsigned long int prev_time;

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
static const u1_t PROGMEM DEVEUI[8] = { 0x6A, 0x93, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x77, 0x77, 0x11, 0x11 };
//static const u1_t PROGMEM APPKEY[16] = { 0x88, 0x88, 0x77, 0x77, 0x66, 0x66, 0x55, 0x55, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11 };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

//static uint8_t mydata[] = "\"AeroNodo\":\"nodo1\"";
static uint8_t mydata[] = "\"deviceID\":\"Aeronodo\",\"temperatura\":\"888\",\"humedad\":\"888\",\"lux\":\"888\",\"pH\":\"888\"";
static osjob_t sendjob;
const int maxSize = 100;
uint8_t msg_cast[maxSize];

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 2000;

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = { /*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32 }
};
//************************************************************************************************************

Adafruit_SHT31 sht31 = Adafruit_SHT31();
float sht_data [2];

//************************************************************************************************************

uint8_t buf[4] = {0};
float lux_data;

//************************************************************************************************************

float ph_acid = 4.0;
float acid_voltage = 2870;
float ph_neutral = 6.8;
float neutral_voltage = 2130;


//************************************************************************************************************

const int wdtTimeout = 10000; //ms
hw_timer_t *wdt_timer = NULL;


int count = 0;
const int delay_time = 2000;//ms

const int heater_wait_timeout_min = 2;//min
const int heater_active_timeout_min = 1;//min

const int heater_wait_timeout_ms = heater_wait_timeout_min*60000;//ms
const int heater_active_timeout_ms =  heater_active_timeout_min*60000;//ms

//************************************************************************************************************
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
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

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//************************************************************************************************************

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

float mean(float* vec, int size){
  float sum =0 ;
  float res;
  for(int i=0;i<size;i++){
    sum = sum + vec[i];
  }
  res = sum/size;

  return res;
}

float median(float vec[], int size){
  float med;
  int mid;
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

void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

//************************************************************************************************************

void FSM_sensors_prueba(Adafruit_SHT31& sht_obj,hw_timer_t* wdt_timer){
  static sensors_states sen_st = INI_WDT;

  bool heater_st=0;
  uint8_t new_msg[]= "aeronodo";
  //int count=0;

  float sht_vec[2] ={-1};
  float lux_val = -1;
  uint8_t lux_buf[4] = {0};
  float ph_val;
  int c = 30;
  float t_vec[c],h_vec[c],lux_vec[c],ph_vec[c];
  float t_mean,h_mean,lux_mean,ph_mean;
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
        if ( Wire.requestFrom(address,2,0) != 0) {
          lux_val = LUX_read(buf,2);
        }
        else{
          lux_val = -1;
          Serial.println("I2C device is not found...");
          Serial.println("Check the illuminance sensor connection");
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
        //delay(1000);
        sen_st = KTD_3;
        break;
      case KTD_3:
        Serial.println("KTD_3");
        timerWrite(wdt_timer,0);
        count++;
        sen_st = MEAN;
        break;
      case MEAN:
        Serial.print("N° de ciclo : ");
        Serial.println(count);
        if(count < 30){
          Serial.println("A");
          t_vec[count] = sht_vec[0];
          h_vec[count] = sht_vec[1];
          lux_vec[count] = lux_val;
          ph_vec[count] = ph_val;
          sen_st = KTD_4;
        }
        else if (count >= 30){
          /*t_mean = mean(t_vec,c);
          h_mean = mean(h_vec,c);
          lux_mean = mean(lux_vec,c);
          ph_mean = mean(ph_vec,c);*/
          Serial.println("B");
          t_mean = 23;
          h_mean = 45;
          lux_mean = 412;
          ph_mean = 6.78;
          count = 0;
          /*sprintf((char*)mydata,"\"deviceID\":\"Aeronodo\",\"temperatura\":\"%.2f\",\"humedad\":\"%.2f\",\"lux\":\"%.2f\",\"pH\":\"%.2f\"", t_mean, h_mean, lux_mean, ph_mean);
          Serial.print((char*)mydata);
          do_send(&sendjob);*/
          sen_st = SEND_DATA;
        }
        break;
      case SEND_DATA:
        Serial.println("SEND_DATA");
        sprintf((char*)mydata,"\"deviceID\":\"Aeronodo\",\"temperatura\":\"%.2f\",\"humedad\":\"%.2f\",\"lux\":\"%.2f\",\"pH\":\"%.2f\"", t_mean, h_mean, lux_mean, ph_mean);
        Serial.print((char*)mydata);

        //Serial.println(sizeof(mydata));
        //mydata = (uint8_t*) mydata;
        //count = 0;
        do_send(&sendjob);
        sen_st = KTD_4;
        break;
      case KTD_4:
        Serial.println("KTD_4");
        timerWrite(wdt_timer,0);
        sen_st = HEATER;
        break;
      case HEATER:
        Serial.println("HEATER");
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
          sht_obj.heater(0);
          heater_time = millis();
        }
        sen_st = KTD_5;
        break;
      case H_OFF:
        //Serial.println("H_OFF");
        if((millis()-heater_time>heater_wait_timeout_ms))
        {
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
        Serial.println("DELAY");
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

void setup() {

  //***********************************************************

  Serial.begin(9600);

  //***********************************************************

    wdt_timer = timerBegin(1000000);
    timerAttachInterrupt(wdt_timer, &resetModule);
    timerAlarm(wdt_timer,wdtTimeout*1000,false,0);

  //***********************************************************
    
    Serial.println(F("Starting"));

    Serial.println("SHT initializated...");
    SHT31_setup(sht31);

  //***********************************************************

    Wire.begin();

  //***********************************************************
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
    for (int b = 0; b < 8; b++)
    {
      LMIC_disableSubBand(b);
    }
    // Then enable the channel(s) you want to use
    LMIC_enableChannel(1); // 903.9 MHz
    //LMIC_selectSubBand(1);
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  //***********************************************************

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
    
}

void loop() {
    os_runloop_once();
    //FSM_sensors(sht31,sht_data,lux_data,buf,mydata,&sendjob,wdt_timer);
/*     Serial.println();
    Serial.println("LOOP ---------------------------------------------------"); */
    //FSM_sensors_prueba(sht31,wdt_timer);
}
