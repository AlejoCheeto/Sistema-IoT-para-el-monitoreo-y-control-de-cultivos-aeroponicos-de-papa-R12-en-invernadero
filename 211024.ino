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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
// MACROS ----------------------------------------------------------------------------------------------------
  //SHT30
  #define SHT_ADDRESS 0x44 // I2C SHT30 address
  // Lux sensor
  #define address 0x23 // I2C Lux sensor address
  #define lux_reg 0x10 // Lux sensor data register
  // pH sensor
  #define PH_PIN 36    // pH sensor pin definition
  // SSD1306 OLEC SCREEN
  #define SCREEN_WIDTH   128 // OLED display width, in pixels
  #define SCREEN_HEIGHT  64 // OLED display height, in pixels 
  #define SCREEN_ADDRESS 0x3C 
  #define OLED_RESET    -1 // Reset 
  #define LOGO_HEIGHT   64
  #define LOGO_WIDTH    128
//************************************************************************************************************
// SDD1306 BITMAP LOGOS --------------------------------------------------------------------------------------
static const unsigned char PROGMEM logo_bmp[] =
{ 
  0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

static const unsigned char PROGMEM agrosavia_logo []  = 
{
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf0, 0x1f, 0xc0, 0x1f, 0xf8, 0x07, 0xfc, 0x01, 0xff, 0x7c, 0xff, 0x8f, 0x1f, 0xff, 
	0xfe, 0xff, 0xc0, 0x07, 0xc0, 0x07, 0xf0, 0x03, 0xf0, 0x01, 0xfe, 0x7c, 0x7f, 0x9f, 0x3e, 0x7f, 
	0xfe, 0x7f, 0x83, 0xc3, 0xcf, 0xc3, 0xe1, 0xe1, 0xf0, 0xff, 0xfe, 0x3c, 0x7f, 0x1e, 0x3e, 0x7f, 
	0xfc, 0x7f, 0x8f, 0xf1, 0xcf, 0xf3, 0xc7, 0xf8, 0xf3, 0xff, 0xfe, 0x3e, 0x3f, 0x3e, 0x3c, 0x3f, 
	0xfc, 0x3f, 0x1f, 0xff, 0xcf, 0xf3, 0x8f, 0xfc, 0x63, 0xff, 0xfc, 0x1e, 0x3e, 0x3e, 0x7c, 0x3f, 
	0xf8, 0x3f, 0x3f, 0xff, 0xcf, 0xf3, 0x8f, 0xfe, 0x73, 0xff, 0xfc, 0x1f, 0x3e, 0x3c, 0x78, 0x1f, 
	0xf9, 0x1f, 0x3f, 0xff, 0xcf, 0xf3, 0x9f, 0xfe, 0x71, 0xff, 0xf8, 0x9f, 0x1c, 0x7c, 0xf8, 0x1f, 
	0xf1, 0x9e, 0x7f, 0xff, 0xcf, 0xf3, 0x9f, 0xfe, 0x70, 0x03, 0xf8, 0x8f, 0x9c, 0x78, 0xf1, 0x8f, 
	0xf3, 0x9e, 0x7e, 0x00, 0xc7, 0xc7, 0x9f, 0xfe, 0x7c, 0x01, 0xf1, 0xcf, 0x8c, 0xf9, 0xf1, 0x8f, 
	0xf3, 0xcf, 0x3e, 0x00, 0xc0, 0x0f, 0x9f, 0xfe, 0x7f, 0xf0, 0xf1, 0xc7, 0x88, 0xf1, 0xf3, 0xcf, 
	0xe7, 0xcf, 0x3f, 0xf0, 0xc0, 0x0f, 0x8f, 0xfe, 0x7f, 0xf8, 0xf3, 0xe7, 0xc1, 0xf1, 0xe3, 0xc7, 
	0xe7, 0xe7, 0x1f, 0xf9, 0xcf, 0xcf, 0x8f, 0xfc, 0x7f, 0xf8, 0xe3, 0xe3, 0xc1, 0xf3, 0xe7, 0xe7, 
	0xc7, 0xe7, 0x9f, 0xf1, 0xcf, 0xc7, 0xc7, 0xf8, 0xff, 0xf8, 0xe7, 0xf3, 0xe1, 0xe3, 0xc7, 0xe3, 
	0xcf, 0xf3, 0x87, 0xc3, 0xcf, 0xe7, 0xe3, 0xf1, 0xff, 0xf0, 0xc7, 0xf1, 0xe3, 0xe7, 0xcf, 0xf3, 
	0x8f, 0xf3, 0xc0, 0x07, 0xcf, 0xf3, 0xf0, 0x03, 0xf0, 0x01, 0xcf, 0xf1, 0xf3, 0xc7, 0x8f, 0xf1, 
	0x9f, 0xf1, 0xf0, 0x1f, 0xcf, 0xf3, 0xf8, 0x07, 0xf0, 0x03, 0x8f, 0xf8, 0xf7, 0xcf, 0x8f, 0xf1, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static const unsigned char PROGMEM cea_logo [] = 
{
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x30, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xf0, 0x30, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xc0, 0x30, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0x80, 0x30, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfe, 0x00, 0x30, 0x00, 0xff, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xc0, 0x03, 
	0xfc, 0x00, 0x78, 0x00, 0x7f, 0xff, 0x00, 0xf8, 0x00, 0xfe, 0x1f, 0xff, 0xfc, 0xff, 0xc0, 0x03, 
	0xf8, 0x00, 0x7f, 0xff, 0xff, 0xfe, 0x18, 0x78, 0x00, 0xfe, 0x0f, 0xff, 0xfc, 0xff, 0xfe, 0x7f, 
	0xf0, 0x00, 0xff, 0xff, 0xff, 0xfc, 0x7e, 0x38, 0xff, 0xfe, 0x0f, 0xff, 0xfc, 0xff, 0xfe, 0x7f, 
	0xf0, 0x00, 0x78, 0x00, 0x1f, 0xfc, 0x7f, 0x38, 0xff, 0xfc, 0x4f, 0xff, 0xfc, 0xf8, 0x3e, 0x7f, 
	0xe0, 0x00, 0x30, 0x00, 0x0f, 0xfc, 0xff, 0xf8, 0xff, 0xfc, 0xc7, 0xff, 0xfc, 0xf0, 0x1e, 0x7f, 
	0xc0, 0x00, 0x30, 0x00, 0x07, 0xf8, 0xff, 0xf8, 0x00, 0xfc, 0xe7, 0xff, 0xfc, 0xe7, 0x8e, 0x7f, 
	0xc0, 0x00, 0x30, 0x00, 0x07, 0xf8, 0xff, 0xf8, 0x00, 0xf8, 0xe3, 0xff, 0xfc, 0xe7, 0xce, 0x7f, 
	0x80, 0x00, 0x30, 0x00, 0x03, 0xf8, 0xff, 0xf8, 0xff, 0xf8, 0xe3, 0xe0, 0x7c, 0xcf, 0xce, 0x7f, 
	0x80, 0x00, 0x30, 0x00, 0x03, 0xf8, 0xff, 0xf8, 0xff, 0xf9, 0xe3, 0xe0, 0x7c, 0xcf, 0xc6, 0x7f, 
	0x80, 0x00, 0x30, 0x00, 0x03, 0xfc, 0x7f, 0x38, 0xff, 0xf0, 0x01, 0xff, 0xfc, 0xcf, 0xce, 0x7f, 
	0x80, 0x00, 0x30, 0x00, 0x03, 0xfc, 0x7e, 0x38, 0xff, 0xf0, 0x01, 0xff, 0xfc, 0xcf, 0xce, 0x7f, 
	0x00, 0x00, 0x30, 0x00, 0x01, 0xfc, 0x3c, 0x78, 0xff, 0xe3, 0xf9, 0xff, 0xfc, 0xe7, 0xce, 0x7f, 
	0x00, 0x00, 0x30, 0x00, 0x81, 0xfe, 0x00, 0x78, 0x00, 0xe3, 0xf8, 0xff, 0xfc, 0xe3, 0x8e, 0x7f, 
	0x00, 0x00, 0x78, 0x03, 0xe1, 0xff, 0x00, 0xf8, 0x00, 0xe3, 0xf8, 0xff, 0xfc, 0xf0, 0x1e, 0x7f, 
	0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 
	0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x00, 0x00, 0x7c, 0x01, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x80, 0x00, 0x0e, 0x01, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x80, 0x00, 0x07, 0x01, 0x83, 0xff, 0x5a, 0xff, 0xdf, 0xf7, 0xff, 0xf7, 0xff, 0xf7, 0xed, 0xff, 
	0xbf, 0xc0, 0x03, 0x81, 0x83, 0xfb, 0xa6, 0x7d, 0x2c, 0x89, 0xdf, 0xdb, 0xb2, 0xe9, 0x33, 0xff, 
	0xff, 0xe0, 0x01, 0xc1, 0x83, 0xff, 0x36, 0x7d, 0x2f, 0x89, 0xdf, 0xcb, 0x7e, 0xe9, 0x13, 0xff, 
	0xff, 0xf0, 0x00, 0xe1, 0x87, 0xfe, 0xdf, 0xee, 0xdf, 0xf6, 0xef, 0x73, 0x7e, 0x74, 0xe5, 0xff, 
	0xff, 0xf8, 0x00, 0x71, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xfc, 0x00, 0x39, 0x87, 0xff, 0xfb, 0xbf, 0xff, 0xdf, 0x7f, 0x9f, 0xff, 0xff, 0xff, 0xff, 
	0xf0, 0xfc, 0x00, 0x1f, 0x8f, 0xfc, 0x18, 0xa5, 0x21, 0x93, 0x13, 0x72, 0x43, 0xff, 0xff, 0xff, 
	0xe0, 0x7e, 0x00, 0x0f, 0x9f, 0xf8, 0x59, 0x27, 0x87, 0x43, 0x13, 0x7a, 0x43, 0xff, 0xff, 0xff, 
	0xc0, 0x7e, 0x00, 0x07, 0x9f, 0xfc, 0x59, 0x27, 0xa5, 0x93, 0x13, 0x92, 0x43, 0xff, 0xff, 0xff, 
	0xc0, 0x7e, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xc0, 0x7e, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xc0, 0x7e, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xe0, 0xfc, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xfc, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const unsigned char PROGMEM unconnected_icon []  = 
{
	0x00, 0x1f, 0xe0, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x01, 0xfb, 0x7e, 0x00, 0x03, 0xb3, 0x37, 0x18, 
	0x0e, 0x63, 0x19, 0xfc, 0x0e, 0x63, 0x19, 0xf8, 0x1f, 0xe3, 0x1f, 0xf0, 0x31, 0xff, 0xff, 0xf0, 
	0x30, 0xff, 0xff, 0xb0, 0x60, 0x83, 0x0f, 0x18, 0x61, 0x83, 0x1e, 0x18, 0xc1, 0x83, 0x7e, 0x0c, 
	0xc1, 0x83, 0xf6, 0x0c, 0xc1, 0x83, 0xe6, 0x0c, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 
	0xc1, 0x9f, 0x06, 0x0c, 0xc1, 0xbf, 0x06, 0x0c, 0xc1, 0xfb, 0x06, 0x0c, 0x61, 0xf3, 0x06, 0x18, 
	0x63, 0xe3, 0x06, 0x18, 0x37, 0xff, 0xfc, 0x10, 0x3f, 0xff, 0xfe, 0x30, 0x3f, 0xe3, 0x1f, 0xe0, 
	0x7e, 0x63, 0x19, 0xe0, 0x7e, 0x63, 0x19, 0xc0, 0x63, 0xb3, 0x33, 0x80, 0x01, 0xfb, 0x7e, 0x00, 
	0x00, 0x7f, 0xf8, 0x00, 0x00, 0x1f, 0xe0, 0x00};

const unsigned char PROGMEM connected_icon [] = 
{ 
  0x00, 0x1f, 0xe0, 0x00, 0x00, 0xff, 0xfc, 0x00, 0x01, 0xfb, 0x7e, 0x00, 0x07, 0x33, 0x33, 0x80, 
	0x0e, 0x63, 0x19, 0xc0, 0x1e, 0x63, 0x19, 0xe0, 0x1f, 0xc3, 0x0f, 0xe0, 0x31, 0xff, 0xfe, 0x30, 
	0x60, 0xff, 0xfc, 0x18, 0x61, 0x83, 0x06, 0x18, 0x61, 0x83, 0x06, 0x18, 0xc1, 0x83, 0x06, 0x0c, 
	0xc1, 0x83, 0x06, 0x0c, 0xc1, 0x83, 0x06, 0x0c, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 
	0xc1, 0x83, 0x06, 0x0c, 0xc1, 0x83, 0x06, 0x0c, 0xc1, 0x83, 0x06, 0x0c, 0x61, 0x83, 0x06, 0x18, 
	0x61, 0x83, 0x06, 0x18, 0x60, 0xff, 0xfc, 0x18, 0x31, 0xff, 0xfe, 0x30, 0x1f, 0xc3, 0x0f, 0xe0, 
	0x1e, 0x63, 0x19, 0xe0, 0x0e, 0x63, 0x19, 0xc0, 0x07, 0x33, 0x33, 0x80, 0x01, 0xfb, 0x7e, 0x00, 
	0x00, 0xff, 0xfc, 0x00, 0x00, 0x1f, 0xe0, 0x00};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
static const u1_t PROGMEM APPEUI[8] = { 0x08, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 };
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little-endian format, see above.
//static const u1_t PROGMEM DEVEUI[8] = { 0xD3, 0x92, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
//static const u1_t PROGMEM DEVEUI[8] = { 0x6A, 0x95, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8] = { 0x6A, 0x92, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x44, 0x44, 0x55, 0x55, 0x66, 0x66, 0x77, 0x77, 0x88, 0x88 };
//static const u1_t PROGMEM APPKEY[16] = { 0x88, 0x88, 0x77, 0x77, 0x66, 0x66, 0x55, 0x55, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11 };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

//static uint8_t mydata[] = "\"deviceID\":\"Aeronodo\",\"temperatura\":\"88888\",\"humedad\":\"88888\",\"lux\":\"88888\",\"pH\":\"88888\"";
static uint8_t mydata[] = "\"device\":\"Aeronodo\",\"temperatura\":\"888888\",\"humedad\":\"888888\",\"lux\":\"88888888\",\"pH\":\"88888\"";
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
// SENSORS FSM VARIABLES -------------------------------------------------------------------------------------
//*****************************************************
// ONLY MODIFY THE VARIABLES INSIDE THIS BOX
// NUMBER OF READING PER SAMPLE
const int c = 10; // Number or readings after compute an averaged sample

// DELAY TIME
const int delay_time = 2000;    // Delay time between each data measurment (ms)

// SHT30 timeouts variables
const int heater_wait_timeout_min = 5;//min 
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
  //5V
//float acidVoltage = 2870;
//float neutralVoltage = 2130;
  //3V3
float acid_voltage = 1980;
float neutral_voltage = 1560;
float ph_acid = 4.0;
float ph_neutral = 6.8;  
float ph_val;                   // pH reading varaible (pH)

// VARIABLES ------------------------------------------------------------
float t_vec[c],h_vec[c],lux_vec[c],ph_vec[c]; //Array for keep the readings before average them
float t_mean = 0,h_mean = 0, lux_mean = 0, ph_mean = 0; //Resulting average for each measured variable

//************************************************************************************************************

/*FN****************************************************************************
*
*   void drawbitmap(int flag)
*
*   Purpose: 
*       Draws a bitmap (logo) on the display, centered on the screen, 
*       based on the value of the 'flag' parameter.
*
*   Parameters:
*       flag: 
*           An integer that determines which logo to draw:
*           - 1: Draws the Agrosavia logo.
*           - 2: Draws the CEA logo.
*
*   Plan: 
*       - Clears the display screen.
*       - Depending on the value of 'flag', draws the corresponding 
*         bitmap at centered coordinates.
*       - Displays the bitmap on the screen and waits 1 second before 
*         allowing another action.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE     COMMENT
*   -----------------------------------------------------------------------
*   Sep 20/2024  J.Rodriguez    Initial implementation of the function.
*   Sep 20/2024  J.Rodríguez    Documentation.
*
*******************************************************************************/
void drawbitmap(int flag) {
  display.clearDisplay();

  if (flag == 1){
    display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    agrosavia_logo, LOGO_WIDTH, LOGO_HEIGHT, 1);
  } else if (flag == 2){
    display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    cea_logo, LOGO_WIDTH, LOGO_HEIGHT, 1);
  }
  display.display();
  delay(2000);
  
}

/*FN****************************************************************************
*
*   void drawlorawan(bool state)
*
*   Purpose: 
*       Displays the current LoRaWAN connection status on the screen, showing 
*       either a "Connected" or "Unconnected" message along with an icon, based 
*       on the value of the 'state' parameter.
*
*   Parameters:
*       state: 
*           A boolean value that indicates the connection status:
*           - true: Shows "Connected" message and draws the connected icon.
*           - false: Shows "Unconnected" message and draws the unconnected icon.
*
*   Plan: 
*       - Clear the display screen to remove any previous content.
*       - Set up text size, color, and cursor position.
*       - Print the LoRaWAN status message based on the value of 'state'.
*       - Draw the appropriate icon (connected or unconnected) centered horizontally.
*       - Display the updated screen content.
*
*   Register of Revisions:
*
*   DATE         RESPONSIBLE     COMMENT
*   -----------------------------------------------------------------------
*   Oct 02/2024  J.Rodriguez    Initial implementation of the function.
*   Oct 03/2024  J.Rodríguez    Documentation.
*
*******************************************************************************/

void drawlorawan(bool state) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("");
  if (state == 1){
    display.clearDisplay();
    display.print(F("LoRaWAN Status : \n"));
    display.println(F("Connected"));
    display.drawBitmap(
    (display.width()  - 30 ) / 2,
    30,
    connected_icon, 30, 30, 1);
  }
  else{
    display.clearDisplay();
    display.print(F("LoRaWAN Status : \n"));
    display.println(F("Unconnected"));
    display.drawBitmap(
    (display.width()  - 30 ) / 2,
    30,
    unconnected_icon, 30, 30, 1);
  }
  display.display();
}
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
            drawlorawan(0);
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            drawlorawan(0);
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
            drawlorawan(1);
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
            drawlorawan(0);
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
  if(!sht_obj.begin(SHT_ADDRESS)){
    Serial.println("SHT30: Impossible to reach SHT30...");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,32);
    display.println(F("Impossible to reach"));
    display.println(F("SHT31..."));
    display.display();
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
        if(!sht31.begin(SHT_ADDRESS)){
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0,32);
          //display.println(F("Impossible to reach"));
          display.println(F("Error de conexiOn"));
          display.println(F("SHT30..."));
          display.display();
          delay(1000);
          sht_vec[0] = -1;
          sht_vec[1] = -1;
        } else{
          SHT31_read(sht_vec,sht_obj);
        }
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
          Serial.println("LUX SENSOR: Impossible to reach lux sensor...");
          Serial.println("LUX SENSOR: Check the illuminance sensor connection");
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0,32);
          //display.println(F("Impossible to reach"));
          //display.println(F("lux sensor..."));
          display.println(F("Error de conexiOn"));
          display.println(F("sensor de luz ..."));
          display.display();
          delay(1000);
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
        /* ph_val = PH_read(ph_acid,acid_voltage,ph_neutral,neutral_voltage); */
        ph_val = 0.0;
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
          sprintf((char*)mydata,"\"device\":\"Aeronodo\",\"temperatura\":\"%06.2f\",\"humedad\":\"%06.2f\",\"lux\":\"%08.2f\",\"pH\":\"%05.2f\"", t_mean, h_mean, lux_mean, ph_mean);
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
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0,32);
          display.println(F("Heater status:"));
          display.println(F("ON"));
          display.display();
          delay(1000);
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
    // SDD1306 OLED SCREEN
   if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    drawbitmap(1);
    drawbitmap(2);
    display.clearDisplay();
  //***********************************************************
    // SHT SENSOR INITIALIZATION
    //Serial.println("SHT30 initializated...");
    //SHT31_setup(sht31);
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
    for (int b = 0; b < 8; b++){
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
