/* Smart_Config.h */
#ifndef SMART_CONFIG
#define SMART_CONFIG

#include "DeviceType.h"

#define IDENTIFY_TYPE DS_OPENTHERM

//есть LPC804
#define USE_LPC804  0

//есть датчик температуры
#define USE_SENSOR_T 1

//есть управление розетками 220V
#define USE_SMART_SOCKET  0

//есть счетчик импульсов
#define USE_COUNTER 0


#if defined(ARDUINO_ARCH_ESP8266)
 #define PROSESSOR_CODE  1
 #define IDENTIFY_TEXT        		"Умный контроллер Open Therm ESP8266"
#elif defined(ARDUINO_ARCH_ESP32)
 #define PROSESSOR_CODE  2
 #define IDENTIFY_TEXT        		"Умный контроллер Open Therm ESP32"
#endif

#define IDENTIFY_CODE   (PROSESSOR_CODE<<24)|(USE_LPC804<<20)|(USE_SMART_SOCKET<<16)|(USE_SENSOR_T<<8)|(USE_COUNTER)


// AutoConnect menu title
// Predefined parameters
// SSID that Captive portal started.
#if defined(ARDUINO_ARCH_ESP8266)
  #define AUTOCONNECT_MENU_TITLE  "Smart Open Therm controller ESP8266"
  #define AUTOCONNECT_APID  "OT_ESP8266"
#elif defined(ARDUINO_ARCH_ESP32)
  #define AUTOCONNECT_MENU_TITLE  "Smart Open Therm controller ESP32"
  #define AUTOCONNECT_APID  "OT_ESP32"
 #else
error not used in this config
 #endif // !ARDUINO_ARCH_ESP8266

#endif //SMART_CONFIG

