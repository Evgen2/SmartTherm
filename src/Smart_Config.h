/* Smart_Config.h */
#ifndef SMART_CONFIG
#define SMART_CONFIG

#include "DeviceType.h"

#define SERIAL_DEBUG 1
#define OT_DEBUG 0
#define SERVER_DEBUG 0
#define T_DEBUG 1

#define IDENTIFY_TYPE DS_OPENTHERM


//есть датчик температуры
#define USE_SENSOR_T 1


#ifndef PROSESSOR_CODE
#if defined(ARDUINO_ARCH_ESP8266)
 #define PROSESSOR_CODE  1
 #define IDENTIFY_TEXT        		"Умный контроллер SmatrTherm ESP8266"
#elif defined(ARDUINO_ARCH_ESP32)
 #define PROSESSOR_CODE  2
 #define IDENTIFY_TEXT        		"Умный контроллер SmatrTherm ESP32"
#endif

#define IDENTIFY_CODE   (PROSESSOR_CODE<<24)|(USE_SENSOR_T<<8)


// AutoConnect menu title
// Predefined parameters
// SSID that Captive portal started.
//remove warning on redefined 
#if defined(AUTOCONNECT_MENU_TITLE)
#undef AUTOCONNECT_MENU_TITLE
#endif
#if defined(AUTOCONNECT_APID)
#undef AUTOCONNECT_APID
#endif

#if defined(ARDUINO_ARCH_ESP8266)
  #define AUTOCONNECT_MENU_TITLE  "SmartTherm controller ESP8266"
  #define AUTOCONNECT_APID  "ST_ESP8266"
#elif defined(ARDUINO_ARCH_ESP32)
  #define AUTOCONNECT_MENU_TITLE  "SmartTherm controller ESP32"
  #define AUTOCONNECT_APID  "ST_ESP32"
 #else
error not used in this config
 #endif // !ARDUINO_ARCH_ESP8266
#endif // PROSESSOR_CODE
#endif //SMART_CONFIG

