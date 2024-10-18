/* Smart_Config.h */
#ifndef SMART_CONFIG
#define SMART_CONFIG

#include "DeviceType.h"

#define SERIAL_DEBUG 1
#define OT_DEBUG 0
#define OT_DEBUGLOG 0
#define SERVER_DEBUG 1
#define T_DEBUG 0
#define MQTT_USE 1

#if MQTT_USE
  #define PID_USE 1 
#else
  #define PID_USE 0
#endif

/* Min & max CH temp */
#define MIN_CH_TEMP  25
#define MAX_CH_TEMP  80

/* Room setpoint Min & max */
#define MIN_ROOM_TEMP  5
#define MAX_ROOM_TEMP  35

#define MAX_PID_SRC 4

#define IDENTIFY_TYPE DS_OPENTHERM
/* TCP/UDP buffer size in bytes */
#define UDP_TSP_BUFSIZE 128

//есть датчик температуры
#define USE_SENSOR_T 1

#ifndef PROSESSOR_CODE
#if defined(ARDUINO_ARCH_ESP8266)
 #define PROSESSOR_CODE  1
 #define IDENTIFY_TEXT        		F("Умный контроллер SmartTherm ESP8266")
#elif defined(ARDUINO_ARCH_ESP32)
 #define PROSESSOR_CODE  2
 #define IDENTIFY_TEXT        		F("Умный контроллер SmartTherm ESP32")
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
  #define AUTOCONNECT_MENU_TITLE  "SmartTherm ESP8266"
  #define AUTOCONNECT_APID  "ST_ESP8266"
#elif defined(ARDUINO_ARCH_ESP32)
  #define AUTOCONNECT_MENU_TITLE  "SmartTherm ESP32"
  #define AUTOCONNECT_APID  "ST_ESP32"
 #else
error not used in this config
 #endif // !ARDUINO_ARCH_ESP8266
#endif // PROSESSOR_CODE

#if defined(ARDUINO_ARCH_ESP8266)
//  #define LED_BUILTIN 2
#elif defined(ARDUINO_ARCH_ESP32)
  #define LED_BUILTIN 2
#endif

#endif //SMART_CONFIG

