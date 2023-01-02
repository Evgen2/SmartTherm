/* main.cpp */
// контроллер OpenTherm на ESP32/ESP8266 с измерением температуры
// WiFi, Captive Portal, Web доступ
// клиент-сервер (UDP или TCP)
// based on OpenTherm Master Communication Example By: Ihor Melnyk
//


#include <time.h>
#include <Arduino.h>
#include <OpenTherm.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
typedef ESP8266WebServer WEBServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
typedef WebServer WEBServer;
#endif

#include "SmartDevice.hpp"
#include "Smart_Config.h"
#include "SD_OpenTherm.hpp"

/************************************/
extern void setup_web_common(void);
extern void loop_web(void);
extern void setup_udp(SmartDevice *psd);

extern void loop_udp(void);
extern void test(void);
/************************************/
class SD_Termo SmOT;

//Задаем пины
#if defined(ARDUINO_ARCH_ESP8266)
//  #define LED_BUILTIN 2

  const int inPin = D7;  // OpenTherm  in   D7 GIP013 Nodemsu
  const int outPin = D8;  // OpenTherm out  D8 GPIO15 Nodemsu

  const int DS1820_1 = D2; //
  const int DS1820_2 = D6; //

#elif defined(ARDUINO_ARCH_ESP32)
  #define LED_BUILTIN 2
  const int inPin = 16;  // OpenTherm  in RX2 esp32
  const int outPin = 4;  // OpenTherm out D4 esp32

  const int DS1820_1 = 15; // D15 esp32  3 снизу
  const int DS1820_2 = 26; // D26 esp32  7 снизу

#endif


OpenTherm ot(inPin, outPin);
void OTprocessResponse(unsigned long response, OpenThermResponseStatus status);
int OTloop(void);
void loop2(void);
/* DS18b20 */
#include <OneWire.h>
#include <DS18B20.h>

void loopDS1820(void);
void setupDS1820(void);

OneWire oneWire1(DS1820_1);
OneWire oneWire2(DS1820_2);
DS18B20 Tsensor1(&oneWire1);
DS18B20 Tsensor2(&oneWire2);
extern int OTDebugInfo[10];

void IRAM_ATTR handleInterrupt() {
    ot.handleInterrupt();
}

int LedSts = 0; //LOW
void setup() {
  time_t now;
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, LedSts);   // Turn the LED on (Note that LOW is the voltage level
  delay(1000);
  Serial.begin(115200);
  Serial.println();

  Serial.println("Start");
  Serial.println(IDENTIFY_TEXT);
  Serial.printf("Vers %d.%d build %s\n",SmOT.Vers, SmOT.SubVers, SmOT.BiosDate);

  now = time(nullptr);
  Serial.printf("Sizeof time_t= %d\n",sizeof( time_t) );
  Serial.printf("time= %s millis()=%li\n", ctime(&now), millis());

    LedSts=1;
     digitalWrite(LED_BUILTIN, LedSts);   

    setupDS1820();

    ot.begin(handleInterrupt, OTprocessResponse);
  setup_web_common();
  setup_udp( &SmOT );
}

/* based on

OpenTherm Master Communication Example
By: Ihor Melnyk
Date: January 19th, 2018

Uses the OpenTherm library to get/set boiler status and water temperature
Open serial monitor at 115200 baud to see output.

Hardware Connections (OpenTherm Adapter (http://ihormelnyk.com/pages/OpenTherm) to Arduino/ESP8266):
-OT1/OT2 = Boiler X1/X2
-VCC = 5V or 3.3V
-GND = GND
-IN  = Arduino (3) / ESP8266 (5) Output Pin
-OUT = Arduino (2) / ESP8266 (4) Input Pin

Controller(Arduino/ESP8266) input pin should support interrupts.
Arduino digital pins usable for interrupts: Uno, Nano, Mini: 2,3; Mega: 2, 3, 18, 19, 20, 21
ESP8266: Interrupts may be attached to any GPIO pin except GPIO16,
but since GPIO6-GPIO11 are typically used to interface with the 
flash memory ICs on most esp8266 modules, applying interrupts to these pins are likely to cause problems
*/


int status_OT = -1;


void setupDS1820(void)
{//  Serial.print("DS18B20 Library version: ");
 //  Serial.println(DS18B20_LIB_VERSION);

  SmOT.status = 0x0;

  if(Tsensor1.begin() == false)
  {   SmOT.stsT1 = -1;
      SmOT.status |= 0x02;
      Serial.printf("ERROR: No DS18b20(1) found on pin %i\n", DS1820_1);
      delay(100);
      if(Tsensor1.begin() )
      {   Serial.printf("2nd attempt(1) Ok\n");
          goto M1;
      }

  }  else {
M1:      SmOT.stsT1 = 0;
      SmOT.status |= 0x01;

      Tsensor1.setResolution(12);
      Tsensor1.setConfig(DS18B20_CRC);  // or 1
      Serial.printf("DS18b20 (1) found on pin %i\n", DS1820_1);
  }

  if(Tsensor2.begin() == false)
  {   SmOT.stsT2 = -1;
      SmOT.status |= 0x0200;
      Serial.printf("ERROR: No DS18b20(2) found on pin %i\n", DS1820_2);
      delay(100);
      if(Tsensor2.begin() )
      {   Serial.printf("2nd attempt(2) Ok\n");
          goto M2;
      }

  }  else {
M2:   SmOT.stsT2 = 0;
      SmOT.status |= 0x0100;
      Tsensor2.setResolution(12);
      Tsensor2.setConfig(DS18B20_CRC);  // or 1
      Serial.printf("DS18b20(2) found on pin %i\n", DS1820_2);
  }
 } 

void loopDS1820(void)
{ static unsigned long int start=0;
  int rc;
  static int nd=0;
  float t;

//  Serial.printf("loopDS1820 nd %i %li\n", nd, millis());
  switch(nd)
  {   case 0:
        if(SmOT.status&0x01)
        { Tsensor1.requestTemperatures();
          nd = 1;
          start = millis();
        }  else nd = 2;
      break;
      case 1:
        if(millis()-start < 700)
              break;
        if(millis()-start > 900)
        {   rc = Tsensor1.isConversionComplete();
            if(!rc)
            { SmOT.status |= 0x04;
              nd = 2;
#if SERIAL_DEBUG 
              Serial.printf("ERROR: DS1 timeout or disconnect %li\n", millis());
#endif                
              break;
            }
        } else {
          rc = Tsensor1.isConversionComplete();
        }
        if(rc)
        { t = Tsensor1.getTempC();
          SmOT.status &= ~0x04; // сброс бита тайаута
          if (t == DEVICE_CRC_ERROR)
          { SmOT.stsT1 = 1;
            SmOT.status |= 0x10;
            Serial.println("ERROR: DS1 CRC error");
          } else {
            SmOT.status &= ~0x10; // сброс бита CRC error
            SmOT.t1 = t;
//            Serial.printf("SmOT T1= %f\n",   SmOT.t1);
          }
          SmOT.status &= ~0x04;
          nd = 2;
        }
        break;

      case 2:
        if(SmOT.status&0x0100)
        { Tsensor2.requestTemperatures();
          nd = 3;
          start = millis();
        }  else nd = 0;
      break;

      case 3:
        if(millis()-start < 700)
              break;
        if(millis()-start > 900) //900
        { rc = Tsensor2.isConversionComplete();
          if(!rc)
          { SmOT.status |= 0x0400;
            nd = 0;
#if SERIAL_DEBUG 
            Serial.println("ERROR: DS2 timeout or disconnect");
#endif                
              break;
          }
        } else {
          rc = Tsensor2.isConversionComplete();
        }
        if(rc)
        {  SmOT.status &= ~0x0400; // сброс бита таймаута

          t = Tsensor2.getTempC();
          if (t == DEVICE_CRC_ERROR)
          { SmOT.stsT2 = 1;
            SmOT.status |= 0x1000;
            Serial.println("ERROR: DS2 CRC error");
          } else {
            SmOT.status &= ~0x1000; // сброс бита CRC error
            SmOT.t2 = t;
//            Serial.printf("SmOT T2= %f\n",   SmOT.t2);
          }
          nd = 0;
        }

      break;
  }
}

////////////////////////////////////////////////////////
    //Set/Get Boiler Status
    bool enableCentralHeating = true;
    bool enableHotWater = true;
    bool enableCooling = false;

//
void OTprocessResponse(unsigned long response, OpenThermResponseStatus status)
{   float t;
    uint16_t u88;
    byte id;

    if (!ot.isValidResponse(response)) {
//        Serial.println("Invalid response: " + String(response, HEX) + ", status=" + String(ot.getLastResponseStatus()));
        OTDebugInfo[1]++;
        return;
    }

    if (status == OpenThermResponseStatus::SUCCESS) {
        SmOT.stsOT = 0;
        SmOT.response = response; 
        OTDebugInfo[0]++;
    } else if (status == OpenThermResponseStatus::NONE) {
       SmOT.stsOT = -1;
        OTDebugInfo[2]++;
    } else if (status == OpenThermResponseStatus::INVALID) {
       SmOT.stsOT = 1;
        OTDebugInfo[3]++;
    } else if (status == OpenThermResponseStatus::TIMEOUT) {
       SmOT.stsOT = 2;
        OTDebugInfo[4]++;
    }
    if(SmOT.stsOT != 0)
        return;

    id = (response >> 16 & 0xFF);
    u88 = (response & 0xffff);
    t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;

    switch (id)
    {
    case OpenThermMessageID::Status:  //0
/*  HB: Master status 
    bit: description [ clear/0, set/1]
0: CH enable [ CH is disabled, CH is enabled]
1: DHW enable [ DHW is disabled, DHW is enabled]
2: Cooling enable [ Cooling is disabled, Cooling is
enabled]
3: OTC active [OTC not active, OTC is active]
4: CH2 enable [CH2 is disabled, CH2 is enabled]
5: reserved
6: reserved
7: reserved
*/
/*  LB: Slave status   
bit: description [ clear/0, set/1]
0: fault indication [ no fault, fault ]
1: CH mode [CH not active, CH active]
2: DHW mode [ DHW not active, DHW active]
3: Flame status [ flame off, flame on ]
4: Cooling status [ cooling mode not active, cooling
mode active ]
5: CH2 mode [CH2 not active, CH2 active]
6: diagnostic indication [no diagnostics, diagnostic event]
7: reserved 
*/   
//        boiler_status = response & 0xFF;

        SmOT.BoilerStatus = u88;
//        Serial.println("Central Heating: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
//        Serial.println("Hot Water: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
//        Serial.println("Flame: " + String(ot.isFlameOn(response) ? "on" : "off"));

//        curr_item->status = boiler_status;
//        Serial.println("Boiler status: " + String(boiler_status, BIN));
        break;

    case OpenThermMessageID::TSet:  // 1
        SmOT.Tset_r = t;
//        Serial.println("Set CH temp: " + String(t));
        break;
    case OpenThermMessageID::Tboiler:  //25
        SmOT.BoilerT = t;
        break;
    case OpenThermMessageID::Tret: //28
        SmOT.RetT = t;
        break;

    case OpenThermMessageID::Tdhw: //26
        SmOT.dhw_t = t;
        break;
    case OpenThermMessageID::MaxRelModLevelSetting: //14 Maximum relative modulation level setting (%) W
        SmOT.MaxRelModLevelSetting = t;
        break;
    case OpenThermMessageID::MaxCapacityMinModLevel:	//15 MaxCapacityMinModLevel, // u8 / u8  Maximum boiler capacity (kW) / Minimum boiler modulation level(%) R
        SmOT.MinModLevel =  (u88 & 0xff);
        SmOT.MaxCapacity =  ((u88>>8) & 0xff);
        break;

    case OpenThermMessageID::RelModLevel: //17 Relative Modulation Level 
        SmOT.FlameModulation = t;
        break;
    case OpenThermMessageID::CHPressure: //18 Water pressure in CH circuit
        SmOT.Pressure = t;
        break;

    case OpenThermMessageID::ASFflags: //5
/*  HB: Application-specific fault flags 
bit: description [ clear/0, set/1]
0: CH enable [ CH is disabled, CH is enabled]
1: DHW enable [ DHW is disabled, DHW is enabled]
2: Cooling enable [ Cooling is disabled, Cooling is
enabled]
3: OTC active [OTC not active, OTC is active]
4: CH2 enable [CH2 is disabled, CH2 is enabled]
5: reserved
6: reserved
7: reserved

LB: OEM fault code
An OEM-specific fault/error cod
*/
        if(u88)
          OTDebugInfo[5]++;
        SmOT.Fault = u88;
        break;        

    case OpenThermMessageID::OEMDiagnosticCode:
        if(u88)
          OTDebugInfo[6]++;
        SmOT.OEMDcode = u88;
        SmOT.rcode[4] = u88;
        break;
        

    default:
//        Serial.println("Response: " + String(response, HEX) + ", id=" + String(id));
      ;
    }
}

unsigned int buildRequest(void)
{   static int st = 0, raz=0;
    unsigned int request = 0;
    switch(st)
    {
      case 0: // запрос статуса
        request = ot.buildSetBoilerStatusRequest(enableCentralHeating, enableHotWater, enableCooling, false,false);
        st++;
      break;
      case 1: //setBoilerTemperature
        st++;
          if(SmOT.need_setT)
          {    //Set Boiler Temperature to 
              request = ot.buildSetBoilerTemperatureRequest(SmOT.Tset);
               SmOT.need_setT = 0;
               break;
          } else {
            raz++;
            if(raz > 100)
                SmOT.need_setT  = 1;
          }
     // break; специально пропущен !!!!

      case 2: //getBoilerTemperature
          request = ot.buildGetBoilerTemperatureRequest();
          st++;
      break;

      case 3: //getReturnTemperature
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tret, 0);
          st++;
      break;

      case 4: //getDHWTemperature
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0);
          st++;
      break;

      case 5: //getModulation
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::RelModLevel, 0);
          st++;
      break;

      case 6: //getPressure
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::CHPressure, 0);
          st++;
      break;

      case  7: //
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::MaxRelModLevelSetting, 0);
          st++;
      break; 

      case 8: //getFault
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::ASFflags, 0);
         st = 0;
        if(SmOT.BoilerStatus & 0x01)
              st++;
      break;

      case 9: //getFault
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::OEMDiagnosticCode, 0);
         st = 0;
      break;

    }
    return request;
}

/* return 0 if no response, 1 if have responce */
int OTloop(void)
{   static int st = 0;
    int rc = 0;
    switch(st)
    {
      case 0:
      if (ot.isReady()) 
      {  unsigned int request = buildRequest();
         if(ot.sendRequestAync(request))    // 	status = OpenThermStatus::RESPONSE_WAITING;    
              st++;
      }
      break;

      case 1:
//        if((ot.status ==  OpenThermStatus::RESPONSE_READY) || (ot.status ==  OpenThermStatus::RESPONSE_INVALID))
        {   st++;
        }

//        if((ot.status !=  OpenThermStatus::RESPONSE_RECEIVING) && (ot.status !=  OpenThermStatus::RESPONSE_WAITING))
//        { st++;
//        }
      break;

      case 2:
       ot.process();
        if(ot.status ==  OpenThermStatus::READY)
        { st = 0;
          rc = 1;
        }
      break;
    }

    return rc;
}

#define OT_CICLE_TIME 500

void loop(void)
{   static unsigned long t0=0; // t1=0;
    unsigned long t;
    int dt;
   t = millis();
   dt = t - t0;
  if(dt < OT_CICLE_TIME)
  {  loop2();
  } else  if( OTloop() ) {
      t0 = millis();
  }  else {
     loop2();
  }
}

/* web, udp, DS1820 */
void loop2(void)
{   static int irot = 0;
    static unsigned long t, dt, t0=0; // t1=0;
     t = millis();
     dt = t - t0;
     if(dt > 1)  //быстро моргаем раз в мсек
     {
    LedSts = (LedSts+1)&0x01;
     digitalWrite(LED_BUILTIN, LedSts);   
    LedSts = (LedSts+1)&0x01;
     digitalWrite(LED_BUILTIN, LedSts);   
        t0 = t;
      }
    switch(irot)
    {  case 0: 
       loop_web();
          irot++;
        break;
        case 1:
         SmOT.loop();
          irot++;
        break;
        case 2:
//Serial.printf("loop_udp\n");

         loop_udp();
          irot++;
        break;
        case 3:
        loopDS1820();
          irot = 0;
        break;
    }
}


void loop_old()
{
//    unsigned long response;
    unsigned long t;
    int dt, dt0;
static int irot = 0;
static unsigned long t0=0; // t1=0;
OpenThermResponseStatus responseStatus;

   t = millis();
   dt = t - t0;

  if(dt < 800)
  {
    switch(irot)
    {  case 0: 
   loop_web();
   dt0 = millis() - t;
          irot++;
        break;
        case 1:
   SmOT.loop();
   dt0 = millis() - t;
          irot++;
        break;
        case 2:
   loop_udp();
   dt0 = millis() - t;
          irot++;
        break;
        case 3:
//    digitalWrite(LED_BUILTIN, HIGH);   
        loopDS1820();
//      digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
   dt0 = millis() - t;
          irot = 0;
        break;
    }
    return;
   }
  if(dt > 800)
  {
      digitalWrite(LED_BUILTIN, HIGH);   
Serial.println("Кю");
            long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    SmOT.rcode[0] = millis() - t;
      digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    responseStatus = ot.getLastResponseStatus();
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
        status_OT = 0;
        SmOT.BoilerStatus = response;
        Serial.println("Central Heating: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
        Serial.println("Hot Water: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
        Serial.println("Flame: " + String(ot.isFlameOn(response) ? "on" : "off"));
        SmOT.response = response; 
    } else if (responseStatus == OpenThermResponseStatus::NONE) {
       status_OT = -1;
        Serial.println("Error: OpenTherm is not initialized");
    } else if (responseStatus == OpenThermResponseStatus::INVALID) {
       if(status_OT != -1)
          Serial.println("Error: Invalid response " + String(response, HEX));
       status_OT = 1;
    } else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
       if(status_OT != 2)
          Serial.println("Error: OT Response timeout"); // 1034 ms
       status_OT = 2;
    }
    SmOT.stsOT = status_OT; 

//    Serial.printf("OT status %x  SmOT.status=%x %li\n", status_OT, SmOT.status, millis());

    
    if(status_OT == 0)
    {  static int round = 0;
      switch(round)
      {
          case 0:
          if(SmOT.need_setT)
          {    //Set Boiler Temperature to 64 degrees C
              if (ot.setBoilerTemperature(SmOT.Tset)) 
              {   SmOT.need_setT = 0;
              }
          }
          round++;
          break;

          case 1:
          {
    //Get Boiler Temperature
    float ch_temperature = ot.getBoilerTemperature();
//    SmOT.rcode[1] = ot.Lastresponse;
    responseStatus = ot.getLastResponseStatus();
    if (responseStatus == OpenThermResponseStatus::SUCCESS) 
    {     Serial.println("CH temperature is " + String(ch_temperature) + " degrees C");
          SmOT.BoilerT = ch_temperature;
//          if( *((int *)&ch_temperature) == 0x40428000)
//          {      SmOT.rcode[2] = SmOT.rcode[0];
//                 SmOT.rcode[3] = ot.Lastresponse;
//          }
    }
   dt0 = millis() - t;
   SmOT.rcode[1] = dt0;

          round++;
          }
        break;
          case 2:
          {
    //Get DHW Temperature
      float dhw_temperature = ot.getDHWTemperature();
    responseStatus = ot.getLastResponseStatus();
    if (responseStatus == OpenThermResponseStatus::SUCCESS) 
    {
      Serial.println("DHW temperature is " + String(dhw_temperature) + " degrees C");
      SmOT.dhw_t = dhw_temperature;
//          if( *((int *)&dhw_temperature) == 0x40428000)
//                SmOT.rcode[1] = ot.Lastresponse;
    }
   dt0 = millis() - t;
   SmOT.rcode[2] = dt0;
          round++;
          }
        break;
          case 3:
          {float ret_temperature = ot.getReturnTemperature();
    responseStatus = ot.getLastResponseStatus();
    if (responseStatus == OpenThermResponseStatus::SUCCESS) 
    {
            SmOT.RetT = ret_temperature;
//          if( *((int *)&ret_temperature) == 0x40428000)
//                SmOT.rcode[2] = ot.Lastresponse;

#if SERIAL_DEBUG 
      Serial.println("ReturnTemperature is " + String(ret_temperature) + " degrees C");
#endif
    }
   dt0 = millis() - t;
   SmOT.rcode[3] = dt0;
          round++;
          }
        break;
          case 4:
          {
            SmOT.FlameModulation = ot.getModulation() ;
//          if( *((int *)&SmOT.FlameModulation) == 0x40428000)
//                SmOT.rcode[3] = ot.Lastresponse;
#if SERIAL_DEBUG 
      Serial.println("FlameModulation is " + String(SmOT.FlameModulation));
#endif
   dt0 = millis() - t;
   SmOT.rcode[4] = dt0;

          round++;
          }
        break;
          case 5:
          { SmOT.Pressure = ot.getPressure();
//          if( *((int *)&SmOT.Pressure) == 0x40428000)
//                SmOT.rcode[4] = ot.Lastresponse;
#if SERIAL_DEBUG 
      Serial.println("Pressure  is " + String(SmOT.Pressure ));
#endif
   dt0 = millis() - t;
          round++;
          }
        break;

          case 6:
          { SmOT.Fault = ot.getFault();
      Serial.println("Fault is " + String(SmOT.Fault));
          round++;
          }
   dt0 = millis() - t;
        break;
          case 7:
          { // int i;
          round = 0;

          }
        break;

      }

//Set DHW setpoint to 40 degrees C
//      ot.setDHWSetpoint(40);
//      Serial.println();
    } else {
;
    }

    t0 = millis();

  }
//    delay(1000);
}
