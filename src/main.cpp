/* main.cpp */
// контроллер OpenTherm на ESP32/ESP8266 с измерением температуры
// WiFi, Captive Portal, Web доступ
// клиент-сервер (UDP или TCP)
// based on OpenTherm Master Communication Example By: Ihor Melnyk
//

#include <time.h>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
typedef ESP8266WebServer WEBServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
typedef WebServer WEBServer;
#endif

#include "OpenTherm.h"
#include "SmartDevice.hpp"
#include "Smart_Config.h"
#include "SD_OpenTherm.hpp"

/************************************/
extern void setup_web_common(void);
extern void loop_web(void);
extern void setup_tcpudp(SmartDevice *psd);

extern void loop_udp(int sts);
extern void loop_tcp(int sts);

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
  setup_tcpudp( &SmOT );
}


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
//    bool enableCentralHeating = true;
//    bool enableHotWater = true;
//    bool enableCooling = false;

//
void OTprocessResponse(unsigned long response, OpenThermResponseStatus status)
{   float t;
    uint16_t u88;
    byte id;
    int parity, messagetype;

    if(SmOT.TestCmd == 2)
    {
      id = (response >> 16 & 0xFF);
      if(id == SmOT.TestId)
      {
//        Serial.printf("TestCmd processResponse %d %d\n", response,  status);
        SmOT.TestResponse = response;
        SmOT.TestStatus = status;
        SmOT.TestCmd = 0;
      }
    }

    parity = ot.parity(response);
    if(parity)
    { OTDebugInfo[1]++;
        Serial.printf("Parity error\n");
      return;
    }
    
    if (status == OpenThermResponseStatus::SUCCESS) {
        SmOT.stsOT = 0;
        SmOT.response = response; 
        OTDebugInfo[0]++;
    } else if (status == OpenThermResponseStatus::NONE) {
      // SmOT.stsOT = -1;  // ??
        OTDebugInfo[2]++;
    } else if (status == OpenThermResponseStatus::INVALID) {
       //SmOT.stsOT = 1;
        OTDebugInfo[3]++;
    } else if (status == OpenThermResponseStatus::TIMEOUT) {
       if(SmOT.stsOT != -1)
            SmOT.stsOT = 2;
        OTDebugInfo[4]++;
        return;
    }

    messagetype = ot.getMessageType(response);
    if(messagetype == DATA_INVALID)
    { OTDebugInfo[7]++;
        Serial.printf("DATA_INVALID\n");
      return;
    }
      
    if(messagetype == UNKNOWN_DATA_ID)
    { OTDebugInfo[8]++;
      id = (response >> 16 & 0xFF);
      ot.update_OTid(id, 0);
//        Serial.printf("UNKNOWN_DATA_ID %d\n", id);
      return;
    }
    if(messagetype != READ_ACK && messagetype != WRITE_ACK )
    { OTDebugInfo[9]++;
        Serial.printf("Messagetype  %d!!! Status %d %d\n", messagetype, status, SmOT.stsOT );
      return;
    }

    if(SmOT.stsOT != 0)
        return;
    SmOT.t_lastwork = time(nullptr);

    id = (response >> 16 & 0xFF);
    u88 = (response & 0xffff);
    t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
    ot.update_OTid(id, 1);

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
    case OpenThermMessageID::TflowCH2: //31
        SmOT.BoilerT2 = t;
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

//  TestCmd = TestId = TestPar =  TestResponce = 0;
unsigned int buildTestRequest(void)
{   unsigned int request = 0;
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, ( OpenThermMessageID) SmOT.TestId, SmOT.TestPar);
    return request;
}

unsigned int buildRequest(void)
{   static int st = 0, raz=0;
    unsigned int request = 0;

    if(SmOT.TestCmd == 1)
    {   request = buildTestRequest();  
        SmOT.TestCmd++;
        return request;
    }

M0:    
    switch(st)
    {
      case 0: // запрос статуса
// Serial.printf("0 Request: %d\n",OpenThermMessageID::Status);
        request = ot.buildSetBoilerStatusRequest(SmOT.enable_CentralHeating, SmOT.enable_HotWater, SmOT.enable_Cooling, false, SmOT.enable_CentralHeating2);
         
        st++;
      break;
      case 1: //setBoilerTemperature
        st++;
          if(SmOT.need_set_T)
          {    //Set Boiler Temperature to 
// Serial.printf("1 Request: %d\n",OpenThermMessageID::TSet);
              request = ot.buildSetBoilerTemperatureRequest(SmOT.Tset); //1
               SmOT.need_set_T = 0;
               break;
          } else if(SmOT.need_set_dhwT) {
// Serial.printf("1a Request: %d\n",OpenThermMessageID::TdhwSet);
              request = ot.setDHWSetpoint(SmOT.TdhwSet); //56
              SmOT.need_set_dhwT = 0;
          } else if(SmOT.need_set_T2) {
              request = ot.buildSetBoilerCH2TemperatureRequest(SmOT.Tset2); //8
               SmOT.need_set_T2 = 0;
               break;
          } else {
            raz++;
            if(raz > 100)
            {   if(SmOT.enable_CentralHeating)
                    SmOT.need_set_T  = 1; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
                if(SmOT.enable_HotWater) 
                    SmOT.need_set_dhwT = 1;                   
                if(SmOT.enable_CentralHeating2)
                    SmOT.need_set_T2  = 1; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
                raz = 0;
            }
          }
     // break; especially omitted = специально пропущен !!!! 

      case 2: //getBoilerTemperature
// Serial.printf("2 Request: %d\n",OpenThermMessageID::Tboiler);
          request = ot.buildGetBoilerTemperatureRequest();
          st++;
      break;

      case 3: //getReturnTemperature
// Serial.printf("3 Request: %d\n",OpenThermMessageID::Tret);
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tret, 0); //28
          st++;
      break;

      case 4: //getDHWTemperature
// Serial.printf("4 Request: %d\n",OpenThermMessageID::Tdhw);
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0); //26
          st++;
      break;

      case 5: //getModulation
// Serial.printf("5 Request: %d\n",OpenThermMessageID::RelModLevel);
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::RelModLevel, 0); //17
          st++;
      break;

      case 6: //getPressure
        st++; 
        if(ot.OTid_used(OpenThermMessageID::CHPressure))
        {  
// Serial.printf("6  cRequest: %d\n",OpenThermMessageID::CHPressure);
           request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::CHPressure, 0); //18
        }  else {
          goto M0;
        }
//          st++;
      break;
      case 7: //TSetCH2
         st++;
        if(SmOT.enable_CentralHeating2)
        {
// Serial.printf("7 Request: %d\n",OpenThermMessageID::TflowCH2);
          request = ot.buildGetBoilerCH2TemperatureRequest(); //TflowCH2
        }  else {
          goto M0;
        }
      break;

      case 8: //getFault flags
 //Serial.printf("8 Request: %d\n",OpenThermMessageID::ASFflags);
        request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::ASFflags, 0);
        if(SmOT.BoilerStatus & 0x01)
            st++;
        else 
           st = 0;
      break;

      case 9: //getFault code
 //Serial.printf("9 Request: %d\n",OpenThermMessageID::OEMDiagnosticCode);
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::OEMDiagnosticCode, 0);
         st = 0;
      break;

    }
    return request;
}

/* return 0 if no response, 1 if have responce */
int OTloop(void)
{   static int st = 1;
    int rc = 0;

    switch(st)
    {
      case 0:
      if (ot.isReady()) 
      {  unsigned int request = buildRequest();
/*      
          unsigned int id;
          id = (request >> 16 & 0xFF);
           Serial.printf("Request:  %d\n",  id);
*/           
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
        { // unsigned int id;
          st = 0;
          rc = 1;
//          id = (ot.getLastResponse() >> 16 & 0xFF);
//             Serial.printf("Last ResponseStatus:  %d Response id %d\n",  ot.getLastResponseStatus(), id);
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
#if 1   
   t = millis();
   dt = t - t0;
  if(dt < OT_CICLE_TIME)
  {  loop2();
  } else  if( OTloop() ) {
      t0 = millis();
  }  else {
     loop2();
  }
#else  //debug without OT  
       loop_web();
#endif // 0

}

int minRamFree=-1;

/* web, udp, DS1820 */
void loop2(void)
{   static int irot = 0;
    static unsigned long  t0=0; // t1=0;
    unsigned long t, dt;
     t = millis();
     dt = t - t0;
     if(LedSts) //быстро моргаем раз в мсек
     {  if(dt > 2)
        { LedSts = (LedSts+1)&0x01;
          digitalWrite(LED_BUILTIN, LedSts);   
          t0 = t;
        }
     } else {
        int wt = 500;
        if(SmOT.stsOT == 0) wt = 2000;
        else if(SmOT.stsOT > 0) wt = 1000;
        if(dt > (unsigned long)wt)
        { LedSts = (LedSts+1)&0x01;
          digitalWrite(LED_BUILTIN, LedSts);   
          t0 = t;          
        }
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
{ static int oldFree = 0;
  int free;
  free = ESP.getFreeHeap();
  if(minRamFree == -1)
	    minRamFree = free;
  else if(free < minRamFree)
  {	  minRamFree = free;
  }
#if defined(ARDUINO_ARCH_ESP8266)
  if(abs(free - oldFree) > 2500)
#elif defined(ARDUINO_ARCH_ESP32)
  if(abs(free - oldFree) > 5000)
#endif
  { Serial.printf("IRAM free: %6d bytes (min %d)\n", free, minRamFree);
    oldFree = free;
  }
}
         loop_udp(SmOT.UDPserver_sts);
         
          irot++;
        break;
        case 3:
         loop_tcp(SmOT.TCPserver_sts);
          irot++;
        break;
        case 4:
        loopDS1820();
          irot = 0;
        break;
    }
}

