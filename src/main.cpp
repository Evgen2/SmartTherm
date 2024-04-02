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

#include "Smart_Config.h"
#include "OpenTherm.h"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

/************************************/
extern void setup_web_common(void);
extern void loop_web(void);
extern void setup_tcpudp(SmartDevice *psd);

extern void loop_udp(int sts);
extern void loop_tcp(int sts);

void loop_time(void);

/************************************/
class SD_Termo SmOT;

//Задаем пины
#if defined(ARDUINO_ARCH_ESP8266)

  const int inPin = D7;  // OpenTherm  in   D7 GIP013 Nodemsu
  const int outPin = D8;  // OpenTherm out  D8 GPIO15 Nodemsu

  const int DS1820_1 = D6; //
  const int DS1820_2 = D2; //

#elif defined(ARDUINO_ARCH_ESP32)
  const int inPin = 16;  // OpenTherm  in RX2 esp32
  const int outPin = 4;  // OpenTherm out D4 esp32

  const int DS1820_1 = 15; // D15 esp32  3 снизу
  const int DS1820_2 = 26; // D26 esp32  7 снизу

#endif

/*  некоторым котлам (например, Baxi Fourtech/Luna 3) не нравится OpenThermMessageID::MConfigMMemberIDcode
    настолько, что они перестают отвечать на запросы
    OTstartSts_MAX  2 - не использовать MConfigMMemberIDcode
    OTstartSts_MAX  3 - использовать MConfigMMemberIDcode (if SmOT.UseID2 with code SmOT.ID2masterID)
*/
static int OTstartSts_MAX = 2;

OpenTherm ot(inPin, outPin);
void OTprocessResponse(unsigned long response, OpenThermResponseStatus status);
int OTloop(void);
void loop2(void);
#if OT_DEBUG
void LogOT(int code, byte id, int messagetype, unsigned int u88);
#endif

/* DS18b20 */
#include <OneWire.h>
#include <DS18B20.h>

void loopDS1820(void);
void setupDS1820(void);

OneWire oneWire1(DS1820_1);
OneWire oneWire2(DS1820_2);
DS18B20 Tsensor1(&oneWire1);
DS18B20 Tsensor2(&oneWire2);
extern int OTDebugInfo[12];


void IRAM_ATTR handleInterrupt() {
    ot.handleInterrupt();
}

static int OTstartSts = 0;
int LedSts = 0; //LOW

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, LedSts);   // Turn the LED on (Note that LOW is the voltage level
  delay(1000);
  Serial.begin(115200);
  
  //Serial.println(Jopa);
  Serial.println(F("Start"));
  Serial.println(IDENTIFY_TEXT);
  Serial.printf("Vers %d.%d.%d build %s\n",SmOT.Vers, SmOT.SubVers,SmOT.SubVers1,  SmOT.BiosDate);


// Serial.printf((PGM_P)F("Vers %d.%d build %s\n"),SmOT.Vers, SmOT.SubVers, SmOT.BiosDate);

//  Serial.printf("IRAM free: %6d bytes\n", ESP.getFreeHeap());
  Serial.printf((PGM_P)F("IRAM free: %6d bytes\n"), ESP.getFreeHeap());

  LedSts=1;
  digitalWrite(LED_BUILTIN, LedSts);   

/*******************************************/
  setupDS1820();

  ot.begin(handleInterrupt, OTprocessResponse);
  setup_web_common();
  setup_tcpudp( &SmOT );

  if(SmOT.UseID2)
      OTstartSts_MAX = 3;
  else 
      OTstartSts_MAX = 2;

#if SERVER_DEBUG
  SmOT.TCPserver_sts = 2;  /* статус сервера */
  SmOT.TCPserver_t = millis();
  SmOT.TCPserver_port = 8876;  
  SmOT.TCPserver_repot_period = 10000;
  SmOT.tcp_remoteIP.fromString("192.168.0.100");

  Serial.printf("TCPserver_repot_period=%d TCPserver_port=%d\n", SmOT.TCPserver_repot_period, SmOT.TCPserver_port);

#endif	

}


int status_OT = -1;
static int _SConfigSMemberIDcode = 0;

void setupDS1820(void)
{//  Serial.print("DS18B20 Library version: ");
 //  Serial.println(DS18B20_LIB_VERSION);

  SmOT.status = 0x0;

  if(Tsensor1.begin() == false)
  {   SmOT.stsT1 = -1;
      SmOT.status |= 0x02;
      Serial.printf((PGM_P)F("ERROR: No DS18b20(1) found on pin %i\n"), DS1820_1);
      delay(100);
      if(Tsensor1.begin() )
      {   Serial.println(F("2nd attempt(1) Ok"));
          goto M1;
      }

  }  else {
M1:      SmOT.stsT1 = 0;
      SmOT.status |= 0x01;

      Tsensor1.setResolution(12);
      Tsensor1.setConfig(DS18B20_CRC);  // or 1
      Serial.printf((PGM_P)F("DS18b20(1) found on pin %i\n"), DS1820_1);
  }

  if(Tsensor2.begin() == false)
  {   SmOT.stsT2 = -1;
      SmOT.status |= 0x0200;
      Serial.printf((PGM_P)F("ERROR: No DS18b20(2) found on pin %i\n"), DS1820_2);
      delay(100);
      if(Tsensor2.begin() )
      {   Serial.println(F("2nd attempt(2) Ok"));
          goto M2;
      }

  }  else {
M2:   SmOT.stsT2 = 0;
      SmOT.status |= 0x0100;
      Tsensor2.setResolution(12);
      Tsensor2.setConfig(DS18B20_CRC);  // or 1
      Serial.printf((PGM_P)F("DS18b20(2) found on pin %i\n"), DS1820_2);
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
              Serial.println(F("ERROR: DS1 timeout or disconnect"));
#endif                
              break;
            }
        } else {
          rc = Tsensor1.isConversionComplete();
        }
        if(rc)
        { t = Tsensor1.getTempC();
          SmOT.status &= ~0x04; // сброс бита таймаута
          if (t == DEVICE_CRC_ERROR)
          { SmOT.stsT1 = 2;
            SmOT.status |= 0x10;
#if SERIAL_DEBUG 
            Serial.println(F("ERROR: DS1 CRC error"));
#endif            
          } else {
            SmOT.status &= ~0x10; // сброс бита CRC error
            SmOT.t1 = t;
            if(SmOT.stsT1 == 1)
                SmOT.t1 = (SmOT.t1 + t) * 0.5;
            else
                SmOT.t1 = t;
            SmOT.stsT1 = 1;
            SmOT.OnChangeT(t,0);    
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
            Serial.println(F("ERROR: DS2 timeout or disconnect"));
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
          { SmOT.stsT2 = 2;
            SmOT.status |= 0x1000;
      #if SERIAL_DEBUG 
            Serial.println(F("ERROR: DS2 CRC error"));
      #endif            
          } else {
            SmOT.status &= ~0x1000; // сброс бита CRC error

            if(SmOT.stsT2 == 1)
                SmOT.t2 = (SmOT.t2 + t) * 0.5;
            else
                SmOT.t2 = t;
            SmOT.stsT2 = 1;
            SmOT.OnChangeT(t,1);    

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
static int timeOutcounter = 0;

    SmOT.RespMillis = millis();
    if(SmOT.TestCmd == 2)
    {
      id = (response >> 16 & 0xFF);
      if(id == (SmOT.TestId & 0xff))
      {
#if OT_DEBUG
    Serial.printf("TestCmd processResponse %x %x\n", response,  status);
#endif
    Serial.printf("TestCmd processResponse %x %x\n", response,  status);
        SmOT.TestResponse = response;
        SmOT.TestStatus = status;
        SmOT.TestCmd = 0;
        return;
      }
    }
  
    if (status == OpenThermResponseStatus::SUCCESS) {
        SmOT.stsOT = timeOutcounter = 0;
        SmOT.response = response; 
        OTDebugInfo[0]++;
    } else if (status == OpenThermResponseStatus::NONE) {
#if OT_DEBUG
      LogOT(-3, 0,  0,  0);
#endif         
      // SmOT.stsOT = -1;  // ??
        OTDebugInfo[2]++;
    } else if (status == OpenThermResponseStatus::INVALID) {
       //SmOT.stsOT = 1;
#if OT_DEBUG
      LogOT(-2, 0,  0,  0);
#endif         
        OTDebugInfo[3]++;
    } else if (status == OpenThermResponseStatus::TIMEOUT) {
      if(SmOT.stsOT != -1)
	    { if(timeOutcounter > 10)
		    {	SmOT.stsOT = 2;
		    } else {
			    timeOutcounter++;
		    }	
      }
      OTDebugInfo[4]++;
#if OT_DEBUG
      LogOT(-1, 0,  0,  0);
#endif         
      return;
    }

#if OT_DEBUG
  { unsigned int u88;
    u88 = (response & 0xffff);
    id = (response >> 16 & 0xFF);
    parity = ot.parity(response);
    messagetype = ot.getMessageType(response);
    if(parity)
      LogOT(0,  id,  messagetype,  u88);
    else 
      LogOT(1,  id,  messagetype,  u88);
  } 
#endif         

    parity = ot.parity(response);
    if(parity)
    { OTDebugInfo[1]++;
#if SERIAL_DEBUG 
        Serial.println(F("Parity error"));
#endif        
      return;
    }

    messagetype = ot.getMessageType(response);
    if(messagetype == DATA_INVALID)
    { OTDebugInfo[7]++;
#if SERIAL_DEBUG 
        Serial.println(F("DATA_INVALID"));
#endif        
      return;
    }
    
      
    if(messagetype == UNKNOWN_DATA_ID)
    { OTDebugInfo[8]++;
      id = (response >> 16 & 0xFF);
      ot.update_OTid(id, 0);
         if(OTstartSts > 0)  OTstartSts++;
//        Serial.printf("UNKNOWN_DATA_ID %d\n", id);
      return;
    }
    if(messagetype != READ_ACK && messagetype != WRITE_ACK )
    { OTDebugInfo[9]++;
         if(OTstartSts > 0)  OTstartSts++;
#if SERIAL_DEBUG 
        Serial.printf("Messagetype  %d!!! Status %d %d\n", messagetype, status, SmOT.stsOT );
#endif        
      return;
    }
    
    if(SmOT.stsOT != 0)
        return;
    SmOT.t_lastwork = time(nullptr);

    id = (response >> 16 & 0xFF);
    u88 = (response & 0xffff);
    t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
    ot.update_OTid(id, 1);
    if(id != ot.LastRequestId)
    { OTDebugInfo[10]++;
#if SERIAL_DEBUG 
        Serial.printf("Resp id %d != Req id %d\n", id, ot.LastRequestId );
#endif        
      return;
    }

    switch (id)
    {
    case OpenThermMessageID::Status:  //0
/*  HB: Master status 
    bit: description [ clear/0, set/1]
0: CH enable [ CH is disabled, CH is enabled]
1: DHW enable [ DHW is disabled, DHW is enabled]
2: Cooling enable [ Cooling is disabled, Cooling is enabled]
3: OTC active [OTC not active, OTC is active]
4: CH2 enable [CH2 is disabled, CH2 is enabled]
5: reserved | Summer/winter mode
6: reserved | Master status: DHW blocking
7: reserved
*/
/*  LB: Slave status   
bit: description [ clear/0, set/1]
0: fault indication [ no fault, fault ]
1: CH mode [CH not active, CH active]
2: DHW mode [ DHW not active, DHW active]
3: Flame status [ flame off, flame on ]
4: Cooling status [ cooling mode not active, cooling mode active ]
5: CH2 mode [CH2 not active, CH2 active]
6: diagnostic/service indication [no diagnostics, diagnostic event]
7: reserved | Electricity production (???)
*/   
//        boiler_status = response & 0xFF;
        if((u88 & 0x08) != (SmOT.BoilerStatus & 0x08))
             SmOT.Bstat.calcNflame(u88 & 0x08);

        SmOT.BoilerStatus = u88;
//        Serial.printf("BoilerStatus: %x %x\n", u88, response);

//        Serial.println("Central Heating: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
//        Serial.println("Hot Water: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
//        Serial.println("Flame: " + String(ot.isFlameOn(response) ? "on" : "off"));

//        curr_item->status = boiler_status;
//        Serial.println("Boiler status: " + String(boiler_status, BIN));
         if(OTstartSts == 0)  OTstartSts++;

        break;

    case OpenThermMessageID::TSet:  // 1
        SmOT.Tset_r = t;
        if(SmOT.Tset_r == SmOT.Tset)
                SmOT.need_set_T = 0;

//        Serial.println("Set CH temp: " + String(t));
        break;
        
    case OpenThermMessageID::MConfigMMemberIDcode: //2
         if(OTstartSts == 2)  OTstartSts++;
       Serial.printf((PGM_P)F("OpenThermMessageID::MConfigMMemberIDcode, %d\n"), OTstartSts);

        break;

    case OpenThermMessageID::SConfigSMemberIDcode:  //3
     _SConfigSMemberIDcode = u88;
         if(OTstartSts == 1) 
         { OTstartSts++;
            if(_SConfigSMemberIDcode & 0x100)
            { SmOT.HotWater_present = true;
            } else {
              SmOT.HotWater_present = false;
              SmOT.enable_HotWater = false;
            }
            if(_SConfigSMemberIDcode & 0x2000)
            {   SmOT.CH2_present  = true;
            } else {
                SmOT.CH2_present  = false;
                SmOT.enable_CentralHeating2  = false;               
            }
            SmOT.OTmemberCode = _SConfigSMemberIDcode & 0xff;
         }
//        Serial.printf("OTstartSts %d: u88 %x SmOT.HotWater_present = %d\n", OTstartSts, u88, SmOT.HotWater_present );
        break;

    case OpenThermMessageID::Tboiler:  //25
        SmOT.BoilerT = t;
        break;

    case OpenThermMessageID::Tdhw: //26
        SmOT.dhw_t = t;
        break;

    case OpenThermMessageID::Toutside: //27
    if( ot.OTid_used(OpenThermMessageID::Toutside) == 1)
      SmOT.Toutside = (SmOT.Toutside + t) * 0.5;
    else
      SmOT.Toutside = t;
    SmOT.OnChangeT(t,2);

        break;

    case OpenThermMessageID::Tret: //28
        SmOT.RetT = t;
        break;

    case OpenThermMessageID::TflowCH2: //31
        SmOT.BoilerT2 = t;
        break;

    case OpenThermMessageID::Texhaust: //33
        SmOT.Texhaust = (float)u88;
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
        SmOT.Bstat.calcIntegral(t);
        break;

    case OpenThermMessageID::CHPressure: //18 Water pressure in CH circuit
        SmOT.Pressure = t;
        break;

    case OpenThermMessageID::ASFflags: //5
/*  HB: Application-specific fault flags 
bit: description [ clear/0, set/1]
0: Servicerequest [servicenotreq’d,servicerequired]
1: Lockout-reset [remoteresetdisabled,rrenabled]
2: Lowwaterpress[noWPfault,waterpressurefault]
3: Gas/flamefault [noG/Ffault,gas/flamefault]
4: Airpressfault [noAPfault,airpressurefault]
5: Waterover-temp[noOvTfault,over-temperat.Fault]
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
//        Serial.printf("OEMDcode: %x\n", SmOT.OEMDcode);
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
    if(SmOT.TestId & 0x1000)
          request = ot.buildRequest(OpenThermMessageType::WRITE_DATA, ( OpenThermMessageID) (SmOT.TestId & 0x0ff), SmOT.TestPar);
    else
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, ( OpenThermMessageID) SmOT.TestId, SmOT.TestPar);
    
    Serial.printf("TestRequest: %x SmOT.TestId %x\n", request, SmOT.TestId);
#if OT_DEBUG
    Serial.printf("TestRequest: %x\n", request);
#endif    
    return request;
}

unsigned int buildRequestOnStart(void)
{  unsigned int request = 0;

    switch(OTstartSts) 
    {
      case 0: // запрос статуса
// Serial.printf("0 Request: %d\n",OpenThermMessageID::Status);
        request = ot.buildSetBoilerStatusRequest(SmOT.enable_CentralHeating, SmOT.enable_HotWater, SmOT.enable_Cooling, false, SmOT.enable_CentralHeating2);
#if OT_DEBUG
  { unsigned int u88;
    u88 = (request & 0xffff);
    LogOT(2,  OpenThermMessageID::Status,  OpenThermMessageType::READ_DATA,  u88);
//  Serial.printf("ReqS: %d READ_DATA %04x (Status %d %d %d %d)\n", OpenThermMessageID::Status,  u88, SmOT.enable_CentralHeating, SmOT.enable_HotWater, SmOT.enable_Cooling,  SmOT.enable_CentralHeating2);
  } 
#endif         
      break;
      
      case 1: // запрос SConfigSMemberIDcode
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::SConfigSMemberIDcode, 0); //3
#if OT_DEBUG
  { unsigned int u88;
    u88 = (request & 0xffff);
    LogOT(3,  OpenThermMessageID::SConfigSMemberIDcode,  OpenThermMessageType::READ_DATA,  u88);
//    Serial.printf("ReqS: %d READ_DATA %04x (SConfigSMemberIDcode)\n", OpenThermMessageID::SConfigSMemberIDcode,  u88);
  } 
#endif         
      break;

      case 2: // OpenThermMessageID::MConfigMMemberIDcode:
          request = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MConfigMMemberIDcode, SmOT.ID2masterID /* (_SConfigSMemberIDcode&0xff) */); //3
#if OT_DEBUG
  { unsigned int u88;
    u88 = (request & 0xffff);
    LogOT(4,  OpenThermMessageID::MConfigMMemberIDcode,  OpenThermMessageType::WRITE_DATA,  u88);
  } 
#endif         
        break;
    }

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
              if(SmOT.need_set_T > 0)  //test
                  SmOT.need_set_T--;
               break;
          } else if(SmOT.need_set_dhwT) {
// Serial.printf("1a Request: %d\n",OpenThermMessageID::TdhwSet);
              request = ot.buildSetDHWSetpointTemperatureRequest(SmOT.TdhwSet); //56
              SmOT.need_set_dhwT = 0;
          } else if(SmOT.need_set_T2) {
              request = ot.buildSetBoilerCH2TemperatureRequest(SmOT.Tset2); //8
               SmOT.need_set_T2 = 0;
               break;
          } else {
            raz++;
            if(raz > 100)
            {   if(SmOT.enable_CentralHeating)
                    SmOT.need_set_T  = 10; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
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
        st++; 
        if(ot.OTid_used(OpenThermMessageID::Tret))
        {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tret, 0); //28
        }  else {
           goto M0;
        }
      break;

      case 4: //getDHWTemperature
// Serial.printf("4 Request: %d\n",OpenThermMessageID::Tdhw);
        st++;
        if(SmOT.HotWater_present) 
        {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0); //26
        }  else {
              goto M0;
        }
      break;

      case 5: //getModulation
// Serial.printf("5 Request: %d\n",OpenThermMessageID::RelModLevel);
        st++; 
        if(ot.OTid_used(OpenThermMessageID::RelModLevel))
        {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::RelModLevel, 0); //17
        }  else {
          goto M0;
        }
      break;

      case 6: //getPressure
        st++; 
        if(ot.OTid_used(OpenThermMessageID::CHPressure))
        {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::CHPressure, 0); //18
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

      case 8:
        st++; 
        if(ot.OTid_used(OpenThermMessageID::Toutside))
        {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Toutside, 0); //27
        }  else {
            goto M0;
        }
      break;

      case 9:
        st++; 
        if(ot.OTid_used(OpenThermMessageID::Texhaust))
        {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Texhaust, 0); //27
        }  else {
            goto M0;
        }
      break;

      case 10: //getFault flags
 //Serial.printf("8 Request: %d\n",OpenThermMessageID::ASFflags);
        request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::ASFflags, 0);
        if(SmOT.BoilerStatus & 0x01)
            st++;
        else 
           st = 0;
      break;

      case 11: //getFault code
 //Serial.printf("9 Request: %d\n",OpenThermMessageID::OEMDiagnosticCode);
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::OEMDiagnosticCode, 0);
         st = 0;
      break;

    }
#if OT_DEBUG
  { unsigned int u88, id;
    int  messagetype;
    u88 = (request & 0xffff);
    id = (request >> 16 & 0xFF);
    messagetype = ot.getMessageType(request);
//    Serial.printf("Req : %d %d %04x\n", id, messagetype,   u88);
    LogOT(5,  id,  messagetype,  u88);

  } 
#endif         

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
      {  unsigned int request;
#if SERIAL_DEBUG 
          if((millis() - SmOT.RespMillis) < 100)
               Serial.printf((PGM_P)F("OTloop too fast: %d **********\n"), int (millis() - SmOT.RespMillis));
#endif
         if(OTstartSts < OTstartSts_MAX)
            request = buildRequestOnStart();
         else
            request = buildRequest();

/*     
          unsigned int id;
          id = (request >> 16 & 0xFF);
           Serial.printf("Request:  %d\n",  id);
 */           
         if(ot.sendRequestAync(request))    // 	status = OpenThermStatus::RESPONSE_WAITING;    
              st++;
#if SERIAL_DEBUG 
         else
           Serial.println(F("sendRequestAync:  return false"));
#endif           

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

#define OT_CICLE_TIME 300

void loop(void)
{   static unsigned long t0=0; // t1=0;
    unsigned long t;
    int dt;
#if T_DEBUG 
  static int count=0, told=0;
  int dt1;
  t = millis();
  dt1 = t-told;
  if(dt1 > 1000)
  { told = t;
   Serial.printf("loop=%d dt=%d\n", count++, dt1);
  }
#endif


#if 1   
   t = millis();
   dt = t - t0;
  if(dt < OT_CICLE_TIME)
  {  loop2();
  } else  if( OTloop() ) {
//   Serial.printf("raz=%d\n", raz);
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
#if defined(ARDUINO_ARCH_ESP8266)
     if(!LedSts) //быстро моргаем раз в мсек
#elif defined(ARDUINO_ARCH_ESP32)
     if(LedSts) //быстро моргаем раз в мсек
#endif
     {  if(dt > 2)
        { LedSts = (LedSts+1)&0x01;
          digitalWrite(LED_BUILTIN, LedSts);   
//   Serial.printf("dt=%d\n", dt);
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
/************************/ 
//test for lost OT connection
      {  time_t now = time(nullptr);
        double dt;
        dt = difftime(now,SmOT.t_lastwork);
        if(dt > 10.)
        {         //sprintf(str0, "Потеря связи с котлом %.f сек назад", dt);
            if(OTstartSts == OTstartSts_MAX)
            {   OTstartSts = 0;  // init start sequence
                SmOT.HotWater_present = false;
                SmOT.enable_CentralHeating2  = false; 
            }
        }
      }
/************************/                 
        }
//        
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
  int free, needrep=0;
  unsigned int maxFreeBlockSize;

  free = ESP.getFreeHeap();
  if(minRamFree == -1)
	    minRamFree = free;
  else if(free < minRamFree)
  {	  
#if defined(ARDUINO_ARCH_ESP8266)
      needrep = 1;
#elif defined(ARDUINO_ARCH_ESP32)
  if( minRamFree - free  > 5000)
      needrep = 1;
#endif
      minRamFree = free;   
  }
#if defined(ARDUINO_ARCH_ESP8266)
  if(oldFree - free  > 2500)
#elif defined(ARDUINO_ARCH_ESP32)
  if(oldFree - free  > 10000)
#endif
      needrep = 1;
  if(needrep)    
  { 
#if defined(ARDUINO_ARCH_ESP32)
  maxFreeBlockSize = ESP.getMaxAllocHeap();
#else
  maxFreeBlockSize = ESP.getMaxFreeBlockSize();
#endif
    
    Serial.printf((PGM_P)F("IRAM free: %6d bytes (min %d) maxFreeBlock %6d\n"), free, minRamFree, maxFreeBlockSize) ;
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
          irot++;
        break;

        case 5:
        loop_time();
          irot = 0;
        break;
    }
}


void loop_time(void)
{ time_t now;
static time_t prev = 0;
static int hour_prev = 0;
static int mday_prev = 0;
    struct tm *nowtime;
    int year, year_prev;
    time_t dt;

  now = time(nullptr);
  if(now == prev)
      return;

#if PID_USE
    SmOT.loop_PID();
#endif

//    SmOT.Bstat.calcIntegral(0.25); //debug

  nowtime = localtime(&prev);
  year_prev = nowtime ->tm_year;
  nowtime = localtime(&now);
  year = nowtime->tm_year;

  
  prev = now;

  if( year_prev == 70 && year  >= 123)  //change time with nttp server
  {    noInterrupts();
        SmOT.Bstat.NflameOn_h = 0;
        SmOT.Bstat.ModIntegral_h = 0.;
        SmOT.Bstat.ModIntegral_d = 0.;
        SmOT.Bstat.sec_h = SmOT.Bstat.sec_d = 0;

        SmOT.init();        
	      interrupts();
  }
  

//  Serial.printf("%s", ctime(&now));
 
#if SERIAL_DEBUG 

/*
Serial.printf( "%02d.%02d.%d %d:%02d:%02d\n",
          nowtime->tm_mday,nowtime->tm_mon+1,nowtime->tm_year+1900,
		  nowtime->tm_hour, nowtime->tm_min, nowtime->tm_sec);
*/      
#endif

  if(hour_prev != nowtime->tm_hour)
  { hour_prev = nowtime->tm_hour;
    SmOT.Bstat.NflameOn_h_prev = SmOT.Bstat.NflameOn_h;
    SmOT.Bstat.Eff_Mod_h_prev = SmOT.Bstat.Eff_Mod_h;
    noInterrupts();
        SmOT.Bstat.NflameOn_h = 0;
        SmOT.Bstat.ModIntegral_h = 0.;
        SmOT.Bstat.sec_h = 0;
	   interrupts();

    if(mday_prev != nowtime->tm_mday)
    { SmOT.Bstat.NflameOn_day_prev = SmOT.Bstat.NflameOn_day;
      SmOT.Bstat.Eff_Mod_d_prev = SmOT.Bstat.Eff_Mod_d;
    	noInterrupts();
        SmOT.Bstat.NflameOn_h = 0;
        SmOT.Bstat.ModIntegral_d = 0.;
        SmOT.Bstat.sec_d = 0;
	    interrupts();
      mday_prev = nowtime->tm_mday;
    }
  } else {
    
    dt = now - SmOT.Bstat.t_I_last;
        if(SmOT.Bstat.sec_h)
      SmOT.Bstat.Eff_Mod_h = SmOT.Bstat.ModIntegral_h / (float)(SmOT.Bstat.sec_h + dt);

//     Serial.printf("sec_hour = %d Eff_Mod_h=%f ModIntegral_h=%f\n", SmOT.Bstat.sec_h, SmOT.Bstat.Eff_Mod_h, SmOT.Bstat.ModIntegral_h );


    if(SmOT.Bstat.sec_d)
      SmOT.Bstat.Eff_Mod_d = SmOT.Bstat.ModIntegral_d / (float)(SmOT.Bstat.sec_d + dt);

//     Serial.printf("sec_d = %d Eff_Mod_d=%f ModIntegral_d=%f\n", SmOT.Bstat.sec_d, SmOT.Bstat.Eff_Mod_d, SmOT.Bstat.ModIntegral_d );
  }
}

    
#if OT_DEBUG

void LogOT(int code, byte id, int messagetype, unsigned int u88)
{ static int ms_old = 0;
  int ms, dms;
  float t;
  ms = millis();
  dms = ms - ms_old;
  ms_old = ms;

#if OT_DEBUGLOG
SmOT.enable_OTlog = 0; //test
	if(SmOT.enable_OTlog)
	{ short int tmp;
    static unsigned char buf[8];
    static int nm=0;
    buf[0] = (unsigned char) (nm&0xff);
    tmp = dms&0xffff;
    memcpy(&buf[1], &tmp, 2);
    //dms // 2b
    buf[3]  = (unsigned char) (code&0xff);
	   //code  1b
    buf[4]  = (unsigned char) (id&0xff);
	   //id 1b
    buf[5]  = (unsigned char) (messagetype&0xff);
	   // messagetype 1b
	   // u88 2b
    tmp = u88&0xffff;
    memcpy(&buf[6], &tmp, 2);
    SmOT.OTlogBuf.Add(buf);
//  Serial.printf("LogOT: Lbuf %d \n", SmOT.OTlogBuf.Lbuf);
//  Serial.printf("LogOT: Lbuf %d ibuf %d ifree %d\n", SmOT.OTlogBuf.GetLbuf(), SmOT.OTlogBuf.ibuf, SmOT.OTlogBuf.ifree);

	} else {
#endif //OT_DEBUGLOG
//if(dms < 500)
//    return;
    
  Serial.printf("%6d %3d ", ms, dms);

  switch(code)
  {
       case -3:
      Serial.println(F("Resp: NONE"));
        break;
       case -2:
      Serial.println(F("Resp: INVALID"));
        break;
       case -1:
      Serial.println(F("Resp: TimeOutn"));
        break;
       case 0:
      Serial.printf((PGM_P)F("Resp: ParityErr %d %d %04x\n"), id, messagetype, u88);
        break;
      case 1:
      {   
          switch(id)
          {  case OpenThermMessageID::TSet:
              t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
              if(messagetype == WRITE_ACK)
                   Serial.printf((PGM_P)F("Resp: TSet Write %.3f\n"), t);
              else if(messagetype == READ_ACK)
                   Serial.printf((PGM_P)F("Resp: TSet read %.3f\n"), t);
              else
                   Serial.printf((PGM_P)F("Resp: TSet ! %d %04x\n"),  messagetype, u88);
            break;

            default:
            Serial.printf((PGM_P)F("Resp: %d %d %04x\n"), id, messagetype, u88);
          }
      }
        break;
      case 2:
    Serial.printf((PGM_P)F("ReqS: %d READ_DATA %04x (Status %d %d %d %d)\n"), OpenThermMessageID::Status,  u88, SmOT.enable_CentralHeating, SmOT.enable_HotWater, SmOT.enable_Cooling,  SmOT.enable_CentralHeating2);
        break;
      case 3:
    Serial.printf((PGM_P)F("ReqS:  SConfigSMemberIDcode READ_DATA %04x\n"),  u88);
        break;
      case 4:
    Serial.printf((PGM_P)F("ReqS: MConfigMMemberIDcode WRITE_DATA %04x\n"),   u88);
        break;
      case 5:
    Serial.printf((PGM_P)F("Req : %d %d %04x\n"), id, messagetype,   u88);
        break;

  }
#if OT_DEBUGLOG  
}
#endif
 }   

#endif