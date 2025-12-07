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
extern void setup_read_config(void);
extern void check_fs(void);
extern void setup_web_common(void);
extern void loop_web(void);
extern void setup_tcpudp(SmartDevice *psd);

extern void loop_udp(int sts);
extern void loop_tcp(int sts);
extern void loop_servertcp(void);

#if OT_DEBUGLOG
void OTlog(unsigned int reqresp, int sts);
void OTlogDelLast(void);
#endif

void loop_time(void);
void loop_LED(void);
void OTloop_callback(void);

int ST_setCpuFrequencyMhz(int code);

#if MQTT_USE 
 #if RELAY_USE
  extern void MQTT_pub_relay(void);
 #endif
 extern int MQTT_pub_cmdCH(int on);
 extern void MQTT_pub_Eff_Mod_h(void);
 extern void mqtt_loop(void);
#endif


/************************************/
class SD_Termo SmOT;

char SmartDevice::LocalUrl[24] = "";

//Задаем пины
#if defined(ARDUINO_ARCH_ESP8266)

  const int inPin = D7;  // OpenTherm  in   D7 GIP013 Nodemsu
  const int outPin = D8;  // OpenTherm out  D8 GPIO15 Nodemsu

  const int DS1820_1 = D6; //
  const int DS1820_2 = D2; //

#elif defined(ARDUINO_ARCH_ESP32)
  const int inPin = 16;  // OpenTherm master in RX2 esp32
  const int outPin = 4;  // OpenTherm master out D4 esp32

#if ST_VERS == 2
  const int inPinSlave  = 19; // OpenTherm slave in
  const int outPinSlave = 18; // OpenTherm slave out
#endif

  const int DS1820_1 = 15; // D15 esp32  3 снизу
  const int DS1820_2 = 26; // D26 esp32  7 снизу
  const int RelayPin = 23;

#endif

/*  некоторым котлам (например, Baxi Fourtech/Luna 3) не нравится OpenThermMessageID::MConfigMMemberIDcode
    настолько, что они перестают отвечать на запросы
    OTstartSts_MAX  2 - не использовать MConfigMMemberIDcode
    OTstartSts_MAX  3 - использовать MConfigMMemberIDcode (if SmOT.UseID2 with code SmOT.ID2masterID)
*/
static int OTstartSts_MAX = 2;

OpenTherm ot(inPin, outPin);
#if ST_VERS == 2
  OpenTherm ot_slave(inPinSlave, outPinSlave, true); //Slave
  extern volatile int ot_SlaveSts;
  extern volatile unsigned long ot_SlaveResponse; 
  extern volatile unsigned long ot_SlaveRequest; 
  extern int OT_slaveloop(void);
  extern int setup_ot_slave(void);
  extern void sendResponse_ot_slave(void);
  void init_ot_slave(void);  
#endif
  
void OTprocessResponse(unsigned long response, OpenThermResponseStatus status);
int OTloop(void);
void loop2(void);
unsigned int buildRequest(int mode);
#if OT_DEBUG
void LogOT(int status, int code, byte id, int messagetype,  unsigned int u88);
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
extern unsigned int OTcount;


void IRAM_ATTR handleInterrupt() {
    ot.handleInterrupt();
}

void init_ot_slave(void)
{
#if 0
    Serial.printf("init_ot_slave\n");
    pinMode(inPinSlave,  INPUT_PULLUP ); //OUTPUT
    pinMode(outPinSlave, INPUT_PULLUP );
    digitalWrite(inPinSlave, 1);   
    digitalWrite(outPinSlave, 1);
    delay(3000);   
    digitalWrite(inPinSlave, 0);   
    digitalWrite(outPinSlave, 0);
#endif    

}

/*******************************/
static int OTstartSts = 0;
int LedSts = 0; //LOW
//RTC_DATA_ATTR
RTC_NOINIT_ATTR  unsigned short int bootCount, bootReason, bootSts, bootSts1, bootSts2;
unsigned short int _bootCount, _bootReason, _bootSts, _bootSts1, _bootSts2; // сохраняем состояние в момент старта


void Led_Info_reset(int code)
{ int i, j;
  for(j=0; j<2; j++)
  {
      digitalWrite(LED_BUILTIN, 1); 
      delay(1000);
      digitalWrite(LED_BUILTIN, 0); 
      delay(1000);
      for(i=0; i<code; i++)
      { digitalWrite(LED_BUILTIN, 1); 
        delay(300);
        digitalWrite(LED_BUILTIN, 0); 
        delay(200);
      }
      if(j < 1)
        delay(1000);
      Serial.printf("%d resetReason %d bootCount %d prevReason %d bootSts=%d bootSts1=%d bootSts2=%d\n",
           j, code,  bootCount, bootReason, bootSts, bootSts1, bootSts2);
  }
}

/*********** watchdog ******************/
#include <esp_task_wdt.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_wdt.h"

#define WDT_TIMEOUT 20 // Timeout in seconds
// Define WTC Watchdog Timer in milliseconds
#define RTC_WDT_TIME_MS (WDT_TIMEOUT *1100 + 1000)

void watchdog_setup(void)
{
//wdt  
  // Deinitialize the default watchdog (if enabled by default)
  esp_task_wdt_deinit();
 
  // Initialize the Task Watchdog
  esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT, true);
  if (err != ESP_OK) {
    Serial.printf("WDT Init failed: %s\n", esp_err_to_name(err));
    return;
  }

  // Add the current task (Arduino loop) to the watchdog watch list
  esp_task_wdt_add(NULL); 
  Serial.printf("Watchdog Timeout set to: %d sec\n", WDT_TIMEOUT);

//rtc_wdt
  rtc_wdt_protect_off(); // Disable RTC WDT write protection
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_RTC); // Set action on timeout
  rtc_wdt_set_time(RTC_WDT_STAGE0, RTC_WDT_TIME_MS ); // Set timeout to WDT_TIMEOUT seconds + 100 ьы
  rtc_wdt_enable(); // Start the RTC WDT timer
  rtc_wdt_protect_on(); // Enable RTC WDT write protection  
  Serial.printf("RTC Watchdog Timeout set to: %d msec\n", RTC_WDT_TIME_MS);
}

void onOTAstart(void)
{ //Serial.println("OTA started");
  esp_task_wdt_delete(NULL);
  esp_task_wdt_deinit();
  rtc_wdt_protect_off(); // Disable RTC WDT write protection
  rtc_wdt_disable(); // stop the RTC WDT timer
  rtc_wdt_protect_on(); // Enable RTC WDT write protection  
}

void exitOTAError(uint8_t err) {
//  Serial.printf("OTA error occurred %d\n", err);
   watchdog_setup();
}

/*****************************/

#include "esp32/rom/rtc.h"
//https://docs.espressif.com/projects/arduino-esp32/en/latest/api/reset_reason.html
/*
1:	Vbat power on reset
3:	Software reset digital core
4:	Legacy watch dog reset digital core
5:	Deep Sleep reset digital core
6:	Reset by SLC module, reset digital core
7:	Timer Group0 Watch dog reset digital core
8:	Timer Group1 Watch dog reset digital core
9:	RTC Watch dog Reset digital core
10:	Instrusion tested to reset CPU
11:	Time Group reset CPU
12:	Software reset CPU
13:	RTC Watch dog Reset CPU
14:	for APP CPU, reset by PRO CPU
15:	Reset when the vdd voltage is not stable
16:	RTC Watch dog reset digital core and rtc module
*/
/* 1, 14 |12,12 |*/
void check_reset(void)
{ int rr0, rr1;
  rr0 = rtc_get_reset_reason(0);
  rr1 = rtc_get_reset_reason(1);
  if(rr0 != 1 && rr0 != 12)
  { Led_Info_reset(rr0);
  }

  if(rr0 == 1)
  {  bootReason = bootCount = bootSts = bootSts1 = bootSts2 = 0;
  } else {
    Serial.printf("reset_reason %d %d bootCount %d sts %d %d\n", rr0, rr1, bootCount, bootSts, bootSts1);
  }
  // сохраняем состояние в момент старта, если не rr0 == 1
  _bootReason = bootReason;
  _bootCount = bootCount;
  _bootSts = bootSts;
  _bootSts1 = bootSts1;
  _bootSts2 = bootSts2;

  bootReason = rr0;

}

void set_rtc_flag(int sts)
{ bootSts1  = sts;
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, LedSts);   // Turn the LED on (Note that LOW is the voltage level
  
  delay(1);
  Serial.begin(115200);

  Serial.println(IDENTIFY_TEXT);
  Serial.printf((PGM_P)F("Vers %d.%d.%d.%d build %s\n"),SmOT.Vers, SmOT.SubVers,SmOT.SubVers1,SmOT.Revision, SmOT.BiosDate);

  check_reset();
   ++bootCount;

  LedSts=1;
  digitalWrite(LED_BUILTIN, LedSts);   

  setup_read_config();

  watchdog_setup();

  Serial.printf("SmOT.useCPU_freq = %d\n", SmOT.useCPU_freq);
  if(SmOT.useCPU_freq > 0)
  { int v = 80;
    if(SmOT.useCPU_freq == 1) v = 160;
    setCpuFrequencyMhz(v);

    Serial.printf("Set CPU Freq to %d\n", v);
  }

  SmOT.RelayInit();
/*******************************************/

  ot.begin(handleInterrupt, OTprocessResponse);
#if ST_VERS == 2
 #if OT_SLAVE_DEBUG
     setup_ot_slave();
 #else
    if(SmOT.OT_slave_present)
    { if(SmOT.OT_slave_mode == 0)
      {    Serial.printf("setup_slave 1\n");
           setup_ot_slave();
      } else {
        init_ot_slave();    
      }
    }
 #endif
#endif

  setupDS1820();

  setup_web_common();
  setup_tcpudp( &SmOT );

  if(SmOT.Immergas_fix_flag)
        ot.Immergas_fix = true;

  if(SmOT.UseID2 || SmOT.Immergas_fix_flag)
      OTstartSts_MAX = 3;
  else 
      OTstartSts_MAX = 2;

#if SERVER_DEBUG
#if 0
  SmOT.TCPserver_sts = 2;  /* статус сервера */
//  SmOT.TCPserver_sts2 = 1; 
  SmOT.TCPserver_t = millis();
  SmOT.TCPserver_port = 8876;  
  SmOT.TCPserver_report_period = 10000;
//  SmOT.tcp_remoteIP.fromString("192.168.10.112");
  SmOT.tcp_remoteIP.fromString("80.237.33.121");

  Serial.printf("TCPserver_report_period=%d TCPserver_port=%d\n", SmOT.TCPserver_report_period, SmOT.TCPserver_port);
#endif
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
          if (t == DEVICE_CRC_ERROR || t == DEVICE_DISCONNECTED)
          { SmOT.stsT1 = 2;
            SmOT.status |= 0x10;
#if SERIAL_DEBUG 
            Serial.println(F("ERROR: DS1 CRC error"));
#endif            
          } else {
            SmOT.status &= ~0x10; // сброс бита CRC error
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
          if (t == DEVICE_CRC_ERROR || t == DEVICE_DISCONNECTED)
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

void OTprocessResponse(unsigned long response, OpenThermResponseStatus status)
{   float t;
    uint16_t u88;
    byte id;
    int parity, messagetype;
static int timeOutcounter = 0;

     OTcount++;
     SmOT.RespMillis = millis();
    if(SmOT.TestCmd == 2)
    {
      id = (response >> 16 & 0xFF);
      if(id == (SmOT.TestId & 0xff))
      {
#if OT_DEBUG
    Serial.printf("TestCmd processResponse %x %x\n", response,  status);
#endif
        SmOT.TestResponse = response;
        SmOT.TestStatus = status;
        SmOT.TestCmd = 0;
        return;
      }
    }
  
    if (status == OpenThermResponseStatus::SUCCESS) {
		   if(SmOT.stsOT != 0)
        {   SmOT.MQTT_need_report = 1;
            SmOT.OnOpenThermRestore();
            buildRequest(1);
        }

        SmOT.stsOT = timeOutcounter = 0;
        SmOT.response = response; 
        OTDebugInfo[0]++;
    } else if (status == OpenThermResponseStatus::NONE) {
#if OT_DEBUG
      LogOT(status, 0, 0,  0,  0);
#endif         
      // SmOT.stsOT = -1;  // ??
        OTDebugInfo[2]++;
    } else if (status == OpenThermResponseStatus::INVALID) {
       //SmOT.stsOT = 1;
        OTDebugInfo[3]++;
    } else if (status == OpenThermResponseStatus::TIMEOUT) {
      if(SmOT.stsOT != -1)
	    { if(timeOutcounter > 10)
		    { if(SmOT.stsOT != 2)
               SmOT.MQTT_need_report = 1;
          SmOT.stsOT = 2;
		    } else {
			    timeOutcounter++;
		    }	
#if OT_DEBUGLOG
        if(SmOT.stsOT == 2)
          OTlogDelLast();
#endif

      } else {
#if OT_DEBUGLOG
        OTlogDelLast();
#endif
      }
      OTDebugInfo[4]++;
#if OT_DEBUG
      LogOT(status, 0, 0,  0,  0);
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
      LogOT(-1, 0, id,  messagetype,  u88);
    else 
      LogOT(status, 0,  id,  messagetype,  u88);
  } 
#endif         

#if OT_DEBUGLOG
 #if ST_VERS == 2
    if(SmOT.OT_slave_present == 0 || SmOT.OT_slave_mode == 0)
      OTlog(response,1);
    else if(SmOT.ot_slave_stsOT == 0)
      OTlog(response,1);
    #else
    OTlog(response,1);
 #endif
#endif

#if ST_VERS == 2
    if(SmOT.OT_slave_present && (SmOT.OT_slave_mode == 1) && (SmOT.ot_slave_stsOT == 0))
    { if(ot_SlaveSts == 2)
      { static int slraz =0;
         ot_SlaveSts = 3;
         ot_SlaveResponse = response;
     
        slraz++;
//Serial.printf("buildRequestIfNeed raz %d\n", raz);
if(SmOT.CapabilitiesDetected == 0)
        {  if(slraz++ > 30)
           {  SmOT.CapabilitiesDetected = 1;
              SmOT.DetectCapabilities();
           } 
        } else if(SmOT.CapabilitiesDetected == 1) {
            if(slraz == 150)
            { SmOT.CapabilitiesDetected = 2;
              SmOT.DetectCapabilities();
              slraz++;   
            } else {
              slraz++;   
            }
        }
      }
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
        Serial.printf("Messagetype  %d!!! Status %d %d ot.LastRequestId %d\n", messagetype, status, SmOT.stsOT, ot.LastRequestId);
#endif        
      return;
    }
    
    if(SmOT.stsOT != 0)
        return;
    SmOT.t_lastwork = time(nullptr);

    id = (response >> 16 & 0xFF);
    
    if(id != ot.LastRequestId)
    { OTDebugInfo[10]++;
        Serial.printf("Resp id %d != Req id %d\n", id, ot.LastRequestId );
#if SERIAL_DEBUG 
        Serial.printf("Resp id %d != Req id %d\n", id, ot.LastRequestId );
#endif        
      return;
    }

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
2: Cooling enable [ Cooling is disabled, Cooling is enabled]
3: OTC active [OTC not active, OTC is active]
4: CH2 enable [CH2 is disabled, CH2 is enabled]
5: Summer/winter mode [winter mode active, summer mode active]
6: DHW blocking  [DHW unblocked, DHW blocked] 
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
7: Electricity production [off, on] 
*/   
//        boiler_status = response & 0xFF;
        if((u88 & 0x08) != (SmOT.BoilerStatus & 0x08))
        {   SmOT.Bstat.calcNflame(u88 & 0x08);
        }
        if(SmOT.HotWater_present)
        { if((u88 & 0x04) != (SmOT.BoilerStatus & 0x04))
          {   SmOT.Bstat.calcN_HW(u88 & 0x04);
          }
        }

#if MQTT_USE
        if(SmOT.HotWater_present)
        {   if((u88 & (0x08|0x04|0x02) ) != (SmOT.BoilerStatus & (0x08|0x04|0x02)))
                SmOT.MQTT_need_report = 1;
        } else {
            if((u88 & (0x08|0x02) ) != (SmOT.BoilerStatus & (0x08|0x02)))
                SmOT.MQTT_need_report = 1;
        }

#endif

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
//       Serial.printf("SmOT.Tset %g Tset_r %g\n", SmOT.Tset, SmOT.Tset_r);

        break;
        
    case OpenThermMessageID::MConfigMMemberIDcode: //2
         if(OTstartSts == 2)  OTstartSts++;
#if SERIAL_DEBUG 
       Serial.printf((PGM_P)F("OpenThermMessageID::MConfigMMemberIDcode, %d\n"), OTstartSts);
#endif
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
            if(_SConfigSMemberIDcode & 0x800) //DHW configuration: storage tank
            {   SmOT.DHW_tank_present  = true;
            } else {
                SmOT.DHW_tank_present  = false;
            }

            SmOT.OTmemberCode = _SConfigSMemberIDcode & 0xff;
//        Serial.printf("SmOT.OTmemberCode %d\n", SmOT.OTmemberCode);
         }
//        Serial.printf("OTstartSts %d: u88 %x SmOT.HotWater_present = %d\n", OTstartSts, u88, SmOT.HotWater_present );
        break;

    case OpenThermMessageID::RemoteRequest: // 4 Remote Request
//    Serial_db.printf("RemoteRequest responce %d\n", u88 );
        break;

    case OpenThermMessageID::ASFflags: //5
/*  HB: Application-specific fault flags 
bit: description [ clear/0, set/1]
0: Servicerequest [servicenotreq’d,servicerequired]
1: Lockout-reset [remoteresetdisabled,rrenabled]
2: Lowwaterpress[noWPfault,waterpressurefault]
3: Gas/flamefault [noG/Ffault,gas/flamefault]
4: Air press fault [noAPfault,airpressurefault]
5: Waterover-temp[noOvTfault,over-temperat.Fault]
6: reserved
7: reserved

LB: OEM fault code
An OEM-specific fault/error code
*/
        if(u88)
          OTDebugInfo[5]++;
        SmOT.Fault = u88;
        break;        

    case OpenThermMessageID::MaxRelModLevelSetting: //14 Maximum relative modulation level setting (%) W
//        SmOT.MaxRelModLevelSetting = t;
        break;

    case OpenThermMessageID::MaxCapacityMinModLevel:	//15 MaxCapacityMinModLevel, // u8 / u8  Maximum boiler capacity (kW) / Minimum boiler modulation level(%) R
        SmOT.MinModLevel =  (u88 & 0xff);
        SmOT.MaxCapacity =  ((u88>>8) & 0xff);
        break;

    case OpenThermMessageID::TrSet: // 16  Room Setpoint (°C) TrSet:  
        break;

    case OpenThermMessageID::Tr: // 24 f8.8  Room temperature (°C)
        break;

    case OpenThermMessageID::Tboiler:  //25
        SmOT.BoilerT = t;
/************************************/
        if((fabs(SmOT.BoilerT-SmOT.Tset) > 3.) && (SmOT.need_set_T == 0))
        { SmOT.need_set_T = 1;
  //        Serial.printf("SmOT.BoilerT %g SmOT.Tset %g\n", SmOT.BoilerT, SmOT.Tset);
        }
/************************************/
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

    case OpenThermMessageID::Tstorage: //29
        SmOT.Tstorage = t;
        break;

    case OpenThermMessageID::TflowCH2: //31
        SmOT.BoilerT2 = t;
        break;

    case OpenThermMessageID::Texhaust: //33
      { float tmp;
        tmp = (float)u88;
        if(tmp < -40. || tmp > 500)
          tmp = t;
        SmOT.Texhaust = tmp;
      }
        break;

    case OpenThermMessageID::RelModLevel: //17 Relative Modulation Level 
        SmOT.FlameModulation = t;
        SmOT.Bstat.calcIntegral(t);
        break;

    case OpenThermMessageID::CHPressure: //18 Water pressure in CH circuit
        SmOT.Pressure = t;
        break;

    case OpenThermMessageID::OEMDiagnosticCode: //115
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
    {   request = ot.buildRequest(OpenThermMessageType::WRITE_DATA, ( OpenThermMessageID) (SmOT.TestId & 0x0ff), SmOT.TestPar);
    }  else {
        unsigned int data = SmOT.TestPar;
    	  if(ot.Immergas_fix &&SmOT.TestId == 0) data |= 0xca;
        request = ot.buildRequest(OpenThermMessageType::READ_DATA, ( OpenThermMessageID) SmOT.TestId, data);
    }
    
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
        SmOT.BoilerStatusRequest = request;
#if OT_DEBUG
  { unsigned int u88;
    u88 = (request & 0xffff);
    LogOT(0, 2,  OpenThermMessageID::Status,  OpenThermMessageType::READ_DATA,  u88);
//  Serial.printf("ReqS: %d READ_DATA %04x (Status %d %d %d %d)\n", OpenThermMessageID::Status,  u88, SmOT.enable_CentralHeating, SmOT.enable_HotWater, SmOT.enable_Cooling,  SmOT.enable_CentralHeating2);
  } 
#endif         
      break;
      
      case 1: // запрос SConfigSMemberIDcode
          request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::SConfigSMemberIDcode, 0); //3
#if OT_DEBUG
  { unsigned int u88;
    u88 = (request & 0xffff);
    LogOT(0, 2,  OpenThermMessageID::SConfigSMemberIDcode,  OpenThermMessageType::READ_DATA,  u88);
//    Serial.printf("ReqS: %d READ_DATA %04x (SConfigSMemberIDcode)\n", OpenThermMessageID::SConfigSMemberIDcode,  u88);
  } 
#endif         
      break;

      case 2: // OpenThermMessageID::MConfigMMemberIDcode:
          if(SmOT.Immergas_fix_flag && !SmOT.UseID2)
            request = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MConfigMMemberIDcode, _SConfigSMemberIDcode); //3
          else 
            request = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MConfigMMemberIDcode, SmOT.ID2masterID /* (_SConfigSMemberIDcode&0xff) */); //3
#if OT_DEBUG
  { unsigned int u88;
    u88 = (request & 0xffff);
    LogOT(0, 2,  OpenThermMessageID::MConfigMMemberIDcode,  OpenThermMessageType::WRITE_DATA,  u88);
  } 
#endif         
        break;
    }

    return request;
}


// rc = 0 - nothing to do
// rc = 1 - build request, need repeat
// rc = 2 - build request, not need repeat
int buildRequestIfNeed(unsigned int &request)
{   int rc = 0, need,flag, i,j,j0, s;
    static int raz=0, rraz=0, sts = 0, idrep=0;
    const int Nneed = 5;

/***************************************************/
  need = flag = 0;
#if  PID_USE
    if(SmOT.enable_CentralHeating_real && SmOT.need_set_T)
#else 
    if(SmOT.enable_CentralHeating  && SmOT.need_set_T)
#endif
                  { need++, flag |= 0x01; s = 1; }

  if(SmOT.enable_HotWater && SmOT.need_set_dhwT)
                  { need++, flag |= 0x02; s = 2; } 
  if(SmOT.enable_CentralHeating2 && SmOT.need_set_T2)
                  { need++, flag |= 0x04; s = 3; }
  if(ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting) && SmOT.need_set_MaxRelModLevel) 
                  { need++, flag |= 0x08; s = 4; }
  if(ot.OTid_used(OpenThermMessageID::RemoteRequest) && SmOT.need_set_RemoteRequest) 
                  { need++, flag |= 0x10; s = 5; } //s = Nneed
  if(ot.OTid_used(OpenThermMessageID::MaxTSet) && SmOT.need_set_MaxTSet) 
                  { need++, flag |= 0x20; s = 6; } //s = Nneed
                  
  if(need == 1)
  { sts = s;
    if(rraz == 0)
    { rraz = 1;
    } else {
      rraz = 0;
      sts = 0;
    }
  } else if (need > 1) {
    j0 = sts -1;
    if(sts == 0) j0 = 0;
    for(i=0; i < Nneed; i++)
    {	j = (j0 + i)%Nneed;
      if(flag & (1<<j) && ((j+1) != sts))
      { sts = j+1;
        break;
      }
      if(rraz < need)
      { rraz++;
      } else {
        rraz = 0;
        sts = 0;
      }
    }   

  } else {
    sts = 0;
  }

  switch(sts)
  { 
    case 0:
        raz++;
//Serial.printf("buildRequestIfNeed raz %d\n", raz);
        if(SmOT.CapabilitiesDetected == 0)
        {  if(raz > 2)
           {   SmOT.CapabilitiesDetected = 1;
               SmOT.DetectCapabilities();
           }
        } else if(SmOT.CapabilitiesDetected == 1) {
            if(raz > 16)
            {   SmOT.CapabilitiesDetected = 2;
              SmOT.DetectCapabilities();
            }
        }
        if(raz > 100)
        {  SmOT.OnOpenThermRestore();
           raz = 0;            
        }
        need = 0;
      break;

    case 1:
        if(SmOT.need_set_T > 0)
        { request = ot.buildSetBoilerTemperatureRequest(SmOT.Tset); //1
          SmOT.need_set_T--;
        }
      break;
    case 2:
        if(SmOT.need_set_dhwT > 0) {
 //Serial.printf("1a Request: %d\n",OpenThermMessageID::TdhwSet);
#if DEBUG_WITH_EMULATOR  //translate to emulator tempoutdoor as TdhwSet
              request = ot.buildSetDHWSetpointTemperatureRequest(SmOT.tempoutdoor); //56
#else              
              request = ot.buildSetDHWSetpointTemperatureRequest(SmOT.TdhwSet); //56
#endif              
              SmOT.need_set_dhwT--;
        }
      break;

    case 3:
        if(SmOT.need_set_T2 > 0) {
              request = ot.buildSetBoilerCH2TemperatureRequest(SmOT.Tset2); //8
               SmOT.need_set_T2--;
        }
      break;

    case 4:
        if(SmOT.need_set_MaxRelModLevel > 0) //14
        { 	unsigned int data = ot.temperatureToData(SmOT.MaxRelModLevelSetting);
	          request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MaxRelModLevelSetting, data);
            SmOT.need_set_MaxRelModLevel--;
        }
      break;

    case 5:
        { 	unsigned int data = 0; //4
        if(SmOT.need_send_Blor)
        {   data = (0x01<<8);  //BLOR        
            SmOT.need_send_Blor = 0;

        }
//Serial.printf("RemoteRequest data %x\n", data);
        
	          request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::RemoteRequest, data);
            SmOT.need_set_RemoteRequest--;
        }
      break;

    case 6: //MaxTSet 
        if(SmOT.MaxTSet >= MIN_CH_TEMP && SmOT.MaxTSet <= MAX_CH_TEMP)
        { 	unsigned int data = ot.temperatureToData(SmOT.MaxTSet);
            request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MaxTSet, data);
            SmOT.need_set_MaxTSet--;
        }

      break;
  }

/********************************/
  if(need > 1)
  { idrep++;
    if(idrep >= need)
    {	idrep = 0;
      rc = 2;
    } else {
      rc = 1;
    }
  } else if(need == 1) {
	  rc = 2;
  } else {
	  rc = 0;
  }

  return rc;
}

unsigned int buildRequest(int mode)
{   static int st = 0, raz = 0;
    unsigned int request = 0;
    int rc;
    if(mode == 1)
    { st = 0;
      return 0;
    }

    if(SmOT.TestCmd == 1)
    {   request = buildTestRequest();  
        SmOT.TestCmd++;
        return request;
    }

M0:    
 //  Serial.printf("st %d\n", st);
    switch(st)
    {
      case 0: // запрос статуса
// Serial.printf("0 Request: %d\n",OpenThermMessageID::Status);
#if PID_USE
      if(!SmOT.usePID)
         SmOT.enable_CentralHeating_real = SmOT.enable_CentralHeating;
      if(SmOT.CH2_DHW_flag && SmOT.enable_HotWater)
      {  request = ot.buildSetBoilerStatusRequest(SmOT.enable_CentralHeating_real, SmOT.enable_HotWater, SmOT.enable_Cooling, SmOT.Use_OTC, 1, SmOT.UseWinterMode);
      } else {
        request = ot.buildSetBoilerStatusRequest(SmOT.enable_CentralHeating_real, SmOT.enable_HotWater, SmOT.enable_Cooling, SmOT.Use_OTC, SmOT.enable_CentralHeating2, SmOT.UseWinterMode);
      }   
{  static int old_CH = -1;
    if(old_CH != SmOT.enable_CentralHeating_real)
    {
      if(ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting) && SmOT.enable_CentralHeating_real)
         SmOT.need_set_MaxRelModLevel = 2; 

// Serial.printf("SmOT.enable_CentralHeating_real  %d %d\n",SmOT.enable_CentralHeating_real, old_CH );
#if MQTT_USE 
       MQTT_pub_cmdCH(SmOT.enable_CentralHeating_real);
       
#endif      
         old_CH = SmOT.enable_CentralHeating_real;
      }

}
#else
      if(SmOT.CH2_DHW_flag && SmOT.enable_HotWater)
      {  request = ot.buildSetBoilerStatusRequest(SmOT.enable_CentralHeating, SmOT.enable_HotWater, SmOT.enable_Cooling, SmOT.Use_OTC, 1, SmOT.UseWinterMode);
      } else {
        request = ot.buildSetBoilerStatusRequest(SmOT.enable_CentralHeating, SmOT.enable_HotWater, SmOT.enable_Cooling,  SmOT.Use_OTC, SmOT.enable_CentralHeating2, SmOT.UseWinterMode);
      }   
#endif
      SmOT.BoilerStatusRequest = request;
        st++;
      break;

      case 1: 
        st++;
        rc = buildRequestIfNeed(request);
        if(rc > 0)
        { if(rc == 1) //при старте запрашиваем всё, но не более 4 id за раз
          { int count, countok;
            static int raz0 = 0;
            ot.Get_OTid_count(OpenThermMessageID::Status, count, countok);
            if(count < 4)
            { raz0++;
              if(raz0 > 4) raz0 = 0;
              else         st--;
            }
          }
          break;
        }
        raz++;

        if(SmOT.CapabilitiesDetected == 0)
        {  if(raz > 2)
             SmOT.CapabilitiesDetected = 1;
        } else if(SmOT.CapabilitiesDetected == 1) {
           if(raz > 16)
           {   SmOT.CapabilitiesDetected = 2;
              SmOT.DetectCapabilities();
           }
        }

        if(raz > 100)
        {   if(SmOT.enable_CentralHeating)
                SmOT.need_set_T  = 10; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
            if(SmOT.enable_HotWater) 
                SmOT.need_set_dhwT = 1;                   
            if(SmOT.enable_CentralHeating2)
                SmOT.need_set_T2  = 10; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
            if(SmOT.Use_MaxRelModLevel)
              SmOT.need_set_MaxRelModLevel = 1; 
            SmOT.need_set_MaxTSet = 1;

            raz = 0;
        }

#if 0
            raz++;

            if(SmOT.CapabilitiesDetected == 0)
            {  if(raz > 2)
                 SmOT.CapabilitiesDetected = 1;
            } else if(SmOT.CapabilitiesDetected == 1) {
               if(raz > 16)
               {   SmOT.CapabilitiesDetected = 2;
                  SmOT.DetectCapabilities();
               }
            }
            if(raz > 100)
            {   if(SmOT.enable_CentralHeating)
                    SmOT.need_set_T  = 10; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
                if(SmOT.enable_HotWater) 
                    SmOT.need_set_dhwT = 1;                   
                if(SmOT.enable_CentralHeating2)
                    SmOT.need_set_T2  = 10; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
                raz = 0;
            }
          }
#endif //0

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
        if(SmOT.HotWater_present && ot.OTid_used(OpenThermMessageID::Tdhw) )
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
      break;

      case 7: //TSetCH2
         st++;
        if(SmOT.enable_CentralHeating2 && ot.OTid_used(OpenThermMessageID::TflowCH2))       
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
        {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Texhaust, 0); //33
        }  else {
            goto M0;
        }
      break;

      case 10:
        st++; 
        if(SmOT.Use_ID29_DHW_flag)
        {
          if(ot.OTid_used(OpenThermMessageID::Tstorage))
          {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tstorage, 0); //29
          }  else {
              goto M0;
          }
          break;
        }

    case 11:
  /* OTC = Outside Temperature Compensation). ID0:HB3*/
  /* or Zota */
        st++; 
        if(SmOT.Use_OTC || SmOT.OTmemberCode == 248) 
        {
//        Serial.printf("st %d SmOT.OTmemberCode %d\n", st,  SmOT.OTmemberCode);
          if(ot.OTid_used(OpenThermMessageID::TrSet)) // 16  Room Setpoint (°C)
          { 
//          unsigned int data = ot.temperatureToData(SmOT.TroomTarget);
            //unsigned int data = ot.temperatureToData(22.f);
            unsigned int data = ot.temperatureToData(24.f);
            request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TrSet, data);
          }  else {
              goto M0;
          }
          break;
        }

    case 12:
        st++;
        if(SmOT.Use_OTC || SmOT.OTmemberCode == 248) /* OTC or Zota **/
        {
          if(ot.OTid_used(OpenThermMessageID::Tr)) //  24 Room temperature (°C)
          { //unsigned int data = ot.temperatureToData(SmOT.tempindoor);
//          unsigned int data = ot.temperatureToData(24.f);
            unsigned int data = ot.temperatureToData(22.f);
	          request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::Tr, data);
          }  else {
              goto M0;
          }
          break;
        }

      case 13: //getFault flags
 //Serial.printf("13 Request: %d\n",OpenThermMessageID::ASFflags);
        st++;
          if(ot.OTid_used(OpenThermMessageID::ASFflags)) 
          {
            request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::ASFflags, 0);
/*
0: fault indication [ no fault, fault ] 0x01
6: diagnostic/service indication [no diagnostics, diagnostic event] 0x40
*/
            if(!(SmOT.BoilerStatus & 0x41 || SmOT.Fault) )
                st = 0;
            break;
          }

      case 14: //getFault code
        st = 0;
          if(ot.OTid_used(OpenThermMessageID::OEMDiagnosticCode)) 
          {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::OEMDiagnosticCode, 0);
          }  else {
              goto M0;
          }
      break;

    }
#if OT_DEBUG
  { unsigned int u88, id;
    int  messagetype;
    u88 = (request & 0xffff);
    id = (request >> 16 & 0xFF);
    messagetype = ot.getMessageType(request);
//    Serial.printf("Req : %d %d %04x\n", id, messagetype,   u88);
    LogOT(0, 1,  id,  messagetype,  u88);

  } 
#endif         

    return request;
}


/* return 0 if no response, 1 if have responce */
int OTloop(void)
{   static int st = 1;
    int rc = 0;

#if ST_VERS == 2
  if(SmOT.OT_slave_present && SmOT.OT_slave_mode == 1)
  { if(SmOT.ot_slave_stsOT == -2)
    { if(SmOT.stsOT == 0)
      {   Serial.printf("setup_slave 2\n");
              setup_ot_slave();
      }
    }
    else
      OT_slaveloop();
  }

#endif

    switch(st)
    {
      case 0:
      if (ot.isReady()) 
      {  unsigned int request=0;
#if SERIAL_DEBUG 
          if((millis() - SmOT.RespMillis) < 100)
               Serial.printf((PGM_P)F("OTloop too fast: %d **********\n"), int (millis() - SmOT.RespMillis));
#endif

#if ST_VERS == 2

#if  OT_SLAVE_DEBUG
if (ot.isReady()) 
{
  static unsigned long int t0=0;
  unsigned long int t;
  t = millis();
  if(t-t0>500)
  {
    request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Status, 0xaaaa);

  t0 = t;
  }

}
#else

    if(SmOT.OT_slave_present && (SmOT.OT_slave_mode == 1)) 
    { if(SmOT.ot_slave_stsOT == 0)
      { if(ot_SlaveSts == 1)
        { int ids; 
          request = ot_SlaveRequest;
          ids = (request & 0xff0000) >>16;
          if(ids ==0)
              SmOT.BoilerStatusRequest = request;
          ot.LastRequestId = ids;
          ot_SlaveSts = 2;
          OTlog(request,2);
        } else {
          break; // do nothing - no reqest from slave
        }
      } else {
        extern unsigned long ot_SlaveRequest_ms;

         if(millis() - ot_SlaveRequest_ms > 2000) //wait 3 sek than work as master
            goto M00;
  
      }
    } else {
M00:
        if(OTstartSts < OTstartSts_MAX)
		request = buildRequestOnStart();
	else
        	request = buildRequest(0);
#if OT_DEBUGLOG
        if(SmOT.OT_slave_mode == 0)
            OTlog(request,0);
#endif          
      }
#endif

#else //ST_VERS == 2
         if(OTstartSts < OTstartSts_MAX)
            request = buildRequestOnStart();
         else
            request = buildRequest(0);
  #if OT_DEBUGLOG
            OTlog(request,0);
  #endif          
#endif

/*     
          unsigned int id;
          id = (request >> 16 & 0xFF);
           Serial.printf("Request:  %d\n",  id);
 */           
         if(ot.sendRequestAync(request))    // 	status = OpenThermStatus::RESPONSE_WAITING;    
         {    st++;
         }

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
static unsigned long OTloopUpdate_t0 =0; 

//loop_callback for MQTT and web portal
void OTloop_callback(void)
{   if( OTloop() ) 
       OTloopUpdate_t0 = millis();
//  Serial.printf("OTloop_callback %ld\n", millis());
    loop_LED();
    loop_time();
} 

void loop(void)
{   unsigned long t;
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


   t = millis();
   dt = t - OTloopUpdate_t0;

#if ST_VERS == 2
  { int dtm = OT_CICLE_TIME;

    if(SmOT.OT_slave_present && (SmOT.OT_slave_mode == 1) && (SmOT.ot_slave_stsOT == 0))
        dtm -= 80;

    if(dt < dtm)
    {  loop2();
    } else  if( OTloop() ) {
        OTloopUpdate_t0 = millis();
    }  else {
        loop2();
    }      
  }
#else

  if(dt < OT_CICLE_TIME)
  {  loop2();
  } else  if( OTloop() ) {
//   Serial.printf("raz=%d\n", raz);
       OTloopUpdate_t0 = millis();
  }  else {
     loop2();
  }
#endif //st_VERS  
}

int minRamFree=-1;

/* web, udp, DS1820 */
void loop2(void)
{   static int irot = 0;
#if 0

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
#endif //0

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
  if( minRamFree - free  > 10000)
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
        if(SmOT.Use_remoteTCPserver && SmOT.TCPserver_sts > 0)
        {    loop_servertcp();
        }
        
        loop_tcp(0);

          irot++;
        break;

        case 4:
        loopDS1820();
          irot++;
        break;

        case 5:
        loop_time();
          irot = 6;
        break;

        case 6:
#if MQTT_USE
    if( WiFi.status()  ==  WL_CONNECTED && (SmOT.useMQTT== 0x03))
         mqtt_loop();
#endif
        irot = 7;
          break;

        case 7:
          loop_LED();
          irot = 0;
          break;

    }
}

void loop_LED(void)
{
  static unsigned long  t0=0; 
  unsigned long t, dt;
  unsigned int wt;
  t = millis();
  dt = t - t0;
#if defined(ARDUINO_ARCH_ESP8266)
  if(!LedSts) //быстро моргаем раз в мсек
#elif defined(ARDUINO_ARCH_ESP32)
  if(LedSts) //быстро моргаем раз в мсек
#endif
  {  wt = 10;
    if(SmOT.stsOT > 0) wt = 1000;
    if(dt >= wt)
    { LedSts = (LedSts+1)&0x01;
      digitalWrite(LED_BUILTIN, LedSts);   
      //t0 = t;
      t0 += wt;
      return;
    }
  } else {
    wt = 500;
    if(SmOT.stsOT == 0) wt = 2000;
    else if(SmOT.stsOT > 0) wt = 1000;
    if(dt >= wt)
    { LedSts = (LedSts+1)&0x01;
      digitalWrite(LED_BUILTIN, LedSts);   
      t0 += wt;
//        t0 = t;        
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

//watchdogs reset      
  esp_task_wdt_reset();
  rtc_wdt_feed();         

  ST_setCpuFrequencyMhz(SmOT.useCPU_freq);

  nowtime = localtime(&prev);
  year_prev = nowtime ->tm_year;
  nowtime = localtime(&now);
  year = nowtime->tm_year;

  if(prev == 0)  check_fs();

  prev = now;

  if( year_prev == 70 && year  >= 123)  //change time with nttp server
  {    noInterrupts();
        SmOT.Bstat.NflameOn_h = 0;
        SmOT.Bstat.ModIntegral_h = 0.;
        SmOT.Bstat.ModIntegral_d = 0.;
        SmOT.Bstat.sec_h = SmOT.Bstat.sec_d = 0;
        SmOT.Bstat.t_flame_on = SmOT.Bstat.t_flame_off = now;      

        SmOT.init(3);        
	      interrupts();
  }
  
#if PID_USE
    if(SmOT.enable_CentralHeating)
        SmOT.loop_PID();
#endif

#if ST_VERS == 2
 #if OT2_SLAVE_DEBUG 
  { //extern int nslaveint;
extern otst otst_d[128];
extern int Notst;
extern int Flag_otst; 
    int i;

//    Serial.printf("v = %d %d %x %d\n", nslaveint, ot_slave.status, ot_slave.getLastResponse(), digitalRead(inPinSlave));

    if(Flag_otst)
    {
      if(Notst > 0)
      {
        Serial.printf("OT data\n");
        for(i=0; i<Notst; i++)
        {
            if(i > 0)
              Serial.printf("%d %d %ld %x %d %x\n", i,  otst_d[i].state, otst_d[i].t - otst_d[i-1].t,  otst_d[i].status, otst_d[i].ind, otst_d[i].resp);
            else 
              Serial.printf("%d %d 0 %x %d %x\n", i,  otst_d[i].state,   otst_d[i].status, otst_d[i].ind, otst_d[i].resp) ;
				        }
      }
      Notst = 0;
      Flag_otst = 0;
    }
/*
    if(nslaveint > 0 && ot_slave.status == 1)
    {
      for(i=0; i<Notst; i++)
      {
          if(i > 0)
            Serial.printf("%d %d %ld %x\n", i,  otst_d[i].state, otst_d[i].t - otst_d[i-1].t,  otst_d[i].status);
          else 
            Serial.printf("%d %d 0 %x\n", i,  otst_d[i].state,   otst_d[i].status);
      }
      Notst = 0;
    }
*/      
  }
 #endif 
#endif 

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
//  SmOT.Bstat.Eff_Mod_h_prev = SmOT.Bstat.Eff_Mod_h; // ?? 
    SmOT.Bstat.Eff_Mod_h_prev = SmOT.Bstat.ModIntegral_h/3600.f;
    noInterrupts();
        SmOT.Bstat.NflameOn_h = 0;
        SmOT.Bstat.ModIntegral_h = 0.;
        SmOT.Bstat.sec_h = 0;
	   interrupts();
#if MQTT_USE 
     MQTT_pub_Eff_Mod_h();
#endif
    if(mday_prev != nowtime->tm_mday)
    { SmOT.Bstat.NflameOn_day_prev = SmOT.Bstat.NflameOn_day;
      SmOT.Bstat.Eff_Mod_d_prev = SmOT.Bstat.Eff_Mod_d;
    	noInterrupts();
        SmOT.Bstat.NflameOn_h = 0;
        SmOT.Bstat.ModIntegral_d = 0.;
        SmOT.Bstat.sec_d = 0;
        SmOT.Bstat.NflameOn_day = 0;
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


void SD_Termo::RelayInit(void)
{
#if RELAY_USE
  if(Relay_present)
  {

    pinMode(RelayPin, OUTPUT);  
    RelayOnOff(Relay_init_sts); 
  }
#endif  
}

void SD_Termo::RelayOnOff(bool onoff)
{
#if RELAY_USE
  if(!Relay_present)
      return;
   if(onoff)
   {  Relay_sts = true;
      digitalWrite(RelayPin, 1);  
   } else {
      Relay_sts = false;
      digitalWrite(RelayPin, 0);  
   }
#if MQTT_USE 
   MQTT_pub_relay();
#endif   

#endif  
}

int ST_setCpuFrequencyMhz(int code)
{   int i, frset = 240;
    int cpuf = getCpuFrequencyMhz();
    if(code == 1)
      frset = 160;
    else if(code == 2)
      frset = 160;
     
    if(cpuf != frset)
    {  bootSts2++;  _bootSts2++;  

      for(i=0;i<10;i++)
      { setCpuFrequencyMhz(frset);
        delay(10+i*2);
        cpuf = getCpuFrequencyMhz();
        if(cpuf !=  frset)
        {   Serial.printf("%d CPQ FREQ set %d, get %d", i,frset, cpuf );
            delay(10+i*2);
        } else break;
      }
    }
    return 0;
}

#if OT_DEBUGLOG
//пишем в кольцевой буфер не более 1024 пакетов
//sts: 0 - request, 1 - response, 
//2 - request from slave interface
//3 - response from slave interface in case of invalid request
void OTlog(unsigned int reqresp, int sts)
{ unsigned int b[2];
  unsigned long t = millis();
  int lb;
  if(!SmOT.Use_remoteTCPserver)
    return;
    
  lb = SmOT.OTlogBuf.Lbuf/SmOT.OTlogBuf.Litem - SmOT.OTlogBuf.GetLbuf(); //

  if(SmOT.nOTlog < 1024 && lb > 1)
  { b[0] = ( (((sts<<6)|(SmOT.nOTlog & 0x3f)) << 24) | (t & 0xffffff));
    
//    Serial.printf("SmOT.nOTlog %d Lbuf= %d sts %d %8x\n", 
//        SmOT.nOTlog, SmOT.OTlogBuf.GetLbuf(), sts, b[0]);

    b[1] =  reqresp;

    SmOT.OTlogBuf.Add( b);
    SmOT.nOTlog++;
    
  } else if(lb <= 1) { //нет места в буфере
    SmOT.nOTlog  = 1024;
  }
}

/*  если лог небольшой (<16), то удаляем последнюю запись в логе
 */
void OTlogDelLast(void)
{ int tmp[2];
  if(SmOT.nOTlog > 0 && SmOT.nOTlog < 16)
  { SmOT.OTlogBuf.Get(&tmp);
    SmOT.nOTlog--;
  }
}

#endif  
	
    
#if OT_DEBUG
//code = 0  responce
//code = 1  request
//code = 2  request at start
void LogOT(int status, int code, byte id, int messagetype,  unsigned int u88)
{ static int ms_old = 0, raz = 0;
  int ms, dms;
  float t;
  char str[10];
  ms = millis();
  dms = ms - ms_old;
  ms_old = ms;

//if(dms < 500)
//    return;
  if(raz > 1000)
    return;
  if(raz == 1000)
  {   Serial.printf("End of OT log\n");
      raz++; 
      return;     
  }
  
  raz++; 

  Serial.printf("%6d %3d ", ms, dms);
  Serial.printf("%3d ", id);
  if(code == 0)
  {
    switch(status)
    { case -1:
        Serial.printf((PGM_P)F("Resp: ParityErr %d %d %04x\n"), id, messagetype, u88);
          break;
      case OpenThermResponseStatus::NONE:
        Serial.println(F("Resp: NONE"));
        break;
      case OpenThermResponseStatus::TIMEOUT:
        Serial.println(F("Resp: TimeOut"));
        break;
      case OpenThermResponseStatus::INVALID:
        Serial.print(F("INVALID "));
        break;
      case OpenThermResponseStatus::SUCCESS:
        break;
    }
    if(status != 1 && status != 2)
      return;

    switch(id)
    {
      case OpenThermMessageID::Status:
        if( messagetype == OpenThermMessageType::READ_ACK)
        {
            Serial.printf("Resp: %2d READ_ACK  %04x (Status LB %d %d %d %d %d %d %d)\n", 
              id,  u88,(u88&0x40)>>6, (u88&0x20)>>5, (u88&0x10)>>4, (u88&0x08)>>3, (u88&0x04)>>2, (u88&0x02)>>1,  (u88&0x01));
        } else {
            Serial.printf("Resp: %2d messagetype %x  %04x (Status LB %d %d %d %d %d %d %d)\n", 
              id, messagetype, u88,(u88&0x40)>>6, (u88&0x20)>>5, (u88&0x10)>>4, (u88&0x08)>>3, (u88&0x04)>>2, (u88&0x02)>>1,  (u88&0x01));
        }
        break;
      case OpenThermMessageID::TSet:
        t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
        if(messagetype == WRITE_ACK)
              Serial.printf((PGM_P)F("Resp: TSet Write %.3f\n"), t);
        else if(messagetype == READ_ACK)
              Serial.printf((PGM_P)F("Resp: TSet read %.3f\n"), t);
        else if(messagetype == WRITE)
              Serial.printf((PGM_P)F("Resp: TSet WRITE  %.3f\n"),  t);
        else
              Serial.printf((PGM_P)F("Resp: TSet ! %d %04x\n"),  messagetype, u88);
        break;
      default:
        Serial.printf((PGM_P)F("Resp: %2d "), id);
	/* Slave to Master */
//	READ_ACK        = B100,
//	WRITE_ACK       = B101,
//	DATA_INVALID    = B110,
//	UNKNOWN_DATA_ID = B111

        if(messagetype == OpenThermMessageType::READ_ACK)
            Serial.print((PGM_P)F("READ_ACK "));
        else if(messagetype == OpenThermMessageType::WRITE_ACK)
            Serial.print((PGM_P)F("WRITE_ACK"));
        else if(messagetype == OpenThermMessageType::DATA_INVALID)
            Serial.print((PGM_P)F("DATA_INVALID"));
        else if(messagetype == OpenThermMessageType::UNKNOWN_DATA_ID)
        {   extern OpenThermID OT_ids[N_OT_NIDS]; 
            Serial.print((PGM_P)F("UNKNOWN_DATA_ID"));
             Serial.printf(" %d %d ", OT_ids[id].count, OT_ids[id].countOk );
        }   else 
            Serial.printf( "%d",messagetype);

        Serial.printf(" %04x\n", u88);
    }
    return;

  } 
  if(code == 1)
    strcpy(str, "Req ");
  else 
    strcpy(str, "ReqS");

  switch(id)
  {
    case OpenThermMessageID::Status:
      if( messagetype == OpenThermMessageType::READ_DATA)
      {
          Serial.printf("%s: %2d READ_DATA %04x (Status HB %d %d %d %d %d %d %d)\n", str,
            id,  u88,(u88&0x4000)>>14,(u88&0x2000)>>13, (u88&0x1000)>>12, (u88&0x0800)>>11, (u88&0x0400)>>10, (u88&0x0200)>>9,  (u88&0x0100)>>8);
      } else {
          Serial.printf("%s: %2d messagetype %x  %04x (Status HB %d %d %d %d %d)\n", str,
            id, messagetype, u88,(u88&0x1000)>>12, (u88&0x0800)>>11, (u88&0x0400)>>10, (u88&0x0200)>>9,  (u88&0x0100)>>8);
      }
      break;
    case OpenThermMessageID::TSet:
      t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
      if(messagetype == WRITE)
            Serial.printf((PGM_P)F("%s: TSet Write %.3f\n"), str,t);
      else if(messagetype == READ)
            Serial.printf((PGM_P)F("%s: TSet read %.3f\n"), str, t);
      else
            Serial.printf((PGM_P)F("%s: TSet ! %d %04x\n"), str, messagetype, u88);
      break;

    case OpenThermMessageID::SConfigSMemberIDcode:
    Serial.printf((PGM_P)F("%s: SConfigSMemberIDcode %d %04x\n"),str, messagetype,  u88);
      break;
    case OpenThermMessageID::MConfigMMemberIDcode:
    Serial.printf((PGM_P)F("%s: MConfigMMemberIDcode %d %04x\n"), str, messagetype,  u88);
      break;

    default:
        Serial.printf((PGM_P)F("%s: %2d "), str, id);
        if(messagetype == OpenThermMessageType::READ_DATA)
            Serial.print((PGM_P)F("READ_DATA"));
        else if(messagetype == OpenThermMessageType::WRITE_DATA)
            Serial.print((PGM_P)F("WRITE_DAT"));
        else 
            Serial.printf( "%d",messagetype);
        Serial.printf((PGM_P)F(" %04x\n"),  u88);
  }

 }   

#endif