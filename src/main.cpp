/* main.cpp */
// контроллер OpenTherm на ESP32 с измерением температуры
// WiFi, Captive Portal, Web доступ, клиент-сервер (UDP или TCP), удалённый контроль
// derived from OpenTherm lib by: Ihor Melnyk
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

#include "esp32/clk.h"
#include "soc/rtc.h"

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
void OTlogErr(int status, int sts);
void OTlogDelLast(void);
#endif
#if OT_MASTER_DEBUG || OT2_SLAVE_DEBUG 
void ot_debug_print(void);
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
extern unsigned int OTDebugInfo[12];
extern unsigned int OTcount;


void IRAM_ATTR handleInterrupt() {
    ot.handleInterrupt();  
}

void init_ot_slave(void)
{
#if 0
    Serial_db.printf("init_ot_slave\n");
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
//  for(j=0; j<3000; j++)
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
      if(j < 2)
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
#define RTC_WDT_TIME_MS (WDT_TIMEOUT *1100 + 10000)

void watchdog_setup(void)
{
//wdt  
  // Deinitialize the default watchdog (if enabled by default)
  esp_task_wdt_deinit();
  // Initialize the Task Watchdog
  esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT, true);
  if (err != ESP_OK) {
    Serial_db.printf("WDT Init failed: %s\n", esp_err_to_name(err));
    return;
  }

  // Add the current task (Arduino loop) to the watchdog watch list
  esp_task_wdt_add(NULL); 
  Serial_db.printf("Watchdog Timeout set to: %d seconds\n", WDT_TIMEOUT);

//rtc_wdt
  rtc_wdt_protect_off(); // Disable RTC WDT write protection
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_RTC); // Set action on timeout
  rtc_wdt_set_time(RTC_WDT_STAGE0, RTC_WDT_TIME_MS ); // Set timeout to WDT_TIMEOUT seconds + 100 ьы
  rtc_wdt_enable(); // Start the RTC WDT timer
  rtc_wdt_protect_on(); // Enable RTC WDT write protection  
  Serial_db.printf("RTC Watchdog Timeout set to: %d ms\n", RTC_WDT_TIME_MS);
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

//watchdogs reset      
void Watchdogsreset(void)
{ static  unsigned long t0 = 0; 
  unsigned long t1 = millis();
  if(t1 - t0 < 5000) //5 sec
      return;
  t0 = t1;
  esp_task_wdt_reset();
  rtc_wdt_feed();         
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
  uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save brownout register
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector  

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  LedSts = 1;
  digitalWrite(LED_BUILTIN, LedSts);   // Turn the LED on (Note that LOW is the voltage level

  Serial.begin(115200);

 ++bootCount;
//  Serial.print("Boot count: ");
//  Serial.println(bootCount);

  heap_caps_check_integrity_all(true);
  Serial_db.printf((PGM_P)F("%s Vers %d.%d.%d.%d build %s\n"),
       IDENTIFY_TEXT, SmOT.Vers, SmOT.SubVers,SmOT.SubVers1,SmOT.Revision, SmOT.BiosDate);
  check_reset();

  setup_read_config();

  watchdog_setup();

  Serial.printf("SmOT.useCPU_freq %d remote control %d\n", SmOT.useCPU_freq, SmOT.Use_remoteTCPserver);
  if(SmOT.useCPU_freq > 0)
  { int v = 80;
    if(SmOT.useCPU_freq == 1) v = 160;
    setCpuFrequencyMhz(v);

    Serial.printf("Set CPU Freq to %d\n", v);
  }
  SmOT.RelayInit();
/*******************************************/
  SmOT.planner_setup();

  ot.begin(handleInterrupt, OTprocessResponse);
#if ST_VERS == 2
 #if OT_SLAVE_DEBUG
     setup_ot_slave();
 #else
    if(SmOT.OT_slave_present)
    { if(SmOT.OT_slave_mode == 0)
      {    Serial_db.printf("setup_slave 1\n");
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

#if SERVER_DEBUG
#if 0
  SmOT.TCPserver_sts = 2;  /* статус сервера */
//  SmOT.TCPserver_sts2 = 1; 
  SmOT.TCPserver_t = millis();
  SmOT.TCPserver_port = 8876;  
  SmOT.TCPserver_report_period = 10000;
  SmOT.tcp_remoteIP.fromString("192.168.10.112");
  Serial_db.printf("TCPserver_report_period=%d TCPserver_port=%d\n", SmOT.TCPserver_report_period, SmOT.TCPserver_port);
#endif	
#endif	
	Serial_db.printf("Use remote server %d remoteIP: %s\n",  SmOT.Use_remoteTCPserver, SmOT.tcp_remoteIP.toString().c_str());

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector  
}


int status_OT = -1;
static int _SConfigSMemberIDcode = 0;

void setupDS1820(void)
{//  Serial.print("DS18B20 Library version: ");
 //  Serial.println(DS18B20_LIB_VERSION);

  SmOT.statusDS18b20 = 0x0;

  if(Tsensor1.begin() == false)
  {   SmOT.stsT1 = -1;
      SmOT.statusDS18b20 |= 0x02;
      Serial_db.printf((PGM_P)F("ERROR: No DS18b20(1) found on pin %i\n"), DS1820_1);
      delay(100);
      if(Tsensor1.begin() )
      {   Serial.println(F("2nd attempt(1) Ok"));
          goto M1;
      }

  }  else {
M1:      SmOT.stsT1 = 0;
      SmOT.statusDS18b20 |= 0x01;

      Tsensor1.setResolution(12);
      Tsensor1.setConfig(DS18B20_CRC);  // or 1
      Serial_db.printf((PGM_P)F("DS18b20(1) found on pin %i\n"), DS1820_1);
  }

  if(Tsensor2.begin() == false)
  {   SmOT.stsT2 = -1;
      SmOT.statusDS18b20 |= 0x0200;
      Serial_db.printf((PGM_P)F("ERROR: No DS18b20(2) found on pin %i\n"), DS1820_2);
      delay(100);
      if(Tsensor2.begin() )
      {   Serial.println(F("2nd attempt(2) Ok"));
          goto M2;
      }

  }  else {
M2:   SmOT.stsT2 = 0;
      SmOT.statusDS18b20 |= 0x0100;
      Tsensor2.setResolution(12);
      Tsensor2.setConfig(DS18B20_CRC);  // or 1
      Serial_db.printf((PGM_P)F("DS18b20(2) found on pin %i\n"), DS1820_2);
  }
 } 

void loopDS1820(void)
{ static unsigned long int start=0;
  int rc;
  static int nd=0;
  float t;

//  Serial_db.printf("loopDS1820 nd %i %li\n", nd, millis());
  switch(nd)
  {   case 0:
        if(SmOT.statusDS18b20&0x01)
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
            { SmOT.statusDS18b20 |= 0x04;  //бит таймаута
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
          SmOT.statusDS18b20 &= ~0x04; // сброс бита таймаута
          if(t == DEVICE_DISCONNECTED)
          {
            SmOT.stsT1 = 4;
            SmOT.statusDS18b20 |= 0x20;
#if SERIAL_DEBUG 
            Serial.println(F("ERROR: DS1 Disconnected"));
#endif            

          } else if (t == DEVICE_CRC_ERROR) {
            SmOT.stsT1 = 2;
            SmOT.statusDS18b20 |= 0x10;
#if SERIAL_DEBUG 
            Serial.println(F("ERROR: DS1 CRC error"));
#endif            
          } else {
            SmOT.statusDS18b20 &= ~0x30; // сброс битов CRC error&Disconnected
            if(SmOT.stsT1 == 1)
                SmOT.t1 = (SmOT.t1 + t) * 0.5;
            else
                SmOT.t1 = t;
            SmOT.stsT1 = 1;
            SmOT.OnChangeT(t,0);    
//            Serial_db.printf("SmOT T1= %f\n",   SmOT.t1);
          }
          SmOT.statusDS18b20 &= ~0x04;
          nd = 2;
        }
        break;

      case 2:
        if(SmOT.statusDS18b20&0x0100)
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
          { SmOT.statusDS18b20 |= 0x0400;//бит таймаута
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
        {  SmOT.statusDS18b20 &= ~0x0400; // сброс бита таймаута

          t = Tsensor2.getTempC();
          if(t == DEVICE_DISCONNECTED)
          {
            SmOT.stsT2 = 4;
            SmOT.statusDS18b20 |= 0x2000;
#if SERIAL_DEBUG 
            Serial.println(F("ERROR: DS1 Disconnected"));
#endif            

          } else  if (t == DEVICE_CRC_ERROR)  {
             SmOT.stsT2 = 2;
            SmOT.statusDS18b20 |= 0x1000;
      #if SERIAL_DEBUG 
            Serial.println(F("ERROR: DS2 CRC error"));
      #endif            
          } else {
            SmOT.statusDS18b20 &= ~0x3000; // сброс битов CRC error&Disconnected

            if(SmOT.stsT2 == 1)
                SmOT.t2 = (SmOT.t2 + t) * 0.5;
            else
                SmOT.t2 = t;
            SmOT.stsT2 = 1;
            SmOT.OnChangeT(t,1);    

//            Serial_db.printf("SmOT T2= %f\n",   SmOT.t2);
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

    SmOT.nr_OT++;

     OTcount++;
     SmOT.RespMillis = millis();
     id = (response >> 16 & 0xFF);

     if(SmOT.TestCmd == 2)
    {
      if(id == (SmOT.TestId & 0xff))
      {
#if OT_DEBUG
    Serial_db.printf("TestCmd processResponse %x %x\n", response,  status);
#endif
        SmOT.TestResponse = response;
        SmOT.TestStatus = status;
        SmOT.TestCmd = 0;
        return;
      }
    }
  
    if (status == OpenThermResponseStatus::SUCCESS) {
		   if(SmOT.stsOT != 0)
        { SmOT.stsOT =  0;  
          SmOT.MQTT_need_report = 1;
          SmOT.OnOpenThermRestore();
        }

        timeOutcounter = 0;
        SmOT.response = response; 
        OTDebugInfo[0]++;
    } else if (status == OpenThermResponseStatus::NONE) {
        OTDebugInfo[2]++;
    } else if (status == OpenThermResponseStatus::INVALID) {
        OTDebugInfo[3]++;
        OTlogErr(2, 1);
        return;
    } else if (status == OpenThermResponseStatus::TIMEOUT) {
      if(SmOT.stsOT != -1)
	    { //if(SmOT.stsOT != 2)
        //    Serial_db.printf("TIMEOUT %d %x t %d %d\n", timeOutcounter, response, millis(), SmOT.stsOT);
        if(timeOutcounter > 10)
		    { if(SmOT.stsOT != 2)
               SmOT.MQTT_need_report = 1;
          SmOT.stsOT = 2;
		    } else {
			    timeOutcounter++;
		    }	
#if OT_DEBUGLOG
        if(SmOT.stsOT == 2)
          OTlogDelLast();
        else 
          OTlogErr(1, 1);
#endif
      } else {
#if OT_DEBUGLOG
        OTlogDelLast();
#endif
      }
      OTDebugInfo[4]++;
      return;
    }

#if OT_DEBUG
  { unsigned int u88;
    u88 = (response & 0xffff);
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
//Serial_db.printf("buildRequestIfNeed raz %d\n", raz);
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

    SmOT.responseID = id;

    messagetype = ot.getMessageType(response);
    if(messagetype == DATA_INVALID)
    { OTDebugInfo[7]++;
      ot.update_OTid(id, 1); //used, bur data invalid  
      #if SERIAL_DEBUG 
      Serial.println(F("DATA_INVALID"));
#endif        
      return;
    }
    
    if(messagetype == UNKNOWN_DATA_ID)
    { OTDebugInfo[8]++;

      ot.update_OTid(id, 0);
         if(OTstartSts > 0)  OTstartSts++;
//      Serial_db.printf("UNKNOWN_DATA_ID %d (%x)\n", id, id);
      return;
    }
    if(messagetype != READ_ACK && messagetype != WRITE_ACK )
    { OTDebugInfo[9]++;
         if(OTstartSts > 0)  OTstartSts++;
#if SERIAL_DEBUG 
        Serial_db.printf("Messagetype  %d!!! Status %d %d ot.LastRequestId %d\n", messagetype, status, SmOT.stsOT, ot.LastRequestId);
#endif        
      return;
    }
    
    if(SmOT.stsOT != 0)
        return;
    SmOT.t_lastwork = time(nullptr);

    id = (response >> 16 & 0xFF);
    
    if(id != ot.LastRequestId)
    { OTDebugInfo[10]++;
#if SERIAL_DEBUG 
        Serial_db.printf("Resp id %d != Req id %d\n", id, ot.LastRequestId );
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

//        Serial_db.printf("BoilerStatus: %x %x\n", u88, response);
//        Serial.println("Central Heating: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
//        Serial.println("Hot Water: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
//        Serial.println("Flame: " + String(ot.isFlameOn(response) ? "on" : "off"));

//        curr_item->status = boiler_status;
//        Serial.println("Boiler status: " + String(boiler_status, BIN));
         if(OTstartSts == 0)  OTstartSts++;

        break;

    case OpenThermMessageID::TSet:  // 1
    SmOT.Tset_r = t;
        if(u88 == ot.temperatureToData(SmOT.Tset) )
          SmOT.Decriment_NeedSet(OpenThermMessageID::TSet);

            break;
        
    case OpenThermMessageID::MConfigMMemberIDcode: //2
         if(OTstartSts == 2)  OTstartSts++;
#if SERIAL_DEBUG 
       Serial_db.printf((PGM_P)F("OpenThermMessageID::MConfigMMemberIDcode, %d\n"), OTstartSts);
#endif
        break;

    case OpenThermMessageID::SConfigSMemberIDcode:  //3
        SmOT.handle_SConfigSMemberIDcode(u88);
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
//        Serial_db.printf("SmOT.OTmemberCode %d\n", SmOT.OTmemberCode);
         }
//        Serial_db.printf("OTstartSts %d: u88 %x SmOT.HotWater_present = %d\n", OTstartSts, u88, SmOT.HotWater_present );
        break;

    case OpenThermMessageID::RemoteRequest: // 4 Remote Request
//    Serial_db.printf("RemoteRequest responce %d\n", u88 );

      SmOT.Decriment_NeedSet(OpenThermMessageID::RemoteRequest);
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

    case  OpenThermMessageID::TsetCH2: // 8 W 
      SmOT.Decriment_NeedSet(OpenThermMessageID::TsetCH2);
        break;

    case OpenThermMessageID::MaxRelModLevelSetting: //14 Maximum relative modulation level setting (%) W
        SmOT.Decriment_NeedSet(OpenThermMessageID::MaxRelModLevelSetting);
    //        SmOT.MaxRelModLevelSetting = t;
        break;

    case OpenThermMessageID::MaxCapacityMinModLevel:	//15 MaxCapacityMinModLevel, // u8 / u8  Maximum boiler capacity (kW) / Minimum boiler modulation level(%) R
        SmOT.MinModLevel =  (u88 & 0xff);
        SmOT.MaxCapacity =  ((u88>>8) & 0xff);
        break;

    case OpenThermMessageID::TrSet: // 16  Room Setpoint (°C) TrSet:  
        break;

    case OpenThermMessageID::RelModLevel: //17 Relative Modulation Level 
        SmOT.FlameModulation = t;
        SmOT.Bstat.calcIntegral(t);
        break;

    case OpenThermMessageID::CHPressure: //18 Water pressure in CH circuit
        SmOT.Pressure = t;
        break;

    case OpenThermMessageID::DHWFlowRate: //19 Water flow rate in DHW circuit. (litres / minute)
      SmOT.DHWFlowRate = t;
        break;

    case OpenThermMessageID::Tr: // 24 f8.8  Room temperature (°C)
        break;

    case OpenThermMessageID::Tboiler:  //25
        SmOT.BoilerT = t;
        if(fabs(SmOT.BoilerT-SmOT.Tset) > 5.) 
        { SmOT.need_set_T(1);
        }          
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

    case OpenThermMessageID::TdhwSetUBTdhwSetLB: //48 s8/s8 DHW Setpoint upper & lower bounds for adjustment(°C)
      SmOT.TdhwSetUB = float((u88>>8) & 0xff);
      SmOT.TdhwSetLB = float(u88 & 0xff);
        break;

    case OpenThermMessageID::MaxTSetUBMaxTSetLB: //49 s8/s8 Max CH water Setpoint upper & lower bounds for adjustment(°C)
      SmOT.MaxTSetUB = float((u88>>8) & 0xff);
      SmOT.MaxTSetLB = float(u88 & 0xff);
        break;

    case OpenThermMessageID::TdhwSet: //56 W
      SmOT.Decriment_NeedSet(OpenThermMessageID::TdhwSet);
        break;

    case OpenThermMessageID::MaxTSet: //57 W
      SmOT.Decriment_NeedSet(OpenThermMessageID::MaxTSet);
        break;
      
    case OpenThermMessageID::OEMDiagnosticCode: //115
        if(u88)
          OTDebugInfo[6]++;
        SmOT.OEMDcode = u88;
//        Serial_db.printf("OEMDcode: %x\n", SmOT.OEMDcode);
        SmOT.rcode[4] = u88;
        break;
        

    default:
#if OT_DEBUG     
        Serial.println("Unknown: Response: " + String(response, HEX) + ", id=" + String(id));
#endif
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
    Serial_db.printf("TestRequest: %x\n", request);
#endif    
    return request;
}


/* return 0 if no response, 1 if have responce */
int OTloop(void)
{   static int st = 1;
    int rc = 0, ot_id;

#if ST_VERS == 2
static int slst = 0;
  if(SmOT.OT_slave_present && SmOT.OT_slave_mode == 1)
  { if(SmOT.ot_slave_stsOT == -2)
    { if(SmOT.stsOT == 0)
      {   Serial_db.printf("setup_slave 2\n");
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
      {  unsigned int request = 0;
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
    { // Serial_db.printf("SmOT.ot_slave_stsOT %d ot_SlaveSts %d  %d\n", SmOT.ot_slave_stsOT, ot_SlaveSts, millis() );

      if(SmOT.ot_slave_stsOT == 0)
      { if(ot_SlaveSts == 1)
        { int ids; 
          request = ot_SlaveRequest;
          ids = (request & 0xff0000) >>16;
          if(ids ==0)
              SmOT.BoilerStatusRequest = request;
          ot.LastRequestId = ids;
          ot_SlaveSts = 2;
          slst = 1;
          OTlog(request,2);
        } else {
          if(slst == 1)
              goto M00;
          break; // do nothing - no reqest from slave
        }
      } else {
        extern unsigned long ot_SlaveRequest_ms;

         if(millis() - ot_SlaveRequest_ms > 2000) //wait 3 sek than work as master
            goto M00;
  
      }
    } else {
M00:
    ot_id = SmOT.planner_loop();
// Serial_db.printf("* OTloop ot_id %d (%d %d) %d\n",  ot_id, SmOT.nOTlog, SmOT.OTlogBuf.GetLbuf(), millis() );
  if(ot_id >= 0)
         request = SmOT.buildRequest(ot_id);       
#if OT_DEBUGLOG
//        if(SmOT.OT_slave_mode == 0)
            OTlog(request,0);
#endif          
        slst = 0;
    }
#endif

#else //ST_VERS == 2
    ot_id = SmOT.planner_loop();
//      Serial_db.printf("OTloop ot_id %d (%d %d)\n",  ot_id, SmOT.nOTlog, SmOT.OTlogBuf.GetLbuf() );
      
    if(ot_id >= 0)
         request = SmOT.buildRequest(ot_id);       
//         else
//            request = buildRequest(0);

 #if OT_DEBUGLOG
         OTlog(request,0);
 #endif          
#endif
        if(ot.sendRequestAync(request))    // 	status = OpenThermStatus::RESPONSE_WAITING;    
        {    st++;
          SmOT.ns_OT++; 
        }

      }
      break;

      case 1:
        {   st++;
        }

      break;

      case 2:
       ot.process();
        if(ot.status ==  OpenThermStatus::READY)
        { 
          st = 0;
          rc = 1;
//          unsigned int id;
//          id = (ot.getLastResponse() >> 16 & 0xFF);
//             Serial_db.printf("Last ResponseStatus:  %d Response id %d\n",  ot.getLastResponseStatus(), id);
        }
      break;
    }

    return rc;
}


unsigned int SD_Termo::buildRequest(int ot_id)
{ unsigned int request = 0;
  int rc;

// Serial_db.printf("SD_Termo::buildRequest %d\n", ot_id);
  switch(ot_id) 
  {
/**************************/
    case OpenThermMessageID::Status: // 0 запрос статуса
    
#if PID_USE
      if(!usePID)
         enable_CentralHeating_real = enable_CentralHeating;
      if(CH2_DHW_flag && enable_HotWater)
      {  request = ot.buildSetBoilerStatusRequest(enable_CentralHeating_real, enable_HotWater, enable_Cooling, Use_OTC, 1, UseWinterMode);
      } else {
         request = ot.buildSetBoilerStatusRequest(enable_CentralHeating_real, enable_HotWater, enable_Cooling, Use_OTC, enable_CentralHeating2, UseWinterMode);
      }   
{  static int old_CH = -1;
  if(old_CH != enable_CentralHeating_real)
    {
      if(ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting) && enable_CentralHeating_real && Use_MaxRelModLevel)
        need_set_MaxRelModLevel(2); 

// Serial_db.printf("enable_CentralHeating_real  %d %d\n",enable_CentralHeating_real, old_CH );
#if MQTT_USE 
      rc = MQTT_pub_cmdCH(enable_CentralHeating_real);
      if(rc)
        old_CH = enable_CentralHeating_real;
#else      
      old_CH = enable_CentralHeating_real;
       
#endif      
      }

}
#else
      if(CH2_DHW_flag && enable_HotWater)
      {  request = ot.buildSetBoilerStatusRequest(enable_CentralHeating, enable_HotWater, enable_Cooling, Use_OTC, 1, UseWinterMode);
      } else {
        request = ot.buildSetBoilerStatusRequest(enable_CentralHeating, enable_HotWater, enable_Cooling,  Use_OTC, enable_CentralHeating2, UseWinterMode);
      }   
#endif

    BoilerStatusRequest = request;
        break;
    
    case OpenThermMessageID::TSet:
      request = ot.buildSetBoilerTemperatureRequest(Tset); //1 W
        break;

/**************************/
    case  OpenThermMessageID::MConfigMMemberIDcode: // 2
      if(Immergas_fix_flag && !UseID2)
        request = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MConfigMMemberIDcode, _SConfigSMemberIDcode); //3
      else 
        request = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MConfigMMemberIDcode, ID2masterID /* (_SConfigSMemberIDcode&0xff) */); //3
        
      break;

/**************************/
    case OpenThermMessageID::SConfigSMemberIDcode: // 3
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::SConfigSMemberIDcode, 0); //3
        break;

/**************************/
  case OpenThermMessageID::RemoteRequest: //4 W
  { 	unsigned int data = 0; //4
      if(need_send_Blor)
      {   data = (0x01<<8);  //BLOR        
          need_send_Blor = 0;
      }
//      Serial_db.printf("RemoteRequest  %d\n", data );

        request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::RemoteRequest, data);
  }
        break;

  case OpenThermMessageID::ASFflags: // 5 R
//todo  if(ot.OTid_used(OpenThermMessageID::ASFflags)) 
  
    request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::ASFflags, 0);
/*
0: fault indication [ no fault, fault ] 0x01
6: diagnostic/service indication [no diagnostics, diagnostic event] 0x40
*/
//todo    if(!(SmOT.BoilerStatus & 0x41 || SmOT.Fault) )
//todo        st = 0;

        break;

  case  OpenThermMessageID::TsetCH2: // 8 W 
    request = ot.buildSetBoilerCH2TemperatureRequest(Tset2); //8
        break;

  case OpenThermMessageID::MaxRelModLevelSetting: //14 W
  { 	unsigned int data = ot.temperatureToData(MaxRelModLevelSetting);
    request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MaxRelModLevelSetting, data);
  }
        break;
  case OpenThermMessageID::MaxCapacityMinModLevel: //15 R
    request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::MaxCapacityMinModLevel, 0);
        break;	        

  case  OpenThermMessageID::TrSet: // 16 W
//todo if(Use_OTC || OTmemberCode == 248) 
//todo    if(ot.OTid_used(OpenThermMessageID::TrSet)) // 16  Room Setpoint (°C)
    { 
//     unsigned int data = ot.temperatureToData(TroomTarget);
      //unsigned int data = ot.temperatureToData(22.f);
      unsigned int data = ot.temperatureToData(24.f);
      request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TrSet, data);
    } 

        break;
       
  case OpenThermMessageID::RelModLevel: //17 R
//todo  if(ot.OTid_used(OpenThermMessageID::RelModLevel))
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::RelModLevel, 0); //17
        break;

  case OpenThermMessageID::CHPressure: //18 R
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::CHPressure, 0); //18
        break;

  case OpenThermMessageID::DHWFlowRate: //19 R 
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::DHWFlowRate, 0); //19
        break;

  case OpenThermMessageID::Tr: //24 W
//todo  if(Use_OTC || OTmemberCode == 248) /* OTC or Zota **/
//todo    if(ot.OTid_used(OpenThermMessageID::Tr)) //  24 Room temperature (°C)
    { //unsigned int data = ot.temperatureToData(tempindoor);
//          unsigned int data = ot.temperatureToData(24.f);
      unsigned int data = ot.temperatureToData(22.f);
      request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::Tr, data);
    } 
        break;

  case OpenThermMessageID::Tboiler: //25 R
      request = ot.buildGetBoilerTemperatureRequest();
        break;

  case OpenThermMessageID::Tdhw: //26 R
//todo  if(HotWater_present && ot.OTid_used(OpenThermMessageID::Tdhw) )
       request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0); //26
        break;

  case OpenThermMessageID::Toutside: //27 R
    request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Toutside, 0); //27
        break;

  case OpenThermMessageID::Tret: //28 R
  // todo if(ot.OTid_used(OpenThermMessageID::Tret))
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tret, 0); //28
        break;

  case OpenThermMessageID::Tstorage: //29 R
//todo  if(Use_ID29_DHW_flag)
//todo    if(ot.OTid_used(OpenThermMessageID::Tstorage))
        request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tstorage, 0); //29
        break;      

  case OpenThermMessageID::TflowCH2: //31 R
//todo  if(enable_CentralHeating2 && ot.OTid_used(OpenThermMessageID::TflowCH2))       
      request = ot.buildGetBoilerCH2TemperatureRequest(); //TflowCH2
        break;
      
  case OpenThermMessageID::Texhaust: //33 R
//todo  if(ot.OTid_used(OpenThermMessageID::Texhaust))
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Texhaust, 0); //33
        break;

  case OpenThermMessageID::TdhwSetUBTdhwSetLB: //48 R
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::TdhwSetUBTdhwSetLB, 0); 
        break;


  case OpenThermMessageID::MaxTSetUBMaxTSetLB: //49 R
      request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::MaxTSetUBMaxTSetLB, 0); 
        break;

  case OpenThermMessageID::TdhwSet: // 56 W
//todo  if(need_set_dhwT > 0) {
      request = ot.buildSetDHWSetpointTemperatureRequest(TdhwSet); 
        break;

  case OpenThermMessageID::MaxTSet: //57 W
  { 	unsigned int data = ot.temperatureToData(MaxTSet);
      request  = ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::MaxTSet, data);
  }
        break;

  case OpenThermMessageID::OEMDiagnosticCode: // 115 R
//todo  if(ot.OTid_used(OpenThermMessageID::OEMDiagnosticCode)) 
  {   request = ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::OEMDiagnosticCode, 0);
  }  
        break;

/**************************/
    default:
        Serial_db.printf("Error: unknown id %d in SD_Termo::buildRequest\n", ot_id);
  }

  return request;
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
{   
    unsigned long t;
    int dt;

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
//   Serial_db.printf("raz=%d\n", raz);
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
 //   static unsigned long  t0=0; // t1=0;
//    unsigned long t;

//    t = millis();
//     dt = t - t0;
 //Serial_db.printf("l2 %d %d\n", irot, t);
 //test debug
  bootSts = irot;

 #if T_DEBUG 
  {
    static int count=0, told=0;
    int dt1;
    t = millis();
    dt1 = t-told;
    if(dt1 > 1000 && told!= 0 )
    { 
      Serial_db.printf("loop2=%d dt=%d, irot %d\n", count++, dt1, irot);
    } 
    told = t;
  }
#endif

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
   
//Serial_db.printf("loop_udp\n");
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
    
    Serial_db.printf("IRAM free: %6d bytes (min %d) maxFreeBlock %6d\n", free, minRamFree, maxFreeBlockSize) ;
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
    Watchdogsreset();
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
//          t0 = t;   
     
#if 1
/************************/ 
//test for lost OT connection
      {  time_t now = time(nullptr);
        double dt;
        dt = difftime(now,SmOT.t_lastwork);
        if(dt > 10.)
        {         //sprintf(str0, "Потеря связи с котлом %.f сек назад", dt);
//          if(OTstartSts == OTstartSts_MAX)
            if(SmOT.plan.mask != MODE_START)
            {  // OTstartSts = 0;  
                SmOT.HotWater_present = false;
                SmOT.enable_CentralHeating2  = false; 
                SmOT.plan.SetMode(MODE_START); // init start sequence
            }
        }
      }
#endif //0      
/************************/                 
          return; 
        }
//        
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
  Watchdogsreset();  
/**************************/  
  ST_setCpuFrequencyMhz(SmOT.useCPU_freq);

/*******************************/
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
  {
#if ST_VERS == 2
    if(!(SmOT.OT_slave_present && SmOT.OT_slave_mode == 1 && SmOT.ot_slave_stsOT == 0))
      SmOT.loop_PID();
#else
      SmOT.loop_PID();
#endif 
  }  
#endif

/*
#if SERIAL_DEBUG 
Serial_db.printf( "%02d.%02d.%d %d:%02d:%02d\n",
          nowtime->tm_mday,nowtime->tm_mon+1,nowtime->tm_year+1900,
		  nowtime->tm_hour, nowtime->tm_min, nowtime->tm_sec);
#endif
*/      

#if OT_MASTER_DEBUG || OT2_SLAVE_DEBUG 
  ot_debug_print();
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

//     Serial_db.printf("sec_hour = %d Eff_Mod_h=%f ModIntegral_h=%f\n", SmOT.Bstat.sec_h, SmOT.Bstat.Eff_Mod_h, SmOT.Bstat.ModIntegral_h );


    if(SmOT.Bstat.sec_d)
      SmOT.Bstat.Eff_Mod_d = SmOT.Bstat.ModIntegral_d / (float)(SmOT.Bstat.sec_d + dt);

//     Serial_db.printf("sec_d = %d Eff_Mod_d=%f ModIntegral_d=%f\n", SmOT.Bstat.sec_d, SmOT.Bstat.Eff_Mod_d, SmOT.Bstat.ModIntegral_d );
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

#if OT_DEBUGLOG
//
#define OT_TIMEOUT 255
#define OT_INVALID 254

void OTlogErr(int status, int sts)
{ unsigned int reqresp;
  if(status == 1) //timeout
      reqresp =  OT_TIMEOUT << 16;
  else            //invalid
      reqresp = OT_INVALID << 16;
	if (ot.parity(reqresp)) reqresp |= (1ul << 31);

//      Serial_db.printf("OTlogErr %d SmOT.nOTlog %d\n", status, SmOT.nOTlog);

  OTlog(reqresp, sts);
}
//пишем в кольцевой буфер не более 1024 пакетов
//sts: 0 - request, 1 - response, 
//2 - request from slave interface
//3 - response from slave interface in case of invalid request
void OTlog(unsigned int reqresp, int sts)
{ unsigned int b[2];
  unsigned long t = millis();
  int lb, rc;
static unsigned long t0 = 0; 

  if(!SmOT.Use_remoteTCPserver)
    return;
    
  lb = SmOT.OTlogBuf.Lbuf/SmOT.OTlogBuf.Litem - SmOT.OTlogBuf.GetLbuf(); //

  if(SmOT.nOTlog < 1024 && lb > 1)
  { b[0] = ( (((sts<<6)|(SmOT.nOTlog & 0x3f)) << 24) | (t & 0xffffff)); //  
    
{
  unsigned long t1;
  t1 = t & 0xffffff;
  if((t1-t0) > 1000 ) 
    Serial_db.printf("==>+dt %ld t1=%ld, t0=%ld\n", t1-t0, t1, t0);
  t0 = t1;
}    
//    Serial_db.printf("SmOT.nOTlog %d Lbuf= %d sts %d %8x\n", 
//        SmOT.nOTlog, SmOT.OTlogBuf.GetLbuf(), sts, b[0]);
    b[1] =  reqresp;

    rc = SmOT.OTlogBuf.Add( b);
    if(rc == 0)
        Serial_db.printf("==>SmOT.nOTlog %d Lbuf= %d sts %d %8x\n", 
              SmOT.nOTlog, SmOT.OTlogBuf.GetLbuf(), sts, b[0]);

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
  {   Serial_db.printf("End of OT log\n");
      raz++; 
      return;     
  }
  
  raz++; 

  Serial_db.printf("%6d %3d ", ms, dms);
  Serial_db.printf("%3d ", id);
  if(code == 0)
  {
    switch(status)
    { case -1:
        Serial_db.printf((PGM_P)F("Resp: ParityErr %d %d %04x\n"), id, messagetype, u88);
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
            Serial_db.printf("Resp: %2d READ_ACK  %04x (Status LB %d %d %d %d %d %d %d)\n", 
              id,  u88,(u88&0x40)>>6, (u88&0x20)>>5, (u88&0x10)>>4, (u88&0x08)>>3, (u88&0x04)>>2, (u88&0x02)>>1,  (u88&0x01));
        } else {
            Serial_db.printf("Resp: %2d messagetype %x  %04x (Status LB %d %d %d %d %d %d %d)\n", 
              id, messagetype, u88,(u88&0x40)>>6, (u88&0x20)>>5, (u88&0x10)>>4, (u88&0x08)>>3, (u88&0x04)>>2, (u88&0x02)>>1,  (u88&0x01));
        }
        break;
      case OpenThermMessageID::TSet:
        t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
        if(messagetype == WRITE_ACK)
              Serial_db.printf((PGM_P)F("Resp: TSet Write %.3f\n"), t);
        else if(messagetype == READ_ACK)
              Serial_db.printf((PGM_P)F("Resp: TSet read %.3f\n"), t);
        else if(messagetype == WRITE)
              Serial_db.printf((PGM_P)F("Resp: TSet WRITE  %.3f\n"),  t);
        else
              Serial_db.printf((PGM_P)F("Resp: TSet ! %d %04x\n"),  messagetype, u88);
        break;
      default:
        Serial_db.printf((PGM_P)F("Resp: %2d "), id);
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
             Serial_db.printf(" %d %d ", OT_ids[id].count, OT_ids[id].countOk );
        }   else 
            Serial_db.printf( "%d",messagetype);

        Serial_db.printf(" %04x\n", u88);
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
          Serial_db.printf("%s: %2d READ_DATA %04x (Status HB %d %d %d %d %d %d %d)\n", str,
            id,  u88,(u88&0x4000)>>14,(u88&0x2000)>>13, (u88&0x1000)>>12, (u88&0x0800)>>11, (u88&0x0400)>>10, (u88&0x0200)>>9,  (u88&0x0100)>>8);
      } else {
          Serial_db.printf("%s: %2d messagetype %x  %04x (Status HB %d %d %d %d %d)\n", str,
            id, messagetype, u88,(u88&0x1000)>>12, (u88&0x0800)>>11, (u88&0x0400)>>10, (u88&0x0200)>>9,  (u88&0x0100)>>8);
      }
      break;
    case OpenThermMessageID::TSet:
      t = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
      if(messagetype == WRITE)
            Serial_db.printf((PGM_P)F("%s: TSet Write %.3f\n"), str,t);
      else if(messagetype == READ)
            Serial_db.printf((PGM_P)F("%s: TSet read %.3f\n"), str, t);
      else
            Serial_db.printf((PGM_P)F("%s: TSet ! %d %04x\n"), str, messagetype, u88);
      break;

    case OpenThermMessageID::SConfigSMemberIDcode:
    Serial_db.printf((PGM_P)F("%s: SConfigSMemberIDcode %d %04x\n"),str, messagetype,  u88);
      break;
    case OpenThermMessageID::MConfigMMemberIDcode:
    Serial_db.printf((PGM_P)F("%s: MConfigMMemberIDcode %d %04x\n"), str, messagetype,  u88);
      break;

    default:
        Serial_db.printf((PGM_P)F("%s: %2d "), str, id);
        if(messagetype == OpenThermMessageType::READ_DATA)
            Serial.print((PGM_P)F("READ_DATA"));
        else if(messagetype == OpenThermMessageType::WRITE_DATA)
            Serial.print((PGM_P)F("WRITE_DAT"));
        else 
            Serial_db.printf( "%d",messagetype);
        Serial_db.printf((PGM_P)F(" %04x\n"),  u88);
  }

 }   

#endif

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

#if OT_MASTER_DEBUG || OT2_SLAVE_DEBUG 
void ot_debug_print(void)
{
  extern otst otst_d[128];
  extern int Notst;
  extern int Flag_otst; 
  int i;

  if(Flag_otst)
  {
    if(Notst > 0)
    {
#if OT_MASTER_DEBUG 
      Serial.printf("OT master data\n");
#elif OT2_SLAVE_DEBUG 
      Serial.printf("OT slave  data\n");
#endif      
      Serial.printf("N state dt_mks status bitIndex  responce_hex\n");
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

}
#endif