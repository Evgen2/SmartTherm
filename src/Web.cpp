/* Web.cpp  UTF-8  */

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
using WiFiWebServer = ESP8266WebServer;
#define FORMAT_ON_FAIL
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
using WiFiWebServer = WebServer;
#define FORMAT_ON_FAIL  true
#endif

#include <time.h>
#include "OpenTherm.h"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

/*
  Include AutoConnectFS.h allows the sketch to retrieve the current file
  system that AutoConnect has selected. It derives a constant
  AUTOCONNECT_APPLIED_FILESYSTEM according to the definition state of
  AC_USE_SPIFFS or AC_USE_LITTLEFS in AutoConnectDefs.h.
  Also, the AutoConnectFS::FS class indicates either SPIFFS or LittleFS
  and will select the appropriate filesystem class depending on the file
  system applied to the sketch by the definition AC_USE_SPIFFS or
  AC_USE_LITTLEFS in AutoConnectDefs.h.
  You no need to change the sketch due to the file system change, declare
  the Filesystem object according to the following usage:
  
  #include <AutoConnectFS.h>
  AutoConnectFS::FS& name = AUTOCONNECT_APPLIED_FILESYSTEM;
  name.begin(AUTOCONNECT_FS_INITIALIZATION);
*/

#include <AutoConnect.h>
#include <AutoConnectFS.h>
AutoConnectFS::FS& FlashFS = AUTOCONNECT_APPLIED_FILESYSTEM;

#if defined(ARDUINO_ARCH_ESP8266)
char SmartDevice::BiosDate[12]=__DATE__;   /* дата компиляции биоса */

#endif

extern  SD_Termo SmOT;
int WiFiDebugInfo[10] ={0,0,0,0,0, 0,0,0,0,0};
int OTDebugInfo[12] ={0,0,0,0,0, 0,0,0,0,0, 0,0};
extern OpenThermID OT_ids[N_OT_NIDS];
unsigned int OTcount = 0;


/*********************************/
const char* INFO_URI  = "/info";
const char* SETUP_URI = "/setup";
const char* RELAY_URI = "/relay";
const char* BLOR_URI  = "/blor";
const char* SETUP_ADD_URI =  "/setupadd";
const char* ABOUT_URI   = "/about";
const char* SET_T_URI   = "/set_t";
const char* SET_PAR_URI = "/set_par";
const char* SET_ADD_URI = "/add";
const char* DEBUG_URI   = "/debug";
#if PID_USE
const char* PID_URI = "/pid";
const char* SET_PID_URI = "/set_pid";
#endif
#if ST_VERS == 2
const char* SET_OT2_URI = "/setot2";
const char* OT2_URI = "/ot2";
#endif

const char* STYLE_WIDTH = "width:15%";
/************* InfoPage ******************/
ACText(Caption, "<b>Статус OT: </b>", "", "", AC_Tag_DIV);
ACText(Info1, "", "", "", AC_Tag_DIV);
ACText(Info2, "", "", "", AC_Tag_DIV);
ACText(Info3, "", "", "", AC_Tag_DIV);
ACText(Info4, "", "", "", AC_Tag_DIV);
ACText(Info5, "", "", "", AC_Tag_DIV);
ACText(Info6, "", "", "", AC_Tag_DIV);
ACText(Info7, "", "", "", AC_Tag_DIV);
//ACInput(SetBoilerTemp,"", "Температура теплоносителя:<br>"); // Boiler Control setpoint
ACInput(SetBoilerTemp,"", "Температура теплоносителя:<br>", "", "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH); // Boiler Control setpoint
//ACInput(SetBoilerTemp,"", "Температура теплоносителя:<br>", "", "Plaseholder",AC_Tag_BR, AC_Input_Number, "size=\"10\"" ); // Boiler Control setpoint
//AutoConnectInput(const char* name, const char* value, const char* label, const char* pattern, const char* placeholder, const ACPosterior_t post, const ACInput_t apply
ACInput(SetDHWTemp,   "", "Температура горячей воды:<br>", "",  "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);  // DHW Control setpoint
ACInput(SetBoilerTemp2,"", "Температура CH2:<br>"); // Boiler CH2 Control setpoint

#if RELAY_USE
ACSubmit(RelayOmFf, "Реле вкл/выкл", RELAY_URI, AC_Tag_None);
#endif
ACSubmit(Apply, "Обновить", INFO_URI, AC_Tag_BR);
ACSubmit(SetNewBoilerTemp,"Задать", SET_T_URI, AC_Tag_DIV);

/************* SetupPage ***************/
ACText(Ctrl2, "", "", "", AC_Tag_DIV);
//ACCheckbox(CtrlChB1,"checkbox", "uniqueapid");
AutoConnectCheckbox CtrlChB1("CtrlChB1","1", "Отопление", false, AC_Behind , AC_Tag_BR);
AutoConnectCheckbox CtrlChB2("CtrlChB2","2", "Горячая вода", false, AC_Behind , AC_Tag_DIV);
AutoConnectCheckbox CtrlChB3("CtrlChB3","3", "Отопление CH2", false, AC_Behind , AC_Tag_DIV);
ACInput(SetMaxMod,"", "проценты","",  "0-100%",AC_Tag_None, AC_Input_Text, STYLE_WIDTH); 
AutoConnectCheckbox CtrlChBMmod("CtrlChBmmod","4", "Макс модуляция", false, AC_Behind , AC_Tag_BR);
#if RELAY_USE
AutoConnectCheckbox CtrlChBUseRelay("ChbUseRelay","5", "Реле", false, AC_Behind , AC_Tag_None);
AutoConnectCheckbox CtrlChBStartRelaySts("ChbStertRelay","6", "Вкл при старте", false, AC_Behind , AC_Tag_BR);
#endif
#if MQTT_USE
AutoConnectCheckbox CtrlChbUseMQTT("ChbUseMQTT","7", "MQTT", false, AC_Behind , AC_Tag_DIV);
ACInput(SetMQTT_server,"", "сервер"); 
ACInput(SetMQTT_port,"", "порт", "",  "", AC_Tag_BR, AC_Input_Number, STYLE_WIDTH); 
ACInput(SetMQTT_user,"", "user"); 
ACInput(SetMQTT_pwd,"", "pwd"); 
ACInput(SetMQTT_topic,"", "топик"); 
ACInput(SetMQTT_devname,"", "имя устройства"); 
ACInput(SetMQTT_interval,"", "интервал, сек", "",  "Введи интервал",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH); 
#endif // MQTT_USE
ACInput(SetTmaxPID,"", "Tmax:","","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH); // 
ACInput(SetTminPID,"", "Tmin:","","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH); // 
AutoConnectCheckbox CtrlChB_UseRemoteControl("CtrlChB5","5", "Разрешить удаленное управление", false, AC_Behind , AC_Tag_DIV);
  
//AutoConnectCheckbox checkbox("checkbox", "uniqueapid", "Use APID unique", false);
//ACCheckbox(CtrlChB2,"a2", "", true,  AC_Behind , AC_Tag_DIV);
ACSubmit(ApplyChB, "Задать", SET_PAR_URI, AC_Tag_DIV);
ACSubmit(ApplyAdd, "Дополнительно", SETUP_ADD_URI, AC_Tag_None);


/************* SetupAdditionPage for MConfigMMemberIDcode ***************/
AutoConnectCheckbox UseID2ChB("UseID2ChB","", "Использовать OT ID2", false, /* AC_Infront */  AC_Behind , AC_Tag_None);
ACInput(ID2MaserID,"", "IDcode","", "", AC_Tag_BR, AC_Input_Text, STYLE_WIDTH); 

AutoConnectCheckbox UseOTC_ChB("UseOTC_ChB","", "Использовать OTC (ID0:HB3)",         false,   AC_Behind, AC_Tag_BR);
AutoConnectCheckbox UseCH2_DHW_ChB("UseCH2DHW","", "Использовать CH2 для горячей воды (ID0:HB4)", false, AC_Behind, AC_Tag_BR);
AutoConnectCheckbox UseWinterModeChB("UseWinterModeChB","", "Режим «лето/зима» (ID0:HB5)", false,   AC_Behind, AC_Tag_BR);
AutoConnectCheckbox UseID29_DHW_ChB("UseID29DHW","", "Использовать ID29 для температуры бойлера", false, AC_Behind, AC_Tag_BR);
AutoConnectCheckbox Immergas_fix_ChB("Immergas","", "Immergas fix", false, AC_Behind, AC_Tag_BR);
ACSubmit(ApplyAddpar,   "Задать", SET_ADD_URI, AC_Tag_BR);
ACSubmit(SendBLOR, "Сброс ошибки", BLOR_URI, AC_Tag_BR);

#if PID_USE
ACSubmit(SetupPID,   "PID", PID_URI, AC_Tag_BR);
AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false, { UseID2ChB, ID2MaserID,  UseOTC_ChB, UseCH2_DHW_ChB, UseWinterModeChB, UseID29_DHW_ChB,Immergas_fix_ChB, ApplyAddpar, SetupPID, Info1, SendBLOR });
#else
AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false, { UseID2ChB, ID2MaserID,  UseOTC_ChB, UseCH2_DHW_ChB, UseWinterModeChB, ApplyAddpar, Info1, SendBLOR});
#endif //#if PID_USE


/************* SetPID ******************/
#if PID_USE
AutoConnectCheckbox UsePID("UsePID","", "Использовать PID", false, AC_Behind , AC_Tag_BR);
AutoConnectCheckbox UsePID_NoLimit("UsePID_NOLIMIT","", "Не ограничивать уставку (5-35°C)", false, AC_Behind , AC_Tag_BR);
ACInput(SetXtagPID,"", "Уставка температуры в помещении:", "",  "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH); 
ACInput(SetTempSrcPID,"", "Источник температуры в помещении:", "",  "",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH); 
ACInput(SetTempExtSrcPID,"", "Источник температуры на улице:", "",  "у",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH); 
ACInput(SetKpPID,  "", "Kp:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);  
ACInput(SetKdPID,  "", "Kd:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH); 
ACInput(SetKiPID,  "", "Ki:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH); 
ACInput(SetIdissPID,"","Idiss:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH); 

ACInput(Set_u0_PID,"", "u0:",  "","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH); // 
ACInput(Set_t0_PID,"", "t0:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH); // 
ACInput(Set_u1_PID,"", "u1:",  "","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH); // 
ACInput(Set_t1_PID,"", "t1:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH); // 
ACInput(Set_CH_GIST,"", "Гистерезис включения горелки, град:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH); // 



ACSubmit(ApplyPID,   "Задать", SET_PID_URI, AC_Tag_BR);
AutoConnectAux PID_Page(PID_URI, "PID", true, {UsePID, UsePID_NoLimit, SetXtagPID, Info1, SetTempSrcPID, SetTempExtSrcPID, 
                      SetKpPID, SetKdPID, SetKiPID,SetIdissPID, 
                      Info3, Set_u0_PID, Set_t0_PID, Set_u1_PID,Set_t1_PID, Info4, Set_CH_GIST, Info5, Info6,  ApplyPID });  // onSetupPID()
#endif
/************* SetPID end ***************/

#if ST_VERS == 2
AutoConnectCheckbox UseOTslave("UseOTslave","", "Использовать OT slave интерфейс", false, AC_Behind ,  AC_Tag_DIV);
AutoConnectRadio OTslaveMode("radio", { "SmartTherm", "Панель" }, "Котлом управляет:", AC_Vertical, 1,  AC_Tag_DIV);
ACSubmit(ApplySlave,   "Задать", SET_OT2_URI, AC_Tag_BR);
AutoConnectAux OTslave_Page(OT2_URI, "OT2", true, {Info1, UseOTslave, OTslaveMode, Info5, Info6, ApplySlave}); //onSetupOT_slave()
AutoConnectAux SetOTslave_Page(SET_OT2_URI, "SetOT2", false, {}, false); //onSetOT_slave()

#endif //

/************* debugPage( ****************/
ACSubmit(DebugApply, "Обновить", DEBUG_URI, AC_Tag_DIV);
/************* AboutPage *****************/
ACText(About_0, "<b>About:</b>", "", "", AC_Tag_DIV);
/*****************************************/

// AutoConnectAux for the custom Web page.

#if RELAY_USE
AutoConnectAux InfoPage(INFO_URI, "SmartTherm", true, { Caption, Info1, Info2, Info3, Info4, Info5, Info6, Info7, RelayOmFf,  Apply, SetBoilerTemp, SetDHWTemp, SetBoilerTemp2, SetNewBoilerTemp });
#else
AutoConnectAux InfoPage(INFO_URI, "SmartTherm", true, { Caption, Info1, Info2, Info3, Info4, Info5, Info6, Info7,  Apply, SetBoilerTemp, SetDHWTemp, SetBoilerTemp2, SetNewBoilerTemp });
#endif 

#if MQTT_USE
  AutoConnectAux Setup_Page(SETUP_URI, "Setup", true, { Ctrl2,  CtrlChB1, CtrlChB2, CtrlChB3, SetMaxMod, CtrlChBMmod, SetTmaxPID, SetTminPID, Info2,
  #if RELAY_USE
CtrlChBUseRelay, CtrlChBStartRelaySts,
  #endif
     CtrlChbUseMQTT, SetMQTT_user, SetMQTT_pwd, SetMQTT_server, SetMQTT_port, SetMQTT_topic, SetMQTT_devname, SetMQTT_interval, CtrlChB_UseRemoteControl, ApplyAdd,ApplyChB});
#else
AutoConnectAux Setup_Page(SETUP_URI, "Setup", true, { Ctrl2, CtrlChB1, CtrlChB2, CtrlChB3, CtrlChBMmod, SetTmaxPID, SetTminPID, Info2,
  #if RELAY_USE
 CtrlChBUseRelay,
  #endif
  CtrlChBStartRelaySts, CtrlChB_UseRemoteControl,  ApplyAdd, ApplyChB});  
#endif // MQTT_USE


AutoConnectAux SetTempPage(SET_T_URI, "SetTemp", false, {}, false);
AutoConnectAux SetParPage(SET_PAR_URI, "SetPar", false, {}, false);
AutoConnectAux SetAddParPage(SET_ADD_URI, "SetAdd", false, {}, false);
#if PID_USE
AutoConnectAux SetPIDPage(SET_PID_URI, "SetPID", false, {}, false);
#endif
AutoConnectAux SetRelayPage(RELAY_URI, "SetRelay", false, {}, false);
AutoConnectAux SendBLORPage(BLOR_URI, "SendBlor", false, {}, false); //onSendBlor()

AutoConnectAux debugPage(DEBUG_URI, "Debug", true, {Info1, Info2, Info3, Info4, Info5, Info6, Info7,  DebugApply});
AutoConnectAux AboutPage(ABOUT_URI, "About", true, { About_0, Info1, Info2, Info3});

AutoConnectConfig config;
AutoConnect portal;


/************************************/
//int test_fs(void);

void setup_web_common(void);
void check_fs(void);
int setup_web_common_onconnect(void);
void loop_web(void);
void onRoot(void);
void loadParam(String fileName);
void onConnect(IPAddress& ipaddr);
#if MQTT_USE
  extern void mqtt_setup(void);
  extern void mqtt_loop(void);
  extern void mqtt_start(void);
  extern int MQTT_pub_usePID(void);
#endif
String onInfo(AutoConnectAux& aux, PageArgument& args);
String on_Setup(AutoConnectAux& aux, PageArgument& args);
String on_SetupAdd(AutoConnectAux& aux, PageArgument& args);
String onSetTemp(AutoConnectAux& aux, PageArgument& args);
String onSetPar(AutoConnectAux& aux, PageArgument& args);
String onSetAddPar(AutoConnectAux& aux, PageArgument& args);
String onDebug(AutoConnectAux& aux, PageArgument& args);
String onAbout(AutoConnectAux& aux, PageArgument& args);
String onSendBlor(AutoConnectAux& aux, PageArgument& args);

#if PID_USE
String onSetupPID(AutoConnectAux& aux, PageArgument& args);
String onSetPID(AutoConnectAux& aux, PageArgument& args);
#endif

#if RELAY_USE
String onSetRelay(AutoConnectAux& aux, PageArgument& args);
#endif

#if ST_VERS == 2
String onSetupOT_slave(AutoConnectAux& aux, PageArgument& args);
String onSetOT_slave(AutoConnectAux& aux, PageArgument& args);
#endif


String utc_time_jc;

/************************************/
unsigned int /* AutoConnect:: */ _toWiFiQuality(int32_t rssi);


void setup_web_common(void)
{    bool b;

//  Serial.println();
//   Serial.println("setup_web_common");

  {  char str[40];
     sprintf(str,"%.1f",SmOT.Tset);
     SetBoilerTemp.value = str;
     sprintf(str,"%.1f",SmOT.TdhwSet);
     SetDHWTemp.value = str;

  #if MQTT_USE
     SetMQTT_server.value = SmOT.MQTT_server;
     SetMQTT_user.value = SmOT.MQTT_user;
     SetMQTT_pwd.value = SmOT.MQTT_pwd;
     sprintf(str,"%d",SmOT.MQTT_port);
     SetMQTT_port.value = str;

     SetMQTT_topic.value = SmOT.MQTT_topic;
     sprintf(str,"%d",SmOT.MQTT_interval);
     SetMQTT_interval.value = str;
     SetMQTT_devname.value = SmOT.MQTT_devname;
  #endif

  }

  InfoPage.on(onInfo);      // Register the attribute overwrite handler.
  Setup_Page.on(on_Setup);
  SetTempPage.on(onSetTemp);
  SetParPage.on(onSetPar);
  SetupAdd_Page.on(on_SetupAdd);
  SetAddParPage.on(onSetAddPar);
#if RELAY_USE  
  SetRelayPage.on(onSetRelay);
#endif

#if ST_VERS == 2
  OTslave_Page.on(onSetupOT_slave);
  SetOTslave_Page.on(onSetOT_slave);
//AutoConnectCheckbox UseOTslave("UseЩOTslave","", "Использовать OT slave интерфейс", false, AC_Behind ,  AC_Tag_DIV);
//AutoConnectAux OTslave_Page(OT2_URI, "OT2", true, {Info1, UseOTslave, Info5, Info6});
#endif //

#if PID_USE
  PID_Page.on(onSetupPID);
  SetPIDPage.on(onSetPID);
#endif  
  SendBLORPage.on(onSendBlor);
  debugPage.on(onDebug);
  AboutPage.on(onAbout);
/**/  
#if MQTT_USE
  #if PID_USE
    portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, 
               SetAddParPage, PID_Page, SetPIDPage, debugPage,  AboutPage});     // Join pages.
  #else
    portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, 
               SetAddParPage, debugPage,  AboutPage});     // Join pages.
  #endif      
#else
  #if PID_USE
    portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, SetAddParPage, PID_Page, SetPIDPage, debugPage,  AboutPage});     // Join pages.
  #else
    portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, SetAddParPage, debugPage,  AboutPage});     // Join pages.
  #endif  
#endif

#if RELAY_USE  
    portal.join({SetRelayPage});
#endif
    portal.join({SendBLORPage});
#if ST_VERS == 2
    portal.join({OTslave_Page,SetOTslave_Page});
#endif    

//  portal.join({InfoPage, Setup_Page, SetTempPage});     // Join pages.
  config.ota = AC_OTA_BUILTIN;
  config.portalTimeout = 1; 
  config.retainPortal = true; 
  config.autoRise = true;
  //config.hostName = AUTOCONNECT_APID;
  // Enable saved past credential by autoReconnect option,
  // even once it is disconnected.
  config.autoReconnect = true;
  config.reconnectInterval = 2; //1;
  config.menuItems = config.menuItems | AC_MENUITEM_DELETESSID;
   Serial.printf("WiFi psk=%s\n", config.psk.c_str());
  
  portal.config(config);
  portal.onConnect(onConnect);  // Register the ConnectExit function
  portal.begin();

  WiFiWebServer&  webServer = portal.host();

  webServer.on("/", onRoot);  // Register the root page redirector.
//  Serial.println("Web server started:" +WiFi.localIP().toString());
  if (WiFi.status() != WL_CONNECTED)  {
    Serial.println(F("WiFi Not connected"));
    WiFi.setAutoReconnect(true);
  }  
  
/* get my MAC*/
#if defined(ARDUINO_ARCH_ESP8266)
//    WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3
    if(WiFi.getMode() == WIFI_OFF)
    {
      wifi_get_macaddr(STATION_IF, SmOT.Mac);

    } else {
      wifi_get_macaddr(STATION_IF, SmOT.Mac);
    }
//    Serial.printf("MAC: %02x %02x %02x %02x %02x %02x\n",SmOT.Mac[0],SmOT.Mac[1],SmOT.Mac[2],SmOT.Mac[3],SmOT.Mac[4],SmOT.Mac[5]);
#elif defined(ARDUINO_ARCH_ESP32)
    if(WiFi.getMode() == WIFI_MODE_NULL){
        esp_read_mac(SmOT.Mac, ESP_MAC_WIFI_STA);
//      Serial.printf( "2 MAC NULL %02x %02x %02x %02x %02x %02x\n", SmOT.Mac[0], SmOT.Mac[1], SmOT.Mac[2], SmOT.Mac[3], SmOT.Mac[4], SmOT.Mac[5]);
    }
    else{
        esp_wifi_get_mac(WIFI_IF_STA, SmOT.Mac);
//      Serial.printf( "2 MACL %02x %02x %02x %02x %02x %02x\n", SmOT.Mac[0], SmOT.Mac[1], SmOT.Mac[2], SmOT.Mac[3], SmOT.Mac[4], SmOT.Mac[5]);
    }  
#endif //
//  Serial.printf("(20) %d\n", millis());

}

int setup_web_common_onconnect(void)
{ static int init = 0;

  //Serial.printf("setup_web_common_onconnect init %d\n", init);

  Serial.print(F("WiFi connected, IP address: "));
  Serial.println(WiFi.localIP());
  sprintf(SmOT.LocalUrl,"http://%s", WiFi.localIP().toString().c_str());
  Serial.printf("WiFi mode = %d\n", WiFi.getMode());
  if(init)
    return 1;

   WiFi.setAutoReconnect(true);

/****************************************************/    
{

 // Specifying the time zone and assigning NTP.
// Required to add the correct local time to the export file name of the
// captured image. This assignment needs to be localized.
// This sketch works even if you omit the NTP server specification. In that
// case, the suffix timestamp of the captured image file is the elapsed time
// since the ESP module was powered on.
const char*  const _ntp1 = "europe.pool.ntp.org";
const char*  const _ntp2 = "pool.ntp.org";

#if SERIAL_DEBUG      
	  time_t  now;
  now = time(nullptr);
  Serial.printf("1 %s", ctime(&now));
#endif 
   // By configuring NTP, the timestamp appended to the capture filename will
    // be accurate. But this procedure is optional. It does not affect ESP32Cam
    // execution.
//    configTzTime(_tz, _ntp1 ,_ntp2);
    configTzTime("UTC0", _ntp1 ,_ntp2);

    //TZoffset
//   delay(1000);
#if SERIAL_DEBUG      
  now = time(nullptr);
  Serial.printf("2 %s\n", ctime(&now));
#endif  
  // uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 ()
#if defined(ARDUINO_ARCH_ESP8266)
// default ntp update  1 hour
// it can be redefined via uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 ()
#elif defined(ARDUINO_ARCH_ESP32)
  Serial.print("Sync time in ms: ");
  Serial.println(sntp_get_sync_interval());  
#endif


#if MQTT_USE

   SmOT.Read_mqtt_fs();
//     mqtt_setup();
//mqtt_setup is called from mqtt_loop()
#endif
}
/****************************************************/

  init  = 1;
  return 0;
}


void onConnect(IPAddress& ipaddr) 
{ int rc;
  rc = setup_web_common_onconnect();
  if(rc)
  {
#if SERIAL_DEBUG      
  Serial.print(F("onConnect:WiFi connected with "));
  Serial.print(WiFi.SSID());
  Serial.print(F(", IP:"));
  Serial.println(ipaddr.toString());
#endif  
  }
}

// Redirects from root to the info page.
void onRoot() {
  WiFiWebServer&  webServer = portal.host();
  webServer.sendHeader("Location", String("http://") + webServer.client().localIP().toString() + String(INFO_URI));
  webServer.send(302, "text/plain", "");
  webServer.client().flush();
  webServer.client().stop();
}

float mRSSi = 0.;
int WiFists = -1;

int OutUTCtime(time_t now);

#include "esp32/rom/rtc.h"

String onDebug(AutoConnectAux& aux, PageArgument& args)
{  char str[180];
  // int l;
extern int minRamFree;

//WiFiDebugInfo
//   sprintf(str,"WiFi statistics:");
//   Info1.value = str;
   Info1.value = F("WiFi statistics:");
   sprintf(str,(PGM_P)F("%d %d  %d %d  %d %d  %d %d"), 
      WiFiDebugInfo[0],WiFiDebugInfo[1],WiFiDebugInfo[2],WiFiDebugInfo[3],WiFiDebugInfo[4],WiFiDebugInfo[5],WiFiDebugInfo[6],WiFiDebugInfo[7]);
   Info2.value = str;
   if(WiFists == WL_CONNECTED)
   {  sprintf(str,(PGM_P)F("RSSI: %d dBm (%i%%), среднее за 10 мин %.1f"),WiFi.RSSI(),_toWiFiQuality(WiFi.RSSI()), mRSSi);
      Info3.value = str;
   } else 
      Info3.value = "";
   sprintf(str,(PGM_P)F("OpenTherm statistics:<br>%d %d  %d %d  % d %d  %d %d  %d %d  %d"), 
      OTDebugInfo[0], OTDebugInfo[1], OTDebugInfo[2], OTDebugInfo[3], OTDebugInfo[4], OTDebugInfo[5], OTDebugInfo[6],OTDebugInfo[7], OTDebugInfo[8],OTDebugInfo[9], OTDebugInfo[10]);
   //l = strlen(str);
   //Serial.printf("4 l=%d\n", l);

   Info4.value = str;
#if ST_VERS == 2
    if(SmOT.OT_slave_present && (SmOT.OT_slave_mode == 1))
    { extern int OTslaveDebugInfo[12];      
      sprintf(str,(PGM_P)F("<br>OT2: %d %d  %d %d  %d "), 
      OTslaveDebugInfo[0], OTslaveDebugInfo[1], OTslaveDebugInfo[2], OTslaveDebugInfo[3], OTslaveDebugInfo[4]);
      Info4.value += str;
    }

#endif      

   sprintf(str,(PGM_P)F("min free RAM %d"), minRamFree);
   Info5.value = str;
  
  {       
   OutUTCtime(time(nullptr));

    Info5.value += (PGM_P)F("<br>Время:");
    Info5.value += utc_time_jc;

  }

   sprintf(str,(PGM_P)F("Вкл горелки:<br>Всего %d<br>За час %d<br>Пред.час %d<br>Сутки %d<br>Пред.сутки %d"), 
          SmOT.Bstat.NflameOn, SmOT.Bstat.NflameOn_h, SmOT.Bstat.NflameOn_h_prev, SmOT.Bstat.NflameOn_day, SmOT.Bstat.NflameOn_day_prev);
//   l = strlen(str);
//   Serial.printf("5 l=%d\n", l);
   
   Info6.value = str;
   sprintf(str,(PGM_P)F("<br>Эффективная модуляция:<br>За час %.2f<br>Пред.час %.2f<br>Сутки %.2f<br>Пред.сутки %.2f"), 
          SmOT.Bstat.Eff_Mod_h, SmOT.Bstat.Eff_Mod_h_prev, SmOT.Bstat.Eff_Mod_d, SmOT.Bstat.Eff_Mod_d_prev);

//   l = strlen(str);
//   Serial.printf("6 l=%d\n", l);

   Info6.value += str;
#if PID_USE
    if(SmOT.usePID)
    {  extern int debcode;
       extern int wait_if_takt;

       sprintf(str,"<br>pid: U= %f u0 = %f  dP=%f, dD=%f dI=%f\n",
        SmOT.mypid.u, SmOT.mypid.ub, SmOT.mypid.dP, SmOT.mypid.dD, SmOT.mypid.dI); 

//      sprintf(str,"<br>debcode %d wait_if_takt %d",  debcode, wait_if_takt); 

      Info6.value += str;

      sprintf(str,"<br>RoomSetpoint change src %d from %f to %f at ", 
      SmOT.src_lastSetPointChange, SmOT.oldTroomSetpoint, SmOT.mypid.xTag); 
      Info6.value += str;
//    t_lastSetPointChange = time(nullptr);
{
    struct tm* tm_info;
  tm_info = localtime(&SmOT.t_lastSetPointChange);

      strftime(str, 26, "%Y-%m-%d %H:%M:%S", tm_info);  
      str[25] = 0;

}

      Info6.value += str;
      Info6.value += " UTC";

    }
#endif   
  //https://docs.espressif.com/projects/arduino-esp32/en/latest/api/reset_reason.html
      sprintf(str,"reset reason: %d %d", rtc_get_reset_reason(0), rtc_get_reset_reason(1));
  Info7.value = str;
#if 0   
   {  int i;
      extern char ot_data_used[60];
      extern int ot_nids;

      for(i=0;i<ot_nids; i++)
      {    sprintf(str,"%d ", ot_data_used[i]); 
        if(i == 0)
           DebugInfo7.value = str;
        else
           DebugInfo7.value += str;
        if(i > 0 && (i%10 == 9))
            DebugInfo7.value += "<br>";
      }
   }
#endif //0   
  return String();
}

String onSetTemp(AutoConnectAux& aux, PageArgument& args)
{  float  v;
   int isChange=0;

    if(SmOT.enable_CentralHeating)
    { if(SetBoilerTemp.enable)
      { v = SmOT.CHtempLimit(SetBoilerTemp.value.toFloat());
        if(v != SmOT.Tset)
        { isChange = 1;
          SmOT.Tset = v;
          SmOT.need_set_T = 1;
        } 
      }
    }

    if(SmOT.enable_HotWater)
    { v = SmOT.CHtempLimit(SetDHWTemp.value.toFloat());    
      if(v != SmOT.TdhwSet)
      { isChange = 1;
        SmOT.TdhwSet = v;
        SmOT.need_set_dhwT = 1;
      }
    }

    if(SmOT.enable_CentralHeating2)
    { v = SmOT.CHtempLimit(SetBoilerTemp2.value.toFloat());
      if(v != SmOT.Tset2) 
      {  isChange = 1;
         SmOT.Tset2 = v;
         SmOT.need_set_T2 = 1;
      }
    }

    if(isChange)
        SmOT.need_write_f = 1;

// redirect/transition to the INFO_URI.
//work only with last false in
//AutoConnectAux SetTempPagee(SET_PAR_URI, "SetTempPage", false, {}, false);
  aux.redirect(INFO_URI);

  return String();
}

// goes here from on_Setup
String onSetPar(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0,  redir = 0, v;
   bool check;

  if( CtrlChB1.checked) check = true;
  else                  check = false;
  if(check != SmOT.enable_CentralHeating)
  { isChange++;
    SmOT.enable_CentralHeating = check;
#if PID_USE
    if(SmOT.usePID && !SmOT.enable_CentralHeating)
    {   
        SmOT.usePID = 0;
    }
#endif // PID_USE 

  }

  if( CtrlChB2.checked) check = true;
  else                  check = false;
  if(check != SmOT.enable_HotWater)
  { isChange++;
    SmOT.enable_HotWater = check;
  }
  
  if(SmOT.CH2_present) 
  { if( CtrlChB3.checked) check = true;
    else                  check = false;
    if(check != SmOT.enable_CentralHeating2)
    { isChange++;
      SmOT.enable_CentralHeating2 = check;
    }
  }

  if( CtrlChB_UseRemoteControl.checked) check = true;
  else                                  check = false;
  if(check != SmOT.Use_remoteTCPserver)
  { isChange++;
    SmOT.Use_remoteTCPserver = check;
    SmOT.init(2);
  }

  v = SmOT.CHtempLimit(SetTmaxPID.value.toFloat());    
  if(v != SmOT.umax)
  { SmOT.umax = v;
    SmOT.need_set_MaxTSet = 2;    
    isChange = 1;
  }

  v = SmOT.CHtempLimit(SetTminPID.value.toFloat());    
  if( v > SmOT.umax - 1.)  v = SmOT.umax -1.;
  if(v != SmOT.umin)
  { SmOT.umin = v;
    isChange = 1;
  }

#if MQTT_USE
  int isChangeMQTT = 0;
  if( CtrlChbUseMQTT.checked) check = true;
  else                  check = false;

  if(check)
  { if(SmOT.useMQTT == 0)
    { SmOT.useMQTT = 1;
      redir = 1;  
    } else if(SmOT.useMQTT == 1) { 
      SmOT.useMQTT = 0x3;
      isChangeMQTT++;
    }
  } else {
    if(SmOT.useMQTT != 0)
    { SmOT.useMQTT = 0;
      isChangeMQTT++;
    }
  }

   if(SmOT.useMQTT && redir== 0)
   {   char str0[80];
      int i;
 
    SetMQTT_server.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_server,str0))
    {  isChangeMQTT++;
       strcpy(SmOT.MQTT_server,str0);      
    }

    SetMQTT_user.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_user,str0))
    { isChangeMQTT++;
       strcpy(SmOT.MQTT_user,str0);      
    }
    SetMQTT_pwd.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_pwd,str0))
    { isChangeMQTT++;
       strcpy(SmOT.MQTT_pwd,str0);      
    }

    SetMQTT_devname.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_devname,str0))
    { isChangeMQTT++;
       strcpy(SmOT.MQTT_devname,str0);      
    }

    SetMQTT_topic.value.toCharArray(str0, sizeof(str0));
    /* check for [a-zA-Z0-9_-] */
    for(i=0; str0[i]; i++)
    {  if(str0[i]>='0' && str0[i]<='9' ) continue;
       if(str0[i]>='A' && str0[i]<='Z' ) continue;
       if(str0[i]>='a' && str0[i]<='z' ) continue;
       if(str0[i] =='_'  ) continue;
       if(str0[i] =='-'  ) continue;
       str0[i] = 0;
       break;
    }
    if(strcmp(SmOT.MQTT_topic,str0))
    { isChangeMQTT++;
       strcpy(SmOT.MQTT_topic,str0);      
    }

    v = SetMQTT_interval.value.toInt();
    if((unsigned int)v !=SmOT.MQTT_interval )
    { isChangeMQTT++;
       SmOT.MQTT_interval = v;
    }

    v = SetMQTT_port.value.toInt();
    if((unsigned int)v !=SmOT.MQTT_port )
    { isChangeMQTT++;
       SmOT.MQTT_port = v;
    }

   }

#endif //MQTT_USE

#if RELAY_USE
  if( CtrlChBUseRelay.checked) check = true;
  else                         check = false;
  if(check != SmOT.Relay_present)
  { isChange++;
    if(check && CtrlChBStartRelaySts.enable == false)
       redir = 1; 
    SmOT.Relay_present = check;
  }  
  if(CtrlChBStartRelaySts.enable == true)
  { if(CtrlChBStartRelaySts.checked) check = true;
    else                             check = false;
    if(check != SmOT.Relay_init_sts)
    {  isChange++;
       SmOT.Relay_init_sts = check;
    }
  }

#endif

  if(SmOT.MaxRelModLevel_present)
  {   if(CtrlChBMmod.checked) check = true;
      else                    check = false;
      if(SmOT.Use_MaxRelModLevel)
      { if(!check)
        {   SmOT.Use_MaxRelModLevel = 0;
            isChange++; 
        } else {
            v = SetMaxMod.value.toInt();
            if(v != int(SmOT.MaxRelModLevelSetting+0.5))
            {   isChange++;
                SmOT.MaxRelModLevelSetting = (float)v;
                SmOT.need_set_MaxRelModLevel = 2;
            }
        }
      } else {
        if(check)
        {   SmOT.Use_MaxRelModLevel = 1;
            redir = 1;

        } else {
            SmOT.Use_MaxRelModLevel = 0;
        }
      }
  } 

  if(isChange)
        SmOT.need_write_f = 1;  //need write changes to FS


#if MQTT_USE
    if(isChangeMQTT)
    { if(SmOT.useMQTT == 0x03)
            mqtt_start();
        SmOT.need_write_f |= 0x2;  //need write changes to FS
    }
#endif //MQTT_USE

    if(SmOT.enable_CentralHeating) //Отопление Вкл
    {     SmOT.need_set_T = 1;
    } else {
        //Отопление вЫкл
    }

    if(SmOT.HotWater_present)
    { if(SmOT.enable_HotWater) //Горячая вода Вкл
      {   SmOT.need_set_dhwT = 1;
      } else {
         //Горячая вода вЫкл
      }
    }

    if(SmOT.CH2_present)
    { if(SmOT.enable_CentralHeating2) //CentralHeating2 Вкл
      { SmOT.need_set_T2 = 1;
      } else {
        //CentralHeating2 вЫкл
      }
    }

// redirect/transition to the INFO_URI.
//work only with last false in
//AutoConnectAux SetParPage(SET_PAR_URI, "SetPar", false, {}, false);

  if(redir)
    aux.redirect(SETUP_URI);
  else
    aux.redirect(INFO_URI);

  return String();
}

// SetAddParPage 
String onSetAddPar(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0;
   unsigned short int icheck;
   unsigned short int v2;

  if( UseID2ChB.checked) icheck = 1;
  else                   icheck = 0;
  if(icheck != SmOT.UseID2)
  { isChange++;
     SmOT.UseID2 = icheck;
  }

  if(UseWinterModeChB.checked)  icheck = 1;
  else                          icheck = 0;
  if(icheck != SmOT.UseWinterMode)
  { isChange++;
     SmOT.UseWinterMode = icheck;
  }

  if(UseOTC_ChB.checked)  icheck = 1;
  else                          icheck = 0;
  if(icheck != SmOT.Use_OTC)
  { isChange++;
     SmOT.Use_OTC = icheck;
  }

  v2 = ID2MaserID.value.toInt();
  if(v2 != SmOT.ID2masterID )
  { isChange++;
    SmOT.ID2masterID = v2;
  }

  if( UseCH2_DHW_ChB.checked) icheck = 1;
  else                   icheck = 0;
  if(icheck != SmOT.CH2_DHW_flag)
  { isChange++;
     SmOT.CH2_DHW_flag = icheck;
  }

  if(UseID29_DHW_ChB.checked) icheck = 1;
  else                        icheck = 0;
  if(icheck != SmOT.Use_ID29_DHW_flag)
  { isChange++;
     SmOT.Use_ID29_DHW_flag = icheck;
  }
  if(Immergas_fix_ChB.checked) icheck = 1;
  else                        icheck = 0;
  if(icheck != SmOT.Immergas_fix_flag)
  { isChange++;
     SmOT.Immergas_fix_flag = icheck;
  }


  if(isChange)
        SmOT.need_write_f = 1;  //need write changes to FS

  aux.redirect(SETUP_URI);
  return String();
}

String on_SetupAdd(AutoConnectAux& aux, PageArgument& args)
{  char str[40];

  if( SmOT.UseID2)
      UseID2ChB.checked = true;
  else
      UseID2ChB.checked = false;
    
  if(SmOT.UseWinterMode)
      UseWinterModeChB.checked = true;
  else
      UseWinterModeChB.checked = false;

  if(SmOT.Use_OTC)
      UseOTC_ChB.checked = true;
  else
      UseOTC_ChB.checked = false;

  sprintf(str,"%d",SmOT.ID2masterID);
  ID2MaserID.value = str;

  if( SmOT.CH2_DHW_flag)
      UseCH2_DHW_ChB.checked = true;
  else
      UseCH2_DHW_ChB.checked = false;

  if( SmOT.Use_ID29_DHW_flag)
      UseID29_DHW_ChB.checked = true;
  else
      UseID29_DHW_ChB.checked = false;

  if( SmOT.Immergas_fix_flag)
      Immergas_fix_ChB.checked = true;
  else
      Immergas_fix_ChB.checked = false;

  if(SmOT.RemoteRequest_present)
  {   SendBLOR.enable = true;
      Info1.value = "Удаленный сброс ошибки (BLOR), я знаю, что я делаю";

  } else {
     SendBLOR.enable = false;
  }


  return String();
}

// Main info page
String onInfo(AutoConnectAux& aux, PageArgument& args) {
  char str0[80];
extern OpenTherm ot;

   switch(SmOT.stsOT)
   {  case -1:
        Info1.value =  String(SmOT.stsOT) + ": <b>Ошибка:</b> OT не инициализирован";
        SetDHWTemp.enable = false;
        SetBoilerTemp.enable = false;
        SetBoilerTemp2.enable = false;
        SetNewBoilerTemp.enable = false;
        break;
      case 0:
      {  char str[40];
        sprintf(str," (%8x)",SmOT.BoilerStatus );
        Info1.value =  String(SmOT.stsOT) +  str;
        /* Если статус ответа не соответсвует статусу запроса, возможно у котла режим readonly  */
        if((SmOT.BoilerStatus&0xff00) != (SmOT.BoilerStatusRequest&0xff00)) 
        {    sprintf(str," (Запрос: %8x)",SmOT.BoilerStatusRequest );
             Info1.value +=  str;
        }

        if(SmOT.BoilerStatus & 0x01)
          Info1.value += "<br>Ошибка";
        if(SmOT.BoilerStatus & 0x02)
          Info1.value += "<br>Отопление Вкл";
        else  
          Info1.value += "<br>Отопление вЫкл";

        if(SmOT.HotWater_present)
        { if(SmOT.BoilerStatus & 0x04)
            Info1.value += "<br>Горячая вода Вкл";
          else  
            Info1.value += "<br>Горячая вода вЫкл";
        }

        if(SmOT.BoilerStatus & 0x08)
          Info1.value += "<br>Горелка Вкл";
        else  
          Info1.value += "<br>Горелка вЫкл";

        if(SmOT.CH2_present && SmOT.enable_CentralHeating2)
        {
          if(SmOT.BoilerStatus & 0x20)
            Info1.value += "<br>CH2 Вкл";
          else  
            Info1.value += "<br>CH2 вЫкл";
        }

        if(SmOT.BoilerStatus & 0x40)
          Info1.value += "<br>Diag";

          if(SmOT.BoilerStatus & 0xff00)
          { Info1.value += "<br><small>Уставки:";
            if(SmOT.BoilerStatus & 0x0100)
              Info1.value += " CH";
            if(SmOT.BoilerStatus & 0x0200)
              Info1.value += " DHW";
            if(SmOT.BoilerStatus & 0x0400)
              Info1.value += " Cool";
            if(SmOT.BoilerStatus & 0x0800)
              Info1.value += " OTC";
            if(SmOT.BoilerStatus & 0x1000)
              Info1.value += " CH2";
            if(SmOT.BoilerStatus & 0x2000)
              Info1.value += " Summer";
            if(SmOT.BoilerStatus & 0x4000)
              Info1.value += " DHWblocking";
              Info1.value += "</small>";
          }
      }
        break;
      case 1:
       Info1.value =  String(SmOT.stsOT) + " Invalid response";
        break;
      case 2:
      {  time_t now = time(nullptr);
        double dt;
        dt = difftime(now,SmOT.t_lastwork);
        if(dt < 3600.)
        {   sprintf(str0, (PGM_P)F("Потеря связи с котлом %.f сек назад"), dt);

        } else {        
            sprintf(str0, (PGM_P)F("Потеря связи связи с котлом %.1f час(ов) назад"), dt);
        }
        Info1.value =  String(SmOT.stsOT) + " : <b>Ошибка:</b> ";
        Info1.value +=  str0;
      }
        break;
   }

/***************************************/
#if MQTT_USE 
if(SmOT.useMQTT)
{  extern int statemqtt;
   extern int state_mqtt;
   Info1.value += "<br>";

  switch(statemqtt)
  {   case -1:
        Info1.value += "MQTT not connected";
        if(SmOT.stsMQTT == 0)
            Info1.value += ", ожидание опроса OT";
        break;
      case 0:
        Info1.value += "MQTT DiSconnected";
        break;
      case 1:
        Info1.value += "MQTT connected";
        break;
  }
  
  switch(state_mqtt)
  {  
      case -1:
        Info1.value += " disconnected";
        break;

      case -2:
        Info1.value += " Connect failed";
        break;

      case -3:
        Info1.value += " Connection lost";
        break;
      case -4:
        Info1.value += " Connection timeout";
        break;
      case 1:
        Info1.value += " Bad protocol";
        break;

      case 2:
        Info1.value += " Bad client id";
        break;

      case 3:
        Info1.value += " unavailable";
        break;

      case 4:
        Info1.value += " Bad credentials";
        break;

      case 5:
        Info1.value += " Unauthorized";
        break;
  }
  
} else {
   if(SmOT.CapabilitiesDetected == 0)
          Info1.value += "<br>Тест котла";
}
#else 
   if(SmOT.CapabilitiesDetected == 0)
          Info1.value += "<br>Тест котла";
#endif // MQTT_USE 
#if PID_USE
    if(SmOT.usePID && SmOT.enable_CentralHeating)
    {   Info1.value += "<br>управление по PID";
        if(SmOT.usePID & 0x02)
              Info1.value += "без ограничений";
      Info1.value += " Tindoor " + String(SmOT.tempindoor) + " ";
      Info1.value += " Toutdoor " + String(SmOT.tempoutdoor);
    }
#endif // PID_USE 

/***************************************/

//  Serial.printf("Info1.value length=%i\n ", strlen(Info1.value.c_str()));

    if(SmOT.stsT1 >= 0 || SmOT.stsT2 >= 0)
    {   Info3.value = " Температура ";
        if(SmOT.stsT1 >= 0)
          Info3.value += "T1 " + String(SmOT.t1) + " ";
        if(SmOT.stsT2 >= 0)
          Info3.value += "T2 " + String(SmOT.t2) ;
        Info3.value += "<br>";
    } else {
        Info3.value = "";
    }
    if(ot.OTid_used(OpenThermMessageID::Toutside))
    {   Info3.value += "Text " + String(SmOT.Toutside) + "<br>";
    }

  if(SmOT.stsOT != -1)
  {
   Info2.value = " Выходная температура  "  + String(SmOT.BoilerT);
      if(ot.OTid_used(OpenThermMessageID::Tret))
      { Info2.value +=  " Обратка " + String(SmOT.RetT);
      }
      if(ot.OTid_used(OpenThermMessageID::Texhaust))
      { sprintf(str0," Выхлоп %.0f", SmOT.Texhaust);
        Info2.value +=  str0;
      }

      if(SmOT.Use_ID29_DHW_flag && ot.OTid_used(OpenThermMessageID::Tstorage))
      {      Info2.value +=  " Бойлер " + String(SmOT.Tstorage);
      } else  if(SmOT.HotWater_present) {
         if(SmOT.enable_HotWater && ot.OTid_used(OpenThermMessageID::Tdhw))
            Info2.value +=  " Горячая вода " + String(SmOT.dhw_t);
      }

      Info2.value += "<br>";

      Info4.value = "";
    
      if(ot.OTid_used(OpenThermMessageID::RelModLevel))
      {  Info4.value += " Flame "  + String(SmOT.FlameModulation) ;
      }

      if(ot.OTid_used(OpenThermMessageID::CHPressure))
      {
           Info4.value += " Pressure " + String(SmOT.Pressure);
      }
      Info4.value += "<br>";

      if(SmOT.enable_CentralHeating2)
      {  Info4.value += "T CH2 " +  String(SmOT.BoilerT2) + "<br>";
      }

// Info5.value = " MaxRelModLevel "  + String(SmOT.MaxRelModLevelSetting) + "<br>" + "Ts="+ String(SmOT.Tset) + "Tsr="+ String(SmOT.Tset_r) + "<br>";
   Info5.value = "Ts "+ String(SmOT.Tset) + " Tsr "+ String(SmOT.Tset_r) + "<br>";


    if(SmOT.OEMDcode || SmOT.Fault)
    {  sprintf(str0, "%x %x", SmOT.Fault, SmOT.OEMDcode);
//      Info5.value = "Fault = " + str0 + "<br>";
      Info6.value  = "";
      if(SmOT.Fault)
      { sprintf(str0, "Fault = %x (HB) %x (LB)<br>", (SmOT.Fault>>8)&0xff, (SmOT.Fault&0xff));
        Info6.value += str0;
        if(SmOT.Fault & 0xff00)
        {    if(SmOT.Fault & 0x0100)
                 Info6.value += " Service request";
             if(SmOT.Fault & 0x0200)
                 Info6.value += " Lockout-reset";
             if(SmOT.Fault & 0x0400)
                 Info6.value += " LowWater press";
             if(SmOT.Fault & 0x0800)
                 Info6.value += " Gas/flame fault";
             if(SmOT.Fault & 0x01000)
                 Info6.value += " Air press fault";
             if(SmOT.Fault & 0x02000)
                 Info6.value += " Water over-temp fault";
            if(SmOT.Fault & 0x00ff)
              Info6.value += " &";
        }
        if(SmOT.Fault & 0x00ff)
        {    sprintf(str0, (PGM_P)F(" OEM-specific fault/error cod = %d ( hex %x)"), (SmOT.Fault&0xff), (SmOT.Fault&0xff));
            Info6.value += str0;
        }
        Info6.value += "<br>";
      }
      if(SmOT.OEMDcode)
      {     sprintf(str0, (PGM_P)F("OEM-specific diagnostic/service code = %d  ( hex %x)<br>"), SmOT.OEMDcode, SmOT.OEMDcode);
            Info6.value += str0;
      }
    } else {
      Info6.value = "";
    }
//    Info7.value = " MinModLevel="  + String(SmOT.MinModLevel) + "<br>"  + " MaxCapacity="  + String(SmOT.MaxCapacity) + "<br>";
    Info7.value = "";

/******************************/  
#if PID_USE
    if(SmOT.enable_CentralHeating && !SmOT.usePID)
      SetBoilerTemp.enable = true;
#else
    if(SmOT.enable_CentralHeating)
      SetBoilerTemp.enable = true;
#endif      
    else 
      SetBoilerTemp.enable = false;

    if(SmOT.CH2_present && SmOT.enable_CentralHeating2)
        SetBoilerTemp2.enable = true;
    else 
      SetBoilerTemp2.enable = false;
    
    if( SmOT.enable_HotWater)
      SetDHWTemp.enable = true;
    else
      SetDHWTemp.enable = false;

    if( SmOT.enable_HotWater || SmOT.enable_CentralHeating||SmOT.enable_CentralHeating2)
        SetNewBoilerTemp.enable = true;
    else 
        SetNewBoilerTemp.enable = false;
    if(SetBoilerTemp.enable)
    { sprintf(str0,"%.1f",SmOT.Tset);
      SetBoilerTemp.value = str0;
    }
    if(SetDHWTemp.enable)
    { sprintf(str0,"%.1f",SmOT.TdhwSet);
      SetDHWTemp.value = str0;
    }
    if(SetBoilerTemp2.enable)
    { sprintf(str0,"%.1f",SmOT.Tset2);
      SetBoilerTemp2.value = str0;
    }

  } else {
        Info2.value = "";
        Info4.value = "";
        Info5.value = "";
        Info6.value = "";
        Info7.value = "";
  }
 
#if ST_VERS == 2

  Info7.value = "OT2: ";
  if(SmOT.OT_slave_present)
  {
    switch(SmOT.ot_slave_stsOT)
    {   case -2:
        case -1:
          Info7.value += "<b>Ошибка:</b> не инициализирован";
          break;
        case 0:
          Info7.value += "работает";
          break;
        case 2:
        {  time_t now = time(nullptr);
          double dt;
          dt = difftime(now,SmOT.ot_slave_t_lastwork);
          if(dt < 3600.)
          {   sprintf(str0, (PGM_P)F("Потеря связи %.f сек назад"), dt);

          } else {        
              sprintf(str0, (PGM_P)F("Потеря связи связи  %.1f час(ов) назад"), dt);
          }
          Info7.value +=  str0;
        }
          break;
    }

    if((SmOT.OT_slave_mode == 1) && (SmOT.ot_slave_stsOT == 0))
          Info7.value +=  ", управление от панели";
    else 
          Info7.value +=  ", управление от контроллера";

    if(SmOT.OT_slave_mode == 1)
    {   SetDHWTemp.enable = false;
        SetBoilerTemp2.enable = false;
        SetBoilerTemp.enable = false;
    }
  }

#endif

#if  RELAY_USE
  if(SmOT.Relay_present)
  {
      RelayOmFf.enable = true;
      if(SmOT.Relay_sts )
      { strcpy(str0,"Реле вЫкл");

      } else {
         strcpy(str0,"Реле Вкл");
      }
      RelayOmFf.value = str0;

  } else {
      RelayOmFf.enable = false;
  }
 
#endif

/********************/
  return String();
}

// SmOT.OTmemberCode
// see as well on_setpar()
String on_Setup(AutoConnectAux& aux, PageArgument& args)
{  const char *pstr; 
   char str[40]; 
    
#if RELAY_USE
    CtrlChBUseRelay.enable = true;
    if(SmOT.Relay_present)
    {   CtrlChBUseRelay.checked = true;
        CtrlChBStartRelaySts.enable = true;
        if(SmOT.Relay_init_sts)
            CtrlChBStartRelaySts.checked = true;
        else
            CtrlChBStartRelaySts.checked = false;
    } else {
        CtrlChBUseRelay.checked = false;
        CtrlChBStartRelaySts.enable = false;
    }
#endif

//  Serial.printf("SmOT.MaxRelModLevel_present =%d SmOT.Use_MaxRelModLevel %d\n ", SmOT.MaxRelModLevel_present, SmOT.Use_MaxRelModLevel);
   
  if(SmOT.MaxRelModLevel_present)
  {     CtrlChBMmod.enable = true;
        if(SmOT.Use_MaxRelModLevel)
        {   CtrlChBMmod.checked = true;
            SetMaxMod.enable = true;
            sprintf(str, "%d",int(SmOT.MaxRelModLevelSetting+0.5));
            SetMaxMod.value = str;           
        } else {
            CtrlChBMmod.checked = false;
            SetMaxMod.enable = false;
        }  
  } else {
       SetMaxMod.enable = false;
       CtrlChBMmod.enable = false;
  }

  if( SmOT.enable_CentralHeating)
      CtrlChB1.checked = true;
  else
      CtrlChB1.checked = false;
/*********************************/      
 if (SmOT.stsOT >= 0)
 {
  if(SmOT.HotWater_present) 
  { CtrlChB2.enable  =  true;    
    if(SmOT.enable_HotWater)
      CtrlChB2.checked = true;
    else
      CtrlChB2.checked = false;
  } else {
     CtrlChB2.enable  = false;
  }

  if(SmOT.CH2_present) 
     CtrlChB3.enable  = true;
  else
     CtrlChB3.enable  = false;

     Info2.value = "<small>Tmax <= 80, Tmin >= 30 (конденсационный котел, иначе 40)</small><br><br>";

     sprintf(str,"%.2f",SmOT.umax);
     SetTmaxPID.value = str;
     sprintf(str,"%.2f",SmOT.umin);
     SetTminPID.value = str;
         
  if(SmOT.Use_remoteTCPserver)
    CtrlChB_UseRemoteControl.checked = true;
  else
    CtrlChB_UseRemoteControl.checked = false;

  Info1.value ="";

  Ctrl2.value = "Котёл: "; 
  pstr = GetOTVendorName(SmOT.OTmemberCode);
  if(pstr)
  {   Ctrl2.value += pstr; 
  } else {
      Ctrl2.value +=  "код " + String(SmOT.OTmemberCode);
  }

  if(SmOT.DHW_tank_present) 
      Ctrl2.value +=  "\nбойлер косвенного нагрева";

/*********************************/      
 } else {
    CtrlChB2.enable  = false;
    CtrlChB3.enable  = false;
    Info1.value ="";
    Ctrl2.value = ""; 
 }
 
#if MQTT_USE
  CtrlChbUseMQTT.enable  = true;
  if(SmOT.useMQTT) 
  { if(SmOT.useMQTT == 1) 
      Info1.value = "проверь после Reset"; 
    CtrlChbUseMQTT.checked = true;
    SetMQTT_server.enable  = true;
    SetMQTT_user.enable  = true;
    SetMQTT_pwd.enable  = true;
    SetMQTT_topic.enable  = true;
    SetMQTT_interval.enable  = true;
    SetMQTT_devname.enable  = true;
    SetMQTT_port.enable  = true;

     SetMQTT_user.value = SmOT.MQTT_user;
     SetMQTT_pwd.value = SmOT.MQTT_pwd;

      SetMQTT_server.value = SmOT.MQTT_server;
      SetMQTT_topic.value = SmOT.MQTT_topic;
      sprintf(str, "%d",SmOT.MQTT_interval);
      SetMQTT_interval.value = str; 
      sprintf(str, "%d",SmOT.MQTT_port);
      SetMQTT_port.value = str; 

      SetMQTT_devname.value = SmOT.MQTT_devname;
    
  } else {
    CtrlChbUseMQTT.checked = false;
    SetMQTT_server.enable  = false;
    SetMQTT_user.enable  = false;
    SetMQTT_pwd.enable  = false;
    SetMQTT_topic.enable  = false;
    SetMQTT_interval.enable  = false;
    SetMQTT_devname.enable  = false;
    SetMQTT_port.enable  = false;
  }
#else //MQTT_USE

/*
    CtrlChbUseMQTT.enable  = false;
    SetMQTT_server.enable  = false;
    SetMQTT_user.enable  = false;
    SetMQTT_pwd.enable  = false;
    SetMQTT_topic.enable  = false;
    SetMQTT_interval.enable  = false;
*/    
#endif //MQTT_USE

  return String();
}

#if PID_USE

String onSetPID(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0;
   unsigned short int icheck, icheck2=0;
   unsigned short int iv;
   float v;

//   Serial.printf((PGM_P)F("onSetPID\n"));

  if( UsePID.checked) 
  {  icheck = 1;
     if( UsePID_NoLimit.checked) icheck2 = 2;
  }  else  {
      icheck = 0;
  }

  if((icheck|icheck2) != SmOT.usePID)
  { SmOT.usePID = icheck|icheck2;
    isChange = 1;
#if MQTT_USE
    MQTT_pub_usePID();    
#endif    

  }


  if(SmOT.usePID)
  { 
    iv = SetTempSrcPID.value.toInt();
    if(iv > MAX_PID_SRC && iv != 255)
      iv = MAX_PID_SRC;

//    Serial.printf("SetTempSrcPID=%s\n", SetTempSrcPID.value);
//    Serial.printf("SetTempSrcPID.value =%d\n", iv);

    if(iv != SmOT.srcTroom)
    { if((iv == -1) ||(iv == 0 && SmOT.stsT1 == 1) ||(iv == 1 && SmOT.stsT2 == 1) || (iv == 2 && SmOT.Toutside_present) || (iv >2 && SmOT.useMQTT) )
      { SmOT.srcTroom = iv;
        isChange = 1;
      }
    }
    iv = SetTempExtSrcPID.value.toInt();
    if(iv > MAX_PID_SRC && iv != 255)
      iv = MAX_PID_SRC;
    if(iv != SmOT.srcText)
    { if((iv == -1) ||(iv == 0 && SmOT.stsT1 == 1) ||(iv == 1 && SmOT.stsT2 == 1) || (iv == 2 && SmOT.Toutside_present) || (iv >2 && SmOT.useMQTT) )
      { SmOT.srcText = iv;
        isChange = 1;
      }
    }
    v = SetKpPID.value.toFloat();
    if(v != SmOT.mypid.Kp)
    { SmOT.mypid.Kp = v;
      isChange = 1;
    }
    
    v = SetKdPID.value.toFloat();
//  Serial.printf("*kdPID = %s %f\n", SetKdPID.value.c_str(), v);

    if(v != SmOT.mypid.Kd)
    { SmOT.mypid.Kd = v;
      isChange = 1;
    }
    v = SetKiPID.value.toFloat();
    if(v != SmOT.mypid.Ki)
    { SmOT.mypid.Ki = v;
      isChange = 1;
    }

    v = SetIdissPID.value.toFloat();
    if(v != SmOT.mypid.Kidiss)
    { SmOT.mypid.Kidiss = v;
      isChange = 1;
    }
    v = Set_CH_GIST.value.toFloat();
    if(v != SmOT.CH_StartGist)
    { SmOT.CH_StartGist = v;
      isChange = 1;
    }
      
//CH_StartGist

    v = SetXtagPID.value.toFloat();
    if(SmOT.usePID == 1)
    {   if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
        else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    }
    if(v != SmOT.mypid.xTag)
    {
      SmOT.set_new_PID_setpoint(v, 0); //change mypid.xTag 
//      SmOT.mypid.xTag = v;
      SmOT.TroomTarget = v;

      isChange = 1;
    }

    v = SmOT.CHtempLimit(Set_u0_PID.value.toFloat());    
    if(v != SmOT.mypid.u0)
    { SmOT.mypid.u0 = v;
      isChange = 1;
    }

    v = Set_t0_PID.value.toFloat();
    if( v > 40.)  v = 40.;
    else if(v<-80.) v = -80.;
    if(v != SmOT.mypid.y0)
    { SmOT.mypid.y0 = v;
      isChange = 1;
    }

    v = SmOT.CHtempLimit(Set_u1_PID.value.toFloat());    
    if(v != SmOT.mypid.u1)
    { SmOT.mypid.u1 = v;
      isChange = 1;
    }

    v = Set_t1_PID.value.toFloat();
    if( v > 40.)  v = 40.;
    else if(v<-80.) v = -80.;
    if(v != SmOT.mypid.y1)
    { SmOT.mypid.y1 = v;
      isChange = 1;
    }
  }

  if(isChange)
        SmOT.need_write_f = 1;  //need write changes to FS

//  Serial.printf("isChange %d onSetPID usePID %d srcText %d srcTroom %d\n",
//         isChange, SmOT.usePID, SmOT.srcText,  SmOT.srcTroom );

  aux.redirect(SETUP_URI);
  return String();
}

// PID_Page
String onSetupPID(AutoConnectAux& aux, PageArgument& args)
{ char str0[80];
  if(SmOT.usePID & 0x01) 
  { UsePID.checked = true;
  } else {
    UsePID.checked = false;    
  }
  if(SmOT.usePID == 3) 
    UsePID_NoLimit.checked = true;
  else 
    UsePID_NoLimit.checked = false;

  Info1.value = "<small>Источник: -1=n/a, 0/1=T1/T2";
  if(SmOT.Toutside_present)
      Info1.value += ", 2=Text";

#if  MQTT_USE 
  if(SmOT.useMQTT)
  { Info1.value += ", MQTT/HA:";
    sprintf(str0,"3=number.%s_t_indoor,",SmOT.MQTT_devname);
    Info1.value += str0;
    sprintf(str0,"4=number.%s_t_outdoor",SmOT.MQTT_devname);
    Info1.value += str0;
  }
#endif

  Info1.value += "</small>";

  sprintf(str0,"%d",SmOT.srcTroom);
  SetTempSrcPID.value = str0;
  sprintf(str0,"%d",SmOT.srcText);
  SetTempExtSrcPID.value = str0;
  sprintf(str0,"%.4f",SmOT.mypid.Kp);
  SetKpPID.value = str0;

  sprintf(str0,"%.4f",SmOT.mypid.Kd);
  SetKdPID.value = str0;
  
  sprintf(str0,"%.4f",SmOT.mypid.Ki);
  SetKiPID.value = str0;

  sprintf(str0,"%.4f",SmOT.mypid.Kidiss);
  SetIdissPID.value = str0;

  sprintf(str0,"%.4f",SmOT.CH_StartGist);
  Set_CH_GIST.value = str0;

  sprintf(str0,"%.2f",SmOT.mypid.xTag);
  SetXtagPID.value = str0;

   Info3.value = "ПЗА: темп.отопления | наружная";

  sprintf(str0,"%.2f",SmOT.mypid.u0);
  Set_u0_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.y0);
  Set_t0_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.u1);
  Set_u1_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.y1);
  Set_t1_PID.value = str0;

  //Info3.value = "";
  Info4.value = "";
  Info5.value = "";
  Info6.value = "";

  return String();

}

#endif

#if RELAY_USE
//AutoConnectAux SetRelayPage(RELAY_URI, "SetRelay", false, {}, false);
String onSetRelay(AutoConnectAux& aux, PageArgument& args)
{
    if(SmOT.Relay_sts)
      SmOT.RelayOnOff(false);
    else
      SmOT.RelayOnOff(true);

  aux.redirect(INFO_URI);

  return String();
}
#endif //RELAY_USE

#if ST_VERS == 2
//SetOTslave_Page
String onSetOT_slave(AutoConnectAux& aux, PageArgument& args)
{ int isChange=0;
  bool check;

  if(UseOTslave.checked) check = true;
  else                   check = false;
  if(SmOT.OT_slave_present != check)
  { isChange++;
    SmOT.OT_slave_present = check;
  }
  if(SmOT.OT_slave_present)
  {   if(OTslaveMode.checked+1 != SmOT.OT_slave_mode)
      { isChange++;
        SmOT.OT_slave_mode = OTslaveMode.checked - 1;
      }
  }

  if(isChange)
        SmOT.need_write_f = 1;  //need write changes to FS

  aux.redirect(INFO_URI);
  return String();
}


//AutoConnectAux OTslave_Page(OT2_URI, "OT2", true, {Info1, UseOTslave, Info5, Info6});
String onSetupOT_slave(AutoConnectAux& aux, PageArgument& args)
{   char str0[80];

   Info1.value = "Интерфейс slave OpenTherm:<br>";
   switch(SmOT.ot_slave_stsOT)
   {  case -2:
      Info1.value +=  String(SmOT.ot_slave_stsOT) + ": <b>Ошибка:</b> не инициализирован без OT";
        break;
      case -1:
      Info1.value +=  String(SmOT.ot_slave_stsOT) + ": <b>Ошибка:</b> не инициализирован";
        break;
      case 0:
        Info1.value +=  String(SmOT.ot_slave_stsOT) + ": работает";
        break;
      case 2:
      {  time_t now = time(nullptr);
        double dt;
        dt = difftime(now,SmOT.ot_slave_t_lastwork);
        if(dt < 3600.)
        {   sprintf(str0, (PGM_P)F("Потеря связи %.f сек назад"), dt);

        } else {        
            sprintf(str0, (PGM_P)F("Потеря связи связи  %.1f час(ов) назад"), dt);
        }
        Info1.value +=  str0;
      }
        break;
   }
//UseOTslave

    if(SmOT.OT_slave_present)
    { UseOTslave.checked = true;
      OTslaveMode.enable = true;
      if(SmOT.OT_slave_mode == 0)
        OTslaveMode.checked = 1;
      else 
        OTslaveMode.checked = 2;
    } else {
        UseOTslave.checked = false;
        OTslaveMode.enable = false;
    }
  
   Info5.value ="";
   Info6.value ="";      

  return String();
}
#endif // ST_VERS

//SendBLORPage
String onSendBlor(AutoConnectAux& aux, PageArgument& args)
{
    SmOT.need_set_RemoteRequest = 1;
    SmOT.need_send_Blor = 1;

  aux.redirect(INFO_URI);
  return String();
}


const char SM_OT_HomePage[]= "https://t.me/smartTherm";
//"https://www.umkikit.ru/index.php?route=product/product&path=67&product_id=103";

String onAbout(AutoConnectAux& aux, PageArgument& args)
{ char str[80];
  Info1.value = IDENTIFY_TEXT;
  sprintf(str, (PGM_P)F("Vers %d.%d.%d  build %s\n"),SmOT.Vers, SmOT.SubVers,SmOT.SubVers1, SmOT.BiosDate);

  Info2.value = str;
  if (WiFi.status() == WL_CONNECTED)
  {   Info3.value = "<a href=";
      Info3.value += SM_OT_HomePage;
      Info3.value += F(">Поддрержка проекта</a>\n");
  } else 
    Info3.value ="";
    
  return String();
}

int sRSSI = 0;
int razRSSI = 0;
extern int LedSts; 

void loop_web()
{  int rc,  dt;
static unsigned long t0=0, raz = 0; // t1=0;

  portal.handleClient();

  /* 3->0->3->7->1->7->1 //изменения статуса при коннекте-реконнекте 
     3->5->1->0->3
  typedef enum {
    WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_CONNECTION_LOST  = 5,
    WL_DISCONNECTED     = 6
//esp8266 
//    WL_WRONG_PASSWORD   = 6, 
//    WL_DISCONNECTED     = 7
} wl_status_t; 
  */
  rc = WiFi.status();
  { static int oldstatus=-1, oldmode=-1, needStopAP=0 ;
    static long t0 = 0;
    int mode = WiFi.getMode();
    int ch = WiFi.channel();

    if((rc != oldstatus) || mode != oldmode)
    {
   Serial.printf("WiFi.status=%i %d ", rc, raz++);
   Serial.printf("WiFi mode = %d chanel=%d\n", mode, ch);
        if(rc == WL_CONNECTED &&  (oldstatus == WL_IDLE_STATUS || oldstatus == WL_DISCONNECTED ||  oldstatus == WL_NO_SSID_AVAIL))
        {   Serial.printf("WiFi status chage to connected");
          needStopAP = 1;
            t0 = millis();

        }
        oldmode = mode;
        oldstatus = rc;
    } else if(needStopAP) {
      if(millis()-t0 > 1000)
      {
          Serial.printf("WiFi stop AP todo\n");
          needStopAP = 0;
          if(mode == WIFI_MODE_APSTA)  /* WiFi station + soft-AP mode */
          {  WiFi.softAPdisconnect(true);
            WiFi.enableAP(false);
          }
      }
    }
  }
   if(rc != WiFists)
  { 
#if SERIAL_DEBUG      
    Serial.printf("WiFi.status=%i\n", rc);
#endif    
    if(rc == WL_CONNECTED)
    {  LedSts = 0;
 //     digitalWrite(LED_BUILTIN, LedSts);   
#if SERIAL_DEBUG      
      Serial.printf((PGM_P)F("RSSI: %d dBm (%i%%)\n"), WiFi.RSSI(),_toWiFiQuality(WiFi.RSSI()));
      Serial.print(F("IP address: "));
      Serial.println(WiFi.localIP());
#endif      
    } else {
      Serial.printf("WiFi disconnected (sts=%d)\n", rc);
      LedSts = 1;
//      digitalWrite(LED_BUILTIN, LedSts);   
    }

    if( rc >=0 && rc <=7)
        WiFiDebugInfo[rc]++;
    WiFists = rc;
  }


  if(rc ==  WL_CONNECTED)
  {  dt = millis() - t0;
     if(dt > 10000)
     {   t0 = millis();
         razRSSI++;
         sRSSI += WiFi.RSSI();
  //Serial.printf(" WiFi.RSSI()=%i %i %i\n",  WiFi.RSSI(), dt, razRSSI);

         if(razRSSI > 6*10)
         {  mRSSi =  float(sRSSI)/float(razRSSI);
            razRSSI = 0;
            sRSSI = 0;
         }
    }
  }

#if MQTT_USE
  if(rc ==  WL_CONNECTED && (SmOT.useMQTT== 0x03))
         mqtt_loop();
#endif

}

/**
 *  Convert dBm to the wifi signal quality.
 *  @param  rssi  dBm.
 *  @return A signal quality percentage.
 */
unsigned int /* AutoConnect:: */ _toWiFiQuality(int32_t rssi) {
  unsigned int  qu;
  if (rssi == 31)   // WiFi signal is weak and RSSI value is unreliable.
    qu = 0;
  else if (rssi <= -100)
    qu = 0;
  else if (rssi >= -50)
    qu = 100;
  else
    qu = 2 * (rssi + 100);
  return qu;
}

int OutUTCtime(time_t now)
{   char str[312];
    char buffer[26];
    struct tm* tm_info;
    const char *s0 = "<em id=\"utcl\"></em><time id=\"upd_at\" dt=\"";
    const char *s1 = "\"></time><script>";
    const char *s2 =
"const src_el=document.getElementById('upd_at');\
const d=new Date(src_el.getAttribute('dt')).toLocaleString();\
document.getElementById(\"utcl\").innerHTML=d;</script>";

/*
<em id="utcl"></em>
<time id="upd_at" dt="2021-06-30 12:21:17Z"></time>
<script>
const src_el = document.getElementById('upd_at');
const d = new Date(src_el.getAttribute('dt')).toLocaleString();
document.getElementById("utcl").innerHTML = d;
</script>
*/  

//  now = time(nullptr);
//  Serial.printf("****** 2 %s\n", ctime(&now));
  tm_info = localtime(&now);

  strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);  
  buffer[25] = 0;
//  Serial.printf("*******3 %s\n", buffer);
  sprintf(str,"%s%sZ%s%s", s0,buffer,s1, s2);
  utc_time_jc = str;
/*  
  Serial.printf("****** %s len=%d\n", str, strlen(str));
  Serial.println(utc_time_jc);
*/
  return 0;
}

/************************************/
void setup_read_config(void)
{ bool b;
  b = FlashFS.begin(AUTOCONNECT_FS_INITIALIZATION);
  if(b == false)
  {   Serial.println(F("FlashFS.begin failed"));
  }
   
  SmOT.Read_ot_fs();
//  SmOT.Read_mqtt_fs();
  SmOT.init(1);

}

void check_fs(void)
{ bool b;
/**********************************/
// Check consistency of reported partiton size info.
/*    
   {  esp_err_t ret;
        Serial.println("Performing SPIFFS_check().");
        ret = esp_spiffs_check(NULL);
        // Could be also used to mend broken files, to clean unreferenced pages, etc.
        // More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
        if (ret != ESP_OK) {
            Serial.printf("SPIFFS_check() failed (%s)\n", esp_err_to_name(ret));
            return;
        } else {
            Serial.println("SPIFFS_check() successful");
        }
    }
*/
/*******************************/
 #if defined(ARDUINO_ARCH_ESP32)
{ File root = FlashFS.open("/");
  File file = root.openNextFile();
 
  while(file){
 
#if SERIAL_DEBUG      
      Serial.print("FILE: ");
      Serial.printf( "%s %d\n", file.name(), file.size());
#endif      
      if(file.size() > 1000000)
       { char str[80];
         sprintf(str,"/%s",file.name() );
      #if SERIAL_DEBUG      
         Serial.printf( "remove %s\n", str);
      #endif         
         file.close();
         b = FlashFS.remove(str);
      #if SERIAL_DEBUG      
         Serial.printf( "remove  rc = %d\n", b);
      #endif         
         break;
       }
      
      file = root.openNextFile();
      
  }
}
#endif

#if SERIAL_DEBUG      
    { int tBytes, uBytes; 
#if defined(ARDUINO_ARCH_ESP8266)
      FSInfo info;
      FlashFS.info(info);
      tBytes  = info.totalBytes;
      uBytes = info.usedBytes;
#else
      tBytes  = FlashFS.totalBytes();
      uBytes = FlashFS.usedBytes();
#endif      
      Serial.printf("FlashFS tBytes = %d used = %d\n", tBytes, uBytes);
    }
#endif //SERIAL_DEBUG     

}


