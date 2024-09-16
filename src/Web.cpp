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
  name.begin(AUTOCONECT_FS_INITIALIZATION);
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


/*********************************/
const char* INFO_URI = "/info";
const char* SETUP_URI = "/setup";
const char* SETUP_ADD_URI =  "/setupadd";
const char* ABOUT_URI = "/about";
const char* SET_T_URI =  "/set_t";
const char* SET_PAR_URI =  "/set_par";
const char* SET_ADD_URI =  "/add";
const char* DEBUG_URI = "/debug";

#if PID_USE
const char* PID_URI = "/pid";
const char* SET_PID_URI = "/set_pid";
#endif

/************* InfoPage ******************/
ACText(Caption, "<b>Статус OT: </b>", "", "", AC_Tag_DIV);
ACText(Info1, "", "", "", AC_Tag_DIV);
ACText(Info2, "", "", "", AC_Tag_DIV);
ACText(Info3, "", "", "", AC_Tag_DIV);
ACText(Info4, "", "", "", AC_Tag_DIV);
ACText(Info5, "", "", "", AC_Tag_DIV);
ACText(Info6, "", "", "", AC_Tag_DIV);
ACText(Info7, "", "", "", AC_Tag_DIV);
ACInput(SetBoilerTemp,"", "Температура теплоносителя:<br>"); // Boiler Control setpoint
ACInput(SetDHWTemp,   "", "Температура горячей воды:<br>");  // DHW Control setpoint
ACInput(SetBoilerTemp2,"", "Температура CH2:<br>"); // Boiler CH2 Control setpoint

ACSubmit(Apply, "Обновить", INFO_URI, AC_Tag_DIV);
ACSubmit(SetNewBoilerTemp,"Задать", SET_T_URI, AC_Tag_DIV);

/************* SetupPage ***************/
ACText(Ctrl1, "Настройки котла:", "", "", AC_Tag_DIV);
ACText(Ctrl2, "", "", "", AC_Tag_DIV);
//ACCheckbox(CtrlChB1,"checkbox", "uniqueapid");
AutoConnectCheckbox CtrlChB1("CtrlChB1","1", "Отопление", false, AC_Behind , AC_Tag_BR);
AutoConnectCheckbox CtrlChB2("CtrlChB2","2", "Горячая вода", false, AC_Behind , AC_Tag_DIV);
AutoConnectCheckbox CtrlChB3("CtrlChB3","3", "Отопление 2", false, AC_Behind , AC_Tag_DIV);
#if MQTT_USE
AutoConnectCheckbox CtrlChB4("CtrlChB4","4", "MQTT", false, AC_Behind , AC_Tag_DIV);
ACInput(SetMQTT_server,"", "сервер"); 
ACInput(SetMQTT_user,"", "user"); 
ACInput(SetMQTT_pwd,"", "pwd"); 
ACInput(SetMQTT_topic,"", "топик"); 
ACInput(SetMQTT_devname,"", "имя устройства"); 
ACInput(SetMQTT_interval,"", "интервал, сек"); 
#endif // MQTT_USE
  
//AutoConnectCheckbox checkbox("checkbox", "uniqueapid", "Use APID unique", false);
//ACCheckbox(CtrlChB2,"a2", "", true,  AC_Behind , AC_Tag_DIV);
ACSubmit(ApplyChB, "Задать", SET_PAR_URI, AC_Tag_DIV);
ACSubmit(ApplyAdd, "Дополнительно", SETUP_ADD_URI, AC_Tag_None);


/************* SetupAdditionPage for MConfigMMemberIDcode ***************/
AutoConnectCheckbox UseID2ChB("UseID2ChB","", "Использовать OT ID2", false, /* AC_Infront */ AC_Behind  , AC_Tag_None);
ACInput(ID2MaserID,"", "IDcode"); 
AutoConnectCheckbox UseCH2_DHW_ChB("UseCH2DHW","", "Использовать CH2 для горячей воды", false, AC_Infront /* AC_Behind */ , AC_Tag_BR);
ACSubmit(ApplyAddpar,   "Задать", SET_ADD_URI, AC_Tag_BR);
#if PID_USE
ACSubmit(SetupPID,   "PID", PID_URI, AC_Tag_BR);
AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false, { Ctrl1, UseID2ChB, ID2MaserID, UseCH2_DHW_ChB, ApplyAddpar, SetupPID});
#else
AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false, { Ctrl1, UseID2ChB, ID2MaserID, UseCH2_DHW_ChB, ApplyAddpar});
#endif //#if PID_USE


/************* SetPID ******************/
#if PID_USE
AutoConnectCheckbox UsePID("UsePID","", "Использовать PID", false, AC_Behind , AC_Tag_BR);
ACInput(SetXtagPID,"", "Xtag:"); // 
ACInput(SetTempSrcPID,"", "Источник температуры в комнате:"); // 
ACInput(SetTempExtSrcPID,"", "Источник температуры на улице:"); // 
ACInput(SetKpPID,"", "Kp:","","",AC_Tag_None); //  
ACInput(SetKdPID,"", "Kd:","","",AC_Tag_None); // 
ACInput(SetKiPID,"", "Ki:","","",AC_Tag_BR); // 
ACInput(SetTmaxPID,"", "Tmax:","","",AC_Tag_None); // 
ACInput(SetTminPID,"", "Tmin:"); // 
ACInput(Set_u0_PID,"", "u0:","","",AC_Tag_None); // 
ACInput(Set_t0_PID,"", "t0:"); // 
ACInput(Set_u1_PID,"", "u1:","","",AC_Tag_None); // 
ACInput(Set_t1_PID,"", "t1:"); // 

ACSubmit(ApplyPID,   "Задать", SET_PID_URI, AC_Tag_BR);
AutoConnectAux PID_Page(PID_URI, "PID", true, {UsePID, Info1,SetXtagPID,  SetTempSrcPID, SetTempExtSrcPID, 
                      SetKpPID, SetKdPID, SetKiPID, SetTmaxPID, SetTminPID, Info2,
                      Info3, Set_u0_PID, Set_t0_PID, Set_u1_PID,Set_t1_PID, Info4, Info5, Info6,  ApplyPID });
#endif
/************* SetPID end ***************/

/************* SetTempPage ***************/
//ACText(SetTemp_info1, "", "", "", AC_Tag_DIV); //Заданная температура:
//ACSubmit(SetTemp_OK, "Ok", INFO_URI, AC_Tag_DIV);
/************* SetParPage ***************/

/************* debugPage( ****************/
ACSubmit(DebugApply, "Обновить", DEBUG_URI, AC_Tag_DIV);
/************* AboutPage *****************/
ACText(About_0, "<b>About:</b>", "", "", AC_Tag_DIV);
/*****************************************/

// AutoConnectAux for the custom Web page.
AutoConnectAux InfoPage(INFO_URI, "SmartTherm", true, { Caption, Info1, Info2, Info3, Info4, Info5, Info6, Info7,  Apply, SetBoilerTemp, SetDHWTemp, SetBoilerTemp2, SetNewBoilerTemp });

#if MQTT_USE
AutoConnectAux Setup_Page(SETUP_URI, "Setup", true, { Ctrl1, CtrlChB1, CtrlChB2, CtrlChB3, Ctrl2, CtrlChB4, SetMQTT_user, SetMQTT_pwd, SetMQTT_server, SetMQTT_topic, SetMQTT_devname, SetMQTT_interval, ApplyAdd,ApplyChB, Info1});
#else
AutoConnectAux Setup_Page(SETUP_URI, "Setup", true, { Ctrl1, CtrlChB1, CtrlChB2, CtrlChB3, Ctrl2, ApplyAdd, ApplyChB});
#endif // MQTT_USE


AutoConnectAux SetTempPage(SET_T_URI, "SetTemp", false, {}, false);
AutoConnectAux SetParPage(SET_PAR_URI, "SetPar", false, {}, false);
AutoConnectAux SetAddParPage(SET_ADD_URI, "SetAdd", false, {}, false);
#if PID_USE
AutoConnectAux SetPIDPage(SET_PID_URI, "SetPID", false, {}, false);
#endif
AutoConnectAux debugPage(DEBUG_URI, "Debug", true, {Info1, Info2, Info3, Info4, Info5, Info6,  DebugApply});
AutoConnectAux AboutPage(ABOUT_URI, "About", true, { About_0, Info1, Info2, Info3});

AutoConnectConfig config;
AutoConnect portal;


/************************************/
//int test_fs(void);

void setup_web_common(void);
int setup_web_common_onconnect(void);
void loop_web(void);
void onRoot(void);
void loadParam(String fileName);
void onConnect(IPAddress& ipaddr);
#if MQTT_USE
  extern void mqtt_setup(void);
  extern void mqtt_loop(void);
#endif
String onInfo(AutoConnectAux& aux, PageArgument& args);
String on_Setup(AutoConnectAux& aux, PageArgument& args);
String on_SetupAdd(AutoConnectAux& aux, PageArgument& args);
String onSetTemp(AutoConnectAux& aux, PageArgument& args);
String onSetPar(AutoConnectAux& aux, PageArgument& args);
String onSetAddPar(AutoConnectAux& aux, PageArgument& args);
String onDebug(AutoConnectAux& aux, PageArgument& args);
String onS(AutoConnectAux& aux, PageArgument& args);
String onAbout(AutoConnectAux& aux, PageArgument& args);
#if PID_USE
String onSetupPID(AutoConnectAux& aux, PageArgument& args);
String onSetPID(AutoConnectAux& aux, PageArgument& args);
#endif

String utc_time_jc;

/************************************/
unsigned int /* AutoConnect:: */ _toWiFiQuality(int32_t rssi);

/************************************/

void setup_web_common(void)
{    bool b;

//  Serial.println();
//   Serial.println("setup_web_common");
  b = FlashFS.begin(AUTOCONNECT_FS_INITIALIZATION);
  if(b == false)
  {   Serial.println(F("FlashFS.begin failed"));
  }
    
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
 
      Serial.print("FILE: ");
      Serial.printf( "%s %d\n", file.name(), file.size());
      if(file.size() > 1000000)
       { char str[80];
         sprintf(str,"/%s",file.name() );
         Serial.printf( "remove %s\n", str);
         file.close();
         b = FlashFS.remove(str);
         Serial.printf( "remove  rc = %d\n", b);
         break;
       }
      
      file = root.openNextFile();
      
  }
}
#endif
//  FlashFS.begin(FORMAT_ON_FAIL); //AUTOCONNECT_FS_INITIALIZATION);

#if SERIAL_DEBUG      
 #if defined(ARDUINO_ARCH_ESP8266)
   Serial.printf("OT_ids[0].used =%d\n", OT_ids[0].used);
 #elif defined(ARDUINO_ARCH_ESP32)
   Serial.printf("OT_ids[0].used =%d %s\n", OT_ids[0].used,  OT_ids[0].descript);
 #endif
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

  SmOT.Read_ot_fs();
  SmOT.init();

  {  char str[40];
     sprintf(str,"%.1f",SmOT.Tset);
     SetBoilerTemp.value = str;
     sprintf(str,"%.1f",SmOT.TdhwSet);
     SetDHWTemp.value = str;

  #if MQTT_USE
     SetMQTT_server.value = SmOT.MQTT_server;
     SetMQTT_user.value = SmOT.MQTT_user;
     SetMQTT_pwd.value = SmOT.MQTT_pwd;

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
  
#if PID_USE
  PID_Page.on(onSetupPID);
  SetPIDPage.on(onSetPID);
#endif  
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
  }  else {
    setup_web_common_onconnect();
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

}

int setup_web_common_onconnect(void)
{ static int init = 0;

  Serial.printf("setup_web_common_onconnect init %d\n", init);

  Serial.println(F("WiFi connected"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

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
   delay(1000);
#if SERIAL_DEBUG      
  now = time(nullptr);
  Serial.printf("2 %s\n", ctime(&now));
#endif  
{ time_t now;
  now = time(nullptr);
  Serial.printf("2 %s\n", ctime(&now));
}
  // uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 ()
#if defined(ARDUINO_ARCH_ESP8266)
// default ntp update  1 hour
// it can be redefined via uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 ()
#elif defined(ARDUINO_ARCH_ESP32)
  Serial.print("Sync time in ms: ");
  Serial.println(sntp_get_sync_interval());  
#endif


#if MQTT_USE
   if(SmOT.useMQTT == 0x03) 
     mqtt_setup();
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
   {  extern int debcode;
      sprintf(str,"<br>debcode %d",  debcode); 

   Info6.value += str;

   }
#endif   

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
    { v = SmOT.CHtempLimit(SetBoilerTemp.value.toFloat());
      if(v != SmOT.Tset)
      { isChange = 1;
        SmOT.Tset = v;
        SmOT.need_set_T = 1;
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

String onSetPar(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0,  redir = 0;
   bool check;

  if( CtrlChB1.checked) check = true;
  else                  check = false;
  if(check != SmOT.enable_CentralHeating)
  { isChange++;
    SmOT.enable_CentralHeating = check;
  }

  if( CtrlChB2.checked) check = true;
  else                  check = false;
  if(check != SmOT.enable_HotWater)
  { isChange++;
    SmOT.enable_HotWater = check;
  }
  
  if( CtrlChB3.checked) check = true;
  else                  check = false;
  if(check != SmOT.enable_CentralHeating2)
  { isChange++;
    SmOT.enable_CentralHeating2 = check;
  }

#if MQTT_USE
  if( CtrlChB4.checked) check = true;
  else                  check = false;

  if(check)
  { if(SmOT.useMQTT == 0)
    { SmOT.useMQTT = 1;
      redir = 1;  
    } else if(SmOT.useMQTT == 1) { 
      SmOT.useMQTT = 0x3;
      isChange++;
    }
  } else {
    if(SmOT.useMQTT != 0)
    { SmOT.useMQTT = 0;
      isChange++;
    }
  }

   if(SmOT.useMQTT && redir== 0)
   {   char str0[80];
      int i,v;
 
    SetMQTT_server.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_server,str0))
    { isChange++;
       strcpy(SmOT.MQTT_server,str0);      
    }

    SetMQTT_user.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_user,str0))
    { isChange++;
       strcpy(SmOT.MQTT_user,str0);      
    }
    SetMQTT_pwd.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_pwd,str0))
    { isChange++;
       strcpy(SmOT.MQTT_pwd,str0);      
    }

    SetMQTT_devname.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.MQTT_devname,str0))
    { isChange++;
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
    { isChange++;
       strcpy(SmOT.MQTT_topic,str0);      
    }

    v = SetMQTT_interval.value.toInt();
    if((unsigned int)v !=SmOT.MQTT_interval )
    { isChange++;
       SmOT.MQTT_interval = v;
    }
   }

#endif //MQTT_USE

  if(isChange)
        SmOT.need_write_f = 1;  //need write changes to FS

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
    

  sprintf(str,"%d",SmOT.ID2masterID);
  ID2MaserID.value = str;

  if( SmOT.CH2_DHW_flag)
      UseCH2_DHW_ChB.checked = true;
  else
      UseCH2_DHW_ChB.checked = false;
  

  return String();
}

// Load the attribute of th
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

}

#endif // MQTT_USE 


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

      if(SmOT.HotWater_present) 
      { Info2.value +=  " Горячая вода " + String(SmOT.dhw_t);
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
                 Info6.value += " Lowwater press";
             if(SmOT.Fault & 0x0800)
                 Info6.value += " Gas/flame fault";
             if(SmOT.Fault & 0x01000)
                 Info6.value += " Air press fault";
             if(SmOT.Fault & 0x02000)
                 Info6.value += " Water over-temp fault";
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
    if(SmOT.enable_CentralHeating)
      SetBoilerTemp.enable = true;
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
  } else {
        Info2.value = "";
        Info4.value = "";
        Info5.value = "";
        Info6.value = "";
        Info7.value = "";
  }
 
/********************/
  return String();
}

// SmOT.OTmemberCode
// see as well on_setpar()
String on_Setup(AutoConnectAux& aux, PageArgument& args)
{  

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

  Info1.value ="";

  Ctrl2.value = "Котёл: "; 
  
/*
Baxi Fourtech/Luna 3  1
Baxi Slim  4
Buderus	8
Ferrolli 	9
Remeha	11
Baxi 27 (Baxi Luna Duo-Tec  P67=0)
Viessmann  VITOPEND 	33
Baxi Luna Duo-Tec 1.24 GA P67=2 56
Navinien 	148
Baxi ampera (electro) 247
Zota Lux-x (electro)  248
*/


  if(SmOT.OTmemberCode ==1)
      Ctrl2.value += "Baxi Fourtech/Luna 3"; 
  else if(SmOT.OTmemberCode == 4)
      Ctrl2.value += "Baxi Slim"; 
  else if(SmOT.OTmemberCode == 8)
      Ctrl2.value += "Buderus"; 
  else if(SmOT.OTmemberCode == 9)
      Ctrl2.value += "Ferrolli"; 
  else if(SmOT.OTmemberCode == 11)
      Ctrl2.value += "Remeha"; 
  else if(SmOT.OTmemberCode == 27)
      Ctrl2.value += "Baxi"; 
  else if(SmOT.OTmemberCode == 33)
      Ctrl2.value += "Viessmann";
  else if(SmOT.OTmemberCode == 56)
      Ctrl2.value += "Baxi Luna Duo-Tec P67=2";
  else if(SmOT.OTmemberCode == 148)
      Ctrl2.value += "Navinien"; 
  else if(SmOT.OTmemberCode == 247)
      Ctrl2.value += "Baxi ampera (electro)"; 
  else if(SmOT.OTmemberCode == 248)
      Ctrl2.value += "Zota Lux-x (electro)"; 
  else 
      Ctrl2.value +=  "код " + String(SmOT.OTmemberCode);
/*********************************/      
 } else {
    CtrlChB2.enable  = false;
    CtrlChB3.enable  = false;
    Info1.value ="";
    Ctrl2.value = ""; 
 }
 
#if MQTT_USE
  CtrlChB4.enable  = true;
  if(SmOT.useMQTT) 
  { char str[40]; 
    if(SmOT.useMQTT == 1) 
      Info1.value = "Нужен Reset"; 
    CtrlChB4.checked = true;
    SetMQTT_server.enable  = true;
    SetMQTT_user.enable  = true;
    SetMQTT_pwd.enable  = true;
    SetMQTT_topic.enable  = true;
    SetMQTT_interval.enable  = true;
    SetMQTT_devname.enable  = true;

      SetMQTT_server.value = SmOT.MQTT_server;
      SetMQTT_topic.value = SmOT.MQTT_topic;
      sprintf(str, "%d",SmOT.MQTT_interval);
      SetMQTT_interval.value = str; 
      SetMQTT_devname.value = SmOT.MQTT_devname;
    
  } else {
    CtrlChB4.checked = false;
    SetMQTT_server.enable  = false;
    SetMQTT_user.enable  = false;
    SetMQTT_pwd.enable  = false;
    SetMQTT_topic.enable  = false;
    SetMQTT_interval.enable  = false;
    SetMQTT_devname.enable  = false;
  }
#else //MQTT_USE

/*
    CtrlChB4.enable  = false;
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
   unsigned short int icheck;
   unsigned short int iv;
   float v;

   Serial.printf((PGM_P)F("onSetPID\n"));

  if( UsePID.checked) icheck = 1;
  else               icheck = 0;
  if(icheck != SmOT.usePID)

  if(icheck != SmOT.usePID)
  { SmOT.usePID = icheck;
    isChange = 1;
  }

  if(SmOT.usePID)
  { 
    iv = SetTempSrcPID.value.toInt();
    if(iv > MAX_PID_SRC && iv != 255)
      iv = MAX_PID_SRC;

    if(iv != SmOT.srcTroom)
    {  SmOT.srcTroom = iv;
       isChange = 1;
    }
    iv = SetTempExtSrcPID.value.toInt();
    if(iv > MAX_PID_SRC && iv != 255)
      iv = MAX_PID_SRC;
    if(iv != SmOT.srcText)
    { SmOT.srcText = iv;
      isChange = 1;
    }
    v = SetKpPID.value.toFloat();
    if(v != SmOT.mypid.Kp)
    { SmOT.mypid.Kp = v;
      isChange = 1;
    }
    v = SetKdPID.value.toFloat();
    if(v != SmOT.mypid.Kd)
    { SmOT.mypid.Kd = v;
      isChange = 1;
    }
    v = SetKiPID.value.toFloat();
    if(v != SmOT.mypid.Ki)
    { SmOT.mypid.Ki = v;
      isChange = 1;
    }
    v = SetXtagPID.value.toFloat();

    if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
    else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    if(v != SmOT.mypid.xTag)
    { SmOT.mypid.xTag = v;
      isChange = 1;
    }

    v = SmOT.CHtempLimit(SetTmaxPID.value.toFloat());    
    if(v != SmOT.mypid.umax)
    { SmOT.mypid.umax = v;
      isChange = 1;
    }

    v = SmOT.CHtempLimit(SetTminPID.value.toFloat());    
    if( v > SmOT.mypid.umax - 1.)  v = SmOT.mypid.umax -1.;
    if(v != SmOT.mypid.umin)
    { SmOT.mypid.umin = v;
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
  if(SmOT.usePID) 
  { UsePID.checked = true;
  } else {
    UsePID.checked = false;    
  }

  Info1.value = "Источник: -1=n/a, 0/1=T1/T2, 2=Text, 3/4=MQTT0/1";

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
  sprintf(str0,"%.2f",SmOT.mypid.xTag);
  SetXtagPID.value = str0;

  Info2.value = "Tmax <= 80, Tmin >= 30 (конденсатный котел, иначе 40)";

  sprintf(str0,"%.2f",SmOT.mypid.umax);
  SetTmaxPID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.umin);
  SetTminPID.value = str0;
 
  Info2.value = "ПЗА: темп.отопления | наружная";

  sprintf(str0,"%.2f",SmOT.mypid.u0);
  Set_u0_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.y0);
  Set_t0_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.u1);
  Set_u1_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.y1);
  Set_t1_PID.value = str0;

  Info3.value = "";
  Info4.value = "";
  Info5.value = "";
  Info6.value = "";

  return String();

}

#endif


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
static unsigned long t0=0; // t1=0;

  portal.handleClient();

  /* 3->0->3->7->1->7->1 //изменения статуса при коннекте-реконнекте
  typedef enum {
    WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_WRONG_PASSWORD   = 6,
    WL_DISCONNECTED     = 7
} wl_status_t; 
  */
  rc = WiFi.status();
//    Serial.printf("WiFi.status=%i %d\n", rc, raz++);
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

//  time_t now = time(nullptr);
//  Serial.println(ctime(&now));
//  ba = Serial.available();
//  Serial.printf("a=%i ", ba);

//  Serial.printf("water_count=%i\n", water_count);
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

#if defined(ARDUINO_ARCH_ESP32)
void TestPower(void)
{  int8_t power =-1;
      static int p = -127;
      int rc;

      rc = esp_wifi_get_max_tx_power(&power);
     Serial.printf("rc=%d max_tx_power =%d \n", rc, power);
/*     
       power = p;
      rc = esp_wifi_set_max_tx_power(power);
      if(rc == ESP_OK)
     Serial.printf("| rc=%d  set to %d\n", rc, power);
      p++;
      if(p<-5) p += 5;
      if(p > 127) p = -1;
*/      
}
#endif

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

  now = time(nullptr);
//  Serial.printf("2 %s\n", ctime(&now));
  tm_info = localtime(&now);

  strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);  
  buffer[25] = 0;
//  Serial.printf("3 %s\n", buffer);
  sprintf(str,"%s%sZ%s%s", s0,buffer,s1, s2);
  utc_time_jc = str;
/*  
  Serial.printf("%s len=%d\n", str, strlen(str));
  Serial.println(utc_time_jc);
*/
  return 0;
}