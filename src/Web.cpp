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
AutoConnectCheckbox UseID2ChB("UseID2ChB","", "Использовать OT ID2", false, AC_Behind , AC_Tag_BR);
ACInput(ID2MaserID,"", "IDcode"); 
ACSubmit(ApplyAddpar,   "Задать", SET_ADD_URI, AC_Tag_BR);
AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false, { Ctrl1, UseID2ChB, ID2MaserID, ApplyAddpar});

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

  debugPage.on(onDebug);
  AboutPage.on(onAbout);

/**/  
  portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, SetAddParPage, debugPage,  AboutPage});     // Join pages.
//  portal.join({InfoPage, Setup_Page, SetTempPage});     // Join pages.
  config.ota = AC_OTA_BUILTIN;
  config.portalTimeout = 1; 
  config.retainPortal = true; 
  config.autoRise = true;
  // Enable saved past credential by autoReconnect option,
  // even once it is disconnected.
//  config.autoReconnect = true;
//  config.reconnectInterval = 1;

  portal.config(config);
  portal.onConnect(onConnect);  // Register the ConnectExit function
  portal.begin();

  WiFiWebServer&  webServer = portal.host();

  webServer.on("/", onRoot);  // Register the root page redirector.
//  Serial.println("Web server started:" +WiFi.localIP().toString());
  if (WiFi.status() != WL_CONNECTED)  {
    Serial.println(F("WiFi Not connected"));
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
  if(init)
    return 1;
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
//    Serial.println("WiFiOn");
    WiFi.setAutoReconnect(true);
/****************************************************/    
{
 // Specifying the time zone and assigning NTP.
// Required to add the correct local time to the export file name of the
// captured image. This assignment needs to be localized.
// This sketch works even if you omit the NTP server specification. In that
// case, the suffix timestamp of the captured image file is the elapsed time
// since the ESP module was powered on.
const char*  const _tz = "MSK-3";
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
    configTzTime(_tz, _ntp1 ,_ntp2);
   delay(1000);
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

String onDebug(AutoConnectAux& aux, PageArgument& args)
{  char str[180];
  // int l;
extern int minRamFree;

//WiFiDebugInfo
//   sprintf(str,"WiFi statistics:");
//   Info1.value = str;
   Info1.value = F("WiFi statistics:");
   sprintf(str,"%d %d  %d %d  %d %d  %d %d", 
      WiFiDebugInfo[0],WiFiDebugInfo[1],WiFiDebugInfo[2],WiFiDebugInfo[3],WiFiDebugInfo[4],WiFiDebugInfo[5],WiFiDebugInfo[6],WiFiDebugInfo[7]);
   Info2.value = str;
   if(WiFists == WL_CONNECTED)
   {  sprintf(str,"RSSI: %d dBm (%i%%), среднее за 10 мин %.1f",WiFi.RSSI(),_toWiFiQuality(WiFi.RSSI()), mRSSi);
      Info3.value = str;
   } else 
      Info3.value = "";
   sprintf(str,"OpenTherm statistics:<br>%d %d  %d %d  % d %d  %d %d  %d %d  %d", 
      OTDebugInfo[0], OTDebugInfo[1], OTDebugInfo[2], OTDebugInfo[3], OTDebugInfo[4], OTDebugInfo[5], OTDebugInfo[6],OTDebugInfo[7], OTDebugInfo[8],OTDebugInfo[9], OTDebugInfo[10]);
   //l = strlen(str);
   //Serial.printf("4 l=%d\n", l);

   Info4.value = str;
   sprintf(str,"min free RAM %d", minRamFree);
   Info5.value = str;
  
  { time_t now;
    struct tm *nowtime;
    now = time(nullptr);
    nowtime = localtime(&now);
    sprintf( str, "<br>%02d.%02d.%d %d:%02d:%02d",
          nowtime->tm_mday,nowtime->tm_mon+1,nowtime->tm_year+1900,
		  nowtime->tm_hour, nowtime->tm_min, nowtime->tm_sec);
 
    Info5.value += str;
  }

   sprintf(str,"Вкл горелки:<br>Всего %d<br>За час %d<br>Пред.час %d<br>Сутки %d<br>Пред.сутки %d", 
          SmOT.Bstat.NflameOn, SmOT.Bstat.NflameOn_h, SmOT.Bstat.NflameOn_h_prev, SmOT.Bstat.NflameOn_day, SmOT.Bstat.NflameOn_day_prev);
//   l = strlen(str);
//   Serial.printf("5 l=%d\n", l);
   
   Info6.value = str;
   sprintf(str,"<br>Эффективная модуляция:<br>За час %.2f<br>Пред.час %.2f<br>Сутки %.2f<br>Пред.сутки %.2f", 
          SmOT.Bstat.Eff_Mod_h, SmOT.Bstat.Eff_Mod_h_prev, SmOT.Bstat.Eff_Mod_d, SmOT.Bstat.Eff_Mod_d_prev);

//   l = strlen(str);
//   Serial.printf("6 l=%d\n", l);

   Info6.value += str;
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
    { v = SetBoilerTemp.value.toFloat();
      if(v != SmOT.Tset)
      { isChange = 1;
        SmOT.Tset = v;
        SmOT.need_set_T = 1;
      }
    }

    if(SmOT.enable_HotWater)
    { v = SetDHWTemp.value.toFloat();
      if(v != SmOT.TdhwSet)
      { isChange = 1;
        SmOT.TdhwSet = v;
        SmOT.need_set_dhwT = 1;
      }
    }

    if(SmOT.enable_CentralHeating2)
    { v = SetBoilerTemp2.value.toFloat();
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
    if(v !=SmOT.MQTT_interval )
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
   Serial.printf("redir = %d uri=%s isChange %d\n", redir, SETUP_URI, isChange);
else
   Serial.printf("redir = %d uri=%s isChange %d\n", redir, INFO_URI, isChange);
  if(redir)
    aux.redirect(SETUP_URI);
  else
    aux.redirect(INFO_URI);

  return String();
}

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
        {   sprintf(str0, "Потеря связи с котлом %.f сек назад", dt);

        } else {        
            sprintf(str0, "Потеря связи связи с котлом %.1f час(ов) назад", dt);
        }
        Info1.value =  String(SmOT.stsOT) + " : <b>Ошибка:</b> ";
        Info1.value +=  str0;
      }
        break;
   }

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
        {    sprintf(str0, " OEM-specific fault/error cod = %d ( hex %x)", (SmOT.Fault&0xff), (SmOT.Fault&0xff));
            Info6.value += str0;
        }
        Info6.value += "<br>";
      }
      if(SmOT.OEMDcode)
      {     sprintf(str0, "OEM-specific diagnostic/service code = %d  ( hex %x)<br>", SmOT.OEMDcode, SmOT.OEMDcode);
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
Buderus	8
Ferrolli 	9
Remeha	11
Baxi 27
Viessmann  VITOPEND 	33
Navinien 	148
Baxi ampera (electro) 247
Zota Lux-x (electro)  248
*/
  if(SmOT.OTmemberCode ==1)
      Ctrl2.value += "Baxi Fourtech/Luna 3"; 
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
  else if(SmOT.OTmemberCode == 148)
      Ctrl2.value += "Navinien"; 
  else if(SmOT.OTmemberCode == 247)
      Ctrl2.value += "Baxi ampera (electro)"; 
  else if(SmOT.OTmemberCode == 248)
      Ctrl2.value += "Zota Lux-x (electro)"; 
  else 
      Ctrl2.value +=  "код " + String(SmOT.OTmemberCode);

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


//расшифровка битов статуса бойлера для бестолковых
//decoding the status bits of the boiler for dummies
/*  LB: Slave status   
bit: description [ clear/0, set/1]
0: fault indication [ no fault, fault ]
1: CH mode [CH not active, CH active]
2: DHW mode [ DHW not active, DHW active]
3: Flame status [ flame off, flame on ]
4: Cooling status [ cooling mode not active, cooling mode active ]
5: CH2 mode [CH2 not active, CH2 active]
6: diagnostic indication [no diagnostics, diagnostic event]
7: reserved 

E = Error 0/1
H = CH = CentralHeating 0/1
W = DHW = HotWater 0/1
F = Flame status  0/1
D =  diagnostic indication 0/1

Tb = BoilerT
Tr = RetT
Th = dhw_t
T1 = T_ds18b20(1)
T2 = T_ds18b20(2)
*/   

char SM_OT_HomePage[]=  "https://www.umkikit.ru/index.php?route=product/product&path=67&product_id=103";

String onAbout(AutoConnectAux& aux, PageArgument& args)
{ char str[80];
  Info1.value = IDENTIFY_TEXT;
  sprintf(str, "Vers %d.%d build %s\n",SmOT.Vers, SmOT.SubVers, SmOT.BiosDate);
  Info2.value = str;
  if (WiFi.status() == WL_CONNECTED)
  {   Info3.value = "<a href=";
      Info3.value += SM_OT_HomePage;
      Info3.value += ">Домашняя страница проекта</a>\n";
  } else 
    Info3.value ="";
    
  return String();
}

int sts = 0;
int sRSSI = 0;
int razRSSI = 0;
extern int LedSts; 

void loop_web()
{  int rc,  dt;
static unsigned long t0=0; // t1=0;

  portal.handleClient();
  if(sts == 0)
  { //  Serial.println("loop_web()");
       sts = 1;
  }
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
  if(rc != WiFists)
  { 
#if SERIAL_DEBUG      
    Serial.printf("WiFi.status=%i\n", rc);
#endif    
    if(rc == WL_CONNECTED)
    {  LedSts = 0;
 //     digitalWrite(LED_BUILTIN, LedSts);   
#if SERIAL_DEBUG      
      Serial.printf("RSSI: %d dBm (%i%%)\n", WiFi.RSSI(),_toWiFiQuality(WiFi.RSSI()));
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
#endif      
    } else {
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

