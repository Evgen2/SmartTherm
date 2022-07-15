/* Web.cpp  UTF-8  */
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
using WiFiWebServer = ESP8266WebServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
using WiFiWebServer = WebServer;
#endif


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

#include "Smart_Config.h"

#include <AutoConnect.h>
#include <AutoConnectFS.h>
AutoConnectFS::FS& FlashFS = AUTOCONNECT_APPLIED_FILESYSTEM;
#include <time.h>
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

#if defined(ARDUINO_ARCH_ESP8266)
char SmartDevice::BiosDate[12]=__DATE__;   /* дата компиляции биоса */

#endif

extern  SD_Termo SmOT;
int WiFiDebugInfo[10] ={0,0,0,0,0, 0,0,0,0,0};
int OTDebugInfo[10] ={0,0,0,0,0, 0,0,0,0,0};


/*********************************/
// Declare AutoConnectText with only a value.
// Qualify the Caption by reading style attributes from the style.json file.
const char* HELLO_URI = "/hello";
const char* UPDATE_T_URI =  "/upd";
const char* DEBUG_URI = "/debug";
const char* SIMPLE_URI = "/s";

ACText(Caption, "<b>Статус OT: </b>", "", "", AC_Tag_DIV);
ACText(Info1, "info text 1", "", "", AC_Tag_DIV);
ACText(Info2, "info text 2", "", "", AC_Tag_DIV);
ACText(Info3, "info text 3", "", "", AC_Tag_DIV);
ACText(Info4, "info text 4", "", "", AC_Tag_DIV);
ACText(Info5, "info text 5", "", "", AC_Tag_DIV);
ACText(Info6, "info6", "", "", AC_Tag_DIV);
ACText(Info7, "info7", "", "", AC_Tag_DIV);
ACRadio(Styles, {}, "");
ACInput(SetBoilerTemp,"44", "Температура теплоносителя");

ACSubmit(Apply, "Обновить", HELLO_URI, AC_Tag_DIV);
ACSubmit(SetNewBoilerTemp,"Задать", UPDATE_T_URI, AC_Tag_DIV);
/*********************************/
ACText(St_inf, "<b>OT:</b>", "", "", AC_Tag_DIV);
ACText(St1, "1", "", "", AC_Tag_DIV);
ACText(St2, "2", "", "", AC_Tag_DIV);
ACText(St3, "3", "", "", AC_Tag_DIV);
ACText(St4, "4", "", "", AC_Tag_DIV);
ACText(St5, "5", "", "", AC_Tag_DIV);
ACText(St6, "6", "", "", AC_Tag_DIV);

/*********************************/
ACText(SetTemp_info1, "Заданная температура:", "", "", AC_Tag_DIV);
ACText(SetTemp_info2, "info text 2", "", "", AC_Tag_DIV);
ACSubmit(SetTemp_OK, "Ok", HELLO_URI, AC_Tag_DIV);
/******************************************/
ACText(DebugInfo1, "Debug 1", "", "", AC_Tag_DIV);
ACText(DebugInfo2, "Debug 2", "", "", AC_Tag_DIV);
ACText(DebugInfo3, "Debug 3", "", "", AC_Tag_DIV);
ACText(DebugInfo4, "Debug 4", "", "", AC_Tag_DIV);
ACText(DebugInfo5, "Debug 5", "", "", AC_Tag_DIV);
ACSubmit(DebugApply, "Обновить", DEBUG_URI, AC_Tag_DIV);
/******************************************/

// AutoConnectAux for the custom Web page.
AutoConnectAux helloPage(HELLO_URI, "OpenTherm", true, { Caption, Info1, Info2, Info3, Info4, Info5, Info6, Info7, Styles, Apply, SetBoilerTemp, SetNewBoilerTemp });
AutoConnectAux SetTempPage(UPDATE_T_URI, "SetTemp", false, { SetTemp_info1, SetTemp_info2,  SetTemp_OK});
AutoConnectAux debugPage(DEBUG_URI, "Debug", true, { DebugInfo1, DebugInfo2, DebugInfo3, DebugInfo4, DebugInfo5, Styles, DebugApply});
AutoConnectAux simplePage(SIMPLE_URI, "S", true, { St_inf, St1, St2, St3, St4, St5, St6});

AutoConnectConfig config;
AutoConnect portal;

// JSON document loading buffer
String ElementJson;

/************************************/

void setup_web_common(void);
void loop_web(void);
void onRoot(void);
void loadParam(String fileName);
void onConnect(IPAddress& ipaddr);

String onHello(AutoConnectAux& aux, PageArgument& args);
String onSetTemp(AutoConnectAux& aux, PageArgument& args);
String onDebug(AutoConnectAux& aux, PageArgument& args);
String onS(AutoConnectAux& aux, PageArgument& args);

/************************************/
unsigned int /* AutoConnect:: */ _toWiFiQuality(int32_t rssi);

/************************************/
void setup_web_common(void)
{
//  Serial.println();
   Serial.println("setup_web_common");
  FlashFS.begin(AUTOCONECT_FS_INITIALIZATION);


  helloPage.on(onHello);      // Register the attribute overwrite handler.
  SetTempPage.on(onSetTemp);
  debugPage.on(onDebug);
  simplePage.on(onS);
  portal.join({helloPage, SetTempPage, debugPage, simplePage});     // Join the hello page.
  config.ota = AC_OTA_BUILTIN;
  config.portalTimeout = 1; 
  config.retainPortal = true; 
  //config.autoRise = true;
  // Enable saved past credential by autoReconnect option,
  // even once it is disconnected.
  config.autoReconnect = true;
  config.reconnectInterval = 1;

  portal.config(config);
  portal.onConnect(onConnect);  // Register the ConnectExit function
  Serial.println("2");
  Serial.println();
  portal.begin();
  Serial.println("3");

  WiFiWebServer&  webServer = portal.host();
  Serial.println("4");

  webServer.on("/", onRoot);  // Register the root page redirector.
  Serial.println("5");
//  Serial.println("Web server started:" +WiFi.localIP().toString());
  if (WiFi.status() != WL_CONNECTED)  {
    Serial.println("WiFi Not connected");
  }
  else {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("WiFiOn");
    WiFi.setAutoReconnect(true);
  }  
}


// Load the element from specified file in the flash on board.
void loadParam(String fileName) {
  if (!fileName.startsWith("/"))
    fileName = String("/") + fileName;
  File param = FlashFS.open(fileName.c_str(), "r");
  if (param) {
    ElementJson = param.readString();
    param.close();
  }
  else
    Serial.println("open failed");
}

void onConnect(IPAddress& ipaddr) {
  Serial.print("onConnect:WiFi connected with ");
  Serial.print(WiFi.SSID());
  Serial.print(", IP:");
  Serial.println(ipaddr.toString());
}

// Redirects from root to the hello page.
void onRoot() {
  WiFiWebServer&  webServer = portal.host();
  webServer.sendHeader("Location", String("http://") + webServer.client().localIP().toString() + String(HELLO_URI));
  webServer.send(302, "text/plain", "");
  webServer.client().flush();
  webServer.client().stop();
}

float mRSSi = 0.;
String onDebug(AutoConnectAux& aux, PageArgument& args)
{  char str[80];
//WiFiDebugInfo
   sprintf(str,"WiFi statistics:");
   DebugInfo1.value = str;
   sprintf(str,"%d %d  %d %d  %d %d  %d %d", 
      WiFiDebugInfo[0],WiFiDebugInfo[1],WiFiDebugInfo[2],WiFiDebugInfo[3],WiFiDebugInfo[4],WiFiDebugInfo[5],WiFiDebugInfo[6],WiFiDebugInfo[7]);
   DebugInfo2.value = str;
   sprintf(str,"RSSI: %d dBm (%i%%), среднее за 10 мин %.1f",WiFi.RSSI(),_toWiFiQuality(WiFi.RSSI()), mRSSi);
   DebugInfo3.value = str;
   sprintf(str,"OpenTherm statistics:");
   DebugInfo4.value = str;
   sprintf(str,"%d %d  %d %d  %d %d  %d %d", 
      OTDebugInfo[0], OTDebugInfo[1], OTDebugInfo[2], OTDebugInfo[3], OTDebugInfo[4], OTDebugInfo[5], OTDebugInfo[6],OTDebugInfo[7]);
   DebugInfo5.value = str;
  return String();
}

String onSetTemp(AutoConnectAux& aux, PageArgument& args) {
//String tmp;
//   tmp = args.arg("SetNewBoilerTemp");
  char str[40];
   int v, v1;

    SetBoilerTemp.value.toCharArray(str, 40);
    sscanf(str, "%i %i", &v, &v1);

   v = SetBoilerTemp.value.toInt();
//   args.arg("SetNewBoilerTemp").toInt();
//    SetTemp_info2.value ="hahah";
    SmOT.Tset = v;
    sprintf(str," %.1f",SmOT.Tset);
    SetTemp_info2.value =   str;
    SmOT.need_setT = 1;

    Serial.printf("v=%i\n", v);

  return String();
}


// Load the attribute of th
String onHello(AutoConnectAux& aux, PageArgument& args) {
  char str0[40];
  // Select the style parameter file and load it into the text element.
  AutoConnectRadio& styles = helloPage["Styles"].as<AutoConnectRadio>();
  loadParam(styles.value());

   switch(SmOT.stsOT)
   {  case -1:
       Info1.value =  String(SmOT.stsOT) + " is not initialize";
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
        if(SmOT.BoilerStatus & 0x04)
          Info1.value += "<br>Горячая вода Вкл";
        else  
          Info1.value += "<br>Горячая вода вЫкл";
        if(SmOT.BoilerStatus & 0x08)
          Info1.value += "<br>Горелка Вкл";
        else  
          Info1.value += "<br>Горелка вЫкл";
        if(SmOT.BoilerStatus & 0x40)
          Info1.value += "<br>Diag";

      }
        break;
      case 1:
       Info1.value =  String(SmOT.stsOT) + " Invalid response";
        break;
      case 2:
       Info1.value =  String(SmOT.stsOT) + "  Response timeout";
        break;
   }


    if(SmOT.stsT1 >= 0 || SmOT.stsT2 >= 0)
    {   Info3.value = " Температура помещения ";
        if(SmOT.stsT1 >= 0)
          Info3.value += "T1=" + String(SmOT.t1) + " ";
        if(SmOT.stsT2 >= 0)
          Info3.value += "T2=" + String(SmOT.t2) ;
        Info3.value += "<br>";
    } else {
        Info3.value = "";
    }

  if(SmOT.stsOT != -1)
  {
   Info2.value = " Выходная температура  "  + String(SmOT.BoilerT) + " Обратка " + String(SmOT.RetT) + "<br>";
   Info4.value = " FlameModulation "  + String(SmOT.FlameModulation) + " Pressure" + String(SmOT.Pressure) + "<br>";
   Info5.value = " MaxRelModLevel "  + String(SmOT.MaxRelModLevelSetting) + "<br>" + "Ts="+ String(SmOT.Tset) + "Tsr="+ String(SmOT.Tset_r) + "<br>";

    if(SmOT.OEMDcode || SmOT.Fault)
    {  sprintf(str0, "%x %x", SmOT.Fault, SmOT.OEMDcode);
//      Info5.value = "Fault = " + str0 + "<br>";
      Info6.value = "Fault = ";
      Info6.value += str0;
      Info6.value += "<br>";
    } else {
      Info6.value = "";
    }
    Info7.value = " MinModLevel="  + String(SmOT.MinModLevel) + "<br>"  + " MaxCapacity="  + String(SmOT.MaxCapacity) + "<br>";
  } else {
        Info2.value = "";
        Info4.value = "";
        Info5.value = "";
        Info6.value = "";
        Info7.value = "";
  }
  // List parameter files stored on the flash.
  // Those files need to be uploaded to the filesystem in advance.
  styles.empty();
#if defined(ARDUINO_ARCH_ESP32)
  File  dir = FlashFS.open("/", "r");
  if (dir) {
    File  parmFile = dir.openNextFile();
    while (parmFile) {
      if (!parmFile.isDirectory())
        styles.add(String(parmFile.name()));
      parmFile = dir.openNextFile();
    }
  }
#elif defined(ARDUINO_ARCH_ESP8266)
  Dir dir = FlashFS.openDir("/");
  while (dir.next()) {
    if (!dir.isDirectory())
      styles.add(dir.fileName());
  }
#endif

  // Apply picked style
  helloPage.loadElement(ElementJson);
  return String();
}

//расшифровка битов статуса бойлера для бестолковых
String onS(AutoConnectAux& aux, PageArgument& args)
{   
        St1.value = "";
        if(SmOT.BoilerStatus & 0x01)
          St1.value += "E 1<br>";
        else 
          St1.value += "E 0<br>";

        if(SmOT.BoilerStatus & 0x02)
          St1.value += "H 1<br>";
        else  
          St1.value += "H 0<br>";
        if(SmOT.BoilerStatus & 0x04)
          St1.value += "W 1<br>";
        else  
          St1.value += "W 0<br>";

        if(SmOT.BoilerStatus & 0x08)
          St1.value += "F 1<br>";
        else  
          St1.value += "F 0<br>";

        if(SmOT.BoilerStatus & 0x40)
          St1.value += "D 1<br>";
        else
          St1.value += "D 0<br>";
 
  return String();
}


int sts = 0;
int WiFists = -1;
int sRSSI = 0;
int razRSSI = 0;
extern int LedSts; 

void loop_web()
{  int rc,  dt;
static unsigned long t0=0; // t1=0;

  portal.handleClient();
  if(sts == 0)
  {   Serial.println("loop_web()");
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
  { Serial.printf("WiFi.status=%i\n", rc);
    if(rc == WL_CONNECTED)
    {  LedSts = 0;
 //     digitalWrite(LED_BUILTIN, LedSts);   
      Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
      Serial.printf("(%i%%)\n", _toWiFiQuality(WiFi.RSSI()));
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
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
