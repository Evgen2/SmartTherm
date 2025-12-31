/* Web.cpp - portal/system glue only; pages are in src/Web/ */

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
#include <AutoConnect.h>
#include <AutoConnectFS.h>
#include "OpenTherm.h"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"
#include "Web/Pages.hpp"

AutoConnectFS::FS& FlashFS = AUTOCONNECT_APPLIED_FILESYSTEM;

extern SD_Termo SmOT;
int WiFiDebugInfo[10] ={0,0,0,0,0, 0,0,0,0,0};
unsigned int OTDebugInfo[12] ={0,0,0,0,0, 0,0,0,0,0, 0,0};
extern OpenThermID OT_ids[N_OT_NIDS];
unsigned int OTcount = 0;

AutoConnectConfig config;
AutoConnect portal;

// Forward
void onRoot(void);
void onConnect(IPAddress& ipaddr);
int setup_web_common_onconnect(void);
void check_fs(void);
void loop_web(void);
int OutUTCtime(time_t now);
extern void onOTAstart(void);
extern void exitOTAError(uint8_t err);
#if MQTT_USE
extern void mqtt_loop(void);
extern void mqtt_start(void);
#endif

String utc_time_jc;

void setup_web_common(void) {
  RegisterWebPages(portal);
  portal.onOTAStart(onOTAstart);
  portal.onOTAError(exitOTAError);

  config.ota = AC_OTA_BUILTIN;
  config.portalTimeout = 1;
  config.retainPortal = true;
  config.autoRise = true;
  config.autoReconnect = true;
  config.reconnectInterval = 1;
  config.menuItems = config.menuItems | AC_MENUITEM_DELETESSID;
  Serial_db.printf("WiFi AP SSID %s psk=%s\n", config.apid.c_str(), config.psk.c_str());

  portal.config(config);
  portal.onConnect(onConnect);
  portal.begin();

  WiFiWebServer&  webServer = portal.host();
  webServer.on("/", onRoot);

  if (WiFi.status() != WL_CONNECTED)  {
    Serial_db.printf("WiFi Not connected\n");
    WiFi.setAutoReconnect(true);
  }

#if defined(ARDUINO_ARCH_ESP8266)
  if(WiFi.getMode() == WIFI_OFF) {
    wifi_get_macaddr(STATION_IF, SmOT.Mac);
  } else {
    wifi_get_macaddr(STATION_IF, SmOT.Mac);
  }
#elif defined(ARDUINO_ARCH_ESP32)
  if(WiFi.getMode() == WIFI_MODE_NULL){
    esp_read_mac(SmOT.Mac, ESP_MAC_WIFI_STA);
  }
  else{
    esp_wifi_get_mac(WIFI_IF_STA, SmOT.Mac);
  }
#endif
}

#include "esp_sntp.h"

void time_sync_notification_cb(struct timeval *tv) {
  Serial_db.printf("Time updated, Unix time: %ld\n", tv->tv_sec);
}

int setup_web_common_onconnect(void) {
  static int init = 0;
  int rc;
  Serial_db.printf("WiFi connected, SSID: %s IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  sprintf(SmOT.LocalUrl,"http://%s", WiFi.localIP().toString().c_str());
  Serial_db.printf("WiFi mode = %d\n", WiFi.getMode());
  if(init)
    return 1;

  WiFi.setAutoReconnect(true);

  const char*  const _ntp1 = "europe.pool.ntp.org";
  const char*  const _ntp2 = "pool.ntp.org";
  configTzTime("UTC0", _ntp1 ,_ntp2);
#if defined(ARDUINO_ARCH_ESP32)
  Serial_db.printf("Sync time in ms: %d\n", sntp_get_sync_interval());
#endif
  esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);

#if MQTT_USE
  Serial_db.printf("Read_mqtt_fs:\n");
  rc = SmOT.Read_mqtt_fs();
  SmOT.stsMQTTcfg = rc;
  Serial_db.printf("SmOT.Read_mqtt_fs() rc = %d\n", rc);
#endif

  init = 1;
  return 0;
}

void onConnect(IPAddress& ipaddr) {
  Serial_db.printf("onConnect %s portalStatus = %d\n", ipaddr.toString().c_str(), portal.portalStatus());
  int rc = setup_web_common_onconnect();
  if(rc) {
#if SERIAL_DEBUG
    Serial.print(F("onConnect:WiFi connected with "));
    Serial.print(WiFi.SSID());
    Serial.print(F(", IP:"));
    Serial.println(ipaddr.toString());
#endif
  }
}

// Redirect from root to INFO_URI
void onRoot() {
  WiFiWebServer&  webServer = portal.host();
  webServer.sendHeader("Location", String("http://") + webServer.client().localIP().toString() + String(INFO_URI));
  webServer.send(302, "text/plain", "");
  webServer.client().flush();
  webServer.client().stop();
}

float mRSSi = 0.f;
int WiFists = -1;
extern int LedSts;

void loop_web() {
  int rc = WiFi.status();
  {
    static int oldstatus=-1, oldmode=-1, needStopAP=0;
    static long t0 = 0;
    int mode = WiFi.getMode();
    int ch = WiFi.channel();

    if((rc != oldstatus) || mode != oldmode) {
      Serial_db.printf("WiFi: status=%d (%d) mode = %d chanel=%d  (%d)\n", rc, oldstatus, mode, ch, millis());
      if(rc == WL_CONNECTED &&  (oldstatus == WL_IDLE_STATUS || oldstatus == WL_DISCONNECTED ||  oldstatus == WL_NO_SSID_AVAIL)) {
        needStopAP = 1; t0 = millis();
      }
      oldmode = mode; oldstatus = rc;
    } else if(needStopAP) {
      if(millis()-t0 > 20000) {
        needStopAP = 0;
        if(mode == WIFI_MODE_APSTA) { WiFi.softAPdisconnect(true); WiFi.enableAP(false); }
      }
    }
  }

  portal.handleClient();

  if(rc != WiFists) {
#if SERIAL_DEBUG
    Serial_db.printf("WiFi.status=%i\n", rc);
#endif
    if(rc == WL_CONNECTED) {
      LedSts = 0;
#if SERIAL_DEBUG
      Serial_db.printf((PGM_P)F("RSSI: %d dBm (%i%%)\n"), WiFi.RSSI(),_toWiFiQuality(WiFi.RSSI()));
      Serial.print(F("IP address: "));
      Serial.println(WiFi.localIP());
#endif
    } else {
      Serial_db.printf("WiFi disconnected (sts=%d t %d stsOT %d %d %d)\n", rc, millis(), SmOT.stsOT, SmOT.ns_OT, SmOT.nr_OT);
      LedSts = 1;
    }
    if( rc >=0 && rc <=7) WiFiDebugInfo[rc]++;
    WiFists = rc;
  }

  if(rc ==  WL_CONNECTED) {
    static int sRSSI = 0, razRSSI = 0; static unsigned long t0 = 0; int dt = millis() - t0;
    if(dt > 10000) { t0 = millis(); razRSSI++; sRSSI += WiFi.RSSI(); if(razRSSI > 60) { mRSSi =  float(sRSSI)/float(razRSSI); razRSSI = 0; sRSSI = 0; } }
  }

#if MQTT_USE
  if(rc ==  WL_CONNECTED && (SmOT.useMQTT== 0x03)) mqtt_loop();
#endif
}

unsigned int _toWiFiQuality(int32_t rssi) {
  unsigned int  qu;
  if (rssi == 31) qu = 0; else if (rssi <= -100) qu = 0; else if (rssi >= -50) qu = 100; else qu = 2 * (rssi + 100);
  return qu;
}

int OutUTCtime(time_t now) {
  char str[312]; char buffer[26]; struct tm* tm_info;
  const char *s0 = "<em id=\"utcl\"></em><time id=\"upd_at\" dt=\"";
  const char *s1 = "\"></time><script>";
  const char *s2 =
"const src_el=document.getElementById('upd_at');const d=new Date(src_el.getAttribute('dt')).toLocaleString();document.getElementById(\"utcl\").innerHTML=d;</script>";
  tm_info = localtime(&now);
  strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info); buffer[25] = 0;
  sprintf(str,"%s%sZ%s%s", s0,buffer,s1, s2);
  utc_time_jc = str;
  return 0;
}

void setup_read_config(void) {
  bool b = FlashFS.begin(AUTOCONNECT_FS_INITIALIZATION);
  if(b == false) { Serial.println(F("FlashFS.begin failed")); }
  SmOT.Read_ot_fs();
  SmOT.init(1);
}

void check_fs(void) {
#if defined(ARDUINO_ARCH_ESP32)
  File root = FlashFS.open("/"); File file = root.openNextFile();
  while(file){
#if SERIAL_DEBUG
    Serial.print("FILE: "); Serial_db.printf( "%s %d\n", file.name(), file.size());
#endif
    if(file.size() > 1000000) { char str[80]; sprintf(str,"/%s",file.name()); file.close(); FlashFS.remove(str); break; }
    file = root.openNextFile();
  }
#endif
#if SERIAL_DEBUG
  { int tBytes, uBytes;
#if defined(ARDUINO_ARCH_ESP8266)
    FSInfo info; FlashFS.info(info); tBytes = info.totalBytes; uBytes = info.usedBytes;
#else
    tBytes  = FlashFS.totalBytes(); uBytes = FlashFS.usedBytes();
#endif
    Serial_db.printf("FlashFS tBytes = %d used = %d\n", tBytes, uBytes);
  }
#endif
}
