/* SetTempPage.cpp - /set_t hidden action page */

#include "Shared.hpp"
#include "SmartDebug.h"

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WebServer.h>
using WiFiWebServer = ESP8266WebServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WebServer.h>
using WiFiWebServer = WebServer;
#endif

// Forward declarations
extern AutoConnectConfig config;
extern AutoConnect portal;
extern unsigned long authRealmCounter;

static AutoConnectAux SetTempPage(SET_T_URI, "SetTemp", false, {}, false);

String onSetTemp(AutoConnectAux& aux, PageArgument& args)
{  
  Serial_db.printf("[onSetTemp] Handler called, URI: %s\n", SET_T_URI);
  
  // ВАЖНО: Для скрытых страниц (responsive=false) AutoConnect НЕ применяет аутентификацию автоматически
  // Нужно проверять вручную перед обработкой запроса
  if (config.auth != AC_AUTH_NONE && config.username.length() > 0) {
    WiFiWebServer& ws = portal.host();
    Serial_db.printf("[onSetTemp] Checking authentication, username: %s\n", config.username.c_str());
    if (!ws.authenticate(config.username.c_str(), config.password.c_str())) {
      Serial_db.printf("[onSetTemp] Authentication FAILED, requesting authentication\n");
      // Для responsive=false страниц используем requestAuthentication() вместо прямого send()
      // Это правильный способ для AutoConnect
      ws.requestAuthentication();
      return String(); // Отменяем обработку запроса
    }
    Serial_db.printf("[onSetTemp] Authentication OK\n");
  }
  
  Serial_db.printf("[onSetTemp] Processing form data...\n");
  float  v;
   int isChange=0;
    if(SmOT.enable_CentralHeating)
    { if(SetBoilerTemp.enable)
      { v = SmOT.CHtempLimit(SetBoilerTemp.value.toFloat());
        if(v != SmOT.Tset)
        { isChange = 1; SmOT.Tset = v; SmOT.need_set_T(1); }
      }
    }
    if(SmOT.enable_HotWater)
    { v = SmOT.CHtempLimit(SetDHWTemp.value.toFloat());    
      if(v != SmOT.TdhwSet)
      { isChange = 1; SmOT.TdhwSet = v; SmOT.need_set_dhwT(1);
        if(SmOT.CH2_present && (ot.OTid_used(OpenThermMessageID::TflowCH2) && SmOT.CH2_DHW_flag))
        { SmOT.Tset2 = SmOT.TdhwSet; SmOT.need_set_T_CH2(1); }
      }
    }
    if(SmOT.enable_CentralHeating2)
    { v = SmOT.CHtempLimit(SetBoilerTemp2.value.toFloat());
      if(v != SmOT.Tset2) {  isChange = 1; SmOT.Tset2 = v; SmOT.need_set_T_CH2(1); }
    }
    if(isChange) SmOT.need_write_f = 1;
  Serial_db.printf("[onSetTemp] Processing complete: isChange=%d, redirecting to INFO_URI\n", isChange);
  aux.redirect(INFO_URI);
  return String();
}

void Register_SetTemp(AutoConnect& portal){ 
  // Аутентификация применяется автоматически через config.authScope (AC_AUTHSCOPE_AUX)
  SetTempPage.on(onSetTemp); 
  portal.join({SetTempPage}); 
}
