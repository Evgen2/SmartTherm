/* SetParPage.cpp - /set_par hidden action page */

#include "Shared.hpp"
#include "SetupControls.hpp"
#include "SmartDebug.h"

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WebServer.h>
using WiFiWebServer = ESP8266WebServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WebServer.h>
using WiFiWebServer = WebServer;
#endif

// Forward declarations for web auth controls
extern AutoConnectText InfoAuth;
extern AutoConnectInput SetWebAuthUser;
extern AutoConnectInput SetWebAuthPwd;
extern AutoConnectConfig config;
extern AutoConnect portal;
extern unsigned long authRealmCounter;

#if MQTT_USE
extern void mqtt_start(void);
#endif

static AutoConnectAux SetParPage(SET_PAR_URI, "SetPar", false, {}, false);

String onSetPar(AutoConnectAux& aux, PageArgument& args)
{ 
  Serial_db.printf("[onSetPar] ========== HANDLER CALLED ==========\n");
  Serial_db.printf("[onSetPar] Handler called, URI: %s\n", SET_PAR_URI);
  Serial_db.printf("[onSetPar] Request method: checking...\n");
  
  // ВАЖНО: Для скрытых страниц (responsive=false) AutoConnect НЕ применяет аутентификацию автоматически
  // Нужно проверять вручную перед обработкой запроса
  if (config.auth != AC_AUTH_NONE && config.username.length() > 0) {
    WiFiWebServer& ws = portal.host();
    Serial_db.printf("[onSetPar] Checking authentication, username: %s\n", config.username.c_str());
    if (!ws.authenticate(config.username.c_str(), config.password.c_str())) {
      Serial_db.printf("[onSetPar] Authentication FAILED, requesting authentication\n");
      // Для responsive=false страниц используем requestAuthentication() вместо прямого send()
      // Это правильный способ для AutoConnect
      ws.requestAuthentication();
      return String(); // Отменяем обработку запроса
    }
    Serial_db.printf("[onSetPar] Authentication OK\n");
  }
  
  Serial_db.printf("[onSetPar] Processing form data...\n");
  int isChange=0,  redir = 0, iv; float fv; bool check;
  if( CtrlChB1.checked) check = true; else check = false;
  if(check != SmOT.enable_CentralHeating)
  { isChange++; SmOT.enable_CentralHeating = check;
#if PID_USE
    if(SmOT.usePID && !SmOT.enable_CentralHeating) { SmOT.usePID = 0; }
#endif
  }
  if( CtrlChB2.checked) check = true; else check = false;
  if(check != SmOT.enable_HotWater) { isChange++; SmOT.enable_HotWater = check; }
  if(SmOT.CH2_present) { if( CtrlChB3.checked) check = true; else check = false;
    if(check != SmOT.enable_CentralHeating2) { isChange++; SmOT.enable_CentralHeating2 = check; } }
  if( CtrlChB_UseRemoteControl.checked) check = true; else check = false;
  if(check != SmOT.Use_remoteTCPserver) { isChange++; SmOT.Use_remoteTCPserver = check; SmOT.init(2); }

  fv = SmOT.CHtempLimit(SetTmaxPID.value.toFloat());    
  if(fv != SmOT.umax) { SmOT.umax = fv; SmOT.need_set_MaxTSet(2); isChange = 1; }
  fv = SmOT.CHtempLimit(SetTminPID.value.toFloat());    
  if( fv > SmOT.umax - 1.)  fv = SmOT.umax -1.;
  if(fv != SmOT.umin) { SmOT.umin = fv; isChange = 1; }

#if MQTT_USE
  int isChangeMQTT = 0;
  if( CtrlChbUseMQTT.checked) check = true; else check = false;
  if(check)
  { if(SmOT.useMQTT == 0) { SmOT.useMQTT = 1; redir = 1; }
    else if(SmOT.useMQTT == 1) { SmOT.useMQTT = 0x3; isChangeMQTT++; }
  } else { if(SmOT.useMQTT != 0) { SmOT.useMQTT = 0; isChangeMQTT++; } }
  if(SmOT.useMQTT && redir== 0)
  {   char str0[80]; int i;
      SetMQTT_server.value.toCharArray(str0, sizeof(str0)); if(strcmp(SmOT.MQTT_server,str0)){ isChangeMQTT++; strcpy(SmOT.MQTT_server,str0);}    
      SetMQTT_user.value.toCharArray(str0, sizeof(str0));   if(strcmp(SmOT.MQTT_user,str0)){ isChangeMQTT++; strcpy(SmOT.MQTT_user,str0);}      
      SetMQTT_pwd.value.toCharArray(str0, sizeof(str0));    if(strcmp(SmOT.MQTT_pwd,str0)) { isChangeMQTT++; strcpy(SmOT.MQTT_pwd,str0);}       
      SetMQTT_devname.value.toCharArray(str0, sizeof(str0));if(strcmp(SmOT.MQTT_devname,str0)) { isChangeMQTT++; strcpy(SmOT.MQTT_devname,str0);} 
      SetMQTT_topic.value.toCharArray(str0, sizeof(str0));  for(i=0; str0[i]; i++)
      {  if((str0[i]>='0' && str0[i]<='9')||(str0[i]>='A' && str0[i]<='Z')||(str0[i]>='a' && str0[i]<='z')||str0[i]=='_'||str0[i]=='-') continue; str0[i]=0; break; }
      if(strcmp(SmOT.MQTT_topic,str0)) { isChangeMQTT++; strcpy(SmOT.MQTT_topic,str0);}      
      iv = SetMQTT_interval.value.toInt(); if((unsigned int) iv !=SmOT.MQTT_interval ){ isChangeMQTT++; SmOT.MQTT_interval = iv; }
      iv = SetMQTT_port.value.toInt();     if((unsigned int) iv !=SmOT.MQTT_port )    { isChangeMQTT++; SmOT.MQTT_port = iv; }
  }
#endif
  // Save web authentication credentials
  { char str0[32];
    SetWebAuthUser.value.toCharArray(str0, sizeof(str0));
    bool username_changed = false;
    bool password_changed = false;
    
    if(strcmp(SmOT.web_auth_username, str0)) {
      strncpy(SmOT.web_auth_username, str0, sizeof(SmOT.web_auth_username)-1);
      SmOT.web_auth_username[sizeof(SmOT.web_auth_username)-1] = 0;
      isChange++;
      username_changed = true;
    }
    SetWebAuthPwd.value.toCharArray(str0, sizeof(str0));
    if(strcmp(SmOT.web_auth_password, str0)) {
      strncpy(SmOT.web_auth_password, str0, sizeof(SmOT.web_auth_password)-1);
      SmOT.web_auth_password[sizeof(SmOT.web_auth_password)-1] = 0;
      isChange++;
      password_changed = true;
    }
    
    // Update AutoConnect config with new credentials if either username or password changed
    if (username_changed || password_changed) {
      config.username = SmOT.web_auth_username;
      config.password = SmOT.web_auth_password;
      Serial_db.printf("[onSetPar] Web authentication credentials updated: username=%s, password=%s\n", 
                        config.username.c_str(), config.password.c_str());
      // Сохраняем веб-аутентификацию в отдельный файл
      SmOT.Write_web_auth_fs();
      // Применяем новую конфигурацию к работающему порталу без перезагрузки
      portal.config(config);
      // Меняем realm, чтобы браузер запросил новые учётные данные при следующем запросе
      authRealmCounter++;
      Serial_db.printf("[onSetPar] portal.config() applied, authRealmCounter=%lu, new credentials active\n", (unsigned long)authRealmCounter);
    }
  }

#if RELAY_USE
  if( CtrlChBUseRelay.checked) check = true; else check = false;
  if(check != SmOT.Relay_present)
  { isChange++; if(check && CtrlChBStartRelaySts.enable == false) redir = 1; SmOT.Relay_present = check; }
  if(CtrlChBStartRelaySts.enable == true)
  { if(CtrlChBStartRelaySts.checked) check = true; else check = false;
    if(check != SmOT.Relay_init_sts) {  isChange++; SmOT.Relay_init_sts = check; }
  }
#endif

  if(SmOT.MaxRelModLevel_present)
  {   if(CtrlChBMmod.checked) check = true; else check = false;
      if(SmOT.Use_MaxRelModLevel)
      { if(!check) {   SmOT.Use_MaxRelModLevel = 0; isChange++; }
        else { iv = SetMaxMod.value.toInt(); if(iv != int(SmOT.MaxRelModLevelSetting+0.5)) { isChange++; SmOT.MaxRelModLevelSetting = (float)iv; SmOT.need_set_MaxRelModLevel(2);} }
      } else { if(check) {   SmOT.Use_MaxRelModLevel = 1; redir = 1; } else SmOT.Use_MaxRelModLevel = 0; }
  }

  if(isChange) SmOT.need_write_f = 1;
#if MQTT_USE
  if(isChangeMQTT) { if(SmOT.useMQTT == 0x03) mqtt_start(); SmOT.need_write_f |= 0x2; }
#endif
  if(SmOT.enable_CentralHeating) SmOT.need_set_T(1);
  if(SmOT.HotWater_present && SmOT.enable_HotWater) {
    SmOT.need_set_dhwT(1);
    if(SmOT.CH2_present && (ot.OTid_used(OpenThermMessageID::TflowCH2) && SmOT.CH2_DHW_flag))
    { SmOT.Tset2 = SmOT.TdhwSet; SmOT.need_set_T_CH2(1); }
  }
  if(SmOT.CH2_present && SmOT.enable_CentralHeating2) SmOT.need_set_T_CH2(1);

  Serial_db.printf("[onSetPar] Processing complete: isChange=%d, redir=%d\n", isChange, redir);
  if(redir) {
    Serial_db.printf("[onSetPar] Redirecting to SETUP_URI\n");
    aux.redirect(SETUP_URI);
  } else {
    Serial_db.printf("[onSetPar] Redirecting to INFO_URI\n");
    aux.redirect(INFO_URI);
  }
  return String();
}

void Register_SetPar(AutoConnect& portal){ 
  // Аутентификация применяется автоматически через config.authScope (AC_AUTHSCOPE_AUX)
  Serial_db.printf("[Register_SetPar] Registering SetParPage with URI: %s\n", SET_PAR_URI);
  SetParPage.on(onSetPar); 
  portal.join({SetParPage}); 
  Serial_db.printf("[Register_SetPar] SetParPage registered successfully\n");
}
