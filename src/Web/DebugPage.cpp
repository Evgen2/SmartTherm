/* DebugPage.cpp - /debug page */

#include "Shared.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp32/rom/rtc.h"
#endif

static AutoConnectSubmit DebugApply("DebugApply", "Обновить", DEBUG_URI, AC_Tag_DIV);
static AutoConnectAux debugPage(DEBUG_URI, "Debug", true);

String onDebug(AutoConnectAux& aux, PageArgument& args)
{  char str[256];
  extern int minRamFree;
   Info1.value = F("WiFi statistics:");
   sprintf(str,(PGM_P)F("%d %d  %d %d  %d %d  %d %d"),
      WiFiDebugInfo[0],WiFiDebugInfo[1],WiFiDebugInfo[2],WiFiDebugInfo[3],WiFiDebugInfo[4],WiFiDebugInfo[5],WiFiDebugInfo[6],WiFiDebugInfo[7]);
   Info2.value = str;
   if(WiFists == WL_CONNECTED)
   {  sprintf(str,(PGM_P)F("RSSI: %d dBm (%i%%), среднее за 10 мин %.1f"),WiFi.RSSI(),_toWiFiQuality(WiFi.RSSI()), mRSSi);
      Info3.value = str;
   } else  Info3.value = "";
   sprintf(str,(PGM_P)F("OpenTherm statistics:<br>%d %d  %d %d  % d %d  %d %d  %d %d  %d"),
      OTDebugInfo[0], OTDebugInfo[1], OTDebugInfo[2], OTDebugInfo[3], OTDebugInfo[4], OTDebugInfo[5], OTDebugInfo[6],OTDebugInfo[7], OTDebugInfo[8],OTDebugInfo[9], OTDebugInfo[10]);
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
   if(SmOT.CrasyState_count > 0)
   {  sprintf(str," CSc %d", SmOT.CrasyState_count);
      Info5.value += str;
   }
  {       
   OutUTCtime(time(nullptr));
    Info5.value += (PGM_P)F("<br>Время:");
    Info5.value += utc_time_jc;
  }
   snprintf(str,sizeof(str),(PGM_P)F("Вкл горелки:<br>Всего %d<br>За час %d<br>Пред.час %d<br>Сутки %d<br>Пред.сутки %d"), 
          SmOT.Bstat.NflameOn, SmOT.Bstat.NflameOn_h, SmOT.Bstat.NflameOn_h_prev, SmOT.Bstat.NflameOn_day, SmOT.Bstat.NflameOn_day_prev);
   Info6.value = str;
   snprintf(str,sizeof(str),(PGM_P)F("<br>Эффективная модуляция:<br>За час %.2f<br>Пред.час %.2f<br>Сутки %.2f<br>Пред.сутки %.2f"), 
          SmOT.Bstat.Eff_Mod_h, SmOT.Bstat.Eff_Mod_h_prev, SmOT.Bstat.Eff_Mod_d, SmOT.Bstat.Eff_Mod_d_prev);
   Info6.value += str;
#if PID_USE
    if(SmOT.usePID)
    { 
       sprintf(str,"<br>pid: U= %f u0 = %f  dP= %f, dD= %f dI= %f",
        SmOT.mypid.u, SmOT.mypid.ub, SmOT.mypid.dP, SmOT.mypid.dD, SmOT.mypid.dI); 
      Info6.value += str;
      sprintf(str,"<br>RoomSetpoint change src %d from %f to %f at ", 
      SmOT.src_lastSetPointChange, SmOT.oldTroomSetpoint, SmOT.mypid.xTag); 
      Info6.value += str;
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
#if defined(ARDUINO_ARCH_ESP32)
  extern unsigned short int _bootCount, _bootReason, _bootSts, _bootSts1, _bootSts2;
  extern unsigned short int bootSts2;
  sprintf(str,"reset reason: %d %d (%d %d %d %d %d|%d)", rtc_get_reset_reason(0), rtc_get_reset_reason(1),
      _bootCount, _bootReason, _bootSts, _bootSts1, _bootSts2, bootSts2);
  Info7.value = str;
#endif
#if MQTT_USE
  sprintf(str,"<br>stsMQTTcfg %d useMQTT %d stsMQTT %d", SmOT.stsMQTTcfg, SmOT.useMQTT, SmOT.stsMQTT );
  Info7.value += str;
#endif
  return String();
}

void Register_Debug(AutoConnect& portal){
  // Аутентификация применяется автоматически через config.authScope (AC_AUTHSCOPE_AUX)
  debugPage.on(onDebug);
  debugPage.add(Info1);
  debugPage.add(Info2);
  debugPage.add(Info3);
  debugPage.add(Info4);
  debugPage.add(Info5);
  debugPage.add(Info6);
  debugPage.add(Info7);
  debugPage.add(DebugApply);
  portal.join({debugPage});
}
