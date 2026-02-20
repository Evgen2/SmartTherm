/* InfoPage.cpp - /info page */

#include "Shared.hpp"

// Info page instance
static AutoConnectAux InfoPage(INFO_URI, "SmartTherm", true);

// Handler copied from original with minor includes
String onInfo(AutoConnectAux& aux, PageArgument& args) {
  char str0[256];
  // OpenTherm instance declared in Shared.hpp
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
        {   snprintf(str0,sizeof(str0), (PGM_P)F("Потеря связи с котлом %.f сек назад"), dt);
        } else {        
            snprintf(str0, sizeof(str0), (PGM_P)F("Потеря связи связи с котлом %.1f час(ов) назад"), dt);
        }
        Info1.value =  String(SmOT.stsOT) + " : <b>Ошибка:</b> ";
        Info1.value +=  str0;
      }
        break;
   }

#if MQTT_USE
  if(SmOT.useMQTT)
  {  extern int statemqtt; extern int state_mqtt;
     Info1.value += "<br>";
     switch(statemqtt)
     { case -1: Info1.value += "MQTT not connected"; if(SmOT.stsMQTT == 0) Info1.value += ", ожидание опроса OT"; break;
       case 0:  Info1.value += "MQTT DiSconnected"; break;
       case 1:  Info1.value += "MQTT connected"; break; }
     switch(state_mqtt)
     { case -1: Info1.value += " disconnected"; break; case -2: Info1.value += " Connect failed"; break;
       case -3: Info1.value += " Connection lost"; break; case -4: Info1.value += " Connection timeout"; break;
       case 1: Info1.value += " Bad protocol"; break; case 2: Info1.value += " Bad client id"; break;
       case 3: Info1.value += " unavailable"; break; case 4: Info1.value += " Bad credentials"; break;
       case 5: Info1.value += " Unauthorized"; break; }
  }
  Info1.value += "<br>";
  if(!SmOT.useMQTT) { if(SmOT.CapabilitiesDetected == 0) Info1.value += "Тест котла"; }
#else
  if(SmOT.CapabilitiesDetected == 0) Info1.value += "Тест котла";
#endif

#if PID_USE
  if(SmOT.usePID && SmOT.enable_CentralHeating)
  {   Info1.value += "управление по PID";
      if(SmOT.usePID & 0x02) Info1.value += "без ограничений";
      Info1.value += " Tindoor " + String(SmOT.tempindoor) + " ";
      Info1.value += " Toutdoor " + String(SmOT.tempoutdoor);
  }
#endif

  if(SmOT.stsT1 >= 0 || SmOT.stsT2 >= 0)
  {   Info3.value = " Температура ";
      if(SmOT.stsT1 >= 0)
      { if(SmOT.stsT1 == 4)      Info3.value += "T1 Disconnect ";
        else if(SmOT.stsT1 == 2) Info3.value += "T1 Crc Err ";
        else                      Info3.value += "T1 " + String(SmOT.t1) + " "; }
      if(SmOT.stsT2 >= 0)
      { if(SmOT.stsT2 == 4)      Info3.value += "T2 Disconnect ";
        else if(SmOT.stsT2 == 2) Info3.value += "T2 Crc Err ";
        else                      Info3.value += "T2 " + String(SmOT.t2) + " "; }
      Info3.value += "<br>";
  } else Info3.value = "";

  if(ot.OTid_used(OpenThermMessageID::Toutside)) Info3.value += "Text " + String(SmOT.Toutside) + "<br>";

  if(SmOT.stsOT != -1)
  {
    Info2.value = " Выходная температура  "  + String(SmOT.BoilerT);
    if(ot.OTid_used(OpenThermMessageID::Tret)) Info2.value +=  " Обратка " + String(SmOT.RetT);
    if(ot.OTid_used(OpenThermMessageID::Texhaust)) { sprintf(str0," Выхлоп %.0f", SmOT.Texhaust); Info2.value +=  str0; }

    if(SmOT.Use_ID29_DHW_flag && ot.OTid_used(OpenThermMessageID::Tstorage))
      Info2.value +=  " Бойлер " + String(SmOT.Tstorage);
    else if(SmOT.HotWater_present) {
      if(SmOT.enable_HotWater) {
        if(SmOT.Dhw_t_present) Info2.value +=  "<br>Горячая вода " + String(SmOT.dhw_t);
        if(SmOT.DHWFlowRate_present && SmOT.DHWFlowRate > 0.f) Info2.value += " Расход "  + String(SmOT.DHWFlowRate);
      }
    }

    Info2.value += "<br>";
    Info4.value = "";
    if(ot.OTid_used(OpenThermMessageID::RelModLevel)) Info4.value += " Flame "  + String(SmOT.FlameModulation);
    if(ot.OTid_used(OpenThermMessageID::CHPressure))  Info4.value += " Pressure " + String(SmOT.Pressure);
    Info4.value += "<br>";
    if((SmOT.enable_CentralHeating2) || (SmOT.CH2_present && ot.OTid_used(OpenThermMessageID::TflowCH2) && SmOT.CH2_DHW_flag))
      Info4.value += "T CH2 " +  String(SmOT.BoilerT2) + "<br>";

    Info5.value = "Ts "+ String(SmOT.Tset) + " Tsr "+ String(SmOT.Tset_r) + "<br>";

    if(SmOT.OEMDcode || SmOT.Fault)
    {  sprintf(str0, "%x %x", SmOT.Fault, SmOT.OEMDcode);
       Info6.value  = "";
       if(SmOT.Fault)
       { sprintf(str0, "Fault = %x (HB) %x (LB)<br>", (SmOT.Fault>>8)&0xff, (SmOT.Fault&0xff)); Info6.value += str0;
         if(SmOT.Fault & 0xff00)
         { if(SmOT.Fault & 0x0100) Info6.value += " Service request";
           if(SmOT.Fault & 0x0200) Info6.value += " Lockout-reset";
           if(SmOT.Fault & 0x0400) Info6.value += " LowWater press";
           if(SmOT.Fault & 0x0800) Info6.value += " Gas/flame fault";
           if(SmOT.Fault & 0x01000) Info6.value += " Air press fault";
           if(SmOT.Fault & 0x02000) Info6.value += " Water over-temp fault";
           if(SmOT.Fault & 0x00ff) Info6.value += " &"; }
         if(SmOT.Fault & 0x00ff)
         { sprintf(str0, (PGM_P)F(" OEM-specific fault/error cod = %d (hex %x)"), (SmOT.Fault&0xff), (SmOT.Fault&0xff)); Info6.value += str0; }
         Info6.value += "<br>";
       }
       if(SmOT.OEMDcode)
       { sprintf(str0, (PGM_P)F("OEM-specific diagnostic/service code = %d (hex %x)<br>"), SmOT.OEMDcode, SmOT.OEMDcode); Info6.value += str0; }
    } else Info6.value = "";

    if(OTDebugInfo[0] > 10)
    { int v =  (OTDebugInfo[3] + OTDebugInfo[4])*100/OTDebugInfo[0]; 
      if(v > 30) { sprintf(str0, "Большое количество ошибок OpenTherm: %d%%<br>", v); Info6.value += str0; } }

    Info7.value = "";

#if PID_USE
    if(SmOT.enable_CentralHeating && !SmOT.usePID) SetBoilerTemp.enable = true;
#else
    if(SmOT.enable_CentralHeating) SetBoilerTemp.enable = true;
#endif
    else SetBoilerTemp.enable = false;

    if(SmOT.CH2_present && SmOT.enable_CentralHeating2) SetBoilerTemp2.enable = true; else SetBoilerTemp2.enable = false;
    if(SmOT.enable_HotWater) SetDHWTemp.enable = true; else SetDHWTemp.enable = false;
    if(SmOT.enable_HotWater || SmOT.enable_CentralHeating||SmOT.enable_CentralHeating2) SetNewBoilerTemp.enable = true; else SetNewBoilerTemp.enable = false;
    if(SetBoilerTemp.enable){ sprintf(str0,"%.1f",SmOT.Tset); SetBoilerTemp.value = str0; }
    if(SetDHWTemp.enable){ sprintf(str0,"%.1f",SmOT.TdhwSet); SetDHWTemp.value = str0; }
    if(SetBoilerTemp2.enable){ sprintf(str0,"%.1f",SmOT.Tset2); SetBoilerTemp2.value = str0; }
  } else {
    Info2.value = ""; Info4.value = ""; Info5.value = ""; Info6.value = ""; Info7.value = "";
  }

#if ST_VERS == 2
  if(SmOT.OT_slave_present)
  {
    Info7.value = "OT2: ";
    switch(SmOT.ot_slave_stsOT)
    {   case -2:
        case -1: Info7.value += "<b>Ошибка:</b> не инициализирован"; break;
        case 0:  Info7.value += "работает"; break;
        case 2:
        {  time_t now = time(nullptr); double dt = difftime(now,SmOT.ot_slave_t_lastwork);
           if(dt < 3600.) sprintf(str0, (PGM_P)F("Потеря связи %.f сек назад"), dt);
           else          sprintf(str0, (PGM_P)F("Потеря связи связи  %.1f час(ов) назад"), dt);
           Info7.value +=  str0; }
          break; }
    if((SmOT.OT_slave_mode == 1) && (SmOT.ot_slave_stsOT == 0))
    {   SetDHWTemp.enable = false;
        SetBoilerTemp2.enable = false;
        SetBoilerTemp.enable = false;
        SetNewBoilerTemp.enable = false;
        Info7.value +=  ", управление от панели";
    } else 
          Info7.value +=  ", управление от контроллера";
  }
#endif

#if  RELAY_USE
  if(SmOT.Relay_present)
  { RelayOnFf.enable = true; if(SmOT.Relay_sts) strcpy(str0,"Реле вЫкл"); else strcpy(str0,"Реле Вкл"); RelayOnFf.value = str0; }
  else RelayOnFf.enable = false;
#endif

  return String();
}

void Register_Info(AutoConnect& portal) {
  // Аутентификация применяется автоматически через config.authScope (AC_AUTHSCOPE_AUX)
  InfoPage.on(onInfo);
  InfoPage.add(Caption);
  InfoPage.add(Info1);
  InfoPage.add(Info2);
  InfoPage.add(Info3);
  InfoPage.add(Info4);
  InfoPage.add(Info5);
  InfoPage.add(Info6);
  InfoPage.add(Info7);
#if RELAY_USE
  InfoPage.add(RelayOnFf);
#endif
  InfoPage.add(Apply);
  InfoPage.add(SetBoilerTemp);
  InfoPage.add(SetDHWTemp);
  InfoPage.add(SetBoilerTemp2);
  InfoPage.add(SetNewBoilerTemp);
  portal.join({InfoPage});
}
