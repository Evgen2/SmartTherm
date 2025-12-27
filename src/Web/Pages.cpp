/* Pages.cpp - extracted AutoConnect pages and handlers from Web.cpp */

#include <time.h>
#include <AutoConnect.h>
#include <AutoConnectFS.h>
#include "OpenTherm.h"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp32/rom/rtc.h"
#endif

// Externs provided by the main Web.cpp
extern SD_Termo SmOT;
extern OpenThermID OT_ids[N_OT_NIDS];
extern unsigned int OTDebugInfo[12];
extern String utc_time_jc;
int OutUTCtime(time_t now);
// Provided by Web.cpp
unsigned int _toWiFiQuality(int32_t rssi);

#if MQTT_USE
extern void mqtt_start(void);
extern int MQTT_pub_usePID(void);
#endif

// URIs
extern const char INFO_URI[]  = "/info"; // give external linkage so Web.cpp can link
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

static const char* STYLE_WIDTH = "width:15%";

// Controls
ACText(Caption, "<b>Статус OT: </b>", "", "", AC_Tag_DIV);
ACText(Info1, "", "", "", AC_Tag_DIV);
ACText(Info2, "", "", "", AC_Tag_DIV);
ACText(Info3, "", "", "", AC_Tag_DIV);
ACText(Info4, "", "", "", AC_Tag_DIV);
ACText(Info5, "", "", "", AC_Tag_DIV);
ACText(Info6, "", "", "", AC_Tag_DIV);
ACText(Info7, "", "", "", AC_Tag_DIV);
ACInput(SetBoilerTemp,"", "Температура теплоносителя:<br>", "", "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);
ACInput(SetDHWTemp,   "", "Температура горячей воды:<br>", "",  "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);
ACInput(SetBoilerTemp2,"", "Температура CH2:<br>");

#if RELAY_USE
ACSubmit(RelayOnFf, "Реле вкл/выкл", RELAY_URI, AC_Tag_None);
#endif
ACSubmit(Apply, "Обновить", INFO_URI, AC_Tag_BR);
ACSubmit(SetNewBoilerTemp,"Задать", SET_T_URI, AC_Tag_DIV);

// Setup page controls
ACText(Ctrl2, "", "", "", AC_Tag_DIV);
AutoConnectCheckbox CtrlChB1("CtrlChB1","1", "Отопление", false, AC_Behind , AC_Tag_BR);
AutoConnectCheckbox CtrlChB2("CtrlChB2","2", "Горячая вода", false, AC_Behind , AC_Tag_DIV);
AutoConnectCheckbox CtrlChB3("CtrlChB3","3", "Отопление CH2", false, AC_Behind , AC_Tag_DIV);
ACInput(SetMaxMod,"", "проценты","",  "0-100%",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
AutoConnectCheckbox CtrlChBMmod("CtrlChBmmod","4", "Макс модуляция", false, AC_Behind , AC_Tag_BR);
#if RELAY_USE
AutoConnectCheckbox CtrlChBUseRelay("ChbUseRelay","5", "Реле", false, AC_Behind , AC_Tag_None);
AutoConnectCheckbox CtrlChBStartRelaySts("ChbStartRelay","6", "Вкл при старте", false, AC_Behind , AC_Tag_BR);
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
ACInput(SetTmaxPID,"", "Tmax:","","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
ACInput(SetTminPID,"", "Tmin:","","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
AutoConnectCheckbox CtrlChB_UseRemoteControl("CtrlChB5","5", "Разрешить удаленное управление", false, AC_Behind , AC_Tag_DIV);
ACSubmit(ApplyChB, "Задать", SET_PAR_URI, AC_Tag_DIV);
ACSubmit(ApplyAdd, "Дополнительно", SETUP_ADD_URI, AC_Tag_None);

// Setup addition controls
AutoConnectCheckbox UseID2ChB("UseID2ChB","", "Использовать OT ID2", false, AC_Behind , AC_Tag_None);
ACInput(ID2MaserID,"", "IDcode","", "", AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);
AutoConnectCheckbox UseOTC_ChB("UseOTC_ChB","", "Использовать OTC (ID0:HB3)",         false,   AC_Behind, AC_Tag_BR);
AutoConnectCheckbox UseCH2_DHW_ChB("UseCH2DHW","", "Использовать CH2 для горячей воды (ID0:HB4)", false, AC_Behind, AC_Tag_BR);
AutoConnectCheckbox UseWinterModeChB("UseWinterModeChB","", "Режим «лето/зима» (ID0:HB5)", false,   AC_Behind, AC_Tag_BR);
AutoConnectCheckbox UseID29_DHW_ChB("UseID29DHW","", "Использовать ID29 для температуры бойлера", false, AC_Behind, AC_Tag_BR);
AutoConnectCheckbox Immergas_fix_ChB("Immergas","", "Immergas fix", false, AC_Behind, AC_Tag_BR);
AutoConnectCheckbox UseCPU_FREQ_ChB("CPUFREQ","", "CPU FREQ", false, AC_Behind, AC_Tag_None);
ACInput(CPU_FREQ,"", " ","", "", AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
ACSubmit(ApplyAddpar,   "Задать", SET_ADD_URI, AC_Tag_BR);
ACSubmit(SendBLOR, "Сброс ошибки", BLOR_URI, AC_Tag_BR);
// Debug page has its own refresh button in original sketch
ACSubmit(DebugApply, "Обновить", DEBUG_URI, AC_Tag_DIV);

#if PID_USE
// PID controls
AutoConnectCheckbox UsePID("UsePID","", "Использовать PID", false, AC_Behind , AC_Tag_BR);
AutoConnectCheckbox UsePID_NoLimit("UsePID_NOLIMIT","", "Не ограничивать уставку (5-35°C)", false, AC_Behind , AC_Tag_BR);
ACInput(SetXtagPID,"", "Уставка температуры в помещении:", "",  "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);
ACInput(SetTempSrcPID,"", "Источник температуры в помещении:", "",  "",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH);
ACInput(SetTempExtSrcPID,"", "Источник температуры на улице:", "",  "у",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH);
ACInput(SetKpPID,  "", "Kp:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACInput(SetKdPID,  "", "Kd:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACInput(SetKiPID,  "", "Ki:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACInput(SetIdissPID,"","Idiss:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACInput(Set_u0_PID,"", "u0:",  "","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
ACInput(Set_t0_PID,"", "t0:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACInput(Set_u1_PID,"", "u1:",  "","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
ACInput(Set_t1_PID,"", "t1:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACInput(Set_x0_PID,"", "Базовая температура помещения:",  "", "", AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACInput(Set_CH_GIST,"", "Гистерезис включения горелки, град:",  "", "", AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
AutoConnectCheckbox UsePIDPWM("UsePIDPWM","", "Использовать PWM при выходе U &lt; Tmin", false, AC_Behind , AC_Tag_BR);
ACInput(PWM_T_PID,"", "Время PWM, сек:",  "", "", AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
ACSubmit(ApplyPID,   "Задать", SET_PID_URI, AC_Tag_BR);
#endif

#if ST_VERS == 2
AutoConnectCheckbox UseOTslave("UseOTslave","", "Использовать OT slave интерфейс", false, AC_Behind ,  AC_Tag_DIV);
AutoConnectRadio OTslaveMode("radio", { "SmartTherm", "Панель" }, "Котлом управляет:", AC_Vertical, 1,  AC_Tag_DIV);
ACSubmit(ApplySlave,   "Задать", SET_OT2_URI, AC_Tag_BR);
#endif

// About page caption
ACText(About_0, "<b>About:</b>", "", "", AC_Tag_DIV);

// Pages
#if RELAY_USE
AutoConnectAux InfoPage(INFO_URI, "SmartTherm", true, { Caption, Info1, Info2, Info3, Info4, Info5, Info6, Info7, RelayOnFf,  Apply, SetBoilerTemp, SetDHWTemp, SetBoilerTemp2, SetNewBoilerTemp });
#else
AutoConnectAux InfoPage(INFO_URI, "SmartTherm", true, { Caption, Info1, Info2, Info3, Info4, Info5, Info6, Info7,  Apply, SetBoilerTemp, SetDHWTemp, SetBoilerTemp2, SetNewBoilerTemp });
#endif

#if MQTT_USE
AutoConnectAux Setup_Page(SETUP_URI, "Setup", true, { Ctrl2,  CtrlChB1, CtrlChB2, CtrlChB3, SetMaxMod, CtrlChBMmod, SetTmaxPID, SetTminPID, Info2,
#if RELAY_USE
CtrlChBUseRelay, CtrlChBStartRelaySts,
#endif
CtrlChbUseMQTT, SetMQTT_user, SetMQTT_pwd, SetMQTT_server, SetMQTT_port, SetMQTT_topic, SetMQTT_devname, SetMQTT_interval, CtrlChB_UseRemoteControl, ApplyAdd, ApplyChB});
#else
AutoConnectAux Setup_Page(SETUP_URI, "Setup", true, { Ctrl2, CtrlChB1, CtrlChB2, CtrlChB3, CtrlChBMmod, SetTmaxPID, SetTminPID, Info2,
#if RELAY_USE
CtrlChBUseRelay, CtrlChBStartRelaySts,
#endif
CtrlChB_UseRemoteControl,  ApplyAdd, ApplyChB});
#endif

AutoConnectAux SetTempPage(SET_T_URI, "SetTemp", false, {}, false);
AutoConnectAux SetParPage(SET_PAR_URI,    "SetPar", false, {}, false);
AutoConnectAux SetAddParPage(SET_ADD_URI, "SetAdd", false, {}, false);
#if PID_USE
AutoConnectAux SetPIDPage(SET_PID_URI, "SetPID", false, {}, false);
#endif
#if RELAY_USE
AutoConnectAux SetRelayPage(RELAY_URI, "SetRelay", false, {}, false);
#endif
AutoConnectAux SendBLORPage(BLOR_URI, "SendBlor", false, {}, false);
AutoConnectAux debugPage(DEBUG_URI, "Debug", true, {Info1, Info2, Info3, Info4, Info5, Info6, Info7,  DebugApply});
AutoConnectAux AboutPage(ABOUT_URI, "About", true, { About_0, Info1, Info2, Info3});

#if PID_USE
AutoConnectAux PID_Page(PID_URI, "PID", true, {UsePID, UsePID_NoLimit, SetXtagPID, Info1, SetTempSrcPID, SetTempExtSrcPID,
                      SetKpPID, SetKdPID, SetKiPID,SetIdissPID,
                      Info3, Set_u0_PID, Set_t0_PID, Set_u1_PID,Set_t1_PID, Set_x0_PID, Set_CH_GIST, Info4,
                      UsePIDPWM, PWM_T_PID, Info5, Info6,  ApplyPID });
#endif

#if ST_VERS == 2
AutoConnectAux OTslave_Page(OT2_URI, "OT2", true, {Info1, UseOTslave, OTslaveMode, Info5, Info6, ApplySlave});
AutoConnectAux SetOTslave_Page(SET_OT2_URI, "SetOT2", false, {}, false);
#endif

// Setup addition page (with PID button when PID_USE)
#if PID_USE
ACSubmit(SetupPID,   "PID", PID_URI, AC_Tag_BR);
AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false, { UseID2ChB, ID2MaserID,  UseOTC_ChB, UseCH2_DHW_ChB, UseWinterModeChB, UseID29_DHW_ChB, Immergas_fix_ChB, UseCPU_FREQ_ChB, CPU_FREQ, Info2, ApplyAddpar, SetupPID, Info1, SendBLOR });
#else
AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false, { UseID2ChB, ID2MaserID,  UseOTC_ChB, UseCH2_DHW_ChB, UseWinterModeChB, UseID29_DHW_ChB, Immergas_fix_ChB, UseCPU_FREQ_ChB, CPU_FREQ, Info2, ApplyAddpar, Info1, SendBLOR});
#endif

// Handlers - declarations
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

// Registration API
void RegisterWebPages(AutoConnect& portal) {
  {
    char str[40];
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

  InfoPage.on(onInfo);
  Setup_Page.on(on_Setup);
  SetTempPage.on(onSetTemp);
  SetParPage.on(onSetPar);
  SetAddParPage.on(onSetAddPar);
#if RELAY_USE
  SetRelayPage.on(onSetRelay);
#endif
#if ST_VERS == 2
  OTslave_Page.on(onSetupOT_slave);
  SetOTslave_Page.on(onSetOT_slave);
#endif
#if PID_USE
  PID_Page.on(onSetupPID);
  SetPIDPage.on(onSetPID);
#endif
  SendBLORPage.on(onSendBlor);
  debugPage.on(onDebug);
  AboutPage.on(onAbout);

#if MQTT_USE
#if PID_USE
  portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, SetAddParPage, PID_Page, SetPIDPage, debugPage,  AboutPage});
#else
  portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, SetAddParPage, debugPage,  AboutPage});
#endif
#else
#if PID_USE
  portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, SetAddParPage, PID_Page, SetPIDPage, debugPage,  AboutPage});
#else
  portal.join({InfoPage, Setup_Page,SetupAdd_Page, SetTempPage, SetParPage, SetAddParPage, debugPage,  AboutPage});
#endif
#endif

#if RELAY_USE
  portal.join({SetRelayPage});
#endif
  portal.join({SendBLORPage});
#if ST_VERS == 2
  portal.join({OTslave_Page,SetOTslave_Page});
#endif
}

// ================= Handlers implementation (copied from Web.cpp) =================

String onDebug(AutoConnectAux& aux, PageArgument& args)
{  char str[256];
extern int minRamFree;
extern int WiFiDebugInfo[10];
extern unsigned int OTDebugInfo[12];
extern int WiFists; extern float mRSSi;
#if MQTT_USE
extern void mqtt_loop(void);
#endif

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
      sprintf(str,"reset reason: %d %d", rtc_get_reset_reason(0), rtc_get_reset_reason(1));
  Info7.value = str;
#if MQTT_USE
  sprintf(str,"<br>stsMQTTcfg %d useMQTT %d stsMQTT %d", SmOT.stsMQTTcfg, SmOT.useMQTT, SmOT.stsMQTT );
  Info7.value += str;
#endif
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
          SmOT.need_set_T(1);
        } 
      }
    }
    if(SmOT.enable_HotWater)
    { v = SmOT.CHtempLimit(SetDHWTemp.value.toFloat());    
      if(v != SmOT.TdhwSet)
      { isChange = 1;
        SmOT.TdhwSet = v;
        SmOT.need_set_dhwT(1);
      }
    }
    if(SmOT.enable_CentralHeating2)
    { v = SmOT.CHtempLimit(SetBoilerTemp2.value.toFloat());
      if(v != SmOT.Tset2) 
      {  isChange = 1;
         SmOT.Tset2 = v;
         SmOT.need_set_T_CH2(1);
      }
    }
    if(isChange)
        SmOT.need_write_f = 1;
  aux.redirect(INFO_URI);
  return String();
}

String onSetPar(AutoConnectAux& aux, PageArgument& args)
{ int isChange=0,  redir = 0, iv;
  float fv;
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
#endif
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
  fv = SmOT.CHtempLimit(SetTmaxPID.value.toFloat());    
  if(fv != SmOT.umax)
  { SmOT.umax = fv;
    SmOT.need_set_MaxTSet(2);    
    isChange = 1;
  }
  fv = SmOT.CHtempLimit(SetTminPID.value.toFloat());    
  if( fv > SmOT.umax - 1.)  fv = SmOT.umax -1.;
  if(fv != SmOT.umin)
  { SmOT.umin = fv;
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
    iv = SetMQTT_interval.value.toInt();
    if((unsigned int) iv !=SmOT.MQTT_interval )
    { isChangeMQTT++;
       SmOT.MQTT_interval = iv;
    }
    iv = SetMQTT_port.value.toInt();
    if((unsigned int) iv !=SmOT.MQTT_port )
    { isChangeMQTT++;
       SmOT.MQTT_port = iv;
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
            iv = SetMaxMod.value.toInt();
            if(iv != int(SmOT.MaxRelModLevelSetting+0.5))
            {   isChange++;
                SmOT.MaxRelModLevelSetting = (float)iv;
                SmOT.need_set_MaxRelModLevel(2);
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
    if(SmOT.enable_CentralHeating)
        SmOT.need_set_T(1);
    if(SmOT.HotWater_present)
    { if(SmOT.enable_HotWater)
      {   SmOT.need_set_dhwT(1);
      }
    }
    if(SmOT.CH2_present)
    { if(SmOT.enable_CentralHeating2)
      { SmOT.need_set_T_CH2(1);
      }
    }
  if(redir)
    aux.redirect(SETUP_URI);
  else
    aux.redirect(INFO_URI);
  return String();
}

String onSetAddPar(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0, redir = 0;
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
  if( UseCPU_FREQ_ChB.checked) icheck = true;
  else                         icheck = false;
  if(icheck)
  { if(SmOT.useCPU_freq == -1)
    {   redir = 1;  
        SmOT.useCPU_freq = 0;
    } else {
      int v=0;
      v2 = CPU_FREQ.value.toInt();
      if(v2 == 240) v = 0;
      else if(v2 == 160) v = 1;
      else if(v2 == 80)  v = 2;
      if(v != SmOT.useCPU_freq)
      { SmOT.useCPU_freq = v;
        isChange = 1;
      }
    }
  } else {
    SmOT.useCPU_freq = -1;
    isChange = 1;
  }
  if(isChange)
        SmOT.need_write_f = 1;  
  if(redir)
        aux.redirect(SETUP_ADD_URI);
  else
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
  if(SmOT.useCPU_freq >=0)
      UseCPU_FREQ_ChB.checked = true;
  else
      UseCPU_FREQ_ChB.checked = false;
  if( UseCPU_FREQ_ChB.checked) 
  {   CPU_FREQ.enable = true;
    if(SmOT.useCPU_freq < 1)  strcpy(str,"240");
    else if(SmOT.useCPU_freq == 1)  strcpy(str,"160");
    else  strcpy(str,"80");
    CPU_FREQ.value = str;
    Info2.value ="<small>Частота процессора: 240/160/80</small>";
  } else {
    CPU_FREQ.enable = false;
    Info2.value ="";
  }
  if(SmOT.RemoteRequest_present)
  {   SendBLOR.enable = true;
      Info1.value = "Удаленный сброс ошибки (BLOR), я знаю, что я делаю";
  } else {
     SendBLOR.enable = false;
  }
  return String();
}

String onInfo(AutoConnectAux& aux, PageArgument& args) {
  char str0[256];
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
#endif 
#if PID_USE
    if(SmOT.usePID && SmOT.enable_CentralHeating)
    {   Info1.value += "<br>управление по PID";
        if(SmOT.usePID & 0x02)
              Info1.value += "без ограничений";
      Info1.value += " Tindoor " + String(SmOT.tempindoor) + " ";
      Info1.value += " Toutdoor " + String(SmOT.tempoutdoor);
    }
#endif 
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
         if(SmOT.enable_HotWater)
         {  if(SmOT.Dhw_t_present)
                Info2.value +=  " Горячая вода " + String(SmOT.dhw_t);
            if(SmOT.DHWFlowRate_present)
            {
                Info1.value += " Расход "  + String(SmOT.DHWFlowRate);
            }
         }
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
   Info5.value = "Ts "+ String(SmOT.Tset) + " Tsr "+ String(SmOT.Tset_r) + "<br>";
    if(SmOT.OEMDcode || SmOT.Fault)
    {  sprintf(str0, "%x %x", SmOT.Fault, SmOT.OEMDcode);
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
        {    sprintf(str0, (PGM_P)F(" OEM-specific fault/error cod = %d (hex %x)"), (SmOT.Fault&0xff), (SmOT.Fault&0xff));
            Info6.value += str0;
        }
        Info6.value += "<br>";
      }
      if(SmOT.OEMDcode)
      {     sprintf(str0, (PGM_P)F("OEM-specific diagnostic/service code = %d (hex %x)<br>"), SmOT.OEMDcode, SmOT.OEMDcode);
            Info6.value += str0;
      }
    } else {
      Info6.value = "";
    }
    if(OTDebugInfo[0] > 10)
    { int v =  (OTDebugInfo[3] + OTDebugInfo[4])*100/OTDebugInfo[0]; 
      if(v > 30)
      {   sprintf(str0, "Большое количество ошибок OpenTherm: %d%%<br>", v);
          Info6.value += str0;
      }
    }
    Info7.value = "";
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
        SetNewBoilerTemp.enable = false;
    }
  }
#endif
#if  RELAY_USE
  if(SmOT.Relay_present)
  {
      RelayOnFf.enable = true;
      if(SmOT.Relay_sts )
      { strcpy(str0,"Реле вЫкл");
      } else {
         strcpy(str0,"Реле Вкл");
      }
      RelayOnFf.value = str0;
  } else {
      RelayOnFf.enable = false;
  }
#endif
  return String();
}

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
  Info1.value ="";
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
    Ctrl2.value = "Котёл: "; 
    pstr = GetOTVendorName(SmOT.OTmemberCode);
    if(pstr)
    {   Ctrl2.value += pstr; 
    } else {
        Ctrl2.value +=  "код " + String(SmOT.OTmemberCode);
    }
    if(SmOT.DHW_tank_present) 
        Ctrl2.value +=  "\nбойлер косвенного нагрева";
 } else {
    CtrlChB2.enable  = false;
    CtrlChB3.enable  = false;
    Ctrl2.value = ""; 
 }
    Info2.value = "<small>Tmax <= 80, Tmin >= 30 (конденсационный котел, иначе 40)</small><br><br>";
    sprintf(str,"%.2f",SmOT.umax);
    SetTmaxPID.value = str;
    sprintf(str,"%.2f",SmOT.umin);
    SetTminPID.value = str;
    
    if(SmOT.Use_remoteTCPserver)
      CtrlChB_UseRemoteControl.checked = true;
    else
      CtrlChB_UseRemoteControl.checked = false;
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
#endif
  return String();
}

#if PID_USE
String onSetPID(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0;
   unsigned short int icheck, icheck1=0, icheck2=0;
   short int iv;
   float v;
  if( UsePID.checked) 
  {  icheck = 1;
     if(UsePID_NoLimit.checked) icheck1 = 2;
     if(UsePIDPWM.checked)      icheck2 = 4;
  }  else  {
      icheck = 0;
  }
  if((icheck|icheck1|icheck2) != SmOT.usePID)
  { SmOT.usePID = icheck|icheck1|icheck2;
    isChange = 1;
#if MQTT_USE
    MQTT_pub_usePID();    
#endif    
  }
  if(SmOT.usePID)
  { 
    iv = SetTempSrcPID.value.toInt();
    if(iv > MAX_PID_SRC)
        iv = MAX_PID_SRC;
    else if (iv < -1)
        iv = -1;
    if(iv != SmOT.srcTroom)
    { if((iv == -1) ||(iv == 0 && SmOT.stsT1 == 1) ||(iv == 1 && SmOT.stsT2 == 1) || (iv == 2 && SmOT.Toutside_present) || (iv >2 && SmOT.useMQTT) )
      { SmOT.srcTroom = iv;
        isChange = 1;
      }
    }
    iv = SetTempExtSrcPID.value.toInt();
    if(iv > MAX_PID_SRC)
      iv = MAX_PID_SRC;
    else if (iv < -1)
      iv = -1;
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
    if(v > 0.5) v = 0.5; 
    else if(v < 0.000001) v = 0.000001;
    if(v != SmOT.mypid.Kidiss)
    { SmOT.mypid.Kidiss = v;
      isChange = 1;
    }
    v = Set_CH_GIST.value.toFloat();
    if(v != SmOT.CH_StartGist)
    { SmOT.CH_StartGist = v;
      isChange = 1;
    }
    v = SetXtagPID.value.toFloat();
    if(SmOT.usePID == 1)
    {   if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
        else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    }
    if(v != SmOT.mypid.xTag)
    {
      SmOT.set_new_PID_setpoint(v, 0);
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
    v = Set_x0_PID.value.toFloat();
    if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
    else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    if(v != SmOT.mypid.x0)
    { SmOT.mypid.x0 = v;
      isChange = 1;
    }
    iv = PWM_T_PID.value.toInt();
    if( iv <  5*60) v = 5*60;
    else if(iv > 3600) v = 3600;
    if(iv != SmOT.PID_PWMperiod)
    { SmOT.PID_PWMperiod = iv;
      isChange = 1;
    } 
  }
  if(isChange)
        SmOT.need_write_f = 1;
  aux.redirect(SETUP_URI);
  return String();
}

String onSetupPID(AutoConnectAux& aux, PageArgument& args)
{ char str0[80];
  if(SmOT.usePID & 0x01) 
  { UsePID.checked = true;
  } else {
    UsePID.checked = false;    
  }
  if((SmOT.usePID & 0x03) == 0x03) 
    UsePID_NoLimit.checked = true;
  else 
    UsePID_NoLimit.checked = false;
  if((SmOT.usePID & 0x05) == 0x05) 
    UsePIDPWM.checked = true;
  else 
    UsePIDPWM.checked = false;
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
  sprintf(str0,"%.2f",SmOT.mypid.x0);
  Set_x0_PID.value = str0;
  sprintf(str0,"%d",SmOT.PID_PWMperiod);
  PWM_T_PID.value = str0;
  Info4.value = "";
  Info5.value = "";
  Info6.value = "";
  return String();
}
#endif

#if RELAY_USE
String onSetRelay(AutoConnectAux& aux, PageArgument& args)
{
    if(SmOT.Relay_sts)
      SmOT.RelayOnOff(false);
    else
      SmOT.RelayOnOff(true);
  aux.redirect(INFO_URI);
  return String();
}
#endif

#if ST_VERS == 2
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
        SmOT.need_write_f = 1;
  aux.redirect(INFO_URI);
  return String();
}

String onSetupOT_slave(AutoConnectAux& aux, PageArgument& args)
{   char str0[256];
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
#endif

String onSendBlor(AutoConnectAux& aux, PageArgument& args)
{
    SmOT.need_set_blor();
  aux.redirect(INFO_URI);
  return String();
}

const char SM_OT_HomePage[]= "https://t.me/smartTherm";

String onAbout(AutoConnectAux& aux, PageArgument& args)
{ char str[80];
  Info1.value = IDENTIFY_TEXT;
  sprintf(str, (PGM_P)F("Vers %d.%d.%d.%d  build %s\n"),SmOT.Vers, SmOT.SubVers,SmOT.SubVers1,SmOT.Revision, SmOT.BiosDate);
  Info2.value = str;
  if (WiFi.status() == WL_CONNECTED)
  {   Info3.value = "<a href=";
      Info3.value += SM_OT_HomePage;
      Info3.value += F(">Поддрержка проекта</a>\n");
  } else 
    Info3.value ="";
  return String();
}
