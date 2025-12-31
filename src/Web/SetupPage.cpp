/* SetupPage.cpp - /setup page */

#include "Shared.hpp"

// Controls definitions
AutoConnectText Ctrl2("Ctrl2", "", "", "", AC_Tag_DIV);
AutoConnectCheckbox CtrlChB1("CtrlChB1","1", "Отопление", false, AC_Behind , AC_Tag_BR);
AutoConnectCheckbox CtrlChB2("CtrlChB2","2", "Горячая вода", false, AC_Behind , AC_Tag_DIV);
AutoConnectCheckbox CtrlChB3("CtrlChB3","3", "Отопление CH2", false, AC_Behind , AC_Tag_DIV);
AutoConnectInput SetMaxMod("SetMaxMod","", "проценты","",  "0-100%",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
AutoConnectCheckbox CtrlChBMmod("CtrlChBmmod","4", "Макс модуляция", false, AC_Behind , AC_Tag_BR);
#if RELAY_USE
AutoConnectCheckbox CtrlChBUseRelay("ChbUseRelay","5", "Реле", false, AC_Behind , AC_Tag_None);
AutoConnectCheckbox CtrlChBStartRelaySts("ChbStartRelay","6", "Вкл при старте", false, AC_Behind , AC_Tag_BR);
#endif
#if MQTT_USE
AutoConnectCheckbox CtrlChbUseMQTT("ChbUseMQTT","7", "MQTT", false, AC_Behind , AC_Tag_DIV);
AutoConnectInput SetMQTT_server("SetMQTT_server","", "сервер");
AutoConnectInput SetMQTT_port("SetMQTT_port","", "порт", "",  "", AC_Tag_BR, AC_Input_Number, STYLE_WIDTH);
AutoConnectInput SetMQTT_user("SetMQTT_user","", "user");
AutoConnectInput SetMQTT_pwd("SetMQTT_pwd","", "pwd");
AutoConnectInput SetMQTT_topic("SetMQTT_topic","", "топик");
AutoConnectInput SetMQTT_devname("SetMQTT_devname","", "имя устройства");
AutoConnectInput SetMQTT_interval("SetMQTT_interval","", "интервал, сек", "",  "Введи интервал",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH);
#endif
AutoConnectInput SetTmaxPID("SetTmaxPID","", "Tmax:","","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
AutoConnectInput SetTminPID("SetTminPID","", "Tmin:","","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
AutoConnectCheckbox CtrlChB_UseRemoteControl("CtrlChB5","5", "Разрешить удаленное управление", false, AC_Behind , AC_Tag_DIV);
AutoConnectButton ApplyChB("ApplyChB", "Задать", SET_PAR_URI, AC_Tag_DIV);
AutoConnectButton ApplyAdd("ApplyAdd", "Дополнительно", SETUP_ADD_URI, AC_Tag_None);

static AutoConnectAux Setup_Page(SETUP_URI, "Setup", true);

String on_Setup(AutoConnectAux& aux, PageArgument& args)
{  const char *pstr; 
   char str[40]; 
#if RELAY_USE
    CtrlChBUseRelay.enable = true;
    if(SmOT.Relay_present)
    {   CtrlChBUseRelay.checked = true; CtrlChBStartRelaySts.enable = true; CtrlChBStartRelaySts.checked = SmOT.Relay_init_sts;
    } else { CtrlChBUseRelay.checked = false; CtrlChBStartRelaySts.enable = false; }
#endif
  if(SmOT.MaxRelModLevel_present)
  {     CtrlChBMmod.enable = true;
        if(SmOT.Use_MaxRelModLevel)
        {   CtrlChBMmod.checked = true; SetMaxMod.enable = true; sprintf(str, "%d",int(SmOT.MaxRelModLevelSetting+0.5)); SetMaxMod.value = str; }
        else { CtrlChBMmod.checked = false; SetMaxMod.enable = false; }
  } else { SetMaxMod.enable = false; CtrlChBMmod.enable = false; }

  CtrlChB1.checked = SmOT.enable_CentralHeating;
  Info1.value = "";
  if (SmOT.stsOT >= 0)
  {
    if(SmOT.HotWater_present) { CtrlChB2.enable  =  true; CtrlChB2.checked = SmOT.enable_HotWater; } else CtrlChB2.enable  = false;
    CtrlChB3.enable  = SmOT.CH2_present;
    Ctrl2.value = "Котёл: ";
    pstr = GetOTVendorName((int)SmOT.OTmemberCode);
    if(pstr) Ctrl2.value += pstr; else Ctrl2.value +=  "код " + String(SmOT.OTmemberCode);
    if(SmOT.DHW_tank_present) Ctrl2.value +=  "\nбойлер косвенного нагрева";
  } else { CtrlChB2.enable  = false; CtrlChB3.enable  = false; Ctrl2.value = ""; }

  Info2.value = "<small>Tmax <= 80, Tmin >= 30 (конденсационный котел, иначе 40)</small><br><br>";
  sprintf(str,"%.2f",SmOT.umax); SetTmaxPID.value = str;
  sprintf(str,"%.2f",SmOT.umin); SetTminPID.value = str;
  CtrlChB_UseRemoteControl.checked = SmOT.Use_remoteTCPserver;
#if MQTT_USE
  CtrlChbUseMQTT.enable  = true;
  if(SmOT.useMQTT) 
  { if(SmOT.useMQTT == 1) Info1.value = "проверь после Reset"; 
    CtrlChbUseMQTT.checked = true;
    SetMQTT_server.enable  = true; SetMQTT_user.enable  = true; SetMQTT_pwd.enable  = true; SetMQTT_topic.enable  = true; SetMQTT_interval.enable  = true; SetMQTT_devname.enable  = true; SetMQTT_port.enable  = true;
    SetMQTT_user.value = SmOT.MQTT_user; SetMQTT_pwd.value = SmOT.MQTT_pwd; SetMQTT_server.value = SmOT.MQTT_server; SetMQTT_topic.value = SmOT.MQTT_topic;
    sprintf(str, "%d",SmOT.MQTT_interval); SetMQTT_interval.value = str; sprintf(str, "%d",SmOT.MQTT_port); SetMQTT_port.value = str; SetMQTT_devname.value = SmOT.MQTT_devname;
  } else {
    CtrlChbUseMQTT.checked = false;
    SetMQTT_server.enable  = false; SetMQTT_user.enable  = false; SetMQTT_pwd.enable  = false; SetMQTT_topic.enable  = false; SetMQTT_interval.enable  = false; SetMQTT_devname.enable  = false; SetMQTT_port.enable  = false;
  }
#endif
  return String();
}

void Add_SetupElements() {
  Setup_Page.add(Ctrl2);
  Setup_Page.add(CtrlChB1);
  Setup_Page.add(CtrlChB2);
  Setup_Page.add(CtrlChB3);
  Setup_Page.add(SetMaxMod);
  Setup_Page.add(CtrlChBMmod);
  Setup_Page.add(SetTmaxPID);
  Setup_Page.add(SetTminPID);
  Setup_Page.add(Info2);
#if RELAY_USE
  Setup_Page.add(CtrlChBUseRelay);
  Setup_Page.add(CtrlChBStartRelaySts);
#endif
#if MQTT_USE
  Setup_Page.add(CtrlChbUseMQTT);
  Setup_Page.add(SetMQTT_user);
  Setup_Page.add(SetMQTT_pwd);
  Setup_Page.add(SetMQTT_server);
  Setup_Page.add(SetMQTT_port);
  Setup_Page.add(SetMQTT_topic);
  Setup_Page.add(SetMQTT_devname);
  Setup_Page.add(SetMQTT_interval);
#endif
  Setup_Page.add(CtrlChB_UseRemoteControl);
  Setup_Page.add(ApplyAdd);
  Setup_Page.add(ApplyChB);
}

void Register_Setup(AutoConnect& portal){
  Add_SetupElements();
  Setup_Page.on(on_Setup);
  portal.join({Setup_Page});
}
