/* PIDPage.cpp - /pid and /set_pid pages */

#include <AutoConnect.h>
#include "Smart_Config.h"
#include "Shared.hpp"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

extern SD_Termo SmOT;
#if MQTT_USE
extern int MQTT_pub_usePID(void);
#endif

#if PID_USE
// Controls
static AutoConnectCheckbox UsePID("UsePID","", "Использовать PID", false, AC_Behind , AC_Tag_BR);
static AutoConnectCheckbox UsePID_NoLimit("UsePID_NOLIMIT","", "Не ограничивать уставку (5-35°C)", false, AC_Behind , AC_Tag_BR);
static AutoConnectInput SetXtagPID("SetXtagPID","", "Уставка температуры в помещении:", "",  "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput SetTempSrcPID("SetTempSrcPID","", "Источник температуры в помещении:", "",  "",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH);
static AutoConnectInput SetTempExtSrcPID("SetTempExtSrcPID","", "Источник температуры на улице:", "",  "у",AC_Tag_BR, AC_Input_Number, STYLE_WIDTH);
static AutoConnectInput SetKpPID("SetKpPID",  "", "Kp:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput SetKdPID("SetKdPID",  "", "Kd:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput SetKiPID("SetKiPID",  "", "Ki:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput SetIdissPID("SetIdissPID","","Idiss:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);

static AutoConnectInput Set_u0_PID("Set_u0_PID","", "u0:",  "","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput Set_t0_PID("Set_t0_PID","", "t0:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput Set_u1_PID("Set_u1_PID","", "u1:",  "","",AC_Tag_None, AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput Set_t1_PID("Set_t1_PID","", "t1:",  "","",AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput Set_x0_PID("Set_x0_PID","", "Базовая температура помещения:",  "", "", AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
static AutoConnectInput Set_CH_GIST("Set_CH_GIST","", "Гистерезис включения горелки, град:",  "", "", AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);
static AutoConnectCheckbox UsePIDPWM("UsePIDPWM","", "Использовать PWM при выходе U &lt; Tmin", false, AC_Behind , AC_Tag_BR);
static AutoConnectInput PWM_T_PID("PWM_T_PID","", "Время PWM, сек:",  "", "", AC_Tag_BR,   AC_Input_Text, STYLE_WIDTH);

static AutoConnectButton ApplyPID("ApplyPID",   "Задать", SET_PID_URI, AC_Tag_BR);
static AutoConnectAux PID_Page(PID_URI, "PID", true);
static AutoConnectAux SetPIDPage(SET_PID_URI, "SetPID", false, {}, false);

// Handlers
String onSetPID(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0;
   unsigned short int icheck, icheck1=0, icheck2=0;
   short int iv; float v;

  if( UsePID.checked) { icheck = 1; if(UsePID_NoLimit.checked) icheck1 = 2; if(UsePIDPWM.checked) icheck2 = 4; }
  else  icheck = 0;

  if((icheck|icheck1|icheck2) != SmOT.usePID)
  { SmOT.usePID = icheck|icheck1|icheck2; isChange = 1;
#if MQTT_USE
    MQTT_pub_usePID();
#endif
  }

  if(SmOT.usePID)
  { iv = SetTempSrcPID.value.toInt(); if(iv > MAX_PID_SRC) iv = MAX_PID_SRC; else if (iv < -1) iv = -1;
    if(iv != SmOT.srcTroom)
    { if((iv == -1) ||(iv == 0 && SmOT.stsT1 == 1) ||(iv == 1 && SmOT.stsT2 == 1) || (iv == 2 && SmOT.Toutside_present) || (iv >2 && SmOT.useMQTT) )
      { SmOT.srcTroom = iv; isChange = 1; }
    }
    iv = SetTempExtSrcPID.value.toInt(); if(iv > MAX_PID_SRC) iv = MAX_PID_SRC; else if (iv < -1) iv = -1;
    if(iv != SmOT.srcText)
    { if((iv == -1) ||(iv == 0 && SmOT.stsT1 == 1) ||(iv == 1 && SmOT.stsT2 == 1) || (iv == 2 && SmOT.Toutside_present) || (iv >2 && SmOT.useMQTT) )
      { SmOT.srcText = iv; isChange = 1; }
    }
    v = SetKpPID.value.toFloat(); if(v != SmOT.mypid.Kp) { SmOT.mypid.Kp = v; isChange = 1; }
    v = SetKdPID.value.toFloat(); if(v != SmOT.mypid.Kd) { SmOT.mypid.Kd = v; isChange = 1; }
    v = SetKiPID.value.toFloat(); if(v != SmOT.mypid.Ki) { SmOT.mypid.Ki = v; isChange = 1; }
    v = SetIdissPID.value.toFloat(); if(v > 0.5) v = 0.5; else if(v < 0.000001) v = 0.000001; if(v != SmOT.mypid.Kidiss) { SmOT.mypid.Kidiss = v; isChange = 1; }
    v = Set_CH_GIST.value.toFloat(); if(v != SmOT.CH_StartGist) { SmOT.CH_StartGist = v; isChange = 1; }
    v = SetXtagPID.value.toFloat(); if(SmOT.usePID == 1) { if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP; else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP; }
    if(v != SmOT.mypid.xTag) { SmOT.set_new_PID_setpoint(v, 0); SmOT.TroomTarget = v; isChange = 1; }
    v = SmOT.CHtempLimit(Set_u0_PID.value.toFloat()); if(v != SmOT.mypid.u0) { SmOT.mypid.u0 = v; isChange = 1; }
    v = Set_t0_PID.value.toFloat(); if( v > 40.)  v = 40.; else if(v<-80.) v = -80.; if(v != SmOT.mypid.y0) { SmOT.mypid.y0 = v; isChange = 1; }
    v = SmOT.CHtempLimit(Set_u1_PID.value.toFloat()); if(v != SmOT.mypid.u1) { SmOT.mypid.u1 = v; isChange = 1; }
    v = Set_t1_PID.value.toFloat(); if( v > 40.)  v = 40.; else if(v<-80.) v = -80.; if(v != SmOT.mypid.y1) { SmOT.mypid.y1 = v; isChange = 1; }
    v = Set_x0_PID.value.toFloat(); if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP; else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP; if(v != SmOT.mypid.x0) { SmOT.mypid.x0 = v; isChange = 1; }
    iv = PWM_T_PID.value.toInt(); if( iv <  5*60) iv = 5*60; else if(iv > 3600) iv = 3600; if(iv != SmOT.PID_PWMperiod) { SmOT.PID_PWMperiod = iv; isChange = 1; }
  }
  if(isChange) SmOT.need_write_f = 1;
  aux.redirect(SETUP_URI);
  return String();
}

String onSetupPID(AutoConnectAux& aux, PageArgument& args)
{ char str0[80];
  UsePID.checked = ((SmOT.usePID & 0x01) != 0);
  UsePID_NoLimit.checked = ((SmOT.usePID & 0x03) == 0x03);
  UsePIDPWM.checked = ((SmOT.usePID & 0x05) == 0x05);
  Info1.value = "<small>Источник: -1=n/a, 0/1=T1/T2";
  if(SmOT.Toutside_present) Info1.value += ", 2=Text";
#if  MQTT_USE
  if(SmOT.useMQTT)
  { Info1.value += ", MQTT/HA:";
    sprintf(str0,"3=number.%s_t_indoor,",SmOT.MQTT_devname); Info1.value += str0;
    sprintf(str0,"4=number.%s_t_outdoor",SmOT.MQTT_devname); Info1.value += str0;
  }
#endif
  Info1.value += "</small>";
  sprintf(str0,"%d",SmOT.srcTroom); SetTempSrcPID.value = str0;
  sprintf(str0,"%d",SmOT.srcText);  SetTempExtSrcPID.value = str0;
  sprintf(str0,"%.4f",SmOT.mypid.Kp); SetKpPID.value = str0;
  sprintf(str0,"%.4f",SmOT.mypid.Kd); SetKdPID.value = str0;
  sprintf(str0,"%.4f",SmOT.mypid.Ki); SetKiPID.value = str0;
  sprintf(str0,"%.4f",SmOT.mypid.Kidiss); SetIdissPID.value = str0;
  sprintf(str0,"%.4f",SmOT.CH_StartGist); Set_CH_GIST.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.xTag); SetXtagPID.value = str0;
  Info3.value = "ПЗА: темп.отопления | наружная";
  sprintf(str0,"%.2f",SmOT.mypid.u0); Set_u0_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.y0); Set_t0_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.u1); Set_u1_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.y1); Set_t1_PID.value = str0;
  sprintf(str0,"%.2f",SmOT.mypid.x0); Set_x0_PID.value = str0;
  sprintf(str0,"%d",SmOT.PID_PWMperiod); PWM_T_PID.value = str0;
  Info4.value = ""; Info5.value = ""; Info6.value = "";
  return String();
}

void Register_PID(AutoConnect& portal){
  PID_Page.on(onSetupPID);
  SetPIDPage.on(onSetPID);
  PID_Page.add(UsePID); PID_Page.add(UsePID_NoLimit); PID_Page.add(SetXtagPID); PID_Page.add(Info1);
  PID_Page.add(SetTempSrcPID); PID_Page.add(SetTempExtSrcPID);
  PID_Page.add(SetKpPID); PID_Page.add(SetKdPID); PID_Page.add(SetKiPID); PID_Page.add(SetIdissPID);
  PID_Page.add(Info3); PID_Page.add(Set_u0_PID); PID_Page.add(Set_t0_PID); PID_Page.add(Set_u1_PID); PID_Page.add(Set_t1_PID);
  PID_Page.add(Set_x0_PID); PID_Page.add(Set_CH_GIST); PID_Page.add(Info4);
  PID_Page.add(UsePIDPWM); PID_Page.add(PWM_T_PID); PID_Page.add(Info5); PID_Page.add(Info6); PID_Page.add(ApplyPID);
  portal.join({PID_Page}); portal.join({SetPIDPage});
}
#endif // PID_USE

