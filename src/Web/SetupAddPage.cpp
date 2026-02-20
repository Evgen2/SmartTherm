/* SetupAddPage.cpp - /setupadd and /add pages */

#include "Shared.hpp"

// Controls
static AutoConnectCheckbox UseID2ChB("UseID2ChB","", "Использовать OT ID2", false, AC_Behind , AC_Tag_None);
static AutoConnectInput    ID2MaserID("ID2MaserID","", "IDcode","", "", AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);

static AutoConnectCheckbox UseOTC_ChB("UseOTC_ChB","", "Использовать OTC (ID0:HB3)",         false,   AC_Behind, AC_Tag_BR);
static AutoConnectCheckbox UseCH2_DHW_ChB("UseCH2DHW","", "Использовать CH2 для горячей воды (ID0:HB4)", false, AC_Behind, AC_Tag_BR);
static AutoConnectCheckbox UseWinterModeChB("UseWinterModeChB","", "Режим «лето/зима» (ID0:HB5)", false,   AC_Behind, AC_Tag_BR);
static AutoConnectCheckbox UseID29_DHW_ChB("UseID29DHW","", "Использовать ID29 для температуры бойлера", false, AC_Behind, AC_Tag_BR);
static AutoConnectCheckbox Immergas_fix_ChB("Immergas","", "Immergas fix", false, AC_Behind, AC_Tag_BR);

static AutoConnectCheckbox UseCPU_FREQ_ChB("CPUFREQ","", "CPU FREQ", false, AC_Behind, AC_Tag_None);
static AutoConnectInput    CPU_FREQ("CPU_FREQ","", " ","", "", AC_Tag_None, AC_Input_Text, STYLE_WIDTH);

static AutoConnectButton   ApplyAddpar("ApplyAddpar",   "Задать", SET_ADD_URI, AC_Tag_BR);
// SendBLOR button is shared from Shared.cpp as SendBLOR

#if PID_USE
static AutoConnectButton   SetupPID("SetupPID",   "PID", PID_URI, AC_Tag_BR);
static AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false);
#else
static AutoConnectAux SetupAdd_Page(SETUP_ADD_URI, "SetupAdd", false);
#endif

static AutoConnectAux SetAddParPage(SET_ADD_URI, "SetAdd", false, {}, false);

// Handlers
String onSetAddPar(AutoConnectAux& aux, PageArgument& args)
{  int isChange=0, redir = 0;
   unsigned short int icheck;
   unsigned short int v2;

  if( UseID2ChB.checked) icheck = 1; else icheck = 0;
  if(icheck != SmOT.UseID2) { isChange++; SmOT.UseID2 = icheck; }

  if(UseWinterModeChB.checked)  icheck = 1; else icheck = 0;
  if(icheck != SmOT.UseWinterMode) { isChange++; SmOT.UseWinterMode = icheck; }

  if(UseOTC_ChB.checked)  icheck = 1; else icheck = 0;
  if(icheck != SmOT.Use_OTC) { isChange++; SmOT.Use_OTC = icheck; }

  v2 = ID2MaserID.value.toInt();
  if(v2 != SmOT.ID2masterID ) { isChange++; SmOT.ID2masterID = v2; }

  if( UseCH2_DHW_ChB.checked) icheck = 1; else icheck = 0;
  if(icheck != SmOT.CH2_DHW_flag) { isChange++; SmOT.CH2_DHW_flag = icheck; }

  if(UseID29_DHW_ChB.checked) icheck = 1; else icheck = 0;
  if(icheck != SmOT.Use_ID29_DHW_flag) { isChange++; SmOT.Use_ID29_DHW_flag = icheck; }

  if(Immergas_fix_ChB.checked) icheck = 1; else icheck = 0;
  if(icheck != SmOT.Immergas_fix_flag) { isChange++; SmOT.Immergas_fix_flag = icheck; }

  if( UseCPU_FREQ_ChB.checked) icheck = true; else icheck = false;
  if(icheck)
  { if(SmOT.useCPU_freq == -1) { redir = 1; SmOT.useCPU_freq = 0; }
    else {
      int v=0; v2 = CPU_FREQ.value.toInt();
      if(v2 == 240) v = 0; else if(v2 == 160) v = 1; else if(v2 == 80)  v = 2;
      if(v != SmOT.useCPU_freq) { SmOT.useCPU_freq = v; isChange = 1; }
    }
  } else { SmOT.useCPU_freq = -1; isChange = 1; }

  if(isChange) SmOT.need_write_f = 1;
  if(redir) aux.redirect(SETUP_ADD_URI); else aux.redirect(SETUP_URI);
  return String();
}

String on_SetupAdd(AutoConnectAux& aux, PageArgument& args)
{  char str[40];
  UseID2ChB.checked = SmOT.UseID2;
  UseWinterModeChB.checked = SmOT.UseWinterMode;
  UseOTC_ChB.checked = SmOT.Use_OTC;
  sprintf(str,"%d",SmOT.ID2masterID); ID2MaserID.value = str;
  UseCH2_DHW_ChB.checked = SmOT.CH2_DHW_flag;
  UseID29_DHW_ChB.checked = SmOT.Use_ID29_DHW_flag;
  Immergas_fix_ChB.checked = SmOT.Immergas_fix_flag;
  UseCPU_FREQ_ChB.checked = (SmOT.useCPU_freq >= 0);
  if( UseCPU_FREQ_ChB.checked) {
    CPU_FREQ.enable = true;
    if(SmOT.useCPU_freq < 1)  strcpy(str,"240");
    else if(SmOT.useCPU_freq == 1)  strcpy(str,"160");
    else  strcpy(str,"80");
    CPU_FREQ.value = str;
    Info2.value ="<small>Частота процессора: 240/160/80</small>";
  } else { CPU_FREQ.enable = false; Info2.value =""; }

  if(SmOT.RemoteRequest_present) { SendBLOR.enable = true; Info1.value = "Удаленный сброс ошибки (BLOR), я знаю, что я делаю"; }
  else { SendBLOR.enable = false; }

  return String();
}

void Register_SetupAdd(AutoConnect& portal){
  // Аутентификация применяется автоматически через config.authScope (AC_AUTHSCOPE_AUX)
  SetupAdd_Page.on(on_SetupAdd);
  SetAddParPage.on(onSetAddPar);
#if PID_USE
  SetupAdd_Page.add(UseID2ChB); SetupAdd_Page.add(ID2MaserID);
  SetupAdd_Page.add(UseOTC_ChB); SetupAdd_Page.add(UseCH2_DHW_ChB); SetupAdd_Page.add(UseWinterModeChB);
  SetupAdd_Page.add(UseID29_DHW_ChB); SetupAdd_Page.add(Immergas_fix_ChB);
  SetupAdd_Page.add(UseCPU_FREQ_ChB); SetupAdd_Page.add(CPU_FREQ);
  SetupAdd_Page.add(Info2); SetupAdd_Page.add(ApplyAddpar); SetupAdd_Page.add(SetupPID); SetupAdd_Page.add(Info1); SetupAdd_Page.add(SendBLOR);
#else
  SetupAdd_Page.add(UseID2ChB); SetupAdd_Page.add(ID2MaserID);
  SetupAdd_Page.add(UseOTC_ChB); SetupAdd_Page.add(UseCH2_DHW_ChB); SetupAdd_Page.add(UseWinterModeChB);
  SetupAdd_Page.add(UseID29_DHW_ChB); SetupAdd_Page.add(Immergas_fix_ChB);
  SetupAdd_Page.add(UseCPU_FREQ_ChB); SetupAdd_Page.add(CPU_FREQ);
  SetupAdd_Page.add(Info2); SetupAdd_Page.add(ApplyAddpar); SetupAdd_Page.add(Info1); SetupAdd_Page.add(SendBLOR);
#endif
  portal.join({SetupAdd_Page});
  portal.join({SetAddParPage});
}
