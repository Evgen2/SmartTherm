/* OT2Pages.cpp - /ot2 and /setot2 pages (ST_VERS == 2) */

#include "Shared.hpp"

#if ST_VERS == 2

static AutoConnectCheckbox UseOTslave("UseOTslave","", "Использовать OT slave интерфейс", false, AC_Behind ,  AC_Tag_DIV);
static AutoConnectRadio OTslaveMode("radio", { "SmartTherm", "Панель" }, "Котлом управляет:", AC_Vertical, 1,  AC_Tag_DIV);
static AutoConnectButton ApplySlave("ApplySlave",   "Задать", SET_OT2_URI, AC_Tag_BR);

static AutoConnectAux OTslave_Page(OT2_URI, "OT2", true);
static AutoConnectAux SetOTslave_Page(SET_OT2_URI, "SetOT2", false, {}, false);

String onSetOT_slave(AutoConnectAux& aux, PageArgument& args)
{ int isChange=0; bool check;
  if(UseOTslave.checked) check = true; else check = false;
  if(SmOT.OT_slave_present != check) { isChange++; SmOT.OT_slave_present = check; }
  if(SmOT.OT_slave_present)
  {   if(OTslaveMode.checked+1 != SmOT.OT_slave_mode) { isChange++; SmOT.OT_slave_mode = OTslaveMode.checked - 1; }
  }
  if(isChange) SmOT.need_write_f = 1;
  aux.redirect(INFO_URI);
  return String();
}

String onSetupOT_slave(AutoConnectAux& aux, PageArgument& args)
{   char str0[256];
   Info1.value = "Интерфейс slave OpenTherm:<br>";
   switch(SmOT.ot_slave_stsOT)
   {  case -2: Info1.value +=  String(SmOT.ot_slave_stsOT) + ": <b>Ошибка:</b> не инициализирован без OT"; break;
      case -1: Info1.value +=  String(SmOT.ot_slave_stsOT) + ": <b>Ошибка:</b> не инициализирован"; break;
      case 0:  Info1.value +=  String(SmOT.ot_slave_stsOT) + ": работает"; break;
      case 2:
      {  time_t now = time(nullptr); double dt = difftime(now,SmOT.ot_slave_t_lastwork);
        if(dt < 3600.) { sprintf(str0, (PGM_P)F("Потеря связи %.f сек назад"), dt);
        } else { sprintf(str0, (PGM_P)F("Потеря связи связи  %.1f час(ов) назад"), dt); }
        Info1.value +=  str0; }
        break; }
    if(SmOT.OT_slave_present) { UseOTslave.checked = true; OTslaveMode.enable = true; OTslaveMode.checked = (SmOT.OT_slave_mode == 0) ? 1 : 2; }
    else { UseOTslave.checked = false; OTslaveMode.enable = false; }
   Info5.value =""; Info6.value ="";      
  return String();
}

void Register_OT2(AutoConnect& portal){
  // Аутентификация применяется автоматически через config.authScope (AC_AUTHSCOPE_AUX)
  OTslave_Page.on(onSetupOT_slave);
  SetOTslave_Page.on(onSetOT_slave);
  OTslave_Page.add(Info1); OTslave_Page.add(UseOTslave); OTslave_Page.add(OTslaveMode); OTslave_Page.add(Info5); OTslave_Page.add(Info6); OTslave_Page.add(ApplySlave);
  portal.join({OTslave_Page}); portal.join({SetOTslave_Page});
}
#endif // ST_VERS == 2
