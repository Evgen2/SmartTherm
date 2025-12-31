/* SetTempPage.cpp - /set_t hidden action page */

#include <AutoConnect.h>
#include "OpenTherm.h"
#include "Shared.hpp"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

extern SD_Termo SmOT;
extern OpenTherm ot;

static AutoConnectAux SetTempPage(SET_T_URI, "SetTemp", false, {}, false);

String onSetTemp(AutoConnectAux& aux, PageArgument& args)
{  float  v;
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
  aux.redirect(INFO_URI);
  return String();
}

void Register_SetTemp(AutoConnect& portal){ SetTempPage.on(onSetTemp); portal.join({SetTempPage}); }
