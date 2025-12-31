/* RelayPage.cpp - /relay page */

#include "Shared.hpp"

#if RELAY_USE

static AutoConnectAux SetRelayPage(RELAY_URI, "SetRelay", false, {}, false);

String onSetRelay(AutoConnectAux& aux, PageArgument& args)
{
    if(SmOT.Relay_sts)
      SmOT.RelayOnOff(false);
    else
      SmOT.RelayOnOff(true);
  aux.redirect(INFO_URI);
  return String();
}

void Register_Relay(AutoConnect& portal){ SetRelayPage.on(onSetRelay); portal.join({SetRelayPage}); }
#endif // RELAY_USE
