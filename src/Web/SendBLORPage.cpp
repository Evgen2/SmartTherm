/* SendBLORPage.cpp - /blor hidden page */

#include "Shared.hpp"

extern SD_Termo SmOT;

static AutoConnectAux SendBLORPage(BLOR_URI, "SendBlor", false, {}, false);

String onSendBlor(AutoConnectAux& aux, PageArgument& args)
{ SmOT.need_set_blor(); aux.redirect(INFO_URI); return String(); }

void Register_SendBLOR(AutoConnect& portal){ SendBLORPage.on(onSendBlor); portal.join({SendBLORPage}); }
