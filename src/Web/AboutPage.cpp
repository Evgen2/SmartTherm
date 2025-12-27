/* AboutPage.cpp - /about page */

#include <AutoConnect.h>
#include "Shared.hpp"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

extern SD_Termo SmOT;

static AutoConnectText About_0("About_0", "<b>About:</b>", "", "", AC_Tag_DIV);
static AutoConnectAux AboutPage(ABOUT_URI, "About", true);

const char SM_OT_HomePage[]= "https://t.me/smartTherm";

String onAbout(AutoConnectAux& aux, PageArgument& args)
{ char str[80];
  Info1.value = IDENTIFY_TEXT;
  sprintf(str, (PGM_P)F("Vers %d.%d.%d.%d  build %s\n"),SmOT.Vers, SmOT.SubVers,SmOT.SubVers1,SmOT.Revision, SmOT.BiosDate);
  Info2.value = str;
  if (WiFi.status() == WL_CONNECTED)
  {   Info3.value = "<a href="; Info3.value += SM_OT_HomePage; Info3.value += F(">Поддрержка проекта</a>\n");
  } else Info3.value ="";
  return String();
}

void Register_About(AutoConnect& portal){
  AboutPage.on(onAbout);
  AboutPage.add(About_0);
  AboutPage.add(Info1);
  AboutPage.add(Info2);
  AboutPage.add(Info3);
  portal.join({AboutPage});
}
