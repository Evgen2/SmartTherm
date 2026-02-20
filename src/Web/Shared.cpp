/* Shared.cpp - Definitions for common URIs and shared UI controls */

#include "Shared.hpp"

// URIs
const char INFO_URI[]      = "/info";
const char SETUP_URI[]     = "/setup";
const char RELAY_URI[]     = "/relay";
const char BLOR_URI[]      = "/blor";
const char SETUP_ADD_URI[] = "/setupadd";
const char ABOUT_URI[]     = "/about";
const char SET_T_URI[]     = "/set_t";
const char SET_PAR_URI[]   = "/set_par";
const char SET_ADD_URI[]   = "/add";
const char DEBUG_URI[]     = "/debug";
#if PID_USE
const char PID_URI[]       = "/pid";
const char SET_PID_URI[]   = "/set_pid";
#endif
#if ST_VERS == 2
const char SET_OT2_URI[]   = "/setot2";
const char OT2_URI[]       = "/ot2";
#endif

// Styles
const char* STYLE_WIDTH = "width:15%";

// Shared controls
AutoConnectText Caption("Caption", "<b>Статус OT: </b>", "", "", AC_Tag_DIV);
AutoConnectText Info1("Info1", "", "", "", AC_Tag_DIV);
AutoConnectText Info2("Info2", "", "", "", AC_Tag_DIV);
AutoConnectText Info3("Info3", "", "", "", AC_Tag_DIV);
AutoConnectText Info4("Info4", "", "", "", AC_Tag_DIV);
AutoConnectText Info5("Info5", "", "", "", AC_Tag_DIV);
AutoConnectText Info6("Info6", "", "", "", AC_Tag_DIV);
AutoConnectText Info7("Info7", "", "", "", AC_Tag_DIV);

AutoConnectInput SetBoilerTemp("SetBoilerTemp","", "Температура теплоносителя:<br>", "", "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);
AutoConnectInput SetDHWTemp("SetDHWTemp",   "", "Температура горячей воды:<br>", "",  "Введи температуру",AC_Tag_BR, AC_Input_Text, STYLE_WIDTH);
AutoConnectInput SetBoilerTemp2("SetBoilerTemp2","", "Температура CH2:<br>");

#if RELAY_USE
AutoConnectSubmit RelayOnFf("RelayOnFf", "Реле вкл/выкл", RELAY_URI, AC_Tag_None);
#endif
AutoConnectButton Apply("Apply", "Обновить", INFO_URI, AC_Tag_BR);
AutoConnectSubmit SetNewBoilerTemp("SetNewBoilerTemp","Задать", SET_T_URI, AC_Tag_DIV);
AutoConnectSubmit SendBLOR("SendBLOR", "Сброс ошибки", BLOR_URI, AC_Tag_BR);
