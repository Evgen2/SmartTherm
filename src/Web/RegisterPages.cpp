
/* RegisterPages.cpp - Aggregate registration for all pages */

#include "Shared.hpp"
#include "SmartDebug.h"

// Forward Register_* from each page module
void Register_Info(AutoConnect& portal);
void Register_About(AutoConnect& portal);
void Register_Debug(AutoConnect& portal);
void Register_SetTemp(AutoConnect& portal);
void Register_SetPar(AutoConnect& portal);
void Register_Setup(AutoConnect& portal);
void Register_SetupAdd(AutoConnect& portal);
void Register_PID(AutoConnect& portal);
void Register_Relay(AutoConnect& portal);
void Register_OT2(AutoConnect& portal);
void Register_SendBLOR(AutoConnect& portal);

// All pages are registered below

void RegisterWebPages(AutoConnect& portal) {
  Serial_db.printf("[RegisterWebPages] Starting page registration...\n");
  Register_Info(portal);
  Register_About(portal);
  Register_Debug(portal);
  Register_Setup(portal);
  Register_SetupAdd(portal);
  #if PID_USE
  Register_PID(portal);
  #endif
  #if RELAY_USE
  Register_Relay(portal);
  #endif
  #if ST_VERS == 2
  Register_OT2(portal);
  #endif
  Register_SendBLOR(portal);
  Register_SetTemp(portal);
  Serial_db.printf("[RegisterWebPages] About to register SetPar page...\n");
  Register_SetPar(portal);
  Serial_db.printf("[RegisterWebPages] All pages registered successfully\n");
}
