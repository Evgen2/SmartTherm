
/* RegisterPages.cpp - Aggregate registration for all pages */

#include "Pages.hpp"

// Forward Register_* from each page module
void Register_Info(AutoConnect& portal);
void Register_About(AutoConnect& portal);
void Register_Debug(AutoConnect& portal);
void Register_SetTemp(AutoConnect& portal);
void Register_SetPar(AutoConnect& portal);
void Register_Setup(AutoConnect& portal);

// TODO: Add remaining pages (SetupAdd, AddPar, PID, Relay, OT2)

void RegisterWebPages(AutoConnect& portal) {
  Register_Info(portal);
  Register_About(portal);
  Register_Debug(portal);
  Register_Setup(portal);
  Register_SetTemp(portal);
  Register_SetPar(portal);
  // Remaining page registrations will be added as we split them out
}
