/* Shared.hpp - Common URIs and shared UI controls for AutoConnect pages */

#pragma once

#include <AutoConnect.h>

// URIs (shared across pages)
extern const char INFO_URI[];
extern const char SETUP_URI[];
extern const char RELAY_URI[];
extern const char BLOR_URI[];
extern const char SETUP_ADD_URI[];
extern const char ABOUT_URI[];
extern const char SET_T_URI[];
extern const char SET_PAR_URI[];
extern const char SET_ADD_URI[];
extern const char DEBUG_URI[];
#if PID_USE
extern const char PID_URI[];
extern const char SET_PID_URI[];
#endif
#if ST_VERS == 2
extern const char SET_OT2_URI[];
extern const char OT2_URI[];
#endif

// Common style attribute
extern const char* STYLE_WIDTH;

// Shared controls used by multiple pages
extern AutoConnectText Caption;
extern AutoConnectText Info1;
extern AutoConnectText Info2;
extern AutoConnectText Info3;
extern AutoConnectText Info4;
extern AutoConnectText Info5;
extern AutoConnectText Info6;
extern AutoConnectText Info7;

extern AutoConnectInput SetBoilerTemp;   // CH setpoint
extern AutoConnectInput SetDHWTemp;      // DHW setpoint
extern AutoConnectInput SetBoilerTemp2;  // CH2 setpoint

#if RELAY_USE
extern AutoConnectButton RelayOnFf;
#endif
extern AutoConnectButton Apply;
extern AutoConnectButton SetNewBoilerTemp;
extern AutoConnectButton SendBLOR;
