/* Shared.hpp - Common URIs and shared UI controls for AutoConnect pages */

#pragma once

#include <AutoConnect.h>
#include <time.h>
#include <stdint.h>
#include "Smart_Config.h"
#include "OpenTherm.h"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

// Forward declarations for common globals
class SD_Termo;
extern SD_Termo SmOT;            // device state
extern OpenTherm ot;             // OpenTherm interface
extern String utc_time_jc;       // formatted UTC time snippet for UI
extern int WiFiDebugInfo[10];
extern unsigned int OTDebugInfo[12];
extern int WiFists;              // last WiFi status
extern float mRSSi;              // averaged RSSI

// Common utilities used by pages
int OutUTCtime(time_t now);
unsigned int _toWiFiQuality(int32_t rssi);

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
extern AutoConnectSubmit RelayOnFf;
#endif
extern AutoConnectButton Apply;
extern AutoConnectSubmit SetNewBoilerTemp;
extern AutoConnectSubmit SendBLOR;

// Register all pages (create controls, attach handlers, join to portal)
void RegisterWebPages(AutoConnect& portal);
