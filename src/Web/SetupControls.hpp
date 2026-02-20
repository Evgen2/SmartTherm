/* SetupControls.hpp - extern declarations for Setup page controls */

#pragma once
#include <AutoConnect.h>

extern AutoConnectText Ctrl2;
extern AutoConnectCheckbox CtrlChB1;
extern AutoConnectCheckbox CtrlChB2;
extern AutoConnectCheckbox CtrlChB3;
extern AutoConnectInput SetMaxMod;
extern AutoConnectCheckbox CtrlChBMmod;
#if RELAY_USE
extern AutoConnectCheckbox CtrlChBUseRelay;
extern AutoConnectCheckbox CtrlChBStartRelaySts;
#endif
#if MQTT_USE
extern AutoConnectCheckbox CtrlChbUseMQTT;
extern AutoConnectInput SetMQTT_server;
extern AutoConnectInput SetMQTT_port;
extern AutoConnectInput SetMQTT_user;
extern AutoConnectInput SetMQTT_pwd;
extern AutoConnectInput SetMQTT_topic;
extern AutoConnectInput SetMQTT_devname;
extern AutoConnectInput SetMQTT_interval;
#endif
extern AutoConnectInput SetTmaxPID;
extern AutoConnectInput SetTminPID;
extern AutoConnectCheckbox CtrlChB_UseRemoteControl;
extern AutoConnectText InfoAuth;
extern AutoConnectInput SetWebAuthUser;
extern AutoConnectInput SetWebAuthPwd;
extern AutoConnectSubmit ApplyChB;
extern AutoConnectSubmit ApplyAdd;
