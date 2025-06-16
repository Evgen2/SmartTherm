/* SD_MQTT.cpp */

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
using WiFiWebServer = ESP8266WebServer;
#define FORMAT_ON_FAIL
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
using WiFiWebServer = WebServer;
#define FORMAT_ON_FAIL  true
#endif

#include <time.h>
#include "OpenTherm.h"
#include "Smart_Config.h"
#include "SmartDevice.hpp"
#include "SD_OpenTherm.hpp"

#if MQTT_USE 


#include <ArduinoHA.h>
#include <PubSubClient.h>

void mqtt_start(void);
void mqtt_loop(void);
void mqtt_setup(void);
int MQTT_pub_data(void);
void MQTT_pub_Eff_Mod_h(void);
#if RELAY_USE
void MQTT_pub_relay(void);
#endif

extern WiFiClient tcp_client;

WiFiClient espClient;
PubSubClient client(espClient);
extern  SD_Termo SmOT;
extern OpenTherm ot;

/*******************************************************************************/
//HADevice *pHAdevice;
//HAMqtt *pMqtt;

HADevice device;
#if RELAY_USE
 #if PID_USE
  HAMqtt mqtt(espClient, device,28);
 #else
  HAMqtt mqtt(espClient, device,13);
 #endif
#else
 #if PID_USE
  HAMqtt mqtt(espClient, device,27);
 #else
  HAMqtt mqtt(espClient, device,12);
 #endif
#endif
const char * temperature_str = "temperature";

HABinarySensor sensorOT(NULL);
HABinarySensor sensorFlame(NULL);
HABinarySensor sensor_CH(NULL);
HABinarySensor sensor_HW(NULL);
HABinarySensor sensor_CMD_on(NULL);
HABinarySensor sensor_CMD_CH_on(NULL);
#if RELAY_USE
HASwitch relayHA(NULL);
#endif
HASensor sensorModulation(NULL);
HASensor sensorBoilerT(NULL);
HASensor sensorBoilerRetT(NULL);
HASensor sensorPressure(NULL);
HASensor sensorDHWFlowRate(NULL);
HASensor sensorT1(NULL);
HASensor sensorT2(NULL);
HASensor sensorText(NULL);
HASensor sensorFreeRam(NULL);
//HASensor sensor_TestNum(NULL);

HASensor sensorState(NULL); //errors
#if PID_USE
//HAText  textTargetTemp(NULL);
//HAText  textPIDinfo(NULL);
//HANumber numPID_v(NULL,HANumber::PrecisionP3);
HANumber numT_outdoor(NULL,HANumber::PrecisionP3);
HANumber numT_indoor(NULL,HANumber::PrecisionP3);
HASensor sensorPID_P(NULL,HANumber::PrecisionP3);
HASensor sensorPID_D(NULL,HANumber::PrecisionP3);
HASensor sensorPID_I(NULL,HANumber::PrecisionP3);
HASensor sensorPID_U(NULL,HANumber::PrecisionP3);
HASensor sensorPID_U0(NULL,HANumber::PrecisionP3);
HASensor sensor_Eff_Mod(NULL,HANumber::PrecisionP3);

#endif

// By default HAHVAC supports only reporting of the temperature.
// You can enable feature you need using the second argument of the constructor.
// Please check the documentation of the HAHVAC class.
HAHVAC hvac(
  NULL,
  HAHVAC::TargetTemperatureFeature | HAHVAC::PowerFeature | HAHVAC::ModesFeature |HAHVAC::ActionFeature,
  HANumber::PrecisionP2
);

HAHVAC hvacDHW(
  NULL,
  HAHVAC::TargetTemperatureFeature |HAHVAC::ModesFeature |HAHVAC::ActionFeature,
  HANumber::PrecisionP2
);

#if PID_USE
HAHVAC hvacPID(
  NULL,
  HAHVAC::TargetTemperatureFeature | HAHVAC::ModesFeature | HAHVAC::ActionFeature,
  HANumber::PrecisionP3
);
#endif
unsigned long lastReadAt = millis();
unsigned long lastAvailabilityToggleAt = millis();
bool lastInputState = false;
void OnMQTTconnected(void);
void OnMQTTdisconnected(void);
void MQTTsenddata(void);

void onTargetTemperatureCommand(HANumeric temperature, HAHVAC* sender) {
    float temperatureFloat = temperature.toFloat();
    if(sender == &hvacDHW)
    {
      SmOT.TdhwSet = temperatureFloat;
      SmOT.need_set_dhwT(2);
#if SERIAL_DEBUG      
    Serial.print("DHW Target temperature: ");
    Serial.println(temperatureFloat);
#endif    
      sender->setTargetTemperature(temperature); // report target temperature back to the HA panel

#if PID_USE
    } else if (sender == &hvacPID) {
//    Serial.print("PID Target temperature: ");
//    Serial.println(temperatureFloat);

    SmOT.set_new_PID_setpoint(temperatureFloat, 1); //change mypid.xTag 
    SmOT.TroomTarget = temperatureFloat;
   Serial_db.printf("**** MQTT Set_NewTag: xTag = %f  = %f\n", SmOT.TroomTarget, SmOT.mypid.xTag);

//todo    
#endif
    } else {
      SmOT.Tset = temperatureFloat;
      SmOT.need_set_T(2);
      sender->setTargetTemperature(temperature); // report target temperature back to the HA panel
#if SERIAL_DEBUG      
    Serial.print("CH Target temperature: ");
    Serial.println(temperatureFloat);
#endif    
    }
}

void onPowerCommand(bool state, HAHVAC* sender) {
  if (state) {
    Serial.println("Power on");
  } else {
    Serial.println("Power off");
  }
}

void onModeCommand(HAHVAC::Mode mode, HAHVAC* sender) {
//PID_USE todo    
    Serial.print("Mode: ");
    if (mode == HAHVAC::OffMode) {
        Serial.println(F("off"));
        SmOT.enable_CentralHeating = false;
    } else if (mode == HAHVAC::HeatMode) {
        Serial.println("heat");
        SmOT.enable_CentralHeating = true;

#if 0        
    } else if (mode == HAHVAC::AutoMode) {
        Serial.println("auto");
    } else if (mode == HAHVAC::CoolMode) {
        Serial.println("cool");
    } else if (mode == HAHVAC::DryMode) {
        Serial.println("dry");
    } else if (mode == HAHVAC::FanOnlyMode) {
        Serial.println("fan only");
#endif        
    }

    sender->setMode(mode); // report mode back to the HA panel
}

void onModeCommandPID(HAHVAC::Mode mode, HAHVAC* sender) {
    Serial.print("Mode: ");
    if (mode == HAHVAC::OffMode) {
        Serial.println(F("PID off"));
        SmOT.usePID = 0;
    } else if (mode == HAHVAC::AutoMode) {
        Serial.println("PID on");
        SmOT.usePID = 1;
    }

    sender->setMode(mode); // report mode back to the HA panel
}

void onModeCommandDHW(HAHVAC::Mode mode, HAHVAC* sender) {
    Serial.print("Mode: ");
    if (mode == HAHVAC::OffMode) {
        Serial.println(F("DHW off"));
        SmOT.enable_HotWater = false;
    } else if (mode == HAHVAC::HeatMode) {
        Serial.println("DHW heat");
        SmOT.enable_HotWater = true;
    }

    sender->setMode(mode); // report mode back to the HA panel
    Serial_db.printf("SmOT.enable_HotWater %d\n", SmOT.enable_HotWater);
}

#if PID_USE

void onNumberCommand(HANumeric number, HANumber* sender)
{   float t = number.toFloat();
//    if (sender == &numPID_v) {
//      Serial_db.printf("NumberCommand numPID_v: %f\n",t);
//
//    } else 
    
    if (sender == &numT_outdoor) {
#if SERIAL_DEBUG      
      Serial_db.printf("NumberCommand numT_outdoor: %f (%d)\n", t, millis()/1000);
#endif      
      SmOT.OnChangeT(t,4);
        
    } else if (sender == &numT_indoor) {
#if SERIAL_DEBUG      
//      Serial_db.printf("NumberCommand numT_indoor: %f (%d)\n", t, millis()/1000);
#endif      
      SmOT.OnChangeT(t,3);
    }
/*
    if (!number.isSet()) {
        // the reset command was send by Home Assistant
    } else {
        // you can do whatever you want with the number as follows:
        int8_t numberInt8 = number.toInt8();
        int16_t numberInt16 = number.toInt16();
        int32_t numberInt32 = number.toInt32();
        uint8_t numberUInt8 = number.toUInt8();
        uint16_t numberUInt16 = number.toUInt16();
        uint32_t numberUInt32 = number.toUInt32();
        float numberFloat = number.toFloat();
    }
*/
    sender->setState(number); // report the selected option back to the HA panel
}
#endif

#if RELAY_USE
void onRelayCommand(bool state, HASwitch* sender)
{ if(sender == &relayHA)
  {
    if(state)
      SmOT.RelayOnOff(1);
    else
      SmOT.RelayOnOff(0);
  }
}
#endif


int statemqtt = -1;
int state_mqtt = -10000;
int attempt_mqtt = 0;

/************************************************************/
void mqtt_setup(void)
{  bool rc;
  char str[80];
extern unsigned int OTcount;

  if (WiFi.status() != WL_CONNECTED)  
        return;

  if(SmOT.stsOT == 0)
  { if(SmOT.CapabilitiesDetected == 0)
        return;
    else
      SmOT.DetectCapabilities();
  } else {
    if(OTcount < 30)
        return;
  }


   Serial_db.printf("mqtt_setup: SmOT.useMQTT = %d\n", SmOT.useMQTT);
   if(SmOT.useMQTT != 0x03) 
      return;

  
  if( mqtt.getDevicesTypesNb_toreg() > mqtt.getDevicesTypesNb())
  {
      Serial_db.printf("Error! Nb = %d, need be %d\n", mqtt.getDevicesTypesNb(),  mqtt.getDevicesTypesNb_toreg() );
//look at 45 HAMqtt mqtt(espClient, device,27);      
    return;
  }

   device.setUniqueIdStr(SmOT.MQTT_topic);

   device.setName(SmOT.MQTT_topic,SmOT.MQTT_devname); //должно быть static!!
  { static char str[40];
    sprintf(str,"%d.%d.%d.%d %s" , SmOT.Vers,SmOT.SubVers,SmOT.SubVers1,SmOT.Revision, SmOT.BiosDate);
    device.setSoftwareVersion(str); //должно быть static!!
    device.setConfigurationUrl(SmOT.LocalUrl);// --//--
  }
    device.enableSharedAvailability();
    device.enableLastWill();

    lastReadAt = millis();
    lastAvailabilityToggleAt = millis();
    sensorOT.setAvailability(false);
    sensorOT.setCurrentState(false); 

    sensorOT.setNameUniqueIdStr(SmOT.MQTT_topic,"OpenTherm", "OT" );
    sensorOT.setDeviceClass("connectivity"); 

    sensorState.setAvailability(true);
    sensorState.setNameUniqueIdStr(SmOT.MQTT_topic,"Ошибки", "Err" );
    sensorState.setIcon("mdi:alert-box");
    sensorState.setValue("");

    sensorFlame.setNameUniqueIdStr(SmOT.MQTT_topic,"Горелка", "Flame");
    sensorFlame.setCurrentState(false); 
    sensorFlame.setAvailability(false);
    sensorFlame.setIcon("mdi:fire");
//    sensorFlame.setDeviceClass("None");
    
    sensorModulation.setNameUniqueIdStr(SmOT.MQTT_topic,"Модуляция", "Modulation");
    sensorModulation.setAvailability(false);
    sensorModulation.setIcon("mdi:fire");
    sensorModulation.setDeviceClass("power_factor"); 
    sensorModulation.setUnitOfMeasurement("%");
/**********/
    sensor_CH.setNameUniqueIdStr(SmOT.MQTT_topic,"Отопление", "CH");
    sensor_CH.setCurrentState(false); 
    sensor_CH.setAvailability(false);
    sensor_CH.setIcon("mdi:heating-coil");
    
    if(SmOT.HotWater_present || SmOT.stsOT == -1)
    { sensor_HW.setNameUniqueIdStr(SmOT.MQTT_topic,"Горячая вода", "HW");
      sensor_HW.setCurrentState(false); 
      sensor_HW.setAvailability(false);
      sensor_HW.setIcon("mdi:water-thermometer");     
    }
    
    sensor_CMD_on.setNameUniqueIdStr(SmOT.MQTT_topic,"Cmd", "cmd");
    sensor_CMD_on.setCurrentState(false); 
    sensor_CMD_on.setAvailability(false);
    sensor_CMD_on.setIcon("mdi:heating-coil");

    sensor_CMD_CH_on.setNameUniqueIdStr(SmOT.MQTT_topic,"CmdCH", "cmdCH");
    sensor_CMD_CH_on.setCurrentState(false); 
    sensor_CMD_CH_on.setAvailability(false);
    sensor_CMD_CH_on.setIcon("mdi:heating-coil");

/**********/

    sensorBoilerT.setNameUniqueIdStr(SmOT.MQTT_topic,"Температура теплоносителя", "BoilerT");
    sensorBoilerT.setAvailability(false);
    sensorBoilerT.setDeviceClass(temperature_str);
    sensorBoilerT.setUnitOfMeasurement("°C");
#if RELAY_USE
  if(SmOT.Relay_present)
  {
    relayHA.setNameUniqueIdStr(SmOT.MQTT_topic,"Реле", "Relay");
    relayHA.setAvailability(true);
    relayHA.setDeviceClass("switch");
    relayHA.setState(SmOT.Relay_sts);
    relayHA.onCommand(onRelayCommand);

  }
#endif//RELAY_USE    

    if(SmOT.RetT_present)
    { sensorBoilerRetT.setNameUniqueIdStr(SmOT.MQTT_topic,"Температура обратки", "RetT");
      sensorBoilerRetT.setAvailability(false);
      sensorBoilerRetT.setDeviceClass(temperature_str); 
      sensorBoilerRetT.setUnitOfMeasurement("°C");
    }

    if(SmOT.Pressure_present)
    { sensorPressure.setNameUniqueIdStr(SmOT.MQTT_topic,"Давление", "Pressure");
      sensorPressure.setAvailability(false);
      sensorPressure.setDeviceClass("pressure"); 
    }

    if(SmOT.DHWFlowRate_present)
    { sensorDHWFlowRate.setNameUniqueIdStr(SmOT.MQTT_topic,"Расход", "DHWFlowRate");
      sensorDHWFlowRate.setAvailability(false);
      sensorDHWFlowRate.setDeviceClass("volume_flow_rate"); 
    }

    sensorFreeRam.setAvailability(true);
    sensorFreeRam.setNameUniqueIdStr(SmOT.MQTT_topic,"Free RAM", "FreeRAM");
    sensorFreeRam.setDeviceClass("data_size"); 
    sensorFreeRam.setUnitOfMeasurement("B");
/*
    sensor_TestNum.setAvailability(true);
    sensor_TestNum.setNameUniqueIdStr(SmOT.MQTT_topic,"Test N", "TestN");
    sensor_TestNum.setDeviceClass("data_size"); 
    sprintf(str,"0");
     sensor_TestNum.setValue(str);  
*/
    // assign callbacks (optional)
    hvac.onTargetTemperatureCommand(onTargetTemperatureCommand);
    hvac.onPowerCommand(onPowerCommand);
    hvac.onModeCommand(onModeCommand);

    // configure HVAC (optional)
    hvac.setNameUniqueIdStr(SmOT.MQTT_topic,"Котёл", "Boiler");

    hvac.setMinTemp(SmOT.umin);
    hvac.setMaxTemp(SmOT.umax);
    hvac.setTempStep(1.);
    hvac.setModes(HAHVAC::OffMode|HAHVAC::HeatMode);
    #if  PID_USE
    if(SmOT.enable_CentralHeating_real)
    #else
    if(SmOT.enable_CentralHeating)
    #endif
          hvac.setMode(HAHVAC::HeatMode);
    else
          hvac.setMode(HAHVAC::OffMode);

    hvac.setAvailability(false);


    if(SmOT.HotWater_present) 
    {
      hvacDHW.onTargetTemperatureCommand(onTargetTemperatureCommand);
      hvacDHW.onModeCommand(onModeCommandDHW);
      if(SmOT.Use_ID29_DHW_flag && ot.OTid_used(OpenThermMessageID::Tstorage))
        hvacDHW.setNameUniqueIdStr(SmOT.MQTT_topic,"Бойлер", "DHW");
      else
        hvacDHW.setNameUniqueIdStr(SmOT.MQTT_topic,"Горячая вода", "DHW");
      hvacDHW.setMinTemp(30);
      hvacDHW.setMaxTemp(80);
      hvacDHW.setTempStep(1.);

      hvacDHW.setModes(HAHVAC::OffMode|HAHVAC::HeatMode);

      if(SmOT.enable_HotWater)
            hvacDHW.setMode(HAHVAC::HeatMode);
      else
            hvacDHW.setMode(HAHVAC::OffMode);
      hvacDHW.setAvailability(false);
    }
/*********************************/
#if PID_USE
     hvacPID.onTargetTemperatureCommand(onTargetTemperatureCommand);
     hvacPID.setNameUniqueIdStr(SmOT.MQTT_topic,"ПИД", "PID");
    if(SmOT.usePID == 0)
    {   hvacPID.setMinTemp(MIN_ROOM_TEMP);
        hvacPID.setMaxTemp(MAX_ROOM_TEMP);
        hvacPID.setAvailability(false);
    } else  if(SmOT.usePID == 1)  {
        hvacPID.setMinTemp(MIN_ROOM_TEMP);
        hvacPID.setMaxTemp(MAX_ROOM_TEMP);
    } else {
      hvacPID.setMinTemp(5);
      hvacPID.setMaxTemp(80);
    }
    hvacPID.setTempStep(0.1);
    hvacPID.setModes(HAHVAC::OffMode|HAHVAC::AutoMode);
    if(SmOT.usePID)
            hvacPID.setMode(HAHVAC::AutoMode);
    else
            hvacPID.setMode(HAHVAC::OffMode);

    hvacPID.onModeCommand(onModeCommandPID);

//todo    
#endif
/*********************************/
    if(SmOT.stsT1 >= 0 )
    { sensorT1.setAvailability(true);
      sensorT1.setNameUniqueIdStr(SmOT.MQTT_topic,"T1", "T1");
      sensorT1.setDeviceClass(temperature_str); 
      sensorT1.setUnitOfMeasurement("°C");
      sprintf(str,"%.3f", SmOT.t1);
      sensorT1.setValue(str);  
//   Serial_db.printf("***000 MQTT T1=%s\n",  str); 

    }  else {
      sensorT1.setAvailability(false);
    }
  
    if(SmOT.stsT2 >= 0 )
    { sensorT2.setAvailability(true);
      sensorT2.setNameUniqueIdStr(SmOT.MQTT_topic,"T2", "T2");
      sensorT2.setDeviceClass(temperature_str); 
      sensorT2.setUnitOfMeasurement("°C");
      sprintf(str,"%.3f", SmOT.t2);
      sensorT2.setValue(str);  
    }  else {
      sensorT2.setAvailability(false);
    }

    if(SmOT.Toutside_present)
    { sensorText.setAvailability(false);
      sensorText.setNameUniqueIdStr(SmOT.MQTT_topic,"Tвн", "Toutside");
      sensorText.setDeviceClass(temperature_str); 
      sensorText.setUnitOfMeasurement("°C");
    }

#if PID_USE
/*
    textTargetTemp.setAvailability(true);
    textTargetTemp.setNameUniqueIdStr(SmOT.MQTT_topic,"Target Temp", "TargetTemp");
    textTargetTemp.setValue("22");

    textPIDinfo.setAvailability(true);
    textPIDinfo.setNameUniqueIdStr(SmOT.MQTT_topic,"PID info", "PIDinfo");
    textPIDinfo.setValue("BlaBlaBla");

    numPID_v.setAvailability(true);
    numPID_v.setNameUniqueIdStr(SmOT.MQTT_topic,"PID  numv", "PIDnumv");
    numPID_v.setMode(HANumber::ModeBox);
    numPID_v.setState(23.f, true);
    numPID_v.setCurrentState(22.f);
    numPID_v.setStep(0.1);
    numPID_v.setMin(-50.);
    numPID_v.setMax( 100.);
    numPID_v.onCommand(onNumberCommand);
*/
    numT_outdoor.setAvailability(true);
    numT_outdoor.setNameUniqueIdStr(SmOT.MQTT_topic,"T outdoor", "Toutdoor");
    numT_outdoor.setMode(HANumber::ModeBox);
    numT_outdoor.setState(10.f, false);
//    numT_outdoor.setCurrentState(10.f);
    numT_outdoor.setStep(0.1);
    numT_outdoor.setMin(-50.);
    numT_outdoor.setMax( 50.);
    numT_outdoor.onCommand(onNumberCommand);

    numT_indoor.setAvailability(true);
    numT_indoor.setNameUniqueIdStr(SmOT.MQTT_topic,"T indoor", "Tindoor");
    numT_indoor.setMode(HANumber::ModeBox);
    numT_indoor.setState(10.f, true);
    numT_indoor.setStep(0.1);
    numT_indoor.setMin(-50.);
    numT_indoor.setMax( 50.);
    numT_indoor.onCommand(onNumberCommand);


    sensorPID_P.setAvailability(true);
    sensorPID_P.setNameUniqueIdStr(SmOT.MQTT_topic,"dP", "pid_dp");
    sensorPID_P.setDeviceClass(temperature_str); 
    sensorPID_D.setAvailability(true);
    sensorPID_D.setNameUniqueIdStr(SmOT.MQTT_topic,"dD", "pid_dd");
    sensorPID_D.setDeviceClass(temperature_str); 
    sensorPID_I.setAvailability(true);
    sensorPID_I.setNameUniqueIdStr(SmOT.MQTT_topic,"dI", "pid_di");
    sensorPID_I.setDeviceClass(temperature_str); 
    sensorPID_U.setAvailability(true);
    sensorPID_U.setNameUniqueIdStr(SmOT.MQTT_topic,"U", "pid_u");
    sensorPID_U.setDeviceClass(temperature_str); 
    sensorPID_U0.setAvailability(true);
    sensorPID_U0.setNameUniqueIdStr(SmOT.MQTT_topic,"U0", "pid_u0");
    sensorPID_U0.setDeviceClass(temperature_str); 
            sprintf(str,"%.4f", SmOT.mypid.ub);
            sensorPID_U0.setValue(str);
//    Serial_db.printf("sensorPID_U0 =%s\n", str);

    
    sensor_Eff_Mod.setAvailability(true);
    sensor_Eff_Mod.setNameUniqueIdStr(SmOT.MQTT_topic,"EffModH", "effmod_h");
    sensor_Eff_Mod.setIcon("mdi:fire");
    sensor_Eff_Mod.setDeviceClass("power_factor"); 
    sensor_Eff_Mod.setUnitOfMeasurement("%");

    //SmOT.Bstat.Eff_Mod_h_prev

#endif
    mqtt.onConnected(OnMQTTconnected);
    mqtt.onDisconnected(OnMQTTdisconnected);
    SmOT.stsMQTT = 1;
    mqtt._mqtt->setSocketTimeout(1); //not work ???


//    rc= mqtt.begin(SmOT.MQTT_server,  SmOT.MQTT_user, SmOT.MQTT_pwd);
    rc= mqtt.begin(SmOT.MQTT_server, SmOT.MQTT_port, SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    {  Serial_db.printf("mqtt.begin ok %s:%d %s %s\n", SmOT.MQTT_server, SmOT.MQTT_port, SmOT.MQTT_user, SmOT.MQTT_pwd);
      SmOT.stsMQTT = 2;
    } else {
   Serial_db.printf("mqtt.begin false\n");
    }
}


void OnMQTTconnected(void)
{ 
  statemqtt = 1;
   Serial_db.printf("On MQTTconnected %d\n", statemqtt );

}
void OnMQTTdisconnected(void)
{ statemqtt = 0;
   Serial_db.printf("On MQTT DISconnected %d\n", statemqtt );
}

void mqtt_start(void)
{
   Serial_db.printf("mqtt_start SmOT.stsMQTT %d\n", SmOT.stsMQTT);
  if(SmOT.stsMQTT == 0)
  {   mqtt_setup();
  } else {
    int rc;
    rc= mqtt.begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    { Serial_db.printf("(1) mqtt.begin ok %s %s %s\n", SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
      SmOT.stsMQTT = 2;
    } else {
      Serial_db.printf("(1)mqtt.begin false\n");
    }
  }
}

void mqtt_loop(void)
{ char str[80];
static int st_old = -2, raz=0;  
unsigned long t0, t00=0;
int dt;

raz++;

if(SmOT.stsMQTT == 0) 
{ t0 = millis();  
  mqtt_setup();
  dt = millis() - t0;
//  if(dt > 100)
//  if(SmOT.stsMQTT != 0)
//      Serial_db.printf("MQTT 0 dt %d t %d %d\n", dt, t0, raz );

     return;
}

    t0 = millis();  
    mqtt.loop();
    dt = millis() - t0;
//    if(dt > 100)
//        Serial_db.printf("MQTT 1 dt %d t %d %d\n", dt, t0, raz );
  
  
    if(mqtt.isConnected())
    {   if(statemqtt != 1)
            Serial.println(F("MQTT connected"));

        statemqtt = 1;
        state_mqtt = mqtt._mqtt->state();
    } else {
        if(statemqtt != 0)
            Serial.println(F("MQTT DiSconnected"));
        statemqtt = 0;
        state_mqtt = mqtt._mqtt->state();
        delay(1);
        return; // return from   mqtt_loop() if not connected
    }

    t00 = millis();  
    dt = t00 - lastAvailabilityToggleAt;
    if ((dt > SmOT.MQTT_interval*1000) || (SmOT.MQTT_need_report && dt > 1000))
    {   
//      Serial_db.printf("MQTT 10 t %d %d\n", millis() , raz );

        if(SmOT.stsOT == -1)
        { sensorOT.setAvailability(false);
          sensorState.setValue("OpenTherm не подключен");
        } else {
          sensorOT.setAvailability(true);
          if(SmOT.stsOT == 2)
          { 
            t0 = millis();
            sensorOT.setState(false);
            hvac.setAvailability(false);
            sensorBoilerT.setAvailability(false);
            sensorFlame.setAvailability(false);
            sensor_CH.setAvailability(false);
            if(SmOT.HotWater_present)
            { sensor_HW.setAvailability(false);
              hvacDHW.setAvailability(false);
            }
            sensor_CMD_on.setAvailability(false);
            sensor_CMD_CH_on.setAvailability(false);

            sensorModulation.setAvailability(false);
            if(SmOT.RetT_present)
              sensorBoilerRetT.setAvailability(false);
            if(SmOT.Pressure_present)
              sensorPressure.setAvailability(false);
            if(SmOT.DHWFlowRate_present)
                sensorDHWFlowRate.setAvailability(false);
            if(SmOT.Toutside_present)
              sensorText.setAvailability(false);
            sensorState.setValue("OpenTherm: потеря связи");
#if PID_USE            
            hvacPID.setAvailability(false);
#endif            
            dt = millis() - t0;
//            if(dt > 100)
                Serial_db.printf("MQTT 2 dt %d t %d %d\n", dt, t0, raz );

          } else {
            if(st_old != SmOT.stsOT)
            {
              t0 = millis();
              sensorOT.setState(true);
              sensorBoilerT.setAvailability(true);
              hvac.setAvailability(true);
//Serial_db.printf("hvac.setAvailability(true)\n");
              sensorFlame.setAvailability(true);
              sensor_CH.setAvailability(true);
              if(SmOT.HotWater_present)
              {   sensor_HW.setAvailability(true);
                  hvacDHW.setAvailability(true);
              }
#if PID_USE            
        if(SmOT.usePID > 0)
              hvacPID.setAvailability(true);
#endif            

              sensor_CMD_on.setAvailability(true);
              sensor_CMD_CH_on.setAvailability(true);

              sensorModulation.setAvailability(true);
              if(SmOT.RetT_present)
                sensorBoilerRetT.setAvailability(true);
              if(SmOT.Pressure_present)
                sensorPressure.setAvailability(true);
              if(SmOT.DHWFlowRate_present)
                sensorDHWFlowRate.setAvailability(true);
              if(SmOT.Toutside_present)
                sensorText.setAvailability(true);
              dt = millis() - t0;
  //     if(dt > 100)
              Serial_db.printf("MQTT 3 dt %d t %d %d\n", dt, t0, raz );
            }
/******************/
            t0 = millis();
            MQTTsenddata();
            dt = millis() - t0;
            if(dt > 100)
                Serial_db.printf("MQTT 4 dt %d\n", dt);
        /******************/
            
/*************************************************/            
          }
        }
        st_old = SmOT.stsOT;
        if(SmOT.stsT1 >= 0)
        {  
#if PID_USE
            if(SmOT.usePID) 
            {  if(SmOT.t_mean[0].can_report)
               { sprintf(str,"%.3f", SmOT.t_mean[0].x);
                  sensorT1.setValue(str);
                  SmOT.t_mean[0].can_report = 0; 
//   Serial_db.printf("***MQTT T1=%s\n",  str); 
                }
            }  else { 
                sprintf(str,"%.3f", SmOT.t1);
                sensorT1.setValue(str); 
            } 
#else
                sprintf(str,"%.3f", SmOT.t1);
                sensorT1.setValue(str); 
#endif             
        }
        if(SmOT.stsT2 >= 0)
        { 
#if PID_USE
            if(SmOT.usePID)
            {   if(SmOT.t_mean[1].can_report)
                { sprintf(str,"%.3f", SmOT.t_mean[1].x);
                  sensorT2.setValue(str);
                  SmOT.t_mean[1].can_report = 0;
                }
            } else { 
                sprintf(str,"%.3f", SmOT.t2);
                sensorT2.setValue(str);
            }  
#else
                sprintf(str,"%.3f", SmOT.t2);
                sensorT2.setValue(str); 
#endif             
        
        }

        { static int raz = 0;
          if(raz++ == 0)
          { sprintf(str,"%d",  ESP.getFreeHeap() );
            sensorFreeRam.setValue(str);  
          } else {
            if(raz == 100)
                raz = 0;
          }
        }

        lastAvailabilityToggleAt = millis();
        SmOT.MQTT_need_report = 0;
    }
//    dt = millis() - t00;
//    Serial_db.printf("MQTT 40 dt %d\n", dt);

}

void MQTTsenddata(void)
{ char str[120];
  sprintf(str,"%.3f", SmOT.BoilerT);           
  sensorBoilerT.setValue(str);
  hvac.setCurrentTemperature(SmOT.BoilerT);
  hvac.setTargetTemperature(SmOT.Tset);
#if  PID_USE
  if(SmOT.enable_CentralHeating_real)
#else
  if(SmOT.enable_CentralHeating)
#endif
    hvac.setMode(HAHVAC::HeatMode);
  else
    hvac.setMode(HAHVAC::OffMode);

#if  PID_USE
  if(SmOT.IsSetTemp & 0x01)
    hvacPID.setCurrentTemperature(SmOT.tempindoor);
  hvacPID.setTargetTemperature(SmOT.TroomTarget);
#endif

  if(SmOT.BoilerStatus & 0x08)
        sensorFlame.setState(true); 
  else
        sensorFlame.setState(false); 

  if(SmOT.BoilerStatus & 0x02)
        sensor_CH.setState(true); 
  else
        sensor_CH.setState(false); 

  if(SmOT.HotWater_present)
  {
    if(SmOT.BoilerStatus & 0x04)
    {      sensor_HW.setState(true); 
    }  else {
          sensor_HW.setState(false); 
    }
    if(SmOT.enable_HotWater)
        hvacDHW.setMode(HAHVAC::HeatMode);
    else
        hvacDHW.setMode(HAHVAC::OffMode);

    if(SmOT.Use_ID29_DHW_flag && ot.OTid_used(OpenThermMessageID::Tstorage))
        hvacDHW.setCurrentTemperature(SmOT.Tstorage);
    else if(SmOT.Dhw_t_present)
        hvacDHW.setCurrentTemperature(SmOT.dhw_t);
      
    hvacDHW.setTargetTemperature(SmOT.TdhwSet);
//   Serial_db.printf("SmOT.TdhwSet %f SmOT.dhw_t %f\n", SmOT.TdhwSet, SmOT.dhw_t );

  }
  sprintf(str,"%.3f", SmOT.FlameModulation);
  sensorModulation.setValue(str);
  if(SmOT.RetT_present)
  { sprintf(str,"%.3f", SmOT.RetT);
    sensorBoilerRetT.setValue(str);  
  }
  if(SmOT.Pressure_present)
  { sprintf(str,"%.3f", SmOT.Pressure);
    sensorPressure.setValue(str);
  }

  if(SmOT.DHWFlowRate_present)
  {   sprintf(str,"%.3f", SmOT.DHWFlowRate);
      sensorDHWFlowRate.setValue(str);
  }

  if(SmOT.Toutside_present)
  { sprintf(str,"%.3f", SmOT.Toutside);
    sensorText.setValue(str);
  }

#if PID_USE
  {
      sprintf(str,"%.4f", SmOT.mypid.dP);
      sensorPID_P.setValue(str);
      sprintf(str,"%.4f", SmOT.mypid.dD);
      sensorPID_D.setValue(str);
      sprintf(str,"%.4f", SmOT.mypid.dI);
      sensorPID_I.setValue(str);
      sprintf(str,"%.4f", SmOT.mypid.u);
      sensorPID_U.setValue(str);
      sprintf(str,"%.4f", SmOT.mypid.ub);
      sensorPID_U0.setValue(str);
        
//            Serial_db.printf("srcText %d srcTroom  %d\n",SmOT.srcText, SmOT.srcTroom );

      if((SmOT.srcTroom >= 0 && SmOT.srcTroom < 3) && (SmOT.IsSetTemp & 0x01))
      {  numT_indoor.setState(SmOT.tempindoor, true);
      }
      if((SmOT.srcText >= 0 && SmOT.srcText < 3) && (SmOT.IsSetTemp & 0x02))
      {   numT_outdoor.setState(SmOT.tempoutdoor, true);
      }

//            sprintf(str,"isset %d nx %d xmean %.3f x %.3f", SmOT.t_mean[4].isset, SmOT.t_mean[4].nx, SmOT.t_mean[4].xmean,  SmOT.t_mean[4].x);
//            textPIDinfo.setValue(str);

  }

#endif

  if(SmOT.OEMDcode || SmOT.Fault ||  (SmOT.needReport_CrasyState&0x01) )
  {  str[0] = 0;
    if(SmOT.Fault)
    { if (SmOT.OEMDcode)
      {
        sprintf(str, "OT Fault %x OEMDcode %x", SmOT.Fault, SmOT.OEMDcode);
      } else {
        sprintf(str, "OT Fault %x", SmOT.Fault);
      }
    } else if (SmOT.OEMDcode) {
        sprintf(str, "OEMDcode %x", SmOT.OEMDcode);
    }
    if( SmOT.needReport_CrasyState & 0x01 )
    {  SmOT.needReport_CrasyState &= ~0x01;
       strcat(str,"CrasyState");
    }
    sensorState.setValue(str);
  } else {
      sensorState.setValue("нет");
  }
#if 0  
  todo     
        if(SmOT.Fault)
        { sprintf(str0, "Fault = %x (HB) %x (LB)<br>", (SmOT.Fault>>8)&0xff, (SmOT.Fault&0xff));
          Info6.value += str0;
          if(SmOT.Fault & 0xff00)
          {    if(SmOT.Fault & 0x0100)
                   Info6.value += " Service request";
               if(SmOT.Fault & 0x0200)
                   Info6.value += " Lockout-reset";
               if(SmOT.Fault & 0x0400)
                   Info6.value += " Lowwater press";
               if(SmOT.Fault & 0x0800)
                   Info6.value += " Gas/flame fault";
               if(SmOT.Fault & 0x01000)
                   Info6.value += " Air press fault";
               if(SmOT.Fault & 0x02000)
                   Info6.value += " Water over-temp fault";
          }
          if(SmOT.Fault & 0x00ff)
          {    sprintf(str0, " OEM-specific fault/error cod = %d ( hex %x)", (SmOT.Fault&0xff), (SmOT.Fault&0xff));
              Info6.value += str0;
          }
          Info6.value += "<br>";
        }
        if(SmOT.OEMDcode)
        {     sprintf(str0, "OEM-specific diagnostic/service code = %d  ( hex %x)<br>", SmOT.OEMDcode, SmOT.OEMDcode);
              Info6.value += str0;
        }
#endif //0
/*******************************************/
  

}

void MQTT_pub_Eff_Mod_h(void)
{ char str[80];
  if(SmOT.stsMQTT != 2)
    return;
  sprintf(str,"%.4f", SmOT.Bstat.Eff_Mod_h_prev);
  sensor_Eff_Mod.setValue(str);
}

int MQTT_pub_data(void)
{
//Serial_db.printf("todo %s\n",__FUNCTION__ );
    return 0;

}

#if RELAY_USE
void  MQTT_pub_relay(void)
{
  if(SmOT.stsMQTT != 2)
    return;
  relayHA.setState(SmOT.Relay_sts);
}
#endif
 
void  MQTT_pub_cmd2(int val)
{ if(SmOT.stsMQTT != 2)
    return;
// char str[80];
//  sprintf(str,"%d",  val);
//  sensor_TestNum.setValue(str);  
}

void  MQTT_pub_cmd(int on)
{ 
  if(SmOT.stsMQTT == 2)
  { if(on)
    sensor_CMD_on.setState(true); 
  else
    sensor_CMD_on.setState(false); 
  }
}

int  MQTT_pub_cmdCH(int on)
{ 

//  Serial_db.printf("MQTT_pub_cmdCH %d SmOT.stsMQTT %d\n", on, SmOT.stsMQTT );

if(SmOT.stsMQTT == 2)
  { if(on)
      sensor_CMD_CH_on.setState(true); 
    else
      sensor_CMD_CH_on.setState(false); 
    return 1;
  } else {
    return 0;
  }
}

int  MQTT_pub_usePID(void)
{
#if PID_USE
  if(SmOT.stsMQTT == 2)
  {
    if(SmOT.usePID)
            hvacPID.setMode(HAHVAC::AutoMode);
    else
            hvacPID.setMode(HAHVAC::OffMode);
  }            
#endif 
  return 0;           
}
/*******************************************************************************/


#endif  //MQTT_USE 