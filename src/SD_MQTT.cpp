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

extern WiFiClient tcp_client;

WiFiClient espClient;
PubSubClient client(espClient);
extern  SD_Termo SmOT;
extern OpenTherm ot;

/*******************************************************************************/
//HADevice *pHAdevice;
//HAMqtt *pMqtt;

HADevice device;
#if PID_USE
HAMqtt mqtt(espClient, device,25);
#else
HAMqtt mqtt(espClient, device,12);
#endif

const char * temperature_str = "temperature";

HABinarySensor sensorOT(NULL);
HABinarySensor sensorFlame(NULL);
HABinarySensor sensor_CH(NULL);
HABinarySensor sensor_HW(NULL);
HABinarySensor sensor_CMD_on(NULL);
HABinarySensor sensor_CMD_CH_on(NULL);
HASensor sensorModulation(NULL);
HASensor sensorBoilerT(NULL);
HASensor sensorBoilerRetT(NULL);
HASensor sensorPressure(NULL);
HASensor sensorT1(NULL);
HASensor sensorT2(NULL);
HASensor sensorText(NULL);
HASensor sensorFreeRam(NULL);
HASensor sensorState(NULL);
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
HASensor sensorPID_Extra(NULL,HANumber::PrecisionP3);

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

unsigned long lastReadAt = millis();
unsigned long lastAvailabilityToggleAt = millis();
bool lastInputState = false;
void OnMQTTconnected(void);
void OnMQTTdisconnected(void);

void onTargetTemperatureCommand(HANumeric temperature, HAHVAC* sender) {
    float temperatureFloat = temperature.toFloat();
    if(sender == &hvacDHW)
    {
      SmOT.TdhwSet = temperatureFloat;
      SmOT.need_set_dhwT = 2;
#if SERIAL_DEBUG      
    Serial.print("DHW Target temperature: ");
    Serial.println(temperatureFloat);
#endif    
      sender->setTargetTemperature(temperature); // report target temperature back to the HA panel
    } else {
      SmOT.Tset = temperatureFloat;
      SmOT.need_set_T = 2;
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
    Serial.printf("SmOT.enable_HotWater %d\n", SmOT.enable_HotWater);
}

#if PID_USE

void onNumberCommand(HANumeric number, HANumber* sender)
{   float t = number.toFloat();
//    if (sender == &numPID_v) {
//      Serial.printf("NumberCommand numPID_v: %f\n",t);
//
//    } else 
    
    if (sender == &numT_outdoor) {
#if SERIAL_DEBUG      
      Serial.printf("*************************\n");
      Serial.printf("NumberCommand numT_outdoor: %f (%d)\n", t, millis()/1000);
#endif      
      SmOT.OnChangeT(t,4);
        
    } else if (sender == &numT_indoor) {
#if SERIAL_DEBUG      
//      Serial.printf("NumberCommand numT_indoor: %f (%d)\n", t, millis()/1000);
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

int statemqtt = -1;
int state_mqtt = -10000;
int attempt_mqtt = 0;

/************************************************************/
void mqtt_setup(void)
{  bool rc;
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

  if( mqtt.getDevicesTypesNb_toreg() > mqtt.getDevicesTypesNb())
  {
      Serial.printf("Error! Nb = %d, need be %d\n", mqtt.getDevicesTypesNb(),  mqtt.getDevicesTypesNb_toreg() );
    return;
  }

   device.setUniqueIdStr(SmOT.MQTT_topic);

   device.setName(SmOT.MQTT_topic,SmOT.MQTT_devname); //должно быть static!!
  { static char str[40];
    sprintf(str,"%d.%d.%d %s" , SmOT.Vers,SmOT.SubVers,SmOT.SubVers1, SmOT.BiosDate);
    device.setSoftwareVersion(str); //должно быть static!!
  }

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
    sensorModulation.setDeviceClass("power_factor"); //"temperature"
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
    
    if(SmOT.RetT_present)
    { sensorBoilerRetT.setNameUniqueIdStr(SmOT.MQTT_topic,"Температура обратки", "RetT");
      sensorBoilerRetT.setAvailability(false);
      sensorBoilerRetT.setDeviceClass(temperature_str); 
      sensorBoilerRetT.setUnitOfMeasurement("°C");
    }

   Serial.printf("(3)mqtt_setup SmOT.Pressure_present %d SmOT.stsOT  %d \n", 
          SmOT.Pressure_present, SmOT.stsOT);


    if(SmOT.Pressure_present || SmOT.stsOT == -1)
    { sensorPressure.setNameUniqueIdStr(SmOT.MQTT_topic,"Давление", "Pressure");
      sensorPressure.setAvailability(false);
      sensorPressure.setDeviceClass("pressure"); 
    }

    sensorFreeRam.setAvailability(true);
    sensorFreeRam.setNameUniqueIdStr(SmOT.MQTT_topic,"Free RAM", "FreeRAM");
    sensorFreeRam.setDeviceClass("data_size"); 

    // assign callbacks (optional)
    hvac.onTargetTemperatureCommand(onTargetTemperatureCommand);
    hvac.onPowerCommand(onPowerCommand);
    hvac.onModeCommand(onModeCommand);

    // configure HVAC (optional)
    hvac.setNameUniqueIdStr(SmOT.MQTT_topic,"Котёл", "Boiler");

    hvac.setMinTemp(10);
    hvac.setMaxTemp(80);
    hvac.setTempStep(0.1);
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
      hvacDHW.setNameUniqueIdStr(SmOT.MQTT_topic,"Горячая вода", "DHW");
      hvacDHW.setMinTemp(30);
      hvacDHW.setMaxTemp(80);
      hvacDHW.setTempStep(0.5);

      hvacDHW.setModes(HAHVAC::OffMode|HAHVAC::HeatMode);

      if(SmOT.enable_HotWater)
            hvacDHW.setMode(HAHVAC::HeatMode);
      else
            hvacDHW.setMode(HAHVAC::OffMode);
      hvacDHW.setAvailability(false);
    }

    if(SmOT.stsT1 >= 0 )
    { sensorT1.setAvailability(true);
      sensorT1.setNameUniqueIdStr(SmOT.MQTT_topic,"T1", "T1");
      sensorT1.setDeviceClass(temperature_str); 
    }  else {
      sensorT1.setAvailability(false);
    }

    if(SmOT.stsT2 >= 0 )
    { sensorT2.setAvailability(true);
      sensorT2.setNameUniqueIdStr(SmOT.MQTT_topic,"T2", "T2");
      sensorT2.setDeviceClass(temperature_str); 
    }  else {
      sensorT2.setAvailability(false);
    }

    sensorText.setAvailability(false);
    sensorText.setNameUniqueIdStr(SmOT.MQTT_topic,"Tвн", "Toutside");
    sensorText.setDeviceClass(temperature_str); 

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
    numT_outdoor.setState(10.f, true);
    numT_outdoor.setCurrentState(10.f);
    numT_outdoor.setStep(0.1);
    numT_outdoor.setMin(-50.);
    numT_outdoor.setMax( 50.);
    numT_outdoor.onCommand(onNumberCommand);

    numT_indoor.setAvailability(true);
    numT_indoor.setNameUniqueIdStr(SmOT.MQTT_topic,"T indoor", "Tindoor");
    numT_indoor.setMode(HANumber::ModeBox);
    numT_indoor.setState(20.f, true);
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
    
    sensorPID_Extra.setAvailability(true);
    sensorPID_Extra.setNameUniqueIdStr(SmOT.MQTT_topic,"Uextra", "pid_extra");
    sensorPID_Extra.setDeviceClass(temperature_str); 

#endif
    mqtt.onConnected(OnMQTTconnected);
    mqtt.onDisconnected(OnMQTTdisconnected);
    SmOT.stsMQTT = 1;
    mqtt._mqtt->setSocketTimeout(1); //not work ???

    rc= mqtt.begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    {  Serial.printf("mqtt.begin ok %s %s %s\n", SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
      SmOT.stsMQTT = 2;
    } else {
   Serial.printf("mqtt.begin false\n");
    }
}


void OnMQTTconnected(void)
{ 
  statemqtt = 1;
//   Serial.printf("OnMQTTconnected %d\n", statemqtt );

}
void OnMQTTdisconnected(void)
{ statemqtt = 0;
//   Serial.printf("OnMQTT disconnected %d\n", statemqtt );
}

void mqtt_start(void)
{
   Serial.printf("mqtt_start SmOT.stsMQTT %d\n", SmOT.stsMQTT);
  if(SmOT.stsMQTT == 0)
  {   mqtt_setup();
  } else {
    int rc;
    rc= mqtt.begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    { Serial.printf("(1) mqtt.begin ok %s %s %s\n", SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
      SmOT.stsMQTT = 2;
    } else {
      Serial.printf("(1)mqtt.begin false\n");
    }
  }
}

void mqtt_loop(void)
{ int sts;
  char str[80];
static int st_old = -2;  
static unsigned int t0=0;
unsigned long t1;
int dt;


if(SmOT.stsMQTT == 0) 
{   mqtt_setup();
     return;
}

    mqtt.loop();

    if(mqtt.isConnected())
    {   if(statemqtt != 1)
            Serial.println(F("MQTT connected"));
        statemqtt = 1;
    } else {
        if(statemqtt != 0)
            Serial.println(F("MQTT DiSconnected"));
        statemqtt = 0;
        delay(1);
        return; // return from   mqtt_loop() if not connected
    }

    if ((millis() - lastAvailabilityToggleAt) > SmOT.MQTT_interval*1000 || SmOT.MQTT_need_report)
    {   if(SmOT.stsOT == -1)
        { sensorOT.setAvailability(false);
          sensorState.setValue("OpenTherm не подключен");
        } else {
          sensorOT.setAvailability(true);
          if(SmOT.stsOT == 2)
          { 
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
            sensorText.setAvailability(false);
            sensorState.setValue("OpenTherm: потеря связи");
          } else {
            if(st_old != SmOT.stsOT)
            {
              sensorOT.setState(true);
              sensorBoilerT.setAvailability(true);
              hvac.setAvailability(true);
//Serial.printf("hvac.setAvailability(true)\n");
              sensorFlame.setAvailability(true);
              sensor_CH.setAvailability(true);
              if(SmOT.HotWater_present)
              {   sensor_HW.setAvailability(true);
                  hvacDHW.setAvailability(true);
              }
              sensor_CMD_on.setAvailability(true);
              sensor_CMD_CH_on.setAvailability(true);

              sensorModulation.setAvailability(true);
              if(SmOT.RetT_present)
                sensorBoilerRetT.setAvailability(true);
              if(SmOT.Pressure_present)
                  sensorPressure.setAvailability(true);
              sensorText.setAvailability(true);
            }
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

              hvacDHW.setCurrentTemperature(SmOT.dhw_t);
              hvacDHW.setTargetTemperature(SmOT.TdhwSet);
//   Serial.printf("SmOT.TdhwSet %f SmOT.dhw_t %f\n", SmOT.TdhwSet, SmOT.dhw_t );

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
            sprintf(str,"%.3f", SmOT.Toutside);
            sensorText.setValue(str);

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
            sprintf(str,"%.4f", SmOT.mypid.dP + SmOT.mypid.dD);
            sensorPID_Extra.setValue(str);
            
//Serial.printf("srcText %d srcTroom  %d\n",SmOT.srcText, SmOT.srcTroom );
            if(SmOT.srcText >= 0 && SmOT.srcText < 3)
            {   numT_outdoor.setState(SmOT.tempoutdoor, true);
            }
            if(SmOT.srcTroom >= 0 && SmOT.srcTroom < 3)
            {  numT_indoor.setState(SmOT.tempindoor, true);
            }

//            sprintf(str,"isset %d nx %d xmean %.3f x %.3f", SmOT.t_mean[4].isset, SmOT.t_mean[4].nx, SmOT.t_mean[4].xmean,  SmOT.t_mean[4].x);
//            textPIDinfo.setValue(str);

        }

#endif

/*************************************************/            
    if(SmOT.OEMDcode || SmOT.Fault)
    { 
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
       sensorState.setValue(str);
    } else {
        sensorState.setValue("нет");
    }

#if 0       
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
        }
        st_old = SmOT.stsOT;
        if(SmOT.stsT1 >= 0)
        {   sprintf(str,"%.3f", SmOT.t1);
            sensorT1.setValue(str);  
        }
        if(SmOT.stsT2 >= 0)
        {   sprintf(str,"%.3f", SmOT.t2);
            sensorT2.setValue(str);  
        }

        sprintf(str,"%d",  ESP.getFreeHeap() );
        sensorFreeRam.setValue(str);  
        lastAvailabilityToggleAt = millis();
        SmOT.MQTT_need_report = 0;
    }

}

int MQTT_pub_data(void)
{
//Serial.printf("todo %s\n",__FUNCTION__ );
    return 0;

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
/*******************************************************************************/


#endif  //MQTT_USE 