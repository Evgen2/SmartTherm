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

void mqtt_loop(void);
void mqtt_setup(void);
int MQTT_pub_data(void);

extern WiFiClient tcp_client;

WiFiClient espClient;
PubSubClient client(espClient);
extern  SD_Termo SmOT;
extern OpenTherm ot;

#define MQTT_VERS 1

#if MQTT_VERS == 1

/*******************************************************************************/
//HADevice *pHAdevice;
//HAMqtt *pMqtt;

HADevice device;
HAMqtt mqtt(espClient, device,12);


HABinarySensor sensorOT("OpenTherm");
HABinarySensor sensorFlame("Flame");
HASensor  sensorBoilerT("BoilerT");
HASensor  sensorModulation("Modulation");
HASensor  sensorBoilerRetT("RetT");
HASensor  sensorPressure("Pressure");
HASensor  sensorT1("T1");
HASensor  sensorT2("T2");
HASensor  sensorText("Text");
HASensor  sensorFreeRam("FreeRAM");

const char * temperature_str = "temperature";
// By default HAHVAC supports only reporting of the temperature.
// You can enable feature you need using the second argument of the constructor.
// Please check the documentation of the HAHVAC class.
HAHVAC hvac(
  "st",
  HAHVAC::TargetTemperatureFeature | HAHVAC::PowerFeature | HAHVAC::ModesFeature |HAHVAC::ActionFeature,
  HANumber::PrecisionP2
);
//
// HAHVAC::ActionFeature

/* 
| HAHVAC::AuxHeatingFeature
MQTT entities with auxiliary heat support found
Это приведет к неисправностям в версии 2024.3.0. Пожалуйста, устраните эту проблему перед обновлением. 
Entity climate.st_test_mqtt_mqtt_hvac has auxiliary heat support enabled, which has been deprecated for MQTT climate devices. Please adjust your configuration and remove deprecated config options from your configuration and restart Home Assistant to fix this issue
*/
unsigned long lastReadAt = millis();
unsigned long lastAvailabilityToggleAt = millis();
bool lastInputState = false;

void onTargetTemperatureCommand(HANumeric temperature, HAHVAC* sender) {
    float temperatureFloat = temperature.toFloat();

    Serial.print("Target temperature: ");
    Serial.println(temperatureFloat);
    SmOT.Tset = temperatureFloat;
    SmOT.need_set_T = 2;
    sender->setTargetTemperature(temperature); // report target temperature back to the HA panel
}

void onPowerCommand(bool state, HAHVAC* sender) {
  if (state) {
    Serial.println("Power on");
  } else {
    Serial.println("Power off");
  }
}

void onModeCommand(HAHVAC::Mode mode, HAHVAC* sender) {
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

void mqtt_setup(void)
{  bool rc;
  if (WiFi.status() != WL_CONNECTED)  
        return;
  Serial.printf("todo %s\n",__FUNCTION__ );

  if( mqtt.getDevicesTypesNb_toreg() > mqtt.getDevicesTypesNb())
  {
      Serial.printf("Error! Nb = %d, need be %d\n", mqtt.getDevicesTypesNb(),  mqtt.getDevicesTypesNb_toreg() );
  
  }

   device.setUniqueIdStr(SmOT.MQTT_topic);

   device.setName("ST"); //должно быть static!!
  { static char str[40];
    sprintf(str,"%d.%d", SmOT.Vers,SmOT.SubVers);
    device.setSoftwareVersion(str); //должно быть static!!
  }

    lastReadAt = millis();
    lastAvailabilityToggleAt = millis();
    sensorOT.setAvailability(false);
    sensorOT.setCurrentState(false); 
    sensorOT.setName("OpenTherm"); 
    sensorOT.setDeviceClass("connectivity"); 
    
    sensorFlame.setName("Горелка");
    sensorFlame.setCurrentState(false); 
    sensorFlame.setAvailability(false);
    sensorFlame.setIcon("mdi:fire");
//    sensorFlame.setDeviceClass("None");

    sensorModulation.setName("Модуляция");
    sensorModulation.setAvailability(false);
    sensorModulation.setIcon("mdi:fire");
    sensorModulation.setDeviceClass("power_factor"); //"temperature"

    sensorBoilerT.setAvailability(false);
    sensorBoilerT.setName("Температура теплоносителя"); 
    sensorBoilerT.setDeviceClass(temperature_str);

    sensorBoilerRetT.setAvailability(false);
    sensorBoilerRetT.setName("Обратка");
    sensorBoilerRetT.setDeviceClass(temperature_str); 

    sensorPressure.setAvailability(false);
    sensorPressure.setName("Давление");
    sensorPressure.setDeviceClass("pressure"); 

    sensorFreeRam.setAvailability(true);
    sensorFreeRam.setName("Free RAM");
    sensorFreeRam.setDeviceClass("data_size"); 


    // assign callbacks (optional)
    hvac.onTargetTemperatureCommand(onTargetTemperatureCommand);
    hvac.onPowerCommand(onPowerCommand);
    hvac.onModeCommand(onModeCommand);

    // configure HVAC (optional)
    hvac.setName("Котёл");
    hvac.setMinTemp(10);
    hvac.setMaxTemp(80);
    hvac.setTempStep(0.1);
    hvac.setModes(HAHVAC::OffMode|HAHVAC::HeatMode);
    hvac.setMode(HAHVAC::HeatMode);
    hvac.setAvailability(false);
  

    if(SmOT.stsT1 >= 0 )
      sensorT1.setAvailability(true);
    else 
      sensorT1.setAvailability(false);
    sensorT1.setName("T1");
    sensorT1.setDeviceClass(temperature_str); 
    if(SmOT.stsT2 >= 0 )
      sensorT2.setAvailability(true);
    else
    sensorT2.setAvailability(false);
    sensorT2.setName("T2");
    sensorT2.setDeviceClass(temperature_str); 

    sensorText.setAvailability(false);
    sensorText.setName("Text");
    sensorText.setDeviceClass(temperature_str); 


//    rc= pMqtt->begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    rc= mqtt.begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    {
   Serial.printf("mqtt.begin ok %s %s %s\n", SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);

    }

}

int statemqtt = -1;
int state_mqtt = -10000;

void mqtt_loop(void)
{ int sts;
  char str[40];
static int st_old = -2;  
//      pMqtt->loop();
      mqtt.loop();
//      if(pMqtt->isConnected())
      if(mqtt.isConnected())
      {   if(statemqtt != 1)
              Serial.println(F("MQTT connected"));
          statemqtt = 1;
      } else {
          if(statemqtt != 0)
              Serial.println(F("MQTT DiSconnected"));
          statemqtt = 0;
      }

//    sts = pMqtt->_mqtt->state(); 
    sts = mqtt._mqtt->state(); 
      if(sts !=state_mqtt )
      {   Serial.printf("MQTT state=%d\n", sts);
          state_mqtt = sts;
      }
//Serial.printf("todo %s\n",__FUNCTION__ );
    if ((millis() - lastAvailabilityToggleAt) > 10000) {

        if(SmOT.stsOT == -1)
        { sensorOT.setAvailability(false);
        } else {
          sensorOT.setAvailability(true);
          if(SmOT.stsOT == 2)
          { 
            sensorOT.setState(false);
            hvac.setAvailability(false);
            sensorBoilerT.setAvailability(false);
            sensorFlame.setAvailability(false);
            sensorModulation.setAvailability(false);
            sensorBoilerRetT.setAvailability(false);
            sensorPressure.setAvailability(false);
            sensorText.setAvailability(false);

          } else {
            if(st_old != SmOT.stsOT)
            {
              sensorOT.setState(true);
              sensorBoilerT.setAvailability(true);
              hvac.setAvailability(true);
              sensorFlame.setAvailability(true);
              sensorModulation.setAvailability(true);
              sensorBoilerRetT.setAvailability(true);
              sensorPressure.setAvailability(true);
              sensorText.setAvailability(true);
            }

            sprintf(str,"%.3f", SmOT.BoilerT);
            sensorBoilerT.setValue(str);
        hvac.setCurrentTemperature(SmOT.BoilerT);
        hvac.setTargetTemperature(SmOT.Tset);
            if(SmOT.BoilerStatus & 0x08)
                  sensorFlame.setState(true); 
            else
                  sensorFlame.setState(false); 

            sprintf(str,"%.3f", SmOT.FlameModulation);
            sensorModulation.setValue(str);
            sprintf(str,"%.3f", SmOT.RetT);
            sensorBoilerRetT.setValue(str);  
            sprintf(str,"%.3f", SmOT.Pressure);
            sensorPressure.setValue(str);  
            sprintf(str,"%.3f", SmOT.Toutside);
            sensorText.setValue(str);  
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
    }

}

int MQTT_pub_data(void)
{
//Serial.printf("todo %s\n",__FUNCTION__ );
    return 0;

}

/*******************************************************************************/
#elif MQTT_VERS == 0

#define MSG_BUFFER_SIZE	(50)
static char msg[MSG_BUFFER_SIZE];
static char topic[MSG_BUFFER_SIZE];

const char* mqtt_user = "SmartTherm";
const char* mqtt_password = "smart_2024";
unsigned long lastReconnectAttempt = 0;
unsigned long lastMsg = 0;
//  state_topic: "srv/t1"  
// MQTT topics

const char* AVAILABLE_OPENTHEM_TOPIC = "OT_available";
const char* AVAILABLE_CH_TOPIC = "CH_available";
const char* STATE_CH_TOPIC = "CH_state";
const char* ENABLE_CH_GET_TOPIC = "CH_set";   //enable_CentralHeating
const char* TSet_GET_TOPIC = "Tset";
const char* T1_TEMP_TOPIC = "t1";
const char* T2_TEMP_TOPIC = "t2";
const char* Toutside_TEMP_TOPIC = "Toutside";
const char* BoilerT_TEMP_TOPIC = "BoilerT";
const char* RetT_TEMP_TOPIC = "RetT";
const char* FlameModulation_TEMP_TOPIC = "FlameModulation";

int MQTTreconnect(void) 
{ int rc = 0;
  if(client.connected())
      return 1;
#if SERIAL_DEBUG      
  Serial.print("Attempting MQTT connection...");
#endif  
  if (client.connect(AUTOCONNECT_APID, mqtt_user, mqtt_password)) 
  {
#if SERIAL_DEBUG      
    Serial.println("connected");
#endif    
    // Once connected, publish an announcement...
    client.publish("outTopic", "hello world"); //todo


  snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, AVAILABLE_OPENTHEM_TOPIC);
  if(SmOT.stsOT >= 0)   strcpy(msg,"online");
  else strcpy(msg,"offline");
//    strcpy(msg,"online");
    client.publish(topic, msg,true); 
    Serial.printf("%s %s true\n",topic, msg );

    snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, AVAILABLE_CH_TOPIC);
//if(CentralHeating_present)   strcpy(msg,"online");
//    else strcpy(msg,"offline");
    strcpy(msg,"online");
    client.publish(topic, msg); 

    snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, STATE_CH_TOPIC);
    if(SmOT.enable_CentralHeating)  strcpy(msg,"ON");
    else strcpy(msg,"OFF");
    client.publish(topic, msg); 
    Serial.printf("%s %s\n",topic, msg );


    // ... and resubscribe
    snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, ENABLE_CH_GET_TOPIC);
    client.subscribe(topic);
    Serial.printf("subscribe %s\n",topic );
    snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, TSet_GET_TOPIC);
    client.subscribe(topic);
    Serial.printf("subscribe %s\n",topic );

    rc = 1;
  } else {
#if SERIAL_DEBUG      
    Serial.print("failed, rc=");
    Serial.print(client.state());
#endif    
    rc = 0;
  }
  return rc;
}

void MQTTcallback(char* cbtopic, byte* payload, unsigned int length) 
{ int i, l;
  char *ptopic;
  Serial.print("Message arrived [");
  Serial.print(cbtopic);
  Serial.print("] ");

for (i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.printf("******************\n");

//check SmOT.MQTT_topic
  l = strlen(SmOT.MQTT_topic);
  if(strncmp(cbtopic,SmOT.MQTT_topic,l))
    return; //strange topic
  if(topic[l] != '/')  
    return; 
  ptopic = cbtopic + l + 1;

  if(!strcmp(ptopic, ENABLE_CH_GET_TOPIC))
  { if(!strcmp((char *)payload, "ON"))
      SmOT.enable_CentralHeating = 1;
    else if(!strcmp((char *)payload, "OFF"))
      SmOT.enable_CentralHeating = 0;
    
    snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, STATE_CH_TOPIC);
    if(SmOT.enable_CentralHeating)  strcpy(msg,"ON");
    else strcpy(msg,"OFF");
    client.publish(topic, msg); 
  }

}

HADevice device;
HAMqtt mqtt(espClient, device);

void mqtt_setup(void)
{
  bool rc;
  char str[40];

  if (WiFi.status() != WL_CONNECTED)  
        return;
  {  HADevice *device1;
     device1 = new HADevice(SmOT.MQTT_topic);
     device = *device1;
     delete device1;
  }

    device.setName(AUTOCONNECT_APID);
    sprintf(str,"%d.%d", SmOT.Vers,SmOT.SubVers);
    device.setSoftwareVersion(str);


  client.setServer( SmOT.MQTT_server, 1883);
  client.setCallback(MQTTcallback);

}

void mqtt_loop(void)
{ unsigned long now;
  if (WiFi.status() != WL_CONNECTED)  
        return;

  if (!client.connected()) {
    now = millis();
    if ((now - lastReconnectAttempt) > ( SmOT.MQTT_interval * 1000 *2))
    {
      lastReconnectAttempt = now;
      if(MQTTreconnect())
      { 
        lastReconnectAttempt = millis();
      }

    }
  } else {
    client.loop();
    now = millis();
    if (now - lastMsg > SmOT.MQTT_interval*1000)
    {   lastMsg = now;
        MQTT_pub_data();
    }
  }
}

int MQTT_pub_data(void)
{  static int sts = 0;
   int id = 0;
  bool rc;
M0:  
    switch(sts)
    {   case 0:
          if(SmOT.stsT1 >= 0)
          { snprintf (msg, MSG_BUFFER_SIZE, "%.3f", SmOT.t1);
            snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, T1_TEMP_TOPIC);
          
           rc =  client.publish(topic, msg);
           id = 1;
          } 
            break;

        case 1:
          if(SmOT.stsT2 >= 0)
          { snprintf (msg, MSG_BUFFER_SIZE, "%.3f", SmOT.t2);
            snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, T2_TEMP_TOPIC);
            rc = client.publish(topic, msg);
           id = 1;
          } 
            break;

        case 2:
          { snprintf (msg, MSG_BUFFER_SIZE, "%.3f", SmOT.BoilerT);
            snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, BoilerT_TEMP_TOPIC);
            rc = client.publish(topic, msg);
           id = 1;
          } 
            break;

        case 3:
          if(ot.OTid_used(OpenThermMessageID::Tret))
          { snprintf (msg, MSG_BUFFER_SIZE, "%.3f", SmOT.RetT);
            snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, RetT_TEMP_TOPIC);
            rc = client.publish(topic, msg);
           id = 1;
          } 
            break;

        case 4:
          if(ot.OTid_used(OpenThermMessageID::RelModLevel))
          { snprintf (msg, MSG_BUFFER_SIZE, "%.3f", SmOT.FlameModulation);
            snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, FlameModulation_TEMP_TOPIC);
            rc = client.publish(topic, msg);
           id = 1;
          } 
            break;

        case 5:
          if(ot.OTid_used(OpenThermMessageID::Toutside))
          { snprintf (msg, MSG_BUFFER_SIZE, "%.3f", SmOT.Toutside);
            snprintf (topic, MSG_BUFFER_SIZE, "%s/%s", SmOT.MQTT_topic, Toutside_TEMP_TOPIC);
            rc = client.publish(topic, msg);
           id = 1;
          } 
            break;

    }
#if SERIAL_DEBUG      
 if(id)
    Serial.printf("%s %s, rc = %d\n",  topic, msg, rc);
#endif //0    
  sts++;
  if(sts > 5)
    sts = 0;
  else if(id == 0)
    goto M0;
    
  return 0;
}

/*******************************************************************************/
#endif // MQTT_VERS


#endif  //MQTT_USE 