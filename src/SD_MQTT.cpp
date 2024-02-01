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
#if 1
/*******************************************************************************/
//HADevice *pHAdevice;
//HAMqtt *pMqtt;

HADevice device;
HAMqtt mqtt(espClient, device);

//HABinarySensor sensor("SmartThermSensor");
HABinarySensor sensorOT("OpenTherm");
HASensor  sensorBoilerT("BoilerT");
unsigned long lastReadAt = millis();
unsigned long lastAvailabilityToggleAt = millis();
bool lastInputState = false;

void mqtt_setup(void)
{  bool rc;
  if (WiFi.status() != WL_CONNECTED)  
        return;
  Serial.printf("todo %s\n",__FUNCTION__ );


  Serial.printf("0 in %s\n",__FUNCTION__ );

/* 
  pHAdevice = new HADevice(SmOT.MQTT_topic);
  if(pHAdevice == NULL)
      Serial.printf("pHAdevice = NULL in %s\n",__FUNCTION__ );
  Serial.printf("1 in %s\n",__FUNCTION__ );
//  pHAdevice->setName(AUTOCONNECT_APID); //должно быть static!!
  pHAdevice->setName("ST_TEST_MQTT"); //должно быть static!!
  { static char str[40];
    sprintf(str,"%d.%d", SmOT.Vers,SmOT.SubVers);
    pHAdevice->setSoftwareVersion(str); //должно быть static!!
  }
  */

   device.setUniqueIdStr(SmOT.MQTT_topic);

   device.setName("ST_TEST_MQTT"); //должно быть static!!
  { static char str[40];
    sprintf(str,"%d.%d", SmOT.Vers,SmOT.SubVers);
    device.setSoftwareVersion(str); //должно быть static!!
  }

  Serial.printf("2 in %s\n",__FUNCTION__ );
/*
  pMqtt = new HAMqtt(espClient, *pHAdevice);
  if(pMqtt == NULL)
      Serial.printf("pMqtt = NULL in %s\n",__FUNCTION__ );
*/      
  Serial.printf("3 in %s\n",__FUNCTION__ );
    lastReadAt = millis();
    lastAvailabilityToggleAt = millis();

 Serial.printf("4 in %s\n",__FUNCTION__ );
    sensorOT.setAvailability(false);
// Serial.printf("5 in %s\n",__FUNCTION__ );
    sensorOT.setCurrentState(false); // optional
    sensorOT.setName("OpenTherm connecton"); // optional
    sensorOT.setDeviceClass("connectivity"); // optional
     Serial.printf("6 in %s\n",__FUNCTION__ );
    sensorBoilerT.setAvailability(true);
    sensorBoilerT.setName("BoilerT"); // optional
    sensorBoilerT.setDeviceClass("temperature"); // optional

//    rc= pMqtt->begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    rc= mqtt.begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    {
   Serial.printf("mqtt.begin ok %s %s %s\n", SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);

    }
  Serial.printf("endof %s\n",__FUNCTION__ );

}


int statemqtt = -1;
int state_mqtt = -10000;
void mqtt_loop(void)
{ int sts;
  char str[40];
//      pMqtt->loop();
      mqtt.loop();
//      if(pMqtt->isConnected())
      if(mqtt.isConnected())
      {   if(statemqtt != 1)
              Serial.printf("MQTT connected\n");
          statemqtt = 1;
      } else {
          if(statemqtt != 0)
              Serial.printf("MQTT DiSconnected\n");
          statemqtt = 0;
      }

//    sts = pMqtt->_mqtt->state(); 
    sts = mqtt._mqtt->state(); 
      if(sts !=state_mqtt )
      {   Serial.printf("MQTT state=%d\n", sts);
          state_mqtt = sts;
      }
//Serial.printf("todo %s\n",__FUNCTION__ );
    if ((millis() - lastAvailabilityToggleAt) > 15000) {

        if(SmOT.stsOT == -1)
        { sensorOT.setAvailability(false);
        } else {
          sensorOT.setAvailability(true);
          if(SmOT.stsOT == 2)
          { sensorOT.setCurrentState(false);
          } else {
            sensorOT.setCurrentState(true); 
          }
          sprintf(str,"%.3f", SmOT.BoilerT);
          sensorBoilerT.setValue(str);

        }

        lastAvailabilityToggleAt = millis();
    }


}
#else //0

HADevice device;
HAMqtt mqtt(espClient, device);

HAButton buttonA("myButtonA");
HAButton buttonB("myButtonB");
HASwitch switch1("mySwitch1");
HASwitch switch2("mySwitch2");

// "myInput" is unique ID of the sensor. You should define you own ID.
HABinarySensor sensor("myInput");
unsigned long lastReadAt = millis();
unsigned long lastAvailabilityToggleAt = millis();
bool lastInputState = false;

void onButtonCommand(HAButton* sender)
{
    if (sender == &buttonA) {
        // button A was clicked, do your logic here
   Serial.printf("onButtonCommand A\n");
    } else if (sender == &buttonB) {
   Serial.printf("onButtonCommand B\n");
        // button B was clicked, do your logic here
    }
}
void onSwitchCommand(bool state, HASwitch* sender)
{
    if (sender == &switch1) {
        // the switch1 has been toggled
   Serial.printf("onSwitchCommand 1 %d\n", state);
        // state == true means ON state
    } else if (sender == &switch2) {
        // the switch2 has been toggled
        // state == true means ON state
   Serial.printf("onSwitchCommand 2 %d\n", state);
    }

    sender->setState(state); // report state back to the Home Assistant
}


void mqtt_setup() {
  bool rc;
  if (WiFi.status() != WL_CONNECTED)  
        return;
    // you don't need to verify return status
//    Ethernet.begin(mac);
   Serial.printf("mqtt_setup\n");

{
  HADevice *device1;
  device1 = new HADevice(SmOT.MQTT_topic);
  device = *device1;
//  delete device1;
}
    // optional device's details
    device.setName("ST_TEST_MQTT"); //должно быть static!!
//    device.setName("Her");
//    device.setName("Arduino");
{
 static char str[40];
    sprintf(str,"%d.%d", SmOT.Vers,SmOT.SubVers);
   device.setSoftwareVersion(str); //должно быть static!!
//     device.setSoftwareVersion("1.1");
   Serial.printf("SoftwareVersion %s\n", str);
//    device.setSoftwareVersion("1.0.0");
//    device.setUniqueId((const byte*)SmOT.MQTT_topic, strlen(SmOT.MQTT_topic));

}
    // optional properties
    buttonA.setIcon("mdi:fire");
    buttonA.setName("Click me A");
    buttonB.setIcon("mdi:home");
    buttonB.setName("Click me B");

    // press callbacks
    buttonA.onCommand(onButtonCommand);
    buttonB.onCommand(onButtonCommand);

    // turn on "availability" feature
    // this method also sets initial availability so you can use "true" or "false"
    sensor.setAvailability(true);
    lastReadAt = millis();
    lastAvailabilityToggleAt = millis();
    sensor.setCurrentState(lastInputState); // optional
    sensor.setName("Door sensor"); // optional
    sensor.setDeviceClass("door"); // optional

    switch1.setName("Pretty label 1");
    switch1.setIcon("mdi:lightbulb");
    switch1.onCommand(onSwitchCommand);
    switch1.setState(false); 

    switch2.setName("Pretty label 2");
    switch2.setIcon("mdi:lightbulb");
    switch2.onCommand(onSwitchCommand);    
    switch2.setState(true); 


    rc= mqtt.begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    {
   Serial.printf("mqtt.begin ok %s %s %s\n", SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);

    }
}

void mqtt_loop() {
//    Ethernet.maintain();
    mqtt.loop();

    if ((millis() - lastAvailabilityToggleAt) > 15000) {
         Serial.printf("sensor.isOnline() %d\n", sensor.isOnline());

        sensor.setAvailability(!sensor.isOnline());
        lastAvailabilityToggleAt = millis();
    }

    if ((millis() - lastReadAt) > 30000) { // read in 30ms interval
        // library produces MQTT message if a new state is different than the previous one
        static int sts = 0;
        sensor.setState(sts);
        lastInputState = sensor.getCurrentState();
         Serial.printf("lastInputState %d\n", lastInputState);
        sts = (sts+1) &0x01;
        if(sts)
            switch2.setState(true); 
        else
            switch2.setState(false); 

        lastReadAt = millis();
    }


}

#endif // 0

int MQTT_pub_data(void)
{
Serial.printf("todo %s\n",__FUNCTION__ );
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