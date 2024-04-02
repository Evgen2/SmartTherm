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
#if PID_USE
HAMqtt mqtt(espClient, device,22);
#else
HAMqtt mqtt(espClient, device,12);
#endif

const char * temperature_str = "temperature";

HABinarySensor sensorOT(NULL);
HABinarySensor sensorFlame(NULL);
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

#endif

// By default HAHVAC supports only reporting of the temperature.
// You can enable feature you need using the second argument of the constructor.
// Please check the documentation of the HAHVAC class.
HAHVAC hvac(
  NULL,
  HAHVAC::TargetTemperatureFeature | HAHVAC::PowerFeature | HAHVAC::ModesFeature |HAHVAC::ActionFeature,
  HANumber::PrecisionP2
);

unsigned long lastReadAt = millis();
unsigned long lastAvailabilityToggleAt = millis();
bool lastInputState = false;


void onTargetTemperatureCommand(HANumeric temperature, HAHVAC* sender) {
    float temperatureFloat = temperature.toFloat();
#if SERIAL_DEBUG      
    Serial.print("Target temperature: ");
    Serial.println(temperatureFloat);
#endif    
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

#if PID_USE

void onNumberCommand(HANumeric number, HANumber* sender)
{   float t = number.toFloat();
//    if (sender == &numPID_v) {
//      Serial.printf("NumberCommand numPID_v: %f\n",t);
//
//    } else 
    
    if (sender == &numT_outdoor) {
#if SERIAL_DEBUG      
      Serial.printf("NumberCommand numT_outdoor: %f (%d)\n", t, millis()/1000);
#endif      
      SmOT.OnChangeT(t,4);
        
    } else if (sender == &numT_indoor) {
#if SERIAL_DEBUG      
      Serial.printf("NumberCommand numT_indoor: %f (%d)\n", t, millis()/1000);
#endif      
      SmOT.OnChangeT(t,3);
    }

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

    sender->setState(number); // report the selected option back to the HA panel
}
#endif

/************************************************************/
void mqtt_setup(void)
{  bool rc;
  if (WiFi.status() != WL_CONNECTED)  
        return;

  if( mqtt.getDevicesTypesNb_toreg() > mqtt.getDevicesTypesNb())
  {
      Serial.printf("Error! Nb = %d, need be %d\n", mqtt.getDevicesTypesNb(),  mqtt.getDevicesTypesNb_toreg() );
  
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

    sensorBoilerT.setNameUniqueIdStr(SmOT.MQTT_topic,"Температура теплоносителя", "BoilerT");
    sensorBoilerT.setAvailability(false);
    sensorBoilerT.setDeviceClass(temperature_str);
    sensorBoilerT.setUnitOfMeasurement("°C");
    
    sensorBoilerRetT.setNameUniqueIdStr(SmOT.MQTT_topic,"Температура обратки", "RetT");
    sensorBoilerRetT.setAvailability(false);
    sensorBoilerRetT.setDeviceClass(temperature_str); 
    sensorBoilerRetT.setUnitOfMeasurement("°C");

    sensorPressure.setNameUniqueIdStr(SmOT.MQTT_topic,"Давление", "Pressure");
    sensorPressure.setAvailability(false);
    sensorPressure.setDeviceClass("pressure"); 

    sensorFreeRam.setAvailability(true);
    sensorPressure.setNameUniqueIdStr(SmOT.MQTT_topic,"Free RAM", "FreeRAM");
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
    if(SmOT.enable_CentralHeating)
          hvac.setMode(HAHVAC::HeatMode);
    else
          hvac.setMode(HAHVAC::OffMode);

    hvac.setAvailability(false);

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
    numT_outdoor.setState(23.f, true);
    numT_outdoor.setCurrentState(22.f);
    numT_outdoor.setStep(0.1);
    numT_outdoor.setMin(-50.);
    numT_outdoor.setMax( 100.);
    numT_outdoor.onCommand(onNumberCommand);

    numT_indoor.setAvailability(true);
    numT_indoor.setNameUniqueIdStr(SmOT.MQTT_topic,"T indoor", "Tindoor");
    numT_indoor.setMode(HANumber::ModeBox);
    numT_indoor.setState(33.f, true);
    numT_indoor.setStep(0.1);
    numT_indoor.setMin(-50.);
    numT_indoor.setMax( 100.);
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

#endif


    SmOT.stsMQTT = 1;
    mqtt._mqtt->setSocketTimeout(1); //not work ???

    rc= mqtt.begin(SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
    if(rc == true)
    {
   Serial.printf("mqtt.begin ok %s %s %s\n", SmOT.MQTT_server,SmOT.MQTT_user, SmOT.MQTT_pwd);
      SmOT.stsMQTT = 2;

    } else {
   Serial.printf("mqtt.begin false\n");

    }
}

int statemqtt = -1;
int state_mqtt = -10000;
int attempt_mqtt = 0;

void mqtt_loop(void)
{ int sts;
  char str[80];
static int st_old = -2;  
static unsigned int t0=0;
unsigned long t1;
int dt;
//      pMqtt->loop();
      if(SmOT.stsMQTT == 0)
          return;
/*******************/    
   t1 = millis();   
 if(state_mqtt != 0)
    Serial.printf("**** state_mqtt=%d attempt_mqtt=%d dt=%d\n",state_mqtt, attempt_mqtt, t1-t0);
  if(state_mqtt == -2) 
  {  dt = t1 - t0;
 Serial.printf("**** state_mqtt=%d attempt_mqtt=%d dt=%d\n",state_mqtt, attempt_mqtt, dt);
    if(dt < 5000 *(attempt_mqtt+3))
        return;
 Serial.printf("*********** state_mqtt= -2 attempt_mqtt=%d dt=%d\n", attempt_mqtt, dt);
    if(attempt_mqtt < 100)
      attempt_mqtt++;
  }  else {
    attempt_mqtt = 0;
  }   
  t0 = t1;
/*******************/          

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
        return; // return from   mqtt_loop() if not connected
    }

    sts = mqtt._mqtt->state(); 
    if(sts !=state_mqtt )
    {   Serial.printf("MQTT state=%d\n", sts);
        state_mqtt = sts;
    }

    if ((millis() - lastAvailabilityToggleAt) > SmOT.MQTT_interval*1000)
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
            sensorModulation.setAvailability(false);
            sensorBoilerRetT.setAvailability(false);
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

#if PID_USE
        if(SmOT.need_report)
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

//            sprintf(str,"isset %d nx %d xmean %.3f x %.3f", SmOT.t_mean[4].isset, SmOT.t_mean[4].nx, SmOT.t_mean[4].xmean,  SmOT.t_mean[4].x);
//            textPIDinfo.setValue(str);

            SmOT.need_report = 0;
        }

/*
{  static float v = 0.;
    numPID_v.setState(v, true);
    v += 0.1;
    textTargetTemp.setValue("22");
    textPIDinfo.setValue("BlaBlaBla");
}
*/
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