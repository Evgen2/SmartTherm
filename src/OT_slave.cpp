/* OT_slave.cpp */


#include <Arduino.h>

#include "SmartDevice.hpp"
#include "Smart_Config.h"

#include "OpenTherm.h"
#include "SD_OpenTherm.hpp"

#if ST_VERS == 2

/************************************/
extern OpenTherm ot_slave;
extern OpenTherm ot;
extern SD_Termo SmOT;

int OTslaveDebugInfo[12] ={0,0,0,0,0, 0,0,0,0,0, 0,0};

//#define OTSLAVE_DEBUG

#if OTSLAVE_DEBUG
void LogOT(int code, byte id, int messagetype, unsigned int u88);
#else 
#define  LogOT(a,b,c,d)
#endif
/* 
0 get request SUCCESS
1 get request ok,  ot_SlaveRequest = request;
2 request from panel go to reqest to boiler
3 get response from slave (boiler), can send response to master (panel)
4  sendResponse to master (panel)
-1 request TIMEOUT
-2 request NONE
-3 request INVALID
-4 request parity ERR
*/
volatile int ot_SlaveSts = 0;
volatile unsigned long ot_SlaveResponse = 0; 
volatile unsigned long ot_SlaveRequest = 0; 
volatile uint16_t ot_SlaveMasterStatus = 0;
unsigned long ot_SlaveRequest_ms = 0;
int OT_slaveloop(void);
int setup_ot_slave(void);
void sendResponse_ot_slave(void);

void OTlog(unsigned int reqresp, int sts);

int nslaveint = 0;

void IRAM_ATTR handleInterruptslave() {
    ot_slave.handleInterrupt();
    nslaveint++;
}

void processRequest(unsigned long request, OpenThermResponseStatus status) {

    unsigned long response = 0;
    int parity;
static int timeOutcounter = 0;

#if  OT_SLAVE_DEBUG
    Serial_db.printf("Slave processRequest: request %x status %x\n", request, status); 
#endif
#if  OT2_SLAVE_DEBUG
    Serial_db.printf("Slave processRequest: request %x status %x\n", request, status); 
#endif

    if (status == OpenThermResponseStatus::SUCCESS) {
        ot_SlaveSts = 0;
//        SmOT.response = response; 
        OTslaveDebugInfo[0]++;
    } else if (status == OpenThermResponseStatus::NONE) {
      // SmOT.stsOT = -1;  // ??
      ot_SlaveSts = -2;
#if OT_DEBUG
      LogOT(-3, 0,  0,  0);
#endif         
        OTslaveDebugInfo[2]++;
    } else if (status == OpenThermResponseStatus::INVALID) {
      ot_SlaveSts = -3;
       //SmOT.stsOT = 1;
#if OT_DEBUG
      LogOT(-2, 0,  0,  0);
#endif         
        OTslaveDebugInfo[3]++;
    } else if (status == OpenThermResponseStatus::TIMEOUT) {
      if(SmOT.ot_slave_stsOT != -1)
	    { if(timeOutcounter > 10)
		    { if(SmOT.ot_slave_stsOT != 2)
               SmOT.MQTT_need_report = 1;
          SmOT.ot_slave_stsOT = 2;
		    } else {
			    timeOutcounter++;
		    }	
      }

//       if( OTsts != -1)
#if OT_DEBUG
      LogOT(-1, 0,  0,  0);
#endif         
             ot_SlaveSts = -1;
        OTslaveDebugInfo[4]++;
        return;
    }

#if OTSLAVE_DEBUG
  { unsigned int  parity, messagetype;
    byte iid;
    iid = (request >> 16 & 0xFF);
    parity = ot_slave.parity(request);
    messagetype = ot_slave.getMessageType(request);
    if(parity)
      LogOT(0,  iid,  messagetype,  u88);
    else 
      LogOT(1,  iid,  messagetype,  u88);
  } 
#endif  

    parity = ot_slave.parity(request);
    if(parity)
    { OTslaveDebugInfo[1]++;
      ot_SlaveSts = -4;

#if SERIAL_DEBUG 
        Serial.println(F("Parity error"));
#endif        
      return;
    }

    OpenThermMessageID id = ot_slave.getDataID(request);
//    uint16_t data = ot_slave.getUInt(request);
    float t = ot_slave.getFloat(request);

//    uint16_t data = ot_slave.getUInt(request);
//    Serial_db.printf("Slave processRequest: id %d data %x\n", id, data); 

        if (!ot_slave.isValidRequest(request))
        {
    Serial_db.printf("Err: invalidRequest %lx\n", request); 
        //build UNKNOWN-DATAID response
        response = ot_slave.buildResponse(OpenThermMessageType::UNKNOWN_DATA_ID, ot_slave.getDataID(request), 0);   
    //send response
            goto SR;
//    delay(20); //20..400ms, usually 100ms
//    ot.sendResponse(response);
 //         return;
        }

   switch(id)
   { 
/*  HB: Master status 
    bit: description [ clear/0, set/1]
0: CH enable [ CH is disabled, CH is enabled]
1: DHW enable [ DHW is disabled, DHW is enabled]
*/
      case OpenThermMessageID::Status:  //0
      {
        uint16_t u88;
        u88 = (request & 0xffff);
        ot_SlaveMasterStatus =  u88>>8;

      }
            break;
      case OpenThermMessageID::TSet:  // 1 W
        if(SmOT.OT_slave_mode == 1)
          SmOT.Tset = t;
        break;
      case OpenThermMessageID::TdhwSet: //56 W
        if(SmOT.OT_slave_mode == 1)
          SmOT.TdhwSet = t;
        break;
      default:
      break;
   }     
/*************************************************/
    SmOT.ot_slave_t_lastwork  = time(nullptr);
    ot_SlaveRequest_ms = millis();
    ot_SlaveRequest = request;
    ot_SlaveSts = 1;
    if(SmOT.ot_slave_stsOT != 0)
      SmOT.need_report_MQTT_panel  = 1;
    SmOT.ot_slave_stsOT = timeOutcounter = 0;
     return;

SR:
//    Serial.println("B" + String(response, HEX)); //slave/boiler response
#if OTSLAVE_DEBUG

  { unsigned int u88;
    byte iid;
    u88 = (response & 0xffff);
    iid = (response >> 16 & 0xFF);
    parity = otslave.parity(response);
    messagetype = otslave.getMessageType(response);
    LogOT(5,  iid,  messagetype,  u88);
  } 
#endif         
//    Serial_db.printf("Slave processRequest: id %x data %x\n", id, data); 

    if(SmOT.ot_slave_stsOT != 0)
      SmOT.need_report_MQTT_panel  = 1;
    SmOT.ot_slave_stsOT = 0;
    //send response
    ot_SlaveResponse = response;
#if OT_DEBUGLOG
    OTlog(response, 3);
#endif    
    ot_SlaveSts = 3;
// мы тут в прерывании.
// напрямую из прерывания посылать ответ - плохо    
//    delay(21); //20..400ms, usually 100ms
//    ot_slave.sendResponse(response);
}

int setup_ot_slave(void)
{
  if(SmOT.ot_slave_stsOT == -2)
  {
//    Serial_db.printf("setup_slave\n");

    Serial_db.printf("setup handleInterruptslave\n");

 ot_slave.begin(handleInterruptslave, processRequest);
    SmOT.ot_slave_stsOT = -1;
  }
    return 0;
}

void sendResponse_ot_slave(void)
{ 
    nslaveint = 0;
//    if(ot_slave.getMessageType(ot_SlaveResponse) == DATA_INVALID)
//       Serial_db.printf("DATA_INVALID SlaveResponse 2\n");

ot_slave.sendResponse(ot_SlaveResponse);
    ot_SlaveSts = 4;
}

//ot_SlaveResponse
//ot_SlaveRequest

void makeRO_Response_ot_slave(void)
{   uint16_t u88;
    OpenThermMessageID id = ot_slave.getDataID(ot_SlaveRequest);

    if(!ot.OTid_used(id))
    {   ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::UNKNOWN_DATA_ID, id, 0);   
        return;
    }

    switch(id)
    { case OpenThermMessageID::Status:  //0
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_SlaveMasterStatus<<8|(SmOT.BoilerStatus&0xff));   
//  Serial_db.printf("Status: resp %x %x\n", ot_SlaveMasterStatus<<8|(SmOT.BoilerStatus&0xff), SmOT.BoilerStatus); 
        break;
      case OpenThermMessageID::TSet:  // 1 W
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.Tset));   
        break;

    case OpenThermMessageID::ASFflags: //5
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, SmOT.Fault);   
        break;

    case OpenThermMessageID::RelModLevel: //17 Relative Modulation Level 
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.FlameModulation));   
        break;

    case OpenThermMessageID::CHPressure: //18 Water pressure in CH circuit
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.Pressure));   
        break;

    case OpenThermMessageID::DHWFlowRate: //19 Water flow rate in DHW circuit. (litres / minute)
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.DHWFlowRate));   
        break;

    case OpenThermMessageID::Tboiler:  //25
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.BoilerT));   
        break;

    case OpenThermMessageID::Tdhw: //26
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.dhw_t));   
            break;

    case OpenThermMessageID::Toutside: //27
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.Toutside));   
        break;

    case OpenThermMessageID::Tret: //28
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.RetT));   
        break;

    case OpenThermMessageID::MaxTSetUBMaxTSetLB: //49 s8/s8 Max CH water Setpoint upper & lower bounds for adjustment(°C)
      u88 = ot_slave.temperatureToData(SmOT.MaxTSetUB)<<8 | ot_slave.temperatureToData(SmOT.MaxTSetLB);
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, u88);   
        break;

      case OpenThermMessageID::TdhwSet: //56 W
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, id, ot_slave.temperatureToData(SmOT.TdhwSet));   
        break;

      default:
        ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::UNKNOWN_DATA_ID, id, 0);   
    }
}


int OT_slaveloop(void)
{
  ot_slave.process();
#if  OT_SLAVE_DEBUG
  static unsigned long int t0=0;
  unsigned long int t;
  t = millis();
  if(t-t0>500)
  {
    ot_SlaveResponse = ot_slave.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::Status, 0xffff);
    Serial_db.printf("Slave test send Response:  %x\n", ot_SlaveResponse); 
    sendResponse_ot_slave();

    t0 = t;
  }
#else

if(SmOT.OT_slave_mode == 1)
{
  if(ot_SlaveSts == 3 && ot_slave.isReady())
  {
    if(millis() - ot_SlaveRequest_ms > 21) //send response after 21 ms
        sendResponse_ot_slave();
  }
} else {
  if(ot_SlaveSts == 1 && ot_slave.isReady())
  {
    makeRO_Response_ot_slave();
    if(millis() - ot_SlaveRequest_ms > 21) //send response after 21 ms
        sendResponse_ot_slave();
  }

}
  
#endif

  {  time_t now = time(nullptr);
      double dt;
      dt = difftime(now,SmOT.ot_slave_t_lastwork);
      if(dt > 10. && SmOT.ot_slave_stsOT == 0)
      { SmOT.ot_slave_stsOT = 2;
        SmOT.need_report_MQTT_panel  = 1;
      }
  }

  return 0;
}

#endif //ST_VERS == 2

