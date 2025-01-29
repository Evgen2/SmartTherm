/* OT_slave.cpp */


#include <Arduino.h>

#include "SmartDevice.hpp"
#include "Smart_Config.h"

#include "OpenTherm.h"
#include "SD_OpenTherm.hpp"

#if ST_VERS == 2

/************************************/
extern OpenTherm ot_slave;
extern SD_Termo SmOT;


int OTslave_sts = -1;
int OTslaveDebugInfo[10];

#if OTSLAVE_DEBUG
void LogOT(int code, byte id, int messagetype, unsigned int u88);
#else 
#define  LogOT
#endif

volatile int ot_SlaveSts = 0;
volatile unsigned long ot_SlaveResponse = 0; 
volatile unsigned long ot_SlaveRequest = 0; 
int OT_slaveloop(void);
int setup_ot_slave(void);
void sendResponse_ot_slave(void);


void IRAM_ATTR handleInterruptslave() {
    ot_slave.handleInterrupt();
}

void processRequest(unsigned long request, OpenThermResponseStatus status) {

    unsigned long response = 0;
    int parity, messagetype;
static int timeOutcounter = 0;

//    Serial.printf("Slave processRequest: request %x status %x\n", request, status); 

    if (status == OpenThermResponseStatus::SUCCESS) {
        ot_SlaveSts = 0;
//        SmOT.response = response; 
        OTslaveDebugInfo[0]++;
    } else if (status == OpenThermResponseStatus::NONE) {
      // SmOT.stsOT = -1;  // ??
#if OT_DEBUG
      LogOT(-3, 0,  0,  0);
#endif         
        OTslaveDebugInfo[2]++;
    } else if (status == OpenThermResponseStatus::INVALID) {
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

#if OT_DEBUG
  { unsigned int u88;
    byte iid;
    u88 = (request & 0xffff);
    iid = (request >> 16 & 0xFF);
    parity = otslave.parity(request);
    messagetype = otslave.getMessageType(request);
    if(parity)
      LogOT(0,  iid,  messagetype,  u88);
    else 
      LogOT(1,  iid,  messagetype,  u88);
  } 
#endif  

    parity = ot_slave.parity(request);
    if(parity)
    { OTslaveDebugInfo[1]++;
#if SERIAL_DEBUG 
        Serial.println(F("Parity error"));
#endif        
      return;
    }

    OpenThermMessageID id = ot_slave.getDataID(request);
    uint16_t data = ot_slave.getUInt(request);
    messagetype = ot_slave.getMessageType(request);

//    Serial.printf("Slave processRequest: id %x data %x\n", id, data); 

        if (!ot_slave.isValidRequest(request))
        {
    Serial.printf("Err: invalidRequest %x\n", request); 
        //build UNKNOWN-DATAID response
        response = ot_slave.buildResponse(OpenThermMessageType::UNKNOWN_DATA_ID, ot_slave.getDataID(request), 0);   
    //send response
            goto SR;
//    delay(20); //20..400ms, usually 100ms
//    ot.sendResponse(response);
 //         return;
        }


//    float f = ot.getFloat(request);

//      Serial.printf("Message id %d\n", id); 
/*************************************************/
#if 0
if(0) //todo
     { extern OpenThermID OT_ids[N_OT_NIDS];
          int i, is=0;
          for(i=0; i< N_OT_NIDS; i++)
          { if(OT_ids[i].id == id)
            {   is = 1;
                break;
            }
          }

          if(is == 0)
          {   Serial.printf("response: UNKNOWN-DATAID %d\n", id); 
              //delay(2000);
          } else {
              if(OT_ids[i].used == 0)
              {    Serial.printf("id %d not used in emulator\n", id); 
                   is = 0;
              }
          }
        if(is == 0)    //build UNKNOWN-DATAID response
        {   response = ot_slave.buildResponse(OpenThermMessageType::UNKNOWN_DATA_ID, id, 0);   
            //send response
            goto SR;
//            delay(20); //20..400ms, usually 100ms
//            ot.sendResponse(response);
//            return;
        }
      }
#endif //0      
/*************************************************/
    SmOT.ot_slave_t_lastwork  = time(nullptr);
    ot_SlaveRequest = request;
    ot_SlaveSts = 1;
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
//    Serial.printf("Slave processRequest: id %x data %x\n", id, data); 

    SmOT.ot_slave_stsOT = 0;
    //send response
    delay(21); //20..400ms, usually 100ms
    ot_slave.sendResponse(response);
}

int setup_ot_slave(void)
{
    Serial.printf("setup_slave\n");

  ot_slave.begin(handleInterruptslave, processRequest);

    return 0;
}

void sendResponse_ot_slave(void)
{ 
  int id;
    id = (ot_SlaveResponse >> 16 & 0xFF);

    ot_slave.sendResponse(ot_SlaveResponse);
    ot_SlaveSts = 0;
}


int OT_slaveloop(void)
{
  ot_slave.process();
  if(ot_SlaveSts == 3 && ot_slave.isReady())
      sendResponse_ot_slave();

  {  time_t now = time(nullptr);
      double dt;
      dt = difftime(now,SmOT.ot_slave_t_lastwork);
      if(dt > 10. && SmOT.ot_slave_stsOT == 0)
        SmOT.ot_slave_stsOT = 2;
  }

  return 0;
}

#endif //ST_VERS == 2

