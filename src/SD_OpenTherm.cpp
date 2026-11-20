/* SD_OpenTherm.cpp */
#include <time.h>
#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
typedef ESP8266WebServer WEBServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
#if defined(ARDUINO_ARCH_ESP32) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
 #include <esp_rtc_time.h>
#else
 #include "esp32/rom/rtc.h"
#endif
typedef WebServer WEBServer;
#endif
#include <AutoConnect.h>
#include <AutoConnectFS.h>
extern AutoConnectFS::FS& FlashFS;

#include "Smart_Config.h"
#include "OpenTherm.h"
#include "SD_OpenTherm.hpp"
#include "Smart_commands.h"


extern int TcpUdp_Lsend;
extern int TcpServer_Lsend;

extern PACKED unsigned char *Udp_MsgOut;

extern IPAddress Udp_remoteIP;  
extern int Udp_RemotePort;
extern U8 *esp_get_buf (U16 size);
extern U8 *server_get_buf (U16 size);

extern int TCPserver_close_on_send;

//struct Msg1 msg;
int indcmd = 0;

/*******************************/

#if OT_DEBUGLOG
 #if defined(ARDUINO_ARCH_ESP8266)
  #define OT_DEBUGLOG_SIZE 8*4   
 #elif defined(ARDUINO_ARCH_ESP32)
  #define OT_DEBUGLOG_SIZE 8*256   
 #endif
 static char OT_DebugLog[OT_DEBUGLOG_SIZE];
 extern Serial_Debug Serial_db;

#endif //OT_DEBUGLOG

void SD_Termo::init(int src)
{
#if OT_DEBUGLOG
    if(src == 1)
    {   OTlogBuf.Init(OT_DebugLog,OT_DEBUGLOG_SIZE,8);
        pSerial_db = &Serial_db;
    }
#endif  
  Bstat.t_I_last =time(nullptr);
  Bstat.sec_h = Bstat.sec_d = 0;
#if PID_USE
  if(usePID && !enable_CentralHeating)
  {   usePID = 0;
  }
  if(usePID && src == 1)
    Tset = MinCHtemp;
  
  if(src != 3)
   _U0start = mypid.u0;
  else
  { mypid.NextTact();
    mypid.dSt.n = mypid.dSt.ind = 0;
  }
  if(mypid.Ku != 1.f)
  {
    Serial_db.printf("Warning: mypid.Ku %gn", mypid.Ku);
    mypid.Ku = 1.f;
  }
  
// Serial_db.printf("src %d _U0start ->mypid.u0\n",  src, _U0start);

#endif   
//set planner
    need_set_T(2);
    need_set_dhwT(2);
    if(!Kotitonttu_fix_flag)
            MaxTSet  = umax;
    else
            MaxTSet = Tset;
    need_set_MaxTSet(2);
    if(Use_remoteTCPserver)
        TCPserver_sts = 2;  /* статус сервера */
    if(TCPserver_port == 0) 
        TCPserver_port = 8876;
}

 extern RTC_NOINIT_ATTR unsigned short int bootCount, bootReason, bootSts, bootSts1, bootSts2;

void SD_Termo::loop(void)
{   int dt;
    extern int WiFists;

    if(need_write_f)
    {   
        if(need_write_f & 0x10) //10 sec delay to write config
        {   if((millis() - t_need_write_config) > 10000) 
            {   need_write_f &= ~0x10;
            }
        } else {
            if(need_write_f & 0x01)
            {   Write_ot_fs();
                need_write_f &= ~0x01;
            }
            if(need_write_f & 0x02)
            {   Write_mqtt_fs();
                need_write_f &= ~0x02;
            }
            if(need_write_f & 0x04)
            {   Write_rc_fs();
                need_write_f &= ~0x04;
            }
            if(need_write_f & 0x08)
            {   Write_sensors_fs();
                need_write_f &= ~0x08;
            }
            delay(1);
        }
     
    } else  if(WiFists  == WL_CONNECTED) {
    
        if(UDPserver_sts)
        {    dt = millis() - UDPserver_t;
            if(dt < UDPserver_repot_period)
            {   return;
            }
            if(TcpUdp_Lsend > 0)
                return;
    //  Serial_db.printf("SD_Termo::loop %li\n",  millis());
            OpenThermInfo();
            UDPserver_t = millis();
        }

    /*   backup 
        if(TCPserver_sts)
        {    dt = millis() - TCPserver_t;
            if(dt < TCPserver_report_period)
            {   return;
            }
            if(TcpUdp_Lsend > 0)
                return;
    
            OpenThermInfo();
            TCPserver_t = millis();
        }
    */   
  
        if(TCPserver_sts)
        {   static unsigned long ts0=0;
	        static int nattemps=0;
/*            
      { static int TCPserver_sts2_old = 0;
        if(TCPserver_sts2 != TCPserver_sts2_old)
        {
            Serial.printf("++++++ SD_Termo::loop  TCPserver_sts2 %d %ld %d\n", 
            TCPserver_sts2, ts0, TCPserver_report_period);
            TCPserver_sts2_old = TCPserver_sts2; 
        }
        }
*/      
            bootSts1 = 500 + TCPserver_sts2;
             switch(TCPserver_sts2)
            {   case 0:
                    if(millis() - ts0 > TCPserver_report_period) 
                    {   TCPserver_sts2 = 1; 
//      Serial_db.printf("++++++ SD_Termo::loop  start%li\n",  millis());
                    }
                        break;
                case 1:
                    TCPserver_rc = 0;
                    Send_to_server_HandShake();
//    Serial_db.printf("++++++ Send_to_server_HandShake%li\n",  millis());
                    TCPserver_sts2++;
                    ts0 = millis();
//    Serial_db.printf("SD_Termo::loop Send_to_server_HandShake\n");
                        break;

                case 2:
    /* ждать ответа на Send_to_server_HandShake */  
    /* если есть ответ - переход на след sts, иначе после таймаута переход на паузу  
    */          
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0;
                        nattemps++; 
                        if(nattemps > 10)
                                start_sts = 0; // связь отвалилась, восстанавливать параметры PID не будем
                        ts0 = millis();
                    } else if(TCPserver_rc == MCMD_HAND_SHAKE) {
                        TCPserver_rc = 0;
                        nattemps = 0; 

                        Send_to_server_IdentifySelf();
//      Serial_db.printf("++++++  Send_to_server_IdentifySelf %li\n",  millis());
                        TCPserver_sts2++;
                        ts0 = millis();
//    Serial_db.printf("SD_Termo::loop Send_to_server_IdentifySelf\n");
                    }

//  Serial_db.printf("TCPserver_sts2 %d dt %d\n", TCPserver_sts2,  millis() - ts0);
                        break;

                case 3: //wait answer MCMD_INTRODUCESELF from server 
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0; 
                        ts0 = millis();
                    } else if(TCPserver_rc == MCMD_INTRODUCESELF) {
                        TCPserver_sts2 = 5; //4; 
//      Serial_db.printf("++++++  Send_to_server_IdentifySelf answer %li\n",  millis());
                        ts0 = millis();
                    }

                        break;

                case 4: //wait SCMD_GET_STS from server ??????
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0; 
//                        Serial_db.printf("SCMD_GET_STS *****\n");
                        ts0 = millis();
                    } else if(TCPserver_rc == SCMD_GET_STS) {
                        TCPserver_sts2 = 5; 
                        ts0 = millis();
                    }
                    break;

                case 5:
                    if(millis() - ts0 > TCPserver_report_period) //todo
                    {   TCPserver_sts2 = 6; 
                        //send  CCMD_SEND_STS
//      Serial_db.printf("SD_Termo::loop  Send_to_server_Sts %li\n",  millis());
                        Send_to_server_Sts();
                        ts0 = millis();
                    } else {
                        static int old_dt = 0;
                        int dt;
                        dt = (millis() - ts0)/ 1000;
                        if(dt != old_dt)
                        { old_dt = dt;
                         //  Serial_db.printf("dt %d (%d)\n", dt, TCPserver_report_period/1000 );
                        }
                    }
                    break;

                    case 6: //wait answer to CCMD_SEND_STS from server
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0; 
//                        Serial_db.printf("CCMD_SEND_STS ****\n");
                        ts0 = millis();
#if OT_DEBUGLOG
                    } else if(TCPserver_rc == CCMD_SEND_STS_S) {
                        TCPserver_sts2 = 5; 
                        ts0 = millis();
                    } else if(TCPserver_rc == CCMD_SEND_OTLOG_S) {
                        TCPserver_sts2 = 7; 
                        ts0 = millis();
                    } else if(TCPserver_rc == CCMD_SEND_LOG_S) {
                        TCPserver_sts2 = 9; 
                        ts0 = millis();
#endif 
                    } else if(TCPserver_rc == SCMD_GET_HAND_SHAKE) {
                     //   Serial_db.printf(">>>>>>>>>>>>>>>>>>>>  Сервер хочет HAND_SHAKE\n" );
                        TCPserver_sts2 = 1; //HandShake
                    }
                    break;
#if OT_DEBUGLOG
                                                                    
                    case 7: //send CCMD_SEND_OTLOG_S
                        Send_to_server_OTlog();
                        ts0 = millis();
                        TCPserver_sts2 = 8; 

                    break;

                    case 8: //wait answer to CCMD_SEND_OTLOG_S from server
                    if(millis() - ts0 > 5000) // timeout todo
                    {   TCPserver_sts2 = 0; 
//                        Serial_db.printf("Timeout CCMD_SEND_OTLOG_S ***\n");
                        ts0 = millis();
                    } else if(TCPserver_rc == SCMD_SEND_OTLOG_C) {
                        TCPserver_sts2 = 5; 
                        ts0 = millis();
                    } else if(TCPserver_rc == CCMD_SEND_OTLOG_S) {
                        TCPserver_sts2 = 7; 
                        ts0 = millis();
                    }
                    break;

                    case 9: //send CCMD_SEND_LOG_S
                        Send_to_server_log(); //todo
                        ts0 = millis();
                        TCPserver_sts2 = 10; 
                    break;

                    case 10: //wait answer to CCMD_SEND_LOG_S from server
                    if(millis() - ts0 > 5000) // timeout todo
                    {   TCPserver_sts2 = 0; 
                        ts0 = millis();
                        Serial_db.printf("Timeout CCMD_SEND_LOG_S **\n");

                    } else if(TCPserver_rc == CCMD_SEND_STS_S) { 
                        TCPserver_sts2 = 5; 
                        ts0 = millis();
                    } else if(TCPserver_rc == CCMD_SEND_LOG_S) { //todo
                        TCPserver_sts2 = 9; 
                        ts0 = millis();
                    }
                    break;

#endif // OT_DEBUGLOG
                    
                }
        }
    }
}


//send to remote MCMD_HAND_SHAKE
void SD_Termo::Send_to_server_HandShake(void)
{   int l;
    unsigned char * MsgOut;
    struct Msg1 *msg;
    l = strlen(HAND_SHAKE_INP);

    TcpServer_Lsend = 6 + l;	

    MsgOut = server_get_buf(TcpServer_Lsend);
    msg  = (struct Msg1 *)MsgOut;
    msg->cmd0 = 0x22;
    msg->cmd  = MCMD_HAND_SHAKE;
    msg->ind = indcmd++;
	memcpy(&MsgOut[6], HAND_SHAKE_INP,l);
    TCPserver_close_on_send = 0;
}

//MCMD_INTRODUCESELF MD_IDENTIFY
void SD_Termo::Send_to_server_IdentifySelf(void)
{ int l, lp; 
    unsigned char * MsgOut;
    struct Msg1 *msg;
extern unsigned short int _bootCount, _bootReason, _bootSts, _bootSts1, _bootSts2; //  состояние в момент старта

    l = strlen((PGM_P)IDENTIFY_TEXT); 

    lp =  sizeof(int)*6 + 6 + 12 + l + 16;
    TcpServer_Lsend = 6 +  sizeof(short int) + lp;	
  
//    Serial_db.printf("==>> Send_IdentifySelf l= %d %d %d\n", l, lp, TcpServer_Lsend);
    MsgOut = server_get_buf(TcpServer_Lsend);
    msg  = (struct Msg1 *)MsgOut;
    msg->cmd0 = 0x22;
    msg->cmd  = MCMD_INTRODUCESELF;
    msg->ind = indcmd++;

    *((PACKED short int *) (&MsgOut[6])) = (short int)lp;
    *((unsigned short int *) (&MsgOut[8]))  = IDENTIFY_TYPE; 
    *((unsigned short int *) (&MsgOut[10])) = IDENTIFY_SUBTYPE; // = ST_VERS
    *((PACKED int *) (&MsgOut[12])) = IDENTIFY_CODE;
    *((PACKED int *) (&MsgOut[16])) = IdNumber;	
    *((PACKED int *) (&MsgOut[20])) = Vers;	
    *((PACKED int *) (&MsgOut[24])) = SubVers;	
    *((PACKED unsigned short int *) (&MsgOut[28])) = SubVers1;	
    *((PACKED unsigned short int *) (&MsgOut[30])) = Revision;	
    
 	memcpy((void *)&MsgOut[32],(void *)BiosDate,12);

	memcpy((void *)&MsgOut[44],(void *)&Mac[0],6);
//    Serial_db.printf("MAC: %02x %02x %02x %02x %02x %02x\n",Mac[0], Mac[1],Mac[2], Mac[3], Mac[4], Mac[5]);

    memcpy_P((void *)&MsgOut[50],(void *)(PGM_P)IDENTIFY_TEXT, l);

    MsgOut[50+l] = start_sts;	

    MsgOut[51+l] = rtc_get_reset_reason(0);	
    MsgOut[52+l] = rtc_get_reset_reason(1);

    MsgOut[53+l] = OTmemberCode;

    *((unsigned short int *) (&MsgOut[54+l])) = stsOT; 
    *((unsigned short int *) (&MsgOut[56+l])) = _bootCount; 
    *((unsigned short int *) (&MsgOut[58+l])) = _bootReason; 
    *((unsigned short int *) (&MsgOut[60+l])) = _bootSts; 
    *((unsigned short int *) (&MsgOut[62+l])) = _bootSts1; 
    *((unsigned short int *) (&MsgOut[64+l])) = _bootSts2; 

//    Serial_db.printf("start_sts %d %d %d l=%d lsend=%d\n", start_sts, MsgOut[51+l], MsgOut[52+l], l, TcpServer_Lsend );

}

 //send to remote MCMD_OT_INFO  
void SD_Termo::OpenThermInfo(void)
{   int i,l;
    unsigned char * MsgOut;
    struct Msg1 *msg;

    l = 16*4+6;
    TcpUdp_Lsend = 6 + l;	

    MsgOut = esp_get_buf(TcpUdp_Lsend);
    msg  = (struct Msg1 *)MsgOut;

    msg->cmd0 = 0x22;
    msg->cmd  = MCMD_OT_INFO;
    msg->ind = indcmd++;
    for(i=0; i<16*4; i++)
       msg->Buf[i] = i;

    memcpy((void *)&msg->Buf[0],(void *) Mac,6); 

#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&msg->Buf[6],(void *) &stOT,1); 
        stOT = ot_slave_stsOT;
        memcpy((void *)&msg->Buf[7],(void *) &stOT,1); 
    }
#else
    memcpy((void *)&msg->Buf[6],(void *) &stsOT,2); 

#endif

    memcpy((void *)&msg->Buf[8],(void *)&BoilerStatus,4); 
    memcpy((void *)&msg->Buf[12],(void *)&BoilerT,4);
    memcpy((void *)&msg->Buf[16],(void *)&RetT,4);

    if(Use_ID29_DHW_flag)   
        memcpy((void *)&msg->Buf[20],(void *)&Tstorage,4);
    else        
        memcpy((void *)&msg->Buf[20],(void *)&dhw_t,4);
    memcpy((void *)&msg->Buf[24],(void *)&FlameModulation,4);
    memcpy((void *)&msg->Buf[28],(void *)&Pressure,4);
    memcpy((void *)&msg->Buf[32],(void *)&statusDS18b20,4);
    memcpy((void *)&msg->Buf[36],(void *)&t1,4);
    memcpy((void *)&msg->Buf[40],(void *)&t2,4);
    memcpy((void *)&msg->Buf[44],(void *)&OEMDcode,4); 
    memcpy((void *)&msg->Buf[48],(void *)&CrasyState_count,4);
     if(needReport_CrasyState & 0x02)
            needReport_CrasyState &= ~0x02;

    memcpy((void *)&msg->Buf[52],(void *)&rcode[2],4);
    memcpy((void *)&msg->Buf[56],(void *)&rcode[3],4);
    memcpy((void *)&msg->Buf[60],(void *)&rcode[4],4);

    memcpy((void *)&msg->Buf[64],(void *)&Fault,1); 	//b1
      
}

//MCMD_GET_CAP
int SD_Termo::callback_Get_Capabilities( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
    short int B_flags, tmp, cap=0;

    unsigned int B_flags4;
  //  Serial_db.printf("callback_Get_Capabilities len %d ", len);
    Lsend = 6 + 20;
        if(len >= 8)
    {  memcpy((void *)&cap,(void *)&bf[6],2);
    }
    if(cap == 1)
        Lsend += 4;

    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
//  Serial_db.printf("callback_Get_Capabilities len %d ", len);
//  Serial_db.printf("MAC: %02x %02x %02x %02x %02x %02x\n",Mac[0], Mac[1],Mac[2], Mac[3], Mac[4], Mac[5]);

    memcpy((void *)&MsgOut[6],(void *) Mac,6); 
    B_flags =  CapabilitiesDetected;
    DetectCapabilities();

	memcpy((void *)&MsgOut[12],(void *) &B_flags,2); 
#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&MsgOut[14],(void *) &stOT,1); 
        stOT = ot_slave_stsOT;
        memcpy((void *)&MsgOut[15],(void *) &stOT,1); 
    }
#else
	memcpy((void *)&MsgOut[14],(void *) &stsOT,2); 
#endif


    B_flags4 = 0;

    if(HotWater_present)      B_flags4 |= 0x01;  //from SConfigSMemberIDcode
    if(CH2_present)           B_flags4 |= 0x02;  //from SConfigSMemberIDcode
    if(RetT_present)          B_flags4 |= 0x04;  //OTid_used
    if(Toutside_present)      B_flags4 |= 0x08;  //OTid_used 
    if(Pressure_present)      B_flags4 |= 0x10;  //OTid_used
    if(Dhw_t_present)         B_flags4 |= 0x2000;  //OTid_used

#if  MQTT_USE
     B_flags4 |= 0x100;  //MQTT_defined
     if(useMQTT)
         B_flags4 |= 0x200;  //use MQTT
#endif
#if  PID_USE
     B_flags4 |= 0x400;  //PID_defined
     if(usePID)
         B_flags4 |= 0x800;  //use PID
#endif
#if RELAY_USE
    if(Relay_present)
        B_flags4 |= 0x4000;  
#endif
#if ST_VERS == 2
    if(OT_slave_present)
        B_flags4 |= 0x8000; 
#endif

	memcpy((void *)&MsgOut[16],(void *) &B_flags4,4); 
    tmp = Use_remoteTCPserver;
	memcpy((void *)&MsgOut[20],(void *) &tmp, 2); 
//#define IDENTIFY_SUBTYPE ST_VERS
    tmp = IDENTIFY_SUBTYPE;
	memcpy((void *)&MsgOut[22],(void *) &tmp, 2); 
	memcpy((void *)&MsgOut[24],(void *) &OTmemberCode, 2);     
    if(cap == 1)
    {  	memcpy((void *)&MsgOut[26],(void *) &Vers, 1);     
    	memcpy((void *)&MsgOut[27],(void *) &SubVers, 1);     
    	memcpy((void *)&MsgOut[28],(void *) &SubVers1, 1);     
    	memcpy((void *)&MsgOut[29],(void *) &Revision, 1);     
    }

     return 0;
}

//MCMD_GET_OT_INFO
int SD_Termo::callback_Get_OpenThermInfo( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
    short int B_flags, tmp;
    int rc = 1, tmp4, statDS;

//    Serial_db.printf("callback_Get_OpenThermInfo len %d ", len);

    if(len != 12)
    { 
        Serial_db.printf("callback_Get_OpenThermInfo Error: len != 12\n");
        return 0;
    }
	memcpy((void *)&tmp4,(void *)&bf[6],4);
	memcpy((void *)&tmp,(void *)&bf[10],2);
    if(tmp & 0x02)
       TCPserver_report_period = tmp4*1000;

//todo debug !!!
/*
    if(tmp & 0x02)
     TCPserver_sts = 1;
    else 
     TCPserver_sts = 0;
*/
    TCPserver_close_on_send = tmp&0x01;

//    Serial_db.printf("TCPserver report_period %d close_on_send %d sts %d\n",
//         TCPserver_report_period, TCPserver_close_on_send, TCPserver_sts);

    Lsend = 6 + 72;
#if ST_VERS == 2
    if(OT_slave_present)
        Lsend += 4;
#endif

    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

 //   Serial_db.printf("callback_Get_OpenThermInfo Mac %02x %02x %02x %02x %02x %02x\n", Mac[0], Mac[1], Mac[2], Mac[3], Mac[4], Mac[5]);

    B_flags = 0;
    if(enable_CentralHeating) B_flags |= 0x01;
    if(enable_HotWater)       B_flags |= 0x02;  
    if(HotWater_present)      B_flags |= 0x10;  
    if(CH2_present)           B_flags |= 0x20;  
    if(Toutside_present)      B_flags |= 0x40;  
    if(Pressure_present)      B_flags |= 0x80; 
    if(RetT_present)          B_flags |= 0x1000;  //OTid_used
    if(Dhw_t_present)         B_flags |= 0x2000;  //OTid_used
#if  MQTT_USE
     B_flags |= 0x100;  //MQTT_defined
     if(useMQTT)
         B_flags |= 0x200;  //use MQTT
#endif
#if  PID_USE
     B_flags |= 0x400;  //PID_defined
     if(usePID)
         B_flags |= 0x800;  //use PID
#endif
#if RELAY_USE
    if(Relay_present)
        B_flags |= 0x4000;  
#endif
#if ST_VERS == 2
    if(OT_slave_present)
        B_flags |= 0x8000; 
#endif

/*    
    Serial_db.printf("B_flags %x\n", B_flags);
    Serial_db.printf("stsOT %x\n", stsOT);
    Serial_db.printf("t_lastwork %x\n", t_lastwork);
    Serial_db.printf("BoilerStatus %x\n", BoilerStatus);
*/         
	 memcpy((void *)&MsgOut[6],(void *) &B_flags,2); 

#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&MsgOut[8],(void *) &stOT,1); 
        stOT = ot_slave_stsOT;
     
        if(OT_slave_present)
        {
            if(OT_slave_mode && ot_slave_stsOT >= 0)
                stOT |= 0x4;

            memcpy((void *)&MsgOut[78],(void *) &ot_slave_t_lastwork,sizeof(time_t));  //sizeof(time_t) 4 ESP32, 8 ESP8266
        }
        memcpy((void *)&MsgOut[9],(void *) &stOT,1); 
    }
 #else
	 memcpy((void *)&MsgOut[8],(void *) &stsOT,2); 
 #endif
 
    memcpy((void *)&MsgOut[10],(void *) &t_lastwork,sizeof(time_t));  //sizeof(time_t) 4 ESP32, 8 ESP8266

{
//     Serial_db.printf("t_lastwork =  %s ", ctime(&t_lastwork));
//     Serial_db.printf("  %02x %02x %02x %02x \n", MsgOut[10], MsgOut[11],MsgOut[12], MsgOut[13]);
}

	memcpy((void *)&MsgOut[14],(void *) &BoilerStatus,4);  //20=12+8
//    Serial_db.printf("BoilerT %f\n", BoilerT);
//    Serial_db.printf("RetT %f\n", RetT);

	memcpy((void *)&MsgOut[18],(void *) &BoilerT,4); 
	memcpy((void *)&MsgOut[22],(void *) &RetT,4); 
	memcpy((void *)&MsgOut[26],(void *) &Tset,4); 
    memcpy((void *)&MsgOut[30],(void *) &Tset_r,4);

    if(Use_ID29_DHW_flag)   
        memcpy((void *)&MsgOut[34],(void *)&Tstorage,4);
    else        
        memcpy((void *)&MsgOut[34],(void *)&dhw_t,4);

    memcpy((void *)&MsgOut[38],(void *) &TdhwSet,4); 
    memcpy((void *)&MsgOut[42],(void *) &FlameModulation,4); 
	memcpy((void *)&MsgOut[46],(void *) &Pressure,4); 
    statDS = 0;
    if(stsT1 > 0)
	    statDS |= (stsT1&03);
    if(stsT2 > 0)
    statDS |= (stsT2&03)<<8;
    #if RELAY_USE
        if(Relay_sts)
            statDS |= 0x1000;
    #endif
    //  memcpy((void *)&MsgOut[50],(void *) &statDS,4); 
    memcpy((void *)&MsgOut[50],(void *) &statDS,2); 
    tmp = 0;
    memcpy((void *)&MsgOut[52],(void *) &tmp,2); 
//todo 52-54     
	memcpy((void *)&MsgOut[54],(void *) &t1,4); 
	memcpy((void *)&MsgOut[58],(void *) &t2,4); 
	memcpy((void *)&MsgOut[62],(void *) &Toutside,4); 
#if PID_USE
	memcpy((void *)&MsgOut[66],(void *) &tempindoor,4); 
	memcpy((void *)&MsgOut[70],(void *) &tempoutdoor,4); 
	memcpy((void *)&MsgOut[74],(void *) &TroomTarget,4); 
#else
    {   float tmp = 0.f;
	 memcpy((void *)&MsgOut[66],(void *) &tmp,4); 
	 memcpy((void *)&MsgOut[70],(void *) &tmp,4); 
	 memcpy((void *)&MsgOut[74],(void *) &tmp,4); 
    }
#endif

//    Serial_db.printf("callback_Get_OpenThermInfo rc %d Lsend %d ",rc, Lsend);

     //78
     return rc;
}

//CCMD_SEND_STS send to remote server
void SD_Termo::Send_to_server_Sts(void) 
{   unsigned char * MsgOut;
    Send_to_server_Sts(MsgOut, TcpServer_Lsend, server_get_buf );
}

//CCMD_SEND_STS_S send to remote server
void SD_Termo::Send_to_server_Sts(unsigned char * &MsgOut, int &Lsend, U8 *(*get_buf) (U16 size) )
{   short int B_flags, tmp;
    int statDS, l;
    struct Msg1 *msg;

    l = 78+8;
    Lsend = 6 +  l;	

    
    MsgOut = get_buf(Lsend);
    msg  = (struct Msg1 *)MsgOut;

    msg->cmd0 = 0x22;
    msg->cmd  = CCMD_SEND_STS_S;
    msg->ind = indcmd++;

//    memcpy((void *)&msg->Buf[0],(void *) Mac,6); 

//	 ClientId & ClientId_k todo; 

    memcpy((void *)&msg->Buf[0],(void *) &ClientId,4); 
 //   Serial_db.printf("ClientId  %d ClientId_k %x TCPserver_report_period %d\n", ClientId, ClientId_k);

    B_flags = 0;
    if(enable_CentralHeating) B_flags |= 0x01;
    if(enable_HotWater)       B_flags |= 0x02;  
    if(HotWater_present)      B_flags |= 0x10;  
    if(CH2_present)           B_flags |= 0x20;  
    if(Toutside_present)      B_flags |= 0x40;  
    if(Pressure_present)      B_flags |= 0x80; 
    if(RetT_present)          B_flags |= 0x1000;  //OTid_used
    if(Dhw_t_present)         B_flags |= 0x2000;  //OTid_used
      
#if  MQTT_USE
     B_flags |= 0x100;  //MQTT_defined
     if(useMQTT)
         B_flags |= 0x200;  //use MQTT
#endif
#if  PID_USE
     B_flags |= 0x400;  //PID_defined
     if(usePID)
     B_flags |= 0x800;  //use PID
#endif

#if RELAY_USE
    if(Relay_present)
        B_flags |= 0x4000;  
#endif
#if ST_VERS == 2
    if(OT_slave_present)
        B_flags |= 0x8000; 
#endif

/* todo Buf[6] -> Buf[4] */
    
	 memcpy((void *)&msg->Buf[6],(void *) &B_flags,2); 
#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&msg->Buf[8],(void *) &stOT,1); 
        stOT = ot_slave_stsOT; //-2 not initialise,  -1 not init interface, 0 - normal work, 2 - timeout
        if(OT_slave_mode && ot_slave_stsOT >= 0)
                stOT |= 0x4;

        memcpy((void *)&msg->Buf[9],(void *) &stOT,1); 
    }
#else
    memcpy((void *)&msg->Buf[8],(void *) &stsOT,2); 

#endif
   
     memcpy((void *)&msg->Buf[10],(void *) &t_lastwork,sizeof(time_t));  //sizeof(time_t) 4 ESP32, 8 ESP8266
	 memcpy((void *)&msg->Buf[14],(void *) &BoilerStatus,4);  //20=12+8
	 memcpy((void *)&msg->Buf[18],(void *) &BoilerT,4); 
	 memcpy((void *)&msg->Buf[22],(void *) &RetT,4); 
	 memcpy((void *)&msg->Buf[26],(void *) &Tset,4); 
	 memcpy((void *)&msg->Buf[30],(void *) &Tset_r,4); 
    if(Use_ID29_DHW_flag)   
        memcpy((void *)&msg->Buf[34],(void *) &Tstorage,4);
    else        
        memcpy((void *)&msg->Buf[34],(void *) &dhw_t,4);
	 memcpy((void *)&msg->Buf[38],(void *) &TdhwSet,4); 
     memcpy((void *)&msg->Buf[42],(void *) &FlameModulation,4); 
	 memcpy((void *)&msg->Buf[46],(void *) &Pressure,4); 
     statDS = 0;
    if(stsT1 > 0)
        statDS |= (stsT1&03);
    if(stsT2 > 0)
        statDS |= (stsT2&03)<<8;
#if RELAY_USE
    if(Relay_sts)
        statDS |= 0x1000;
#endif
    
    memcpy((void *)&msg->Buf[50],(void *) &statDS,2); 

#if OT_DEBUGLOG
    tmp = OTlogBuf.GetLbuf();
//  Serial.printf("Send_to_server_Sts OTlogBuf.GetLbuf=%d, nOTlog=%d sp_n=%d\n", tmp, nOTlog, Serial_db.sp_n);
    if(tmp  == 0)
    {   tmp = Serial_db.sp_n;
        tmp |= 0x8000;
//        Serial.printf("==>statDS=%x\n", tmp);
    } else  if(nOTlog < 16) tmp  = 0;
   
    memcpy((void *)&msg->Buf[52],(void *) &tmp,2); 
#else    
    tmp = 0;
    memcpy((void *)&msg->Buf[52],(void *) &tmp,2); 
#endif    

	 memcpy((void *)&msg->Buf[54],(void *) &t1,4); 
	 memcpy((void *)&msg->Buf[58],(void *) &t2,4); 
	 memcpy((void *)&msg->Buf[62],(void *) &Toutside,4); 
 #if PID_USE
	 memcpy((void *)&msg->Buf[66],(void *) &tempindoor,4); 
	 memcpy((void *)&msg->Buf[70],(void *) &tempoutdoor,4); 
	 memcpy((void *)&msg->Buf[74],(void *) &TroomTarget,4); 
	 memcpy((void *)&msg->Buf[78],(void *) &mypid.InT,4); 
	 memcpy((void *)&msg->Buf[82],(void *) &mypid.ub,4); 
#else
    {   float tmp = 0.f;
	 memcpy((void *)&msg->Buf[66],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[70],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[74],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[78],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[82],(void *) &tmp,4); 
    }
#endif
    TCPserver_rc = 0;
    TCPserver_close_on_send = 0; //wait answer
     //78
}

#if OT_DEBUGLOG

//CCMD_SEND_OTLOG_S send to server OpenTHerm log
void SD_Termo::Send_to_server_OTlog(void)
{   unsigned char * MsgOut;
    short int  tmp2;
    int  i, l, lb;
    struct Msg1 *msg;
    unsigned char buf[8];

//  if(nOT_need_send * OTlogBuf.Litem > (120-6)) nOT_need_send = (120-6)/OTlogBuf.Litem;
    if(nOT_need_send * OTlogBuf.Litem > (UDP_TSP_BUFSIZE-12)) 
        nOT_need_send = (UDP_TSP_BUFSIZE-12)/OTlogBuf.Litem;
    
    lb = OTlogBuf.GetLbuf();
    if(nOT_need_send > lb) nOT_need_send = lb;
    l = nOT_need_send * OTlogBuf.Litem;

    TcpServer_Lsend = 6 + 6 + l;	
  
    MsgOut =  server_get_buf(TcpServer_Lsend);
    msg  = (struct Msg1 *)MsgOut;

  //  Serial_db.printf("Sending_to_server_OTlog %d Lb %d L%d\n",  nOT_need_send, OTlogBuf.GetLbuf(), nOTlog );
 
    msg->cmd0 = 0x22;
    msg->cmd  = CCMD_SEND_OTLOG_S;

    msg->ind = indcmd++;

    memcpy((void *)&msg->Buf[0],(void *) &ClientId,4); 
    if(lb > 255) lb = 255;
    tmp2 = nOT_need_send | (lb<<8);

    memcpy((void *)&msg->Buf[4],(void *) &tmp2,2); 
    OTlogBuf.StartRead();
    for(i=0;i<nOT_need_send;i++)
    {  // OTlogBuf.Get(buf);
        OTlogBuf.Read(buf);
        memcpy((void *)&msg->Buf[6+i*OTlogBuf.Litem],(void *)buf,OTlogBuf.Litem); 
#if 0        
{   unsigned int ic, ind, tms;
    static int ic8=0, icold=0;
    unsigned int b[2];
        memcpy((void *)&b[0],(void *)buf,OTlogBuf.Litem); 
		ic = (b[0]>>24);
		sts = ic>>6;
		ic = ic & 0x3f;
        if(ic < icold)
        { ic8++;
        }
 		icold = ic;
		ind = ic8*0x40 + ic;

		tms = b[0] & 0xffffff;
        Serial.printf("===>[%d %d] %d\n",ic, tms, ind);
}
#endif // 0
//        Serial_db.printf("buf %02x %02x %02x %02x %02x %02x %02x %02x %02x \n",
//                buf[0],buf[1],buf[2],buf[3], buf[4],buf[5],buf[6],buf[7]);
    }
    TCPserver_rc = 0;
    TCPserver_close_on_send = 0; //wait answer
}

//SCMD_SEND_OTLOG_C = CCMD_SEND_OTLOG_S answer
int SD_Termo::server_answerOTLog( U8 *bf, int len)
{   unsigned short int tmp2;
    int rc = 0;
    nOTsend += nOT_need_send;
//    Serial_db.printf("SCMD_SEND_OTLOG_C len=%d ", len);
    if(len == 8)
    {   memcpy((void *)&tmp2,(void *)&bf[6],2);
//      Serial_db.printf("SCMD_SEND_OTLOG_C get=%d\n", tmp2);
        if(tmp2 > 16) tmp2 = 16;
        nOT_need_send = tmp2; 
        OTlogBuf.EndRead(); //Освобождаем буфер
//        Serial_db.printf("C len=%d, nOT_need_send =%d\n", tmp2, nOT_need_send);
        if(nOT_need_send > 0)          
        {   TCPserver_rc = CCMD_SEND_OTLOG_S;
            TCPserver_close_on_send = 0; //wait answer
            rc = 1; //not call asTCPserver.closeTCP() after net_ServerCallback()

        } else {
            TCPserver_rc = SCMD_SEND_OTLOG_C;
            TCPserver_close_on_send = 1; //not wait answer
        }
    }
    return rc;
}


//CCMD_SEND_LOG_S
void SD_Termo::Send_to_server_log(void) 
{   unsigned char * MsgOut;
    int  i, l, ln,n;
    struct Msg1 *msg;
    char *pstr=NULL;
    int maxlen = UDP_TSP_BUFSIZE - 20; // 20 !!!!!
//    TcpServer_Lsend = 6 + 14 + n;	
    
    i = pSerial_db->ind;
    l = 0;  
    TCPserver_rc = 0;

    if(pSerial_db->pmsg)
    {   if(i < pSerial_db->sp_n && pSerial_db->pmsg[i])
        {   pstr = pSerial_db->pmsg[i];
            if(pstr)
            {   l = strlen(pstr);
            }
        }
    }
    if(l == 0)
        return;

    n = l;
    if(l > maxlen )
    {   ln = pSerial_db->ls + maxlen;
        if(ln > l) ln = l;
        n = ln - pSerial_db->ls;
        pstr += pSerial_db->ls;
    } else {
        pSerial_db->ls = 0;
        ln = l;
    }

    TcpServer_Lsend = 6 + 14 + n;	

    MsgOut =  server_get_buf(TcpServer_Lsend);
    msg  = (struct Msg1 *)MsgOut;
    
    msg->cmd0 = 0x22;
    msg->cmd  = CCMD_SEND_LOG_S;
    msg->ind = indcmd++;
    
    memcpy((void *)&msg->Buf[0],(void *) &ClientId,4); 

    memcpy((void *)&msg->Buf[4],(void *) &pSerial_db->ind,2); //message index to send
    memcpy((void *)&msg->Buf[6],(void *) &pSerial_db->sp_n,2); //total number of messages 
    memcpy((void *)&msg->Buf[8],(void *) &pSerial_db->ls,2); //start index to send
    memcpy((void *)&msg->Buf[10],(void *) &n,2);  //number of chars to send
    memcpy((void *)&msg->Buf[12],(void *) &l,2);  //total length of string to send
    memcpy((void *)&msg->Buf[14],(void *) pstr, n);  //n chars to send pstr=pSerial_db->pmsg[pSerial_db->ind]
    
    pSerial_db->ls_s = pSerial_db->ls + n;
//    if(pSerial_db->) 
//    
//    Serial.printf("CCMD_SEND_LOG_S ind %d, %d byes from %d >%s< total %d\n", msg->ind,  n, l, pstr, TcpServer_Lsend );

    TCPserver_rc = 0;
    TCPserver_close_on_send = 0; //wait answer
}

//SCMD_SEND_LOG_C = CCMD_SEND_LOG_S answer
int SD_Termo::server_answerLog( U8 *bf, int len)
{   unsigned short int tmp2;
    int  i, l,  rc = 0;
    char *pstr=NULL;
    memcpy((void *)&tmp2,(void *)&bf[6],2);

    if(tmp2 == pSerial_db->ls)
    {
        i = pSerial_db->ind;
        l = 0;
        if(pSerial_db->pmsg)
        {   if(i < pSerial_db->sp_n && pSerial_db->pmsg[i])
            {   pstr = pSerial_db->pmsg[i];
                if(pstr)
                {   l = strlen(pstr);
                }
            }
        }
        if(l == 0)
            return 0;
        
        if(pSerial_db->ls_s == l)
        {          
  //          Serial.printf("0 pstr =%p pSerial_db->pmsg[pSerial_db->ind] = %p\n", pstr, pSerial_db->pmsg[pSerial_db->ind] );
            free(pstr);
            pstr = NULL;
    //        Serial.printf("1 pstr =%p pSerial_db->pmsg[pSerial_db->ind] = %p\n", pstr, pSerial_db->pmsg[pSerial_db->ind] );
            pSerial_db->sp_l -= l;
            pSerial_db->pmsg[pSerial_db->ind] = NULL;
            pSerial_db->ls = 0;
      //      Serial.printf("2 pstr =%p pSerial_db->pmsg[pSerial_db->ind] = %p\n", pstr, pSerial_db->pmsg[pSerial_db->ind] );
            pSerial_db->ind++;
            if(pSerial_db->ind >= pSerial_db->sp_n)
            {   pSerial_db->sp_n = 0;
                pSerial_db->ind = 0;
                TCPserver_rc = CCMD_SEND_STS_S;
            } else {
                TCPserver_rc = CCMD_SEND_LOG_S;
                TCPserver_close_on_send = 0; //wait answer
                rc = 1;
            }
        } else {
            pSerial_db->ls  = pSerial_db->ls_s;
            TCPserver_rc = CCMD_SEND_LOG_S;
            rc = 1;
        }   
    }
    return rc;
}


#endif //OT_DEBUGLOG

//MCMD_INTRODUCESELF answer
int SD_Termo::server_answer_IdentifySelf( U8 *bf, int len)
{   int tmp4;
    if(len < 12)
        return -1;
    TCPserver_rc = MCMD_INTRODUCESELF;
    memcpy((void *)&ClientId,(void *)&bf[6],4);
	memcpy((void *)&ClientId_k,(void *)&bf[10],4);
	memcpy((void *)&tmp4,(void *)&bf[14],4);
    TCPserver_report_period = tmp4*1000;
//    Serial_db.printf("******* server_answer_IdentifySelf ClientId %d ClientId_k %x TCPserver_report_period %d\n", 
//                    ClientId, ClientId_k, TCPserver_report_period);
//    Serial_db.printf("******* %li\n",  millis());

    return 0;
}

//CCMD_SEND_STS_S answer from remote server
// see also callback_Set_State()
int SD_Termo::servercallback_send_Sts_answ( U8 *bf, int len)
{   int tmp4, rc=0;
    short int remote_cmd;
    float v, vT;
    int isChange = 0;

    TCPserver_rc = CCMD_SEND_STS_S;
	memcpy((void *)&tmp4,(void *)&bf[6],4);
    TCPserver_report_period = tmp4*1000;
    memcpy((void *)&remote_cmd,(void *)&bf[10],2);
    TCPserver_close_on_send = 1; //not wait answer

    if(len == 6+4*4+2*2) //26
    {  
//  Serial.printf("####=== 1\n");
        if(remote_cmd == 1)
        {
            short int B_flags_toSet;
            float TroomTarget_toSet;
            float TdhwSet_toSet;
            memcpy((void *)&B_flags_toSet,(void *)&bf[12],2);
            memcpy((void *)&vT, (void *)&bf[14],4);
            memcpy((void *)&v,  (void *)&bf[18],4);
            memcpy((void *)&TdhwSet_toSet,      (void *)&bf[22],4);
//  Serial_db.printf("servercallback_send_Sts_answ TCPserver_report_period  %d\n", TCPserver_report_period);
//    Serial_db.printf("B_flags_toSet %x Tset_toSet %f TroomTarget_toSet %f TdhwSet_toSet %f\n", B_flags_toSet, vT, v, TdhwSet_toSet );
//    Serial_db.printf("B_flags_toSet %x \n", B_flags_toSet);

//B_flags_toSet todo
#if RELAY_USE    
        if(Relay_present)
        {  int r = 0;
        if(B_flags_toSet & 0x1000)
                r = 1;
            RelayOnOff(r);
        }
#endif

#if ST_VERS == 2
    if(OT_slave_present)
    {   if(B_flags_toSet & 0x2000)
            OT_slave_mode = 1;
        else
            OT_slave_mode = 0;
    }
#endif    


            vT = CHtempLimit(vT);
            TdhwSet_toSet = CHtempLimit(TdhwSet_toSet);

#if  PID_USE
            v =  RoomtempLimit(v);
            TroomTarget_toSet = v;

            if(usePID)
            {   if(mypid.xTag !=  TroomTarget_toSet)
                {   set_new_PID_setpoint(TroomTarget_toSet, 2); 
//   Serial_db.printf("**** servercallback_send_Sts_answ: TroomTarget = %f xTag = %f\n", TroomTarget, mypid.xTag);
                    
                    isChange = 1;
                }
            } else {
                if(vT != Tset)
                { Tset = vT;
                    need_set_T(1);
                    isChange = 1;
                } 
            }
#else
            if(vT != Tset)
            { Tset = vT;
                need_set_T = 1;
                isChange = 1;
            } 
#endif
            if(TdhwSet_toSet != TdhwSet)
            {   TdhwSet = TdhwSet_toSet;
                need_set_dhwT(1);
                isChange = 1;
            } 

            if(isChange)
                need_write_f = 1;  //need write changes to FS
        } 
    } else if(len == 6+4*3+2) { //20
//      Serial.printf("####=== 2\n");
        if(remote_cmd == 0x10)
        {   float _It, _U0;
            start_sts = 0;  
            memcpy((void *)&_It, (void *)&bf[12],4);
            memcpy((void *)&_U0, (void *)&bf[16],4);
//todo             
//   Serial_db.printf("**** servercallback_send_Sts_answ: _It = %f _U0 = %f\n", _It, _U0);
            if(_U0 != 0.f)
                    _U0start = _U0;
            mypid.InT = _It;
            mypid.dSt.n = mypid.dSt.ind = 0;
            InTstartset = 1;       
        }
    } else if(remote_cmd == 0x02) {
#if OT_DEBUGLOG
        
        if(len == 6+4*2+2) //16       
        {
            memcpy((void *)&tmp4,(void *)&bf[12],4);
//   Serial.printf("****$$ servercallback_send_Sts_answ: len = %d tmp4 = %d\n", len, tmp4);
            if(tmp4 > 16) tmp4 = 16;
            nOT_need_send = tmp4; 
            if(nOT_need_send > 0)          
            {    TCPserver_rc = CCMD_SEND_OTLOG_S;
                TCPserver_close_on_send = 0; // wait answer
                rc = 1;
            }
        }
#endif    
//    } else if((len == 6+4+2) && remote_cmd == 0x03) {
    } else if( remote_cmd == 0x03) {
        if(len == 6+4*2+2) //16
        {   memcpy((void *)&tmp4,(void *)&bf[12],4);
            Serial_db.LogMode = tmp4;
        }

//     Serial.printf("****^^ servercallback_send_Sts_answ: len = %d remote_cmd = %d tmp4 = %d\n", len, remote_cmd, tmp4);
      
            TCPserver_rc = CCMD_SEND_LOG_S;
            TCPserver_close_on_send = 0; // wait answer
            rc = 1;
        }

    return rc;
}


//SCMD_GET_STS answer  to remote server
int SD_Termo::servercallback_Get_Sts( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   int  tmp4;
  
//    Serial_db.printf("!!!!!!!!servercallback_Get_Sts len %d\n", len);
    if(len != 10)
    { 
        Serial_db.printf("Error: len != 10\n");
        return 0;
    }
    TCPserver_rc = SCMD_GET_STS;
	memcpy((void *)&tmp4,(void *)&bf[6],4);
    TCPserver_report_period = tmp4*1000;
//    Serial_db.printf("!!!!!!!!TCPserver_report_period  %d\n", TCPserver_report_period);

    Send_to_server_Sts( MsgOut, Lsend, get_buf); 
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); // msg->cmd  = SCMD_GET_ST;
      return 1; 
};

//MCMD_SET_OT_DATA:
// set
// enable_CentralHeating
// enable_HotWater
// Tset or mypid.xTag if PID used and enabled
// 20 bytes in, 6 bytes out
void SD_Termo::callback_Set_OpenThermData( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
    short int B_flags;
    float v, vT;
    bool flag;
    int isChange = 0;

    Lsend = 6;
    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

	memcpy((void *)&B_flags,(void *)&bf[6],2);
    if(B_flags & 0x01)
            flag = true;
    else
            flag  = false;
    if(flag != enable_CentralHeating)
    {   enable_CentralHeating = flag;
        isChange = 1;
    }

    if(HotWater_present) 
    {   if(B_flags & 0x02)
            flag = true;
        else
            flag = false;

        if(flag != enable_HotWater)
        {   enable_HotWater = flag;
            isChange = 1;
        }
    }

	memcpy((void *)&v,(void *)&bf[6+2],4); //Tset
    vT = CHtempLimit(v);

#if  PID_USE
    float roomSetpointT;
	memcpy((void *)&v,(void *)&bf[6+6],4); //roomSetpointT
    if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
    else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    roomSetpointT = v;

    if(usePID)
    {   if(mypid.xTag != roomSetpointT)
        {   set_new_PID_setpoint(roomSetpointT, 3); 
//   Serial_db.printf("**** callback_Set_OpenThermData: TroomTarget = %f xTag = %f\n", TroomTarget, mypid.xTag);
            isChange = 1;
        }
    } else {
      if(vT != Tset)
      { Tset = vT;
        need_set_T(1);
        isChange = 1;
      } 
    }
#else
      if(vT != Tset)
      { Tset = vT;
        need_set_T = 1;
        isChange = 1;
      } 
#endif
	memcpy((void *)&v,(void *)&bf[6+10],4); //TdhwSet
    vT = CHtempLimit(v);
    if(vT != TdhwSet)
    { TdhwSet = vT;
        need_set_dhwT(1);
        isChange = 1;
      } 


    if(isChange)
        need_write_f = 1;  //need write changes to FS

}

//ACMD_SET_STATE_C
// see also servercallback_send_Sts_answ
void SD_Termo::callback_Set_State( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   short int B_flags;
    float v, vT;
    bool flag;
    int isChange = 0;

    Lsend = 6;
    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

	memcpy((void *)&B_flags,(void *)&bf[6],2);
    if(B_flags & 0x01)
            flag = true;
    else
            flag  = false;
    if(flag != enable_CentralHeating)
    {   enable_CentralHeating = flag;
        isChange = 1;
    }

    if(HotWater_present) 
    {   if(B_flags & 0x02)
        flag = true;
        else
            flag = false;

        if(flag != enable_HotWater)
        {   enable_HotWater = flag;
            isChange = 1;
        }
    }

#if RELAY_USE
    if(Relay_present)
    {  int r = 0;
       if(B_flags & 0x1000)
            r = 1;
        RelayOnOff(r);
    }
#endif    

#if ST_VERS == 2
    if(OT_slave_present)
    {   if(B_flags & 0x2000)
            OT_slave_mode = 1;
        else
            OT_slave_mode = 0;
    }
#endif    

	memcpy((void *)&v,(void *)&bf[6+2],4); //Tset
    vT = CHtempLimit(v);

#if  PID_USE
    float roomSetpointT;
	memcpy((void *)&v,(void *)&bf[6+6],4); //roomSetpointT
    if(usePID == 1)
    {   if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
        else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    }
    roomSetpointT = v;

    if(usePID)
    {   if(mypid.xTag != roomSetpointT)
        {   set_new_PID_setpoint(roomSetpointT, 4);
//   Serial_db.printf("**** callback_Set_State: TroomTarget = %f xTag = %f\n", TroomTarget, mypid.xTag);
            isChange = 1;
        }
    } else {
      if(vT != Tset)
      { Tset = vT;
        need_set_T(1);
        isChange = 1;
      } 
    }
#else
      if(vT != Tset)
      { Tset = vT;
        _need_set_T = 1;
        isChange = 1;
      } 
#endif
	memcpy((void *)&v,(void *)&bf[6+10],4); //TdhwSet
    vT = CHtempLimit(v);
 // Serial_db.printf("%s, v=%f vT=%f TdhwSet=%f\n", __FUNCTION__, v, vT, TdhwSet); 
    if(vT != TdhwSet)
    { TdhwSet = vT;
        need_set_dhwT(1);
        isChange = 1;
    } 


    if(isChange)
        need_write_f = 1;  //need write changes to FS

}


//MCMD_GETDATA
void  SD_Termo::callback_getdata( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
   Lsend = 6 + 6 + sizeof(int)*2 + sizeof(float)*9; 

   MsgOut = get_buf(Lsend);

	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&MsgOut[6],(void *) Mac,6); 
	 memcpy((void *)&MsgOut[12],(void *)&BoilerStatus,4); 
	 memcpy((void *)&MsgOut[16],(void *)&BoilerT, 4); 
	 memcpy((void *)&MsgOut[20],(void *)&RetT, 4); 

    if(Use_ID29_DHW_flag)   
        memcpy((void *)&MsgOut[24],(void *)&Tstorage, 4); 
    else        
        memcpy((void *)&MsgOut[24],(void *)&dhw_t, 4); 
        
	 memcpy((void *)&MsgOut[28],(void *)&FlameModulation, 4); 
	 memcpy((void *)&MsgOut[32],(void *)&Pressure, 4); 
	 memcpy((void *)&MsgOut[36],(void *)&Tset, 4); 
	 memcpy((void *)&MsgOut[40],(void *)&TdhwSet, 4); 
	 memcpy((void *)&MsgOut[44],(void *)&statusDS18b20, 4);  //статус внешних датчиков температуры - (не OT)
	 memcpy((void *)&MsgOut[48],(void *)&t1,4); 
	 memcpy((void *)&MsgOut[52],(void *)&t2,4); 

#if SERIAL_DEBUG      
  Serial_db.printf("%s, BoilerStatus=%d T1=%f T2=%f\n", __FUNCTION__, BoilerStatus, t1, t2 ); 
#endif
}

//MCMD_SET_TCPSERVER
void SD_Termo::callback_set_tcp_server( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   int s, dt, p, ischange = 0; // i, rc;
    char buf[20];
    IPAddress  ip;

    Lsend = 6; 
    MsgOut = get_buf(Lsend);
	
    memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
    if(!Use_remoteTCPserver)
        return;

    memcpy((void *)&s,(void *)&bf[6],4); 
    memcpy((void *)buf,(void *)&bf[10],20); 

//  Serial_db.printf("callback_set_tcp_server sts=%d remoteIP =%s\n", s, buf);

    ip.fromString(buf);
    if(tcp_remoteIP != ip)
    {   ischange |= 0x01;
        tcp_remoteIP = ip;
    }

//  Serial_db.printf("tcp_remoteIP = %s TCPserver_sts =%d\n",tcp_remoteIP.toString().c_str(), TCPserver_sts); 

#if SERIAL_DEBUG 
//  Serial_db.printf("callback_set_tcp_server sts=%d remoteIP =%s\n", s, buf);
//  tcp_remoteIP.fromString(buf);
//  Serial_db.printf("==");
//  Serial.println(tcp_remoteIP); // print the parsed IPAddress 

#endif
    memcpy((void *)&dt, (void *)&bf[30],4); 
    memcpy((void *)&p,(void *)&bf[34],4); 

    if(s > 0)
    {   TCPserver_sts = 1;  /* статус сервера */
        TCPserver_t = millis();
        s--;
        if(38+s <= len)
        {   char *pstr = (char *)&bf[38]; 
            int isch=1;
            if(len < size)
                bf[38 + s] = 0;

            if(remote_Host != nullptr)
            {   if(strlen(remote_Host) == s)
                {   if(strncpy(pstr,remote_Host,s))
                    { //  Serial.printf("remote_Host same\n");
                        isch = 0;
                    }
                }
                if(isch)
                {   free(remote_Host);
                    remote_Host = strdup(pstr);
                    ischange |= 0x08;
//                       Serial.printf("remote_Host Not same\n");
                }
            } else { 
                    remote_Host = strdup(pstr);
                    ischange |= 0x08;
//              Serial.printf("remote_Host nullptr\n");
            }

        }
    } else { 
        TCPserver_sts = 0;
    }

    if(TCPserver_port != p)
    {   ischange |= 0x02;
        TCPserver_port = p;  
    }
  
    if(TCPserver_report_period != dt)
    {   ischange |= 0x04;
        TCPserver_report_period = dt;
    }
  
    if(ischange)
    {   t_need_write_config = millis();
        need_write_f |= 0x04;
    }
    
  //Serial_db.printf("need_write_f %d %x TCPserver_port %d sts=%d TCPserver_report_period =%d\n", 
  //      need_write_f,  ischange, TCPserver_port, TCPserver_sts, TCPserver_report_period);

}


void  SD_Termo::callback_testcmd( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
   Lsend = 6; 
   MsgOut = get_buf(Lsend);
	
	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&TestId,(void *)&bf[6],4); 
	 memcpy((void *)&TestPar,(void *)&bf[6+4],4); 
    TestCmd = 1;
    TestResponse = -1;
    TestStatus = -1;
#if SERIAL_DEBUG 
//    Serial_db.printf("%s, TestCmd =%d TestId=%i TestPar=%i\n", __FUNCTION__, TestCmd, TestId, TestPar ); 
#endif    
}

void  SD_Termo::callback_testcmdanswer( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
   Lsend = 6 + 4*2; 
   MsgOut = get_buf(Lsend);
	
	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&MsgOut[6],(void *)&TestResponse,4); 
	 memcpy((void *)&MsgOut[6+4],(void *)&TestStatus,4 ); 

}

//src 0/1 - T1/T2, 2 - Text, 3  MQTT0 4 MQTT1 
void SD_Termo::OnChangeT(float t, int src)
{
#if PID_USE
    if(src>= 0 && src <= MAX_PID_SRC)
    {
        t_mean[src].add(t);
//   if(src ==2)
//     Serial_db.printf("OnChangeT 2, %li t =%f\n", millis(), t); 
//    Serial_db.printf("OnChangeT src =%d, t =%f mean =%f nx=%d\n", src, t, t_mean[src].xmean, t_mean[src].nx); 
    }
#endif
}

#if PID_USE


#endif // PID_USE

#if OT_DEBUGLOG
void SD_Termo::callback_GetOTLog( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   short int logsts, nitems, l0;
    int i, l, li;
    unsigned char buf[8];

//Set  enable_OTlog to logsts
//if buffer is no empty - get nitems (or less)
	 memcpy((void *)&logsts,(void *)&bf[6],sizeof(short int)); 
	 memcpy((void *)&nitems,(void *)&bf[8],sizeof(short int)); 

    li = OTlogBuf.Litem;
    l0 = OTlogBuf.GetLbuf();
    l = l0;
    if(l > 0 && nitems > 0)
    {   if(l < nitems)
            nitems = l;
        l *= li;
        l += 6+6;
        if(l > UDP_TSP_BUFSIZE)
        {   l = UDP_TSP_BUFSIZE - (6+6);
            nitems = l /li;
            l = li * nitems + 6 + 6;
        }
        Lsend = l + 6 + 6; 
    } else {
        Lsend = 6 + 6; 
        nitems = 0;
    }
     MsgOut = get_buf(Lsend);
	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
     
     i = 0;
     if(enable_OTlog) i = 1;
	 memcpy((void *)&MsgOut[6],(void *)&i,2); 
	 memcpy((void *)&MsgOut[8],(void *)&l0,2);  // length of buffer in items
	 memcpy((void *)&MsgOut[10],(void *)&nitems,2); //number of items send
     if(nitems)
     {  for (i = 0; i < nitems; i++)
        {   OTlogBuf.Get(buf);
        	memcpy((void *)&MsgOut[12+(i*li)],(void *)buf,li); 
        }
     }
     if(logsts)
        enable_OTlog = true;
    else
        enable_OTlog = false;

}
#endif //OT_DEBUGLOG

 /* return t within limit MinCHtemp MAX_CH_TEMP*/
float SD_Termo::CHtempLimit(float _t)
{   if(_t < MinCHtemp) 
        return MinCHtemp;
    else  if(_t > MAX_CH_TEMP) 
        return MAX_CH_TEMP;
    return _t;
}

/* return t within limit MIN_ROOM_TEMP MAX_ROOM_TEMP */
float SD_Termo::RoomtempLimit(float _t)
{   
#if PID_USE
    if(usePID == 1)
    {   
        if(_t < MIN_ROOM_TEMP) 
            return MIN_ROOM_TEMP;
        else  if(_t > MAX_ROOM_TEMP) 
            return MAX_ROOM_TEMP;
    }
#endif        
    return _t;
}


void SD_Termo::DetectCapabilities(void)
{
extern OpenTherm ot;
    if(CapabilitiesDetected  == 1)
    {   int count, countok;
            ot.Get_OTid_count(OpenThermMessageID::CHPressure, count, countok);
            if(countok > 1)
                Pressure_present = true; 
            else
                Pressure_present = false; 
            ot.Get_OTid_count(OpenThermMessageID::Toutside, count, countok);
            if(countok > 1)
                Toutside_present = true;                 
            else
                Toutside_present  = false; 
            ot.Get_OTid_count(OpenThermMessageID::Tret, count, countok);
            if(countok > 1)
                RetT_present = true;                 
            else
                RetT_present  = false; 

            ot.Get_OTid_count(OpenThermMessageID::Tstorage, count, countok); //ID 29
            if(countok > 1)
                Tstorage_present = true;                 
            else
                Tstorage_present = false;

            ot.Get_OTid_count(OpenThermMessageID::Tdhw, count, countok); //ID 26
            if(countok > 1)
                Dhw_t_present = true;                 
            else
                Dhw_t_present = false;


            ot.Get_OTid_count(OpenThermMessageID::MaxRelModLevelSetting, count, countok); //ID 14
// Serial_db.printf("SD_Termo::DetectCapabilities MaxRelModLevelSetting %d %d\n", count, countok);
            if(countok > 1)
                MaxRelModLevel_present = true;                 
            else
                MaxRelModLevel_present = false;
//    Serial.printf("1 MaxRelModLevel_present %d countok %d \n", MaxRelModLevel_present, countok); 

            ot.Get_OTid_count(OpenThermMessageID::RemoteRequest, count, countok); //ID 4
            if(countok > 1)
                RemoteRequest_present = true;                 
            else
                RemoteRequest_present = false;

            ot.Get_OTid_count(OpenThermMessageID::DHWFlowRate, count, countok); //ID 19
            if(countok > 1)
                DHWFlowRate_present = true;                 
            else
                DHWFlowRate_present = false;                            

    } else  if(CapabilitiesDetected  == 2) {
        ot.update_OTids();

        if(!ot.OTid_used(OpenThermMessageID::SConfigSMemberIDcode))
        {
//            Serial.printf("Клиника детектед\n");
//  котёл Stout Plus
            if(ot.OTid_used(OpenThermMessageID::Tdhw) && ot.OTid_used(OpenThermMessageID::TdhwSet))
            { 
               // Serial.printf("Есть температура горячей воды и её уставка \n");
                HotWater_present = true;
            }
        } else if(HotWater_present == false && ot.OTid_used(OpenThermMessageID::Tdhw) && ot.OTid_used(OpenThermMessageID::TdhwSet)) { // Hubert AGB 24 DP ?
            HotWater_present = true;
        }

        if(ot.OTid_used(OpenThermMessageID::CHPressure))
                Pressure_present = true;
        else
                Pressure_present = false;
        if(ot.OTid_used(OpenThermMessageID::Toutside))
                Toutside_present = true;
        else
                Toutside_present  = false;

        if(ot.OTid_used(OpenThermMessageID::Tret))
                RetT_present = true;
        else
                RetT_present  = false; 

        if(ot.OTid_used(OpenThermMessageID::Tstorage))
                Tstorage_present = true;
        else
                Tstorage_present  = false;

        if(ot.OTid_used(OpenThermMessageID::Tdhw))
                Dhw_t_present = true;
        else
                Dhw_t_present  = false;

//  Serial_db.printf("SD_Termo::DetectCapabilities 2 MaxRelModLevelSetting used %d \n", ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting));

        if(ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting))
                MaxRelModLevel_present = true;
        else
                MaxRelModLevel_present  = false;

        if(ot.OTid_used(OpenThermMessageID::RemoteRequest))
                RemoteRequest_present = true;
        else
                RemoteRequest_present  = false;

        if(ot.OTid_used(OpenThermMessageID::DHWFlowRate))
                DHWFlowRate_present = true;                 
        else
                DHWFlowRate_present = false;                    
    }               
}

void SD_Termo::OnOpenThermRestore(void)
{
extern OpenTherm ot;

#if  PID_USE
    if(enable_CentralHeating_real)
#else 
    if(SmOT.enable_CentralHeating)
#endif
        need_set_T(2); // if request fail, i.e. with errors in  sendind data we need to set T multiple times
    if(enable_HotWater) 
            need_set_dhwT(2);                   
    if(enable_CentralHeating2)
            need_set_T_CH2(2);

    if(ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting) && Use_MaxRelModLevel)
    {   need_set_MaxRelModLevel(2);
    }
}

/* считаем число включений горелки */
void BoilerStatisic::calcNflame(int newSts)
{
    if(newSts) // включение
    {
        NflameOn++;
        NflameOn_h++;
        NflameOn_day++;
        t_flame_on = time(nullptr);
    } else {  //Выключение
        t_flame_off = time(nullptr);
    }
}

/* считаем число включений горячей воды */
void BoilerStatisic::calcN_HW(int newSts)
{
    if(newSts) // включение
    {
//        NflameOn++;
//        NflameOn_h++;
//        NflameOn_day++;
        t_HW_on = time(nullptr);
    } else {  //Выключение
        t_HW_off = time(nullptr);
    }
}

/* Считаем интеграл пламени */
void BoilerStatisic::calcIntegral(float flame)
{   time_t now; 
    int dt;
    float d;
    static float flame_prev = 0.f;
    now = time(nullptr);
    dt = now - t_I_last;
    if(dt == 0)
      return;
    t_I_last = now;
    if(flame > 0.f || flame_prev > 0.f)
    {   d =  (flame_prev + flame) * 0.5f * dt;
        ModIntegral_h += d;
        ModIntegral_d += d;
    }
    flame_prev = flame;
    
    sec_h += dt;
    sec_d += dt;
}

