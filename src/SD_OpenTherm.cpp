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
typedef WebServer WEBServer;
#endif
#include <AutoConnect.h>
#include <AutoConnectFS.h>
extern AutoConnectFS::FS& FlashFS;

#include "SD_OpenTherm.hpp"
#include "Smart_commands.h"


extern int TcpUdp_Lsend;
extern PACKED unsigned char *Udp_MsgOut;

extern IPAddress Udp_remoteIP;  
extern int Udp_RemotePort;
extern U8 *esp_get_buf (U16 size);

//struct Msg1 msg;
static int indcmd = 0;

#if MQTT_USE
  #if defined(ARDUINO_ARCH_ESP8266)
//-110 = 102
     #if PID_USE
        #define FS_BUF 256
    #else
        #define FS_BUF 108  
    #endif
  #elif defined(ARDUINO_ARCH_ESP32)
    #define FS_BUF 256
  #endif
#else
    #if PID_USE
        #define FS_BUF 70
    #else
        #define FS_BUF 66  
    #endif
#endif

const char *path="/smot_par";

int SD_Termo::Read_ot_fs(void)
{  int rc, n, nw;
    uint8_t Buff[FS_BUF];

    rc = Read_data_fs((char *)path, Buff, FS_BUF, nw);
    if(rc)
        return 1;
#if SERIAL_DEBUG      
    Serial.printf((PGM_P)F("Read %i bytes\n"), nw);
#endif    

    n = sizeof(enable_CentralHeating);
    memcpy((void *) &enable_CentralHeating, &Buff[0], n);
    memcpy((void *) &enable_HotWater, &Buff[n], sizeof(enable_HotWater));
    n += sizeof(enable_HotWater);
    memcpy((void *) &Tset, &Buff[n], sizeof(Tset));
    n += sizeof(Tset);
    memcpy((void *) &TdhwSet, &Buff[n], sizeof(TdhwSet));
    n += sizeof(TdhwSet);
    memcpy((void *) &UDPserver_repot_period, &Buff[n], sizeof(UDPserver_repot_period));
    n += sizeof(UDPserver_repot_period);
    memcpy((void *) &UDPserver_port, &Buff[n], sizeof(UDPserver_port));
    n += sizeof(UDPserver_port);
    memcpy((void *) &UseID2, &Buff[n], sizeof(UseID2));
    n += sizeof(UseID2);
    memcpy((void *) &ID2masterID, &Buff[n], sizeof(ID2masterID));
    n += sizeof(ID2masterID);

#if MQTT_USE
  if(n < nw)
  {
    memcpy((void *) &useMQTT, &Buff[n], sizeof(useMQTT));
    n += sizeof(useMQTT);
    memcpy((void *) MQTT_server, &Buff[n], sizeof(MQTT_server));
    n += sizeof(MQTT_server);
    memcpy((void *) MQTT_user, &Buff[n], sizeof(MQTT_user));
    n += sizeof(MQTT_user);
    memcpy((void *) MQTT_pwd, &Buff[n], sizeof(MQTT_pwd));
    n += sizeof(MQTT_pwd);
    memcpy((void *) MQTT_topic, &Buff[n], sizeof(MQTT_topic));
    n += sizeof(MQTT_topic);
    memcpy((void *) MQTT_devname, &Buff[n], sizeof(MQTT_devname));
    n += sizeof(MQTT_devname);
    memcpy((void *) &MQTT_interval, &Buff[n], sizeof(MQTT_interval));
    n += sizeof(MQTT_interval);
  }
#endif
#if PID_USE
    if(n >= nw) goto END;
    memcpy((void *) &usePID, &Buff[n], sizeof(usePID));
    n += sizeof(usePID);
    if(n >= nw) goto END;
        Serial.printf("usePID=%d\n", usePID);
    memcpy((void *) &srcTroom, &Buff[n], sizeof(srcTroom));
    n += sizeof(srcTroom);
    if(n >= nw) goto END;
    memcpy((void *) &srcText, &Buff[n], sizeof(srcText));
    n += sizeof(srcText);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.Kp, &Buff[n], sizeof(mypid.Kp));
    n += sizeof(mypid.Kp);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.Kd, &Buff[n], sizeof(mypid.Kd));
    n += sizeof(mypid.Kd);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.Ki, &Buff[n], sizeof(mypid.Ki));
    n += sizeof(mypid.Ki);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.xTag, &Buff[n], sizeof(mypid.xTag));
    n += sizeof(mypid.xTag);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.umax, &Buff[n], sizeof(mypid.umax));
    n += sizeof(mypid.umax);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.umin, &Buff[n], sizeof(mypid.umin));
    n += sizeof(mypid.umin);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.u0, &Buff[n], sizeof(mypid.u0));
    n += sizeof(mypid.u0);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.y0, &Buff[n], sizeof(mypid.y0));
    n += sizeof(mypid.y0);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.u1, &Buff[n], sizeof(mypid.u1));
    n += sizeof(mypid.u1);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.y1, &Buff[n], sizeof(mypid.y1));
    n += sizeof(mypid.y1);
    if(n >= nw) goto END;
   
#endif

    if(n >= nw) goto END;
    memcpy((void *) &CH2_DHW_flag, &Buff[n], sizeof(CH2_DHW_flag));
    n += sizeof(CH2_DHW_flag);
END:

    if(n != nw)
        Serial.printf((PGM_P)F("Warning:read %d bytes, use %d\n"), nw, n);

#if SERIAL_DEBUG      
    if(n != nw)
        Serial.printf((PGM_P)F("Warning:read %d bytes, use %d\n"), nw, n);

    Serial.printf((PGM_P)F("enable_CentralHeating=%i\n"), enable_CentralHeating);
    Serial.printf((PGM_P)F("enable_HotWater=%i\n"), enable_HotWater);
    Serial.printf((PGM_P)F("Tset=%.1f TdhwSet=%.1f\n"), Tset, TdhwSet);

#if MQTT_USE
    Serial.printf((PGM_P)F("useMQTT=%i\n"), useMQTT);
#endif
#endif // SERIAL_DEBUG      

    return 0;
}


int SD_Termo::Read_data_fs(char *_path, uint8_t *dataBuff, int len, int &rlen)
{   int  n, nw, i, l;
    unsigned short int crs, crs_r, nn;
    uint8_t Buff[FS_BUF];

    rlen = 0;
#if SERIAL_DEBUG      
    Serial.printf((PGM_P)F("Reading file: %s\n"), _path);
#endif

#if defined(ARDUINO_ARCH_ESP8266)
//    File file = FlashFS.open(_path,"r" );
#else
//    File file = FlashFS.open(_path, FILE_READ );
#endif
    File file = FlashFS.open(_path,"r" );
    if(!file || file.isDirectory())
    {  
#if SERIAL_DEBUG      
         if(!file)
                Serial.println(F("- failed to open file for reading"));
        else
                Serial.println(F("- file.isDirectory"));
#endif                
        if(file)
        {   file.close();
#if SERIAL_DEBUG      
            Serial.println(F("file.close()"));
#endif            
        }

        return 1;
    }

//read 2 byte - length of data
    n = file.read((unsigned char *)&nn, sizeof(nn));
    if(n != sizeof(nn))
    {   file.close();
#if SERIAL_DEBUG      
        Serial.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,sizeof(nn));
#endif        
        return 3;
    }
    if((nn +  sizeof(unsigned short int)) > (sizeof(Buff) ))
        return 10;

    if(len  < int(nn +  sizeof(unsigned short int)))
        return 11;
    nw = nn;
    n = sizeof(nn);
    memcpy(&Buff[0],(void *) &nn, n);
//    nw += sizeof(short int);
    l = n;
    n = file.read((unsigned char *)&Buff[l], nw); //read nn bytes of data
    if(n != nw)
    {   file.close();
#if SERIAL_DEBUG      
        Serial.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,nw);
#endif        
        return 3;
    }
    l += n;
    n = file.read((unsigned char *)&crs_r, sizeof(short int));  //read 2 bytes control sum

/***************************/    
//    for(i=0; i<l; i++)
//            Serial.printf("%02x ",Buff[i]);
//    Serial.printf("\n");
/***************************/    

    crs = 0;
    for(i=0; i<l; i++)
    {  crs += Buff[i];
    }

    if(crs !=  crs_r )
    {   file.close();
#if SERIAL_DEBUG      
        Serial.printf((PGM_P)F("crs = %i, must be =%i\n"),crs_r,crs);
#endif        
        return 4;
    }

    file.close();
#if SERIAL_DEBUG      
    Serial.println(F("file.close()"));
#endif
    rlen = l-sizeof(short int);
    memcpy(&dataBuff[0], &Buff[sizeof(short int)], rlen);
  
    return 0;
}


int SD_Termo::Write_data_fs(char *_path, uint8_t *dataBuff, int len)
{   int rc=0, i, n, nw;
    unsigned short int crs, nn;
    uint8_t Buff[FS_BUF];

    if((unsigned int)len > (sizeof(Buff) -2 * sizeof(unsigned short int)))
    {
        Serial.printf((PGM_P)F("Error: Writing %d bytes to %d buff\n"),  len +2 * sizeof(unsigned short int), sizeof(Buff));
#if SERIAL_DEBUG      
#endif // SERIAL_DEBUG      

        return 10;
    }

#if SERIAL_DEBUG      
    Serial.printf((PGM_P)F("Writing file: %s %d bytes\n"), _path, len);
#endif // SERIAL_DEBUG      
 //??
 /*  b = FlashFS.begin(AUTOCONNECT_FS_INITIALIZATION);
    if(b == false)
    {   Serial.println("FlashFS.begin failed");
    }
*/

//    File file = FlashFS.open(_path, FILE_WRITE);  //FILE_WRITE
    File file = FlashFS.open(_path, "w");  //FILE_WRITE
    if(!file)
    {  
#if SERIAL_DEBUG      
         Serial.println(F("- failed to open file for writing"));
#endif         
        return 1;
    }

    nn = len;
    n = sizeof(unsigned short int);
    memcpy(&Buff[0],(void *) &nn, n);
    memcpy(&Buff[n],(void *) dataBuff, len);
    len += n;
    crs = 0;
    for(i=0; i<len; i++)
      crs += Buff[i];

    memcpy(&Buff[len],(void *) &crs, sizeof(crs));
    len += sizeof(crs);
/***************************/    
//    for(i=0; i<len; i++)
//            Serial.printf("%02x ",Buff[i]);
//    Serial.printf("\n");
/***************************/    
    nw = file.write((unsigned char *) &Buff[0], len);
    if(nw != len)
        rc = 1;
    file.close();
    return rc;
}

int SD_Termo::Write_ot_fs(void)
{   int rc, n;
    uint8_t Buff[FS_BUF];
    
    n = sizeof(enable_CentralHeating);
    memcpy(&Buff[0],(void *) &enable_CentralHeating, n);
#if SERIAL_DEBUG      
Serial.printf("SD_Termo::Write_ot_fs  enable_CentralHeating %d \n", enable_CentralHeating);
#endif
    memcpy(&Buff[n],(void *) &enable_HotWater, sizeof(enable_HotWater));
    n += sizeof(enable_HotWater);
    memcpy(&Buff[n],(void *) &Tset, sizeof(Tset));
    n += sizeof(Tset);
    memcpy(&Buff[n],(void *) &TdhwSet, sizeof(TdhwSet));
    n += sizeof(TdhwSet);
    memcpy(&Buff[n],(void *) &UDPserver_repot_period, sizeof(UDPserver_repot_period));
    n += sizeof(UDPserver_repot_period);
    memcpy(&Buff[n],(void *) &UDPserver_port, sizeof(UDPserver_port));
    n += sizeof(UDPserver_port);
    memcpy(&Buff[n],(void *) &UseID2, sizeof(UseID2));
    n += sizeof(UseID2);
    memcpy(&Buff[n],(void *) &ID2masterID, sizeof(ID2masterID));
    n += sizeof(ID2masterID);

#if MQTT_USE
    memcpy(&Buff[n],(void *) &useMQTT, sizeof(useMQTT));
    n += sizeof(useMQTT);
    memcpy(&Buff[n],(void *) MQTT_server, sizeof(MQTT_server));
    n += sizeof(MQTT_server);
    memcpy(&Buff[n],(void *) MQTT_user, sizeof(MQTT_user));
    n += sizeof(MQTT_user);
    memcpy(&Buff[n],(void *) MQTT_pwd, sizeof(MQTT_pwd));
    n += sizeof(MQTT_pwd);
    memcpy(&Buff[n],(void *) MQTT_topic, sizeof(MQTT_topic));
    n += sizeof(MQTT_topic);
    memcpy(&Buff[n],(void *) MQTT_devname, sizeof(MQTT_devname));
    n += sizeof(MQTT_devname);

    memcpy(&Buff[n],(void *) &MQTT_interval, sizeof(MQTT_interval));
    n += sizeof(MQTT_interval);

#endif
#if PID_USE
    memcpy(&Buff[n],(void *) &usePID, sizeof(usePID));
    n += sizeof(usePID);
    memcpy(&Buff[n],(void *) &srcTroom , sizeof(srcTroom));
    n += sizeof(srcTroom);
    memcpy(&Buff[n],(void *) &srcText , sizeof(srcText));
    n += sizeof(srcText);
    memcpy(&Buff[n],(void *) &mypid.Kp , sizeof(mypid.Kp));
    n += sizeof(mypid.Kp);
    memcpy(&Buff[n],(void *) &mypid.Kd , sizeof(mypid.Kd));
    n += sizeof(mypid.Kd);
    memcpy(&Buff[n],(void *) &mypid.Ki , sizeof(mypid.Ki));
    n += sizeof(mypid.Ki);
    memcpy(&Buff[n],(void *) &mypid.xTag , sizeof(mypid.xTag));
    n += sizeof(mypid.xTag);
    memcpy(&Buff[n],(void *) &mypid.umax , sizeof(mypid.umax));
    n += sizeof(mypid.umax);
    memcpy(&Buff[n],(void *) &mypid.umin , sizeof(mypid.umin));
    n += sizeof(mypid.umin);
    memcpy(&Buff[n],(void *) &mypid.u0 , sizeof(mypid.u0));
    n += sizeof(mypid.u0);
    memcpy(&Buff[n],(void *) &mypid.y0 , sizeof(mypid.y0));
    n += sizeof(mypid.y0);
    memcpy(&Buff[n],(void *) &mypid.u1 , sizeof(mypid.u1));
    n += sizeof(mypid.u1);
    memcpy(&Buff[n],(void *) &mypid.y1 , sizeof(mypid.y1));
    n += sizeof(mypid.y1);
#endif

    memcpy(&Buff[n],(void *) &CH2_DHW_flag, sizeof(CH2_DHW_flag));
    n += sizeof(CH2_DHW_flag);

#if SERIAL_DEBUG      
    if( n >= sizeof(Buff) )    
         Serial.printf("Error: %s buff size %d, need %d\n", __FUNCTION__,  sizeof(Buff), n);
#endif         
    
   Serial.printf("%s buff size %d, need %d\n", __FUNCTION__,  sizeof(Buff), n);
    rc = Write_data_fs((char *)path, Buff, n);

    return rc;
}

#if OT_DEBUGLOG
 #if defined(ARDUINO_ARCH_ESP8266)
  #define OT_DEBUGLOG_SIZE 8*4   
 #elif defined(ARDUINO_ARCH_ESP32)
  #define OT_DEBUGLOG_SIZE 8*256   
 #endif
 static char OT_DebugLog[OT_DEBUGLOG_SIZE];
#endif //OT_DEBUGLOG

void SD_Termo::init(void)
{
#if OT_DEBUGLOG
  OTlogBuf.Init(OT_DebugLog,OT_DEBUGLOG_SIZE,8);
#endif  
  Bstat.t_I_last =time(nullptr);
  Bstat.sec_h = Bstat.sec_d = 0;
}

 
void SD_Termo::loop(void)
{  int dt;

    if(need_write_f)
    {   
#if SERIAL_DEBUG 
        int rc,  t0;
        t0 = millis();
        rc = Write_ot_fs();
        dt = millis() - t0;
        Serial.printf("Write_fs rc=%d  dt %d ms", rc, dt);
        need_write_f = 0;
#else
         Write_ot_fs();
        need_write_f = 0;
#endif // SERIAL_DEBUG      
     
    }
    
    if(UDPserver_sts)
    {    dt = millis() - UDPserver_t;
         if(dt < UDPserver_repot_period)
         {   return;
         }
        if(TcpUdp_Lsend > 0)
            return;
//  Serial.printf("SD_Termo::loop %li\n",  millis());
        OpenThermInfo();
        UDPserver_t = millis();
    }
    if(TCPserver_sts)
    {    dt = millis() - TCPserver_t;
         if(dt < TCPserver_repot_period)
         {   return;
         }
        if(TcpUdp_Lsend > 0)
            return;
  
        OpenThermInfo();
        TCPserver_t = millis();
    }
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

    memcpy((void *)&msg->Buf[6],(void *)&stsOT,2); 	
    memcpy((void *)&msg->Buf[8],(void *)&BoilerStatus,4); 
    memcpy((void *)&msg->Buf[12],(void *)&BoilerT,4);
    memcpy((void *)&msg->Buf[16],(void *)&RetT,4);
    memcpy((void *)&msg->Buf[20],(void *)&dhw_t,4);
    memcpy((void *)&msg->Buf[24],(void *)&FlameModulation,4);
    memcpy((void *)&msg->Buf[28],(void *)&Pressure,4);
    memcpy((void *)&msg->Buf[32],(void *)&status,4);
    memcpy((void *)&msg->Buf[36],(void *)&t1,4);
    memcpy((void *)&msg->Buf[40],(void *)&t2,4);
    memcpy((void *)&msg->Buf[44],(void *)&rcode[0],4); //todo
    memcpy((void *)&msg->Buf[48],(void *)&rcode[1],4);
    memcpy((void *)&msg->Buf[52],(void *)&rcode[2],4);
    memcpy((void *)&msg->Buf[56],(void *)&rcode[3],4);
    memcpy((void *)&msg->Buf[60],(void *)&rcode[4],4);

    memcpy((void *)&msg->Buf[64],(void *)&Fault,1); 	//b1
      
}
    
//MCMD_GET_OT_INFO
void SD_Termo::callback_Get_OpenThermInfo( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
    short int B_flags;
    Lsend = 6 + 64;
    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
    memcpy((void *)&MsgOut[6],(void *) Mac,6); 

    B_flags = 0;
    if(enable_CentralHeating) B_flags |= 0x01;
    if(enable_HotWater)       B_flags |= 0x02;  
    if(HotWater_present)      B_flags |= 0x10;  
    if(CH2_present)           B_flags |= 0x20;  
    if(Toutside_present)      B_flags |= 0x40;  
    if(Pressure_present)      B_flags |= 0x80; 
         
	 memcpy((void *)&MsgOut[12],(void *) &B_flags,2); 
	 memcpy((void *)&MsgOut[14],(void *) &stsOT,2); 
//    &MsgOut[16]  todo

     memcpy((void *)&MsgOut[18],(void *) &t_lastwork,sizeof(time_t));  //sizeof(time_t) 4 ESP32, 8 ESP8266
     
	 memcpy((void *)&MsgOut[26],(void *) &BoilerStatus,4);  //20=12+8
	 memcpy((void *)&MsgOut[30],(void *) &BoilerT,4); 
	 memcpy((void *)&MsgOut[34],(void *) &RetT,4); 
	 memcpy((void *)&MsgOut[38],(void *) &Tset,4); 
	 memcpy((void *)&MsgOut[42],(void *) &Tset_r,4); 
	 memcpy((void *)&MsgOut[46],(void *) &dhw_t,4); 
	 memcpy((void *)&MsgOut[50],(void *) &FlameModulation,4); 
	 memcpy((void *)&MsgOut[54],(void *) &Pressure,4); 
	 memcpy((void *)&MsgOut[58],(void *) &status,4); 
	 memcpy((void *)&MsgOut[62],(void *) &t1,4); 
	 memcpy((void *)&MsgOut[66],(void *) &t2,4); 
     //70
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
	 memcpy((void *)&MsgOut[24],(void *)&dhw_t, 4); 
	 memcpy((void *)&MsgOut[28],(void *)&FlameModulation, 4); 
	 memcpy((void *)&MsgOut[32],(void *)&Pressure, 4); 
	 memcpy((void *)&MsgOut[36],(void *)&Tset, 4); 
	 memcpy((void *)&MsgOut[40],(void *)&TdhwSet, 4); 
	 memcpy((void *)&MsgOut[44],(void *)&status, 4);  //статус внешних датчиков температуры - (не OT)
	 memcpy((void *)&MsgOut[48],(void *)&t1,4); 
	 memcpy((void *)&MsgOut[52],(void *)&t2,4); 

#if SERIAL_DEBUG      
  Serial.printf("%s, BoilerStatus=%d T1=%f T2=%f\n", __FUNCTION__, BoilerStatus, t1, t2 ); 
#endif
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
    Serial.printf("%s, TestCmd =%d TestId=%i TestPar=%i\n", __FUNCTION__, TestCmd, TestId, TestPar ); 
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
    if(src>= 0 && src <5)
    {
        t_mean[src].add(t);
//    Serial.printf("OnChangeT src =%d, t =%f mean =%f nx=%d\n", src, t, t_mean[src].xmean, t_mean[src].nx); 
    }
#endif
}

#if PID_USE
void SD_Termo::loop_PID(void)
{  static int start = 2;
   static  unsigned long int /* start_t=0,*/ t0=0;
   unsigned long int t;
   float tempindoor=0., tempoutdoor=0.;
   float u;
   int rc;

   int is = 0;
    if(!usePID)
        return;
    t = millis();
    if(t - t0 < 30000)
            return;
    t0 = t;
       
/**********************************************/
    if(start)
    {   if(start == 2)
        {  // start_t = t;
            start = 1;
            mypid.NextTact();
        }

        if(srcTroom < 0 || srcTroom > 4)
        {  is = 0;
        } else {
//  Serial.printf("0 srcTroom =%d, isset=%d xmean=%f nx=%d\n",
//         srcTroom, t_mean[srcTroom].isset,t_mean[srcTroom].xmean, t_mean[srcTroom].nx); 

            if(t_mean[srcTroom].isset == -1)
            {   if(t_mean[srcTroom].nx > 1)
                {    tempindoor = t_mean[srcTroom].xmean / float(t_mean[srcTroom].nx) ; 
                    is |= 1;
                }
            } else {
                tempindoor = t_mean[srcTroom].x;
                is |= 1;
                start = 0; //
            }
            if(srcText < 0 || srcText > 4) 
            {
                is &= ~2;

            } else if(t_mean[srcText].isset == -1) {
                if(t_mean[srcText].nx > 1)
                {    tempoutdoor = t_mean[srcText].xmean / float(t_mean[srcText].nx) ; 
                    is |= 2;
                }
            } else {
                tempoutdoor = t_mean[srcText].x;
                is |= 2;
            }
        }
//        Serial.printf("0 is =%d, tempindoor =%f tempoutdoor=%f\n", is, tempindoor, tempoutdoor ); 
    } else {  // start == 0

        if(srcTroom >= 0 && srcTroom <= 3) //3!!
        {
//    Serial.printf("00 srcTroom =%d, isset=%d xmean=%f nx=%d\n",
//         srcTroom, t_mean[srcTroom].isset,t_mean[srcTroom].xmean, t_mean[srcTroom].nx); 

            if(t_mean[srcTroom].isset >= 0)
            {   tempindoor = t_mean[srcTroom].x;
                is |= 1;
            }
        }
        if(srcText >= 0 && srcText <= 4) //4!!
        {
            if(t_mean[srcText].isset >= 0)
            {   tempoutdoor = t_mean[srcText].x;
                is |= 2;
            }
        }
//    Serial.printf("1 is =%d, tempindoor =%f tempoutdoor=%f\n", is, tempindoor, tempoutdoor ); 
    }
/************ endof  if(start) **********************************/

    if(is & 0x02)
    {   
        u = mypid.u0 + (mypid.u1 - mypid.u0) * (tempoutdoor - mypid.y0) /(mypid.y1 - mypid.y0);

//        Serial.printf("2 u = %f u0 %f u1 %f y0 %f y1 %f\n", u, mypid.u0, mypid.u1, mypid.y0, mypid.y1); 

//      u = u0 + (u1 - u0) * (y - y0)/(y1 - y0);  //40* 4/12

    } else {
        u = mypid.u0;
//        Serial.printf("2a u = %f\n", u); 
    } 

    if(is & 0x01)
    {   rc = mypid.Pid(tempindoor, u);
        if(rc == 1)
        {  Tset = mypid.u;
           need_set_T = 1;  // for OpenTherm
           need_report = 1; // for MQTT
    for(int i=0; i < 8; i++)
    {
//Serial.printf( "%d isset %d nx %d\n", i,t_mean[i].isset,t_mean[i].nx );
         if(t_mean[i].isset == -1 && t_mean[i].nx == 0)
            continue;
        t_mean[i].get();
        if(t_mean[i].nx > 8 || (i == 4 && t_mean[i].isset == 1)) /* 4 - outdoor mqtt */
                t_mean[i].init();
    }

        }
    }
}
#endif

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

 /* return t within limit MIN_CH_TEMP MAX_CH_TEMP*/
float SD_Termo::CHtempLimit(float _t)
{   if(_t < MIN_CH_TEMP) 
        return MIN_CH_TEMP;
    else  if(_t > MAX_CH_TEMP) 
        return MAX_CH_TEMP;
    return _t;
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
/* Считаем интеграл пламени */
void BoilerStatisic::calcIntegral(float flame)
{   time_t now; 
    int dt;
    float d;
    now = time(nullptr);
    dt = now - t_I_last;
    if(dt == 0)
      return;
    t_I_last = now;
    if(flame > 0.f)
    {   d =  flame * dt;
        ModIntegral_h += d;
        ModIntegral_d += d;
    }
    
    sec_h += dt;
    sec_d += dt;
}

