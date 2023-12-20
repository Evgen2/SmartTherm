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

#define FS_BUF 128

const char *path="/smot_par";

int SD_Termo::Read_ot_fs(void)
{  int rc, n, nw;
    uint8_t Buff[FS_BUF];
    rc = 0;


    nw = sizeof(enable_CentralHeating)+sizeof(enable_HotWater) + sizeof(Tset) + sizeof(TdhwSet);
    nw += sizeof(UDPserver_repot_period) + sizeof(UDPserver_port);
    rc = Read_data_fs((char *)path, Buff, nw);
    if(rc)
        return 1;
    Serial.printf("Read =%i bytes\n", nw);

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

    Serial.printf("enable_CentralHeating=%i\n", enable_CentralHeating);
    Serial.printf("enable_HotWater=%i\n", enable_HotWater);
    Serial.printf("Tset=%.1f TdhwSet=%.1f\n", Tset, TdhwSet);
    Serial.printf("UDPserver_repot_period=%d UDPserver_port=%d\n", UDPserver_repot_period, UDPserver_port);

    return 0;
}

int SD_Termo::Read_data_fs(char *_path, uint8_t *dataBuff, int len)
{   int  n, nw, i, l;
    uint8_t Buff[FS_BUF];
    unsigned short int crs, crs_r, nn;

    if((unsigned int)len > sizeof(Buff)-2 * sizeof(unsigned short int))
        return 10;

    Serial.printf("Reading file: %s\r\n", _path);

#if defined(ARDUINO_ARCH_ESP8266)
//    File file = FlashFS.open(_path,"r" );
#else
//    File file = FlashFS.open(_path, FILE_READ );
#endif
    File file = FlashFS.open(_path,"r" );
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        if(file)
        {   file.close();
            Serial.println("file.close()");
        }

        return 1;
    }

    n = file.read((unsigned char *)&nn, sizeof(nn));
    if(n != sizeof(nn))
    {   file.close();
        Serial.printf("file.read rc %i, must be =%i\n",n,sizeof(nn));
        return 3;
    }
    nw = nn;
    n = sizeof(nn);
    memcpy(&Buff[0],(void *) &nn, n);
    nw += sizeof(short int);
    Serial.printf("read %i bytes nn=%i\n",n, nw);
    l = n;
    n = file.read((unsigned char *)&Buff[l], nw);
    if(n != nw)
    {   file.close();
        Serial.printf("file.read rc %i, must be =%i\n",n,nw);
        return 3;
    }
    l += n;

/***************************/    
//    for(i=0; i<l; i++)
//            Serial.printf("%02x ",Buff[i]);
//    Serial.printf("\n");
/***************************/    

    l -= sizeof(crs);
    crs = 0;

    for(i=0; i<l; i++)
      crs += Buff[i];
    crs_r = *((unsigned short int *)&Buff[l]);
    if(crs !=  crs_r )
    {   file.close();
        Serial.printf("crs = %i, must be =%i\n",crs_r,crs);
        return 4;
    }

    file.close();
    
    memcpy(&dataBuff[0], &Buff[sizeof(short int)], l-sizeof(short int));

    return 0;
}

int SD_Termo::Write_data_fs(char *_path, uint8_t *dataBuff, int len)
{   int rc=0, i, n, nw;
    uint8_t Buff[FS_BUF];
    unsigned short int crs, nn;

    if((unsigned int)len > sizeof(Buff)-2 * sizeof(unsigned short int))
        return 10;

    Serial.printf("Writing file: %s %d bytes\r\n", _path, len);
    FlashFS.begin(AUTOCONNECT_FS_INITIALIZATION);
    
//    File file = FlashFS.open(_path, FILE_WRITE);  //FILE_WRITE
    File file = FlashFS.open(_path, "w");  //FILE_WRITE
    if(!file)
    {   Serial.println("- failed to open file for writing");
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

    rc = Write_data_fs((char *)path, Buff, n);

    return rc;
}

#if defined(ARDUINO_ARCH_ESP8266)
#define OT_DEBUGLOG_SIZE 8*4   
#elif defined(ARDUINO_ARCH_ESP32)
#define OT_DEBUGLOG_SIZE 8*256   
#endif
static char OT_DebugLog[OT_DEBUGLOG_SIZE];

void SD_Termo::init(void)
{
  OTlogBuf.Init(OT_DebugLog,OT_DEBUGLOG_SIZE,8);

}

 
void SD_Termo::loop(void)
{  int dt;

    if(need_write_f)
    {   int rc, dt, t0;
        char str[40];
        t0 = millis();
        rc = Write_ot_fs();
        dt = millis() - t0;
        sprintf(str,"\nrc=%d dt = %d\n", rc, dt);
        Serial.printf("Write_fs %d %d", rc, dt);
        need_write_f = 0;
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
  Serial.printf("SD_Termo::loop %li\n",  millis());
        OpenThermInfo();
        TCPserver_t = millis();
    }
}
  
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

void  SD_Termo::callback_getdata( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
   Lsend = 6 + 6 + sizeof(int)*2 + sizeof(float)*7; 

   MsgOut = get_buf(Lsend);

	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&MsgOut[6],(void *) Mac,6); 
	 memcpy((void *)&MsgOut[12],(void *)&BoilerStatus,4); 
	 memcpy((void *)&MsgOut[16],(void *)&BoilerT, 4); 
	 memcpy((void *)&MsgOut[20],(void *)&RetT, 4); 
	 memcpy((void *)&MsgOut[24],(void *)&dhw_t, 4); 
	 memcpy((void *)&MsgOut[28],(void *)&FlameModulation, 4); 
	 memcpy((void *)&MsgOut[32],(void *)&Pressure, 4); 
	 memcpy((void *)&MsgOut[36],(void *)&status, 4);  //статус внешних датчиков температуры - (не OT)
	 memcpy((void *)&MsgOut[40],(void *)&t1,4); 
	 memcpy((void *)&MsgOut[44],(void *)&t2,4); 

    Serial.printf("%s, BoilerStatus=%d T1=%f T2=%f\n", __FUNCTION__, BoilerStatus, t1, t2 ); 

}

void  SD_Termo::callback_testcmd( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
   Lsend = 6; 
   MsgOut = get_buf(Lsend);
	
	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&TestId,(void *)&bf[6],4); 
	 memcpy((void *)&TestStatus,(void *)&bf[6+4],4); 
    TestCmd = 1;
    TestResponse = -1;
    TestStatus = -1;
    Serial.printf("%s, TestCmd =%d TestId=%i TestPar=%i\n", __FUNCTION__, TestCmd, TestId, TestPar ); 
}

void  SD_Termo::callback_testcmdanswer( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
   Lsend = 6 + 4*2; 
   MsgOut = get_buf(Lsend);
	
	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&MsgOut[6],(void *)&TestResponse,4); 
	 memcpy((void *)&MsgOut[6+4],(void *)&TestStatus,4 ); 

}

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
    if(now == t_I_last)
      return;
    dt = now - t_I_last;
    t_I_last = now;
    if(flame > 0.f)
    {   d =  flame * dt;
        ModIntegral_h += d;
        ModIntegral_d += d;
    }
}

