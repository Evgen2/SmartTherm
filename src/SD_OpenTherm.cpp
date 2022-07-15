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

#include "SD_OpenTherm.hpp"
#include "Smart_commands.h"


extern int Udp_Lsend;
extern PACKED unsigned char *Udp_MsgOut;

extern IPAddress Udp_remoteIP;  
extern int Udp_RemotePort;
extern U8 *esp_get_buf (U16 size);

struct Msg1 msg;
static int indcmd = 0;

void SD_Termo::loop(void)
{  int dt;
    if(server_sts == 0)
            return;
    dt = millis() - server_t;
    if(dt < server_repot_period)
        return;
    if(Udp_Lsend > 0)
        return;
//  Serial.printf("SD_Termo::loop %li\n",  millis());
    OpenThermInfo();
    server_t = millis();
}
  
void SD_Termo::OpenThermInfo(void)
{   int i,l;
    unsigned char * MsgOut;
    msg.cmd0 = 0x22;
    msg.cmd  = MCMD_OT_INFO;
    msg.ind = indcmd++;
    for(i=0; i<16*4; i++)
       msg.Buf[i] = i;
    l = 16*4+6;
	
    memcpy((void *)&msg.Buf[0],(void *)&stsOT,4); 
    memcpy((void *)&msg.Buf[4],(void *)&BoilerStatus,4); 
    memcpy((void *)&msg.Buf[8],(void *)&BoilerT,4); 
    memcpy((void *)&msg.Buf[12],(void *)&RetT,4);   
    memcpy((void *)&msg.Buf[16],(void *)&dhw_t,4); 
    memcpy((void *)&msg.Buf[20],(void *)&FlameModulation,4); 
    memcpy((void *)&msg.Buf[24],(void *)&Pressure,4); 
    memcpy((void *)&msg.Buf[28],(void *)&status,4); 
    memcpy((void *)&msg.Buf[32],(void *)&t1,4); 
    memcpy((void *)&msg.Buf[36],(void *)&t2,4); 
    memcpy((void *)&msg.Buf[40],(void *)&rcode[0],4); 
    memcpy((void *)&msg.Buf[44],(void *)&rcode[1],4); 
    memcpy((void *)&msg.Buf[48],(void *)&rcode[2],4); 
    memcpy((void *)&msg.Buf[52],(void *)&rcode[3],4); 
    memcpy((void *)&msg.Buf[56],(void *)&rcode[4],4); 

    memcpy((void *)&msg.Buf[60],(void *)&Fault,1); 

    Udp_Lsend = 6 + l;	
    MsgOut = esp_get_buf(Udp_Lsend);
  	memcpy((void *)&MsgOut[0],(void *)&msg,Udp_Lsend);
      
}
    
 
void SD_Termo::udp_OpenThermInfo( U8 *bf, unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{


}
