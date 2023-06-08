/* SmartDevice.cpp */
#include <time.h>
//#include <sys\time.h>
#include <sys\timeb.h>
#include <stdlib.h>
#include <string.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
typedef ESP8266WebServer WEBServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
typedef WebServer WEBServer;
#endif

#include "SmartDevice.hpp"
#include "Smart_commands.h"


/*********************************************************************************/
// MCMD_HAND_SHAKE
void SmartDevice::udp_callback_HandShake( U8 *bf,PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int i, l;
  l = sizeof(HAND_SHAKE_OUT);
	Lsend = l + sizeof(short int)*3+1; //17
	MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)bf,6);

  Serial.printf("MCMD_HAND_SHAKE\n");
	for(i=0; i<10;i++)
		Serial.printf("%02x ", bf[6+i]);

	if((bf[6+l-1] == 0) && !strcmp((char *)&bf[6], HAND_SHAKE_INP))
	{   memcpy(&MsgOut[6], HAND_SHAKE_OUT,l);
	} else {
	    memcpy(&MsgOut[6], HAND_SHAKE_ERR,l);
	}		 	
}

//MD_ECHO
void SmartDevice::callback_Echo( U16 len, U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{		Lsend = len; 
		MsgOut = get_buf(Lsend);
		memcpy((void *)&MsgOut[0],(void *)bf, Lsend);
}

//MD_IDENTIFY
void SmartDevice::udp_callback_Identify( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int l; 
	
	l = sizeof(IDENTIFY_TEXT);
  Lsend = 6+sizeof(int)*3+sizeof(short int)+l;	
  MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)bf,6);
  *((PACKED short int *) (&MsgOut[6])) = (short int)(sizeof(int)*3+l);

  *((PACKED  int *) (&MsgOut[8]))  =  IDENTIFY_TYPE;
  *((PACKED  int *) (&MsgOut[12]))  =  IDENTIFY_CODE;
  *((PACKED  int *) (&MsgOut[16]))  =  IdNumber;	
  memcpy((void *)&MsgOut[20],(void *)IDENTIFY_TEXT, l);

  Serial.printf("IDENTIFY_TEXT l=%i Lsend =%i\n", l, Lsend );
  Serial.print(IDENTIFY_TEXT);

}

/*
struct timeb 
{ time_t time;
  unsigned short millitm;
  short int timezone;
  short int dstflag;
 };
*/

//MCMD_GETTIME  прочесть время
void SmartDevice::udp_callback_gettime( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   int  tm_l;

	 tm_l = sizeof(time_t); //4 ESP32, 8 ESP8266
   Lsend = 6 + tm_l + 2; 
   if(tm_l == 4)
      Lsend += 4; 

   MsgOut = get_buf(Lsend);
  time_t now = time(nullptr);
//  Serial.println(ctime(&now));
//  Serial.printf("Sizeof timeb %i\n", sizeof(timeb));

	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&MsgOut[6],(void *)&tm_l,2); 
	 memcpy((void *)&MsgOut[8],(void *)&now,tm_l); 
}

//MCMD_SETTIME  установить время
void SmartDevice::udp_callback_settime( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   int  tm_l;
	  time_t  now;
	  struct timeb  tb;
//	  int i;
    char tzbuf[20];
timeval tv = { 0, 0 };
//timezone tz = { 0, 0 };
//sizeof(struct tm) = 44 > 9*4=36, из-за extra membres	
	 tm_l = sizeof(time_t);
   Lsend = 6; 
   MsgOut = get_buf(Lsend);
	
	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&tb,(void *)&bf[6],sizeof(struct timeb)); 

//for(i=0; i<16;i++)
//		Serial.printf("%02x ", bf[6+i]);
// Serial.printf("\n");

   if(tm_l  == 4)
   {	
       memcpy((void *)&tb,(void *)&bf[6+4],sizeof(struct timeb)); 
       memcpy((void *)&tb.time,(void *)&bf[6],sizeof(time_t));        
    
//    Serial.printf("t=%lx %x %x %x\n", tb.time, tb.millitm, tb.timezone, tb.dstflag);
//    Serial.printf("t=%ld %d %d %d\n", tb.time, tb.millitm, tb.timezone, tb.dstflag);
   }

tv.tv_sec = tb.time;
tv.tv_usec = tb.millitm*1000;
//tz.tz_minuteswest = tb.timezone;
//tz.tz_dsttime = tb.dstflag;
sprintf(tzbuf,"TZ%i", tb.timezone/60);
setenv("TZ", tzbuf, 1); 
tzset();

#if defined(ARDUINO_ARCH_ESP32)
  Serial.printf("t=%ld %d %d %d\n", tb.time, tb.millitm, tb.timezone, tb.dstflag);
#else  
  Serial.printf("t=%lld %d %d %d\n", tb.time, tb.millitm, tb.timezone, tb.dstflag);
#endif  

  now = time(nullptr);
  Serial.printf("1 %s\n", ctime(&now));
 
  Serial.printf("Set time to:");

  // А теперь магические строчки
	//settimeofday(&tv, &tz);
  settimeofday(&tv, nullptr);
  
  now = time(nullptr);
  Serial.printf("2 %s\n", ctime(&now));
}

//MCMD_SET_TCPSERVER
void SmartDevice::callback_set_tcp_server( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int s, dt, p; // i, rc;
//  char tzbuf[20];
  char buf[20];

  Lsend = 6; 
  MsgOut = get_buf(Lsend);
	
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

	memcpy((void *)&s,(void *)&bf[6],4); 
  memcpy((void *)buf,(void *)&bf[10],20); 

  Serial.printf("callback_set_tcp_server sts=%d remoteIP =%s\n", s, buf);
  tcp_remoteIP.fromString(buf);
  Serial.printf("==");
  Serial.println(tcp_remoteIP); // print the parsed IPAddress 

  memcpy((void *)&dt, (void *)&bf[30],4); 
  memcpy((void *)&p,(void *)&bf[34],4); 

  TCPserver_sts = s;  /* статус сервера */
  if(s)
    TCPserver_t = millis();
  TCPserver_port = p;  
  TCPserver_repot_period = dt;

}

//MCMD_SET_UDPSERVER
void SmartDevice::callback_set_udp_server( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int s, dt, p; // i, rc;
//  char tzbuf[20];
extern int Udp_RemotePort;

  Lsend = 6; 
  MsgOut = get_buf(Lsend);
	
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

	memcpy((void *)&s,(void *)&bf[6],4); 
  memcpy((void *)&dt, (void *)&bf[10],4); 
  memcpy((void *)&p,(void *)&bf[14],4); 
  UDPserver_sts = s;
  if(s)
    UDPserver_t = millis();
  UDPserver_port = p;  
  Udp_RemotePort = p;
  UDPserver_repot_period = dt;
}
