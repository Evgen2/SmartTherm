/* SmartDevice.cpp */
#include <time.h>
#include <sys/timeb.h>
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

extern int TCPserver_close_on_send;


/*********************************************************************************/
// MCMD_HAND_SHAKE
void SmartDevice::callback_HandShake( U8 *bf,PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int l;
  l = sizeof(HAND_SHAKE_OUT);
	Lsend = l + sizeof(short int)*3; //16
	MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)bf,6);
#if SERIAL_DEBUG 
//  Serial.printf("MCMD_HAND_SHAKE  Lsend=%i\n", Lsend );
#endif
	
  
	if((bf[6+l-1] == 0) && !strncmp((char *)&bf[6], HAND_SHAKE_INP,l))
	{   memcpy(&MsgOut[6], HAND_SHAKE_OUT,l);
	} else {
	    memcpy(&MsgOut[6], HAND_SHAKE_ERR,l);
	}
/*  		 	
  for(int i=0; i<Lsend;i++)
		Serial.printf("%d ", MsgOut[i]);
Serial.printf("\n");
*/
}

//MCMD_HAND_SHAKE
int SmartDevice::servercallback_HandShake( U8 *bf, int len)
{ int l, rc = 0;
#if SERIAL_DEBUG 
//  Serial.printf("servercallback_HandShake len=%d\n", len);
#endif

  if(len >= 16)
  { l = sizeof(HAND_SHAKE_OUT);
    if(strncmp((char *)&bf[6], HAND_SHAKE_OUT,l) == 0)
    {
//        Serial.printf("HAND_SHAKE_OUT detected\n");
        rc = 1;
        TCPserver_rc = MCMD_HAND_SHAKE;
        TCPserver_close_on_send = 0;
    } else {
        TCPserver_close_on_send = 1;
    }
  }

  return rc;
}


//SCMD_GET_HAND_SHAKE
int SmartDevice::server_send_HandShake(unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int l, rc = 0;

    TCPserver_rc = SCMD_GET_HAND_SHAKE;
    TCPserver_sts2 = 1;
#if SERIAL_DEBUG 
//  Serial.printf("server_send_HandShake SCMD_GET_HAND_SHAKE\n");
//  Serial.printf("TCPserver_sts2 = 1\n");
#endif

/*  
    struct Msg1 *msg;
extern int indcmd;
    
#if SERIAL_DEBUG 
  Serial.printf("server_send_HandShake\n");
#endif

    l = strlen(HAND_SHAKE_INP);

    Lsend = 6 + l;	

    MsgOut = get_buf(Lsend);
    msg  = (struct Msg1 *)MsgOut;
    msg->cmd0 = 0x22;
    msg->cmd  = MCMD_HAND_SHAKE;
    msg->ind = indcmd++;
   	memcpy(&MsgOut[6], HAND_SHAKE_INP,l);
    TCPserver_close_on_send = 0;
*/
  return rc;
}

//MD_ECHO
void SmartDevice::callback_Echo( U16 len, U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{		Lsend = len; 
		MsgOut = get_buf(Lsend);
		memcpy((void *)&MsgOut[0],(void *)bf, Lsend);
}

//MD_IDENTIFY
void SmartDevice::callback_Identify( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int l, lp; 

  l = strlen((PGM_P)IDENTIFY_TEXT); 

  lp =  sizeof(int)*6 + 6 + 12 + l;
  Lsend = 6 +  sizeof(short int) + lp;	
  
  MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)bf,6);
  *((PACKED short int *) (&MsgOut[6])) = (short int)lp;
  *((PACKED int *) (&MsgOut[8]))  =  IDENTIFY_TYPE; 
  *((PACKED int *) (&MsgOut[12]))  =  IDENTIFY_CODE;
  *((PACKED int *) (&MsgOut[16]))  =  IdNumber;	
  *((PACKED int *) (&MsgOut[20]))  =  Vers;	
  *((PACKED int *) (&MsgOut[24]))  =  SubVers;	
  *((PACKED int *) (&MsgOut[28]))  =  SubVers1;	
 	memcpy((void *)&MsgOut[32],(void *)BiosDate,12);

	memcpy((void *)&MsgOut[44],(void *)&Mac[0],6);
  memcpy_P((void *)&MsgOut[50],(void *)(PGM_P)IDENTIFY_TEXT, l);
  
//  Serial.printf("IDENTIFY_TEXT 4 l=%i Lsend =%i\n", l, Lsend );
//  Serial.print(IDENTIFY_TEXT);
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
void SmartDevice::callback_gettime( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   int  tm_l;
    struct timeval tv;

	 tm_l = sizeof(time_t); //4 ESP32, 8 ESP8266
   Lsend = 6 + 2 + tm_l + 4; 
   if(tm_l == 4)
      Lsend += 4; 

   MsgOut = get_buf(Lsend);
   time_t now = time(nullptr);
	
    gettimeofday(&tv, NULL);  
//  Serial.println(ctime(&now));
//  Serial.printf("Sizeof timeb %i\n", sizeof(timeb));

	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
	 memcpy((void *)&MsgOut[6],(void *)&tm_l,2); 
   if(tm_l == 4)
   {
	    memcpy((void *)&MsgOut[8],(void *)&tv.tv_sec,tm_l); 
	    memcpy((void *)&MsgOut[12],(void *)&tv.tv_usec,4); 
//  Serial.printf("################## tv  %d %d\n", tv.tv_sec, tv.tv_usec);
   } else {
	    memcpy((void *)&MsgOut[8],(void *)&tv.tv_sec,tm_l); 
	    memcpy((void *)&MsgOut[8+tm_l],(void *)&tv.tv_usec,4); 
   }
}

//MCMD_SETTIME  установить время
void SmartDevice::callback_settime( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   int  tm_l;
	  struct timeb  tb;
    char tzbuf[20];
timeval tv = { 0, 0 };
	 tm_l = sizeof(time_t);
   Lsend = 6; 
   MsgOut = get_buf(Lsend);
	
	 memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

	 memcpy((void *)&tv.tv_sec,(void *)&bf[6], 4); 
	 memcpy((void *)&tv.tv_usec,(void *)&bf[10],4); 
//  Serial.printf("###****################ tv  %d %d\n", tv.tv_sec, tv.tv_usec);

//{  time_t now;
//  now = time(nullptr);
//  Serial.printf("1 %s\n", ctime(&now));
//}  


  settimeofday(&tv, nullptr);

#if SERIAL_DEBUG 
//{  time_t now;
//  now = time(nullptr);
//  Serial.printf("2 %s\n", ctime(&now));
//}  
#endif  
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

#if SERIAL_DEBUG 
//  Serial.printf("callback_set_tcp_server sts=%d remoteIP =%s\n", s, buf);
//  tcp_remoteIP.fromString(buf);
//  Serial.printf("==");
//  Serial.println(tcp_remoteIP); // print the parsed IPAddress 

#endif
  memcpy((void *)&dt, (void *)&bf[30],4); 
  memcpy((void *)&p,(void *)&bf[34],4); 

  TCPserver_sts = s;  /* статус сервера */
  if(s)
    TCPserver_t = millis();
  TCPserver_port = p;  
  TCPserver_report_period = dt;

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
