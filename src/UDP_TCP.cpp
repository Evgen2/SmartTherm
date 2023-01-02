/* UDP_TCP.cpp */

#include <time.h>

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

#include <AutoConnect.h>

/************************************/

void setup_udp(SmartDevice *psd);
void loop_udp(void);
int net_callback(U8 *bf, int  bflen, PACKED unsigned char * &bt, int &btlen, int btmax, U8  *(*get_buf) (U16 size));
U8 *esp_get_buf (U16 size);

/************************************/

extern AutoConnectConfig config;
extern AutoConnect portal;
#define BUFSIZE 256

WiFiUDP Udp;
unsigned int g_port = 6769;

class SmartDevice *p_sd = NULL;

static char str_udp_out[BUFSIZE];
static char udp_incomingPacket[BUFSIZE];

int Udp_Lsend=0;
IPAddress Udp_remoteIP;  
int Udp_RemotePort = 0;

static int sts=0, raz=0;

PACKED unsigned char *Udp_MsgOut=NULL;


//функция, выделяющая память под буфер для отправки по UDP
U8 *esp_get_buf (U16 size)
{
	if(size < BUFSIZE)
		return (U8 *)str_udp_out;
	else 
		return 0;
}

#if 0
static unsigned char buf[256];
unsigned char * Msg;
void test(void)
{  int 	raz = 0x1234;
     Msg = (unsigned char *) buf;
  Serial.printf("raz =%x\n", raz);
	*((int *) (&buf[6])) = raz;
  Serial.printf("buf[6-9] =%x %x %x %x\n", buf[6], buf[7], buf[8], buf[9]);
   raz = 0x6789;
//	*(( int *) (&Msg[6])) = raz; //exeption 9
	memcpy((void *)&Msg[6],(void *)&raz,4);

  Serial.printf("buf[6-9] =%x %x %x %x\n", buf[6], buf[7], buf[8], buf[9]);
  Serial.printf("end\n");
}
#endif //0

#pragma pack(1) 
#pragma pack() 


void setup_udp(SmartDevice *psd)
{
	p_sd = psd;
 	Udp.begin(g_port);

}

void loop_udp(void)
{	int rc;
//    if (Udp_Lsend > 0) 
//		Serial.printf("udp send l=%i\n",  Udp_Lsend);		
	
	int packetSize = Udp.parsePacket();
	if (packetSize)
	{
    	//Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    	int len = Udp.read(udp_incomingPacket, BUFSIZE-1);
    	if (len > 0)
    	{
      	udp_incomingPacket[len] = '\0';
    	}
   	rc = net_callback((U8 *)udp_incomingPacket, len, Udp_MsgOut, Udp_Lsend, BUFSIZE, esp_get_buf);
   	if(rc == 0)
	{//	Serial.printf("net_callback rc=%i l=%i\n", rc, Udp_Lsend);	
		Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
		Udp.write(Udp_MsgOut, Udp_Lsend);
		Udp_Lsend  = 0;
		Udp.endPacket();
    }
  } else if (Udp_Lsend > 0) {
//		Serial.printf("udp send l=%i bytes to %s port %d\n",  Udp_Lsend, Udp_remoteIP.toString().c_str(), Udp_RemotePort);	
		Udp.beginPacket( Udp_remoteIP, Udp_RemotePort);
		Udp.write(Udp_MsgOut, Udp_Lsend);
		Udp_Lsend  = 0;
		Udp.endPacket();
  }
}


/* bf - полученный буфер
   bflen - длина буфера
   MsgOut - буфер на отправку
   Lsend - длина буфера на отправку
   btmax - макс длина буфера на отправку
   get_buf - функция, выделяющая память под буфер для отправки
*/
int net_callback(U8 *bf, int  len, PACKED unsigned char * &MsgOut, int &Lsend, int btmax, U8  *(*get_buf) (U16 size))
{  short int cmd;
   unsigned short int par;
   int  isrep=0;
static unsigned short lastInd=0;
static unsigned int jj=0xffff, Nlost=0;


  cmd  = *((short int *)&bf[2]);
  par  = *((unsigned short int *)&bf[4]);
  if(par != (lastInd+1)) /* параметр всегда должен инкрементироваться, чтобы обеспечить защиту от повторных посылок */
  {  if(par == lastInd)
             isrep = 1; /* повторная посылка */
	 else 
	         Nlost++;                 /* потеря данных */  
  }
#if SERIAL_DEBUG
    Serial.printf("net_callback cmd %i  par %i\n",  cmd, par);
#endif
  lastInd = par;
   if(cmd & 0x8000) //тест обмена
   {  Lsend = len;
      if(Lsend > 1400) Lsend = 1400;
      MsgOut = get_buf(Lsend);
  
	memcpy((void *)&MsgOut[0],(void *)bf,6);
  
//	*((PACKED int *) (&MsgOut[6])) = ++raz;
  ++raz;
  memcpy((void *)&MsgOut[6],(void *)&raz,4);
  
	*((PACKED short int *) (&MsgOut[10])) = p_sd->BiosCode;
	*((PACKED short int *) (&MsgOut[12])) = p_sd->Vers;
	*((PACKED short int *) (&MsgOut[14])) = p_sd->SubVers;
	*((PACKED short int *) (&MsgOut[16])) = 0;
	*((PACKED short int *) (&MsgOut[18])) = 0; 

	*((PACKED int *) (&MsgOut[20])) = cmd;
	*((PACKED int *) (&MsgOut[24])) = par;
	*((PACKED int *) (&MsgOut[28])) = jj;
	*((PACKED int *) (&MsgOut[32])) = Nlost;
	memcpy((void *)&MsgOut[36],(void *)&p_sd->BiosDate,12);
	*((PACKED int *) (&MsgOut[48])) = sts;
	*((PACKED int *) (&MsgOut[52])) = lastInd;
	*((PACKED int *) (&MsgOut[56])) = SMARTDEVICE_VERSION;

    MsgOut[60] = jj;
    MsgOut[61] = jj;
    MsgOut[62] = jj;
    MsgOut[63] = jj;
    MsgOut[64] = jj;
    MsgOut[65] = jj;
    MsgOut[66] = jj;
    MsgOut[67] = jj;
    MsgOut[68] = jj;
    MsgOut[69] = jj;
       lastInd = par;
	   
//  Serial.printf("TestT end\n");
      return (0);
   }
   switch(cmd)
		 {  case MCMD_HAND_SHAKE:
  	   p_sd->udp_callback_HandShake(bf, MsgOut, Lsend,get_buf);
			 break;
			
		 case MCMD_ECHO: // эхо
       p_sd->udp_callback_Echo(len,bf, MsgOut, Lsend,get_buf);
			 break;
		 
		 case MCMD_IDENTIFY: // идентификация
       p_sd->udp_callback_Identify(bf, MsgOut, Lsend,get_buf);
			 break;
			case MCMD_GETTIME:
  	   p_sd->udp_callback_gettime(bf, MsgOut, Lsend,get_buf);
				break;
			case MCMD_SETTIME:
  	   p_sd->udp_callback_settime(bf, MsgOut, Lsend, get_buf);
				break;

			case MCMD_SERVER_INFO:
			p_sd->udp_callback_serverinfo(bf, MsgOut, Lsend, get_buf);
			p_sd->remoteIP = Udp.remoteIP();
			Udp_remoteIP = p_sd->remoteIP;  
				break;
#if 0		 
	   case MCMD_GETDATA:
				roz.udp_callback_getdata(bf,isrep, MsgOut, Lsend,get_buf);
	        break;

	   case MCMD_GETINFO:
				roz.udp_callback_getinfo(bf,isrep, MsgOut, Lsend,get_buf);
	        break;
		 
  	  case MCMD_SETKON: /* input: 2 int, output: 1 int*/
  	   roz.udp_callback_setkon(bf,isrep, MsgOut, Lsend,get_buf);
	    break;
			case MCMD_INIT_COUNT:
  	   roz.udp_callback_initcount(bf,isrep, MsgOut, Lsend,get_buf);
				break;
			case MCMD_GETLOG: /* запрос лога */
  	   roz.udp_callback_getlog(bf,isrep, MsgOut, Lsend,get_buf);
	    break;
			case MCMD_LOGON:
  	   roz.udp_callback_logon(bf,isrep, MsgOut, Lsend,get_buf);
				break;
			case  MCMD_SET_PAR:
  	   roz.udp_callback_setpar(bf,isrep, MsgOut, Lsend,get_buf);
				break;
			case  MCMD_GET_PAR:
  	   roz.udp_callback_getpar(bf,isrep, MsgOut, Lsend,get_buf);
				break;
			
			case MCMD_GET_ADC:
  	   roz.udp_callback_getADC(bf,isrep, MsgOut, Lsend,get_buf);
				break;
			case MCMD_GET_DEV_PMS7003:
  	   roz.udp_callback_getPMS7003(bf,isrep, MsgOut, Lsend,get_buf);
				break;
		
	 case MCMD_VIRT_UART:
		   roz.udp_callback_uart(bf,isrep, MsgOut, Lsend,get_buf);
		 break;


	 case MCMD_SET_TEST: /* установить режим тестирования */
        Lsend = 6;
        MsgOut = get_buf(Lsend);

//	   memcpy((void *)&MsgOut[0],(void *)&bf[0],6);
//       memcpy((void *)&lpctest,(void *)&bf[6],sizeof(struct lpc_test));

//			lpcm.sts = LPC_TEST;
//        test_init();
	    break;
	 case MCMD_GETCONFIG:
		   roz.udp_callback_getConfigAux(bf,isrep, MsgOut, Lsend,get_buf);	 
		 break;
	 case MCMD_SETCONFIG:
		   roz.udp_callback_setConfigAux(bf,isrep, MsgOut, Lsend,get_buf);	 
		 break;
    
#endif //0
	 default:
    Serial.printf("net_callback Unknown cmd %i\n",  cmd);
        Lsend = 6+sizeof(int)+sizeof(short int);
        MsgOut = get_buf(Lsend);
	  	memcpy((void *)&MsgOut[0],(void *)&bf[0],6);
		*((PACKED short int *) (&MsgOut[0])) |= 0x8000; /* error: wrong cmd */
		*((PACKED int *) (&MsgOut[6])) = cmd; /* wrong cmd */
	     break;
   }

  return (0);
}
