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
#include <AutoConnect.h>

#include "SmartDevice.hpp"
#include "Smart_commands.h"
#include "As_TCP.h"

/************************************/

void setup_tcpudp(SmartDevice *psd);
void loop_udp(int sts);
void loop_tcp(int sts);
int net_callback(U8 *bf, int  bflen, PACKED unsigned char * &bt, int &btlen, int btmax, U8  *(*get_buf) (U16 size));
U8 *esp_get_buf (U16 size);

/************************************/

extern AutoConnectConfig config;
extern AutoConnect portal;
#define BUFSIZE 128

WiFiUDP Udp;
WiFiClient tcp_client;
#if defined(ARDUINO_ARCH_ESP8266)
unsigned int g_port = 6769;
WiFiServer server(g_port);
#elif defined(ARDUINO_ARCH_ESP32)
WiFiServer server;
unsigned int g_port = 6769;
#endif


class SmartDevice *p_sd = NULL;

static char buf_tcpudp_out[BUFSIZE];
static char tcpudp_incomingPacket[BUFSIZE];

int Udp_Lsend=0;
IPAddress Udp_remoteIP;  
IPAddress Tcp_remoteIP;  
int Udp_RemotePort = 0;
int Tcp_RemotePort = 0;
class As_TCP asTCP;

static int sts=0, raz=0;

PACKED unsigned char *Udp_MsgOut=NULL;


//функция, выделяющая память под буфер для отправки по UDP
U8 *esp_get_buf (U16 size)
{
	if(size < BUFSIZE)
		return (U8 *)buf_tcpudp_out;
	else 
		return 0;
}



void setup_tcpudp(SmartDevice *psd)
{
	p_sd = psd;
 	Udp.begin(g_port);
	server.begin(g_port);
}

int tcp_sts = 0;
int t0 = 0;
int t00 = 0;

void loop_tcp(int sts)
{   static int count = 0;
    int rc, len;
	static int nb = 0;
{
	static int ols_sts=-1;
	if(tcp_sts != ols_sts)
	{
		Serial.printf("tcp_sts=%d\n",  tcp_sts);
		ols_sts = tcp_sts;
	}

}	
	switch(tcp_sts)
{
		case 0:// listen for incoming clients)
		// Doc: Gets a client that is connected to the server and has data available for reading.
		//      The connection persists when the returned client object goes out of scope
//			tcp_client = server.available(); //WiFiServer::available(uint8_t*)' is deprecated: Renamed to accept(). [-Wdeprecated-declarations]
			tcp_client = server.accept();
			if(tcp_client)
			 {  t0 = millis();
			 	tcp_client.setTimeout(5);
				nb = 0;
			 	tcp_sts++;
			 } else if (sts == 2 && Udp_Lsend > 0) {
				tcp_sts = 3;
			}
		  break;

		case 1:
			rc = tcp_client.available();
		 	if(rc > 0)
				tcp_sts++;
			else
		  		count++;
			if(!tcp_client.connected() || millis() - t0 > 300000)
			{	Serial.printf("disconnected at %d\n",  count);
				tcp_client.stop();
				tcp_sts  = 0;
			} else {
				delay(1);
			}
		  break;

		  case 2:
			rc = tcp_client.available();
			if(rc > 0)
			{   if(nb == 0)
				{	Serial.printf("%ld tcp client from ", millis());
					Serial.println(tcp_client.remoteIP().toString());
				}

				if(rc != nb)
				{ //	Serial.printf("%d available %d\n", millis(), rc);
					if(rc >= BUFSIZE)
							rc = BUFSIZE-1;
					len = tcp_client.readBytes(tcpudp_incomingPacket, rc);
					Serial.printf("%d readBytes %d\n", millis(),  len);
				   	rc = net_callback((U8 *)tcpudp_incomingPacket, len, Udp_MsgOut, Udp_Lsend, BUFSIZE, esp_get_buf);
					Serial.printf("%d net_callback rc %d, Udp_Lsend=%d\n",  millis(), rc, Udp_Lsend) ;
					if(rc == 0)
					{//	Serial.printf("net_callback rc=%i l=%i\n", rc, Udp_Lsend);	
					 //	Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
						tcp_client.write(Udp_MsgOut, Udp_Lsend);
						Udp_Lsend  = 0;
						nb = 0;
					}
				}

				nb = rc;

				if(!tcp_client.connected())
				{	Serial.printf("disconnected at %d\n",  count);
					tcp_client.stop();
					tcp_sts  = 0;
				} else {
					Serial.printf(".");
					delay(1);
				}		
			} else {
				tcp_sts = 1;
			}
		  break;

			case 3:
				{	Serial.printf("send to IP:");
  					Serial.println(p_sd->tcp_remoteIP.toString());
					Serial.printf(" port %d bytes %d\n", p_sd->TCPserver_port, Udp_Lsend);
					t00 = millis();
					rc = asTCP.connect_0(p_sd->tcp_remoteIP,p_sd->TCPserver_port,5000);
					if(rc == 1)
					{	tcp_sts = 4;
					}  else {
						Udp_Lsend = 0;
						tcp_sts = 0;
					}
				}
		  break;

			case 4:
					rc = asTCP.connect_a();
					if(rc == 0) //wait
					{    //Serial.printf("Wait connection\n");
					} else if(rc == 1) {
					    Serial.printf("Establish a connection\n");
						tcp_sts = 5;
					} else {
				    Serial.printf("Cann't establish a connection\n");
						Udp_Lsend = 0;
						tcp_sts = 0;
						asTCP.closeTCP();

					}
		break;

		  	case 5:

#if defined(ARDUINO_ARCH_ESP8266)
       Serial.printf("todo send sockfd\n");
#elif defined(ARDUINO_ARCH_ESP32)

   				rc = send(asTCP.sockfd, buf_tcpudp_out, Udp_Lsend, 0);
    			Serial.printf("send rc = %d\n");
#endif //
				Udp_Lsend = 0;
				tcp_sts = 6;
				asTCP.read_0();
		break;

		  	case 6:
				rc = asTCP.read_a();
				if( rc == 0) //wait
				{
				} else if (rc == 1) { //read ready
					tcp_sts = 7;
				} else { // -1 - error,  2 timeout
						tcp_sts = 0;
						asTCP.closeTCP();
				}
		break;

		  	case 7:
			 rc =  asTCP.Read(tcpudp_incomingPacket, sizeof(tcpudp_incomingPacket));
			 if(rc > 0)
			 {	tcp_sts = 8;

			 } else {
				tcp_sts = 0;
				asTCP.closeTCP();
			 }

		break;

		  	case 8:
    			Serial.printf("case 8,  time used %d ms\n", millis()-t00);
						tcp_sts = 0;
						asTCP.closeTCP();
		break;
		   
	}

}

void loop_udp(int sts)
{	int rc;
//    if (Udp_Lsend > 0) 
//		Serial.printf("udp send l=%i\n",  Udp_Lsend);		
	
	int packetSize = Udp.parsePacket();
	if (packetSize)
	{
    	//Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    	int len = Udp.read(tcpudp_incomingPacket, BUFSIZE-1);
    	if (len > 0)
    	{
      	tcpudp_incomingPacket[len] = '\0';
    	}
   	rc = net_callback((U8 *)tcpudp_incomingPacket, len, Udp_MsgOut, Udp_Lsend, BUFSIZE, esp_get_buf);
   	if(rc == 0)
	{//	Serial.printf("net_callback rc=%i l=%i\n", rc, Udp_Lsend);	
		Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
		Udp.write(Udp_MsgOut, Udp_Lsend);
		Udp_Lsend  = 0;
		Udp.endPacket();
    }
  } else if (sts && Udp_Lsend > 0) {
		Serial.printf("udp send l=%i bytes to %s port %d\n",  Udp_Lsend, Udp_remoteIP.toString().c_str(), Udp_RemotePort);	
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
static unsigned short lastInd=0;
static unsigned int jj=0xffff, Nlost=0;


  cmd  = *((short int *)&bf[2]);
  par  = *((unsigned short int *)&bf[4]);
  if(par != (lastInd+1)) /* параметр всегда должен инкрементироваться, чтобы обеспечить защиту от повторных посылок */
  {  //if(par == lastInd)
     //        isrep = 1; /* повторная посылка */
	 //else 
	         Nlost++;                 /* потеря данных */  
  }
#if SERIAL_DEBUG
//    Serial.printf("net_callback cmd %i (%x)  par %i (%x)\n",  cmd, cmd, par, par);
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
       p_sd->callback_Echo(len,bf, MsgOut, Lsend,get_buf);
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

			case MCMD_GETDATA:
  	   p_sd->callback_getdata(bf, MsgOut, Lsend, get_buf);
//  	   p_sd->callback_set_tcp_server(bf, MsgOut, Lsend, get_buf);
				break;

      case  MCMD_TESTCMD:
  	   p_sd->callback_testcmd(bf, MsgOut, Lsend, get_buf);
				break;

      case  MCMD_TESTCMDANSWER:
  	   p_sd->callback_testcmdanswer(bf, MsgOut, Lsend, get_buf);
				break;

			case MCMD_SET_UDPSERVER:
			p_sd->callback_set_udp_server(bf, MsgOut, Lsend, get_buf);
			p_sd->udp_remoteIP = Udp.remoteIP();
			Udp_remoteIP = p_sd->udp_remoteIP;  
				break;
			case MCMD_SET_TCPSERVER:
			p_sd->callback_set_tcp_server(bf, MsgOut, Lsend, get_buf);
				break;
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
/**********************************************************************/

/**********************************************************************/

#if defined(ARDUINO_ARCH_ESP8266)
int As_TCP::connect_0(IPAddress ip, uint16_t port, int32_t _timeout)
{
	Serial.printf("TODO %s\n", __FUNCTION__);
    
    return 1;
}

// -1 - error
// 0 wait connect
// 1 connect
// 2 timeout

int As_TCP::connect_a(void)
{   int res;

	Serial.printf("TODO %s\n", __FUNCTION__);
	    return 1;
}

int As_TCP::read_0(void)
{
    t0 = millis();
    return 0;
}

// -1 - error
// 0 wait read
// 1 read ready
// 2 timeout

int As_TCP::read_a(void)
{   int res=-1;
	Serial.printf("TODO %s\n", __FUNCTION__);

	return res;
}


int As_TCP::Read(char bufin[], int len)
{	int rc=-1;
	Serial.printf("TODO %s\n", __FUNCTION__);
	return rc;
}

void As_TCP::closeTCP(void)
{
	Serial.printf("TODO %s\n", __FUNCTION__);

}

/**********************************************************************/
/**********************************************************************/
#elif defined(ARDUINO_ARCH_ESP32)
int As_TCP::connect_0(IPAddress ip, uint16_t port, int32_t _timeout)
{
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        log_e("socket: %d", errno);
        return 0;
    }
    fcntl( sockfd, F_SETFL, fcntl( sockfd, F_GETFL, 0 ) | O_NONBLOCK );

    uint32_t ip_addr = ip;
    struct sockaddr_in serveraddr;
    memset((char *) &serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    memcpy((void *)&serveraddr.sin_addr.s_addr, (const void *)(&ip_addr), 4);
    serveraddr.sin_port = htons(port);
    int res = connect(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr));
    if (res < 0 && errno != EINPROGRESS) {
        Serial.printf("connect on fd %d, errno: %d, \"%s\"", sockfd, errno, strerror(errno));
        closeTCP();
        return 0;
    }
    t0 = millis();
    timeout = _timeout;
    return 1;
}

// -1 - error
// 0 wait connect
// 1 connect
// 2 timeout

int As_TCP::connect_a(void)
{   int res;

    FD_ZERO(&fdset);
    FD_SET(sockfd, &fdset);
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10ms

	res = select(sockfd + 1, nullptr, &fdset, nullptr,  &tv);
    if (res < 0) {
        Serial.printf("select on fd %d, errno: %d, \"%s\"\n", sockfd, errno, strerror(errno));
        closeTCP();
		return -1;
    } else if (res == 0) {
		if(millis()-t0 > timeout)
        {   Serial.printf("connect_a returned due to timeout %d ms for fd %d\n", timeout, sockfd);
			closeTCP();
			return 2;
		}
        return 0;
    } else {
        int sockerr;
        socklen_t len = (socklen_t)sizeof(int);
        res = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &sockerr, &len);

        if (res < 0) {
            Serial.printf("getsockopt on fd %d, errno: %d, \"%s\"\n", sockfd, errno, strerror(errno));
            closeTCP();
			return -2;
        }

        if (sockerr != 0) {
            Serial.printf("socket error on fd %d, errno: %d, \"%s\"\n", sockfd, sockerr, strerror(sockerr));
            closeTCP();
            return -3;
        }
    }

 //   fcntl( sockfd, F_SETFL, fcntl( sockfd, F_GETFL, 0 ) & (~O_NONBLOCK) );
//    clientSocketHandle.reset(new WiFiClientSocketHandle(sockfd));
//    _rxBuffer.reset(new WiFiClientRxBuffer(sockfd));
//    _connected = true;
	    return 1;
}

int As_TCP::read_0(void)
{
    t0 = millis();
    return 0;
}

// -1 - error
// 0 wait read
// 1 read ready
// 2 timeout

int As_TCP::read_a(void)
{   int res;

    FD_ZERO(&fdset);
    FD_SET(sockfd, &fdset);
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10ms

  Serial.printf("read_a select call %d %d\n", millis()-t0, t0 );
	res = select(sockfd + 1,  &fdset, nullptr, nullptr,  &tv);
  Serial.printf("read_a select rc=%d\n",res);
    if (res < 0)
	{   Serial.printf("select on fd %d, errno: %d, \"%s\"\n", sockfd, errno, strerror(errno));
        closeTCP();
		return -1;
    } else if (res == 0) {
		if(millis()-t0 > timeout)
        {   Serial.printf("read_a returned due to timeout %d ms for fd %d\n", timeout, sockfd);
			closeTCP();
			return 2;
		}
        return 0;
	}
	return res;
}


int As_TCP::Read(char bufin[], int len)
{	int rc;
 	socklen_t addr_len;
    struct sockaddr client;  //адрес  клиента

	addr_len = sizeof(struct sockaddr);

	rc = recvfrom(sockfd, bufin, len, 0, &client, &addr_len); 
    if (rc < 0)
	{   Serial.printf("select on fd %d, errno: %d, \"%s\"\n", sockfd, errno, strerror(errno));
        closeTCP();
		return -1;
	}		
	Serial.printf("read  %d bytes\n", rc);
	return rc;
}

void As_TCP::closeTCP(void)
{
	if(sockfd >= 0)
    {	close(sockfd);
		sockfd = -1;
	}

}

#endif //
/**********************************************************************/
