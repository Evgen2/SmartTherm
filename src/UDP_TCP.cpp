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

WiFiUDP Udp;
WiFiClient tcp_client;
unsigned int g_port = 6769;

#if defined(ARDUINO_ARCH_ESP8266)
WiFiServer server(g_port);
WiFiClient client2;

#elif defined(ARDUINO_ARCH_ESP32)
WiFiServer server;
#endif


class SmartDevice *p_sd = NULL;

static char buf_tcpudp_out[UDP_TSP_BUFSIZE];
static char tcpudp_incomingPacket[UDP_TSP_BUFSIZE];

int TcpUdp_Lsend=0;
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
	if(size < UDP_TSP_BUFSIZE)
		return (U8 *)buf_tcpudp_out;
	else 
		return 0;
}



void setup_tcpudp(SmartDevice *psd)
{
	p_sd = psd;
 	Udp.begin(g_port);
	server.begin(g_port);
	asTCP.id = 1;
}

int tcp_sts = 0;
int t0 = 0;
int t00 = 0;

void loop_tcp(int sts)
{   static int count = 0;
    int rc, len;
	static int nb = 0;

	static int ols_sts=-1;
	if(tcp_sts != ols_sts)
	{
#if SERIAL_DEBUG
		Serial.printf("tcp_sts=%d\n",  tcp_sts);
#endif		
		ols_sts = tcp_sts;
	}

	
	switch(tcp_sts)
	{
		case 0:// listen for incoming clients)
		//Next tcp_sts: 0/1 (sts=0) | 0/3 (sts=2)
		// Doc: Gets a client that is connected to the server and has data available for reading.
		//      The connection persists when the returned client object goes out of scope
//			tcp_client = server.available(); //WiFiServer::available(uint8_t*)' is deprecated: Renamed to accept(). [-Wdeprecated-declarations]
			tcp_client = server.accept();
			if(tcp_client)
			 {  t0 = millis();
			 	tcp_client.setTimeout(5);
				nb = 0;
			 	tcp_sts++;
			 } else if (sts == 2 && TcpUdp_Lsend > 0) {
				tcp_sts = 3;
			}
		  break;

		case 1:
		//Next tcp_sts: 0/1/2 (sts=0)
			rc = tcp_client.available();
		 	if(rc > 0)
				tcp_sts++;
			else
		  		count++;
			if(!tcp_client.connected() || millis() - t0 > 300000)
			{	
#if SERIAL_DEBUG
				Serial.printf("disconnected at %d\n",  count);
#endif				
				tcp_client.stop();
				tcp_sts  = 0;
			} else {
				delay(1);
			}
		  break;

		  case 2:
			rc = tcp_client.available();
			if(rc > 0)
			{   
#if SERIAL_DEBUG
				if(nb == 0)
				{	Serial.printf("%ld tcp client from %s to port %d ", 
							millis(),tcp_client.remoteIP().toString().c_str(), tcp_client.localPort() );
//					Serial.println(tcp_client.remoteIP().toString());
				}
#endif 
				if(rc != nb)
				{ //	Serial.printf("%d available %d\n", millis(), rc);
					if(rc >= UDP_TSP_BUFSIZE)
							rc = UDP_TSP_BUFSIZE-1;
					len = tcp_client.readBytes(tcpudp_incomingPacket, rc);
#if SERIAL_DEBUG
					if(nb == 0)
						Serial.printf("readBytes %d\n",  len);
					else
					Serial.printf("%ld readBytes %d\n", millis(),  len);
#endif
				   	rc = net_callback((U8 *)tcpudp_incomingPacket, len, Udp_MsgOut, TcpUdp_Lsend, UDP_TSP_BUFSIZE, esp_get_buf);

#if SERIAL_DEBUG
				Serial.printf("%li net_callback rc %d, TcpUdp_Lsend=%d\n",  millis(), rc, TcpUdp_Lsend) ;
#endif				
					if(rc == 0)
					{//	Serial.printf("net_callback rc=%i l=%i\n", rc, TcpUdp_Lsend);	
					 //	Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
						tcp_client.write(Udp_MsgOut, TcpUdp_Lsend);
						TcpUdp_Lsend  = 0;
						nb = 0;
					}
				}

				nb = rc;

				if(!tcp_client.connected())
				{	
#if SERIAL_DEBUG
					Serial.printf("disconnected at %d\n",  count);
#endif					
					tcp_client.stop();
					tcp_sts  = 0;
				} else {
#if SERIAL_DEBUG
//					Serial.printf(".");
#endif					
					delay(1);
				}		
			} else {
				tcp_sts = 1;
			}
		  break;

			case 3:
		//Next tcp_sts: 0/4 (sts=2)
				if(!p_sd->tcp_remoteIP) //Empty IP !!!
				{	tcp_sts = 0;
				} else {
#if SERIAL_DEBUG
					Serial.printf("send to IP: %s", p_sd->tcp_remoteIP.toString().c_str());
//  					Serial.println(p_sd->tcp_remoteIP.toString());
					Serial.printf(" port %d bytes %d\n", p_sd->TCPserver_port, TcpUdp_Lsend);
#endif					
					t00 = millis();
					rc = asTCP.connect_0(p_sd->tcp_remoteIP,p_sd->TCPserver_port,500);
					if(rc == 1)
					{	tcp_sts = 4;
#if SERIAL_DEBUG
						Serial.printf("Ok connect_0 in %ld ms\n", millis()-t00);
#endif						
					}  else {
#if SERIAL_DEBUG
						Serial.printf("Error connect_0 in %ld ms\n", millis()-t00);
#endif						
						TcpUdp_Lsend = 0;
						tcp_sts = 0;
					}
				}
		  break;

			case 4:
		//Next tcp_sts: 0/5 (sts=2)
					rc = asTCP.connect_a();
					if(rc == 0) //wait
					{    //Serial.printf("Wait connection\n");
					} else if(rc == 1) {
#if SERIAL_DEBUG
					    Serial.printf("Establish a connection\n");
#endif						
						tcp_sts = 5;
					} else {
#if SERIAL_DEBUG
				    Serial.printf("Cann't establish a connection\n");
#endif					
						TcpUdp_Lsend = 0;
						tcp_sts = 0;
						asTCP.closeTCP();

					}
		break;

		  	case 5:
		//Next tcp_sts: 6 (sts=2)
#if defined(ARDUINO_ARCH_ESP8266)
	   rc = client2.write(buf_tcpudp_out, TcpUdp_Lsend);
#if SERIAL_DEBUG
		Serial.printf("client2.write rc = %d\n", rc);
	   if (rc > 0)
	   {	int i;
	   		for(i=0; i<TcpUdp_Lsend; i++)
			{
				Serial.printf("%0x ", buf_tcpudp_out[i]);
				if(i%16 == 15)
						Serial.printf("\n");
			}
	   }
#endif		
#elif defined(ARDUINO_ARCH_ESP32)

   				rc = send(asTCP.sockfd, buf_tcpudp_out, TcpUdp_Lsend, 0);
    			Serial.printf("send rc = %d\n", rc);
#endif //
				TcpUdp_Lsend = 0;
				tcp_sts = 6;
				asTCP.read_0();
		break;

		  	case 6:
		//Next tcp_sts: 0/7 (sts=2)
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
		//Next tcp_sts: 0/8 (sts=2)
			 rc =  asTCP.Read(tcpudp_incomingPacket, sizeof(tcpudp_incomingPacket));
			 if(rc > 0)
			 {	tcp_sts = 8;

			 } else {
				tcp_sts = 0;
				asTCP.closeTCP();
			 }

		break;

		  	case 8:
		//Next tcp_sts: 0 (sts=2)
#if SERIAL_DEBUG
    			Serial.printf("case 8,  time used %ld ms\n", millis()-t00);
#endif				
						tcp_sts = 0;
						asTCP.closeTCP();
		break;
		   
	}

}

void loop_udp(int sts)
{	int rc;
//    if (TcpUdp_Lsend > 0) 
//		Serial.printf("udp send l=%i\n",  TcpUdp_Lsend);		
	
	int packetSize = Udp.parsePacket();
	if (packetSize)
	{
    	//Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    	int len = Udp.read(tcpudp_incomingPacket, UDP_TSP_BUFSIZE-1);
    	if (len > 0)
    	{
      	tcpudp_incomingPacket[len] = '\0';
    	}
   	rc = net_callback((U8 *)tcpudp_incomingPacket, len, Udp_MsgOut, TcpUdp_Lsend, UDP_TSP_BUFSIZE, esp_get_buf);
   	if(rc == 0)
	{//	Serial.printf("net_callback rc=%i l=%i\n", rc, TcpUdp_Lsend);	
		Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
		Udp.write(Udp_MsgOut, TcpUdp_Lsend);
		TcpUdp_Lsend  = 0;
		Udp.endPacket();
    }
  } else if (sts && TcpUdp_Lsend > 0) {
#if SERIAL_DEBUG
		Serial.printf("udp send l=%i bytes to %s port %d\n",  TcpUdp_Lsend, Udp_remoteIP.toString().c_str(), Udp_RemotePort);	
#endif		
		Udp.beginPacket( Udp_remoteIP, Udp_RemotePort);
		Udp.write(Udp_MsgOut, TcpUdp_Lsend);
		TcpUdp_Lsend  = 0;
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
  	   p_sd->callback_HandShake(bf, MsgOut, Lsend,get_buf);
			 break;
			
		 case MCMD_ECHO: // эхо
       p_sd->callback_Echo(len,bf, MsgOut, Lsend,get_buf);
			 break;
		 
		 case MCMD_IDENTIFY: // идентификация
       p_sd->callback_Identify(bf, MsgOut, Lsend,get_buf);
			 break;
			case MCMD_GETTIME:
  	   p_sd->callback_gettime(bf, MsgOut, Lsend,get_buf);
				break;
			case MCMD_SETTIME:
  	   p_sd->callback_settime(bf, MsgOut, Lsend, get_buf);
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

			case MCMD_GET_OT_INFO:
		p_sd->callback_Get_OpenThermInfo(bf, MsgOut, Lsend, get_buf);
				break;

			case MCMD_SET_OT_DATA:
		p_sd->callback_Set_OpenThermData(bf, MsgOut, Lsend, get_buf);
				break;
				
#if OT_DEBUGLOG
			case MCMD_OT_DEBUG:
		p_sd->callback_GetOTLog(bf, MsgOut, Lsend, get_buf);
				break;
#endif	

	 default:
#if SERIAL_DEBUG
    Serial.printf("net_callback Unknown cmd %i\n",  cmd);
#endif	
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
int As_TCP::connect_0(IPAddress ip, uint16_t port, int32_t __timeout)
{  int rc;
//	Serial.printf("TODO %s\n", __FUNCTION__);

	client2.setTimeout(_timeout);
	rc = client2.connect(ip, port);
    _timeout = __timeout;

//	Serial.printf("TODO %s\n", __FUNCTION__);
//	Serial.print(ip.toString());
//	Serial.printf(" port %d\n", port);

//	Serial.printf("rc = %d\n", rc);
    return rc;
}

// -1 - error
// 0 wait connect
// 1 connect
// 2 timeout

int As_TCP::connect_a(void)
{   int rc;

//	Serial.printf("TODO %s\n", __FUNCTION__);

	rc = client2.connected();
	
	return rc;
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
{   int  rc;
	rc = client2.available();
	if (rc == 0)
	{
		if(int(millis()-t0) > _timeout)
        {  
#if SERIAL_DEBUG
			 Serial.printf("read_a returned due to timeout %d ms\n", timeout);
#endif			 
			closeTCP();
			rc = 2;
		}
	}

	return rc;
}


int As_TCP::Read(char bufin[], int len)
{	int rc=-1;
#if SERIAL_DEBUG
	Serial.printf("TODO %s\n", __FUNCTION__);
#endif	
	return rc;
}

void As_TCP::closeTCP(void)
{
#if SERIAL_DEBUG
	Serial.printf("TODO %s\n", __FUNCTION__);
#endif	
	client2.stop();

}

/**********************************************************************/
/**********************************************************************/
#elif defined(ARDUINO_ARCH_ESP32)
int As_TCP::connect_0(IPAddress ip, uint16_t port, int32_t __timeout)
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
#if SERIAL_DEBUG
        Serial.printf("(%d) connect on fd %d, errno: %d, \"%s\"", id, sockfd, errno, strerror(errno));
#endif		
        closeTCP();
        return 0;
    }
    _t0 = millis();
	nraz = 0;
    _timeout = __timeout;
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
#if SERIAL_DEBUG
        Serial.printf("(%d) select on fd %d, errno: %d, \"%s\"\n", id, sockfd, errno, strerror(errno));
#endif		
        closeTCP();
		return -1;
    } else if (res == 0) {
		if(millis() - _t0 > _timeout)
        {
#if SERIAL_DEBUG
			   Serial.printf("(%d) connect_a returned due to timeout %d ms for fd %d, nraz %d\n", id, _timeout, sockfd, nraz);
#endif			   
			closeTCP();
			return 2;
		}
		nraz++;
        return 0;
    } else {
        int sockerr;
        socklen_t len = (socklen_t)sizeof(int);
        res = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &sockerr, &len);

        if (res < 0) {
#if SERIAL_DEBUG
            Serial.printf("(%d) getsockopt on fd %d, errno: %d, \"%s\"\n", id, sockfd, errno, strerror(errno));
#endif			
            closeTCP();
			return -2;
        }

        if (sockerr != 0) {
#if SERIAL_DEBUG
            Serial.printf("socket error on fd %d, errno: %d, \"%s\"\n", sockfd, sockerr, strerror(sockerr));
#endif			
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
    _t0 = millis();
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

#if SERIAL_DEBUG
 // Serial.printf("read_a select call %d %d\n", millis()-t0, t0 );
#endif  
	res = select(sockfd + 1,  &fdset, nullptr, nullptr,  &tv);
#if SERIAL_DEBUG
//  Serial.printf("read_a select rc=%d\n",res);
#endif  
    if (res < 0)
	{ 
#if SERIAL_DEBUG
		  Serial.printf("select on fd %d, errno: %d, \"%s\"\n", sockfd, errno, strerror(errno));
#endif		  
        closeTCP();
		return -1;
    } else if (res == 0) {
		if(millis() - _t0 > _timeout)
        {
#if SERIAL_DEBUG
			   Serial.printf("(%d) read_a returned due to timeout %d ms for fd %d\n", id, _timeout, sockfd);
#endif			   
			closeTCP();
			return 2;
		}
        return 0;
	} else  {
  Serial.printf("read_a select rc=%d\n",res);
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
	{ 
#if SERIAL_DEBUG
		  Serial.printf("(%d) select on fd %d, errno: %d, \"%s\"\n", id, sockfd, errno, strerror(errno));
#endif		  
        closeTCP();
		return -1;
	}		
#if SERIAL_DEBUG
	Serial.printf("(%d) read  %d bytes\n", id, rc);
#endif	
	return rc;
}

void As_TCP::closeTCP(void)
{
Serial.printf("(%d) closeTCP\n", id);
	if(sockfd >= 0)
    {	close(sockfd);
		sockfd = -1;
	}

}

#endif //
/**********************************************************************/
