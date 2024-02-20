/* SmartDevice.hpp */
#ifndef SMARTDEVICE
#define SMARTDEVICE

#include "Smart_Config.h"

#pragma pack(1)


#define MASTER_BIOSCODE   23
#define MASTER_VERSION     0
#define MASTER_SUBVERSION  7
#define MASTER_SUBVERSION1 2
/***************/

#define SMARTDEVICE_VERSION 0 /* esp8266/esp32  */

#define U8   unsigned char
#define U16  unsigned short
#define U32  unsigned int
#define S8   char
#define S16  short
#define S32  int
#define Bool unsigned char;
#define TRUE  1
#define FALSE 0
#define PACKED  /* */

#define FLASH_MARK 0x1AA11BBf
#define FLASH_WRITESIZE (36+4*16)

//макрос для автоматического определения размера статической части класса для записи-чтения во флеш
#define FLASH_WRITESIZE0 ((int)(&this->sts) - (int)(&this->mark))


struct Msg1
{   short int cmd0; //команда
    short int cmd; //команда
    short int ind; //параметр
    unsigned char Buf[128];
};

class SmartDevice
{
 public:
  const  int mark;  /* пометка для определения наличия во флеше  */
  const  int size;  /* размер для чтения-записи-сравнения версий */
/* константы */
  const int BiosCode;     /* код биоса */
  const int Vers;         /* версия */
  const int SubVers;      /* подверсия */
  const int SubVers1;     /* дверсия подверсии */
#if defined(ARDUINO_ARCH_ESP8266)
  static char  BiosDate[12];     /* дата компиляции биоса */
#elif defined(ARDUINO_ARCH_ESP32)
  const char  BiosDate[12];     /* дата компиляции биоса */
#endif
  int IdNumber;            /* номер устройства */
#if defined(ARDUINO_ARCH_ESP8266)
  unsigned char Mac[6];   /* mac - адрес */
  unsigned char  _foo[2];  
#elif defined(ARDUINO_ARCH_ESP32)
  unsigned char Mac[6];   /* mac - адрес */
#endif
  int UDPserver_port;  /* порт сервера */
  int UDPserver_repot_period;  /* периодичность отправки данных серверу, сек */
  int UDPserver_sts;  /* статус сервера */
  long int UDPserver_t; /* время последнего сообщения серверу, следующее через server_repot_period */

  int TCPserver_port;  /* порт сервера */
  int TCPserver_repot_period;  /* периодичность отправки данных серверу, сек */
  int TCPserver_sts;  /* статус сервера */
  long int TCPserver_t; /* время последнего сообщения серверу, следующее через server_repot_period */

	IPAddress udp_remoteIP;  
	IPAddress tcp_remoteIP;  


  const int ReservParam[16];    /* резерв параметры */ 

  int status;
  int sts;                 /* состояние       */  
  int sts_next;            /* состояние на следующий такт  */
  

#if defined(ARDUINO_ARCH_ESP8266)
  SmartDevice(void):mark(FLASH_MARK), size (FLASH_WRITESIZE0), 
               BiosCode(MASTER_BIOSCODE),Vers(MASTER_VERSION), 
               SubVers(MASTER_SUBVERSION), SubVers1(MASTER_SUBVERSION1),
							 ReservParam()
#elif defined(ARDUINO_ARCH_ESP32)
  SmartDevice(void):mark(FLASH_MARK), size (FLASH_WRITESIZE0), 
               BiosCode(MASTER_BIOSCODE),Vers(MASTER_VERSION), 
               SubVers(MASTER_SUBVERSION), 
               SubVers1(MASTER_SUBVERSION1), 
               BiosDate(__DATE__),
							 ReservParam()
#endif
			   
  {  // int i;  
	sts = sts_next = 0;
        UDPserver_port = 0;  
        UDPserver_repot_period = 0;
        UDPserver_sts = 0;
        UDPserver_t = 0;
	TCPserver_port = 0;
	TCPserver_repot_period = 0;
	TCPserver_sts = 0;
	TCPserver_t = 0;

  status = -1;
  }
  void callback_HandShake( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_Echo( U16 len, U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_Identify( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_gettime( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_settime( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_set_udp_server( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_set_tcp_server( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

//  virtual void udp_OpenThermInfo( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
//  {};
  virtual void callback_getdata( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
  {  };

  virtual void callback_testcmd( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
  {  };
  virtual void callback_testcmdanswer( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
  {  };
  virtual void callback_Get_OpenThermInfo( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
  {  };

#if OT_DEBUGLOG
  virtual void callback_GetOTLog( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
  { };
#endif

};
#pragma pack()

 #endif //SMARTDEVICE
