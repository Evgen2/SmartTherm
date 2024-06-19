/* Smart_commands.h */
#ifndef SMART_COMMANDS
#define SMART_COMMANDS

/* команды мастеру */
#define HAND_SHAKE_INP           "TCPiptEsT"
#define HAND_SHAKE_OUT           "ipTCPTeSt"
#define HAND_SHAKE_ERR           "NoOkError"

#define MCMD_HAND_SHAKE  		0x2020
#define MCMD_ECHO        		0x01
#define MCMD_IDENTIFY        		0x80 //запрос у сервера
#define MCMD_INTRODUCESELF     		0x81 //передача данных о себе серверу

#define  MCMD_GETINFO       0x02 //запрос информации
#define  MCMD_VIRT_UART	    0x03 //виртуальный уарт
#define  MCMD_GETCONFIG		0x04 //запрос информации о конфигурации
#define  MCMD_DATA_FROM	    	0x05 //данные от станции
#define  MCMD_DATA_TO	    	0x06 //передать данные/команду станции
#define  MCMD_SETCONFIG		0x07 //установить информацию о конфигурации
#define  MCMD_SETDEVUPTIME      0x08 //установить информацию об аптайме и  количестве просыпаний устройства
#define  MCMD_DATA   	    	0x09 //передать данные (отладка)

#define  MCMD_GETTIME		0x10 // читать/задать время RTC
#define  MCMD_SETTIME		0x11 // читать/задать время RTC
#define  MCMD_GETDATA				0x12 //получить данные
#define  MCMD_TESTCMD				0x13 
#define  MCMD_TESTCMDANSWER			0x14 
#define  MCMD_SET_UDPSERVER 0x20 // задать UDP сервер, порт и время обновления информации
#define  MCMD_OT_INFO	    0x21 //Open Therm info
#define  MCMD_SET_TCPSERVER 0x22 // задать TCP сервер, порт и время обновления информации
#define  MCMD_GET_OT_INFO	0x23 // get Open Therm info
#define  MCMD_SET_OT_DATA	0x24 // set Open Therm data
#define  MCMD_OT_DEBUG  0x30 // set/get Open Therm debug 0/1

#define  MCMD_GETLOG		    0x15 //запрос лога
#define  MCMD_LOGON		    	0x16 // лог вкючить-выключить
#define  MCMD_GET_PAR       0x17 //
#define  MCMD_GET_ADC       0x18 //





#endif //SMART_COMMANDS

