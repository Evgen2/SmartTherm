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
#include "esp32/rom/rtc.h"
typedef WebServer WEBServer;
#endif
#include <AutoConnect.h>
#include <AutoConnectFS.h>
extern AutoConnectFS::FS& FlashFS;

#include "OpenTherm.h"
#include "SD_OpenTherm.hpp"
#include "Smart_commands.h"


extern int TcpUdp_Lsend;
extern int TcpServer_Lsend;

extern PACKED unsigned char *Udp_MsgOut;

extern IPAddress Udp_remoteIP;  
extern int Udp_RemotePort;
extern U8 *esp_get_buf (U16 size);
extern U8 *server_get_buf (U16 size);

extern int TCPserver_close_on_send;

//struct Msg1 msg;
int indcmd = 0;

/*******************************/
const int FS_BUF = sizeof(SD_Termo::enable_CentralHeating) + sizeof(SD_Termo::enable_HotWater) + sizeof(SD_Termo::Tset) + sizeof(SD_Termo::TdhwSet) + sizeof(SD_Termo::UDPserver_repot_period) +
                 sizeof(SD_Termo::UDPserver_port) + sizeof(SD_Termo::TCPserver_report_period) + sizeof(SD_Termo::TCPserver_port) + sizeof(SD_Termo::tcp_remoteIP) + sizeof(SD_Termo::Use_remoteTCPserver) + sizeof(SD_Termo::UseID2) +
                 sizeof(SD_Termo::ID2masterID) + sizeof(SD_Termo::CH2_DHW_flag) + sizeof(SD_Termo::UseWinterMode) + sizeof(SD_Termo::Use_OTC) +sizeof(SD_Termo::Use_ID29_DHW_flag) +
                 sizeof(SD_Termo::Immergas_fix_flag) +
                 sizeof(SD_Termo::CH_StartGist) + sizeof(SD_Termo::Use_MaxRelModLevel) + sizeof(SD_Termo::MaxRelModLevelSetting) +

#if RELAY_USE    
                sizeof(SD_Termo::Relay_present) +  sizeof(SD_Termo::Relay_init_sts) +
#endif

#if ST_VERS == 2
                sizeof(SD_Termo::OT_slave_present) +  sizeof(SD_Termo::OT_slave_mode) +
#endif

#if PID_USE
            sizeof(SD_Termo::usePID) + sizeof(SD_Termo::srcTroom) + sizeof(SD_Termo::srcText) + sizeof(SD_Termo::mypid.Kp) + sizeof(SD_Termo::mypid.Kd) +
            sizeof(SD_Termo::mypid.Ki) + sizeof(SD_Termo::mypid.xTag) + sizeof(SD_Termo::umax) + sizeof(SD_Termo::umin) + sizeof(SD_Termo::mypid.u0) +
            sizeof(SD_Termo::mypid.y0) +  sizeof(SD_Termo::mypid.u1)  + sizeof(SD_Termo::mypid.y1) + sizeof(SD_Termo::mypid.Kidiss)
#endif
    ;

#if MQTT_USE
const int FS_BUFMQTT =     
            sizeof(SD_Termo::useMQTT) + sizeof(SD_Termo::MQTT_server) + sizeof(SD_Termo::MQTT_user) + sizeof(SD_Termo::MQTT_pwd) + sizeof(SD_Termo::MQTT_topic) +
            sizeof(SD_Termo::MQTT_devname) + sizeof(SD_Termo::MQTT_interval) + sizeof(SD_Termo::MQTT_port);
#endif

/**^^^******************************/


const char *path="/smot_par";
const char *pathmqtt="/smotmqtt";

int SD_Termo::Read_ot_fs(void)
{  int rc, n, nw;
    uint8_t Buff[FS_BUF];

    rc = Read_data_fs((char *)path, Buff, FS_BUF, nw, 0);
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

    memcpy((void *) &TCPserver_report_period, &Buff[n], sizeof(TCPserver_report_period));
    n += sizeof(TCPserver_report_period);
    memcpy((void *) &TCPserver_port, &Buff[n], sizeof(TCPserver_port));
    n += sizeof(TCPserver_port);
    

    memcpy((void *) &tcp_remoteIP, &Buff[n], sizeof(tcp_remoteIP));
    n += sizeof(tcp_remoteIP);
    memcpy((void *) &Use_remoteTCPserver, &Buff[n], sizeof(Use_remoteTCPserver));
    n += sizeof(Use_remoteTCPserver);

    memcpy((void *) &UseID2, &Buff[n], sizeof(UseID2));
    n += sizeof(UseID2);
    memcpy((void *) &ID2masterID, &Buff[n], sizeof(ID2masterID));
    n += sizeof(ID2masterID);

    memcpy((void *) &CH2_DHW_flag, &Buff[n], sizeof(CH2_DHW_flag));
    n += sizeof(CH2_DHW_flag);
    if(n >= nw) goto END;
    memcpy((void *) &UseWinterMode, &Buff[n], sizeof(UseWinterMode));
    n += sizeof(UseWinterMode);
    if(n >= nw) goto END;
    memcpy((void *) &Use_OTC, &Buff[n], sizeof(Use_OTC));
    n += sizeof(Use_OTC);
    memcpy((void *) &Use_ID29_DHW_flag, &Buff[n], sizeof(Use_ID29_DHW_flag));
    n += sizeof(Use_ID29_DHW_flag);
    memcpy((void *) &Immergas_fix_flag, &Buff[n], sizeof(Immergas_fix_flag));
    n += sizeof(Immergas_fix_flag);
    memcpy((void *) &CH_StartGist, &Buff[n], sizeof(CH_StartGist));
    n += sizeof(CH_StartGist);

    memcpy((void *) &Use_MaxRelModLevel, &Buff[n], sizeof(Use_MaxRelModLevel));
    n += sizeof(Use_MaxRelModLevel);
    memcpy((void *) &MaxRelModLevelSetting, &Buff[n], sizeof(MaxRelModLevelSetting));
    n += sizeof(MaxRelModLevelSetting);

#if RELAY_USE    
    memcpy((void *) &Relay_present, &Buff[n], sizeof(Relay_present));
    n += sizeof(Relay_present);
    memcpy((void *) &Relay_init_sts, &Buff[n], sizeof(Relay_init_sts));
    n += sizeof(Relay_init_sts);
#endif

#if ST_VERS == 2
    memcpy((void *) &OT_slave_present, &Buff[n], sizeof(OT_slave_present));
    n += sizeof(OT_slave_present);
    memcpy((void *) &OT_slave_mode, &Buff[n], sizeof(OT_slave_mode));
    n += sizeof(OT_slave_mode);
#endif

#if PID_USE
    if(n >= nw) goto END;
    memcpy((void *) &usePID, &Buff[n], sizeof(usePID));
    n += sizeof(usePID);
    if(n >= nw) goto END;
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
    TroomTarget = mypid.xTag;
    n += sizeof(mypid.xTag);
    if(n >= nw) goto END;
    memcpy((void *) &umax, &Buff[n], sizeof(umax));
    n += sizeof(umax);
    if(n >= nw) goto END;
    memcpy((void *) &umin, &Buff[n], sizeof(umin));
    n += sizeof(umin);
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
    memcpy((void *) &mypid.Kidiss, &Buff[n], sizeof(mypid.Kidiss));
    n += sizeof(mypid.Kidiss);
    if(n >= nw) goto END;

#endif //PID_USE


END:

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


int SD_Termo::Read_data_fs(char *_path, uint8_t *dataBuff, int len, int &rlen, int mode)
{   int  n, nw, i, l;
    unsigned short int crs, crs_r, nn, v;

    rlen = 0;
#if SERIAL_DEBUG      
    Serial.printf((PGM_P)F("Reading file: %s\n"), _path);
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

/* *config version & length control */
    n = file.read((unsigned char *)&v, sizeof(nn));
   
    if(v != CONFIG_VERSION)
    {   file.close();
        return 2;
    }

    n = file.read((unsigned char *)&v, sizeof(nn));
    if(((mode == 0) && (v != FS_BUF)) ||((mode == 1) && (v != FS_BUFMQTT)) ) 
    {   file.close();
        return 3;
    }

//read 2 byte - length of data
    n = file.read((unsigned char *)&nn, sizeof(nn));
    if(n != sizeof(nn))
    {   file.close();
#if SERIAL_DEBUG      
        Serial.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,sizeof(nn));
#endif        
        return 4;
    }
    if(nn  > len)
    {
        Serial.printf((PGM_P)F("read file Buff size  %d, must be =%d\n"), len, nn);
        return 10;
    }

    crs = nn;
    nw = nn;
    n = file.read((unsigned char *) dataBuff, nw); //read nn bytes of data
    if(n != nw)
    {   file.close();
#if SERIAL_DEBUG      
        Serial.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,nw);
#endif        
        return 5;
    }
    l = n;
    n = file.read((unsigned char *)&crs_r, sizeof(short int));  //read 2 bytes control sum

    for(i=0; i<l; i++)
    {  crs += dataBuff[i];
    }

    if(crs !=  crs_r )
    {   file.close();
#if SERIAL_DEBUG      
        Serial.printf((PGM_P)F("crs = %i, must be =%i\n"),crs_r,crs);
#endif        
        return 6;
    }

    file.close();
#if SERIAL_DEBUG      
    Serial.println(F("file.close()"));
#endif
    rlen = l;
  
    return 0;
}


int SD_Termo::Write_data_fs(char *_path, uint8_t *dataBuff, int len, int mode)
{   int rc=0, i, n, nw;
    unsigned short int crs, v;

#if SERIAL_DEBUG      
    Serial.printf((PGM_P)F("Writing file: %s %d bytes\n"), _path, len);
#endif // SERIAL_DEBUG      

//    File file = FlashFS.open(_path, FILE_WRITE);  //FILE_WRITE
    File file = FlashFS.open(_path, "w");  //FILE_WRITE
    if(!file)
    {  
#if SERIAL_DEBUG      
         Serial.println(F("- failed to open file for writing"));
#endif         
        return 1;
    }
    v = CONFIG_VERSION;
    nw = file.write((unsigned char *) &v, sizeof(unsigned short int));
    v = FS_BUF;
    if(mode == 1)
        v = FS_BUFMQTT;
    n = file.write((unsigned char *) &v, sizeof(unsigned short int));
    nw += n;

    crs = (unsigned short int) len;
    n = file.write((unsigned char *) &crs, sizeof(unsigned short int));
    nw += n;
    for(i=0; i<len; i++)
      crs += dataBuff[i];
    n = file.write((unsigned char *) &dataBuff[0], len);
    if(n != len)
        rc = 1;
    else nw += n;    

    nw += file.write((unsigned char *) &crs, sizeof(unsigned short int));
    if(nw != (len + 4*sizeof(unsigned short int) ) )
        rc = 2;
//    Serial.printf("write nw =%d len=%d rc=%d\n", nw, len, rc);
    file.close();
    return rc;
}


/* 
    при изменении числа записываемых параметров
    не забыть изменить определение FS_BUF и номер CONFIG_VERSION
*/
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
    memcpy(&Buff[n],(void *) &TCPserver_report_period, sizeof(TCPserver_report_period));
    n += sizeof(TCPserver_report_period);
    memcpy(&Buff[n],(void *) &TCPserver_port, sizeof(TCPserver_port));
    n += sizeof(TCPserver_port);
    
    memcpy(&Buff[n],(void *) &tcp_remoteIP, sizeof(tcp_remoteIP));
    n += sizeof(tcp_remoteIP);
    memcpy(&Buff[n],(void *) &Use_remoteTCPserver, sizeof(Use_remoteTCPserver));
    n += sizeof(Use_remoteTCPserver);

    memcpy(&Buff[n],(void *) &UseID2, sizeof(UseID2));
    n += sizeof(UseID2);
    memcpy(&Buff[n],(void *) &ID2masterID, sizeof(ID2masterID));
    n += sizeof(ID2masterID);

    memcpy(&Buff[n],(void *) &CH2_DHW_flag, sizeof(CH2_DHW_flag));
    n += sizeof(CH2_DHW_flag);
    memcpy(&Buff[n],(void *) &UseWinterMode, sizeof(UseWinterMode));
    n += sizeof(UseWinterMode);
    memcpy(&Buff[n],(void *) &Use_OTC, sizeof(Use_OTC));
    n += sizeof(Use_OTC);
    memcpy(&Buff[n],(void *) &Use_ID29_DHW_flag, sizeof(Use_ID29_DHW_flag));
    n += sizeof(Use_ID29_DHW_flag);    
    memcpy(&Buff[n],(void *) &Immergas_fix_flag, sizeof(Immergas_fix_flag));
    n += sizeof(Immergas_fix_flag);    
    memcpy(&Buff[n],(void *) &CH_StartGist , sizeof(CH_StartGist));
    n += sizeof(CH_StartGist);

    memcpy(&Buff[n],(void *) &Use_MaxRelModLevel, sizeof(Use_MaxRelModLevel));
    n += sizeof(Use_MaxRelModLevel);
    memcpy(&Buff[n],(void *) &MaxRelModLevelSetting, sizeof(MaxRelModLevelSetting));
    n += sizeof(MaxRelModLevelSetting);

#if RELAY_USE    
    memcpy(&Buff[n],(void *) &Relay_present, sizeof(Relay_present));
    n += sizeof(Relay_present);
    memcpy(&Buff[n],(void *) &Relay_init_sts, sizeof(Relay_init_sts));
    n += sizeof(Relay_init_sts);
#endif

#if ST_VERS == 2
    memcpy(&Buff[n],(void *) &OT_slave_present, sizeof(OT_slave_present));
    n += sizeof(OT_slave_present);
    memcpy(&Buff[n],(void *) &OT_slave_mode, sizeof(OT_slave_mode));
    n += sizeof(OT_slave_mode);
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
    memcpy(&Buff[n],(void *) &umax , sizeof(umax));
    n += sizeof(umax);
    memcpy(&Buff[n],(void *) &umin , sizeof(umin));
    n += sizeof(umin);
    memcpy(&Buff[n],(void *) &mypid.u0 , sizeof(mypid.u0));
    n += sizeof(mypid.u0);
    memcpy(&Buff[n],(void *) &mypid.y0 , sizeof(mypid.y0));
    n += sizeof(mypid.y0);
    memcpy(&Buff[n],(void *) &mypid.u1 , sizeof(mypid.u1));
    n += sizeof(mypid.u1);
    memcpy(&Buff[n],(void *) &mypid.y1 , sizeof(mypid.y1));
    n += sizeof(mypid.y1);
    memcpy(&Buff[n],(void *) &mypid.Kidiss , sizeof(mypid.Kidiss));
    n += sizeof(mypid.Kidiss);

#endif


#if SERIAL_DEBUG      
    if( n > sizeof(Buff) )    
         Serial.printf("Error: %s buff size %d, need %d\n", __FUNCTION__,  sizeof(Buff), n);
   Serial.printf("%s buff size %d, need %d\n", __FUNCTION__,  sizeof(Buff), n);
#endif         
    
    rc = Write_data_fs((char *)path, Buff, n, 0);

    return rc;
}

int SD_Termo::Read_mqtt_fs(void)
{   int rc, n, nw;
    uint8_t Buff[FS_BUFMQTT];
    uint8_t len;


    rc = Read_data_fs((char *)pathmqtt, Buff, FS_BUFMQTT, nw, 1);
#if SERIAL_DEBUG      
    Serial.printf("Read %s rc %i\n", pathmqtt, rc);
#endif    
    if(rc)
        return 1;
#if MQTT_USE
    n = sizeof(useMQTT);
    memcpy((void *) &useMQTT, &Buff[0], sizeof(useMQTT));

    memcpy((void *) &len, &Buff[n], 1);  n++;
    memcpy((void *) MQTT_server, &Buff[n], len);  n += len;

    memcpy((void *) &len, &Buff[n], 1);  n++;
    memcpy((void *) MQTT_user, &Buff[n], len);  n += len;
    
    memcpy((void *) &len, &Buff[n], 1);  n++;
    memcpy((void *) MQTT_pwd, &Buff[n], len);  n += len;

    memcpy((void *) &len, &Buff[n], 1);  n++;
    memcpy((void *) MQTT_topic, &Buff[n], len);  n += len;

    memcpy((void *) &len, &Buff[n], 1);  n++;
    memcpy((void *) MQTT_devname, &Buff[n], len);  n += len;

    memcpy((void *) &MQTT_interval, &Buff[n], sizeof(MQTT_interval));
    n += sizeof(MQTT_interval);
    {   unsigned short _port;
        memcpy((void *) &_port, &Buff[n], sizeof(MQTT_port));
        MQTT_port = _port;
        n += sizeof(MQTT_port);
    }
#endif
    return 0;
}

int SD_Termo::Write_mqtt_fs(void)
{   int rc, n=0;
    uint8_t Buff[FS_BUFMQTT];
    uint8_t len;

#if MQTT_USE
    memcpy(&Buff[n],(void *) &useMQTT, sizeof(useMQTT));
    n += sizeof(useMQTT);

    len = strlen(MQTT_server)+1;
    memcpy(&Buff[n],(void *) &len, 1); n++;
    memcpy(&Buff[n],(void *) MQTT_server, len); n += len;

    len = strlen(MQTT_user)+1;
    memcpy(&Buff[n],(void *) &len, 1);  n++;
    memcpy(&Buff[n],(void *) MQTT_user, len);   n += len;

    len = strlen(MQTT_pwd)+1; 
    memcpy(&Buff[n],(void *) &len, 1); n++;
    memcpy(&Buff[n],(void *) MQTT_pwd, len);    n += len;

    len = strlen(MQTT_topic)+1; 
    memcpy(&Buff[n],(void *) &len, 1); n++;
    memcpy(&Buff[n],(void *) MQTT_topic, len);  n += len;

    len = strlen(MQTT_devname)+1; 
    memcpy(&Buff[n],(void *) &len, 1); n++;
    memcpy(&Buff[n],(void *) MQTT_devname, len);    n += len;

    memcpy(&Buff[n],(void *) &MQTT_interval, sizeof(MQTT_interval));
    n += sizeof(MQTT_interval);
    memcpy(&Buff[n],(void *) &MQTT_port, sizeof(MQTT_port));
    n += sizeof(MQTT_port);

#endif

    rc = Write_data_fs((char *)pathmqtt, Buff, n, 1);
#if SERIAL_DEBUG      
    Serial.printf("Write %s rc %i\n", pathmqtt, rc);
#endif

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

void SD_Termo::init(int src)
{
#if OT_DEBUGLOG
    if(src == 1)
        OTlogBuf.Init(OT_DebugLog,OT_DEBUGLOG_SIZE,8);
#endif  
  Bstat.t_I_last =time(nullptr);
  Bstat.sec_h = Bstat.sec_d = 0;
#if PID_USE
  if(usePID && !enable_CentralHeating)
  {   usePID = 0;
  }
  if(src != 3)
   _U0start = mypid.u0;
  else
  { mypid.NextTact();
    mypid.dSt.n = mypid.dSt.ind = 0;
  }
// Serial.printf("src %d _U0start ->mypid.u0\n",  src, _U0start);

#endif   
    if(Use_remoteTCPserver)
        TCPserver_sts = 2;  /* статус сервера */
    if(TCPserver_port == 0) 
        TCPserver_port = 8876;
}

 
void SD_Termo::loop(void)
{   int dt;
    extern int WiFists;

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
        if(need_write_f & 0x01)
        {   Write_ot_fs();
            need_write_f &= ~0x01;
        }
        if(need_write_f & 0x02)
        {   Write_mqtt_fs();
            need_write_f &= ~0x02;
        }
#endif // SERIAL_DEBUG      
     
    } else  if(WiFists  == WL_CONNECTED) {
    
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

    /*   backup 
        if(TCPserver_sts)
        {    dt = millis() - TCPserver_t;
            if(dt < TCPserver_report_period)
            {   return;
            }
            if(TcpUdp_Lsend > 0)
                return;
    
            OpenThermInfo();
            TCPserver_t = millis();
        }
    */   
  
        if(TCPserver_sts)
        {   static unsigned long ts0=0;
	        static int _start = 1, nattemps=0;

             switch(TCPserver_sts2)
            {   case 0:
                    if(millis() - ts0 > TCPserver_report_period ||  _start) //todo
                    {   TCPserver_sts2 = 1; 
                    if(_start)
//      Serial.printf("SD_Termo::loop  start%li\n",  millis());
                        _start = 0;
                    }
                        break;
                case 1:
                    TCPserver_rc = 0;
                    Send_to_server_HandShake();
//      Serial.printf("++++++  Send_to_server_HandShake%li\n",  millis());
                    TCPserver_sts2++;
                    ts0 = millis();
//    Serial.printf("SD_Termo::loop Send_to_server_HandShake\n");
                        break;

                case 2:
    /* ждать ответа на Send_to_server_HandShake */  
    /* если есть ответ - переход на след sts, иначе после таймаута переход на паузу  
    */          
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0;
                        nattemps++; 
                        if(nattemps > 10)
                                start_sts = 0; // связь отвалилась, восстанавливать параметры PID не будем
                        ts0 = millis();
                    } else if(TCPserver_rc == MCMD_HAND_SHAKE) {
                        TCPserver_rc = 0;
                        nattemps = 0; 

                        Send_to_server_IdentifySelf();
//      Serial.printf("++++++  Send_to_server_IdentifySelf %li\n",  millis());
                        TCPserver_sts2++;
                        ts0 = millis();
//    Serial.printf("SD_Termo::loop Send_to_server_IdentifySelf\n");
                    }

//  Serial.printf("TCPserver_sts2 %d dt %d\n", TCPserver_sts2,  millis() - ts0);
                        break;

                case 3: //wait answer MCMD_INTRODUCESELF from server 
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0; 
                        ts0 = millis();
                    } else if(TCPserver_rc == MCMD_INTRODUCESELF) {
                        TCPserver_sts2 = 5; //4; 
//      Serial.printf("++++++  Send_to_server_IdentifySelf answer %li\n",  millis());
                        ts0 = millis();
                    }

                        break;

                case 4: //wait SCMD_GET_STS from server ??????
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0; 
                        ts0 = millis();
                    } else if(TCPserver_rc == SCMD_GET_STS) {
                        TCPserver_sts2 = 5; 
                        ts0 = millis();
                    }
                    break;

                case 5:
                    if(millis() - ts0 > TCPserver_report_period) //todo
                    {   TCPserver_sts2 = 6; 
                        //send  CCMD_SEND_STS
//      Serial.printf("SD_Termo::loop  Send_to_server_Sts %li\n",  millis());
                        Send_to_server_Sts();
                        ts0 = millis();
                    } else {
                        static int old_dt = 0;
                        int dt;
                        dt = (millis() - ts0)/ 1000;
                        if(dt != old_dt)
                        { old_dt = dt;
                         //  Serial.printf("dt %d (%d)\n", dt, TCPserver_report_period/1000 );
                        }
                    }
                    break;

                    case 6: //wait answer to CCMD_SEND_STS from server
                    if(millis() - ts0 > 5000) //todo
                    {   TCPserver_sts2 = 0; 
                        ts0 = millis();
                        #if OT_DEBUGLOG
                    } else if(TCPserver_rc == CCMD_SEND_STS_S) {
                        TCPserver_sts2 = 5; 
                        ts0 = millis();
                    } else if(TCPserver_rc == CCMD_SEND_OTLOG_S) {
                        TCPserver_sts2 = 7; 
                        ts0 = millis();
#endif                        
                    } else if(TCPserver_rc == SCMD_GET_HAND_SHAKE) {
                     //   Serial.printf(">>>>>>>>>>>>>>>>>>>>  Сервер хочет HAND_SHAKE\n" );
                        TCPserver_sts2 = 1; //HandShake
                    }
                    break;
#if OT_DEBUGLOG
                                                                    
                    case 7: //send CCMD_SEND_OTLOG_S
                        Send_to_server_OTlog();
                        ts0 = millis();
                        TCPserver_sts2 = 8; 

                    break;

                    case 8: //wait answer to CCMD_SEND_OTLOG_S from server
                    if(millis() - ts0 > 5000) // timeout todo
                    {   TCPserver_sts2 = 0; 
                        ts0 = millis();
                    } else if(TCPserver_rc == SCMD_SEND_OTLOG_C) {
                        TCPserver_sts2 = 5; 
                        ts0 = millis();
                    } else if(TCPserver_rc == CCMD_SEND_OTLOG_S) {
                        TCPserver_sts2 = 7; 
                        ts0 = millis();
                    }
                    break;
#endif // OT_DEBUGLOG
                    
                }
        }
    }
}


//send to remote MCMD_HAND_SHAKE
void SD_Termo::Send_to_server_HandShake(void)
{   int l;
    unsigned char * MsgOut;
    struct Msg1 *msg;
    l = strlen(HAND_SHAKE_INP);

    TcpServer_Lsend = 6 + l;	

    MsgOut = server_get_buf(TcpServer_Lsend);
    msg  = (struct Msg1 *)MsgOut;
    msg->cmd0 = 0x22;
    msg->cmd  = MCMD_HAND_SHAKE;
    msg->ind = indcmd++;
	memcpy(&MsgOut[6], HAND_SHAKE_INP,l);
    TCPserver_close_on_send = 0;
}

//MCMD_INTRODUCESELF MD_IDENTIFY
void SD_Termo::Send_to_server_IdentifySelf(void)
{ int l, lp; 
    unsigned char * MsgOut, ch;
    struct Msg1 *msg;

    l = strlen((PGM_P)IDENTIFY_TEXT); 

    lp =  sizeof(int)*6 + 6 + 12 + l +3;
    TcpServer_Lsend = 6 +  sizeof(short int) + lp;	
  
//    Serial.printf("Send_IdentifySelf l= %d %d %d\n", l, lp, TcpUdp_Lsend);
    MsgOut = server_get_buf(TcpServer_Lsend);
    msg  = (struct Msg1 *)MsgOut;
    msg->cmd0 = 0x22;
    msg->cmd  = MCMD_INTRODUCESELF;
    msg->ind = indcmd++;

    *((PACKED short int *) (&MsgOut[6])) = (short int)lp;
    *((unsigned short int *) (&MsgOut[8]))   =  IDENTIFY_TYPE; 
    *((unsigned short int *) (&MsgOut[10]))  =  IDENTIFY_SUBTYPE; // = ST_VERS
    *((PACKED int *) (&MsgOut[12]))  =  IDENTIFY_CODE;
    *((PACKED int *) (&MsgOut[16]))  =  IdNumber;	
    *((PACKED int *) (&MsgOut[20]))  =  Vers;	
    *((PACKED int *) (&MsgOut[24]))  =  SubVers;	
    *((PACKED int *) (&MsgOut[28]))  =  SubVers1;	
 	memcpy((void *)&MsgOut[32],(void *)BiosDate,12);

	memcpy((void *)&MsgOut[44],(void *)&Mac[0],6);
//    Serial.printf("MAC: %02x %02x %02x %02x %02x %02x\n",Mac[0], Mac[1],Mac[2], Mac[3], Mac[4], Mac[5]);

    memcpy_P((void *)&MsgOut[50],(void *)(PGM_P)IDENTIFY_TEXT, l);

    MsgOut[50+l] =  start_sts;	
    MsgOut[51+l] =  rtc_get_reset_reason(0);	
    MsgOut[52+l] =  rtc_get_reset_reason(1);

//    Serial.printf("start_sts %d %d %d l=%d lsend=%d\n", start_sts, MsgOut[51+l], MsgOut[52+l], l, TcpServer_Lsend );

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

#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&msg->Buf[6],(void *) &stOT,1); 
        stOT = ot_slave_stsOT;
        memcpy((void *)&msg->Buf[7],(void *) &stOT,1); 
    }
#else
    memcpy((void *)&msg->Buf[6],(void *) &stsOT,2); 

#endif

    memcpy((void *)&msg->Buf[8],(void *)&BoilerStatus,4); 
    memcpy((void *)&msg->Buf[12],(void *)&BoilerT,4);
    memcpy((void *)&msg->Buf[16],(void *)&RetT,4);

    if(Use_ID29_DHW_flag)   
        memcpy((void *)&msg->Buf[20],(void *)&Tstorage,4);
    else        
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

//MCMD_GET_CAP
int SD_Termo::callback_Get_Capabilities( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
    short int B_flags, tmp;
    unsigned int B_flags4;
  //  Serial.printf("callback_Get_Capabilities len %d ", len);
    
    Lsend = 6 + 20;
    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
//  Serial.printf("callback_Get_Capabilities len %d ", len);
//  Serial.printf("MAC: %02x %02x %02x %02x %02x %02x\n",Mac[0], Mac[1],Mac[2], Mac[3], Mac[4], Mac[5]);

    memcpy((void *)&MsgOut[6],(void *) Mac,6); 
    B_flags =  CapabilitiesDetected;
    DetectCapabilities();

	memcpy((void *)&MsgOut[12],(void *) &B_flags,2); 
#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&MsgOut[14],(void *) &stOT,1); 
        stOT = ot_slave_stsOT;
        memcpy((void *)&MsgOut[15],(void *) &stOT,1); 
    }
#else
	memcpy((void *)&MsgOut[14],(void *) &stsOT,2); 
#endif


    B_flags4 = 0;

    if(HotWater_present)      B_flags4 |= 0x01;  //from SConfigSMemberIDcode
    if(CH2_present)           B_flags4 |= 0x02;  //from SConfigSMemberIDcode
    if(RetT_present)          B_flags4 |= 0x04;  //OTid_used
    if(Toutside_present)      B_flags4 |= 0x08;  //OTid_used 
    if(Pressure_present)      B_flags4 |= 0x10;  //OTid_used
    if(Dhw_t_present)         B_flags4 |= 0x2000;  //OTid_used

#if  MQTT_USE
     B_flags4 |= 0x100;  //MQTT_defined
     if(useMQTT)
         B_flags4 |= 0x200;  //use MQTT
#endif
#if  PID_USE
     B_flags4 |= 0x400;  //PID_defined
     if(usePID)
         B_flags4 |= 0x800;  //use PID
#endif
#if RELAY_USE
    if(Relay_present)
        B_flags4 |= 0x4000;  
#endif
#if ST_VERS == 2
    if(OT_slave_present)
        B_flags4 |= 0x8000; 
#endif

	memcpy((void *)&MsgOut[16],(void *) &B_flags4,4); 
    tmp = Use_remoteTCPserver;
	memcpy((void *)&MsgOut[20],(void *) &tmp, 2); 
//#define IDENTIFY_SUBTYPE ST_VERS
    tmp = IDENTIFY_SUBTYPE;
	memcpy((void *)&MsgOut[22],(void *) &tmp, 2); 
	memcpy((void *)&MsgOut[24],(void *) &OTmemberCode, 2);     

     return 0;
}

//MCMD_GET_OT_INFO
int SD_Termo::callback_Get_OpenThermInfo( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
    short int B_flags, tmp;
    int rc = 1, tmp4, statDS;

//    Serial.printf("callback_Get_OpenThermInfo len %d ", len);

    if(len != 12)
    { 
        Serial.printf("callback_Get_OpenThermInfo Error: len != 12\n");
        return 0;
    }
	memcpy((void *)&tmp4,(void *)&bf[6],4);
	memcpy((void *)&tmp,(void *)&bf[10],2);
    if(tmp & 0x02)
       TCPserver_report_period = tmp4*1000;

//todo debug !!!
/*
    if(tmp & 0x02)
     TCPserver_sts = 1;
    else 
     TCPserver_sts = 0;
*/
    TCPserver_close_on_send = tmp&0x01;

//    Serial.printf("TCPserver report_period %d close_on_send %d sts %d\n",
//         TCPserver_report_period, TCPserver_close_on_send, TCPserver_sts);

    Lsend = 6 + 72;
    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

 //   Serial.printf("callback_Get_OpenThermInfo Mac %02x %02x %02x %02x %02x %02x\n", Mac[0], Mac[1], Mac[2], Mac[3], Mac[4], Mac[5]);

    B_flags = 0;
    if(enable_CentralHeating) B_flags |= 0x01;
    if(enable_HotWater)       B_flags |= 0x02;  
    if(HotWater_present)      B_flags |= 0x10;  
    if(CH2_present)           B_flags |= 0x20;  
    if(Toutside_present)      B_flags |= 0x40;  
    if(Pressure_present)      B_flags |= 0x80; 
    if(RetT_present)          B_flags |= 0x1000;  //OTid_used
    if(Dhw_t_present)         B_flags |= 0x2000;  //OTid_used
#if  MQTT_USE
     B_flags |= 0x100;  //MQTT_defined
     if(useMQTT)
         B_flags |= 0x200;  //use MQTT
#endif
#if  PID_USE
     B_flags |= 0x400;  //PID_defined
     if(usePID)
         B_flags |= 0x800;  //use PID
#endif
#if RELAY_USE
    if(Relay_present)
        B_flags |= 0x4000;  
#endif
#if ST_VERS == 2
    if(OT_slave_present)
        B_flags |= 0x8000; 
#endif

/*    
    Serial.printf("B_flags %x\n", B_flags);
    Serial.printf("stsOT %x\n", stsOT);
    Serial.printf("t_lastwork %x\n", t_lastwork);
    Serial.printf("BoilerStatus %x\n", BoilerStatus);
*/         
	 memcpy((void *)&MsgOut[6],(void *) &B_flags,2); 

#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&MsgOut[8],(void *) &stOT,1); 
        stOT = ot_slave_stsOT;
        memcpy((void *)&MsgOut[9],(void *) &stOT,1); 
    }
 #else
	 memcpy((void *)&MsgOut[8],(void *) &stsOT,2); 
 #endif
 
     memcpy((void *)&MsgOut[10],(void *) &t_lastwork,sizeof(time_t));  //sizeof(time_t) 4 ESP32, 8 ESP8266
     
{
//     Serial.printf("t_lastwork =  %s ", ctime(&t_lastwork));
//     Serial.printf("  %02x %02x %02x %02x \n", MsgOut[10], MsgOut[11],MsgOut[12], MsgOut[13]);
}

	 memcpy((void *)&MsgOut[14],(void *) &BoilerStatus,4);  //20=12+8
//    Serial.printf("BoilerT %f\n", BoilerT);
//    Serial.printf("RetT %f\n", RetT);

	 memcpy((void *)&MsgOut[18],(void *) &BoilerT,4); 
	 memcpy((void *)&MsgOut[22],(void *) &RetT,4); 
	 memcpy((void *)&MsgOut[26],(void *) &Tset,4); 
	 memcpy((void *)&MsgOut[30],(void *) &Tset_r,4); 
    if(Use_ID29_DHW_flag)   
        memcpy((void *)&MsgOut[34],(void *)&Tstorage,4);
    else        
        memcpy((void *)&MsgOut[34],(void *)&dhw_t,4);

	memcpy((void *)&MsgOut[38],(void *) &TdhwSet,4); 
    memcpy((void *)&MsgOut[42],(void *) &FlameModulation,4); 
	memcpy((void *)&MsgOut[46],(void *) &Pressure,4); 
    statDS = 0;
    if(stsT1 > 0)
	    statDS |= (stsT1&03);
    if(stsT2 > 0)
    statDS |= (stsT2&03)<<8;
    #if RELAY_USE
        if(Relay_sts)
            statDS |= 0x1000;
    #endif
    //  memcpy((void *)&MsgOut[50],(void *) &statDS,4); 
    memcpy((void *)&MsgOut[50],(void *) &statDS,2); 
//todo 52-54     
	memcpy((void *)&MsgOut[54],(void *) &t1,4); 
	memcpy((void *)&MsgOut[58],(void *) &t2,4); 
	memcpy((void *)&MsgOut[62],(void *) &Toutside,4); 
 #if PID_USE
	memcpy((void *)&MsgOut[66],(void *) &tempindoor,4); 
	memcpy((void *)&MsgOut[70],(void *) &tempoutdoor,4); 
	memcpy((void *)&MsgOut[74],(void *) &TroomTarget,4); 
#else
    {   float tmp = 0.f;
	 memcpy((void *)&MsgOut[66],(void *) &tmp,4); 
	 memcpy((void *)&MsgOut[70],(void *) &tmp,4); 
	 memcpy((void *)&MsgOut[74],(void *) &tmp,4); 
    }
#endif

//    Serial.printf("callback_Get_OpenThermInfo rc %d Lsend %d ",rc, Lsend);

     //78
     return rc;
}

//CCMD_SEND_STS send to remote server
void SD_Termo::Send_to_server_Sts(void) 
{   unsigned char * MsgOut;
    Send_to_server_Sts(MsgOut, TcpServer_Lsend, server_get_buf );
}

//CCMD_SEND_STS_S send to remote server
void SD_Termo::Send_to_server_Sts(unsigned char * &MsgOut, int &Lsend, U8 *(*get_buf) (U16 size) )
{   short int B_flags, tmp;
    int rc = 1, tmp4, statDS, l;
    struct Msg1 *msg;

    l = 78+8;
    Lsend = 6 +  l;	
  
    MsgOut = get_buf(Lsend);
    msg  = (struct Msg1 *)MsgOut;

    msg->cmd0 = 0x22;
    msg->cmd  = CCMD_SEND_STS_S;
    msg->ind = indcmd++;

//    memcpy((void *)&msg->Buf[0],(void *) Mac,6); 

//	 ClientId & ClientId_k todo; 

    memcpy((void *)&msg->Buf[0],(void *) &ClientId,4); 
 //   Serial.printf("ClientId  %d ClientId_k %x TCPserver_report_period %d\n", ClientId, ClientId_k);

    B_flags = 0;
    if(enable_CentralHeating) B_flags |= 0x01;
    if(enable_HotWater)       B_flags |= 0x02;  
    if(HotWater_present)      B_flags |= 0x10;  
    if(CH2_present)           B_flags |= 0x20;  
    if(Toutside_present)      B_flags |= 0x40;  
    if(Pressure_present)      B_flags |= 0x80; 
    if(RetT_present)          B_flags |= 0x1000;  //OTid_used
    if(Dhw_t_present)         B_flags |= 0x2000;  //OTid_used
      
#if  MQTT_USE
     B_flags |= 0x100;  //MQTT_defined
     if(useMQTT)
         B_flags |= 0x200;  //use MQTT
#endif
#if  PID_USE
     B_flags |= 0x400;  //PID_defined
     if(usePID)
     B_flags |= 0x800;  //use PID
#endif

#if RELAY_USE
    if(Relay_present)
        B_flags |= 0x4000;  
#endif
#if ST_VERS == 2
    if(OT_slave_present)
        B_flags |= 0x8000; 
#endif

/* todo Buf[6] -> Buf[4] */
    
	 memcpy((void *)&msg->Buf[6],(void *) &B_flags,2); 
#if ST_VERS == 2
    {   char stOT;
        stOT = stsOT;
        memcpy((void *)&msg->Buf[8],(void *) &stOT,1); 
        stOT = ot_slave_stsOT;
        memcpy((void *)&msg->Buf[9],(void *) &stOT,1); 
    }
#else
    memcpy((void *)&msg->Buf[8],(void *) &stsOT,2); 

#endif
   
     memcpy((void *)&msg->Buf[10],(void *) &t_lastwork,sizeof(time_t));  //sizeof(time_t) 4 ESP32, 8 ESP8266
	 memcpy((void *)&msg->Buf[14],(void *) &BoilerStatus,4);  //20=12+8
	 memcpy((void *)&msg->Buf[18],(void *) &BoilerT,4); 
	 memcpy((void *)&msg->Buf[22],(void *) &RetT,4); 
	 memcpy((void *)&msg->Buf[26],(void *) &Tset,4); 
	 memcpy((void *)&msg->Buf[30],(void *) &Tset_r,4); 
    if(Use_ID29_DHW_flag)   
        memcpy((void *)&msg->Buf[34],(void *) &Tstorage,4);
    else        
        memcpy((void *)&msg->Buf[34],(void *) &dhw_t,4);
	 memcpy((void *)&msg->Buf[38],(void *) &TdhwSet,4); 
     memcpy((void *)&msg->Buf[42],(void *) &FlameModulation,4); 
	 memcpy((void *)&msg->Buf[46],(void *) &Pressure,4); 
     statDS = 0;
    if(stsT1 > 0)
        statDS |= (stsT1&03);
    if(stsT2 > 0)
        statDS |= (stsT2&03)<<8;
#if RELAY_USE
    if(Relay_sts)
        statDS |= 0x1000;
#endif
    
    memcpy((void *)&msg->Buf[50],(void *) &statDS,2); 

#if OT_DEBUGLOG
    statDS = OTlogBuf.GetLbuf();
    if(nOTlog < 16) statDS  = 0;
    memcpy((void *)&msg->Buf[52],(void *) &statDS,2); 
#endif    

	 memcpy((void *)&msg->Buf[54],(void *) &t1,4); 
	 memcpy((void *)&msg->Buf[58],(void *) &t2,4); 
	 memcpy((void *)&msg->Buf[62],(void *) &Toutside,4); 
 #if PID_USE
	 memcpy((void *)&msg->Buf[66],(void *) &tempindoor,4); 
	 memcpy((void *)&msg->Buf[70],(void *) &tempoutdoor,4); 
	 memcpy((void *)&msg->Buf[74],(void *) &TroomTarget,4); 
	 memcpy((void *)&msg->Buf[78],(void *) &mypid.InT,4); 
	 memcpy((void *)&msg->Buf[82],(void *) &mypid.ub,4); 
#else
    {   float tmp = 0.f;
	 memcpy((void *)&msg->Buf[66],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[70],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[74],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[78],(void *) &tmp,4); 
	 memcpy((void *)&msg->Buf[82],(void *) &tmp,4); 
    }
#endif
    TCPserver_rc = 0;
    TCPserver_close_on_send = 0; //wait answer
     //78
}

#if OT_DEBUGLOG

//CCMD_SEND_OTLOG_S 
void SD_Termo::Send_to_server_OTlog(void)
{   unsigned char * MsgOut;
    short int  tmp2;
    int rc = 1, tmp4, i, l, lb;
    struct Msg1 *msg;
    unsigned char buf[8];
//    Send_to_server_Sts(MsgOut, TcpServer_Lsend, server_get_buf );

    if(nOT_need_send * OTlogBuf.Litem > (120-6)) nOT_need_send = (120-6)/OTlogBuf.Litem;
    lb = OTlogBuf.GetLbuf();
    if(nOT_need_send > lb) nOT_need_send = lb;
    l = nOT_need_send * OTlogBuf.Litem;

    TcpServer_Lsend = 6 + 6 + l;	
  
    MsgOut =  server_get_buf(TcpServer_Lsend);
    msg  = (struct Msg1 *)MsgOut;

//    Serial.printf("Send_to_server_OTlog %d Lb %d L%d\n",  nOT_need_send, OTlogBuf.GetLbuf(), nOTlog );
 
    msg->cmd0 = 0x22;
    msg->cmd  = CCMD_SEND_OTLOG_S;

    msg->ind = indcmd++;

    memcpy((void *)&msg->Buf[0],(void *) &ClientId,4); 
    if(lb > 255) lb = 255;
    tmp2 = nOT_need_send | (lb<<8);

    memcpy((void *)&msg->Buf[4],(void *) &tmp2,2); 
    OTlogBuf.StartRead();
    for(i=0;i<nOT_need_send;i++)
    {  // OTlogBuf.Get(buf);
        OTlogBuf.Read(buf);
        memcpy((void *)&msg->Buf[6+i*OTlogBuf.Litem],(void *)buf,OTlogBuf.Litem); 
//        Serial.printf("buf %02x %02x %02x %02x %02x %02x %02x %02x %02x \n",
//                buf[0],buf[1],buf[2],buf[3], buf[4],buf[5],buf[6],buf[7]);
    }
    TCPserver_rc = 0;
    TCPserver_close_on_send = 0; //wait answer
}

//SCMD_SEND_OTLOG_C = CCMD_SEND_OTLOG_S answer
int SD_Termo::server_answerOTLog( U8 *bf, int len)
{   unsigned char * MsgOut;
    unsigned short int tmp2;
    int rc = 0;
    nOTsend += nOT_need_send;
    if(len == 8)
    {   memcpy((void *)&tmp2,(void *)&bf[6],2);
        if(tmp2 > 16) tmp2 = 16;
        nOT_need_send = tmp2; 
        OTlogBuf.EndRead(); //Освобождаем буфер
//        Serial.printf(" get answer SCMD_SEND_OTLOG_C len=%d, nOT_need_send =%d\n", tmp2, nOT_need_send);
        if(nOT_need_send > 0)          
        {   TCPserver_rc = CCMD_SEND_OTLOG_S;
            TCPserver_close_on_send = 0; //wait answer
            rc = 1; //not call asTCPserver.closeTCP() after net_ServerCallback()

        } else {
            TCPserver_rc = SCMD_SEND_OTLOG_C;
            TCPserver_close_on_send = 1; //not wait answer
        }
    }
    return rc;
}
#endif //OT_DEBUGLOG

//MCMD_INTRODUCESELF answer
int SD_Termo::server_answer_IdentifySelf( U8 *bf, int len)
{   int tmp4;
    if(len < 12)
        return -1;
        TCPserver_rc = MCMD_INTRODUCESELF;
        memcpy((void *)&ClientId,(void *)&bf[6],4);
	memcpy((void *)&ClientId_k,(void *)&bf[10],4);
	memcpy((void *)&tmp4,(void *)&bf[14],4);
    TCPserver_report_period = tmp4*1000;
//    Serial.printf("******* server_answer_IdentifySelf ClientId %d ClientId_k %x TCPserver_report_period %d\n", 
//                    ClientId, ClientId_k, TCPserver_report_period);
//    Serial.printf("******* %li\n",  millis());

    return 0;
}

//CCMD_SEND_STS_S answer from remote server
// see also callback_Set_State()
int SD_Termo::servercallback_send_Sts_answ( U8 *bf, int len)
{   int tmp4, rc=0;
    short int remote_cmd;
    float v, vT;
    int isChange = 0;

//  Serial.printf("##### servercallback_send_Sts_answ len %d\n", len);
    TCPserver_rc = CCMD_SEND_STS_S;
	memcpy((void *)&tmp4,(void *)&bf[6],4);
    TCPserver_report_period = tmp4*1000;
    memcpy((void *)&remote_cmd,(void *)&bf[10],2);
    TCPserver_close_on_send = 1; //not wait answer

    if(len == 6+4*4+2*2)
    {  
        if(remote_cmd == 1)
        {
            short int B_flags_toSet;
            float Tset_toSet;    
            float TroomTarget_toSet;
            float TdhwSet_toSet;
            memcpy((void *)&B_flags_toSet,(void *)&bf[12],2);
            memcpy((void *)&vT, (void *)&bf[14],4);
            memcpy((void *)&v,  (void *)&bf[18],4);
            memcpy((void *)&TdhwSet_toSet,      (void *)&bf[22],4);
//  Serial.printf("servercallback_send_Sts_answ TCPserver_report_period  %d\n", TCPserver_report_period);
//    Serial.printf("B_flags_toSet %x Tset_toSet %f TroomTarget_toSet %f TdhwSet_toSet %f\n", B_flags_toSet, vT, v, TdhwSet_toSet );
//    Serial.printf("B_flags_toSet %x \n", B_flags_toSet);

//B_flags_toSet todo
#if RELAY_USE    
        if(Relay_present)
        {  int r = 0;
        if(B_flags_toSet & 0x1000)
                r = 1;
            RelayOnOff(r);
        }
#endif

            vT = CHtempLimit(vT);
            TdhwSet_toSet = CHtempLimit(TdhwSet_toSet);

#if  PID_USE
            v =  RoomtempLimit(v);
            TroomTarget_toSet = v;

            if(usePID)
            {   if(mypid.xTag !=  TroomTarget_toSet)
                {   set_new_PID_setpoint(TroomTarget_toSet, 2); 
//   Serial.printf("**** servercallback_send_Sts_answ: TroomTarget = %f xTag = %f\n", TroomTarget, mypid.xTag);
                    
                    isChange = 1;
                }
            } else {
                if(vT != Tset)
                { Tset = vT;
                    need_set_T = 1;
                    isChange = 1;
                } 
            }
#else
            if(vT != Tset)
            { Tset = vT;
                need_set_T = 1;
                isChange = 1;
            } 
#endif
            if(TdhwSet_toSet != TdhwSet)
            {   TdhwSet = TdhwSet_toSet;
                need_set_dhwT = 1;
                isChange = 1;
            } 

            if(isChange)
                need_write_f = 1;  //need write changes to FS
        } 
    } else if(len == 6+4*3+2) {
        if(remote_cmd == 0x10)
        {   float _It, _U0;
            start_sts = 0;  
            memcpy((void *)&_It, (void *)&bf[12],4);
            memcpy((void *)&_U0, (void *)&bf[16],4);
//todo             
//   Serial.printf("**** servercallback_send_Sts_answ: _It = %f _U0 = %f\n", _It, _U0);
            if(_U0 != 0.f)
                    _U0start = _U0;
             mypid.InT = _It;
             InTstartset = 1;       
        }
    } else if(len == 6+4*2+2) {
#if OT_DEBUGLOG
        if(remote_cmd == 0x02)
        {
            memcpy((void *)&tmp4,(void *)&bf[12],4);
            if(tmp4 > 16) tmp4 = 16;
            nOT_need_send = tmp4; 
            if(nOT_need_send > 0)          
            {    TCPserver_rc = CCMD_SEND_OTLOG_S;
                TCPserver_close_on_send = 0; // wait answer
                rc = 1;
            }
        }
#endif    

    }

    return rc;
}


//SCMD_GET_STS answer  to remote server
int SD_Termo::servercallback_Get_Sts( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   short int B_flags, tmp;
    int rc = 1, tmp4, statDS, l;
    struct Msg1 *msg;

    Serial.printf("!!!!!!!!servercallback_Get_Sts len %d\n", len);
    if(len != 10)
    { 
        Serial.printf("Error: len != 10\n");
        return 0;
    }
    TCPserver_rc = SCMD_GET_STS;
	memcpy((void *)&tmp4,(void *)&bf[6],4);
    TCPserver_report_period = tmp4*1000;
    Serial.printf("!!!!!!!!TCPserver_report_period  %d\n", TCPserver_report_period);

    Send_to_server_Sts( MsgOut, Lsend, get_buf); 
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); // msg->cmd  = SCMD_GET_ST;
      return 1; 
};

//MCMD_SET_OT_DATA:
// set
// enable_CentralHeating
// enable_HotWater
// Tset or mypid.xTag if PID used and enabled
// 20 bytes in, 6 bytes out
void SD_Termo::callback_Set_OpenThermData( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{
    short int B_flags;
    float v, vT;
    bool flag;
    int isChange = 0;

    Lsend = 6;
    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

	memcpy((void *)&B_flags,(void *)&bf[6],2);
    if(B_flags & 0x01)
            flag = true;
    else
            flag  = false;
    if(flag != enable_CentralHeating)
    {   enable_CentralHeating = flag;
        isChange = 1;
    }

    if(HotWater_present) 
    {   if(B_flags & 0x02)
            flag = true;
        else
            flag = false;

        if(flag != enable_HotWater)
        {   enable_HotWater = flag;
            isChange = 1;
        }
    }

	memcpy((void *)&v,(void *)&bf[6+2],4); //Tset
    vT = CHtempLimit(v);

#if  PID_USE
    float roomSetpointT;
	memcpy((void *)&v,(void *)&bf[6+6],4); //roomSetpointT
    if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
    else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    roomSetpointT = v;

    if(usePID)
    {   if(mypid.xTag != roomSetpointT)
        {   set_new_PID_setpoint(roomSetpointT, 3); 
//   Serial.printf("**** callback_Set_OpenThermData: TroomTarget = %f xTag = %f\n", TroomTarget, mypid.xTag);
            isChange = 1;
        }
    } else {
      if(vT != Tset)
      { Tset = vT;
        need_set_T = 1;
        isChange = 1;
      } 
    }
#else
      if(vT != Tset)
      { Tset = vT;
        need_set_T = 1;
        isChange = 1;
      } 
#endif
	memcpy((void *)&v,(void *)&bf[6+10],4); //TdhwSet
    vT = CHtempLimit(v);
    if(vT != TdhwSet)
    { TdhwSet = vT;
        need_set_dhwT = 1;
        isChange = 1;
      } 


    if(isChange)
        need_write_f = 1;  //need write changes to FS

}

//ACMD_SET_STATE_C
// see also servercallback_send_Sts_answ
void SD_Termo::callback_Set_State( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{   short int B_flags;
    float v, vT;
    bool flag;
    int isChange = 0;

    Lsend = 6;
    MsgOut = get_buf(Lsend);
	memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 

	memcpy((void *)&B_flags,(void *)&bf[6],2);
    if(B_flags & 0x01)
            flag = true;
    else
            flag  = false;
    if(flag != enable_CentralHeating)
    {   enable_CentralHeating = flag;
        isChange = 1;
    }

    if(HotWater_present) 
    {   if(B_flags & 0x02)
        flag = true;
        else
            flag = false;

        if(flag != enable_HotWater)
        {   enable_HotWater = flag;
            isChange = 1;
        }
    }

#if RELAY_USE
    if(Relay_present)
    {  int r = 0;
       if(B_flags & 0x1000)
            r = 1;
        RelayOnOff(r);
    }
#endif    

	memcpy((void *)&v,(void *)&bf[6+2],4); //Tset
    vT = CHtempLimit(v);

#if  PID_USE
    float roomSetpointT;
	memcpy((void *)&v,(void *)&bf[6+6],4); //roomSetpointT
    if(usePID == 1)
    {   if(v <  MIN_ROOM_TEMP) v =  MIN_ROOM_TEMP;
        else if(v > MAX_ROOM_TEMP) v = MAX_ROOM_TEMP;
    }
    roomSetpointT = v;

    if(usePID)
    {   if(mypid.xTag != roomSetpointT)
        {   set_new_PID_setpoint(roomSetpointT, 4);
//   Serial.printf("**** callback_Set_State: TroomTarget = %f xTag = %f\n", TroomTarget, mypid.xTag);
            isChange = 1;
        }
    } else {
      if(vT != Tset)
      { Tset = vT;
        need_set_T = 1;
        isChange = 1;
      } 
    }
#else
      if(vT != Tset)
      { Tset = vT;
        need_set_T = 1;
        isChange = 1;
      } 
#endif
	memcpy((void *)&v,(void *)&bf[6+10],4); //TdhwSet
    vT = CHtempLimit(v);
 // Serial.printf("%s, v=%f vT=%f TdhwSet=%f\n", __FUNCTION__, v, vT, TdhwSet); 
    if(vT != TdhwSet)
    { TdhwSet = vT;
        need_set_dhwT = 1;
        isChange = 1;
    } 


    if(isChange)
        need_write_f = 1;  //need write changes to FS

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

    if(Use_ID29_DHW_flag)   
        memcpy((void *)&MsgOut[24],(void *)&Tstorage, 4); 
    else        
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

//MCMD_SET_TCPSERVER
void SD_Termo::callback_set_tcp_server( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size))
{ int s, dt, p, ischange = 0; // i, rc;
//  char tzbuf[20];
  char buf[20];
  IPAddress  ip;

  Lsend = 6; 
  MsgOut = get_buf(Lsend);
	
    memcpy((void *)&MsgOut[0],(void *)&bf[0],6); 
    if(!Use_remoteTCPserver)
        return;

	memcpy((void *)&s,(void *)&bf[6],4); 
    memcpy((void *)buf,(void *)&bf[10],20); 

//  Serial.printf("callback_set_tcp_server sts=%d remoteIP =%s\n", s, buf);

    ip.fromString(buf);
    if(tcp_remoteIP != ip)
    {   ischange |= 0x01;
        tcp_remoteIP = ip;
    }

//  Serial.printf("tcp_remoteIP = %s TCPserver_sts =%d\n",tcp_remoteIP.toString().c_str(), TCPserver_sts); 

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

    if(TCPserver_port != p)
    {   ischange |= 0x02;
        TCPserver_port = p;  
    }
  

    if(TCPserver_report_period != dt)
    {   ischange |= 0x04;
        TCPserver_report_period = dt;
    }
  
    if(ischange)
        need_write_f = 1;

  //Serial.printf("need_write_f %d %x TCPserver_port %d sts=%d TCPserver_report_period =%d\n", 
  //      need_write_f,  ischange, TCPserver_port, TCPserver_sts, TCPserver_report_period);

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
//    Serial.printf("%s, TestCmd =%d TestId=%i TestPar=%i\n", __FUNCTION__, TestCmd, TestId, TestPar ); 
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
    if(src>= 0 && src <= MAX_PID_SRC)
    {
        t_mean[src].add(t);
//   if(src ==2)
//     Serial.printf("OnChangeT 2, %li t =%f\n", millis(), t); 
//    Serial.printf("OnChangeT src =%d, t =%f mean =%f nx=%d\n", src, t, t_mean[src].xmean, t_mean[src].nx); 
    }
#endif
}

#if PID_USE


#endif // PID_USE

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

/* return t within limit MIN_ROOM_TEMP MAX_ROOM_TEMP */
float SD_Termo::RoomtempLimit(float _t)
{   
#if PID_USE
    if(usePID == 1)
    {   
        if(_t < MIN_ROOM_TEMP) 
            return MIN_ROOM_TEMP;
        else  if(_t > MAX_ROOM_TEMP) 
            return MAX_ROOM_TEMP;
    }
#endif        
    return _t;
}


void SD_Termo::DetectCapabilities(void)
{
extern OpenTherm ot;
    if(CapabilitiesDetected  == 1)
    {   int count, countok;
            ot.Get_OTid_count(OpenThermMessageID::CHPressure, count, countok);
            if(countok > 1)
                Pressure_present = true; 
            else
                Pressure_present = false; 
            ot.Get_OTid_count(OpenThermMessageID::Toutside, count, countok);
            if(countok > 1)
                Toutside_present = true;                 
            else
                Toutside_present  = false; 
            ot.Get_OTid_count(OpenThermMessageID::Tret, count, countok);
            if(countok > 1)
                RetT_present = true;                 
            else
                RetT_present  = false; 

            ot.Get_OTid_count(OpenThermMessageID::Tstorage, count, countok); //ID 29
            if(countok > 1)
                Tstorage_present = true;                 
            else
                Tstorage_present = false;

            ot.Get_OTid_count(OpenThermMessageID::Tdhw, count, countok); //ID 26
            if(countok > 1)
                Dhw_t_present = true;                 
            else
                Dhw_t_present = false;

            ot.Get_OTid_count(OpenThermMessageID::MaxRelModLevelSetting, count, countok); //ID 14
            if(countok > 1)
                MaxRelModLevel_present = true;                 
            else
                MaxRelModLevel_present = false;

            ot.Get_OTid_count(OpenThermMessageID::RemoteRequest, count, countok); //ID 4
            if(countok > 1)
                RemoteRequest_present = true;                 
            else
                RemoteRequest_present = false;

    } else  if(CapabilitiesDetected  == 2) {
        if(ot.OTid_used(OpenThermMessageID::CHPressure))
                Pressure_present = true;
        else
                Pressure_present = false;
        if(ot.OTid_used(OpenThermMessageID::Toutside))
                Toutside_present = true;
        else
                Toutside_present  = false;

        if(ot.OTid_used(OpenThermMessageID::Tret))
                RetT_present = true;
        else
                RetT_present  = false; 

        if(ot.OTid_used(OpenThermMessageID::Tstorage))
                Tstorage_present = true;
        else
                Tstorage_present  = false;

        if(ot.OTid_used(OpenThermMessageID::Tdhw))
                Dhw_t_present = true;
        else
                Dhw_t_present  = false;

        if(ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting))
                MaxRelModLevel_present = true;
        else
                MaxRelModLevel_present  = false;

        if(ot.OTid_used(OpenThermMessageID::RemoteRequest))
                RemoteRequest_present = true;
        else
                RemoteRequest_present  = false;

    }

//  Serial.printf("**** DetectCapabilities CapabilitiesDetected %d:\n", CapabilitiesDetected) ;
//    Serial.printf("Pressure_present %d  Toutside_present %d RetT_present %d:\n", 
//                Pressure_present, Toutside_present, RetT_present  ) ;
//    Serial.printf("MaxRelModLevel_present %d  \n", ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting)); 
                
}

void SD_Termo::OnOpenThermRestore(void)
{
extern OpenTherm ot;
extern OpenThermID OT_ids[N_OT_NIDS];

#if  PID_USE
    if(enable_CentralHeating_real)
#else 
    if(SmOT.enable_CentralHeating)
#endif
        need_set_T  = 4; // if request fail, i.e. with errors in  sendind data we need to set T multiple times
    if(enable_HotWater) 
            need_set_dhwT = 2;                   
    if(enable_CentralHeating2)
            need_set_T2  = 2; // if request fail, i.e. with errors in  sendind data we need to set T multiple times


    if(ot.OTid_used(OpenThermMessageID::MaxRelModLevelSetting))
    {   need_set_MaxRelModLevel = 2;
    }
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

/* считаем число включений горячей воды */
void BoilerStatisic::calcN_HW(int newSts)
{
    if(newSts) // включение
    {
//        NflameOn++;
//        NflameOn_h++;
//        NflameOn_day++;
        t_HW_on = time(nullptr);
    } else {  //Выключение
        t_HW_off = time(nullptr);
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

