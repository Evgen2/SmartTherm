/* SD_Config.cpp */

#include <AutoConnect.h>
#include <AutoConnectFS.h>
extern AutoConnectFS::FS& FlashFS;

#include "SD_OpenTherm.hpp"

/*******************************/
const int FS_BUF = sizeof(SD_Termo::enable_CentralHeating) + sizeof(SD_Termo::enable_HotWater) + sizeof(SD_Termo::Tset) + sizeof(SD_Termo::TdhwSet) + sizeof(SD_Termo::UDPserver_repot_period) +
                 sizeof(SD_Termo::UDPserver_port) + sizeof(SD_Termo::TCPserver_report_period) + sizeof(SD_Termo::TCPserver_port) + sizeof(SD_Termo::tcp_remoteIP) + sizeof(SD_Termo::Use_remoteTCPserver) +
                 sizeof(SD_Termo::UseID2) + sizeof(SD_Termo::ID2masterID) + sizeof(SD_Termo::CH2_DHW_flag) + sizeof(SD_Termo::UseWinterMode) + sizeof(SD_Termo::Use_OTC) +
                 sizeof(SD_Termo::Use_ID29_DHW_flag) + sizeof(SD_Termo::Immergas_fix_flag) + sizeof(SD_Termo::Kotitonttu_fix_flag) + 
                 sizeof(SD_Termo::CH_StartGist) + sizeof(SD_Termo::Use_MaxRelModLevel) + sizeof(SD_Termo::MaxRelModLevelSetting) +

#if RELAY_USE    
                sizeof(SD_Termo::Relay_present) +  sizeof(SD_Termo::Relay_init_sts) +
#endif

#if ST_VERS == 2
                sizeof(SD_Termo::OT_slave_present) +  sizeof(SD_Termo::OT_slave_mode) +
#endif

#if PID_USE
            sizeof(SD_Termo::usePID)   + sizeof(SD_Termo::srcTroom)   + sizeof(SD_Termo::srcText)  + sizeof(SD_Termo::mypid.Kp)     + sizeof(SD_Termo::mypid.Kd) +
            sizeof(SD_Termo::mypid.Ki) + sizeof(SD_Termo::mypid.xTag) + sizeof(SD_Termo::umax)     + sizeof(SD_Termo::umin)         + sizeof(SD_Termo::mypid.u0) +
            sizeof(SD_Termo::mypid.y0) + sizeof(SD_Termo::mypid.u1)   + sizeof(SD_Termo::mypid.y1) + sizeof(SD_Termo::mypid.Kidiss) + sizeof(SD_Termo::mypid.Ku) +
            sizeof(SD_Termo::mypid.x0) + sizeof(int) * 2 //reserved 
            + sizeof(SD_Termo::useCPU_freq) + sizeof(SD_Termo::PID_PWMperiod) + sizeof(SD_Termo::MinCHtemp) + sizeof(SD_Termo::usePIDPWM)
            + sizeof(byte) * 3 + sizeof(int) * 14 //reserved
#else            
            + sizeof(SD_Termo::useCPU_freq) + sizeof(SD_Termo::MinCHtemp) + sizeof(int) * 15 //reserved 
#endif
 
    ;

#if MQTT_USE
const int FS_BUFMQTT =     
            sizeof(SD_Termo::useMQTT) + sizeof(SD_Termo::MQTT_server) + sizeof(SD_Termo::MQTT_user) + sizeof(SD_Termo::MQTT_pwd) + sizeof(SD_Termo::MQTT_topic) +
            sizeof(SD_Termo::MQTT_devname) + sizeof(SD_Termo::MQTT_interval) + sizeof(SD_Termo::MQTT_port);
#endif

#define ReadFromBuff(X, dataBuff, n ) do { \
    memcpy((void *)&X, &dataBuff[n], sizeof(X)); \
    n += sizeof(X); \
} while (0)

#define WriteToBuff(X, dataBuff, n ) do { \
    memcpy( &dataBuff[n], (void *)&X, sizeof(X)); \
    n += sizeof(X); \
} while (0)

/**^^^******************************/

const char *path="/smot_par";
const char *pathmqtt="/smotmqtt";
const char *path_rc="/smot_rc";
const char *path_sensors="/sensors";


int SD_Termo::Read_ot_fs(void)
{  int rc, n, nw;
    uint8_t Buff[FS_BUF];

    rc = Read_data_fs((char *)path, Buff, FS_BUF, nw, 0);
    if(rc)
        return 1;
#if SERIAL_DEBUG      
    Serial_db.printf((PGM_P)F("Read %i bytes\n"), nw);
#endif    
    n = 0;
    ReadFromBuff(enable_CentralHeating,Buff,n);
    ReadFromBuff(enable_HotWater,Buff,n);
    ReadFromBuff(Tset,Buff,n);
    ReadFromBuff(TdhwSet,Buff,n);
    ReadFromBuff(UDPserver_repot_period,Buff,n);
    ReadFromBuff(UDPserver_port,Buff,n);
    ReadFromBuff(TCPserver_report_period,Buff,n);
    ReadFromBuff(TCPserver_port,Buff,n);
    ReadFromBuff(tcp_remoteIP,Buff,n);
    ReadFromBuff(Use_remoteTCPserver,Buff,n);
    ReadFromBuff(UseID2,Buff,n);
    ReadFromBuff(ID2masterID,Buff,n);
    ReadFromBuff(CH2_DHW_flag,Buff,n);
    ReadFromBuff(UseWinterMode, Buff,n);
    ReadFromBuff(Use_OTC,Buff,n);
    ReadFromBuff(Use_ID29_DHW_flag,Buff,n);
    ReadFromBuff(Immergas_fix_flag,Buff,n);
    ReadFromBuff(Kotitonttu_fix_flag,Buff,n);
    ReadFromBuff(CH_StartGist,Buff,n);
    ReadFromBuff(Use_MaxRelModLevel,Buff,n);
    ReadFromBuff(MaxRelModLevelSetting,Buff,n);
#if RELAY_USE    
    ReadFromBuff(Relay_present, Buff,n);
    ReadFromBuff(Relay_init_sts,Buff,n);
#endif

#if ST_VERS == 2
    ReadFromBuff(OT_slave_present,Buff,n);
    ReadFromBuff(OT_slave_mode,Buff,n);
#endif

#if PID_USE
    if(n >= nw) goto END;
    ReadFromBuff(usePID,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(srcTroom,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(srcText,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.Kp,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.Kd,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.Ki,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.xTag,Buff,n);
    TroomTarget = mypid.xTag;
    if(n >= nw) goto END;
    ReadFromBuff(umax,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(umin,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.u0,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.y0,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.u1,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.y1,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.Kidiss,Buff,n);
    if(n >= nw) goto END;

    ReadFromBuff(mypid.Ku,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(mypid.x0,Buff,n);
    if(n >= nw) goto END;
    n += sizeof(int) * 2; //reserved
    if(n >= nw) goto END;

    ReadFromBuff(useCPU_freq,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(PID_PWMperiod,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(MinCHtemp,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(usePIDPWM,Buff,n);
    if(n >= nw) goto END;
// sizeof(byte) * 3 + sizeof(int) * 14 //reserved
#else    
    ReadFromBuff(useCPU_freq,Buff,n);
    if(n >= nw) goto END;
    ReadFromBuff(MinCHtemp,Buff,n);

#endif //PID_USE

    if(MinCHtemp < MIN_CH_TEMP)
        MinCHtemp  = MIN_CH_TEMP;

END:

#if SERIAL_DEBUG      
    if(n != nw)
        Serial_db.printf((PGM_P)F("Warning:read %d bytes, use %d\n"), nw, n);

    Serial_db.printf((PGM_P)F("enable_CentralHeating=%i\n"), enable_CentralHeating);
    Serial_db.printf((PGM_P)F("enable_HotWater=%i\n"), enable_HotWater);
    Serial_db.printf((PGM_P)F("Tset=%.1f TdhwSet=%.1f\n"), Tset, TdhwSet);

#if MQTT_USE
    Serial_db.printf((PGM_P)F("useMQTT=%i\n"), useMQTT);
#endif
#endif // SERIAL_DEBUG      

    return 0;
}

/* 
    при изменении числа записываемых параметров
    не забыть изменить определение FS_BUF и номер CONFIG_VERSION
*/
int SD_Termo::Write_ot_fs(void)
{   int rc, n;
    uint8_t Buff[FS_BUF];

    n = 0;
    WriteToBuff(enable_CentralHeating,Buff,n);
    WriteToBuff(enable_HotWater,Buff,n);
    WriteToBuff(Tset,Buff,n);
    WriteToBuff(TdhwSet,Buff,n);
    WriteToBuff(UDPserver_repot_period,Buff,n);
    WriteToBuff(UDPserver_port,Buff,n);
    WriteToBuff(TCPserver_report_period,Buff,n);
    WriteToBuff(TCPserver_port,Buff,n);
    WriteToBuff(tcp_remoteIP,Buff,n);
    WriteToBuff(Use_remoteTCPserver,Buff,n);
    WriteToBuff(UseID2,Buff,n);
    WriteToBuff(ID2masterID,Buff,n);
    WriteToBuff(CH2_DHW_flag,Buff,n);
    WriteToBuff(UseWinterMode, Buff,n);
    WriteToBuff(Use_OTC,Buff,n);
    WriteToBuff(Use_ID29_DHW_flag,Buff,n);
    WriteToBuff(Immergas_fix_flag,Buff,n);
    WriteToBuff(Kotitonttu_fix_flag,Buff,n);
    WriteToBuff(CH_StartGist,Buff,n);
    WriteToBuff(Use_MaxRelModLevel,Buff,n);
    WriteToBuff(MaxRelModLevelSetting,Buff,n);
#if RELAY_USE    
    WriteToBuff(Relay_present, Buff,n);
    WriteToBuff(Relay_init_sts,Buff,n);
#endif

#if ST_VERS == 2
    WriteToBuff(OT_slave_present,Buff,n);
    WriteToBuff(OT_slave_mode,Buff,n);
#endif

#if PID_USE
    WriteToBuff(usePID,Buff,n);
    WriteToBuff(srcTroom,Buff,n);
    WriteToBuff(srcText,Buff,n);
    WriteToBuff(mypid.Kp,Buff,n);
    WriteToBuff(mypid.Kd,Buff,n);
    WriteToBuff(mypid.Ki,Buff,n);
    WriteToBuff(mypid.xTag,Buff,n);
    WriteToBuff(umax,Buff,n);
    WriteToBuff(umin,Buff,n);
    WriteToBuff(mypid.u0,Buff,n);
    WriteToBuff(mypid.y0,Buff,n);
    WriteToBuff(mypid.u1,Buff,n);
    WriteToBuff(mypid.y1,Buff,n);
    WriteToBuff(mypid.Kidiss,Buff,n);
    WriteToBuff(mypid.Ku,Buff,n);
    WriteToBuff(mypid.x0,Buff,n);
    memset(&Buff[n],0, sizeof(int) * 2); //reserved 
    n += sizeof(int) * 2; //reserved
    WriteToBuff(useCPU_freq,Buff,n);
    WriteToBuff(PID_PWMperiod,Buff,n);
    WriteToBuff(MinCHtemp,Buff,n);
    WriteToBuff(usePIDPWM,Buff,n);
    memset(&Buff[n],0, sizeof(byte) * 3 + sizeof(int) * 14); //reserved 
    n +=  sizeof(byte) * 3 + sizeof(int) * 14;
#else    
    WriteToBuff(useCPU_freq,Buff,n);
    WriteToBuff(MinCHtemp,Buff,n);
    memset(&Buff[n],0, sizeof(int) * 15); //reserved 
    n += sizeof(int) * 15;

#endif //PID_USE

#if SERIAL_DEBUG      
    if( n > sizeof(Buff) )    
         Serial_db.printf("Error: %s buff size %d, need %d\n", __FUNCTION__,  sizeof(Buff), n);
   Serial_db.printf("%s buff size %d, need %d\n", __FUNCTION__,  sizeof(Buff), n);
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

    WriteToBuff(MQTT_interval,Buff,n);
    WriteToBuff(MQTT_port,Buff,n);

#endif

    rc = Write_data_fs((char *)pathmqtt, Buff, n, 1);
#if SERIAL_DEBUG      
    Serial.printf("Write %s rc %i\n", pathmqtt, rc);
#endif

    return rc;
}

int SD_Termo::Read_rc_fs(void)
{   int  n, i, l;
    unsigned short int crs, crs_r, nn;
    unsigned char name[128];

    File file = FlashFS.open(path_rc,"r" );
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

//read 2 byte - length of data
    n = file.read((unsigned char *)&nn, sizeof(nn));
    if(n != sizeof(nn))
    {   file.close();
#if SERIAL_DEBUG      
        Serial.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,sizeof(nn));
#endif        
        return 4;
    }
    if(nn == 0)
    {   if(remote_Host != nullptr)
        { *remote_Host = 0;
        }
    } else {
        if(nn > sizeof(name))
            nn = sizeof(name);
        crs = nn;
        n = file.read(name, nn); //read nn bytes of data
        if(n != nn)
        {   file.close();
#if SERIAL_DEBUG      
            Serial.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,nw);
#endif        
            return 5;
        }
        l = n;
        n = file.read((unsigned char *)&crs_r, sizeof(short int));  //read 2 bytes control sum
        for(i=0; i<l; i++)
        {  crs += name[i];
        }

        if(crs !=  crs_r )
        {   file.close();
    #if SERIAL_DEBUG      
            Serial.printf((PGM_P)F("crs = %i, must be =%i\n"),crs_r,crs);
    #endif        
            return 6;
        }
         
        if(remote_Host != nullptr)
        {   if(strlen(remote_Host) <= l )
                strcpy(remote_Host, (char *) name);
            else
            {   free(remote_Host);
                remote_Host = strdup((char *)name);
            }
        } else {
                remote_Host = strdup((char *)name);
        }
    }

    file.close();

    return 0;
}

int SD_Termo::Write_rc_fs(void)
{   int rc =0, i, n, nw, len = 0; 
    unsigned short int crs;

//    File file = FlashFS.open(_path, FILE_WRITE);  //FILE_WRITE
    File file = FlashFS.open(path_rc, "w");  //FILE_WRITE
    if(!file)
    {  
#if SERIAL_DEBUG      
         Serial.println(F("- failed to open file for writing"));
#endif         
        return 1;
    }

    if(remote_Host)
    {   len = strlen(remote_Host);
    }
    crs = (unsigned short int) len;
    nw = file.write((unsigned char *) &crs, sizeof(unsigned short int));

    if(len > 0)
    {   for(i=0; i<len; i++)
           crs += remote_Host[i];
        n = file.write((unsigned char *) remote_Host, len);
        if(n != len)
            rc = 1;
        else nw += n;    
    }
    nw += file.write((unsigned char *) &crs, sizeof(unsigned short int));
    if(nw != (len + 2*sizeof(unsigned short int) ) )
        rc = 2;

    file.close();
    return rc;
}

int SD_Termo::Read_sensors_fs(void)
{  int rc =0, len, n, nw;
    uint8_t Buff[FS_BUF];

    len =  sizeof(useDS18B20correction) + sizeof(shift_DS_1) + sizeof(shift_DS_2);

    rc = Read_data_fs((char *)path_sensors, Buff, len,  nw, FS_BUF, 0);
#if SERIAL_DEBUG      
    Serial.printf("Read_sensors_fs Read %s rc %i\n", path_sensors, rc);
#endif    
    if(rc)
        return 1;
    memcpy((void *)  &useDS18B20correction, &Buff[0], sizeof(useDS18B20correction));
    n = sizeof(useDS18B20correction);
    memcpy((void *)  &shift_DS_1, &Buff[n], sizeof(shift_DS_1));
    n += sizeof(shift_DS_1);
    memcpy((void *)  &shift_DS_2, &Buff[n], sizeof(shift_DS_2));
    n += sizeof(shift_DS_2);

    return rc;
}

int SD_Termo::Write_sensors_fs(void)
{   int rc =0, n;
    uint8_t Buff[FS_BUF];
    memcpy(&Buff[0],(void *) &useDS18B20correction, sizeof(useDS18B20correction));
    n = sizeof(useDS18B20correction);
    memcpy(&Buff[n],(void *) &shift_DS_1 , sizeof(shift_DS_1));
    n += sizeof(shift_DS_1);
    memcpy(&Buff[n],(void *) &shift_DS_2 , sizeof(shift_DS_2));
    n += sizeof(shift_DS_2);
    rc = Write_data_fs((char *)path_sensors, Buff, n, FS_BUF, 0);
#if SERIAL_DEBUG      
    Serial.printf("Write_sensors_fs %s rc %i\n", path_sensors, rc);
#endif
    return rc;
}


int SD_Termo::Read_data_fs(char *_path, uint8_t *dataBuff, int len, int &rlen, int mode)
{   int rc, buff_len;
    buff_len = FS_BUF;
    if(mode) 
        buff_len = FS_BUFMQTT;
    rc = Read_data_fs(_path, dataBuff,  len, rlen,  buff_len, 1);
    return rc;
}

int SD_Termo::Read_data_fs(char *_path, uint8_t *dataBuff, int len, int &rlen,  int buff_len, int need_ConfigVersion)
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
    if(need_ConfigVersion)
    {   n = file.read((unsigned char *)&v, sizeof(nn));
    
        if(v != CONFIG_VERSION)
        {   file.close();
            return 2;
        }
    }

    n = file.read((unsigned char *)&v, sizeof(nn));
    if( v != buff_len ) 
    {   file.close();
        return 3;
    }

//read 2 byte - length of data
    n = file.read((unsigned char *)&nn, sizeof(nn));
    if(n != sizeof(nn))
    {   file.close();
#if SERIAL_DEBUG      
        Serial_db.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,sizeof(nn));
#endif        
        return 4;
    }
    if(nn  > len)
    {
        Serial_db.printf((PGM_P)F("read file Buff size  %d, must be =%d\n"), len, nn);
        return 10;
    }

    crs = nn;
    nw = nn;
    n = file.read((unsigned char *) dataBuff, nw); //read nn bytes of data
    if(n != nw)
    {   file.close();
#if SERIAL_DEBUG      
        Serial_db.printf((PGM_P)F("file.read rc %i, must be =%i\n"),n,nw);
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
        Serial_db.printf((PGM_P)F("crs = %i, must be =%i\n"),crs_r,crs);
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
{   int rc, buff_len;
    buff_len = FS_BUF;
    if(mode) 
        buff_len = FS_BUFMQTT;
    rc = Write_data_fs(_path, dataBuff,  len, buff_len, 1);
    return rc;
}

int SD_Termo::Write_data_fs(char *_path, uint8_t *dataBuff, int len, int buff_len, int need_ConfigVersion)
{   int rc=0, i, n, nw;
    unsigned short int crs, v;

#if SERIAL_DEBUG      
    Serial_db.printf((PGM_P)F("Writing file: %s %d bytes\n"), _path, len);
#endif // SERIAL_DEBUG      

    File file = FlashFS.open(_path, "w");  //FILE_WRITE
    if(!file)
    {  
#if SERIAL_DEBUG      
         Serial.println(F("- failed to open file for writing"));
#endif         
        return 1;
    }
    nw = 0;
    if(need_ConfigVersion)
    {
        v = CONFIG_VERSION;
        nw = file.write((unsigned char *) &v, sizeof(unsigned short int));
    }
    v =  buff_len;
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
//    Serial_db.printf("write nw =%d len=%d rc=%d\n", nw, len, rc);
    file.close();
    return rc;
}

