/* SD_Config.cpp */

#include <AutoConnect.h>
#include <AutoConnectFS.h>
extern AutoConnectFS::FS& FlashFS;

#include "SD_OpenTherm.hpp"

/*******************************/
const int FS_BUF = sizeof(SD_Termo::enable_CentralHeating) + sizeof(SD_Termo::enable_HotWater) + sizeof(SD_Termo::Tset) + sizeof(SD_Termo::TdhwSet) + sizeof(SD_Termo::UDPserver_repot_period) +
                 sizeof(SD_Termo::UDPserver_port) + sizeof(SD_Termo::TCPserver_report_period) + sizeof(SD_Termo::TCPserver_port) + sizeof(SD_Termo::tcp_remoteIP) + sizeof(SD_Termo::Use_remoteTCPserver) + sizeof(SD_Termo::UseID2) +
                 sizeof(SD_Termo::ID2masterID) + sizeof(SD_Termo::CH2_DHW_flag) + sizeof(SD_Termo::UseWinterMode) + sizeof(SD_Termo::Use_OTC) +sizeof(SD_Termo::Use_ID29_DHW_flag) +
                 sizeof(SD_Termo::Immergas_fix_flag) + sizeof(SD_Termo::Kotitonttu_fix_flag) + 
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
            sizeof(SD_Termo::mypid.y0) + sizeof(SD_Termo::mypid.u1)   + sizeof(SD_Termo::mypid.y1) + sizeof(SD_Termo::mypid.Kidiss) + sizeof(SD_Termo::mypid.Ku) +
            sizeof(SD_Termo::mypid.x0) + sizeof(int) * 2 //reserved 

#endif
            + sizeof(SD_Termo::useCPU_freq) + sizeof(int) * 17 //reserved 
    ;

#if MQTT_USE
const int FS_BUFMQTT =     
            sizeof(SD_Termo::useMQTT) + sizeof(SD_Termo::MQTT_server) + sizeof(SD_Termo::MQTT_user) + sizeof(SD_Termo::MQTT_pwd) + sizeof(SD_Termo::MQTT_topic) +
            sizeof(SD_Termo::MQTT_devname) + sizeof(SD_Termo::MQTT_interval) + sizeof(SD_Termo::MQTT_port);
#endif

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

    Serial.printf("Use_remoteTCPserver %d tcp_remoteIP: %s\n",Use_remoteTCPserver, tcp_remoteIP.toString().c_str());

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
    memcpy((void *) &Kotitonttu_fix_flag, &Buff[n], sizeof(Kotitonttu_fix_flag));
    n += sizeof(Kotitonttu_fix_flag);
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

    memcpy((void *) &mypid.Ku, &Buff[n], sizeof(mypid.Ku));
    n += sizeof(mypid.Ku);
    if(n >= nw) goto END;
    memcpy((void *) &mypid.x0, &Buff[n], sizeof(mypid.x0));
    n += sizeof(mypid.x0);
    if(n >= nw) goto END;

    n += sizeof(int) * 2; //reserved
    if(n >= nw) goto END;

#endif //PID_USE

    memcpy((void *) &useCPU_freq, &Buff[n], sizeof(useCPU_freq));
    n += sizeof(useCPU_freq);
    if(n >= nw) goto END;

    memset(&Buff[n],0, sizeof(int) * 17); //reserved 
    n += sizeof(int) * 17;

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
   Serial.printf("%s Read_ot_fs  buff size %d, read %d need %d\n", __FUNCTION__,  sizeof(Buff), nw, n);
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
    memcpy(&Buff[n],(void *) &Kotitonttu_fix_flag, sizeof(Kotitonttu_fix_flag));
    n += sizeof(Kotitonttu_fix_flag);    
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

    memcpy(&Buff[n],(void *) &mypid.Ku , sizeof(mypid.Ku));
    n += sizeof(mypid.Ku);
    memcpy(&Buff[n],(void *) &mypid.x0 , sizeof(mypid.x0));
    n += sizeof(mypid.x0);
    memset(&Buff[n],0, sizeof(int) * 2); //reserved 
    n += sizeof(int) * 2;
    
#endif
    memcpy(&Buff[n],(void *) &useCPU_freq , sizeof(useCPU_freq));
    n += sizeof(useCPU_freq);


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
//    Serial.printf("write nw =%d len=%d rc=%d\n", nw, len, rc);
    file.close();
    return rc;
}

