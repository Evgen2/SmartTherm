/* SD_pid_control.cpp */
#include <time.h>
#include <Arduino.h>

#include "OpenTherm.h"
#include "SD_OpenTherm.hpp"
#include "Smart_commands.h"

#if PID_USE

void  MQTT_pub_cmd(int on);
void  MQTT_pub_cmd2(int val);

int debcode = 0;
int wait_if_takt = 60*3;

void SD_Termo::loop_PID(void)
{   static int start = 2;
    static int start_heat = 2;
    static  unsigned long int  t0=0, t_start_heat=0, flame_old = 0;
    static float _ustart = 0.f;
    static int OldBoilerStatus=0, issF = 0;
    unsigned long int t;
    float  u0, _u, _uu;
    int rc, dt;
    time_t now; 
    extern OpenTherm ot;
    int is = 0;
    int need_heat = 0;

    if(!usePID)
        return;

    t = millis();

    if((stsOT == 0) && ((OldBoilerStatus & 0x08) !=  (BoilerStatus & 0x08)) )
        issF = 4;


    if(!issF && (t - t0 < (unsigned long int)(mypid.t_interval*1000))) 
            return;

//     Serial.printf("==>PID dt %d iss %d\n", t-t0, issF); 

    OldBoilerStatus = BoilerStatus;
    if(issF > 0)
    {   if(issF < 4 && (t - t0 < 4000)) 
            return;
        issF--;      
    }

    t0 = t;

    { static int raz = 0;
       MQTT_pub_cmd(raz);
       raz = (raz + 1)&0x01;
    }

    loop_mean(); //получаем средние значения для используемых температур
/************** считаем u0 ********************/
    is = loop_pid_gettemp(start);

    if(is & 0x02)
    {   if(tempoutdoor <= mypid.y0)
            u0 = mypid.u0 + (mypid.u1 - mypid.u0) * (tempoutdoor - mypid.y0) /(mypid.y1 - mypid.y0);
        else
        {  if(mypid.y0 != mypid.xTag)
                  u0 = mypid.xTag + (mypid.u0 - mypid.xTag)  * (tempoutdoor - mypid.xTag) /(mypid.y0 - mypid.xTag);
           else
                  u0 = mypid.xTag;
        }
    } else { //нет внешней температуры
        u0 = mypid.u0;
    } 
/**********************************************/
    if(!(is & 0x01))  // если нет  tempindoor 
                return;
                
    rc = mypid.Pid(tempindoor, u0); //PID

    if(rc != 1)  // если PID не OK
                return;
    now = time(nullptr);

    if(HotWater_present)
    {  if(BoilerStatus & 0x04) /* при включении горячей воды не занимаемся регулированием, хотя PID все равно вызываем */
                return;
        if(enable_CentralHeating_real && !(BoilerStatus& 0x08)) //flame off если горелка выключена
        {   dt = now - Bstat.t_HW_off;
            if(dt < 180 /* 500 */)  //если HW выключилось 500 сек назад или раньше, то не регулируем
                return;
        }
    }

    _u = mypid.u;
    if(_u > mypid.umax)
        _u =  mypid.umax;

    if(_u <= mypid.xTag || (_u <= mypid.umin -1.f) )
    {    need_heat = 0;
    }  else {
        need_heat = 1;
    }
    
    if(need_heat == 1 && (start_heat == 0 || start_heat == 2)) //включение отопления
    {   enable_CentralHeating_real = true;
//       MQTT_pub_cmd(enable_CentralHeating_real);
        start_heat = 1;
        t_start_heat = now; //время включения отопления
        _ustart  = _u;
    } else if(need_heat == 0 && (start_heat == 1 || start_heat == 2)) { //выключение отопления
        enable_CentralHeating_real = false;
        _u = mypid.umin;
//        MQTT_pub_cmd(enable_CentralHeating_real);
        start_heat = 0;
    }

    if(start_heat == 1 && need_heat == 1) //отопление включено
    {   if(BoilerStatus& 0x08) //если горелка включена
        {   dt = now - Bstat.t_flame_on;
            _uu = _u;
            if(issF == 3)
                _ustart = _u;

            if(dt < 15*60) //пытаемся плавно повышать температуру
            {   float r, du;
                r = dt/(60.*15.);
                _uu = _u * r +  _ustart  * (1-r); //то корректируем уставку температуры
            }
            if(BoilerT > _uu) //однако, если температура  теплоносителя уже достигла заданного значения
            {   _uu = BoilerT;  
/* пытаемся предотвратить тактование */
                if(_uu - _u > 4.f)  /* допускаем повышение температуры бойлера не более чем на 4 градуса выше PID  */
                    _uu = _u + 4.f;
                if(_uu >  mypid.umax)  // ограничиваем max
                    _uu =  mypid.umax;
            }

           _u = _uu;

        } else {   //если горелка еще выключена
            if(_u - BoilerT > 10.f)
            {   _u = BoilerT + 10.f; //ограничиваем  температуру теплоносителя при включении
                _ustart  = _u;
            }
        }  
    }

    Tset = _u;
    need_set_T = 1;  // for OpenTherm
#if MQTT_USE
    MQTT_need_report = 1; // for MQTT
    MQTT_pub_cmd2(millis() - t);

#endif            

}


//получаем средние значения для используемых температур
void SD_Termo::loop_mean(void) 
{ 
    for(int i=0; i < 8; i++)
    {
         if(t_mean[i].isset == -1 && t_mean[i].nx == 0)
            continue;
        t_mean[i].get();
//debug
//    Serial.printf("t_mean[%d] x=%f  mean =%f nx=%d isset %d\n", i, t_mean[i].x, t_mean[i].xmean, t_mean[i].nx, t_mean[i].isset ); 

        if(t_mean[i].nx > 8 || (i == 4 && t_mean[i].isset == 1)) /* 4 - outdoor mqtt */
                t_mean[i].init();
    }
}

int SD_Termo::loop_pid_gettemp(int &_start) //получаем значения tindoor и toutdoor
{   int is;
    if(_start)
    {   int start0 = 0;
        if(_start == 2)
        {  // start_t = t;
            _start = 1;
            start0 = 1;
            mypid.NextTact();
        }

        if(srcTroom < 0 || srcTroom > 4)
        {  is = 0;
        } else {
//  Serial.printf("0 srcTroom =%d, isset=%d xmean=%f nx=%d\n",
//         srcTroom, t_mean[srcTroom].isset,t_mean[srcTroom].xmean, t_mean[srcTroom].nx); 
            if(t_mean[srcTroom].isset == -1)
            {   if(t_mean[srcTroom].nx > 1)
                {    tempindoor = t_mean[srcTroom].xmean / float(t_mean[srcTroom].nx) ; 
                    is |= 1;
                }
            } else {
                tempindoor = t_mean[srcTroom].x;
                is |= 1;
                _start = 0; //
            }
            if(srcText < 0 || srcText > MAX_PID_SRC) 
            {
                is &= ~2;

            } else if(t_mean[srcText].isset == -1) {
                if(t_mean[srcText].nx > 1)
                {    tempoutdoor = t_mean[srcText].xmean / float(t_mean[srcText].nx) ; 
                    is |= 2;
                }
            } else {
                tempoutdoor = t_mean[srcText].x;
                is |= 2;
            }

            if(start0 && (is & 0x01))
                    mypid.Init_I(tempindoor );

        }
//        Serial.printf("0 is =%d, tempindoor =%f tempoutdoor=%f\n", is, tempindoor, tempoutdoor ); 
    } else {  // start == 0
        if(srcTroom >= 0 && srcTroom <= 3 ) // !4
        {
//    Serial.printf("00 srcTroom =%d, isset=%d xmean=%f nx=%d\n",
//         srcTroom, t_mean[srcTroom].isset,t_mean[srcTroom].xmean, t_mean[srcTroom].nx); 

            if(t_mean[srcTroom].isset >= 0)
            {   tempindoor = t_mean[srcTroom].x;
                //tempindoor = (tempindoor + t_mean[srcTroom].x) * 0.5;
                is |= 1;
            }
        }
        if((srcText >= 0 && srcText <= 2) || (srcText >= 4 && srcText <= MAX_PID_SRC)) // !3 MAX_PID_SRC!!
        {
            if(t_mean[srcText].isset >= 0)
            {   tempoutdoor = t_mean[srcText].x; 
                //tempoutdoor = (tempoutdoor + t_mean[srcText].x) * 0.5;
#if DEBUG_WITH_EMULATOR  //translate to emulator tempoutdoor as TdhwSet
                need_set_dhwT = 1;
#endif
                is |= 2;
            }
        }
//    Serial.printf("1 is =%d, tempindoor =%f tempoutdoor=%f\n", is, tempindoor, tempoutdoor ); 
    }
/************ endof  if(_start) **********************************/
    return is;
}

#endif // PID_USE
