/* SD_pid_control.cpp */
#include <time.h>
#include <Arduino.h>

#include "OpenTherm.h"
#include "SD_OpenTherm.hpp"
#include "Smart_commands.h"

#if PID_USE

void  MQTT_pub_cmd(int on);

int debcode = 0;
int wait_if_takt = 60*3;

void SD_Termo::loop_PID(void)
{   static int start = 2;
    static int start_heat = 2;
    static  unsigned long int  t0=0, t_start_heat=0, flame_old = 0;
    static float _ustart = 0.f;
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
    if(t - t0 < (unsigned long int)(mypid.t_interval*1000))
            return;
    t0 = t;

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
            if(dt < 500)  //если HW выключилось 500 сек назад или раньше, то не регулируем
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
        MQTT_pub_cmd(enable_CentralHeating_real);
        start_heat = 1;
        t_start_heat = now; //время включения отопления
        _ustart  = _u;
    } else if(need_heat == 0 && (start_heat == 1 || start_heat == 2)) { //выключение отопления
        enable_CentralHeating_real = false;
        _u = mypid.umin;
        MQTT_pub_cmd(enable_CentralHeating_real);
        start_heat = 0;
    }

    if(start_heat == 1 && need_heat == 1) //отопление включено
    {   if(BoilerStatus& 0x08) //если горелка включена
        {   dt = now - Bstat.t_flame_on;
            _uu = _u;
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
#endif            

}

void SD_Termo::loop_PIDold(void)
{   static int start = 2;
    static int start_heat = -1;
    static  unsigned long int /* start_t=0,*/ t0=0, t_start_heat=0, flame_old = 0;
    unsigned long int t;
    float  u0, umin_on;
    int rc, dt;
    time_t now; 
    extern OpenTherm ot;
    int is = 0;

    if(!usePID)
        return;

    t = millis();
    if(t - t0 < (unsigned long int)(mypid.t_interval*1000))
            return;
    t0 = t;

    loop_mean(); //получаем средние значения для используемых температур

    umin_on = mypid.umin + 5;
/**********************************************/
    is = loop_pid_gettemp(start);

    if(is & 0x02)
    {   
        if(tempoutdoor <= mypid.y0)
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

    if(is & 0x01)
    {   rc = mypid.Pid(tempindoor, u0); //PID

        if(HotWater_present)
        {  if(BoilerStatus & 0x04) /* при включении горячей воды не занимаемся регулированием, хотя PID все равно вызываем */
                rc = 0;
            else if(enable_CentralHeating_real && !(BoilerStatus& 0x08)) //flame off если горелка выключена
            {   now = time(nullptr);
                dt = now - Bstat.t_HW_off;
                if(dt < 500)  //если HW выключилось 500 сек назад или раньше, то не регулируем
                    rc = 0;
            }
        }
        if(rc == 1)
        {  float _u, _uu;
            int need_heat = 0;
            _u = mypid.u;
            if(_u > mypid.umax)
                    _u =  mypid.umax;
            if(_u <= mypid.xTag)
            {    need_heat = 0;
            debcode = 1;
#if SERIAL_DEBUG      
      Serial.printf("loopPID(%d): _u %f < xTag %f need_heat 0\n", debcode, _u, mypid.xTag);
#endif      
            }
            else if(_u <= mypid.umin)
            {   if(mypid.umin - _u > 2.)
                {    need_heat = 0;
                debcode = 2;
#if SERIAL_DEBUG      
      Serial.printf("loopPID(%d): _u %f < mypid.umin %f - 5 need_heat 0\n", debcode, _u, mypid.umin);
#endif      
                }
                else //если целевая температура не ниже минимальной минус 5 градусов, выдаем минимальную
                {   _u = mypid.umin;
                    need_heat = 1;
                    debcode = 3;
#if SERIAL_DEBUG      
      Serial.printf("loopPID(%d): _u %f , mypid.umin %f need_heat 1\n", debcode, _u, mypid.umin);
#endif      
                }
            }  else 
                need_heat = 1;

            if(need_heat) 
            { 
#if SERIAL_DEBUG      
    if(BoilerStatus& 0x08)
      Serial.printf("loopPID: need_heat 1 , flame ON  _u %.2f BoilerT  %.2f RetT %.2f\n", _u, BoilerT, RetT);
    else  
      Serial.printf("loopPID: need_heat 1 , flame OFF _u %f BoilerT  %.2f RetT %.2f\n", _u, BoilerT, RetT ) ;
#endif      
                   
                if(!(BoilerStatus& 0x08)) //flame off если горелка выключена
                {   now = time(nullptr);
                    dt = now - Bstat.t_flame_off;
                    if(dt < wait_if_takt) //wait_if_takt/60 минут
                    {   need_heat = 0;
                        debcode = 4; 
                        if(start_heat == 1) //горелка выключена, но флаг start_heat стоит, значит котел выключился сам (тактование)
                        {   if(wait_if_takt < 60*15) //15 мин макс
                               wait_if_takt += 30;
                        }
#if SERIAL_DEBUG      
      Serial.printf("loopPID(%d): need_heat 1 -> 0 , flame Off dt =%d\n",debcode, dt);
#endif      
                    } else if(dt < 60*30) {   // ограничение на полчаса. рекомендация увеличить выбег насоса в настройках котла
                        if(ot.OTid_used(OpenThermMessageID::Tret))  //если есть обратка
                        {   if(fabs(tempindoor - mypid.xTag) < 0.3 ) //если разница температур меньше 0.3 
                            {   if(RetT - tempindoor > 5.)  //если обратка теплее tempindoor на 5 град
                                {            need_heat = 0;  //то отопление не включаем ????
                                debcode = 5;
#if SERIAL_DEBUG      
      Serial.printf("loopPID (%d): need_heat 1 -> 0, flame Off,  dt =%d tempindoor %.2f, RetT %.2f\n", debcode, dt, tempindoor, RetT ) ;
#endif      
                                }   
                            }
                        } else { //смотрим на BoilerT
                            if(fabs(tempindoor - mypid.xTag) < 0.3 ) //если разница температур меньше 0.3 
                            {   if(BoilerT - tempindoor > 5.)  //если подача теплее tempindoor на 5 град 
                                            need_heat = 0;  //то отопление не включаем ????
#if SERIAL_DEBUG      
                                debcode = 6;
      Serial.printf("loopPID (%d): need_heat 1 -> 0, flame Off,  dt =%d tempindoor %.2f, RetT %.2f\n", debcode, dt, tempindoor, RetT ) ;
#endif      
                            }
                        }
                    }
                    if(need_heat)
                    {   if(start_heat == 0)
                        {   start_heat = 1; //флаг возможности корректировки выходной температуры после включения
                            if(_u > umin_on)
                                _u = umin_on;
                            //_u = mypid.umin;
                            t_start_heat = now; //время включения отопления
                            debcode = 15;                           
#if SERIAL_DEBUG      
      Serial.printf("loopPID: need_heat 1, start_heat 0 -> 1, flame Off,  _u -> %.2f\n", _u ) ;
#endif      
                        } else { //горелка выключена, но флаг start_heat стоит, значит котел выключился сам (тактование)
                            now = time(nullptr);
                            dt = now - Bstat.t_flame_off;
                            if(dt < wait_if_takt) // wait_if_takt/60  минут
                            {    need_heat = 0;
                                if(wait_if_takt < 60*15) //15 мин макс
                                    wait_if_takt += 30;
                            } else {
                                start_heat = 1; 
                                if(_u > umin_on)
                                    _u = umin_on;
                               // _u = mypid.umin;
                                t_start_heat = now; //время включения отопления
                            }

                        }
                    } else {
                        start_heat = 0;                        
#if SERIAL_DEBUG      
      Serial.printf("loopPID: need_heat 0, start_heat 0\n" ) ;
#endif      
                    }
                } else { //flame on [if(!(BoilerStatus& 0x08)) ]
                    if(!start_heat)
                    {   if(flame_old == 0) //котёл включил горелку без нашей команды
                        {   start_heat = 1;
                            now = time(nullptr);
                            t_start_heat = now; //время включения отопления 
                            if(_u > umin_on)
                                _u = umin_on;
                          //  _u = mypid.umin;
                            debcode = 16;                           
#if SERIAL_DEBUG      
      Serial.printf("loopPID: котёл включил горелку без нашей команды\n" ) ;
#endif      
                        }
                    }
                    if(start_heat)
                    {   now = time(nullptr);
                        dt = now - t_start_heat;
                        if(dt > 60*15 || BoilerT > _u) //15 min
                        {   start_heat = 0;
#if SERIAL_DEBUG      
      Serial.printf("loopPID: need_heat 1, start_heat 1 -> 0, flame On,  dt=%d\n", dt ) ;
#endif      
                        } else {
                             _uu = _u;
                            if(ot.OTid_used(OpenThermMessageID::Tret))  //если есть обратка
                            {  //если обратка холоднее прямой на 5 град или температура отопления ниже уставки на 5 градусов или модуляция выше 30
#if SERIAL_DEBUG      
      Serial.printf("loopPID: need_heat 1, start_heat 1, flame On, dt=%d RetT=%.2f Tset =%.2f BoilerT=%.2f _u =%.2f\n", dt, RetT, Tset, BoilerT, _u ) ;
#endif      
                                if(((_u > Tset) && (((BoilerT - RetT) > 5.) || (Tset - BoilerT) > 5.)) || (FlameModulation > 30) )  
                                {   float r, du;
                                    r = dt/(60.*15.);
                                    _uu = _u * r +  mypid.umin * (1-r); //то корректируем уставку температуры
                                    if(BoilerT > _uu)
                                            _uu = BoilerT; 
                                    du = _uu - Tset;
                                    if( du > 5.) _uu += 5.; //повышаем не более 5 градусов за квант времени
                                    if(_uu > _u)
                                        _uu = _u;
#if SERIAL_DEBUG      
      Serial.printf("loopPID:*** _u -> %.2f\n",  _u ) ;
#endif      
                                }

                            } else { //смотрим на BoilerT
#if SERIAL_DEBUG      
//    { int xxx;
//       xxx = (mypid.umin  - RetT > 5.);
//      Serial.printf("loopPID:WTF*+*  %f %f  xxx %d\n",  mypid.umin,  RetT, xxx) ;
//    }
#endif      
//??                            if(((_u > Tset) && (((mypid.umin  - RetT) > 5.)   || (Tset - BoilerT) > 5.)) || (FlameModulation > 30) )  
                                if(((_u > Tset) && (((BoilerT - mypid.umin) > 5.) || (Tset - BoilerT) > 5.)) || (FlameModulation > 30) )  
                                {   float r;
                                    r = dt/(60.*15.);
                                    _uu = _u * r +  mypid.umin * (1-r); //то корректируем уставку температуры
                                    if(BoilerT > _uu)
                                            _uu = BoilerT; 
                                    if(_uu > _u)
                                        _uu = _u;
#if SERIAL_DEBUG      
      Serial.printf("loopPID:*+* _u -> %.2f\n",  _u ) ;
#endif      
                                }
                            }
                            if(BoilerT - Tset > 2.f) // если темп бойлера больше заданной, то может выключится по перегреву
                            {   if(_uu < BoilerT - 1.f)
                                {    _uu  = BoilerT - 1.f;
                                    if(_uu > _u + 1.f) // повышаем заданную t, но не более 1 градуса
                                            _uu = _u + 1.f;
                                }
                            }
                            _u = _uu;
                        }
                    }
                } //endof if(!(BoilerStatus& 0x08))  
            }  //endof  (need_heat) 
 /*****************************************/
            if(!need_heat && (BoilerStatus& 0x08))   //отопление не нужно, но горелка включена
            {   now = time(nullptr);
                dt = now - Bstat.t_flame_on;
                if(dt < 60*3) //3 минуты не выключаем, но горелку на минимум
                {    need_heat = 1;
                     debcode = 21;
                    _u = mypid.umin;
#if SERIAL_DEBUG      
      Serial.printf("loopPID(%d): need_heat 0->1 dt from flame on %d\n", debcode, dt) ;
#endif      
                }
            }                    
 /*****************************************/
           if(need_heat && !(BoilerStatus& 0x08))   //отопление  нужно, но горелка выключена
           {   now = time(nullptr);
               dt = now - Bstat.t_flame_off;
                if(dt < 60*3) //3 минуты не включаем
                {    need_heat = 0;
                }
           }
 
 /*****************************************/
            
            if(need_heat)
                enable_CentralHeating_real = true;
            else
            {    enable_CentralHeating_real = false;
                 _u = mypid.umin;
            }
            MQTT_pub_cmd(enable_CentralHeating_real);
            
#if SERIAL_DEBUG      
      Serial.printf("loopPID: result: need_heat %d Tset=%.2f\n", need_heat, _u ) ;
#endif      
            Tset = _u;
            need_set_T = 1;  // for OpenTherm
#if MQTT_USE
            MQTT_need_report = 1; // for MQTT
#endif

            flame_old = BoilerStatus& 0x08;

        } //end if(rc == 1)
    } //end if(is & 0x01)
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
    {   if(_start == 2)
        {  // start_t = t;
            _start = 1;
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
        }
//        Serial.printf("0 is =%d, tempindoor =%f tempoutdoor=%f\n", is, tempindoor, tempoutdoor ); 
    } else {  // start == 0
        if(srcTroom >= 0 && srcTroom <= 3 ) // !4
        {
//    Serial.printf("00 srcTroom =%d, isset=%d xmean=%f nx=%d\n",
//         srcTroom, t_mean[srcTroom].isset,t_mean[srcTroom].xmean, t_mean[srcTroom].nx); 

            if(t_mean[srcTroom].isset >= 0)
            {   tempindoor = t_mean[srcTroom].x;
                is |= 1;
            }
        }
        if((srcText >= 0 && srcText <= 2) || (srcText >= 4 && srcText <= MAX_PID_SRC)) // !3 MAX_PID_SRC!!
        {
            if(t_mean[srcText].isset >= 0)
            {   tempoutdoor = t_mean[srcText].x;
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
