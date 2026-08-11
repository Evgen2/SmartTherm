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
char tmpDebugstr[128] ="";
char U0_debug[256];

/* mode = 1 for OT2 slave control on
*/
void SD_Termo::loop_PID(int mode)
{   static int start = 2;
    static int start_heat = 2;
    static  unsigned long int  t0=0, t_start_heat=0, t_stop_heat = 0;
    static float _ustart = 0.f;
    static int OldBoilerStatus=0, issF = 0;
    static int HW_flag = 0, smooth_increase_flag = 0;
    static int smooth_increase_temp_t = 60*15;
    unsigned long int t;
    float  u0, _u, _uu;
    int  dt;
    time_t now; 
    
    int is = 0;
    static int need_heat = 0;

    if(!usePID)
        return;

    t = millis();
//    if(t - t0_mean >= (unsigned long int)(mypid.t_interval*1000)) 
//    {   loop_mean(); //получаем средние значения для используемых температур
//        t0_mean = t;
//    }

    if((stsOT == 0) && !(OldBoilerStatus & 0x08) &&  (BoilerStatus & 0x08) ) //Flame status changed from off to on
        issF = 4;

    OldBoilerStatus = BoilerStatus;
//if Flame status changed  then 4 times continue with 4 sec interval  
    if(issF > 0)
    {   if(issF < 4 && (t - t0 < 4000)) 
            return;
        issF--;      
    }

    is = loop_pid_gettemp(start);

    if(!issF)
    {   if(mypid.xChanged&0x01)
        {   if((t - mypid.xChanged_t) < (unsigned long int)((mypid.t_interval)*1000*2))
            {
                if(t - t0 < (unsigned long int)(mypid.t_interval*1000))
                    return; 
            }
        } else {
                if(t - t0 < (unsigned long int)(mypid.t_interval*1000))
                    return; 
        }
    }

//     Serial_db.printf("==>PID dt %d issF %d xChanged %d (t - mypid.xChanged_t) %d\n", t-t0, issF,  mypid.xChanged, t - mypid.xChanged_t); 

    t0 = t;
    if(mode == 1)
        return;

    {  static int raz = 0;
       MQTT_pub_cmd(raz);
       raz = (raz + 1)&0x01;
    }

/************** считаем u0 ********************/
    if(is & 0x02)
    {   if(tempoutdoor <= mypid.y0) /* for example xTag=20, y0 =10 tempoutdoor = -5*/
        {    u0 = mypid.u0 + (mypid.u1 - mypid.u0) * (tempoutdoor - mypid.y0) /(mypid.y1 - mypid.y0) + mypid.Ku * (mypid.xTag - mypid.x0);
sprintf(U0_debug, "0 %x Toutdoor %g u0 %g mypid:y0 %g xTag %g Ku %g", is, tempoutdoor, u0, mypid.y0, mypid.xTag, mypid.Ku );        
        }
        else
        {  if(mypid.xTag > tempoutdoor) /* for example xTag=20, y0 =10 tempoutdoor = 15 */
            {   u0 = mypid.xTag + (mypid.u0 + mypid.Ku * (mypid.xTag - mypid.x0) - mypid.xTag)  * (tempoutdoor - mypid.xTag) /(mypid.y0 - mypid.xTag);
            } else { /* for example xTag=8, y0 =10  tempoutdoor = 15*/
                u0 = mypid.xTag;
            }
        }
sprintf(U0_debug, "1 %x Toutdoor %g u0 %g mypid:y0 %g xTag %g", is, tempoutdoor, u0, mypid.y0, mypid.xTag);        
    } else { //нет внешней температуры
        u0 = _U0start + mypid.Ku * (mypid.xTag - mypid.x0);
sprintf(U0_debug, "2 %x u0 %g _U0start %g mypid: Ku %g xTag %g x0 %g", is,  u0, _U0start, mypid.Ku, mypid.xTag, mypid.x0 );        
    } 
    u0 =  safeFloat( u0); 
    if(u0 < 0.f)
        u0 = 0.f;
    else if(u0 > umax)
        u0 = umax;

//   Serial_db.printf("loop_pid_gettemp is =%d start=%d tempoutdoor =%f u0=%f InT=%f\n",
//             is, start, tempoutdoor, u0, mypid.InT );

/**********************************************/
    if(!(is & 0x01))  // если нет  tempindoor 
                return;
                
    mypid.Pid(tempindoor, u0); //PID

   Serial_db.printfm(DEBUG_PID, "pid: U= %.3f u0 = %.3f dP=%.3f dD=%.3f dI=%.3f x=%.3f Xtag=%.3f dt=%d flame %d\n",
        mypid.u, u0,  mypid.dP, mypid.dD, mypid.dI, mypid.x, mypid.xTag, mypid.dt, (BoilerStatus & 0x08)); 
   
    now = time(nullptr);

    if(HotWater_present)
    {   if(BoilerStatus & 0x04) /* при включении горячей воды не занимаемся регулированием, хотя PID все равно вызываем */
        {   HW_flag = 1;
            return;
        }
        if(enable_CentralHeating_real && !(BoilerStatus& 0x08)) //flame off если горелка выключена
        {   dt = now - Bstat.t_HW_off;
            if(dt < 180 /* 500 */)  //если HW выключилось < 180 сек назад, то не регулируем
                return;
        }
    }

    _u = mypid.u;
    if(_u > umax)
        _u = umax;
    if(usePID & 0x04)
    {  loop_pwm(_u, need_heat);
    } 
    
    if(_u <= mypid.xTag || (_u <= umin - 0.5f) )
    {   need_heat = 0;
    }  else if(_u >= umin + 0.5f) {
        need_heat = 1;
    }
    
//    Serial_db.printf("==>PID _u %f need_heat %d\n", _u, need_heat); 

    if(need_heat == 1 && (start_heat == 0 || start_heat == 2)) //включение отопления
    {   dt =  now - t_stop_heat;
        if(dt > 180) //3 минуты - защита от кратковременного вЫключения
        {
            enable_CentralHeating_real = true;
            start_heat = 1;
            t_start_heat = now; //время включения отопления
            _ustart  = _u;
        }
    } else if(need_heat == 0) { // отопление не нужно

        if(start_heat == 1 || start_heat == 2) { //выключение отопления
            dt =  now - t_start_heat;
            if(dt > 180) //3 минуты - защита от кратковременного включения
            {
                enable_CentralHeating_real = false;
                _u = umin;
                start_heat = 0;
                smooth_increase_flag = 0;
                HW_flag = 0;
                t_stop_heat = now; //время выключения отопления
            }
        } else if (enable_CentralHeating_real) { // сюда попадем, если отключим PID при отсутствии необходимости в отоплении, а замем снова включим его
                enable_CentralHeating_real = false;
                HW_flag = 0;
                t_stop_heat = now; //время выключения отопления
        }
    }

//    Serial_db.printf("==>PID _u %f need_heat %d start_heat %d enable_CentralHeating_real %d\n",
//             _u, need_heat, start_heat, enable_CentralHeating_real); 

    if(start_heat == 1 && need_heat == 1) //отопление включено
    {   if(BoilerStatus& 0x08) //если горелка включена
        {   
            if(smooth_increase_flag == 0)
            {
                if( ((mypid.xTag - tempindoor) > 2.f) || HW_flag)
                    smooth_increase_temp_t = 5*60;
                else
                    smooth_increase_temp_t = 15*60;
                smooth_increase_flag = 1;
            }

            dt = now - Bstat.t_flame_on;
            _uu = _u;
//            if(issF == 3)
//                _ustart = _u;

            if(dt < smooth_increase_temp_t) //пытаемся плавно повышать температуру
            {   float r;
                r = dt/ float(smooth_increase_temp_t);
                _uu = _u * r +  _ustart  * (1-r); // корректируем уставку температуры

//    Serial_db.printf("==>PID smooth_increase t %d r %g, _uu %g  _u %g  _ustart %g\n", smooth_increase_temp_t, r, _uu, _u, _ustart); 

            }
            if(BoilerT > _uu) //однако, если температура  теплоносителя уже достигла заданного значения
            {   _uu = BoilerT;  
/* пытаемся предотвратить тактование */
                if(_uu - _u > 4.f)  /* допускаем повышение температуры бойлера не более чем на 4 градуса выше PID  */
                    _uu = _u + 4.f;
                if(_uu >  umax)  // ограничиваем max
                    _uu =  umax;
            }

           _u = _uu;

        } else {   //если горелка еще выключена
            if(_u - BoilerT > CH_StartGist)  //CH_StartGist= 2 ...15
            {   _uu = BoilerT + CH_StartGist; //ограничиваем  температуру теплоносителя при включении
                if(_uu < umin)
                   _uu =  umin;
                _u = _uu;
                _ustart  = _u;
            } else {
                _ustart  = _u;
            }
        }  
    }

    Tset = CHtempLimit(_u);
    sprintf(tmpDebugstr,"Tset=%g _u=%g", Tset, _u);
//    Serial_db.printf("==>PID Tset %f_u %f need_heat %d enable_CentralHeating_real %d\n",
//             Tset, _u, need_heat, enable_CentralHeating_real); 
    need_set_T(1);  // for OpenTherm
#if MQTT_USE
    MQTT_need_report = 1; // for MQTT
    MQTT_pub_cmd2(millis() - t);

#endif            

}

//src = 0 - Web, 1 - MQTT, 2 servercallback_send_Sts_answ, 3 callback_Set_OpenThermData,
// 4 callback_Set_State
void SD_Termo::set_new_PID_setpoint(float Tsetpoint, int src)
{
#if PID_USE
oldTroomSetpoint = mypid.xTag;
    src_lastSetPointChange = src;
    mypid.Set_NewTag(Tsetpoint, oldTroomSetpoint,  tempindoor);

    t_lastSetPointChange = time(nullptr);
#endif    
}

void SD_Termo::loop_pwm(float &_u, int &need_heat)
{   if(_u > umin)
    {   PID_PWM_sts = 0;
        Serial_db.printfm(DEBUG_PID,"PID_PWM_sts %d, _u = %g\n", PID_PWM_sts, _u);
             return;
    }
    if( _u <= mypid.xTag)
    {   PID_PWM_sts = 0;
        need_heat = 0;
//Serial_db.printfm(DEBUG_PID,"PID_PWM_sts %d, need_heat = 0,  _u = %g\n", PID_PWM_sts, _u);
        return;
    }
    unsigned long int t = millis();
    float pwm;
    pwm = (_u - mypid.xTag) / (umin - mypid.xTag);

    switch(PID_PWM_sts) 
    {   case 0:
        {
//Serial_db.printfm(DEBUG_PID,"PID_PWM_sts %d at %ld pwm %g\n", PID_PWM_sts, t, pwm);
            PID_PWM_sts = 1;
            PID_PWM_t0 = t;
            _u = umin;
            need_heat = 1;
        }
            break;
        case 1:
        if(t - PID_PWM_t0 > int(PID_PWMperiod*1000 * pwm))
        {   PID_PWM_sts = 2;
//Serial_db.printfm(DEBUG_PID,"PID_PWM_sts change to %d at %ld, dt %ld\n", PID_PWM_sts, t, t-PID_PWM_t0);
            _u = umin - 10;
            need_heat = 0;
        } else {
            _u = umin;
            need_heat = 1;
//Serial_db.printfm(DEBUG_PID,"PID_PWM_sts %d dt %ld _u%g pwm %g\n", PID_PWM_sts, t-PID_PWM_t0, _u, pwm);
        }
            break;
        case 2:
        if(t-PID_PWM_t0 >= PID_PWMperiod*1000)
        {   PID_PWM_sts = 1;
//Serial.printf("PID_PWM_sts change to %d at %ld, dt %ld\n", PID_PWM_sts, t, t-PID_PWM_t0);
            _u = umin;
            need_heat = 1;
            PID_PWM_t0 += PID_PWMperiod*1000;
        } else {
            _u = umin - 10;
            need_heat = 0;
//Serial.printf("PID_PWM_sts %d dt %ld _u%g pwm %g\n", PID_PWM_sts, t-PID_PWM_t0, _u, pwm);
        }
            break;
    }   
}

//получаем средние значения для используемых температур
void SD_Termo::loop_mean(void) 
{   unsigned long t = millis();
    static  unsigned long int  t0_mean=0;

    if(t - t0_mean < (unsigned long int)(mypid.t_interval*1000)) 
        return;

    t0_mean = t;

    for(int i=0; i <= MAX_PID_SRC; i++)
    {
         if(t_mean[i].isset == -1 && t_mean[i].nx == 0)
            continue;
        t_mean[i].get();
//debug
//if(i < 2)
//    Serial_db.printf("t_mean[%d] x=%f  mean =%f nx=%d isset %d\n", i, t_mean[i].x, t_mean[i].xmean, t_mean[i].nx, t_mean[i].isset ); 

////        if(t_mean[i].nx > 2 || (i == 4 && t_mean[i].isset == 1)) /* 4 - outdoor mqtt */
//        if(t_mean[i].nx > 2 || (t_mean[i].isset == 1 && (t - t_mean[i].t_set > 30000))) 
//                t_mean[i].init(1);
        if (t_mean[i].isset == 1 && (t - t_mean[i].t_set > 600000)) // 10 min
                t_mean[i].init(0x2);
        else if(t_mean[i].nx > 2 )
                t_mean[i].init(1);
        else if (t_mean[i].isset == 1 && (t - t_mean[i].t_set > 30000)) //30 sec
                t_mean[i].init(0x3);
    }
}

int SD_Termo::loop_pid_gettemp(int &_start) //получаем значения tindoor и toutdoor
{   int is=0;
    if(_start)
    {   if(_start == 2)
        {  // start_t = t;
            _start = 1;
            mypid.NextTact();
        }

        if(srcTroom < 0 || srcTroom > 4)
        {  is = 0;
        } else {
//  Serial_db.printf("0 srcTroom =%d, isset=%d xmean=%f nx=%d\n",
//         srcTroom, t_mean[srcTroom].isset,t_mean[srcTroom].xmean, t_mean[srcTroom].nx); 
            if(t_mean[srcTroom].isset != -1)
            {   tempindoor = t_mean[srcTroom].x;
                t_mean[srcTroom].fsf.Max_Gap = 3.5;
                t_mean[srcTroom].fsf.Adapt_Limit = 3;
                is |= 1;
                IsSetTemp |= 0x01;
                _start = 0; 
            }

            if(srcText < 0 || srcText > MAX_PID_SRC) 
            { is &= ~2;
            } else if(t_mean[srcText].isset != -1) {
                t_mean[srcText].fsf.Max_Gap = 3.5 * 2;
                t_mean[srcText].fsf.Adapt_Limit = 2;
                tempoutdoor = t_mean[srcText].x;
                is |= 2;
                IsSetTemp |= 0x02;
            }
        }
    } else {  // start == 0
        if(srcTroom >= 0 && srcTroom <= 3 ) // !4
        {
            if(t_mean[srcTroom].isset >= 0)
            {   tempindoor = t_mean[srcTroom].x;
                is |= 1;
                IsSetTemp |= 0x01;
                if(t_mean[srcTroom].changed)
                {   t_mean[srcTroom].changed = 0;
                    if(mypid.xChanged == 0)
                    {   mypid.xChanged |= 0x01; //indoor T changed
                        mypid.xChanged_t = millis();
                    }

//  Serial_db.printf("tempindoor Changed  srcTroom =%d, isset=%d xmean=%f nx=%d\n",
//         srcTroom, t_mean[srcTroom].isset,t_mean[srcTroom].xmean, t_mean[srcTroom].nx); 

                }
            }
        }
        if((srcText >= 0 && srcText <= 2) || (srcText >= 4 && srcText <= MAX_PID_SRC)) // !3 MAX_PID_SRC!!
        {
            if(t_mean[srcText].isset >= 0)
            {   tempoutdoor = t_mean[srcText].x; 
                is |= 2;
                IsSetTemp |= 0x02;
            }
        }
    }
/************ endof  if(_start) **********************************/
    return is;
}

#endif // PID_USE
