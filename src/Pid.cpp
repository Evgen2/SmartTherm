/* Pid.cpp */
#include <Arduino.h>
#include <time.h>
#include "Smart_Config.h"
#if PID_USE
#include "pid.hpp"

 int pid::Pid(float _x, float _u0)
 {  unsigned long int t, dt, _t;
    float _xerr, dX, dtf, _dft, _u;
    t  = millis();
    dt = t - pid_t; // dt, msec

   Serial.printf("pid: dt = %ld\n", dt );

    if(dt < (unsigned long int)(t_interval*1000))
    {   return 0;
    } 

//P    
   x = _x;
   xerr = xTag - x; //grad

//D   
   dSt.get(_xerr, _t);
   if(dSt.n < NB)
   {   dX = 0.;   
   } else {
      _dft = float(t - _t) / 1000.f; // dt, sec
      dX = (xerr - _xerr) /_dft * 3600; //grad/hour
   }
   dSt.add(xerr, t);
//I
   dtf = float(dt) / 1000.f; // dt, sec
   InT = InT*(1.-Kidiss) + xerr * dtf; // grad * sec
#if SERIAL_DEBUG 
   Serial.printf("pid: dt %d xerr=%f, InT=%f dX=%f\n",
          dt , xerr, InT, dX); 
   Serial.printf("pid: _x %f xTag=%f, u0=%f\n",
          _x , xTag, _u0); 
#endif          
   dP = xerr * Kp;
   dD = dX * Kd;
   dI = InT * Ki;
   _u = dP + dD + dI;
   u = _u + _u0;
#if SERIAL_DEBUG 
   Serial.printf("pid: U= %f u0 = %f _u = %f dP=%f, dD=%f dI=%f\n",
        u, _u0, _u , dP, dD, dI); 
#endif         
   ub = _u0;

   NextTact();

    return 1;
 }
  
#endif //PID_USE