/* Pid.cpp */
#include <Arduino.h>
#include <time.h>
#include "Smart_Config.h"
#if PID_USE
#include "pid.hpp"

 int pid::Pid(float _x, float _u0)
 {  unsigned long int t, dt, _t;
    float _xerr, dX, dtf, _dft, _u;
    float _dft0;
    t  = millis();
    dt = t - pid_t; // dt, msec

      Serial.printf("****pid: dt = %ld\n", dt );

 //   if(dt < (unsigned long int)(t_interval*1000))
//    {   return 0;
//    } 

//P    
   x = _x;
   xerr = xTag - x; //grad

//D   
/*
   dSt.get(_xerr, _t);
   if(dSt.n < NB)
   {   dX = 0.;   
   } else {
      _dft = float(t - _t) / 1000.f; // dt, sec
      dX = (xerr - _xerr) /_dft * 3600; //grad/hour
   }
*/
//   Serial.printf("dSt.n=%d dX=%f t=%ld\n",dSt.n, dX, t ) ;

//   dSt0.calcD(xerr, t, _dft0);
   dSt.calcD(xerr, t, _dft);
//   Serial.printf("====>>  _dft0=%e  _dft=%e diff=%e\n",  _dft0,  _dft,  _dft0 - _dft);
   { 
      dX = _dft * 3600.f* 1000.f;
   Serial.printf("====>> dX=%f\n", dX) ;
   }

//   dSt0.add(xerr, t);
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
  

int dstack::calcD(float xerr, unsigned long int tt, float &diff)
{  int i, ii;
   unsigned long int  t0, dt, tmid, _t;  
   float _d, dmid, _xerr, xm, ym, xm2,xym, _x, _y, b;
   float  _dft, dX; 
//https://www.freecodecamp.org/news/the-least-squares-regression-method-explained/   
//   float d[NB];
//   unsigned long int t[NB];

   Serial.printf("dstack::calcD n =%i ind =%d xerr=%f tt=%ld\n",n, ind, xerr, tt ) ;

      if( n < 2)
      {  diff = 0.f;
         return 0;
      }

//      t0 = t[ind];
//      get(_xerr, _t);
//      Serial.printf("----- _xerr =%f _t = %d t0 %d-------\n", _xerr, _t, t0) ;
//      Serial.printf("-----  xerr =%f tt = %ld  -------\n",  xerr, tt) ;
//      _dft = float(tt - _t) / 1000.f; // dt, sec
//      dX = (xerr - _xerr) /_dft * 3600; //grad/hour
//      Serial.printf("----- dX(*) =%e dX =%f dt = %d-------\n", (xerr - _xerr)/float(tt - _t) ,  dX, tt - _t) ;


   dmid = 0.f;
   tmid = 0;

      for (ii = 0; ii<n; ii++)
      {
         i = ind - n + ii;
         if(i < 0)
            i = i+NB;
         if(ii == 0)
            t0 = t[i]; 
         dmid += d[i];
         tmid += (t[i] - t0); 

      }
      dmid += xerr;
      tmid += (tt - t0); 

  //    Serial.printf("dmid %f tmid %d t0* %d\n", dmid, tmid, t0) ;
      xm = tmid / (n+1);
      ym = dmid / (n+1);
      xm2 = xym = 0;
      for (ii = 0; ii<n; ii++)
      {
         i = ind - n + ii;
         if(i < 0)
            i = i+NB;
         _x = (t[i] - t0) - xm;
         _y = d[i] - ym;
         xm2 += _x * _x;
         xym += _x * _y;
      }   

      _x = (tt - t0) - xm;
      _y = xerr - ym;
      xm2 += _x * _x;
      xym += _x * _y;
      if (xm2 == 0.f)
         return 2;

 //  Serial.printf("xym =%f xm2b = %f\n", xym , xm2) ;
      b = xym / xm2; 
//      Serial.printf("b = %e dt =%d n=%d\n", b, millis()-tt, n );
      diff = b;
   return 1;
}  

#endif //PID_USE