/* Pid.cpp */
#include <Arduino.h>
#include <time.h>
#include "Smart_Config.h"
#if PID_USE
#include "pid.hpp"

float fast_sqrt(float x);

float  safeFloat(float v) 
{ return (isnan(v) || isinf(v)) ? 0.0f : v; };

/* 
 U(Xtag) = U(Xtag0) + Ku*(Xtag-Xtag0)
 U0(Text) = U0(Text0) + K0 * (Text - Text0)
 U0(Text,Tin) = U0(Text0,Tin)  + K0 * (Text - Text0) = U0(Text0,Tin0) + K1 *(Tin - Tin0) + K0 * (Text - Text0) =  U0(Text,Tin0) + K1 *(Tin - Tin0)
 при точном ПЗА
 U0(Text,Xtag) = U(Xtag) 
 U0(Text,Xtag) = U0(Text,Xtag0) + K1 *(Xtag - Xtag0), K1 = Ku

*/

/* 
Выход ПИД в равновесии в зависимости от уcтавки:  U(Xtag) = U(Xtag0) + Ku*(Xtag-Xtag0)
линейное приближение. Тогда
U(Xtag0) = Kp*(X-Xtag0) + Ki*I(Xtag0)
U(Xtag)  = Kp*(X-Xtag) + Ki*I(Xtag) = U(Xtag0) + Ku*(Xtag-Xtag0)
Kp*(X-Xtag) + Ki*I(Xtag) = Kp*(X-Xtag0) + Ki*I(Xtag0) + Ku*(Xtag-Xtag0)
Ki*I(Xtag) = Kp*(Xtag-Xtag0) + Ki*I(Xtag0) + Ku*(Xtag-Xtag0)
I(Xtag) = I(Xtag0) + (Kp + Ku)/Ki*(Xtag-Xtag0)

*/
//  _NewTag,  _OldTag - новая и старая целевая
//  _CurrentT  - текущая
//
void pid::Set_NewTag( float _NewTag, float _OldTag, float _CurrentT)
{  float InTold, InTnew; 
   int sts;
   InTold = InT;

   InTnew = InTold + (Kp + Ku)/Ki * (_NewTag -_OldTag);
   sts = 0;
   if( _OldTag > _CurrentT)  
   {  if(_NewTag > _OldTag)  sts = 1; // старая уставка выше текущей температуры, новая уставка выше старой
      else
      {  if(_NewTag > _CurrentT) sts = 2; // старая уставка выше текущей температуры, новая уставка ниже старой и выше текущей температуры
         else sts = 3;                    // старая уставка выше текущей температуры, новая уставка ниже старой и ниже текущей температуры 
      }
   } else { // старая уставка ниже текущей температуры, 
      if(_NewTag > _OldTag) 
      {  if(_NewTag > _CurrentT) sts = 4; // старая уставка ниже текущей температуры, новая уставка выше старой и выше текущей температуры
         else           sts = 5;          // старая уставка ниже текущей температуры, новая уставка выше старой и ниже текущей температуры
      } else {
         sts = 6; // старая уставка ниже текущей температуры, новая уставка ниже старой
      }
   }

   switch(sts)
   {  case 1: // старая уставка выше текущей температуры, новая уставка выше старой
      {  if(InTold < 0.) // старый интеграл отрицательный, клиент хочет тепла
            InTold = 0.;
         InTnew = InTold + (Kp + Ku)/Ki * (_NewTag -_OldTag);
      }
            break;
      case 2: // старая уставка выше текущей температуры, новая уставка ниже старой и выше текущей температуры
         InTnew = InTold + (Kp + Ku)/Ki * (_NewTag -_OldTag);
         if(InTnew < 0.) // клиент хочет тепла
               InTnew = 0.;
            break;

      case 3:  // старая уставка выше текущей температуры, новая уставка ниже старой и ниже текущей температуры 
         InTnew = InTold + (Kp + Ku)/Ki * (_NewTag -_OldTag);
         if(InTnew > 0.) // клиент не хочет тепла
               InTnew = 0.;
            break;

      case 4: // старая уставка ниже текущей температуры, новая уставка выше старой и выше текущей температуры
         InTnew = InTold + (Kp + Ku)/Ki * (_NewTag -_OldTag);   //_dtag * 2.f/Ki;
         if(InTnew < 0.) // клиент хочет тепла
               InTnew = 0.;
            break;

      case 5: // старая уставка ниже текущей температуры, новая уставка выше старой и ниже текущей температуры
            InTnew = InTold + (Kp + Ku)/Ki * (_NewTag -_OldTag);
            if(InTnew > 0.) // клиент не хочет тепла
                  InTnew = 0.;
               break;

      case 6: // старая уставка ниже текущей температуры, новая уставка ниже старой
      InTnew = InTold + (Kp + Ku)/Ki * (_NewTag -_OldTag);
         if(InTnew > 0.) // клиент не хочет тепла
               InTnew = 0.;
            break;  
   }

   InTnew = safeFloat(InTnew);

   Serial_db.printf("_NewTag %g _OldTag %g  InTnew  %g InTold %g \n", _NewTag, _OldTag, InTnew , InTold);


   if(fabs(_NewTag -_OldTag) > 0.5)
         dSt.n = dSt.ind = 0;

   InT = InTnew;
   xTag = _NewTag;
}


 void pid::Pid(float _x, float _u0)
 {  unsigned long int t;
    static unsigned long int t_d = 0;
    float dX, dtf,  _u;
    static float _dft = 0.f;
    float _Kidiss;
    t  = millis();
    dt = t - pid_t; // dt, msec

//  Serial_db.printf("****pid: dt = %ld\n", dt );

//P    
   x = _x;
   xerr = xTag - x; //grad

//D   

// calcD() - derivative calculation, return _dft
// _dft dimension is grad/msec
   if(xChanged)
   {  if(t - t_d >= t_interval*1000)
      {  dSt.calcD(xerr, t, _dft);
  
         dSt.add(xerr, t);
//      Serial_db.printf("****xChanged pid: t - t_d = %d, t_interval %d\n", t - t_d, t_interval );
         t_d = t;
         xChanged = 0;
      }
   }
   { 
//    dX = _dft * 3600.f* 1000.f; //grad/hour
      dX = _dft * 3.600f; //grad/hour
//   Serial_db.printf("====>> dX=%f\n", dX) ;
   }

//Kidiss magic: dissipation of the integral automagically limit of integral & limiting the influence of old values
//characteristic time: t_interval/Kidiss (sec) 
//Limit for InT with constant  xerr:  InTlim = xerr * t_interval/Kidiss

   _Kidiss = Kidiss;
   
   if (InT * xerr < 0.f)
   { // more dissipation on different signs of InT and xerr
      if(fabs(xerr) < 1.f)
          _Kidiss *= 2.f * fast_sqrt(fabs(xerr));
      else
         _Kidiss *= 2.f * fabs(xerr);
   } else if (fabs(xerr) < 1.f) {
//      _Kidiss *= fabs(xerr); // Limit to zero dissipation of the integral with small xerr
//      _Kidiss *= fast_sqrt(xerr); //??
//      _Kidiss *= xerr*xerr; //??
      _Kidiss *= xerr * fast_sqrt(fabs(xerr));
   }

   if(fabs(InT* Ki) > 40.f) // more dissipation on big InT  
   {   _Kidiss = Kidiss* 2.f;
      if(fabs(InT* Ki) > 80.f)   
         _Kidiss *= 4.f;  
      if (InT * xerr < 0.f)
         _Kidiss *= 2.f;  
   }

   dtf = float(dt) / 1000.f; // dt, sec 
   _Kidiss =  _Kidiss * dtf / float(t_interval); // normalize for time interval

   if(_Kidiss > 0.5) _Kidiss = 0.5;

   InT = safeFloat(InT * (1.f - _Kidiss) + xerr * dtf); // grad * sec

#if SERIAL_DEBUG 
//   Serial_db.printf("pid: dt %d xerr=%f, InT=%f dX=%f\n",
//          dt , xerr, InT, dX); 
//   Serial_db.printf("pid: _x %f xTag=%f, u0=%f\n",
//          _x , xTag, _u0); 
#endif          
   dP = xerr * Kp; //dP - grad, Kp - dimensionless 
   
   dD = dX * Kd;   //dD - grad, Kd - hour
   if(dD > dDmax) dD = dDmax;
   else if (dD < -dDmax) dD = -dDmax;

   dI = InT * Ki;  //dI - grad, Ki - (1/sec)
   _u = dP + dD + dI;

   if(dSt.n <= 4)
         u = _u + _u0;
   else
         u = (u  +  _u + _u0) * 0.5; //filter output of pid

//   Serial_db.printfm(DEBUG_DEFAULT|DEBUG_PID, "pid: U= %.3f u0 = %.3f _u = %.3f dP=%.3f dD=%.3f dI=%.3f x=%.3f Xtag=%.3f dt=%d\n",
//        u, _u0, _u , dP, dD, dI, x, xTag, dt); 

#if SERIAL_DEBUG 
//   Serial_db.printf("pid: U= %f u0 = %f _u = %f dP=%f, dD=%f dI=%f\n",
//        u, _u0, _u , dP, dD, dI); 
#endif         
   ub = _u0;

   NextTact();

 }

#define N_X 3

int IncrCalculateMatrixYfX2(float x, float y, int *Np);
int CalculateMNKYfX2(float coeff[],int *Np);

int TempStack::calcD(float xerr, unsigned long int tt, float &diff)
{  int i=0, ii;
   unsigned long int  t0=0, tmid;  
   float dmid, xm, ym;
   int Np;
   float coeff[N_X];
const float NormC = 1000000.;

  //https://www.freecodecamp.org/news/the-least-squares-regression-method-explained/   

//  Serial_db.printf("TempStack::calcD n =%i ind =%d xerr=%f tt=%ld\n",n, ind, xerr, tt ) ;

//    if( n < 2)
      if( n < 4)
      {  diff = 0.f;
         return 0;
      }

      if( get_dt(tt) < 360 * 1000)
      {  diff = 0.f;
         return 0;
      }

//      t0 = t[ind];
//      get(_xerr, _t);
//      Serial_db.printf("----- _xerr =%f _t = %d t0 %d-------\n", _xerr, _t, t0) ;
//      Serial_db.printf("-----  xerr =%f tt = %ld  -------\n",  xerr, tt) ;
//      _dft = float(tt - _t) / 1000.f; // dt, sec
//      dX = (xerr - _xerr) /_dft * 3600; //grad/hour
//      Serial_db.printf("----- dX(*) =%e dX =%f dt = %d-------\n", (xerr - _xerr)/float(tt - _t) ,  dX, tt - _t) ;
//unsigned long int Tm0;
//Tm0 = micros();

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
  //    Serial_db.printf("dmid %f tmid %d t0* %d\n", dmid, tmid, t0) ;
      xm = tmid / (n+1);
      ym = dmid / (n+1);
#if 0      
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
//    Serial_db.printf("[%d] d %f dt %d\n", ii, d[i], t[i] - t0) ;
      }   

      b = xym / xm2; 
//    Serial_db.printf("[%d] d %f dt %d b=%e\n", n, xerr, tt - t0, b) ;
      _x = (tt - t0) - xm;
      _y = xerr - ym;
      xm2 += _x * _x;
      xym += _x * _y;
      if (xm2 == 0.f)
         return 2;

 //  Serial_db.printf("xym =%f xm2b = %f\n", xym , xm2) ;
      b = xym / xm2; 
  //    Serial_db.printf("b = %e dt =%d n=%d\n", b, tt- t0, n );
      diff = b;
#endif

      Np =0;
      for(i = 0; i < n; i++)
      {  IncrCalculateMatrixYfX2(((t[i] - t0) - xm)/NormC, d[i] - ym, &Np);
      }
      IncrCalculateMatrixYfX2( ((tt - t0) - xm)/NormC, xerr - ym, &Np);
         
      CalculateMNKYfX2(coeff,&Np);
//    Serial_db.printf("MNK coeff = %e %e  %e ", coeff[0], coeff[1], coeff[2] );
/* Y = a + b * X + c * X**2                */
/* Y' = b + 2c * X */
      {  float ydf;
         ydf = coeff[1] + 2* coeff[2] * ((tt - t0) - xm)/NormC;
      diff = ydf;

      }

//    Serial_db.printf("MNK coeff Y' = %e n=%d dt=%ld\n", diff, n, micros()-Tm0 );

   return 1;
}  


static float XX[N_X][N_X],XXM[N_X][N_X],XX_1[N_X][N_X],Yx[N_X],YxM[N_X];
int MatrixInvert(int n, float A[N_X][N_X], float Out[N_X][N_X]);


int IncrCalculateMatrixYfX2(float x, float y, int *Np, float _XX[N_X][N_X ], float _Yx[N_X] )
{  int i,j;
   double x2;
 //  n = 3;
   x2 = x * x;
   if(*Np == 0)
   {  for(i=0;i<3;i++)
      {  for(j=0;j<3;j++) _XX[i][j] = 0.f;
         _Yx[i] = 0.f;
      }
      _XX[0][0] = 1.;
   }
   _XX[1][0] += x;
   _XX[1][1] += x2;
   _XX[2][2] += x2 * x2;
   _XX[2][1] += x2 * x;

   _Yx[0]    += y;
   _Yx[1]    += y * x;
   _Yx[2]    += y * x2;
   (*Np)++;
   return 0;
}

/* инкpиментальный подсчет матpицы для МНК */
/* Y = a + b * X + c * X**2                */
/* Np - число точек в статистике           */
/* Np = 0 - обнуление матpиц               */

int IncrCalculateMatrixYfX2(float x, float y, int *Np)
{
   return IncrCalculateMatrixYfX2(x,y, Np, XX,  Yx );
}

/* pасчитать коэффициенты Y= a + b * X + c * X**2 */
int CalculateMNKYfX2(float coeff[],int *Np, float _XX[N_X][N_X], float _Yx[N_X] )
{   int i,j,n;
    float v;
    n = 3;
   if(*Np <= 0) return 1;
   v = 1./ float(*Np);
/* пеpеписываем матpицы в осpедненном виде */
   for(i=0;i<3;i++)
       YxM [i] = _Yx[i] * v;

   XXM[0][0] = 1.;
   XXM[1][0] = _XX[1][0] * v;
   XXM[0][1] = XXM[1][0];
   XXM[1][1] = _XX[1][1] * v;
   XXM[2][2] = _XX[2][2] * v;

   XXM[2][0] = XXM[1][1];
   XXM[0][2] = XXM[2][0];
   XXM[2][1] = _XX[2][1] * v;
   XXM[1][2] = XXM[2][1];

  if(XXM[1][1] == 0.)
      Serial_db.printf("CalculateMNKYfX2 coeff XXM[1][1] = 0\n");
  if(XXM[2][2] == 0.)
      Serial_db.printf("CalculateMNKYfX2 coeff XXM[2][2] = 0\n");

/* считаем обpатную */
  MatrixInvert(n,XXM,XX_1);
/* вычисляем коэффициенты */
  for(i=0;i<n;i++)
  {
    coeff[i]=0.;
    for(j=0;j<n;j++)
    { coeff[i] += XX_1[i][j] * YxM[j];
    }
  }
  return 0;
}

int CalculateMNKYfX2(float coeff[],int *Np)
{
   return CalculateMNKYfX2(coeff,Np,XX,Yx);
}

int MatrixInvert(int n, float A[N_X][N_X], float Out[N_X][N_X])
{  int i,j,k;
   float d,mulby;
   float B[N_X][N_X];
   for(i=0;i<n;i++)
     for(j=0;j<n;j++) { Out[i][j] = 0; B[i][j] = A[i][j]; };
   for(i=0;i<n;i++)   Out[i][i] = 1.;
/*   Matrix Out(1.), B = A; */

   for(i=0;i<n;i++)
   {
      d = B[i][i];
      if(d != 1.0 && d != 0.)
      {    d = 1.f/d;
           for(j=0;j<n;j++)
           {  Out[i][j] *= d;
              B[i][j]  *= d;
           }
      }

      for(j=0;j<n;j++)
      {
          if(j != i)
          {  if(B[j][i] != 0.0)
             {   mulby = B[j][i];
                 for(k=0;k<n;k++)
                 {  B[j][k] -= mulby * B[i][k];
                    Out[j][k] -= mulby * Out[i][k];
                 }
             }
          }
      }
   }
   return 0;
}

float fast_small_sqrt(float x)
{  float xx, sq;
   
   if(x > 0.5) // x = 1 + xx; sqrt(1 + xx) = 1 + xx/2 - (xx*xx)/8 ....
   {  xx = x - 1.f;
      sq = 1.f + xx * (0.5f - xx * 0.125f);
   } else {  //sqrt(x) = 1  + (x-1)/2 - (x-1)^2/8
      xx = x - 1.f;
      sq = 1.f + xx * (0.5f - xx * 0.125f);
   }

   return sq;
}

union {
    float f;
    int i;
} pun;

//https://github.com/itchyny/fastinvsqrt
//about 13.6 times faster than sqrtf in esp32
float fast_sqrt(float x)
{
    float xhalf = 0.5f * x;
//original
//    int i = *((int*)&x);            // Bit-level access
//    i = 0x5f3759df - (i >> 1);    // The "Magic Number"
//eleminate strict-aliasing warning
    pun.f = x;
    pun.i = 0x5f3759df - (pun.i >> 1);    
//original
//    float y = *(float*)&i;        // Back to float (1/sqrt(x) approx)
//eleminate strict-aliasing warning
    float y = pun.f;
    y = y * (1.5f - xhalf * y * y); // One Newton-Raphson iteration
    return x * y;                 // sqrt(x) = x * (1/sqrt(x))
}

#endif //PID_USE