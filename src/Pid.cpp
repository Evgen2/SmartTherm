/* Pid.cpp */
#include <Arduino.h>
#include <time.h>
#include "Smart_Config.h"
#if PID_USE
#include "pid.hpp"

void  pid::Set_NewTag( float _Tag, float _x)
{  float dtag, _xerr, _xerrnew;
   
//   Serial.printf("**** Set_NewTag: xTag = %f I = %f I*Ki=%f\n", xTag, InT, InT * Ki );
   _xerr = xTag - _x;
   _xerrnew = _Tag - _x;
   dtag = _Tag - xTag;
   xTag = _Tag;
   if(fabs(dtag) > 0.1)
   {  //Init_I(_x);
      Init_I(dtag,  _xerrnew);

//   Serial.printf("**** Set_NewTag: dtag = %f xerr =%f xerrnew =%f\n", dtag, _xerr, _xerrnew);
//   Serial.printf("**** Set_NewTag: xTag = %f I = %f I*Ki=%f\n", xTag, InT, InT * Ki );

      dSt.n = dSt.ind = 0;
      dSt.nlast = dSt.ind_last = 0;
   }
}

/* 
U =  Kp*(Xtag-X) + Ki*I
Xtag ->Xtagnew = Xtag + _dtag, I -> Inew = I + di
di = coeff * _dtag

*/
void pid::Init_I(float _dtag, float _xernew)
{  float di, I1;
   if(Ki == 0.)
         return;
   di = _dtag * 2.f/Ki;
   if((_xernew > 0.f && InT < 0.f) || (_xernew < 0.f && InT > 0.f))
         InT = 0.f;
   I1 = InT + di;
   if(I1 * Ki > 30.f )
      I1 = 30.f/Ki;
   else
      if(I1 * Ki < -30.f )
            I1 = -30.f/Ki;
   InT = I1;
}

void pid::Init_I(float _x)
{  float  _xerr, I0, I1;
   if(Ki == 0.)
         return;
//Limit for InT with constant  xerr:  xerr * dt/Kidiss  
   _xerr = xTag - _x;
   I0 = _xerr * (t_interval) /Kidiss * 0.5;
   I1 = I0 * Ki;
   if(I1 > 30.f)
   {  I0 = 30.f / Ki;
   } else if(I1 < -30.f) {
     I0 = -30.f / Ki;
   }
   InT = I0;
   
//   Serial.printf("**** Init_I _x =%f, _xerr %f InT %f InT * Ki %f\n", _x, _xerr, InT, InT * Ki ); 

}

 int pid::Pid(float _x, float _u0)
 {  unsigned long int t, dt, _t;
    float _xerr, dX, dtf, _dft, _u;
    float _dft0, _Kidiss;
    t  = millis();
    dt = t - pid_t; // dt, msec

//      Serial.printf("****pid: dt = %ld\n", dt );

 //   if(dt < (unsigned long int)(t_interval*1000))
//    {   return 0;
//    } 

//P    
   x = _x;
   xerr = xTag - x; //grad

//D   

// calcD() - derivative calculation, return _dft
// _dft dimension is grad/msec
   dSt.calcD(xerr, t, _dft);
//   Serial.printf("====>>  _dft0=%e  _dft=%e diff=%e\n",  _dft0,  _dft,  _dft0 - _dft);
   { 
      dX = _dft * 3600.f* 1000.f; //grad/hour
//   Serial.printf("====>> dX=%f\n", dX) ;
   }

   dSt.add(xerr, t);

//Kidiss magic: dissipation of the integral automagically limit of integral & limiting the influence of old values
//characteristic time: t_interval/Kidiss (sec) 
//Limit for InT with constant  xerr:  InTlim = xerr * t_interval/Kidiss  

   _Kidiss = Kidiss;
   if(fabs(xerr) < 1.f)              //Kidiss magic, part 2:
   {  _Kidiss = Kidiss* fabs(xerr);  //Limit to zero dissipation of the integral with small xerr
   }
   dtf = float(dt) / 1000.f; // dt, sec
   _Kidiss =  _Kidiss * dtf / float(t_interval);
   if(_Kidiss > 0.5) _Kidiss = 0.5;

   InT = InT * (1.f - _Kidiss) + xerr * dtf; // grad * sec
#if SERIAL_DEBUG 
//   Serial.printf("pid: dt %d xerr=%f, InT=%f dX=%f\n",
//          dt , xerr, InT, dX); 
//   Serial.printf("pid: _x %f xTag=%f, u0=%f\n",
//          _x , xTag, _u0); 
#endif          
   dP = xerr * Kp; //dP - grad, Kp - dimensionless 
   dD = dX * Kd;   //dD - grad, Kd - hour
   dI = InT * Ki;  //dI - grad, Ki - (1/sec)
   _u = dP + dD + dI;
   u = _u + _u0;

#if SERIAL_DEBUG 
//   Serial.printf("pid: U= %f u0 = %f _u = %f dP=%f, dD=%f dI=%f\n",
//        u, _u0, _u , dP, dD, dI); 
#endif         
   ub = _u0;

   NextTact();

    return 1;
 }

#define N_X 3

int IncrCalculateMatrixYfX2(float x, float y, int *Np);
int CalculateMNKYfX2(float coeff[],int *Np);

int dstack::calcD(float xerr, unsigned long int tt, float &diff)
{  int i, ii;
   unsigned long int  t0, dt, tmid, _t;  
   float _d, dmid, _xerr, xm, ym, xm2,xym, _x, _y, b;
   float  _dft, dX; 
   int Np;
   float coeff[N_X];

   //https://www.freecodecamp.org/news/the-least-squares-regression-method-explained/   
//   float d[NB];
//   unsigned long int t[NB];

//   Serial.printf("dstack::calcD n =%i ind =%d xerr=%f tt=%ld\n",n, ind, xerr, tt ) ;

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
//    Serial.printf("[%d] d %f dt %d\n", ii, d[i], t[i] - t0) ;
      }   

      b = xym / xm2; 
//    Serial.printf("[%d] d %f dt %d b=%e\n", n, xerr, tt - t0, b) ;
      _x = (tt - t0) - xm;
      _y = xerr - ym;
      xm2 += _x * _x;
      xym += _x * _y;
      if (xm2 == 0.f)
         return 2;

 //  Serial.printf("xym =%f xm2b = %f\n", xym , xm2) ;
      b = xym / xm2; 
  //    Serial.printf("b = %e dt =%d n=%d\n", b, tt- t0, n );
      diff = b;
#endif
      Np =0;
      for(i = 0; i < n; i++)
      {  IncrCalculateMatrixYfX2((t[i] - t0) - xm, d[i] - ym, &Np);
      }
      IncrCalculateMatrixYfX2( (tt - t0) - xm, xerr - ym, &Np);
         
      CalculateMNKYfX2(coeff,&Np);
//      Serial.printf("MNK coeff = %e %e  %e\n", coeff[0], coeff[1], coeff[2] );
/* Y = a + b * X + c * X**2                */
/* Y' = b + 2c * X */
      {  float ydf;
         ydf = coeff[1] + 2* coeff[2] * ((tt - t0) - xm);
//      Serial.printf("MNK coeff Y' = %e\n", ydf );
      diff = ydf;

      }
   return 1;
}  


static float XX[N_X][N_X],XXM[N_X][N_X],XX_1[N_X][N_X],Yx[N_X],YxM[N_X], Yy[N_X],A[N_X];
int MatrixInvert(int n, float A[N_X][N_X], float Out[N_X][N_X]);


int IncrCalculateMatrixYfX2(float x, float y, int *Np, float _XX[N_X][N_X ], float _Yx[N_X] )
{  int i,j,n;
   double x2;
   n = 3;
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
   v = 1./ double(*Np);
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
      if(d != 1.0 )
      {    for(j=0;j<n;j++)
           {  Out[i][j]/= d;
              B[i][j]  /= d;
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

#endif //PID_USE