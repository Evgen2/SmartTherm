/* pid.hpp */
#ifndef PID_DEFINED
#define PID_DEFINED

#if PID_USE
#define NB 10
class dstack
{
  public:
   float d[NB];
   unsigned long int t[NB];
   int ind;
   int n;
   dstack(void)
   {  int i;
      for(i=0; i<NB; i++) 
      {  d[i] = 0.f;
         t[i] = 0;
      }
      ind = n = 0;
  }
  void add (float _d, unsigned long int _t)
  {   d[ind] = _d;
      t[ind] = _t;
      ind++;
      if(ind >= NB) ind = 0;
      if(n < NB) n++;
  }
  
  void get( float &_d,  unsigned long int &_t)
  { int i;
    if( n < NB)
    { if(n == 0)
      {  _d = 0.f;
         _t = 0;
      } else {
         _d = d[n-1];
         _t = t[n-1];
      }
    } else {
      i = ind + 1;
      if(i >= NB) i = 0;
      _d = d[i];
      _t = t[i];
    }

  }
  
};

class pid
{
  public:
   float x;  //  управляемый сигнал (температура)
   float xTag; // целевое значение
   float u;    // управляющий сигнал
   float ub;   // Базовый управляющий сигнал
   float y;    // параметр (внешняя температура)
   float xerr;
   float dP;
   float dD;
   float dI;
   float InT;

  int t_interval; //интервал времени цикла управления, сек
   float Kp;
   float Kd;
   float Ki;
   float umin;
   float umax;
   float Kidiss;

   float u0; //базовый управляющий сигнал при y = y0;
   float y0;
   float u1; //базовый управляющий сигнал при y = y1;
   float y1;

   long int pid_t; /* время начала такта */
   dstack  dSt;
   
   pid(void)
   {  Kp = 1.;
      Kd = 1.;
      Ki = 0.002;
      Kidiss = 0.002;
      x = xTag = 0.;
      t_interval = 120;
      u0 = 40.;
      y0 = 10.;
      u1 = 80.;
      y1 = -30.;
//      u = u0 + (u1 - u0) * (y - y0)/(y1 - y0);  //40* 4/12
	umin = 40;
	umax = 80;
      x = xerr = 0.;
      dP = dD = dI = 0.;
      InT = 0;
      u = ub = 0;
      NextTact();
   }
   void NextTact(void)
   {	pid_t = millis();
   }

   int Pid(float _x, float u0);

};

#endif //PID_USE
#endif //PID_DEFINED