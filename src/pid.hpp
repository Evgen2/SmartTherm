/* pid.hpp */
#ifndef PID_DEFINED
#define PID_DEFINED

#if PID_USE

/* циклический стек/буфер для хранения последних NB значений */
/* нужен для корректного расчета дифференциальной части PID  */
#define NB_MAX 128

class dstack
{
  public:
   float d[NB_MAX];
   unsigned long int t[NB_MAX];
   int ind;
   int n;
   int NB;

   dstack(void)
   {  int i;
      for(i=0; i<NB_MAX; i++) 
      {  d[i] = 0.f;
         t[i] = 0;
      }
      ind = n = 0;
      NB = 16;
  }
  void add (float _d, unsigned long int _t)
  {   d[ind] = _d;
      t[ind] = _t;
      ind++;
      if(ind >= NB) ind = 0;
      if(n < NB) n++;
  }
  
   void get( float &_d,  unsigned long int &_t)
   {  if( n < NB)
      { if(n == 0)
         {  _d = 0.f;
            _t = 0;
         } else {
            _d = d[0];
            _t = t[0];
         }
      } else {
         int i = ind;
         if(i >= NB) i = 0;
         _d = d[i];
         _t = t[i];
      }
   }

};

class TempStack:public dstack
{
  public:
//   int nlast;
//   int ind_last;
   TempStack(void)
   { //  nlast = ind_last = 0;

   }
   void add (float _d, unsigned long int _t)
   { 	if(n == NB)
		{	int ind_last = ind + 1;
			unsigned long int dt;
			int prev = ind-1;
			if(prev < 0)
				prev = NB -1;
			//if(_t == t[prev])
			//	printf("hren\n");
			if(ind_last >= NB)
				ind_last = 0;
			dt = _t - t[ind_last];
			if(dt < 3600*1000)
			{	if(NB < NB_MAX)
				{	int i;
				    for(i = NB; i>ind; i--)
					{ d[i] = d[i-1];
					  t[i] = t[i-1];
					}
					NB++;
				}
			} else if(dt >3780*1000) {
				if(NB > 10 )
				{	int i;				
				    for(i = ind; i<NB-1; i++)
					{ d[i] = d[i+1];
					  t[i] = t[i+1];
					}
					NB--;
				}
			}
		}
      dstack::add(_d, _t);
   }

   unsigned long int get_dt(unsigned long int _t)
   {	int i;
      unsigned long int dt;
      if(n < NB)
      {  dt = _t - t[0];
      } else {
         i = ind + 1;
         if(i >= NB)
            i = 0;
         dt = _t - t[i];
      }
      return dt;
   }

   int calcD(float _d, unsigned long int _t, float &diff);

};

/* PID регулятор */
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
   float Kidiss; // коэффициент диссипации интеграла
   float Ku;
//ПЗА
   float u0; //базовый управляющий сигнал при y = y0;
   float y0;
   float u1; //базовый управляющий сигнал при y = y1;
   float y1;
   float x0; //целевой сигнал для пар u0/y0 и u1/y1
//ограничение
   float dDmax;
   long int pid_t; /* время начала такта */
   TempStack dSt;
//   dstack  dSt0;
   int dt;
   int xChanged;
   long unsigned xChanged_t; /* время изменения */

   pid(void)
   {  Kp = 1.;
      Kd = 0.2;
      Ki = 0.004;
      Ku = 1.;
      x = xTag = 0.;
      y = 0.;
      t_interval = 30;
      Kidiss = 0.010 * t_interval / 60.f;
      u0 = 40.;
      y0 = 10.;
      u1 = 80.;
      y1 = -20.;
      x0 = 20.;
//      u = u0 + (u1 - u0) * (y - y0)/(y1 - y0);  
      x = xerr = 0.;
      dP = dD = dI = 0.;
      InT = 0;
      u = ub = 0;
      dDmax = 50.;
      dt = 0;
      xChanged = 0;
      xChanged_t = 0;
      NextTact();
   }
   void NextTact(void)
   {	pid_t = millis();
   }

   void Pid(float _x, float u0);
   void Set_NewTag( float _NewTag, float _OldTag, float _CurrentT);
};

float  safeFloat(float v); 

#endif //PID_USE
#endif //PID_DEFINED