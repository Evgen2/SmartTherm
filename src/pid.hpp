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

   TempStack(void)
   { 

   }
   void add (float _d, unsigned long int _t)
   {  if(n == NB)
		{	int ind_last = ind + 1;
			unsigned long int dt;
			int prev = ind-1;
			if(prev < 0)
				prev = NB -1;
			if(ind_last >= NB)
				ind_last = 0;
			dt = _t - t[ind_last];
//Serial.printf("0===>add  ind =%d, dt=%d Nb=%d\n", ind, dt, NB);
			if(dt < PID_D_PERIOD *1000)
			{	if(NB < NB_MAX)
				{	int i;
			       for(i = NB; i>ind; i--)
					{ d[i] = d[i-1];
					  t[i] = t[i-1];
					}
					NB++;              
            }
			} else if(dt >(PID_D_PERIOD+120)*1000) { 
//We will be in this place if we change the fast temperature sensor to a slow one on the fly.
//This situation is possible if the slow sensor from HomeAssistant disappears for some time, 
//the algorithm switches to the fast DS sensor, and then the slow sensor works again.
//Switching the sensor source from the main one in the HA to the built-in DS is todo
				if(NB > 10 )
            {	int i, j;
//Serial.printf("0===>NB--  ind =%d, dt=%d Nb=%d\n", ind, dt, NB);
//               for(i = 0; i<NB; i++)
//                  Serial.printf("%d %f %d\n",i, d[i],t[i]);

               for(j=0; j<10; j++)
               {  if(ind == NB-1)
                  { ind = 0;
                  } else {
                     for(i = ind; i<NB-1; i++)
                     {  d[i] = d[i+1];
                        t[i] = t[i+1];
                     }
                  }
                  NB--;
                  n--;
                  if(NB == 10)
                     break;
                  ind_last = ind + 1;
                  if(ind_last >= NB)
                     ind_last = 0;
                  dt = _t - t[ind_last];
                  if(dt < (PID_D_PERIOD+120)*1000)                   
                     break;
               }
            }
//         { int i;
//            Serial.printf("1===>NB--  ind =%d, dt=%d Nb=%d\n", ind, dt, NB);
//               for(i = 0; i<NB; i++)
//                  Serial.printf("%d %f %d\n",i, d[i],t[i]);
//         }  
#if 0            
				{	int i;
				    for(i = ind; i<NB-1; i++)
					{ d[i] = d[i+1];
					  t[i] = t[i+1];
					}
					NB--;
               n--;
				}
#endif            

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