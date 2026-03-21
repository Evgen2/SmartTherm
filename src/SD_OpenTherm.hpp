/* SD_OpenTherm.hpp */
#ifndef SD_OPENTHERM
#define SD_OPENTHERM

#include "SmartDevice.hpp"

#include "pid.hpp"
#include "mybuffer.hpp"
#include "Planner.hpp"
#include "SD_filter.hpp"
class x_mean
{
  public:
   float x;
   float xmean; //среднее для вычисления x
   float x0, xold;
   int nx;       // число отсчетов xmean
   int isset;
   int canfilter;
   int can_report;
   int index;
   int changed;
   unsigned long t_set;
   fast_safe_filter fsf;

   x_mean(void)
   { x = xold = x0 = 0.f;
     init(0);
     isset = -1;
     canfilter = 0;
     can_report = 0; 
     index = 0;
     t_set = 0;
     changed = 0;
   }

   void init(int canf)
   {  if(canf & 0x01)
      { xold = x;
        canfilter = 1;
      } else {
        canfilter = 0;
      }
      if(canf & 0x02 && isset)
      { xmean = x;
        nx = 1;
      } else {
        xmean = 0;
        isset = 0;
        nx = 0;
      }
      can_report = 1;
//if(index == 3)        
//  Serial_db.printfm(DEBUG_DEFAULT|DEBUG_PID, "init t_mean[%d]  canfilter=%d\n", index, canfilter);

   }
   void add(float _x)
   {  fsf.filter(_x);
      if(fsf.suspect_count == 0)
      { xmean += _x;
        nx++;
        t_set = millis();
        changed = 1;
      }
//if(index == 3)        
//  Serial_db.printfm(DEBUG_DEFAULT|DEBUG_PID, "add t_mean[%d] _x =%f xmean=%f x=%f nx= %d\n", index, _x, xmean, x,  nx);
   }

   float get(void)
   {  if(nx > 0) 
      { x0 = xmean/float(nx);
        if(isset != 1)
        {   x = x0;
            isset = 1;
        } 
        
        if(canfilter)
        { x = (x0 + xold) * 0.5;
        } else {
          x = x0;
        }
//if(index == 3 || index == 4)        
//  Serial_db.printfm(DEBUG_DEFAULT|DEBUG_PID, "t_mean[%d] x0=%f x =%f xmean = %f nx=%d\n", index, x0, x, xmean, nx); 
        xold = x;
      }
      return x;
   }
};

class BoilerStatisic
{
  public:
/* число включений горелки */  
  unsigned int NflameOn; //число включений горелки
  unsigned int NflameOn_h; //число включений горелки за час
  unsigned int NflameOn_day; //число включений горелки за день
  unsigned int NflameOn_h_prev; //число включений горелки за предыдущий час
  unsigned int NflameOn_day_prev; //число включений горелки за предыдущий день
  float ModIntegral_h; //интеграл модуляции с начала часа
  float ModIntegral_d; //интеграл модуляции с начала суток
  float Eff_Mod_h;    // эффективная модуляция за текущий час
  float Eff_Mod_d;    // эффективная модуляция за текущие сутки
  float Eff_Mod_h_prev;    // эффективная модуляция за предыдущий час
  float Eff_Mod_d_prev;    // эффективная модуляция за предыдущие сутки
  time_t t_flame_on;  //время включения горелки
  time_t t_flame_off; //время выключения горелки
  time_t t_I_last;    //предыдущее время подсчета интеграла
  time_t t_HW_on;  //время включения горячей воды
  time_t t_HW_off; //время выключения горячей воды
  unsigned int sec_h; //секунд с начала часа
  unsigned int sec_d; //секунд с начала суток
  BoilerStatisic()
  {
      NflameOn = NflameOn_h =  NflameOn_day = NflameOn_h_prev = NflameOn_day_prev = 0;
      t_flame_on = t_flame_off = t_I_last = 0;
      t_HW_on = t_HW_off = 0;
       ModIntegral_h = 0.;
       ModIntegral_d = 0.;
       Eff_Mod_h = Eff_Mod_d =  Eff_Mod_h_prev =  Eff_Mod_d_prev = 0.;
       sec_h = sec_d = 0;
  }
  void calcNflame(int newSts);
  void calcN_HW(int newSts);
  void calcIntegral(float flame);

};

class SD_Termo:public SmartDevice
{
public:
  short int stsOT; // -1 not init, 0 - normal work, 2 - timeout
  time_t t_lastwork; // time of last stsOT = 0
  int ns_OT, nr_OT; // тест число пакетов посланных и полученных
  int stsT1;
  int stsT2;
  float t1;
  float t2;
//sizeof(unsigned long)=4
    //Set Boiler Status
  bool enable_CentralHeating;     //user set
  #if  PID_USE
  bool enable_CentralHeating_real;//real used. Without PID equal to enable_CentralHeating 
  #endif

  bool enable_HotWater;
  bool enable_Cooling;
  bool enable_CentralHeating2;

  bool HotWater_present;
  bool RetT_present; 
  bool CH2_present;
  bool DHW_tank_present; //DHW configuration: storage tank
  bool Toutside_present; 
  bool Pressure_present;
  bool Dhw_t_present;  //у Buderus'а с косвенным нагревом есть dhw и нет dhw_t
  bool Tstorage_present; // ID29
  bool MaxRelModLevel_present; // ID14  MaxRelModLevelSetting 
  bool RemoteRequest_present; // ID4 present, can be used for BLOR = Boiler Lock-out Reset  
  bool DHWFlowRate_present; // ID19 DHWFlowRate 
#if RELAY_USE  
  bool Relay_present; //Relay present and use
  bool Relay_init_sts; //Relay state at start
  bool Relay_sts;      //Relay state 
#endif  
#if ST_VERS == 2
  bool OT_slave_present; //OT_slave  present and use
  short int OT_slave_mode; /* 0 slave readonly, 1 master readonly */
  short int ot_slave_stsOT; //-2 not initialise,  -1 not init interface, 0 - normal work, 2 - timeout
  time_t ot_slave_t_lastwork; // time of last ot_slave_stsOT = 0
#endif

  unsigned int OTmemberCode;
  unsigned long response;
  short int responseID;
  float Tset;    // ID1 Control setpoint  ie CH  water temperature setpoint (°C)
  float Tset_r;  // Temp set from responce
  float Tset2;   // ID8 TsetCH2: Control setpoint for 2e CH circuit (°C)
  float Tset2_r; // Temp2 set from responce
	float MaxTSet; // f8.8  Max CH water setpoint (°C) (Remote parameters 2) ID57
  float MaxTSetUB; // 49 MaxTSetUBMaxTSetLB:  Max CH water Setpoint upper & lower bounds for adjustment(°C)
  float MaxTSetLB; // -- // --
  float BoilerT;   // Boiler flow water temperature (°C) CH
  float BoilerT2;  // Boiler CH2 water temperature (°C) CH
  float RetT;      // 28 Return water temperature (°C) CH
	float TdhwSet;   // 56 TdhwSet: f8.8  DHW setpoint (°C)    (Remote parameter 1)  
  float dhw_t;     // 26 Tdhw DHW temperature (°C)
  float TdhwSetUB; // 48 TdhwSetUBTdhwSetLBSetpoint DHW Setpoint upper & lower bounds for adjustment(°C)  
  float TdhwSetLB; // -- // --
  float Toutside; 
  float Tstorage; // [Solar] storage temperature (°C)
  float Texhaust; // ID33 s16  Boiler exhaust temperature (°C)
  float FlameModulation; //Relative Modulation Level (%)
  float Pressure; // Water pressure in CH circuit  (bar)
  float MaxRelModLevelSetting; // if MaxRelModLevel_present + need_set_MaxRelModLevel
  float DHWFlowRate; // ID19 Water flow rate in DHW circuit. (litres / minute)
  unsigned int MaxCapacity;
  unsigned int MinModLevel;
  unsigned int Fault;
  unsigned int OEMDcode;
  unsigned int rcode[5];
  int BoilerStatus;
  int BoilerStatusRequest;
  //byte need_set_T2; 
//  byte need_set_MaxRelModLevel;
  byte need_send_Blor;
  byte need_write_f; 
  byte need_report_MQTT_panel; 
  unsigned long t_need_write_config;
//..  byte need_set_MaxTSet;

  int TestCmd;
  int TestId;
  int TestPar;
  int TestResponse;
  int TestStatus;

  unsigned long RespMillis;
  BoilerStatisic Bstat;
#if OT_DEBUGLOG
  bool enable_OTlog; //Включаем лог OT
  int nOTlog; //пакетов в логе
  short int nOT_need_send;//
  int nOTsend;//
  myBuffer2 OTlogBuf;
#endif // OT_DEBUGLOG
  
  
#if MQTT_USE
  byte useMQTT;  //0 = not use, 1 use but not setup, 0x3 - use & setup
  byte stsMQTT;
  int stsMQTTcfg;
 #if defined(ARDUINO_ARCH_ESP8266)
 //20+20+4+10+10+10= 74
  char MQTT_server[20]; 
  char MQTT_topic[20];
  unsigned int  MQTT_interval; //sec
  char MQTT_user[10];
  char MQTT_pwd[10];
  char MQTT_devname[10];
 #elif defined(ARDUINO_ARCH_ESP32)
 //40+40+4+40+20+40=  184 (+110)
  char MQTT_server[80];
  char MQTT_topic[40];
  int MQTT_interval; //sec
  char MQTT_user[40];
  char MQTT_pwd[20];
  char MQTT_devname[40];
 #endif
  unsigned short MQTT_port;  /* MQTT port, default 1883 */
  int MQTT_need_report;
#endif //MQTT_USE
#if PID_USE
  byte usePID; // 1/0 использовать PID да/нет
  signed char srcTroom; // источник температуры в комнате -1 - n/a,  0/1 - T1/T2, 2 - Text, 3,4  MQTT t_indoor/t_outdoor
  signed char srcText;  // источник температуры на улице  -1 - n/a,  0/1 - T1/T2, 2 - Text, 3,4  MQTT  t_indoor/t_outdoor 
  class pid mypid;
  x_mean t_mean[MAX_PID_SRC+1];
  float tempindoor;
  #define TroomTarget mypid.xTag
  float tempoutdoor;
  float _U0start;
  int InTstartset;
  int IsSetTemp; //01 tempIndoor set | 0x02 tempOutdoor set
  int PID_PWMperiod;
  int PID_PWM_sts;
  unsigned long int PID_PWM_t0;
#endif
  int start_sts; //1 - start state, need ask server for last I and U0(?),  &0x02 - OT start log, 0 - not start
  unsigned short int UseID2;
  unsigned short int ID2masterID;
  unsigned short int CH2_DHW_flag;
  unsigned short int UseWinterMode;
  unsigned short int Use_OTC;
  unsigned short int Use_ID29_DHW_flag; // У одноконтурного Будеруса с БКН нужно ставить галку использовать ID29, чтобы температуру воды в бойлере показывал.
  unsigned short int Immergas_fix_flag;
  unsigned short int Use_MaxRelModLevel; 

  //гистерезис включения отопления, минимальная разница между заданной и текущей температурой теплоносителя
  // при которой включится горелка. У Mizudo может быть 15 и больше градусов 
  float CH_StartGist; 

  int CapabilitiesDetected;
  time_t t_lastSetPointChange;
  int  src_lastSetPointChange;
  float oldTroomSetpoint; 
  float umin; //минимальная температура теплоносителя при включенном отоплении
  float MinCHtemp; //минимум температуры теплоносителя
  float umax; //максимальная температура теплоносителя
  planner plan;
  int useCPU_freq; //0 =240, 1=160, 2=80
  int CrasyState_count;
  int needReport_CrasyState;
  Serial_Debug * pSerial_db;

  SD_Termo(void)
  {	  
    enable_CentralHeating = true;
    #if PID_USE
    enable_CentralHeating_real   = enable_CentralHeating;
    #endif
    #if RELAY_USE
      Relay_present = true;
      Relay_init_sts = false;
      Relay_sts = false;
    #endif
    HotWater_present  = false;
    DHW_tank_present  = false;

    Dhw_t_present = false;
    RetT_present  = false;
    CH2_present  = false;
    Toutside_present  = false; 
    Pressure_present  = false;
    Tstorage_present = false;
    enable_HotWater = true;
    enable_Cooling = false;
    enable_CentralHeating2 = false;
    MaxRelModLevel_present = false;
    RemoteRequest_present  = false; 
    DHWFlowRate_present = false; // ID19 DHWFlowRate 
  
    CapabilitiesDetected = 0;

      stsOT = -1;
      ns_OT = nr_OT = 0; 
      t_lastwork = 0;
      t_lastSetPointChange = 0;
	    stsT1 = -1;
	    stsT2 = -1;
      t1 = t2 = 0.;
      response = 0;
      responseID = -1;
      BoilerT =  BoilerT2 = 0.;
      Tset = 40.;
      Tset_r = 0.;
      Tset2 = 41.;
      Tset2_r = 0.;

      TdhwSet = 40.;
/* look at int OpenTherm::update_OTid(int id, int sts) */      
      
      //need_set_T2 = 0;
      need_send_Blor = 0;
//      need_set_MaxTSet = 1;
/********************************/      
      need_write_f = 0;
      t_need_write_config = 0;
      need_report_MQTT_panel = 0;
      RetT = 0.;
      dhw_t = 0.;
      TdhwSetUB = 60.f;
      TdhwSetLB = 30.f;      
      Toutside = 0.;
      Texhaust = 0.;
      Tstorage = 0.;
      FlameModulation = 0.;
      Pressure = 0.;
      DHWFlowRate = 0.; 
      MaxRelModLevelSetting = 100.;
      MaxCapacity = MinModLevel = 0;
      Fault = 0;
      OEMDcode = 0;
      OTmemberCode = 0;
      rcode[0] = rcode[1] = rcode[2] = rcode[3] = rcode[4] = 0;
      BoilerStatus = BoilerStatusRequest = 0;
      TestCmd = TestId = TestPar =  TestResponse = 0;
      TestStatus = 0;
      RespMillis = 0;
#if OT_DEBUGLOG
    	enable_OTlog = false;
      nOTlog = nOTsend = 0;
      nOT_need_send = 0;
#endif            
#if MQTT_USE
      useMQTT = 0;
      stsMQTT = 0;
      stsMQTTcfg = -1;
      strcpy(MQTT_server,"192.168.1.1");
      strcpy(MQTT_topic,"ST");
      strcpy(MQTT_devname,"Boiler");
      MQTT_user[0] = 0;
      MQTT_pwd[0] = 0;
      MQTT_interval = 10; //sec
      MQTT_need_report = 0;
      MQTT_port = 1883;
#endif     
#if PID_USE
      usePID = 0;
      srcTroom =  srcText = 0;
      tempindoor =  tempoutdoor = 0.;
      TroomTarget = 18.f;
      IsSetTemp = 0;
      PID_PWMperiod = 15*60; //15 мин 
      PID_PWM_sts = 0;
      PID_PWM_t0 = 0; 
    _U0start = 0;
    InTstartset = 0;
#endif
      UseID2 = 0;
      ID2masterID = 0;
      CH2_DHW_flag = 0;
      UseWinterMode = 0;
      Use_OTC = 0;
      Use_ID29_DHW_flag = 0;
      Immergas_fix_flag = 0;
      CH_StartGist = 10.f;
      Use_MaxRelModLevel = 0;
      umin = 40;
      MinCHtemp = MIN_CH_TEMP; 
      umax = 80;
      MaxTSet = MAX_CH_TEMP;
      MaxTSetUB = MAX_CH_TEMP;
      MaxTSetLB = umin;
    start_sts = 1;
    oldTroomSetpoint = 0.;
    src_lastSetPointChange = -1;
#if ST_VERS == 2
    OT_slave_present = true;
    OT_slave_mode = 0; /* 0 slave readonly, 1 master readonly */
    ot_slave_stsOT = -2; //-2 not initialise,  -1 not init interface, 0 - normal work, 2 - timeout
    ot_slave_t_lastwork = 0; // time of last ot_slave_stsOT = 0
#endif
    useCPU_freq = -1; //2;
    CrasyState_count = 0;
    needReport_CrasyState = 0;
    pSerial_db = NULL;

    t_mean[0].index = 0;
    t_mean[1].index = 1;
    t_mean[2].index = 2;
    t_mean[3].index = 3;
    t_mean[4].index = 4; //MAX_PID_SRC
//    t_mean[5].index = 5;
//    t_mean[6].index = 6;
//    t_mean[7].index = 7;
  }
  void RelayInit(void);
  void RelayOnOff(bool onoff);
  void init(int src);
  void loop(void);
  void OpenThermInfo(void);
  void Send_to_server_HandShake(void);
  void Send_to_server_IdentifySelf(void);
  void Send_to_server_Sts(unsigned char * &MsgOut, int &Lsend, U8 *(*get_buf) (U16 size));
  void Send_to_server_Sts(void); // PACKED unsigned char * &MsgOut, int &Lsend, U8 *(*get_buf) (U16 size));
#if OT_DEBUGLOG
  void Send_to_server_OTlog(void); 
  int  server_answerOTLog( U8 *bf, int len);
  void Send_to_server_log(void); 
#endif  
  int servercallback_send_Sts_answ( U8 *bf, int len);
  int server_answer_IdentifySelf( U8 *bf, int len);
  void callback_set_tcp_server( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

//  void udp_OpenThermInfo( U8 *bf, unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  int callback_Get_OpenThermInfo( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_Set_OpenThermData( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_Set_State( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

  void callback_getdata( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_testcmd( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_testcmdanswer( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
//  void servercallback_GetOtInfo( U8 *bf, int len);
  int callback_Get_Capabilities( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

  int servercallback_Get_Sts( U8 *bf, int len, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

#if OT_DEBUGLOG
  void callback_GetOTLog( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  int  server_answerLog( U8 *bf, int len);
  
#endif
  int Write_data_fs(char *path, uint8_t *dataBuff, int len, int mode);
  int Read_data_fs(char *path, uint8_t *dataBuff, int len, int &rlen, int mode);
  int Read_ot_fs(void);
  int Write_ot_fs(void);
  int Read_mqtt_fs(void);
  int Write_mqtt_fs(void);

  float CHtempLimit(float _t); /* return t within limit MIN_CH_TEMP MAX_CH_TEMP*/
  float RoomtempLimit(float _t); /* return t within limit MIN_ROOM_TEMP MAX_ROOM_TEMP*/

  void OnChangeT(float t, int src);
  void OnOpenThermRestore(void);
#if PID_USE
  void loop_PID(int mode);
  void loop_mean(void); //получаем средние значения для используемых температур
  int loop_pid_gettemp(int &_start); //получаем значения tindoor и toutdoor
  void set_new_PID_setpoint(float Tsetpoint, int src);
  void loop_pwm(float &u, int &need_heat);
#endif
  void DetectCapabilities(void);
  void handle_SConfigSMemberIDcode(uint16_t u88);
  void planner_setup(void);
  int  planner_loop(void);
  void planner_validate(void);
  void NeedSet(int needId, int nc);
  void Decriment_NeedSet(int needId);
  unsigned int buildRequest(int ot_id);
  void need_set_T(int n);
  void need_set_T_CH2(int n);
  void need_set_dhwT(int n); 
  void need_set_blor(void); 
  void need_set_MaxRelModLevel(int n);
  void need_set_MaxTSet(int n);

};

#endif // SD_OPENTHERM