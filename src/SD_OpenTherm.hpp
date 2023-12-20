/* SD_OpenTherm.hpp */
#ifndef SD_OPENTHERM
#define SD_OPENTHERM

#include "SmartDevice.hpp"
#include "mybuffer.hpp"

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

  BoilerStatisic()
  {
      NflameOn = NflameOn_h =  NflameOn_day = NflameOn_h_prev = NflameOn_day_prev = 0;
      t_flame_on = t_flame_off = t_I_last = 0;
       ModIntegral_h = 0.;
       ModIntegral_d = 0.;
       Eff_Mod_h = Eff_Mod_d =  Eff_Mod_h_prev =  Eff_Mod_d_prev = 0.;
  }
  void calcNflame(int newSts);
  void calcIntegral(float flame);

};

class SD_Termo:public SmartDevice
{
public:
  short int stsOT; // -1 not init, 0 - normal work, 2 - timeout
  time_t t_lastwork; // time of last stsOT = 0
  int stsT1;
  int stsT2;
  float t1;
  float t2;
//sizeof(unsigned long)=4
    //Set Boiler Status
    bool enable_CentralHeating;
    bool enable_HotWater;
    bool enable_Cooling;
    bool enable_CentralHeating2;
    bool HotWater_present;
    bool CH2_present;
    bool Toutside_present; 
    bool Pressure_present;
  unsigned int OTmemberCode;
  unsigned long response;
  float Tset;    // Control setpoint  ie CH  water temperature setpoint (°C)
  float Tset_r;  // Temp set from responce
  float Tset2;   // Control setpoint for 2e CH circuit (°C)
  float Tset2_r; // Temp2 set from responce

  float BoilerT; // Boiler flow water temperature (°C) CH
  float BoilerT2; // Boiler CH2 water temperature (°C) CH
  float RetT;    // Return water temperature (°C) CH
	float TdhwSet; // f8.8  DHW setpoint (°C)    (Remote parameter 1)
  float dhw_t;   // DHW temperature (°C)
  float Toutside; 
  float Texhaust;// s16  Boiler exhaust temperature (°C)
  float FlameModulation;
  float Pressure;
  float MaxRelModLevelSetting;
  unsigned int MaxCapacity;
  unsigned int MinModLevel;
  unsigned int Fault;
  unsigned int OEMDcode;
  unsigned int rcode[5];
  int BoilerStatus;
  byte need_set_T; 
  byte need_set_T2; 
  byte need_set_dhwT;
  byte need_write_f; 

  int TestCmd;
  int TestId;
  int TestPar;
  int TestResponse;
  int TestStatus;

  unsigned long RespMillis;
  BoilerStatisic Bstat;

  bool enable_OTlog; //Включаем лог OT
  myBuffer2 OTlogBuf;

  SD_Termo(void)
  {	  
    enable_CentralHeating = true;
    HotWater_present  = false;
    enable_HotWater = true;
    enable_Cooling = false;
    enable_CentralHeating2 = false;
    CH2_present  = false;
    Toutside_present  = false; 
    Pressure_present  = false;

      stsOT = -1;
      t_lastwork = 0;
	    stsT1 = -1;
	    stsT2 = -1;
      t1 = t2 = 0.;
      response = 0;
      BoilerT =  BoilerT2 = 0.;
      Tset = 40.;
      Tset_r = 0.;
      Tset2 = 41.;
      Tset2_r = 0.;

      TdhwSet = 40.;
      need_set_T = 1;
      need_set_T2 = 0;
      need_set_dhwT = 1;
      need_write_f = 0;
      RetT = 0.;
      dhw_t = 0.;
      Toutside = 0.;
      Texhaust = 0.;
      FlameModulation = 0.;
      Pressure = 0.;
      MaxRelModLevelSetting = 0.;
      MaxCapacity = MinModLevel = 0;
      Fault = 0;
      OEMDcode = 0;
      OTmemberCode = 0;
      rcode[0] = rcode[1] = rcode[2] = rcode[3] = rcode[4] = 0;
      BoilerStatus = 0;
      TestCmd = TestId = TestPar =  TestResponse = 0;
      TestStatus = 0;
      RespMillis = 0;
	enable_OTlog = false;
  }
  
  void init(void);
  void loop(void);
  void OpenThermInfo(void);
//  void udp_OpenThermInfo( U8 *bf, unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_Get_OpenThermInfo( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_getdata( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_testcmd( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_testcmdanswer( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_GetOTLog( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

  int Write_data_fs(char *path, uint8_t *dataBuff, int len);
  int Read_data_fs(char *path, uint8_t *dataBuff, int len);
  int Read_ot_fs(void);
  int Write_ot_fs(void);

};

#endif // SD_OPENTHERM