/* SD_OpenTherm.hpp */
#ifndef SD_OPENTHERM
#define SD_OPENTHERM

#include "SmartDevice.hpp"

class SD_Termo:public SmartDevice
{
public:
  int stsOT; // -1 not init, 0 - normal work, 2 - timeout
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
    bool enableCentralHeating2;

  unsigned long response;
  float Tset;    // Control setpoint  ie CH  water temperature setpoint (°C)
  float Tset_r;  // Temp set from responce
  float Tset2;   // Control setpoint for 2e CH circuit (°C)
  float Tset2_r; // Temp2 set from responce

  float BoilerT; // Boiler flow water temperature (°C) CH
  float RetT;    // Return water temperature (°C) CH
	float TdhwSet; // f8.8  DHW setpoint (°C)    (Remote parameter 1)
  float dhw_t;   // DHW temperature (°C)

  float FlameModulation;
  float Pressure;
  float MaxRelModLevelSetting;
  unsigned int MaxCapacity;
  unsigned int MinModLevel;
  unsigned char Fault;
  unsigned int OEMDcode;
  unsigned int rcode[5];
  int BoilerStatus;
  int need_set_T; 
  int need_set_dhwT;
  int need_write_f; 

  int TestCmd;
  int TestId;
  int TestPar;
  int TestResponse;
  int TestStatus;

  SD_Termo(void)
  {	  
    enable_CentralHeating = true;
    enable_HotWater = true;
    enable_Cooling = false;
    enableCentralHeating2 = false;
    
      stsOT = -1;
      t_lastwork = 0;
	    stsT1 = -1;
	    stsT2 = -1;
      t1 = t2 = 0.;
      response = 0;
      BoilerT = 0.;
      Tset = 40.;
      Tset_r = 0.;
      Tset2 = 41.;
      Tset2_r = 0.;

      TdhwSet = 40.;
      need_set_T = 1;
      need_set_dhwT = 1;
      need_write_f = 0;
      RetT = 0.;
      dhw_t = 0.;
      FlameModulation = 0.;
      Pressure = 0.;
      MaxRelModLevelSetting = 0.;
      MaxCapacity = MinModLevel = 0;
      Fault = 0;
      OEMDcode = 0;
      rcode[0] = rcode[1] = rcode[2] = rcode[3] = rcode[4] = 0;
      BoilerStatus = 0;
      TestCmd = TestId = TestPar =  TestResponse = 0;
      TestStatus = 0;
  }
  
  void loop(void);
  void OpenThermInfo(void);
  void udp_OpenThermInfo( U8 *bf, unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_Get_OpenThermInfo( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_getdata( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_testcmd( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));
  void callback_testcmdanswer( U8 *bf, PACKED unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

  int Write_data_fs(char *path, uint8_t *dataBuff, int len);
  int Read_data_fs(char *path, uint8_t *dataBuff, int len);
  int Read_ot_fs(void);
  int Write_ot_fs(void);
  int Read_udp_fs(void);
  int Write_udp_fs(void);

};

#endif // SD_OPENTHERM