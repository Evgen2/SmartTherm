/* SD_OpenTherm.hpp */
#ifndef SD_OPENTHERM
#define SD_OPENTHERM

#include "SmartDevice.hpp"

class SD_Termo:public SmartDevice
{
public:
  int stsOT;
  int stsT1;
  int stsT2;
  float t1;
  float t2;
//sizeof(unsigned long)=4
  unsigned long response;
  float Tset;
  float Tset_r; //Tset from responce
  float BoilerT;
  float RetT;
  float dhw_t;
  float FlameModulation;
  float Pressure;
  float MaxRelModLevelSetting;
  unsigned int MaxCapacity;
  unsigned int MinModLevel;
  unsigned char Fault;
  unsigned int OEMDcode;
  unsigned int rcode[5];
  int BoilerStatus;
  int need_setT; 
  SD_Termo(void)
  {	  stsOT = -1;
	    stsT1 = -1;
	    stsT2 = -1;
      t1 = t2 = 0.;
      response = 0;
      BoilerT = 0.;
      Tset = 40.;
      Tset_r = 0.;
      need_setT = 1;
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
  }
  
  void loop(void);
  void OpenThermInfo(void);
  void udp_OpenThermInfo( U8 *bf, unsigned char * &MsgOut,int &Lsend, U8 *(*get_buf) (U16 size));

};

#endif // SD_OPENTHERM