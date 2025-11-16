/* Planner.cpp */

#include <Arduino.h>

#include "SmartDevice.hpp"
#include "Smart_Config.h"

#include "OpenTherm.h"
#include "SD_OpenTherm.hpp"

void SD_Termo::planner_setup(void)
{   int rc;
	int _outside_fun = 0;
#if PID_USE
	if(srcTroom == 2)
			_outside_fun = 1;
#endif
    rc = plan.add(1, OpenThermMessageID::Status,				MODE_START|MODE_IDLE|MODE_CH|MODE_HW, 0); // 0
    rc = plan.add(1, OpenThermMessageID::SConfigSMemberIDcode,	MODE_START,0); //3

/*  некоторым котлам (например, Baxi Fourtech/Luna 3) не нравится OpenThermMessageID::MConfigMMemberIDcode
    настолько, что они перестают отвечать на запросы
    UseID2 || Immergas_fix_flag - использовать MConfigMMemberIDcode (if SmOT.UseID2 with code SmOT.ID2masterID)
*/
    if(UseID2 || Immergas_fix_flag)
        rc = plan.add(2, OpenThermMessageID::MConfigMMemberIDcode,	MODE_START,0); //2 из конфига

    rc = plan.add(1, OpenThermMessageID::TdhwSetUBTdhwSetLB,	MODE_START,0); //48
    rc = plan.add(1, OpenThermMessageID::MaxTSetUBMaxTSetLB,	MODE_START,0); //49
	if(Use_MaxRelModLevel)
    	rc = plan.add(2, OpenThermMessageID::MaxRelModLevelSetting, MODE_START|MODE_CH,0); //14 (**)
    rc = plan.add(2, OpenThermMessageID::RemoteRequest,		MODE_START|MODE_CH|MODE_HW,0); //4 (**) (*)
    rc = plan.add(1, OpenThermMessageID::MaxCapacityMinModLevel,	MODE_START,0); //15
	

	rc = plan.add(1, OpenThermMessageID::Tboiler,				MODE_CH|MODE_IDLE,	MODE_HW);	//25
    rc = plan.add(1, OpenThermMessageID::Tret,					MODE_CH,			MODE_HW|MODE_IDLE);	//28
	rc = plan.add(1, OpenThermMessageID::Tdhw,					MODE_HW,			MODE_CH|MODE_IDLE);	//26
	rc = plan.add(1, OpenThermMessageID::RelModLevel,			MODE_HW|MODE_CH,	MODE_IDLE);			//17
	rc = plan.add(1, OpenThermMessageID::CHPressure,			MODE_IDLE,			MODE_CH|MODE_HW);	//18 (*)
	rc = plan.add(2, OpenThermMessageID::TSet,					MODE_CH,			MODE_HW|MODE_IDLE);	//1 (set if need **)
	rc = plan.add(2, OpenThermMessageID::TsetCH2,				MODE_CH,			MODE_HW|MODE_IDLE);	//8 (set if need **)
	
	rc = plan.add(2, OpenThermMessageID::TdhwSet,				MODE_CH,			MODE_HW);	//56 (**)
	rc = plan.add(2, OpenThermMessageID::MaxTSet,				MODE_START|MODE_CH,		0);	//57 
	rc = plan.add(1, OpenThermMessageID::TflowCH2,				MODE_CH,			MODE_HW);	//31 (*)
	if(_outside_fun)
		rc = plan.add(1, OpenThermMessageID::Toutside,			MODE_CH|MODE_IDLE,	MODE_HW);	//27 
	else
		rc = plan.add(1, OpenThermMessageID::Toutside,				0,				MODE_CH|MODE_HW|MODE_IDLE);	//27 
	rc = plan.add(1, OpenThermMessageID::Texhaust,					0,				MODE_CH|MODE_HW|MODE_IDLE);	//33 
	if(Use_ID29_DHW_flag)
		rc = plan.add(1, OpenThermMessageID::Tstorage,				0,				MODE_CH|MODE_HW|MODE_IDLE);	//29 (*)
    rc = plan.add(1, OpenThermMessageID::TrSet,					MODE_CH,			0);	//16 (*)
	rc = plan.add(1, OpenThermMessageID::Tr,					MODE_CH,			0);	//24 (*)
	rc = plan.add(1, OpenThermMessageID::DHWFlowRate,			MODE_HW,		MODE_CH);	//19 
	rc = plan.add(1, OpenThermMessageID::ASFflags,				MODE_ERROR,	MODE_CH|MODE_HW|MODE_IDLE);	//5  (*)
	rc = plan.add(1, OpenThermMessageID::OEMDiagnosticCode,		MODE_ERROR,	0);	//115  (*)

    if(rc == 0)
    {	Serial_db.printf("Error: increase NUM_PLAN %d\n", NUM_PLAN);
    }
 
    plan.SetMode(MODE_START);
//    Serial_db.printf("planner_setup Ok %d\n", plan.n);

}

void SD_Termo::planner_validate(void)
{	int i, rc;
	int count, countok;
	extern OpenTherm ot;
	for(i=0;i<plan.n;i++)
	{	rc =  ot.Get_OTid_count((OpenThermMessageID)plan.it[i].cmd, count, countok);
		if((rc == 1 || (count > 0 &&  countok > 0)) && plan.it[i].type > 0)
		{	plan.it[i].type |= 0x10;
			ot.SetUsed_OTid((OpenThermMessageID)plan.it[i].cmd, 1);			
		} else {
			plan.it[i].type = 0;
			ot.SetUsed_OTid((OpenThermMessageID)plan.it[i].cmd, 0);			
		}
	}
	CapabilitiesDetected = 2;
	DetectCapabilities();
	count = 0;
	for(i=0;i<plan.n;i++)
	{	if(plan.it[i].type & 0x10)
		{	plan.it[count] = plan.it[i];
			plan.it[count].type &= ~0x10; 
			count++;
		}
	}
//	Serial_db.printf("planner validate: %d -> %d\n", plan.n, count);
	plan.n = count;
}

void SD_Termo::need_set_T(int n)
{	NeedSet(OpenThermMessageID::TSet, n);
}
void SD_Termo::need_set_dhwT(int n)
{	NeedSet(OpenThermMessageID::TdhwSet, n);
}
void SD_Termo::need_set_blor(void)
{	need_send_Blor = 1;
	NeedSet(OpenThermMessageID::RemoteRequest, 1);
}

void SD_Termo::need_set_MaxRelModLevel(int n)
{	NeedSet(OpenThermMessageID::MaxRelModLevelSetting, n);
}

void SD_Termo::need_set_T_CH2(int n)
{	NeedSet(OpenThermMessageID::TsetCH2, n);
}
void SD_Termo::need_set_MaxTSet(int n)
{	if(MaxTSet > MAX_CH_TEMP)
		MaxTSet = MAX_CH_TEMP;
	else if(MaxTSet < MIN_CH_TEMP)
		MaxTSet = MIN_CH_TEMP;

	NeedSet(OpenThermMessageID::MaxTSet, n);
}

void SD_Termo::NeedSet(int needId, int nc)
{	int i;
	for(i=0; i<plan.n; i++)
	{	if(needId == plan.it[i].cmd)
		{	if(plan.it[i].type & 0x02)
			{	plan.it[i].count = nc;
				break;
			}
		}
	}
}

void SD_Termo::Decriment_NeedSet(int needId)
{	int i;
	for(i=0; i<plan.n; i++)
	{	if(needId == plan.it[i].cmd)
		{	if(plan.it[i].type & 0x02)
			{	if(plan.it[i].count > 0)
				{	plan.it[i].count--;
					break;
				}
			}
		}
	}
}
void SD_Termo::handle_SConfigSMemberIDcode(uint16_t u88)
{
	OTmemberCode = u88 & 0xff;

	if(u88 & 0x100)
	{ HotWater_present = true;
	} else {
	  HotWater_present = false;
	  enable_HotWater = false;
	}
	if(u88 & 0x2000)
	{   CH2_present  = true;
		plan.set_used(OpenThermMessageID::TsetCH2, 2); 
		plan.set_used(OpenThermMessageID::TflowCH2, 1); 
	} else {
		CH2_present  = false;
		enable_CentralHeating2  = false;
		plan.set_used(OpenThermMessageID::TsetCH2, 0); 
		plan.set_used(OpenThermMessageID::TflowCH2, 0); 
	}
	if(u88 & 0x800) //DHW configuration: storage tank
	{   DHW_tank_present  = true;
	} else {
		DHW_tank_present  = false;
	}
 	
	if(Use_OTC || OTmemberCode == 248) 
	{
		plan.set_used(OpenThermMessageID::Tr, 1); 
		plan.set_used(OpenThermMessageID::TrSet, 1); 

	} else {
		plan.set_used(OpenThermMessageID::Tr, 0); 
		plan.set_used(OpenThermMessageID::TrSet, 0); 
	}
}

int SD_Termo::planner_loop(void)
{   int rc,ind, lev;

    if(plan.mask == MODE_START)
    {   if(plan.step == 1 && responseID  == -1)
			plan.SetMode(MODE_START);

		ind = plan.run(lev, 0);
		if(plan.sts & 0x01)
        { //  Serial_db.printf("planner MODE_START end\n");
            plan.SetMode(MODE_TEST);
			//t0 = millis();
            goto M_TEST;
        }
		plan.step++;

        rc = plan.it[ind].cmd;

	} else 	if(plan.mask == MODE_TEST) {
M_TEST:	ind = plan.run(lev, 0);

		if(plan.sts & 0x01)
		{  if(plan.step >= plan.n*2)
			{	extern unsigned int OTDebugInfo[12];
				int v, n;
				if(OTDebugInfo[0] > 10)
				{	v =  (OTDebugInfo[3] + OTDebugInfo[4])*100/OTDebugInfo[0]; 
					n = plan.n*4; // при большом количестве ошибок увеличиваем время теста
					if(v > 30)
							n = plan.n*8;
//					Serial.printf("planner error ratio %d\n", v);
					if( v < 10 || plan.step >= n)
					{   //			 Serial_db.printf("planner MODE_TEST end\n");
						planner_validate();
						plan.SetMode(MODE_IDLE);
						goto M0;
					}
				}
			}
			plan.sts = 0;
			plan.count0 = plan.ind[0] = plan.vind[0] = 0;
        }
		plan.step++;

//        Serial_db.printf("plan test %2d %2d %d %d\n",  plan.it[ind].cmd, lev, plan.step, millis() - t0);
        rc = plan.it[ind].cmd;

    } else {
M0:     
		switch(plan.mask & (MODE_IDLE|MODE_CH|MODE_HW))
		{	case MODE_IDLE: // IDLE -> HW || CH
			
		if(HotWater_present && (BoilerStatus & 0x04))
			plan.SetMode(MODE_HW);
		else  if(
#if PID_USE
		enable_CentralHeating_real
#else 
		enable_CentralHeating
#endif
		)	plan.SetMode(MODE_CH);
		else if(HotWater_present && (BoilerStatus & 0x08)) 
			plan.SetMode(MODE_HW); //HW off, CH off, Flame on ==> indirect heating boiler heat on
				break;

			case MODE_CH: // CH -> HW | IDLE
		if(HotWater_present && (BoilerStatus & 0x04))
            plan.SetMode(MODE_HW);
		else if(!
#if PID_USE
				enable_CentralHeating_real
#else 
				enable_CentralHeating
#endif
				)	plan.SetMode(MODE_IDLE);
		
				break;

			case MODE_HW: //HW -> CH | IDLE

		if( !(BoilerStatus & 0x04))
		{
			if(
				#if PID_USE
				enable_CentralHeating_real
				#else 
				enable_CentralHeating
				#endif
			  )
				plan.SetMode(MODE_CH);
			else
				plan.SetMode(MODE_IDLE);
		}
				break;
		} //endof switch()

		if(BoilerStatus & 0x01) //fault
		plan.mask |= MODE_ERROR;

		ind = plan.run(lev, 1);
 //       Serial_db.printf("plan (%x) %2d t%2d c%2d lv%2d\n", plan.mask,  plan.it[ind].cmd,plan.it[ind].type, plan.it[ind].count, lev);

        rc = plan.it[ind].cmd; 

//		Serial_db.printf("plan(%x)  %2d %2d\n", plan.mask,  plan.it[ind].cmd, lev);

    }

	responseID = -1;
    return rc;
}

int planner::run(int &lev, int mode)
{	int rc=0, rc0, rc1, vrc0=0, vrc1;
M0:
	rc0 = find_next(0, vrc0);
//	if(mask == MODE_TEST)
//	Serial_db.printf("find_next rc0 = %d vrc0=%d\n", rc0,vrc0);

	if(rc0 < 0)
	{	if(mode == 0)
		{	sts |= 0x01;
			return 0;	
		}
		rc1 = find_next(1, vrc1);
		if(rc1 < 0)
		{	ind[1] = -1;
			sts |= 0x02;
			rc1 = find_next(1, vrc1);
			if(rc1 < 0)
			{	vind[0] += ind[0]+1;
				if(vind[0] >= n)
				{	sts |= 0x01;
					vind[0] -= n;
				}
				count0 = 0;
				ind[0] = -1;
				goto M0;
			}
		} 
		rc = vrc1; //rc1; // it[rc1].cmd;
		lev = 1;
		ind[1] = rc1;
		vind[0] += ind[0]+1;
		if(vind[0] >= n)
		{	sts |= 0x01;
			vind[0] -= n;
		}
		count0 = 0;
		ind[0] = -1;
	} else {
		rc = vrc0; // rc0; //it[rc0].cmd;
		lev = 0;
		count0++;
		ind[0] = rc0;
		sts &= ~0x02;
	}
	return rc;
}

int planner::find_next(int level, int &indrc)
{	int ii, vii;

	if(level == 0)
	{	ii = ind[0]+1;
//		if(ind[0] == n0 -1)
		if(count0 == n0)
			return -1;
		for(; ii<n;  ii++) 
		{	vii = vind[0] + ii; 
			if(vii >=n) 
				vii -= n;
			if((it[vii].mask[0] & mask ) && (it[vii].type  > 0))
			{	if(it[vii].type & 0x02) 
				{	if(it[vii].count > 0)
					{	indrc = vii;
						return ii;
					}
				} else {
					indrc = vii;
					return ii;
				}
			}
		}

	} else {
		ii = ind[1]+1;
		if(ind[1] == n -1)
			return -1;
		for(; ii<n;  ii++)
		{	if((it[ii].mask[1] & mask ) && (it[ii].type  > 0))
			{	indrc = ii;
				return ii;
			}
		}
	}
	return -2;
}

void planner::set_used(int cmd, int _use)
{	int i;
	for(i=0; i<n; i++)
	{	if(it[i].cmd == cmd)
		{	it[i].type = _use;
			break;
		}
	}
}
	