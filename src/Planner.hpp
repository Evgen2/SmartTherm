/* Planner.hpp */

#ifndef PLANNER_DEFINED
#define PLANNER_DEFINED

#define MODE_START 0x1000
#define MODE_TEST  0x2000

#define MODE_IDLE  0x0001
#define MODE_CH	   0x0002
#define MODE_HW	   0x0004
#define MODE_ERROR 0x0010

#define NUM_PLAN 26

class plan_item
{
  public:
	unsigned char cmd;
	unsigned char type; //0 - not used, 1 - normal, 2 - use, if count > 1  (set with need_set())
	unsigned char count;
	short int mask[2];
	short int st;
	  plan_item(void)
	  {	cmd = 0;
		type = 0;
		mask[0] = mask[1] = 0;
		st = 0;
		count = 0;
	  }

};

class planner
{
  public:
	int n;
	int sts;
	int ind0, n0, count0;
	int ind1;
	int ind[2];
	int vind;
	int step;
	int mask;
	unsigned int ncycle;
	plan_item it[NUM_PLAN];

	planner(void)
	{ n = 0;
		n0 = 0;
		count0 = ncycle = 0;
		ind0 = ind1 = 0;
		ind[0] = ind[1] = -1;
		vind = 0;
		mask = 0;
		sts = 0;
		step = 0;
	}

	int add(int type, int cmd,  int _mask0,  int _mask1)
	{	if(n < NUM_PLAN)
		{	it[n].cmd = cmd;
			it[n].type = type;
			it[n].mask[0] = _mask0;
			it[n].mask[1] = _mask1;
			if(type == 2)
				it[n].count = 4;
			n++;
			return 1;
		} else {
			return 0;
		}
	}
	void set_used(int cmd, int _use);
	void set_mask(int cmd, int _mask0,  int _mask1);
	int run(int &lev, int mode);
	void SetMode(int _mask)
	{	mask = _mask;
		ind[0] = ind[1]  = -1;
		vind = 0;
		sts = 0;
		count0 = 0;
		ncycle = 0;
//		Serial.printf("Planner set mode to %x\n", _mask);

		if(_mask == MODE_START)
		{	n0 = n;
			step = 0; 
		} else if(_mask == MODE_TEST) {
			int i;
			for(i=0; i<n; i++)
					it[i].mask[0] |=  MODE_TEST; 
			n0 = n;					
			step = 0; 
		} else
			n0 = 6;
	}
	int find_next(int level, int &indrc);

};

#endif //PLANNER_DEFINED