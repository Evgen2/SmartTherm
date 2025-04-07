/* SmartDebug.cpp */

#include <Arduino.h>
#include <time.h>
#include "Smart_Config.h"

#if defined(Serial)
#undef Serial
#endif

static char sp_buffer[312];
int sp_n = 0, sp_l=0;

class Serial_Debug Serial_db;
static char *pmsg[1024];

size_t Serial_Debug::printf(const char *_format, ...)
{	size_t rc;
	va_list args;
    va_start(args, _format);
	int l;
	if(_format == NULL)
			return 0;
//    vsprintf(sp_buffer,_format, args);
	vsnprintf(sp_buffer, sizeof(sp_buffer),_format, args);
    va_end(args);
	l = strlen(sp_buffer);

	{	int free;
		char *pm;
		free = ESP.getFreeHeap();

		if(free > 64000)
		{	
			if(sp_n < 1024)
			{	pm = strdup( sp_buffer);
				pmsg[sp_n] = pm;
				sp_n++;
				sp_l += l;
				if(sp_n >= 1024)
				{	need_drop = 1;
					sp_n = 1025;
				}
			}
		}
	}

//	rc = Serial.printf("%d %d %s", sp_n, sp_l, sp_buffer);	
	rc = Serial.printf("%s", sp_buffer);	
	return rc;
}

size_t  Serial_Debug::write(uint8_t c)
{	size_t  rc;
	rc = Serial.write(c);
	return rc;
}
size_t Serial_Debug::write(const uint8_t* s, size_t n)
{	size_t  rc;
	rc = Serial.write(s, n);
	return rc;
}

void  Serial_Debug::drop(void)
{	int i;
	Serial.printf("drop %d\n", sp_n);
	if(sp_n <= 1024 || need_drop )
	{	if(sp_n > 1024)
				sp_n  = 1024;
		for(i=0; i<sp_n; i++)
		{	if(pmsg[i])
				Serial.printf(">%i %s", i, pmsg[i] );
		}

		for(i=sp_n-1; i>=0; i--)
		{	if(pmsg[i] )
			{	free(pmsg[i]);
				pmsg[i]  = NULL;
			}
		}
	}
	need_drop  = 0;
	sp_n = 0;
}


