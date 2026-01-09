/* SmartDebug.cpp */

#include <Arduino.h>
#include <time.h>
#include "Smart_Config.h"
#include "SmartDebug.h"

#if defined(Serial)
#undef Serial
#endif

static char sp_buffer[312];

class Serial_Debug Serial_db;

static char *pmsg0[1024];
//char Serial_Debug::**pmsg = pmsg0;
char  * * Serial_Debug::pmsg = pmsg0;

size_t Serial_Debug::printf(const char *_format, ...)
{	size_t rc=0;
    va_list args;
    va_start(args, _format);
    rc  = v_logger(_format, args); // Forward list to the helper
    va_end(args);
	return 	rc;
}

size_t Serial_Debug::printfm(int mode, const char *_format, ...)
{ 	size_t rc=0;
	if(_format == NULL)
			return 0;
	if(mode & LogMode)
	{	va_list args;
		va_start(args, _format);      // Initialize list at the last named parameter
		rc  = v_logger(_format, args); // Forward list to the helper
		va_end(args);
	}
    return rc;
}

/******************************************************************************/
//  The WORKHORSE: Accepts va_list
size_t Serial_Debug::v_logger(const char *fmt, va_list args) 
{	size_t rc;
	int l;
	if(fmt == NULL)
			return 0;
	vsnprintf(sp_buffer, sizeof(sp_buffer), fmt, args);
    va_end(args);
	l = strlen(sp_buffer);
	if(l>=sizeof(sp_buffer))
	{	Serial.printf("Serial_Debug::printf Error: need buff %d\n", l);	
	}

//	if(0)
	{	int free;
		char *pm;
		free = ESP.getFreeHeap();

		if(free > 64000)
		{	
			if(sp_n < 1024)
			{	pm = strdup( sp_buffer);
				if(pm == NULL)
						 Serial.printf("Error: strdup return NULL for %d\n", l);	
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


