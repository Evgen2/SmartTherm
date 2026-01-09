/* SmartDebug.h */
#ifndef SMART_DEBUG
#define SMART_DEBUG
#include <stdarg.h>
#include <stdio.h>

#define DEBUG_DEFAULT 0x01  // default log mode
#define DEBUG_PID     0x02  // PID log mode
#define DEBUG_MQTT_INOUTDOOR 0x200  // MQTT indoor/outdoor temperature log mode 

//maximum number of strings in log
#define MAX_NUM_STR_LOG 1024
class Serial_Debug
{
public:
  int sp_n; // number of strings in log
  int sp_l; // total length of all srtings
  int ind;  // drop index
  int ls;
  int ls_s;
  int need_drop;
  static char **pmsg;
  int LogMode;  /* режим логирования 0 - default  */

  Serial_Debug(void)
  {
    need_drop = 0;
    sp_n = sp_l = 0;
    ind = ls = ls_s = 0;
    LogMode = DEBUG_DEFAULT;
  }
  size_t printf(const char *format, ...);
  size_t printfm(int mode, const char *format, ...);
  size_t my_logger(const char *fmt, ...);
  size_t v_logger(const char *fmt, va_list args);

  size_t write(uint8_t c);
  size_t write(const uint8_t *s, size_t n);
  void drop(void);
};

extern Serial_Debug Serial_db;

#endif // SMART_DEBUG
