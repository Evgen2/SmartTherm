/* SmartDebug.h */
#ifndef SMART_DEBUG
#define SMART_DEBUG
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

  Serial_Debug(void)
  {
    need_drop = 0;
    sp_n = sp_l = 0;
    ind = ls = ls_s = 0;
  }
  size_t printf(const char *format, ...);
  size_t write(uint8_t c);
  size_t write(const uint8_t *s, size_t n);
  void drop(void);
};

extern Serial_Debug Serial_db;

#endif // SMART_DEBUG
