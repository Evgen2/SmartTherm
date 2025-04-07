/* SmartDebug.h */
#ifndef SMART_DEBUG
#define SMART_DEBUG

class Serial_Debug
{
  public:
  int need_drop;
    Serial_Debug(void)
    { need_drop = 0; 

    }
    size_t printf(const char *format, ...);
    size_t write(uint8_t c); 
    size_t write(const uint8_t* s, size_t n);
    void  drop(void);
};

extern  Serial_Debug Serial_db;

#endif // SMART_DEBUG
