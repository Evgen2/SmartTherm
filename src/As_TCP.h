/* As_TCP.h */

#include <lwip/sockets.h>
//#include <lwip/netdb.h>
#include <errno.h>

class As_TCP
{
public:
    int sockfd;
    int sts; 
    int t0;
    int timeout;
    fd_set fdset;
    struct timeval tv;

    As_TCP(void)
    {  sts = 0;
       sockfd = -1;
       t0 = timeout = 0;
    }
    int connect_0(IPAddress ip, uint16_t port, int32_t _timeout);
    int connect_a(void);
    int read_0(void);
    int read_a(void);
    int Read(char bufin[], int len);
    void closeTCP(void);

};
