/* mybuffer.hpp */
#ifndef MYBUFFER
#define MYBUFFER

class myBuffer
{
 public:
   char *pbuf; // указатель на буфер
   int Lbuf;  // длина буфера в байтах
   int ibuf;  // индекс начала занятой области 
   int ibuf2; // индекс начала прочитанной области 
   int ifree; // индекс начала свободной области 
//   int Litem; // длина элемента в int, 0=переменная длина элемента

   myBuffer(void)
   {  pbuf = NULL;
      Lbuf = 0;
      ibuf = ibuf2 = ifree = 0;
//      Litem = 0;
   };
    void Init(char *pb, int Lb);
	  int Add(char byte);
    int Add(char *pb, int Lb);
//    int Add2(int *pb, int Lb);
    int Get(void);
    void StartRead(void);
    int GetUnread(void);
    int Read(char *el);
    void EndRead(void);
    int GetFree(void);
    int GetLbuf(void);
};

class myBuffer2:public myBuffer
{
 public:
   int Litem; // длина элемента в int, (???)0=переменная длина элемента

   myBuffer2(void)
   {  Litem = 0;
   };
   void Init(void *pb, int Lb, int _Litem);
	 int Add(void *prt);
   int Get(void *prt);
   int GetLbuf(void);
};

#endif // MYBUFFER
