/* mybuffer.cpp */
#include <stdlib.h>
#include <string.h>

#include "mybuffer.hpp"

void QueryCom(void);

/* отдаем под myBuffer lb байт по адресу pb */
void myBuffer::Init(char *pb, int Lb)
{   pbuf = pb;
    Lbuf = Lb;
    ibuf = ifree = 0;	 
}

/* отдаем под myBuffer lb байт по адресу pb, запоминаем длину элемента  _Litem */
void myBuffer2::Init(void *pb, int Lb, int _Litem)
{   pbuf = (char *)pb;
    Litem	= _Litem;
    Lbuf = Lb/Litem*Litem; // Lbuf выравнивается на длину элемента
    ibuf = ifree = 0;
}

/* добавить в myBuffer Lb байт с адреса pb */
int myBuffer::Add(char *pb, int Lb)
{ 
	if(GetFree() <= Lb)
		  return 0;

	if((ifree + Lb) < Lbuf)
	{	memcpy((void *)&pbuf[ifree],(void *)pb ,Lb);
		ifree += Lb;
	} else {
		int k0, k1;
		k0 = ifree + Lb -	Lbuf;
		k1 = Lb - k0;
		memcpy((void *)&pbuf[ifree],(void *)pb, k1 );
		if(k0)
		   memcpy((void *)&pbuf[0], (void *)&pb[k1], k0);
		ifree = k0;
		
	}
	return 1;
}


/* добавить в myBuffer один байт */
int myBuffer::Add(char byte)
{ 
	if(GetFree() <= 1)
		  return 0;
  pbuf[ifree] = byte;
	if((ifree + 1) < Lbuf)
	{	ifree++;
	} else {
		ifree = 0;
	}
	return 1;
}


/* добавить в myBuffer2 один элемент */
int myBuffer2::Add(void *ptr)
{  int is_wrap = 0, l;
	 l = GetFree();
	if(l < Litem)
		  return 0;
	if(l == Litem) 
		 is_wrap = 1;
	
	memcpy((void *)&pbuf[ifree], ptr, Litem);
	if((ifree + Litem) < Lbuf)
	{	ifree +=Litem;
	} else {
		ifree = 0;
	}
	if(is_wrap)
	{ if((ibuf + Litem) < Lbuf)
		{	ibuf +=Litem;
		} else {
			ibuf = 0;
		}
	}
	return 1;
}

/* Вернуть элемент буфера и освободить место */
int myBuffer::Get(void)
{   int rc; 
	if(GetLbuf() <= 0)
		  return -1;
	 rc = pbuf[ibuf];
   ibuf++;	
    if(ibuf == Lbuf)
			     ibuf = 0;
	return rc;
}

/* Вернуть элемент буфера и освободить место */
int myBuffer2::Get(void *ptr)
{   
	if(GetLbuf() <= 0)
		  return -1;
   memcpy(ptr,(void *)&pbuf[ibuf], Litem);
   ibuf += Litem;	
    if(ibuf == Lbuf)
			     ibuf = 0;
	return 0;
}

/**************************************************/
/* Вернуть элемент буфера и и не освободить место */
/* начать чтение */
void myBuffer::StartRead(void)
{    ibuf2 = ibuf;
}
/* закончить чтение и освободить место в буфере */
void myBuffer::EndRead(void)
{
	 ibuf = ibuf2;
}

int myBuffer::GetUnread(void)
{   int l, l2;
/*******************/
//QueryCom(); //PC
     l = ifree - ibuf;
	 if(l < 0) l += Lbuf;	// l = длина буфера
	 if(l <= 0)
		 return 0; // буфер пустой
//  ibuf...................ifree
//  ....|...ibuf2.........|.....
     l2 = ibuf2 - ibuf;
	 if(l2 < 0) l2 += Lbuf;	// l2 = длина прочитанной части буфера
	 if(l2 >= l)
		 return 0; // буфер прочитан
	 return l - l2;
}

/* Собственно Вернуть элемент буфера и не освободить место */
int myBuffer::Read(char *el)
{   int  l, l2; 
/*******************/
     l = ifree - ibuf;
	 if(l < 0) l += Lbuf;	// l = длина буфера
	 if(l <= 0)
		 return -1; // буфер пустой
//  ibuf...................ifree
//  ....|...ibuf2.........|.....
     l2 = ibuf2 - ibuf;
	 if(l2 < 0) l2 += Lbuf;	// l2 = длина прочитанной части буфера
	 if(l2 >= l)
		 return -2; // буфер прочитан

/******************/
	*el = pbuf[ibuf2];
    ibuf2++;	
    if(ibuf2 == Lbuf)
			     ibuf2 = 0;
	return 0;
}

/* Сколько свободно в буфере */
int myBuffer::GetFree(void)
{ 
	return Lbuf - GetLbuf();
}

/* Длина занятого буфера в char */
int myBuffer::GetLbuf(void)
{  int l;
// QueryCom(); //PC
     l = ifree - ibuf;
	 if(l < 0) l += Lbuf;	
	return l;
}
/* Длина занятого буфера в элементах */
int myBuffer2::GetLbuf(void)
{  int l;
     l = ifree - ibuf;
	 if(l < 0) l += Lbuf;	
	return l/Litem;
}

