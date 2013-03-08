//*****************************************************************************
//
//				Stellaris IMU (FREEIMU PORT)
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//		Author 				: Terence Ang - terenceang@mac.com
//		Original Code 		:  Hari Nair- hair.nair@gmail.com
//		Original i2c code 	: JOERG QUINTEN (aBUGSworstnightmare)
//		Original ftoa code	: JOERG QUINTEN (aBUGSworstnightmare)
//
//					1st Draft .. 09/March/2013
//
//*****************************************************************************

#ifndef __TYPE_H__
#define __TYPE_H__

typedef unsigned long long  u64;
typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u08;

typedef signed long long   s64;
typedef signed long    s32;
typedef signed short   s16;
typedef signed char    s08;

typedef unsigned char       uint8;   //8 bits
typedef unsigned short      uint16;  //16 bits
typedef unsigned long       uint32;  //32 bits
typedef unsigned long long  uint64;  //64 bits


typedef signed char         int8;    //8 bits
typedef signed short        int16;   //16 bits
typedef signed long         int32;   //32 bits
typedef signed long long    int64;   //64 bits



typedef union _union16 {
  u16 u;
  s16 i;
  u08 b[2];
} union16;

typedef union _union32 {
  u32 lu;
  s32 li;
  u16 u[2];
  s16 i[2];
  u08 b[4];
} union32;

typedef union _union64 {
  u32 lu[2];
  s32 li[2];
  u16 u[4];
  s16 i[4];
  u08 b[8];
} union64;
#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

#endif  /* __TYPE_H__ */
