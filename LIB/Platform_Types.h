/*Platform types for STMicroelectronics ST30 could have the follolwing mapping to the ANSI C types*/
#ifndef MY_PLATFORM_TYPES_H_
#define MY_PLATFORM_TYPES_H_

/*--------Symbols------------*/
#define CPU_TYPE CPU_TYPE_32
#define CPU_BIT_ORDER LSB_FIRST
#define CPU_BYTE_ORDER LOW_BYTE_FIRST

/*--------Types------------*/
#define NULL                ((void *)(0))

typedef unsigned char       boolean;
#define TRUE                (1U)
#define FALSE               (0U)


typedef char                sint8;
typedef short               sint16;
typedef long                sint32;
typedef long long           sint64;

typedef unsigned char       uint8;
typedef unsigned short      uint16;
typedef unsigned long       uint32;
typedef unsigned long long  uint64;

typedef float               float32;
typedef double              float64;

typedef volatile sint8      vsint8_t;
typedef volatile sint16     vsint16_t;
typedef volatile sint32     vsint32_t;
typedef volatile sint64     vsint64_t;

typedef volatile uint8      vuint8_t;
typedef volatile uint16     vuint16_t;
typedef volatile uint32     vuint32_t;
typedef volatile uint64     vuint64_t;

typedef void(* fptr_Callback)(void);
/**
 * @brief This enum specifies the status of the function call. 
 * 
 */
typedef enum
{
    E_OK,
    E_NOK
}eStatus_t;

#endif