/****************************************************************************
 * *
 * * The MIT License (MIT)
 * *
 * * Copyright (c) 2025 Advanced Micro Devices, Inc. All right reserved.
 * *
 * * Permission is hereby granted, free of charge, to any person obtaining a
 * * copy of this software and associated documentation files (the "Software"),
 * * to deal in the Software without restriction, including without limitation
 * * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * * and/or sell copies of the Software, and to permit persons to whom the
 * * Software is furnished to do so, subject to the following conditions:
 * *
 * * The above copyright notice and this permission notice shall be included in
 * * all copies or substantial portions of the Software.
 * *
 * * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * * DEALINGS IN THE SOFTWARE.
 * *
 * ****************************************************************************/
/**
 * Module    : Operating System Abstraction Layer
 *
 * Hierarchy :
 *
 * Purpose   : Encapsulates and abstracts services from different operating
 *             system, including user-mode as well as kernel-mode services.
 ******************************************************************************/
#ifdef FREERTOS

#include <stdlib.h>
#include <oslayer/oslayer.h>
#include "ebase/types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"


//#include <ucos_ii.h>
//#include "types.h"
//#include "dct_assert.h"

//AMD,Copied below defines from ebase/types.h
// Need to check how to include types.h file

#if 0
typedef unsigned char        uint8_t;
typedef signed   char        int8_t;
typedef unsigned short       uint16_t;
typedef          short       int16_t;
typedef unsigned int         uint32_t;
typedef          int         int32_t;
typedef unsigned char INT8U;
typedef unsigned int INT32U;
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define UCOSII_STACK_SIZE   1024
#define OSLAYER_ASSERT      DCT_ASSERT


typedef int32_t (*osThreadFunc)(void *);
typedef int32_t (*osIsrFunc)(void *);
typedef int32_t (*osDpcFunc)(void *);



#ifdef OSLAYER_EVENT
/*****************************************************************************/
/*  @brief  Event object (Linux Version) of OS Abstraction Layer */
typedef struct _osEvent
{

    EventGroupHandle_t event_flags;
    int32_t automatic;		/* Decides if flag to be consumed (xClearOnExit) */
    int32_t state;			/* Not used in Freertos */
    int32_t waitforall;		/* Added for future extension */
    char *name_ptr;
} osEvent;
#endif /* OSLAYER_EVENT */


#ifdef OSLAYER_MUTEX
/*****************************************************************************/
/*  @brief  Mutex object (Linux Version) of OS Abstraction Layer */
typedef struct _osMutex
{
	SemaphoreHandle_t mutex;
    char *name_ptr;

} osMutex;
#endif /* OSLAYER_MUTEX */


#ifdef OSLAYER_SEMAPHORE
/*****************************************************************************/
/*  @brief  Semaphore object (Linux Version) of OS Abstraction Layer */
typedef struct _osSemaphore
{
	SemaphoreHandle_t sem;
    char *name_ptr;
} osSemaphore;
#endif /* OSLAYER_SEMAPHORE */

#ifdef OSLAYER_QUEUE
typedef struct _osQueue
{
	xQueueHandle	qHandle;
	UBaseType_t 	qlen;
	UBaseType_t 	qitemsize;
} osQueue;
#endif /*OSLAYER_QUEUE */




#ifdef OSLAYER_THREAD

/*****************************************************************************/
/*  @brief  Thread object (Linux Version) of OS Abstraction Layer */
typedef struct _osThread
{
  //  void (*Task)(void *);			//passed as argument to osthread create
      char *pcName;
	 configSTACK_DEPTH_TYPE usStackDepth;
//	void * const pvParameters;		//passed as argument to osthread create
	UBaseType_t uxPriority;
	TaskHandle_t pxCreatedTask;
} osThread;
#endif /* OSLAYER_THREAD */

#ifdef OSLAYER_MISC
/*****************************************************************************/
/*  @brief  Spin Lock object (Linux Kernel Version only) of OS Abstraction */
/*          Layer */
typedef struct _osSpinLock
{
	UINTPTR Xil_Spinlock_Addr_tr;
	UINTPTR Xil_Spinlock_Flag_Addr_tr;
	UINTPTR base_addr;				//this points to memory region ,where this spinlock physically Resides at runtime
	u32		initstatus;				// This helps in avoiding access of spinlock varaible ,before being initialized
} osSpinLock;


typedef struct spinlock_sharedmem_t
{
	UINTPTR start_addr;
	UINTPTR end_addr;
	u32 num_regions;
	u32 initmpuregion;
	 u32 erg_size;
}spinlock_sharedmem_t;



#endif  /* OSLAYER_MISC */
#ifdef __cplusplus
}
#endif



#endif /* FREERTOS */
