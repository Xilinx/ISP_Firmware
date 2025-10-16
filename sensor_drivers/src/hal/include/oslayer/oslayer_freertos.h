/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Advanced Micro Devices, Inc. All right reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************/

#ifdef FREERTOS

#include <stdlib.h>
#include <oslayer/oslayer.h>
#include "ebase/types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define UCOSII_STACK_SIZE	(1024)
#define OSLAYER_ASSERT		(DCT_ASSERT)


typedef int32_t (*osThreadFunc)(void *);
typedef int32_t (*osIsrFunc)(void *);
typedef int32_t (*osDpcFunc)(void *);



#ifdef OSLAYER_EVENT
/*****************************************************************************/
/**
 *          osEvent
 *
 * @brief   Event object (FreeRTOS Version) of OS Abstraction Layer
 *
 * @note    This structure encapsulates FreeRTOS event group functionality
 *          for cross-platform event synchronization in the OS abstraction layer.
 *
 *****************************************************************************/
typedef struct _osEvent {
	EventGroupHandle_t	event_flags;
	int32_t			automatic;
	int32_t			state;
	int32_t			waitforall;
	char *name_ptr;
} osEvent;
#endif


#ifdef OSLAYER_MUTEX
/*****************************************************************************/
/**
 *          osMutex
 *
 * @brief   Mutex object (FreeRTOS Version) of OS Abstraction Layer
 *
 * @note    This structure encapsulates FreeRTOS semaphore functionality
 *          to provide mutex synchronization primitives for the OS abstraction layer.
 *
 *****************************************************************************/
typedef struct {
	SemaphoreHandle_t	mutex;
	char			*name_ptr;
} osMutex;
#endif


#ifdef OSLAYER_SEMAPHORE
/*****************************************************************************/
/**
 *          osSemaphore
 *
 * @brief   Semaphore object (FreeRTOS Version) of OS Abstraction Layer
 *
 * @note    This structure encapsulates FreeRTOS semaphore functionality
 *          for counting semaphore operations in the OS abstraction layer.
 *
 *****************************************************************************/
typedef struct {
	SemaphoreHandle_t	sem;
	char			*name_ptr;
} osSemaphore;
#endif

#ifdef OSLAYER_QUEUE
/*****************************************************************************/
/**
 *          osQueue
 *
 * @brief   Queue object (FreeRTOS Version) of OS Abstraction Layer
 *
 * @note    This structure encapsulates FreeRTOS queue functionality
 *          for inter-task communication in the OS abstraction layer.
 *
 *****************************************************************************/
typedef struct {
	xQueueHandle		qHandle;
	UBaseType_t		qlen;
	UBaseType_t		qitemsize;
} osQueue;
#endif




#ifdef OSLAYER_THREAD
/*****************************************************************************/
/**
 *          osThread
 *
 * @brief   Thread object (FreeRTOS Version) of OS Abstraction Layer
 *
 * @note    This structure encapsulates FreeRTOS task functionality
 *          for thread management in the OS abstraction layer.
 *
 *****************************************************************************/
typedef struct {
	char			*pcName;
	configSTACK_DEPTH_TYPE	usStackDepth;
	UBaseType_t		uxPriority;
	TaskHandle_t		pxCreatedTask;
} osThread;
#endif

#ifdef OSLAYER_MISC
/*****************************************************************************/
/**
 *          osSpinLock
 *
 * @brief   Spin Lock object (FreeRTOS Version) of OS Abstraction Layer
 *
 * @note    This structure provides low-level spinlock functionality
 *          for atomic operations and hardware-level synchronization.
 *
 *****************************************************************************/
typedef struct {
	UINTPTR			Xil_Spinlock_Addr_tr;
	UINTPTR			Xil_Spinlock_Flag_Addr_tr;
	UINTPTR			base_addr;
	u32			initstatus;
} osSpinLock;

/*****************************************************************************/
/**
 *          spinlock_sharedmem_t
 *
 * @brief   Spinlock shared memory configuration structure
 *
 * @note    This structure defines memory regions and configuration
 *          parameters for spinlock shared memory operations.
 *
 *****************************************************************************/
typedef struct {
	UINTPTR			start_addr;
	UINTPTR			end_addr;
	u32			num_regions;
	u32			initmpuregion;
	u32			erg_size;
} spinlock_sharedmem_t;

#endif
#ifdef __cplusplus
}
#endif
#endif
