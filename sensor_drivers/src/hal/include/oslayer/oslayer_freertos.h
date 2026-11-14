/******************************************************************************\
|* Copyright 2010, Dream Chip Technologies GmbH. used with permission by      *|
|* VeriSilicon.                                                               *|
|* Copyright (c) <2020> by VeriSilicon Holdings Co., Ltd. ("VeriSilicon")     *|
|* All Rights Reserved.                                                       *|
|*                                                                            *|
|* The material in this file is confidential and contains trade secrets of    *|
|* of VeriSilicon.  This is proprietary information owned or licensed by      *|
|* VeriSilicon.  No part of this work may be disclosed, reproduced, copied,   *|
|* transmitted, or used in any way for any purpose, without the express       *|
|* written permission of VeriSilicon.                                         *|
|*                                                                            *|
\******************************************************************************/

/* VeriSilicon 2020 */

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
#include "types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "FreeRTOS.h"
// #include "ff.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define UCOSII_STACK_SIZE   1024
#define OSLAYER_ASSERT      DCT_ASSERT

typedef TickType_t osTickType;
typedef BaseType_t osBaseType;
typedef UBaseType_t osUBaseType;
typedef StackType_t osStackType;
typedef StaticTask_t osStaticTaskTCBType;
typedef TaskHandle_t osTaskHandle;

#define osMaxDelay portMAX_DELAY
#define osTaskIdlePriority tskIDLE_PRIORITY

typedef void (*osThreadFunc)(void *);
typedef int32_t (*osIsrFunc)(void *);
typedef int32_t (*osDpcFunc)(void *);


#ifdef OSLAYER_EVENT
/*****************************************************************************/
/*  @brief  Event object (FreeRTOS Version) of OS Abstraction Layer */
typedef struct _osEvent
{
    EventGroupHandle_t event_flags;
    int32_t automatic;		/* Decides if flag to be consumed (xClearOnExit) */
    int32_t state;			/* Not used in Freertos */
    int32_t waitforall;		/* Added for future extension, need waiting for event bit */
    char *name_ptr;
#if(STATIC_ALLOCATION)
    StaticEventGroup_t event_group_buf;
#endif
} osEvent;
#endif /* OSLAYER_EVENT */


#ifdef OSLAYER_MUTEX
/*****************************************************************************/
/*  @brief  Mutex object (FreeRTOS Version) of OS Abstraction Layer */
typedef struct _osMutex
{
	volatile uint8_t isInited; /**<init flag: 0=uninit, 1=inited>*/
	SemaphoreHandle_t mutex;
    char *name_ptr;
#if(STATIC_ALLOCATION)
    StaticSemaphore_t mutex_buf;  /* Static storage for mutex */
#endif
} osMutex;
#endif /* OSLAYER_MUTEX */


#ifdef OSLAYER_SEMAPHORE
/*****************************************************************************/
/*  @brief  Semaphore object (FreeRTOS Version) of OS Abstraction Layer */
typedef struct _osSemaphore
{
	SemaphoreHandle_t sem;
    char *name_ptr;
#if(STATIC_ALLOCATION)
    StaticSemaphore_t semaphore_buf;  /* Static storage for semaphore */
#endif
} osSemaphore;
#endif /* OSLAYER_SEMAPHORE */


#ifdef OSLAYER_QUEUE

#if(STATIC_ALLOCATION)

#define MAX_QUEUE_ITEMS 80
#define MAX_ITEM_SIZE  16
#endif

typedef struct _osQueue
{
	xQueueHandle	qHandle;
	UBaseType_t 	qlen;
	UBaseType_t 	qitemsize;
    BaseType_t      xHigherPriorityTaskWoken;
#if(STATIC_ALLOCATION)
    StaticQueue_t   queueBuf;
    uint8_t         queueStorage[MAX_QUEUE_ITEMS * MAX_ITEM_SIZE];
#endif
} osQueue;
#endif /*OSLAYER_QUEUE */


#ifdef OSLAYER_THREAD
/*****************************************************************************/
/*  @brief  Thread object (FreeRTOS Version) of OS Abstraction Layer */

typedef struct _osThread
{
    char *pcName;
	configSTACK_DEPTH_TYPE usStackDepth;
	UBaseType_t uxPriority;
	TaskHandle_t pxCreatedTask;
    char threadName[20];
    osEvent exit_event;
    void (*pThreadFunc)(void *);
    void *p_arg;
#if(STATIC_ALLOCATION)
    StackType_t *pxStackBuffer;
    StaticTask_t *pxTaskBuffer;
    StackType_t stackArray[OS_THREAD_STACK_MAX_WORDSIZE];
    StaticTask_t taskBlock;
#endif
} osThread;
#endif /* OSLAYER_THREAD */

#ifdef OSLAYER_TIMESTAMP
/*****************************************************************************/
/*  @brief  time stamp object of OS Abstraction, just for compilation. */
typedef uint32_t osTimeStamp;
#endif


#ifdef __cplusplus
}
#endif


#endif /* FREERTOS */
