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
 *****************************************************************************/

#ifndef OSLAYER_H
#define OSLAYER_H

/*****************************************************************************
 *          OSLAYER_TIMESTAMP
 *
 * @brief   Include time-stamp handling in OS Abstraction Layer library
 *
 *****************************************************************************/
#define OSLAYER_TIMESTAMP

/*****************************************************************************
 *          OSLAYER_EVENT
 *
 * @brief   Include event handling in OS Abstraction Layer library
 *
 *****************************************************************************/
#define OSLAYER_EVENT

/*****************************************************************************
 *          OSLAYER_MUTEX
 *
 * @brief   Include mutex handling in OS Abstraction Layer library
 *
 *****************************************************************************/
#define OSLAYER_MUTEX

/*****************************************************************************
 *          OSLAYER_SEMAPHORE
 *
 * @brief   Include semaphore handling in OS Abstraction Layer library
 *
 *****************************************************************************/
#define OSLAYER_SEMAPHORE

/*****************************************************************************
 *          OSLAYER_QUEUE
 *
 * @brief   Include queue handling in OS Abstraction Layer library
 *
 *****************************************************************************/
#if defined (OSLAYER_SEMAPHORE) && defined (OSLAYER_MUTEX)
#define OSLAYER_QUEUE
#endif

/*****************************************************************************
 *          OSLAYER_ATOMIC
 *
 * @brief   Include atomic operations in OS Abstraction Layer library
 *
 *****************************************************************************/
#define OSLAYER_ATOMIC

/*****************************************************************************
 *          OSLAYER_THREAD
 *
 * @brief   Include thread handling in OS Abstraction Layer library
 *
 *****************************************************************************/
#if defined (OSLAYER_EVENT) && defined (OSLAYER_MUTEX)
#define OSLAYER_THREAD
#endif

/*****************************************************************************
 *          OSLAYER_IRQ
 *
 * @brief   Include IRQ service functionality in OS Abstraction Layer library
 *
 *****************************************************************************/
#if defined (OSLAYER_EVENT) && defined (OSLAYER_MUTEX) && defined (OSLAYER_THREAD)
#define OSLAYER_IRQ
#endif

/******************************************************************************/
/** @brief  Include task notification functionality in OS Abstraction Layer library */
#if defined OSLAYER_THREAD
#define OS_THREAD_STACK_MAX_WORDSIZE (20 * 1024)
#define OSLAYER_TASKNOTIFICATION
#endif

/*****************************************************************************
 *          OSLAYER_MISC
 *
 * @brief   Include misc functionality in OS Abstraction Layer library
 *
 *****************************************************************************/
#define OSLAYER_MISC

//!@} defgroup OS_LAYER_CONFIG

/*****************************************************************************
 *          OS_LAYER
 *
 * @defgroup OS_LAYER OS layer interface
 * @{
 * @brief   Encapsulates and abstracts services from different operating
 *          system, including user-mode as well as kernel-mode services,
 *          in a operating system independent way.
 *
 *****************************************************************************/

#ifdef WIN32
#include "oslayer_win32.h"
#endif
#ifdef LINUX
  #ifdef MSVD_COSIM
   #include "oslayer_systemc.h"
  #else
   #include "oslayer_linux.h"
  #endif
#endif
#ifdef UCOSII
   #include "oslayer_ucosii.h"
#endif
#ifdef MFS
#include "oslayer_mfs.h"
#endif
#ifdef SAFERTOS
#include "oslayer_safertos.h"
#endif

#ifdef FREERTOS
#include <oslayer/oslayer_freertos.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef OSLAYER_ASSERT
/*****************************************************************************
 *          OSLAYER_ASSERT
 *
 * @brief   Default assertion macro for OS Abstraction Layer
 *
 *****************************************************************************/
#define OSLAYER_ASSERT(x)
#endif

#define OSLAYER_CROAK(expr)\
    do {\
        if ((expr)) {\
            printf("oslayer assert %s, %d  %s\n", __FILE__, __LINE__, #expr);\
            return (OSLAYER_INVALID_PARAM);\
        }\
    } while(0)

/*****************************************************************************
 *          OSLAYER_STATUS
 *
 * @brief   Status codes of OS Abstraction Layer operation
 *
 *****************************************************************************/
typedef enum {
	OSLAYER_OK			= 0,
	OSLAYER_ERROR			= -1,
	OSLAYER_INVALID_PARAM		= -2,
	OSLAYER_OPERATION_FAILED	= -3,
	OSLAYER_NOT_INITIALIZED		= -4,
	OSLAYER_TIMEOUT			= -5,
	OSLAYER_SIGNAL_PENDING		= -6
} OSLAYER_STATUS;

/*****************************************************************************
 *          OSLAYER_THREAD_PRIO
 *
 * @brief   Priority of thread created by OS Abstraction Layer
 *
 *****************************************************************************/
typedef enum {
	OSLAYER_THREAD_PRIO_HIGHEST,
	OSLAYER_THREAD_PRIO_HIGH,
	OSLAYER_THREAD_PRIO_NORMAL,
	OSLAYER_THREAD_PRIO_LOW,
	OSLAYER_THREAD_PRIO_LOWEST
} OSLAYER_THREAD_PRIO;

/******************************************************************************/
/** @brief  Action for task Notification created by OS Abstraction Layer */
typedef enum _osTaskNotificationAction
{
	OSLAYER_TASKNOTIFICATION_NOACTION = 0,
	OSLAYER_TASKNOTIFICATION_SETBITS,
	OSLAYER_TASKNOTIFICATION_INCREMENT,
	OSLAYER_TASKNOTIFICATION_SETVALUEWITHOVERWRITE,
	OSLAYER_TASKNOTIFICATION_SETVALUEWITHOUTOVERWRITE,
} osTaskNotificationAction;

/*****************************************************************************
 *          osInterrupt
 *
 * @brief   Interrupt service structure for OS Abstraction Layer
 *
 * @note    This structure encapsulates interrupt handling functionality
 *          including thread management, event signaling, and queue operations
 *          for interrupt service routines.
 *
 *****************************************************************************/
typedef struct _osInterrupt {
#ifdef OSLAYER_KERNEL
#else
	osThread	isr_thread;
	osEvent		isr_event;
	osEvent		isr_exit_event;
	osMutex		isr_access_lock;
	osEvent		isr_irq_event;
	osQueue		isr_mis_queue;
#endif
	uint32_t	irq_num;
	osIsrFunc	IsrFunc;
	osDpcFunc	DpcFunc;
	void		*p_context;
} osInterrupt;


#ifdef OSLAYER_TIMESTAMP
/******************************************************************************
 *  osTimeStampUs()
 *****************************************************************************
 *  @brief  Returns a 64-bit timestamp [us]
 *
 *  @param  pEvent         Reference of the timestamp object
 *
 *  @return                Status of operation
 *  @retval OSLAYER_OK     Event successfully created
 *
 *****************************************************************************/
extern OSLAYER_STATUS osTimeStampUs(int64_t *pTimeStamp);

/******************************************************************************
 *  osTimeStampNs()
 *****************************************************************************
 *  @brief  Returns a 64-bit timestamp [ns]
 *
 *  @param  pEvent         Reference of the timestamp object
 *
 *  @return                Status of operation
 *  @retval OSLAYER_OK     Event successfully created
 *
 *****************************************************************************/
extern OSLAYER_STATUS osTimeStampNs(float32_t *pTimeStamp);
extern OSLAYER_STATUS osTimeStampLocalTime(osTimeStamp *pTimeStamp);
#endif


#ifdef OSLAYER_EVENT
/******************************************************************************
 *  osEventInit()
 *****************************************************************************
 *  @brief  Initialize an event object.
 *
 *  Init an event. Automatic reset flag as well as initial state could be set,
 *  but event is not signaled. Automatic reset means osEventReset() must
 *  not be called to set event state to zero. This also implies that all
 *  waiting threads, otherwise only one thread, will be restarted.
 *
 *  @param  pEvent         Reference of the event object
 *
 *  @param  Automatic      Automatic Reset flag
 *
 *  @param  InitState      Initial state of the event object (true/false)
 *
 *  @return                Status of operation
 *  @retval OSLAYER_OK     Event successfully created
 *  @retval OSLAYER_ERROR  Event is not created
 *
 *****************************************************************************/
extern OSLAYER_STATUS osEventInit(osEvent *pEvent, int32_t Automatic,
				int32_t InitState);

/******************************************************************************
 *  osEventSignal()
 *****************************************************************************
 *  @brief  Set the event state to true.
 *
 *  Set the state of the event object to true and signal waiting thread(s).
 *
 *  @param  pEvent                    Reference of the event object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Signal send successfully
 *  @retval OSLAYER_ERROR             Tried to signal a not initialized event
 *  @retval OSLAYER_OPERATION_FAILED  Signal not send
 *
 *****************************************************************************/
extern OSLAYER_STATUS osEventSignal(osEvent *pEvent);

/******************************************************************************
 *  osEventReset()
 *****************************************************************************
 *  @brief  Reset the event state to true.
 *
 *  Reset the state of the event object to false.
 *
 *  @param  pEvent                    Reference of the event object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Event reset successfully
 *  @retval OSLAYER_ERROR             Tried to reset a not initialized event
 *  @retval OSLAYER_OPERATION_FAILED  Event not reset
 *
 *****************************************************************************/
extern OSLAYER_STATUS osEventReset(osEvent *pEvent);

/******************************************************************************
 *  osEventPulse()
 *****************************************************************************
 *  @brief  Pulse the event false -> true -> false.
 *
 *  Pulse the state of the event object with the following sequence:
 *  false -> true -> false and signal waiting thread(s).
 *
 *  @param  pEvent                    Reference of the event object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Event pulsed successfully
 *  @retval OSLAYER_ERROR             Tried to pulse a not initialized event
 *  @retval OSLAYER_OPERATION_FAILED  Event not pulsed
 *
 *****************************************************************************/
extern OSLAYER_STATUS osEventPulse(osEvent *pEvent);

/******************************************************************************
 *  osEventWait()
 *****************************************************************************
 *  @brief  Blocking wait for event to be true.
 *
 *  Wait for the state of the event object becoming true and block calling
 *  thread. The function call returns immediatly, if event is already signaled.
 *
 *  @param  pEvent                    Reference of the event object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Wait for event succeeded and function
 *                                    returned due to signal sent
 *  @retval OSLAYER_ERROR             Tried to wait for a not initialized event
 *  @retval OSLAYER_OPERATION_FAILED  Wait for event failed
 *
 *****************************************************************************/
extern OSLAYER_STATUS osEventWait(osEvent *pEvent);

/******************************************************************************
 *  osEventTimedWait()
 *****************************************************************************
 *  @brief  Blocking wait with timeout for event to be true.
 *
 *  Wait for the state of the event object becoming true and block calling
 *  thread. The function call returns immediatly, if event is already signaled.
 *  If specified timeout elapses, the function returns with apropriate timeout
 *  error code.
 *
 *  @param  pEvent      reference of the event object
 *
 *  @param  msec        timeout value in milliseconds
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Wait for event succeeded and function
 *                                    returned due to signal sent
 *  @retval OSLAYER_TIMEOUT           Wait for event succeeded and function
 *                                    returned due to timeout and no signal
 *                                    was sent
 *  @retval OSLAYER_OPERATION_FAILED  Wait for event failed
 *
 *****************************************************************************/
extern OSLAYER_STATUS osEventTimedWait(osEvent *pEvent, uint32_t msec);

/******************************************************************************
 *  osEventDestroy()
 *****************************************************************************
 *  @brief  Destroy the event.
 *
 *  Destroy the event and osFree resources associated with event object.
 *
 *  @param  pEvent      Reference of the event object
 *
 *  @return             always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osEventDestroy(osEvent *pEvent);
#endif


#ifdef OSLAYER_MUTEX
/******************************************************************************
 *  osMutextInit()
 *****************************************************************************
 *  @brief  Initialize a mutex object.
 *
 *  @param  pMutex         Reference of the mutex object
 *
 *  @return                Status of operation
 *  @retval OSLAYER_OK     Mutex successfully created
 *  @retval OSLAYER_ERROR  Mutex is not created
 *
 *****************************************************************************/
extern OSLAYER_STATUS osMutexInit(osMutex *pMutex);

/******************************************************************************
 *  osMutexLock()
 *****************************************************************************
 *  @brief  Lock a mutex object.
 *
 *          Lock the mutex. Thread will be blocked if mutex already locked.
 *
 *  @param  pMutex         Reference of the mutex object
 *
 *  @return                Status of operation
 *  @retval OSLAYER_OK     Mutex successfully locked
 *
 *****************************************************************************/
extern OSLAYER_STATUS osMutexLock(osMutex *pMutex);

/******************************************************************************
 *   osMutexUnlock()
 *****************************************************************************
 *  @brief  Unlock a mutex object.
 *
 *  @param  pMutex         Reference of the mutex object
 *
 *  @return                Status of operation
 *  @retval OSLAYER_OK     Mutex successfully unlocked
 *
 *****************************************************************************/
extern OSLAYER_STATUS osMutexUnlock(osMutex *pMutex);

/******************************************************************************
 *  osMutexTryLock()
 *****************************************************************************
 *  @brief  Try to lock a mutex object.
 *
 *  Try to lock mutex in Non-Blocking mode. Returns OSLAYER_OK if successful.
 *
 *  @param  pMutex                    Reference of the mutex object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Mutex successfully locked
 *  @retval OSLAYER_OPERATION_FAILED  Mutex not locked (already locked by
 *                                    someone else)
 *
 *****************************************************************************/
extern OSLAYER_STATUS osMutexTryLock(osMutex *pMutex);

/******************************************************************************
 *  osMutexDestroy()
 *****************************************************************************
 *  @brief  Destroy a mutex object.
 *
 *  @param  pMutex      Reference of the mutex object
 *
 *  @return             always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osMutexDestroy(osMutex *pMutex);
#endif


#ifdef OSLAYER_SEMAPHORE
/******************************************************************************
 *  osSemaphoreInit()
 *****************************************************************************
 *  @brief  Init a semaphore with init count.
 *
 *  @param  pSem           Reference of the semaphore object
 *  @param  init_count     Semaphore object count
 *
 *  @return                Status of operation
 *  @retval OSLAYER_OK     Semaphore successfully created
 *  @retval OSLAYER_ERROR  Semaphore is not created
 *
 *****************************************************************************/
extern OSLAYER_STATUS osSemaphoreInit(osSemaphore * pSem, uint32_t init_count);

/******************************************************************************
 *  osSemaphoreTimedWait()
 *****************************************************************************
 *  @brief  Decrease the semaphore value in blocking mode, but with timeout.
 *
 *  @param  pSem                      Reference of the semaphore object
 *
 *  @param  msec                      Timeout value in milliseconds
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Wait for semaphore succeeded and
 *                                    function returned due to signal sent
 *  @retval OSLAYER_TIMEOUT           Wait for semaphore succeeded and
 *                                    function returned due to timeout and
 *                                    no signal was sent
 *  @retval OSLAYER_OPERATION_FAILED  Wait for semaphore failed
 *  @retval OSLAYER_ERROR             Tried to wait for a not initialized
 *                                    semaphore
 *
 *****************************************************************************/
extern OSLAYER_STATUS osSemaphoreTimedWait(osSemaphore *pSem, uint32_t msec);

/******************************************************************************
 *  osSemaphoreWait()
 *****************************************************************************
 *  @brief  Decrease the semaphore value in blocking mode.
 *
 *  @param  pSem                      Reference of the semaphore object
 *
 *  @param  msec                      Timeout value in milliseconds
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Wait for semaphore succeeded and
 *                                    function returned due to signal sent
 *  @retval OSLAYER_OPERATION_FAILED  Wait for semaphore failed
 *  @retval OSLAYER_ERROR             Tried to wait for a not initialized
 *                                    semaphore
 *
 *****************************************************************************/
extern OSLAYER_STATUS osSemaphoreWait(osSemaphore *pSem);

/******************************************************************************
 *  osSemaphoreTryWait()
 *****************************************************************************
 *  @brief  Try to wait for a semaphore object.
 *
 *          Try to decrease the semaphore value in non-blocking mode. This
 *          functionality is realized internally with a timed wait called
 *          with a timeout value of zero.
 *
 *  @param  pSem                      Reference of the mutex object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Wait for semaphore succeeded and
 *                                    function returned due to signal sent
 *  @retval OSLAYER_TIMEOUT           Wait for semaphore succeeded and
 *                                    function returned due to timeout and
 *                                    no signal was sent
 *  @retval OSLAYER_OPERATION_FAILED  Wait for semaphore failed
 *  @retval OSLAYER_ERROR             Tried to wait for a not initialized
 *                                    semaphore
 *
 *****************************************************************************/
extern OSLAYER_STATUS osSemaphoreTryWait(osSemaphore *pSem);

/******************************************************************************
 *  osSemaphorePost()
 *****************************************************************************
 *  @brief  Increase the semaphore value.
 *
 *  @param  pSem                      Reference of the semaphore object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Semaphore value successfully increased
 *  @retval OSLAYER_ERROR             Tried to increase the value of a not
 *                                    initialized semaphore object
 *  @retval OSLAYER_OPERATION_FAILED  Increase of semaphore value failed
 *
 *****************************************************************************/
extern OSLAYER_STATUS osSemaphorePost(osSemaphore *pSem);

/******************************************************************************
 *   osSemaphoreDestroy()
 *****************************************************************************
 *  @brief  Destroy the semaphore object.
 *
 *  @param  pSem        Reference of the semaphore object
 *
 *  @return             always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osSemaphoreDestroy(osSemaphore *pSem);
#endif


#ifdef OSLAYER_QUEUE
/******************************************************************************
 *  osQueueInit()
 *****************************************************************************
 *  @brief  Initialize queue object.
 *
 *  Init a queue. Item size and item number can be set.
 *
 *  @param  pQueue         Reference of the queue object.
 *
 *  @param  ItemNum        Number of items this queue can hold.
 *
 *  @param  ItemSize       Size of a single queue item.
 *
 *  @return                Status of operation.
 *  @retval OSLAYER_OK     Queue successfully created.
 *  @retval OSLAYER_ERROR  Queue is not created.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osQueueInit(osQueue *pQueue, uint32_t ItemNum, uint32_t ItemSize);



/******************************************************************************
 *  osQueueEmpty()
 *****************************************************************************
 *  @brief   is the queue empty.
 *
 *    is the queue empty.
 *
 *  @param  pQueue         Reference of the queue object.
 *
 *  @return                Status of operation.
 *  @retval OSLAYER_OK     Queue is empty.
 *  @retval OSLAYER_ERROR  Queue is not empty.
 *
 *****************************************************************************/
extern bool_t osQueueEmpty(osQueue *pQueue);

/******************************************************************************
 *  osQueueRead()
 *****************************************************************************
 *  @brief  Blocking read from the queue.
 *
 *  Wait for item being available in the queue and block calling thread, then
 *  copy item from queue.
 *  The function call returns immediatly, if an item is already available.
 *
 *  @param  pQueue                    Reference of the queue object.
 *  @param  pvItem                    Reference to item to read from queue.
 *
 *  @return                           Status of operation.
 *  @retval OSLAYER_OK                Reading from queue succeeded.
 *  @retval OSLAYER_ERROR             Tried to read from a uninitialized queue
 *  @retval OSLAYER_OPERATION_FAILED  Reading from queue failed.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osQueueRead(osQueue *pQueue, void *pvItem);

/******************************************************************************
 *  osQueueTimedRead()
 *****************************************************************************
 *  @brief  Blocking read from the queue with timeout.
 *
 *  Wait for item being available in the queue and block calling thread, then
 *  copy item from queue.
 *  The function call returns immediatly, if an item is already available.
 *
 *  @param  pQueue                    Reference of the queue object.
 *  @param  pvItem                    Reference to item to read from queue.
 *  @param  msec                      Value of Waiting Time.
 *
 *  @return                           Status of operation.
 *  @retval OSLAYER_OK                Reading from queue succeeded.
 *  @retval OSLAYER_ERROR             Tried to read from a uninitialized queue
 *  @retval OSLAYER_TIMEOUT           Timeout occurred while waiting for item
 *                                    being available in the queue.
 *  @retval OSLAYER_OPERATION_FAILED  Reading from queue failed.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osQueueTimedRead(osQueue *pQueue, void *pvItem, uint32_t msec);

/******************************************************************************
 *  osQueueTryRead()
 *****************************************************************************
 *  @brief  Non-blocking read from the queue.
 *
 *  If an item is available in the queue, then copy item from queue. If no
 *  item is available, then return with error.
 *
 *  @param  pQueue                    Reference of the queue object.
 *
 *  @param  pvItem                    Reference to item to read from queue.
 *
 *  @return                           Status of operation.
 *  @retval OSLAYER_OK                Reading from queue succeeded.
 *  @retval OSLAYER_ERROR             Tried to read from a uninitialized queue.
 *  @retval OSLAYER_TIMEOUT           No item was available in the queue.
 *  @retval OSLAYER_OPERATION_FAILED  Reading from queue failed.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osQueueTryRead(osQueue *pQueue, void *pvItem);

/******************************************************************************
 *  osQueueWrite()
 *****************************************************************************
 *  @brief  Blocking write into the queue.
 *
 *  Wait for space in queue being available and block calling thread, then
 *  copy item into queue.
 *  The function call returns immediatly, if space is already available.
 *
 *  @param  pQueue                    Reference of the queue object.
 *  @param  pvItem                    Reference to item to write into queue.
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Writing into queue succeeded.
 *  @retval OSLAYER_ERROR             Tried to write into a uninitialized queue
 *  @retval OSLAYER_OPERATION_FAILED  Writing into queue failed.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osQueueWrite(osQueue *pQueue, void *pvItem);

OSLAYER_STATUS osQueueWriteFromISR(osQueue *pQueue, void* pvItem);

/******************************************************************************
 *  osQueueTimedWrite()
 *****************************************************************************
 *  @brief  Blocking write into the queue with timeout.
 *
 *  Wait for space in queue being available and block calling thread, then
 *  copy item into queue.
 *  The function call returns immediatly, if space is already available.
 *
 *  @param  pQueue                    Reference of the queue object.
 *  @param  pvItem                    Reference to item to write into queue.
 *  @param  msec                      Timeout value in milliseconds.
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Writing into queue succeeded.
 *  @retval OSLAYER_ERROR             Tried to write into a uninitialized queue
 *  @retval OSLAYER_TIMEOUT           Timeout occurred while waiting for space
 *                                    being available in the queue.
 *  @retval OSLAYER_OPERATION_FAILED  Writing into queue failed.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osQueueTimedWrite(osQueue *pQueue, void *pvItem, uint32_t msec);

/******************************************************************************
 *  osQueueTryWrite()
 *****************************************************************************
 *  @brief  Non-blocking write into the queue.
 *
 *  If a osFree item is available in the queue, then copy item into queue. If
 *  no osFree item is available, then return with error.
 *
 *  @param  pQueue                    Reference of the queue object.
 *
 *  @param  pvItem                    Reference to item to read from queue.
 *
 *  @return                           Status of operation.
 *  @retval OSLAYER_OK                Writing into queue succeeded.
 *  @retval OSLAYER_ERROR             Tried to write into a not initialized
 *                                    queue.
 *  @retval OSLAYER_TIMEOUT           Writing into queue failed, as no osFree
 *                                    item was available in the queue.
 *  @retval OSLAYER_OPERATION_FAILED  Writing into queue failed.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osQueueTryWrite(osQueue *pQueue, void *pvItem);

/******************************************************************************
 *  osQueueDestroy()
 *****************************************************************************
 *  @brief  Destroy the queue.
 *
 *  Destroy the queue and osFree resources associated with queue object.
 *
 *  @param  pQueue      Reference of the queue object
 *
 *  @return             always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osQueueDestroy(osQueue *pQueue);
#ifdef OSLAYER_QUEUE_DEBUG
extern OSLAYER_STATUS osQueuePrintAll(void);
#endif
#endif


#ifdef OSLAYER_ATOMIC
/******************************************************************************
 *  osAtomicInit()
 *****************************************************************************
 *  @brief  Initialize atomic operation functionality.
 *
 *  This function must be called before any other osAtomicXXX call.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osAtomicInit(void);

/******************************************************************************
 *  osAtomicShutdown()
 *****************************************************************************
 *  @brief  Shutdown atomic operation functionality.
 *
 *  This function must be called before process is terminated
 *  when osAtomicInit has been called before.
 *
 *****************************************************************************/
extern OSLAYER_STATUS osAtomicShutdown(void);

/******************************************************************************
 *  osAtomicTestAndClearBit()
 *****************************************************************************
 *  @brief  Test and set a bit position atomically.
 *
 *  Test if a bit position inside a variable is set and clears the bit
 *  afterwards. The complete operation is atomic (includes IRQ safety).
 *
 *  @param  pVar           32-bit unsigned variable to be modified
 *  @param  bitpos         Bit to be tested and cleared
 *
 *  @return                *pVar & (1 << bitpos)
 *
 *****************************************************************************/
extern OSLAYER_STATUS osAtomicTestAndClearBit(uint32_t *pVar, uint32_t bitpos);

/******************************************************************************
 *  osAtomicIncrement()
 ******************************************************************************
 *  @brief  Increments a 32-bit unsigned variable atomically.
 *
 *  Increments a 32-bit unsigned variable atomically and returns the value of
 *  the variable after the increment operation. The complete operation is
 *  atomic (includes IRQ safety).
 *
 *  @param  pVar           32-bit unsigned variable to be modified
 *
 *  @return                ++(*pVar)
 *
 *****************************************************************************/
extern OSLAYER_STATUS osAtomicIncrement(uint32_t *pVar);

/******************************************************************************
 *  osAtomicDecrement()
 ******************************************************************************
 *  @brief  Decrements a 32-bit unsigned variable atomically.
 *
 *  Decrements a 32-bit unsigned variable atomically and returns the value of
 *  the variable after the decrement operation. The complete operation is
 *  atomic (includes IRQ safety).
 *
 *  @param  pVar           32-bit unsigned variable to be modified
 *
 *  @return                --(*pVar)
 *
 *****************************************************************************/
extern OSLAYER_STATUS osAtomicDecrement(uint32_t *pVar);

/******************************************************************************
 *  osAtomicSetBit()
 *****************************************************************************
 *  @brief  Set a bit position atomically.
 *
 *  Set a bit position inside a variable.
 *  The operation is atomic (includes IRQ safety).
 *
 *  @param  pVar           32-bit unsigned variable to be modified
 *  @param  bitpos         Bit to be set
 *
 *  @return                always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osAtomicSetBit(uint32_t *pVar, uint32_t bitpos);

/******************************************************************************
 * osAtomicSet()
 *****************************************************************************
 *  @brief  Set value atomically.
 *
 *  Set variable to value. The operation is atomic (includes IRQ safety).
 *
 *  @param  pVar           32-bit unsigned variable to be modified
 *
 *  @param  value          Value to be set
 *
 *  @return                always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osAtomicSet(uint32_t *pVar, uint32_t value);
#endif


#ifdef OSLAYER_THREAD
/******************************************************************************
 *  osThreadCreate()
 *****************************************************************************
 *  @brief  Create a thread.
 *
 *  @param  pThread                   Reference of the semaphore object
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Thread object created successfully
 *  @retval OSLAYER_OPERATION_FAILED  Creation of thread object failed
 *
 *****************************************************************************/
extern OSLAYER_STATUS osThreadCreate(osThread *pThread, osThreadFunc thread_func,
				void *arg);

/******************************************************************************
 *  osThreadSetPriority()
 *****************************************************************************
 *  @brief  Set thread priority.
 *
 *  @param  pThread                   Reference of the semaphore object
 *
 *  @param  priority                  New thread priority to be set
 *
 *  @return                           Status of operation
 *  @retval OSLAYER_OK                Thread priority changed successfully
 *  @retval OSLAYER_ERROR             Invalid thread priority value passed
 *  @retval OSLAYER_OPERATION_FAILED  Thread priority change failed
 *
 *****************************************************************************/
extern OSLAYER_STATUS osThreadSetPriority(osThread *pThread,
				OSLAYER_THREAD_PRIO priority);

/******************************************************************************
 *   osThreadWait()
 *****************************************************************************
 *  @brief  Wait until thread exits.
 *
 *          To avoid memory leak, always call @ref osThreadWait and wait
 *          for the child thread to terminate in the calling thread.
 *
 *  @param  pThread        Reference of the semaphore object
 *
 *  @return                always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osThreadWait(osThread *pThread);
extern OSLAYER_STATUS osThreadTimedWait(osThread *pThread, uint32_t msec);

/******************************************************************************
 *  osThreadClose()
 *****************************************************************************
 *  @brief  Destroy thread object.
 *
 *  @param  pThread       Reference of the semaphore object
 *
 *  @return               always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osThreadClose(osThread *pThread);
extern OSLAYER_STATUS osThreadCancel(osThread *pThread);
extern OSLAYER_STATUS osThreadTimedClose(osThread *pThread, uint32_t msec);
#endif

OSLAYER_STATUS osTaskNotifyWait(uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit,
				uint32_t *pulNotificationValue, osTickType xTicksToWait);
void osTaskDelete(osThread *pThread);
void osTaskDelay(uint32_t msec);
OSLAYER_STATUS osTaskNotifyFromISR(osTaskHandle *pxTaskToNotify, uint32_t ulValue,
				osTaskNotificationAction eAction, osBaseType *pxHigherPriorityTaskWoken);
void osPortYieldFromISR(osBaseType *pxHigherPriorityTaskWoken);
void osTaskYield(void);
OSLAYER_STATUS osPortInstallInterruptHandler(uint16_t ucInterruptID, XInterruptHandler pxHandler, void *pvCallBackRef);
OSLAYER_STATUS osPortEnableInterrupt(uint16_t ucInterruptID);
OSLAYER_STATUS osPortDisableInterrupt(uint16_t ucInterruptID);
#if(SAFERTOS)
OSLAYER_STATUS osPortSetInterruptPriority(uint16_t uxInterruptID, portUInt32Type ulPriority);
#endif
OSLAYER_STATUS osInitScheduler();
OSLAYER_STATUS osStartScheduler();


#ifdef OSLAYER_MISC
/******************************************************************************
 *  osSleep()
 *****************************************************************************
 *  @brief  Yield the execution of current thread for msec miliseconds.
 *
 *  @param  msec       Wait time in millisecobds
 *
 *  @return            always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osSleep(uint32_t msec);

/******************************************************************************
 *   osGetTick()
 *****************************************************************************
 *  @brief  Obtain the clock tick.
 *
 *  @return            Current clock tick. The resoultion of clock tick can
 *                     be requested with @ref osGetFrequency
 *
 *****************************************************************************/
extern uint64_t osGetTick(void);

/******************************************************************************
 *  osGetFrequency()
 *****************************************************************************
 *  @brief  Obtain the clock resolution.
 *
 *  @return            The resoultion of the clock tick.
 *
 *****************************************************************************/
extern uint64_t osGetFrequency(void);

#if defined LINUX && OSLAYER_KERNEL
/******************************************************************************
 *  osMallocEx()
 *****************************************************************************
 *  @brief  Allocate a continuous block of memory.
 *
 *  @param  size       Size of memory block to be allocated
 *
 *  @param  type       Type of memory block to be allocated
 *
 *****************************************************************************/
void *osMallocEx(uint32_t size, uint32_t type);
#endif

/******************************************************************************
 *  osMalloc()
 *****************************************************************************
 *  @brief  Allocate a continuous block of memory.
 *
 *  @param  size       Size of memory block to be allocated
 *
 *****************************************************************************/
extern void *osMalloc(size_t size);

/******************************************************************************
 *  osFree()
 *****************************************************************************
 *  @brief  osFree a continuous block of memory.
 *
 *  @param  p       Pointer to previously allocated memory block
 *
 *  @return         always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osFree(void *p);


#ifdef OSLAYER_KERNEL
/******************************************************************************
 *  osSpinLockInit()
 *****************************************************************************
 *  @brief  Init a spin lock object.
 *
 *  @param  p_spin_lock      Reference of the spin lock object
 *
 *  @return                  always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osSpinLockInit(osSpinLock * p_spin_lock);

/******************************************************************************
 *  osSpinLockAcquire()
 *****************************************************************************
 *  @brief  Acquire a spin lock object.
 *
 *  @param  p_spin_lock      Reference of the spin lock object
 *
 *  @param  flags            Interrupt context flags which need to be safed
 *                           in a local variable
 *
 *  @return                  always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osSpinLockAcquire(osSpinLock *p_spin_lock, uint32_t flags);

/******************************************************************************
 *   osSpinLockRelease()
 *****************************************************************************
 *  @brief  Release a spin lock object.
 *
 *  @param  p_spin_lock      Reference of the spin lock object
 *
 *  @param  flags            Interrupt context flags which need to be safed
 *                           in a local variable
 *
 *  @return                  always OSLAYER_OK
 *****************************************************************************/
extern OSLAYER_STATUS osSpinLockRelease(osSpinLock *p_spin_lock, uint32_t flags);

#endif
#endif

#ifdef OS_FILE_SYSTEM_SUPPORT
typedef struct _osFile
{
#ifdef LINUX
	FILE *file;
#endif
#if defined(FREERTOS) || defined(SAFERTOS)
	FIL file;
#endif
} osFile;

osFile* osFopen(const char *filename, const char *mode);
void osFclose(osFile* pVsiFile);
size_t osFread(void* ptr, size_t size, size_t count, osFile* pVsiFile);
size_t osFwrite(const void* ptr, size_t size, size_t count, osFile* pVsiFile);
int osFseek(osFile* pVsiFile, long int offset, int origin);
long int osFtell(osFile* pVsiFile);
long int osFileSize(osFile* pVsiFile);
int osFeof(osFile* pVsiFile);
int osFerror(osFile* pVsiFile);
int osFgetc(osFile* pVsiFile);
char* osFgets(char* str, int size, osFile* pVsiFile);
int osFputs(const char* str, osFile* file);
int osFscanf(osFile* pVsiFile, const char* format, ...);
int osFprintf(osFile* pVsiFile, const char* format, ...);
#endif

void osUSleep(unsigned int microseconds);

#ifdef __cplusplus
}
#endif

#endif
