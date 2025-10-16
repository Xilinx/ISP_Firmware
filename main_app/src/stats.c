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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"

#define TIMER_ID                        1
#define DELAY_1_SECOND          5000UL
/*-----------------------------------------------------------*/
 //int mcm_debug_trace_var_debug;

/* The prototype of entry point of tasks */
static void vTimerCallback( TimerHandle_t pxTimer );
/*-----------------------------------------------------------*/

static TimerHandle_t xTimer = NULL;
char taskbuffer[4096];
char taskbuffer1[4096];

int stats( void )
{
        const TickType_t x1seconds = pdMS_TO_TICKS( 10*DELAY_1_SECOND );

        xil_printf( "\r\nHello from Freertos example main\r\n" );

        /* Create a timer with a timer expiry of 1 seconds. The timer would expire
         after 1 seconds and the timer call back would get called. In the timer call back
         checks are done to ensure that the tasks have been running properly till then.
         The tasks are deleted in the timer call back and a message is printed to convey that
         the example has run successfully.
         The timer expiry is set to 1 seconds and the timer set to not auto reload. */
        xTimer = xTimerCreate( (const char *) "Timer",
                                                        x1seconds,
                                                        //pdFALSE,
														pdTRUE,
                                                        (void *) TIMER_ID,
                                                        vTimerCallback);
        /* Check the timer was created. */
        configASSERT( xTimer );

        /* start the timer with a block time of 0 ticks. This means as soon
           as the schedule starts the timer will start running and will expire after
           1 seconds */
        xTimerStart( xTimer, 0 );
}
/*-----------------------------------------------------------*/
static void vTimerCallback( TimerHandle_t pxTimer )
{
        HeapStats_t * heapStats;
        long lTimerId;
        configASSERT( pxTimer );

        lTimerId = ( long ) pvTimerGetTimerID( pxTimer );

        if (lTimerId != TIMER_ID) {
                xil_printf("\r\nFreeRTOS Task Stats Example FAILED\r\n");
        }

#if (configGENERATE_RUN_TIME_STATS == 1)
        /*
         * The existing example assumes the freertos tick rate of 100.
         * Xilinx implementation of statistic counter uses a timer that has a
         * resolution of tick rate * 10. The multiplying factor of 10 is hard-coded
         * for Xilinx FreeRTOS port and cannot be changed.
         * For this example the statistics counter unit based on the above
         * assumptions is milliseconds (ms).
         * In case users configure different tick rate the stats counter unit will
         * have to be changed accordingly.
         */
        vTaskGetRunTimeStats(taskbuffer);
        xil_printf("\r\nTask Name  Task Abs Time(ms)  Task Time (in percentage)\r\n");
        xil_printf("_________  _________________  ___________________________\r\n");
        xil_printf("%s\r\n",taskbuffer);

        vTaskList(taskbuffer1);
        xil_printf("\r\nTask Name  State  Priority  Stack  Task No\r\n");
        xil_printf("_________  _______  ________  ______  _______\r\n");
        xil_printf("%s\r\n",taskbuffer1);
//        vPortGetHeapStats(heapStats);
//        xil_printf("xAvailableHeapSpaceInBytes: %d\n",heapStats->xAvailableHeapSpaceInBytes);
#endif
  //      xil_printf("FreeRTOS Task Stats Example PASSED");

}
