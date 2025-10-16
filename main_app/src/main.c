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

/* Xilinx includes. */
#include "xil_printf.h"
#include "cpu_info.h"

#define VER_MAJOR (0U)
#define VER_MINOR (3U)

#define SDK_RELEASE_YEAR    (2025)
#define SDK_RELEASE_QUARTER (2)

#if (ELF_FLAG)
int main( void )
{
	xil_printf("\n****************************************\n");
	xil_printf("Versal Gen2 ISP Firmware\n");
	xil_printf("Release %d.%d ",SDK_RELEASE_YEAR, SDK_RELEASE_QUARTER);
	xil_printf("%s - %s \n", __DATE__, __TIME__);
	xil_printf("ISP FW Version : v%d.%d\n", VER_MAJOR, VER_MINOR);
	xil_printf("****************************************\n");
	print_memory_layout_info();
#else
int main_lib( void )
{
	xil_printf("\n****************************************\n");
	xil_printf("Versal Gen2 ISP Firmware\n");
	xil_printf("Release %d.%d ",SDK_RELEASE_YEAR, SDK_RELEASE_QUARTER);
	xil_printf("%s - %s \n", __DATE__, __TIME__);
	xil_printf("ISP FW Version : v%d.%d\n", VER_MAJOR, VER_MINOR);
	xil_printf("****************************************\n");
#endif
	xil_printf( "Running Firmware for cpu-id: %d\r\n", get_cpu_id() );

	amp_core_data_init();
//  test_spinlock();
	control_init();
//  stats();
	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
}

