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
#include "isp_fw_main.h"
#include "xil_mpu.h"
#include "xreg_cortexr5.h"

XMpuConfig_Initial InitialMpu_Config __attribute__((section(".bootdata"))) = {

	{
		/* TCM  */
		0x00000000U,
		0x2000,
		NORM_NSHARED_WT_NWA | PRIV_RO_USER_RO,
	},
	{
		/* BootData */
		0x2400,
		0x20000,
		NORM_NSHARED_WT_NWA | PRIV_RW_USER_RW,
	}, 
	{
		RPU6_FW_START_ADDR,
		RPU6_FW_SIZE,
		NORM_NSHARED_WT_NWA | PRIV_RW_USER_RW,
	},
	{
	
		RPU6_LOAD_CALIB_START_ADDR,
      		RPU6_LOAD_CALIB_PRIV_MEM_SIZE,
		STRONG_ORDERD_SHARED | PRIV_RW_USER_RW,


	},
	{

	        RPU6_MBOX_START_ADDR,
		RPU6_MBOX_RPUSHM_SIZE,
		STRONG_ORDERD_SHARED | PRIV_RW_USER_RW,
	},
	{
		/* 512 MB LPD to AFI fabric slave port */
		0x80000000U,
		0x1FFFFFFF,
		DEVICE_NONSHARED | PRIV_RW_USER_RW,
	},
	{
		0xA0000000U,
		0x17FFFFFF,
		DEVICE_NONSHARED | PRIV_RW_USER_RW,
	},
	{
		/* 2 MB OCM */
		0xBBE00000U,
		0x1FFFFF,
		NORM_NSHARED_WT_NWA | PRIV_RW_USER_RW,
	},
	{
		/* 512 MB xSPI + 16 MB Coresight */
		0xC0000000U,
		0x20FFFFFF,
		DEVICE_NONSHARED | PRIV_RW_USER_RW,
	},
	{
		/* 2MB RPU GIC */
		0xE2000000U,
		0x1FFFFF,
		DEVICE_NONSHARED | PRIV_RW_USER_RW,
	},
	{
	    /* 8MB VCU and ISP */
		0xE8000000U,
		0x7FFFFF,
		DEVICE_NONSHARED | PRIV_RW_USER_RW,
        },
	{
		/* 16MB FPD + 32MB LPD + 16MB MMI */
		0xEA000000U,
		0x3FFFFFF,
		DEVICE_NONSHARED | PRIV_RW_USER_RW,
	},
	{
		/* 128MB PMC + 64MB PS_FPD_CMN */
		0xF0000000U,
		0xBFFFFFF,
		DEVICE_NONSHARED | PRIV_RW_USER_RW,
	},
	/* A total of 9 MPU regions are allocated with another 7 being free for users */
	{
		0U
	}
};

uint32_t cam_load_calib			= RPU6_LOAD_CALIB_START_ADDR;
uint32_t MBOX_start_Addr  		= RPU6_MBOX_START_ADDR;
uint32_t _MBOX_MEM_SIZE 		= RPU6_MBOX_SIZE;
uint32_t HAL_RESERVED_MEM_PRIV_START 	= RPU6_PRIV_MEM_START_ADDR ;
uint32_t HAL_RESERVED_MEM_PRIV_SIZE	= RPU6_PRIV_MEM_SIZE;

extern int main_lib();

int main()
{
    print_memory_layout_info();
    main_lib();
    return 0;
}

void print_memory_layout_info( )
{
	xil_printf("RPU_FW_START_ADDR - 0x%x \n",RPU6_FW_START_ADDR);
	xil_printf("RPU_FW_SIZE - 0x%x \n",RPU6_FW_SIZE);
	xil_printf("RPU_LOAD_CALIB_START_ADDR - 0x%x \n",RPU6_LOAD_CALIB_START_ADDR);
	xil_printf("RPU_LOAD_CALIB_SIZE - 0x%x \n",RPU6_LOAD_CALIB_SIZE);
	xil_printf("RPU_PRIV_MEM_START_ADDR - 0x%x \n",RPU6_PRIV_MEM_START_ADDR);
	xil_printf("RPU_PRIV_MEM_SIZE - 0x%x \n",RPU6_PRIV_MEM_SIZE);
	xil_printf("RPU_MBOX_START_ADDR - 0x%x \n",RPU6_MBOX_START_ADDR);
	xil_printf("RPU_MBOX_SIZE - 0x%x \n",RPU6_MBOX_SIZE);
	xil_printf("RPU_SHM_SIZE - 0x%x \n",RPU6_SHM_SIZE);
	xil_printf("RPU_MBOX_RPUSHM_SIZE - 0x%x \n",RPU6_MBOX_RPUSHM_SIZE);
	xil_printf("RPU_LOAD_CALIB_PRIV_MEM_SIZE - 0x%x \n",RPU6_LOAD_CALIB_PRIV_MEM_SIZE);
}
