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

#include "xil_mpu.h"
#include "xreg_cortexr5.h"
#include "xipipsu.h"
#include "xwdttb.h"
#include "isp_fw_main.h"
#include <stdint.h>
#include "memory_layout.h"




#if defined (RPU6_FW)

UINTPTR XTickTimer_Instance = XPAR_TTC12_BASEADDR;

XIpiPsu_Config XIpiPsu_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,zynqmp-ipi-mailbox", /* compatible */
		0xeb3b2000, /* reg */
		0x1000, /* xlnx,ipi-bitmask */
		0xffff, /* xlnx,ipi-buf-index */
		0x4042, /* interrupts */
		0xe2000000, /* interrupt-parent */
		0x10, /* xlnx,ipi-target-count */
		{
			{
				1, /* xlnx,ipi-bitmask */
				0 /* xlnx,ipi-buf-index */
			},
			{
				4, /* xlnx,ipi-bitmask */
				2 /* xlnx,ipi-buf-index */
			},
			{
				8, /* xlnx,ipi-bitmask */
				3 /* xlnx,ipi-buf-index */
			},
			{
				1024, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				16, /* xlnx,ipi-bitmask */
				4 /* xlnx,ipi-buf-index */
			},
			{
				2048, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32, /* xlnx,ipi-bitmask */
				5 /* xlnx,ipi-buf-index */
			},
			{
				4096, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				64, /* xlnx,ipi-bitmask */
				6 /* xlnx,ipi-buf-index */
			},
			{
				8192, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				128, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				16384, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				512, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32768, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				2, /* xlnx,ipi-bitmask */
				1 /* xlnx,ipi-buf-index */
			},
			{
				256, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
		}, /* child,required */
	},
	{
	NULL
	}
};
#endif

#if defined (RPU7_FW)

UINTPTR XTickTimer_Instance = XPAR_TTC15_BASEADDR;


XIpiPsu_Config XIpiPsu_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {
	
	{
		"xlnx,zynqmp-ipi-mailbox", /* compatible */
		0xeb3b3000, /* reg */
		0x2000, /* xlnx,ipi-bitmask */
		0xffff, /* xlnx,ipi-buf-index */
		0x4043, /* interrupts */
		0xe2000000, /* interrupt-parent */
		0x10, /* xlnx,ipi-target-count */
		{
			{
				1, /* xlnx,ipi-bitmask */
				0 /* xlnx,ipi-buf-index */
			},
			{
				4, /* xlnx,ipi-bitmask */
				2 /* xlnx,ipi-buf-index */
			},
			{
				8, /* xlnx,ipi-bitmask */
				3 /* xlnx,ipi-buf-index */
			},
			{
				1024, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				16, /* xlnx,ipi-bitmask */
				4 /* xlnx,ipi-buf-index */
			},
			{
				2048, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32, /* xlnx,ipi-bitmask */
				5 /* xlnx,ipi-buf-index */
			},
			{
				4096, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				64, /* xlnx,ipi-bitmask */
				6 /* xlnx,ipi-buf-index */
			},
			{
				8192, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				128, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				16384, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				512, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32768, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				2, /* xlnx,ipi-bitmask */
				1 /* xlnx,ipi-buf-index */
			},
			{
				256, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
		}, /* child,required */
	},
	{
		 NULL
	}
};
#endif

#if defined (RPU8_FW)

UINTPTR XTickTimer_Instance = XPAR_TTC18_BASEADDR;

XIpiPsu_Config XIpiPsu_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {
	{
		"xlnx,zynqmp-ipi-mailbox", /* compatible */
		0xeb3b4000, /* reg */
		0x4000, /* xlnx,ipi-bitmask */
		0xffff, /* xlnx,ipi-buf-index */
		0x4044, /* interrupts */
		0xe2000000, /* interrupt-parent */
		0x10, /* xlnx,ipi-target-count */
		{
			{
				1, /* xlnx,ipi-bitmask */
				0 /* xlnx,ipi-buf-index */
			},
			{
				4, /* xlnx,ipi-bitmask */
				2 /* xlnx,ipi-buf-index */
			},
			{
				8, /* xlnx,ipi-bitmask */
				3 /* xlnx,ipi-buf-index */
			},
			{
				1024, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				16, /* xlnx,ipi-bitmask */
				4 /* xlnx,ipi-buf-index */
			},
			{
				2048, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32, /* xlnx,ipi-bitmask */
				5 /* xlnx,ipi-buf-index */
			},
			{
				4096, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				64, /* xlnx,ipi-bitmask */
				6 /* xlnx,ipi-buf-index */
			},
			{
				8192, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				128, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				16384, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				512, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32768, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				2, /* xlnx,ipi-bitmask */
				1 /* xlnx,ipi-buf-index */
			},
			{
				256, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
		}, /* child,required */
	},
	{
		NULL
	}
};
#endif

#if defined (RPU9_FW)

UINTPTR XTickTimer_Instance = XPAR_TTC21_BASEADDR;

XIpiPsu_Config XIpiPsu_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,zynqmp-ipi-mailbox", /* compatible */
		0xeb3b5000, /* reg */
		0x8000, /* xlnx,ipi-bitmask */
		0xffff, /* xlnx,ipi-buf-index */
		0x4045, /* interrupts */
		0xe2000000, /* interrupt-parent */
		0x10, /* xlnx,ipi-target-count */
		{
			{
				1, /* xlnx,ipi-bitmask */
				0 /* xlnx,ipi-buf-index */
			},
			{
				4, /* xlnx,ipi-bitmask */
				2 /* xlnx,ipi-buf-index */
			},
			{
				8, /* xlnx,ipi-bitmask */
				3 /* xlnx,ipi-buf-index */
			},
			{
				1024, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				16, /* xlnx,ipi-bitmask */
				4 /* xlnx,ipi-buf-index */
			},
			{
				2048, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32, /* xlnx,ipi-bitmask */
				5 /* xlnx,ipi-buf-index */
			},
			{
				4096, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				64, /* xlnx,ipi-bitmask */
				6 /* xlnx,ipi-buf-index */
			},
			{
				8192, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				128, /* xlnx,ipi-bitmask */
				7 /* xlnx,ipi-buf-index */
			},
			{
				16384, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				512, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				32768, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
			{
				2, /* xlnx,ipi-bitmask */
				1 /* xlnx,ipi-buf-index */
			},
			{
				256, /* xlnx,ipi-bitmask */
				65535 /* xlnx,ipi-buf-index */
			},
		}, /* child,required */
	},
	{
		 NULL
	}
};

#endif

XMpuConfig_Initial InitialMpu_Config __attribute__((section(".bootdata"))) = {

{
		/* TCM  */
		0x00000000U,
		0x2000,
		NORM_NSHARED_WT_NWA | PRIV_RO_USER_RO,
	},
 
	{
		/* BootData */
		0x2000,
		0x1E000,
		NORM_NSHARED_WT_NWA | PRIV_RW_USER_RW,
	},
	{
		/* TEXT */
		RPU_FW_START_ADDR,
		RPU_FW_SIZE,
		NORM_NSHARED_WT_NWA | PRIV_RW_USER_RW,
	},

	{
		/* TRACE LOG */
		RPU_TRACE_LOG_START_ADDR,
		RPU_TRACE_LOG_SIZE,
		STRONG_ORDERD_SHARED | PRIV_RW_USER_RW,
	},

	{
	
		RPU_LOAD_CALIB_START_ADDR,
      	RPU_LOAD_CALIB_PRIV_MEM_SIZE,
		 STRONG_ORDERD_SHARED | PRIV_RW_USER_RW,


	},

	{

	    RPU_MBOX_START_ADDR,
		RPU_MBOX_RPUSHM_FMC_SIZE,
		STRONG_ORDERD_SHARED | PRIV_RW_USER_RW,
	},

#ifdef BM_TESTAPP
	{

	    RPU_TDATABASE_BIN_ADDR,
		RPU_TDATABASE_BIN_SIZE,
		STRONG_ORDERD_SHARED | PRIV_RW_USER_RW,
	},
#endif

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




uint32_t cam_load_calib = RPU_LOAD_CALIB_START_ADDR; //TODO:Change load calib based on CPU_ID
uint32_t MBOX_start_Addr  = RPU_MBOX_START_ADDR;
uint32_t  _MBOX_MEM_SIZE = RPU_MBOX_SIZE;

uint32_t HAL_RESERVED_MEM_PRIV_START = RPU_PRIV_MEM_START_ADDR;
uint32_t HAL_RESERVED_MEM_PRIV_SIZE  = RPU_PRIV_MEM_SIZE;

#ifdef BM_TESTAPP
uint32_t VSI_MMB_RESERVED_MEM_OPEN_START = RPU_OPEN_MEM_START_ADDR;
uint32_t VSI_MMB_RESERVED_MEM_OPEN_SIZE  = RPU_OPEN_MEM_SIZE;
#else
uint32_t VSI_MMB_RESERVED_MEM_OPEN_START =
    RPU_PRIV_MEM_START_ADDR + RPU_PRIV_MEM_SIZE - VSI_MMB_OPEN_REGION_SIZE;

uint32_t VSI_MMB_RESERVED_MEM_OPEN_SIZE = VSI_MMB_OPEN_REGION_SIZE;
#endif

uint32_t TRACE_LOG_START_ADDR = RPU_TRACE_LOG_START_ADDR;
void print_memory_layout_info( )
{
	xil_printf("RPU_FW_START_ADDR - 0x%x \n",RPU_FW_START_ADDR);
	xil_printf("RPU_FW_SIZE - 0x%x \n",RPU_FW_SIZE);
	xil_printf("RPU_LOAD_CALIB_START_ADDR - 0x%x \n",RPU_LOAD_CALIB_START_ADDR);
	xil_printf("RPU_LOAD_CALIB_SIZE - 0x%x \n",RPU_LOAD_CALIB_SIZE);
	xil_printf("RPU_PRIV_MEM_START_ADDR - 0x%x \n",RPU_PRIV_MEM_START_ADDR);
	xil_printf("RPU_PRIV_MEM_SIZE - 0x%x \n",RPU_PRIV_MEM_SIZE);
	xil_printf("RPU_MBOX_START_ADDR - 0x%x \n",RPU_MBOX_START_ADDR);
	xil_printf("RPU_MBOX_SIZE - 0x%x \n",RPU_MBOX_SIZE);
	xil_printf("RPU_SHM_SIZE - 0x%x \n",RPU_SHM_SIZE);
	xil_printf("RPU_MBOX_RPUSHM_FMC_SIZE - 0x%x \n",RPU_MBOX_RPUSHM_FMC_SIZE);
	xil_printf("RPU_LOAD_CALIB_PRIV_MEM_SIZE - 0x%x \n",RPU_LOAD_CALIB_PRIV_MEM_SIZE);
}

/* -----------------------------------------------------------------------
 * XWdtTb_ConfigTable - per-core LPD WWDT override
 *
 * The BSP (built for freertos_cortexr52_6) only enumerates lpd_wwdt0 and
 * lpd_wwdt1.  We override the weak BSP symbol here -- one entry per ELF,
 * selected by the compile-time RPUx_FW macro -- so that XWdtTb_LookupConfig()
 * finds the correct instance regardless of which ELF is running.
 *
 * Field order matches XWdtTb_Config (SDT flow):
 *   Name, BaseAddr, EnableWinWdt, MaxCountWidth, SstCountWidth, Clock,
 *   IntrId[4], IntrParent
 *
 * Base addresses follow a linear 0x10000 stride per instance.
 * Interrupt IDs follow a +4 stride per instance.
 * ----------------------------------------------------------------------- */

#if defined(RPU6_FW)
XWdtTb_Config XWdtTb_ConfigTable[] __attribute__((section(".drvcfg_sec"))) = {
	{
		"xlnx,versal-wwdt-1.0", /* compatible */
		0xEB000000,             /* lpd_wwdt0 base address */
		0x0,                    /* xlnx,enable-window-wdt */
		0x0,                    /* xlnx,max-count-width */
		0x0,                    /* xlnx,sst-count-width */
		0x8F0CBA9,              /* xlnx,wdt-clk-freq-hz (~150 MHz) */
		{0x10EC, 0x10ED, 0x10EE, 0x10EF}, /* interrupts */
		0xE2000000              /* interrupt-parent (RPU GIC) */
	},
	{ NULL }
};
#elif defined(RPU7_FW)
XWdtTb_Config XWdtTb_ConfigTable[] __attribute__((section(".drvcfg_sec"))) = {
	{
		"xlnx,versal-wwdt-1.0", /* compatible */
		0xEB010000,             /* lpd_wwdt1 base address */
		0x0,                    /* xlnx,enable-window-wdt */
		0x0,                    /* xlnx,max-count-width */
		0x0,                    /* xlnx,sst-count-width */
		0x8F0CBA9,              /* xlnx,wdt-clk-freq-hz (~150 MHz) */
		{0x10F0, 0x10F1, 0x10F2, 0x10F3}, /* interrupts */
		0xE2000000              /* interrupt-parent (RPU GIC) */
	},
	{ NULL }
};
#elif defined(RPU8_FW)
XWdtTb_Config XWdtTb_ConfigTable[] __attribute__((section(".drvcfg_sec"))) = {
	{
		"xlnx,versal-wwdt-1.0", /* compatible */
		0xEB020000,             /* lpd_wwdt2 base address */
		0x0,                    /* xlnx,enable-window-wdt */
		0x0,                    /* xlnx,max-count-width */
		0x0,                    /* xlnx,sst-count-width */
		0x8F0CBA9,              /* xlnx,wdt-clk-freq-hz (~150 MHz) */
		{0x10F4, 0x10F5, 0x10F6, 0x10F7}, /* interrupts */
		0xE2000000              /* interrupt-parent (RPU GIC) */
	},
	{ NULL }
};
#elif defined(RPU9_FW)
XWdtTb_Config XWdtTb_ConfigTable[] __attribute__((section(".drvcfg_sec"))) = {
	{
		"xlnx,versal-wwdt-1.0", /* compatible */
		0xEB030000,             /* lpd_wwdt3 base address */
		0x0,                    /* xlnx,enable-window-wdt */
		0x0,                    /* xlnx,max-count-width */
		0x0,                    /* xlnx,sst-count-width */
		0x8F0CBA9,              /* xlnx,wdt-clk-freq-hz (~150 MHz) */
		{0x10F8, 0x10F9, 0x10FA, 0x10FB}, /* interrupts */
		0xE2000000              /* interrupt-parent (RPU GIC) */
	},
	{ NULL }
};
#endif
