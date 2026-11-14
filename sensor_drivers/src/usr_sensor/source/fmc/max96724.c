/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc. All right reserved.
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
 ****************************************************************************
 *
 *  MAX96724 quad-link GMSL deserializer FMC driver.
 *
 *  Topology assumed for this board port:
 *     - 3 physical MAX96724 chips
 *     - Each chip has 4 GMSL links (A/B/C/D) -> 4 sensors per chip
 *     - Total: 12 sensors across 3 deserializers
 *
 *  Framework adaptation:
 *     desInterface (in isi_fmc.h) now carries 4 dslink members
 *     (link_a, link_b, link_c, link_d), so each MAX96724 chip is exposed
 *     as ONE desInterface entry with all four GMSL links populated:
 *
 *        ndes_arr[0] = chip 1, Links A/B/C/D
 *        ndes_arr[1] = chip 2, Links A/B/C/D
 *        ndes_arr[2] = chip 3, Links A/B/C/D
 *
 *     The pipe -> chip mapping is MAPPING_INPIPE_TO_DES_ID(pipe) = pipe/4
 *     and the link within the chip is MAPPING_INPIPE_TO_LINK_IDX(pipe) =
 *     pipe % 4 (0=A, 1=B, 2=C, 3=D). See ms_fw.c init_sensor().
 *
 *  Symbol naming:
 *     The exported FMC instance keeps the historical name `g_fmc_rave`
 *     so ms_fw.c does not need to change. All internal callbacks are
 *     renamed nQuad_* to avoid collision if max96716.c is kept in the
 *     build during the porting transition.
 *
 *  TODO_PORT markers:
 *     Calls to HalReadI2CReg / HalWriteI2CReg passing IIC_INSTANCE_ZERO
 *     historically routed through the PS I2C controller (bus 0). On the
 *     new board PS I2C is being removed; replace those calls with the
 *     correct AXI bus ID used to reach the expander / LDAC / potentiometer
 *     hardware on the new carrier.
 *
 ****************************************************************************/

#include <fmc/max96724.h>
#include "isi/isi_fmc.h"
#include <hal/amp.h>
#include <sleep.h>

/* sensor_lib is self-contained and has no ebase/dct_assert.h (that lives in
 * telluride/BSP). Provide a local fail-loud assert matching the driver's
 * DCT_ASSERT(0) usage without pulling in telluride headers. */
#ifndef DCT_ASSERT
#define DCT_ASSERT(x) do { if (!(x)) { \
	xil_printf("DCT_ASSERT failed: %s:%d\r\n", __func__, __LINE__); } } while (0)
#endif

extern unsigned int *FMC_INIT_STATUS_REG;
extern unsigned int *DES_REMAPPED_STATUS_REG;
extern unsigned int *SHARED_DES_ARRAY_SRUCT;

extern struct serializer_driver max9295_instance[];
extern struct sensor_driver sensor_instance[];
extern struct serializer_driver max96717_instance[];


/* Forward declarations */
static int nQuad_IsiCreateFmcIss(void);
static int nQuad_Fmc_Setup(int ndes_arr_id);
static int nQuad_Deser_setup(desInterface *des);
static int nQuad_Deser_Enable(u8 pos);
static int nQuad_Deser_Disable(u8 pos);
/* nQuad_Remap_I2C_addresses kept under #if 0 below - each MAX96724
 * is on its own AXI I2C bus on this board, so no remap is required. */
static void nQuad_Probe_links(u8 Deser_addr, desInterface *desIface);

/****************************************************************************
 *  Exported FMC descriptor.
 ****************************************************************************/
IsiFmc_t g_fmc_rave = {
	.FmcName			= "max96724_rave2_iob",
	.pIsiCreateFmcIss		= nQuad_IsiCreateFmcIss,
	.pIsiIsiFmcSetup		= nQuad_Fmc_Setup,
	.pIsiDeserSetup			= nQuad_Deser_setup,
	.pIsiDeserEnable		= nQuad_Deser_Enable,
	.pIsiDeserDisable		= nQuad_Deser_Disable,
	.serializer_array		= {
		&max9295_instance[IN_PIPE_0], //&max96717_instance[IN_PIPE_0],  // Quanta vs Leopard
		&max9295_instance[IN_PIPE_1], //&max96717_instance[IN_PIPE_1],
		&max9295_instance[IN_PIPE_2], //&max96717_instance[IN_PIPE_2],
		&max9295_instance[IN_PIPE_3], //&max96717_instance[IN_PIPE_3],
		&max9295_instance[IN_PIPE_4], // DES1
		&max9295_instance[IN_PIPE_5], // DES1
		&max9295_instance[IN_PIPE_6], // DES1
		&max9295_instance[IN_PIPE_7], // DES1
		&max9295_instance[IN_PIPE_8], // DES2
		&max9295_instance[IN_PIPE_9], // DES2
		&max9295_instance[IN_PIPE_10], // DES2
		&max9295_instance[IN_PIPE_11], // DES2
		&max9295_instance[IN_PIPE_12]  // Unused
	},
	.sensor_array = { NULL, NULL, NULL, NULL, NULL, NULL,
			  NULL, NULL, NULL, NULL, NULL, NULL },
	.accessiic_array = { NULL, NULL, NULL, NULL, NULL, NULL,
			     NULL, NULL, NULL, NULL, NULL, NULL },
};

static volatile u32 fmcinitDone;
static u32 core_des_status;

/****************************************************************************
 *  Core -> deserializer chip -> I2C bus mapping.
 *
 *  Each core owns one MAX96724 chip (one desInterface entry). The
 *  des_indices values are positions into ndes_arr[].
 ****************************************************************************/
const core_des_mapping_t max96724_core_des_map[] = {
	/* Core 6 -> chip 1 (ndes_arr[0]) on AXI I2C bus 1 */
	{6, 1, {0, 0, 0}, {1, 0, 0}},

	/* Core 7 -> chip 2 (ndes_arr[1]) on AXI I2C bus 2 */
	{7, 1, {1, 0, 0}, {2, 0, 0}},

	/* Core 8 -> chip 3 (ndes_arr[2]) on AXI I2C bus 3 */
	{8, 1, {2, 0, 0}, {3, 0, 0}},
};

#define CORE_DES_MAP_SIZE (sizeof(max96724_core_des_map) / sizeof(core_des_mapping_t))

/* Exported map size for ms_fw.c GetI2cBusIdForDes (FMC_ID_1 = MAX96724 quad). */
u8 MAX96724_CORE_DES_MAP_SIZE =
    (sizeof(max96724_core_des_map) / sizeof(core_des_mapping_t));

/****************************************************************************
 *  nQuad_GetI2cBusIdForDes - Lookup AXI I2C bus for a given ndes_arr index.
 *  Non-static: also referenced by ms_fw.c and the sensor drivers
 *  (imx623/imx728/ox03f10/ox05b1s/ox08b40) via extern declaration.
 ****************************************************************************/
static u8 nQuad_GetI2cBusIdForDes(u8 desId)
{
	u8 i, j;

	for (i = 0; i < CORE_DES_MAP_SIZE; i++) {
		for (j = 0; j < max96724_core_des_map[i].num_deserializers; j++) {
			if (max96724_core_des_map[i].des_indices[j] == desId) {
				u8 busId = max96724_core_des_map[i].i2cBusIds[j];
				xil_printf("%s: desId=%d -> i2cBusId=%d (core=%d)\r\n",
					__func__, desId, busId, max96724_core_des_map[i].core_id);
				return busId;
			}
		}
	}
	xil_printf("ERROR: %s - desId %d not in core_des_map\r\n", __func__, desId);
	return 0xFF;
}

/****************************************************************************
 *  ndes_arr[]
 *
 *  One entry per physical MAX96724 chip. Each entry carries all 4 GMSL
 *  links (A/B/C/D) directly via the framework desInterface fields.
 ****************************************************************************/
desInterface ndes_arr[MAX96724_NUM_ENTRIES] = {
	/* --- Chip 1 : Links A / B / C / D --- */
	{
		M724_DS1_DEFAULT_ADDRESS,
		M724_DS1_ALIAS_ADDRESS,
		0,                              /* chip index */
		in_deinit,
		NO_LINK,
		{/* Link A */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_0_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_0_ALIAS_ADDR},
		{/* Link B */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_1_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_1_ALIAS_ADDR},
		{/* Link C */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_2_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_2_ALIAS_ADDR},
		{/* Link D */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_3_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_3_ALIAS_ADDR}
	},
	/* --- Chip 2 : Links A / B / C / D --- */
	{
		M724_DS2_DEFAULT_ADDRESS,
		M724_DS2_ALIAS_ADDRESS,
		1,                              /* chip index */
		in_deinit,
		NO_LINK,
		{/* Link A */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_4_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_4_ALIAS_ADDR},
		{/* Link B */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_5_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_5_ALIAS_ADDR},
		{/* Link C */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_6_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_6_ALIAS_ADDR},
		{/* Link D */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_7_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_7_ALIAS_ADDR}
	},
	/* --- Chip 3 : Links A / B / C / D --- */
	{
		M724_DS3_DEFAULT_ADDRESS,
		M724_DS3_ALIAS_ADDRESS,
		2,                              /* chip index */
		in_deinit,
		NO_LINK,
		{/* Link A */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_8_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_8_ALIAS_ADDR},
		{/* Link B */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_9_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_9_ALIAS_ADDR},
		{/* Link C */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_10_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_10_ALIAS_ADDR},
		{/* Link D */ SERIALIZER_DEFAULT_ADDR, SERIALIZER_11_ALIAS_ADDR,
			SENSOR_IMX623_ADDRESS, SENSOR_11_ALIAS_ADDR}
	},
};

/****************************************************************************/
static void n_on_board_topology(void)
{
	u8 i;
	xil_printf(" MAX96724 On-Board Sensor Status==>\r\n");
	xil_printf(" Chip  A   B   C   D\r\n");
	for (i = 0; i < MAX96724_NUM_ENTRIES; i++) {
		u8 lt = ndes_arr[i].link_type;
		u8 a  = (lt != NO_LINK) && (lt & LINK_A);
		u8 b  = (lt != NO_LINK) && (lt & LINK_B);
		u8 c  = (lt != NO_LINK) && (lt & LINK_C);
		u8 d  = (lt != NO_LINK) && (lt & LINK_D);
		xil_printf(" %d    %s   %s   %s   %s\r\n", i + 1,
			a ? "Y" : "N", b ? "Y" : "N",
			c ? "Y" : "N", d ? "Y" : "N");
	}
}

/****************************************************************************/
static int nQuad_IsiCreateFmcIss(void)
{
	nQuad_Context_t *pCtx = 0;
	if (!pCtx)
		return -2;
	memset(pCtx, 0, sizeof(nQuad_Context_t));
	return 0;
}

static void nWrFmcDataTo_sharedmem(void)
{
	byte_memcpy(SHARED_DES_ARRAY_SRUCT, ndes_arr, sizeof(ndes_arr));
	xil_printf("Des-arr structure (MAX96724) loaded by Core-%x for all other cores\r\n",
		cpu_id);
}

static void nRdFmcReadFrom_sharedmem(void)
{
	byte_memcpy(ndes_arr, SHARED_DES_ARRAY_SRUCT, sizeof(ndes_arr));
	xil_printf("Des-arr structure (MAX96724) loaded for Core-%x\r\n", cpu_id);
}

/****************************************************************************
 *  nQuad_Fmc_Setup - board-level FMC bring-up
 *
 ****************************************************************************/
static int nQuad_Fmc_Setup(int ndes_arr_id)
{
	int Status = XST_SUCCESS;
	u16 expander_addr = 0;
	u32 register_addr = 0;
	u16 bytes_read = 1;
	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};
	u32 core_id;
	HalI2cConfig_t i2cConfig;

	xil_printf("nQuad_Fmc_Setup\r\n");

	(void)ndes_arr_id;
	core_id = cpu_id;

	//get_fmcinit_lock();

	fmcinitDone = (*FMC_INIT_STATUS_REG == 1) ? 1 : 0;
	xil_printf("FMC_INIT_STATUS_REG=%x fmcinitDone=%x cpuid=%x\r\n",
		*FMC_INIT_STATUS_REG, fmcinitDone, core_id);

	if (fmcinitDone == 0) {
		xil_printf("**** MAX96724 FMC Init Started ****\r\n");

		*FMC_INIT_STATUS_REG = 1;
		fmcinitDone = 1;
	} else {
		if (core_des_status == 0) {
			//nRdFmcReadFrom_sharedmem(); //TODOTODOTODO LT
		} else {
			xil_printf("Des-arr already loaded for core-%x\r\n", core_id);
		}
	}

	//xil_printf("MAX96724: I2C address remap SKIPPED "
	//	"(each deserializer is on its own AXI I2C bus)\r\n");

	if (core_des_status == 0) {
		u8 i, j;
		bool_t core_found = BOOL_FALSE;

		for (i = 0; i < CORE_DES_MAP_SIZE; i++) {
			if (max96724_core_des_map[i].core_id != core_id)
				continue;
			core_found = BOOL_TRUE;
			xil_printf("Core %d initializing %d chip(s)\r\n",
				core_id, max96724_core_des_map[i].num_deserializers);

			for (j = 0; j < max96724_core_des_map[i].num_deserializers; j++) {
				u8 des_idx = max96724_core_des_map[i].des_indices[j];
				u8 i2c_bus = max96724_core_des_map[i].i2cBusIds[j];
				u32 ndes_bit_mask = (1U << des_idx);

				memset(&i2cConfig, 0, sizeof(HalI2cConfig_t));
				i2cConfig.i2cBusId   = i2c_bus;
				i2cConfig.HalI2cMode = HAL_AXI_I2C_MODE;
				i2cConfig.hHalI2c    = NULL;

				if (HalI2cInit(&i2cConfig) != RET_SUCCESS) {
					xil_printf("%s: HalI2cInit bus %d FAILED\r\n",
						__func__, i2c_bus);
					return RET_FAILURE;
				}

				if (*DES_REMAPPED_STATUS_REG & ndes_bit_mask) {
					xil_printf("Chip %d already initialized, skip\r\n",
						des_idx + 1);
					continue;
				}
				xil_printf("Chip %d ready (I2C bus %d, no remap needed)\r\n",
					des_idx + 1, i2c_bus);
				/* I2C remap intentionally skipped: each MAX96724 sits on
				 * its own AXI I2C bus, so default addresses do not collide. */
				*DES_REMAPPED_STATUS_REG |= ndes_bit_mask;
			}
			break;
		}

		if (!core_found)
			xil_printf("Warning: no mapping for core %d\r\n", core_id);

		core_des_status = 1;

		//n_on_board_topology();
		//nWrFmcDataTo_sharedmem();
	}

	xil_printf("MAX96724 FMC Init Done\r\n");
	//release_fmcinit_lock();
	return Status;
}

/****************************************************************************
 *  nQuad_reg6_from_links - build the REG6 (0x06) value from a detected-link
 *  bitmask.
 ****************************************************************************/
#define MAX96724_GMSL2_MODE_ALL		(0xF0)   /* GMSL2_A..D = 1 */

static u8 nQuad_reg6_from_links(u8 link_type)
{
	u8 link_en = 0;

	if (link_type != NO_LINK) {
		if (link_type & LINK_A) link_en |= (1 << 0);
		if (link_type & LINK_B) link_en |= (1 << 1);
		if (link_type & LINK_C) link_en |= (1 << 2);
		if (link_type & LINK_D) link_en |= (1 << 3);
	}

	return MAX96724_GMSL2_MODE_ALL | link_en;
}

/****************************************************************************
 *  nQuad_Probe_links - probe all 4 GMSL links of one chip and populate
 *  desIface->link_type with a presence bitmask (LINK_A|LINK_B|LINK_C|LINK_D).
 *  If no link locks, link_type is left as NO_LINK.
 ****************************************************************************/
static void nQuad_Probe_links(u8 Deser_addr, desInterface *desIface)
{
	int Status;

    //xil_printf("nQuad_Probe_links \r\n");

	u8 read_data[2] = {0};
	const osTickType xDelay2 = 50;
	u8 i2cBusId = nQuad_GetI2cBusIdForDes(desIface->Port_DES_index);
	u8 mask = 0;
	u8 itr_num;

	xil_printf("nQuad_Probe_links: bus=%d des=0x%x\r\n", i2cBusId, Deser_addr);

	/* Per-link GMSL2 lock status registers (MAX96724 datasheet).
	 * All four GMSL2 links are already enabled by the init table
	 * (reg 0x06 = 0xFF), so we simply read each link's LOCKED bit:
	 *   Link A -> CTRL3  (0x1A) bit 3 LOCKED_A
	 *   Link B -> CTRL12 (0x0A) bit 3 LOCKED_B
	 *   Link C -> CTRL13 (0x0B) bit 3 LOCKED_C
	 *   Link D -> CTRL14 (0x0C) bit 3 LOCKED_D
	 */
	static const struct {
		u16 lock_reg;
		u8  fw_link_flag;
	} link_tbl[MAX96724_LINKS_PER_CHIP] = {
		{M724_CTRL3_REG,  LINK_A},
		{CTRL12_REG, LINK_B},
		{CTRL13_REG, LINK_C},
		{CTRL14_REG, LINK_D},
	};

	for (u8 li = 0; li < MAX96724_LINKS_PER_CHIP; li++) {
		for (itr_num = 1; itr_num <= NUM_INTERATION; itr_num++) {
			Status = HalReadI2CReg(i2cBusId, Deser_addr,
				link_tbl[li].lock_reg, 0x2, read_data, 1);
			if (Status != XST_SUCCESS)
				DCT_ASSERT(0);
			if (read_data[0] & (1 << LINK_STATUS_BIT)) {
				mask |= link_tbl[li].fw_link_flag;
				break;
			}
			//osSleep(xDelay2);
		}
	}

	desIface->link_type = mask ? mask : NO_LINK;
}

/****************************************************************************
 *  nQuad_program_one_serializer - aliasing of one serializer + sensor
 ****************************************************************************/
static void nQuad_program_one_serializer(u8 i2cBusId, dslink *link_ptr)
{
	int Status;
	u8 ser_addr, read_data[2] = {0}, wr_data[2] = {0};
	u16 reg_addr;

	xil_printf("nQuad_program_one_serializer - should not be used \r\n");

	/* Update serializer's own address */
	ser_addr   = (link_ptr->serializer_default_addr) >> 1;
	reg_addr   = 0x00;
	wr_data[0] = link_ptr->serializer_alias_addr;
	(void)HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
	Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	/* Speak to it at the new alias */
	ser_addr = (link_ptr->serializer_alias_addr) >> 1;

	/* Program sensor virtual address */
	reg_addr   = I2C_2;
	wr_data[0] = link_ptr->sensor_alias_addr;
	Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	reg_addr   = I2C_3;
	wr_data[0] = link_ptr->sensor_default_addr;
	Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
}

/****************************************************************************
 *  nget_ndes_array - return per-chip init table.
 ****************************************************************************/
static u32 nget_ndes_array(desInterface *des, RegI2CT **arr)
{
	u8 chip = des->Port_DES_index;
	u32 len;

	switch (chip) {
	case 0:
		*arr = m724_des1_init;
		len  = sizeof(m724_des1_init) / sizeof(RegI2CT);
		break;
	case 1:
		*arr = m724_des2_init;
		len  = sizeof(m724_des2_init) / sizeof(RegI2CT);
		break;
	case 2:
		*arr = m724_des3_init;
		len  = sizeof(m724_des3_init) / sizeof(RegI2CT);
		break;
	default:
		DCT_ASSERT(0);
		*arr = m724_des1_init;
		len  = sizeof(m724_des1_init) / sizeof(RegI2CT);
		break;
	}
	return len;
}

/****************************************************************************
 *  nQuad_init_serializer - run the serializer init table for each active
 *  GMSL link of one MAX96724 chip via the framework serializer_driver
 *  init_serializer callback (max9295_init), instead of open-coding it.
 *  Serializers must already be reachable at their alias address (done by
 *  the per-link setup in nQuad_Deser_setup) before this runs.
 ****************************************************************************/
static void nQuad_init_serializer(desInterface *des)
{
	u8 i2cBusId = nQuad_GetI2cBusIdForDes(des->Port_DES_index);
	u8 base = des->Port_DES_index * MAX96724_LINKS_PER_CHIP;
	struct {
		u8	flag;
		dslink	*link;
	} lk[MAX96724_LINKS_PER_CHIP] = {
		{ LINK_A, &des->link_a },
		{ LINK_B, &des->link_b },
		{ LINK_C, &des->link_c },
		{ LINK_D, &des->link_d },
	};

	if (des->link_type == NO_LINK)
		return;

	for (u8 li = 0; li < MAX96724_LINKS_PER_CHIP; li++) {
		struct serializer_driver *ser = &max9295_instance[base + li];

		if (!(des->link_type & lk[li].flag))
			continue;

		ser->bus_num    = i2cBusId;
		ser->alias_addr = lk[li].link->serializer_alias_addr;
		if (ser->init_serializer)
			ser->init_serializer(ser);
	}
}

/****************************************************************************
 *  nQuad_Deser_setup - run register init table against the chip.
 ****************************************************************************/
static int nQuad_Deser_setup(desInterface *des)
{
	//xil_printf("nQuad_Deser_setup setup start\r\n");
    
    int Status = XST_SUCCESS;
	u8 wr_data[4] = {0};
	u8 rd_data[4] = {0};
	u8 i2cBusId = nQuad_GetI2cBusIdForDes(des->Port_DES_index);
	u32 len, j;
	RegI2CT *init_tbl;

	if (des->des_state != in_deinit) {
		xil_printf("Deserializer already running (state %d)\r\n", des->des_state);
		return Status;
	}

    des->des_alias_addr = (0x4E); // Hardcoded
	u16 Deser_addr = (des->des_alias_addr) >> 1;
	len = nget_ndes_array(des, &init_tbl);

    xil_printf("nQuad_Deser_setup: I2C writes\r\n");


	/*
	// configure for 3G - Quanta
	xil_printf("Setting CFG1 to 3G...\r\n");

	u16 cfg1_pot_addr = (0x2B) ;
	wr_data[0] = 0x32;
	Status = HalWriteI2CReg(i2cBusId, cfg1_pot_addr, 0x11, 0x1, wr_data, 1);
	if (Status != XST_SUCCESS) DCT_ASSERT(0);


	xil_printf("Reseting deserializer...\r\n");
	wr_data[0] = 0x40;
	Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0013, 0x2, wr_data, 1);
	if (Status != XST_SUCCESS) DCT_ASSERT(0);

	osSleep(200);

	// Quanta setup end
	*/


	for (j = 0; j < len; j++) {
		if ((init_tbl + j)->addr == MAX929X_TABLE_END)
			break;
		if ((init_tbl + j)->addr == MAX929X_TABLE_WAIT) {
			osSleep((init_tbl + j)->val);
			continue;
		}
		wr_data[0] = (init_tbl + j)->val;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr,
			(init_tbl + j)->addr, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		Status = HalReadI2CReg(i2cBusId, Deser_addr,
			(init_tbl + j)->addr, 0x2, rd_data, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
#if 0		
		xil_printf("[%s] 0x%x -> 0x%x\r\n", __func__,
			(init_tbl + j)->addr, rd_data[0]);
#endif
		if (j == 0)
			osSleep(50); // Why was this so long was 5000
	}

	des->des_state = in_running;

	/* TODOTODO use probe response to setup correct links */
	nQuad_Probe_links(Deser_addr, des);
	n_on_board_topology();

	//TODO REMAP Serializer and Camera addresses here
	//xil_printf("nQuad_Deser_setup: Remap serialzers\r\n");

	max9295_instance[0].alias_addr = des->link_a.serializer_alias_addr; // TODOTODOTODO
	max9295_instance[1].alias_addr = des->link_b.serializer_alias_addr;
	max9295_instance[2].alias_addr = des->link_c.serializer_alias_addr;
	max9295_instance[3].alias_addr = des->link_d.serializer_alias_addr;

	max9295_instance[4].alias_addr = des->link_a.serializer_alias_addr; // TODOTODOTODO
	max9295_instance[5].alias_addr = des->link_b.serializer_alias_addr;
	max9295_instance[6].alias_addr = des->link_c.serializer_alias_addr;
	max9295_instance[7].alias_addr = des->link_d.serializer_alias_addr;

	max9295_instance[8].alias_addr = des->link_a.serializer_alias_addr; // TODOTODOTODO
	max9295_instance[9].alias_addr = des->link_b.serializer_alias_addr;
	max9295_instance[10].alias_addr = des->link_c.serializer_alias_addr;
	max9295_instance[11].alias_addr = des->link_d.serializer_alias_addr;


	u16 Ser_default_addr = (SERIALIZER_DEFAULT_ADDR) >> 1;

	//TODO REMOVE
	/*if (des->Port_DES_index == 2)
	{
		Ser_default_addr = (SERIALIZER_DEFAULT_ADDR) >> 1;
	}
	else
	{
		Ser_default_addr = (0x80) >> 1; //Quanta
		/*xil_printf("Checking CFG1 is 3G...\r\n");

		u16 cfg1_pot_addr = (0x2B) ;
		Status = HalReadI2CReg(i2cBusId, cfg1_pot_addr, 0x21, 0x1, rd_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);

		//xil_printf("rd_data[0] = 0x%x 0x%x 0x%x 0x%x \r\n", rd_data[0],rd_data[1],rd_data[2],rd_data[3]);
		
		if ( rd_data[0] > 0 ) {
			xil_printf("Deserializer not configured for 3G, configuring, reboot required\r\n");
			wr_data[0] = 0x32;
			Status = HalWriteI2CReg(i2cBusId, cfg1_pot_addr, 0x21, 0x1, wr_data, 1);
			if (Status != XST_SUCCESS) DCT_ASSERT(0);
		}
	}*/
	/*u16 cfg1_pot_addr = (0x2B) ;
	wr_data[0] = 0x00;
	Status = HalWriteI2CReg(i2cBusId, cfg1_pot_addr, 0x21, 0x1, wr_data, 1);
	if (Status != XST_SUCCESS) DCT_ASSERT(0);*/


	if( des->link_type & LINK_A )
	{
		//Serializer 1 setup
		wr_data[0] = 0xFD;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0003, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0xF1;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0006, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0xE0;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0018, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);

		osSleep(50);

		// Change I2C address of Link A serializer
		wr_data[0] = max9295_instance[0].alias_addr; //0xC2;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0000, 0x2, wr_data, 1);  // 0x62 = C4 Leopard
		if (Status != XST_SUCCESS) DCT_ASSERT(0);

		xil_printf("nQuad_Deser_setup: LinkA configured\r\n");
	}


	if( des->link_type & LINK_B )
	{
		//Serializer 2 setup
		wr_data[0] = 0xF7;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0003, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0xF2;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0006, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0xD0;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0018, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);

		osSleep(50);

		// Change Sensor Address remap first
		wr_data[0] = SENSOR_1_ALIAS_ADDR;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0042, 0x2, wr_data, 1); // SRC_A: New address
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = SENSOR_IMX623_ADDRESS;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0043, 0x2, wr_data, 1); // DST_A: Default Address
		if (Status != XST_SUCCESS) DCT_ASSERT(0);	


		// Change I2C address of Link B serializer
		wr_data[0] = max9295_instance[1].alias_addr;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0000, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);

		xil_printf("nQuad_Deser_setup: LinkB configured\r\n");
	}


	if( des->link_type & LINK_C )
	{	
		//Serializer 3 setup
		wr_data[0] = 0xDF;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0003, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0xF4;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0006, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0xB0;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0018, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		osSleep(50);


		// Change Sensor Address remap first
		wr_data[0] = SENSOR_2_ALIAS_ADDR;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0042, 0x2, wr_data, 1); // New address
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = SENSOR_IMX623_ADDRESS;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0043, 0x2, wr_data, 1); // Default Address
		if (Status != XST_SUCCESS) DCT_ASSERT(0);	

		// Change I2C address of Link A serializer
		wr_data[0] = max9295_instance[2].alias_addr;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0000, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		xil_printf("nQuad_Deser_setup: LinkC configured\r\n");
		
	}

	if( des->link_type & LINK_D )
	{
		//Serializer 4 setup
		wr_data[0] = 0x7F;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0003, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0xF8;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0006, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = 0x70;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0018, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);

		osSleep(100);

		// Change Sensor Address remap first
		wr_data[0] = SENSOR_3_ALIAS_ADDR;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0042, 0x2, wr_data, 1); // New address
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
		wr_data[0] = SENSOR_IMX623_ADDRESS;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0043, 0x2, wr_data, 1); // Default Address
		if (Status != XST_SUCCESS) DCT_ASSERT(0);	

		// Change I2C address of Link A serializer
		wr_data[0] = max9295_instance[3].alias_addr;
		Status = HalWriteI2CReg(i2cBusId, Ser_default_addr, 0x0000, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);

		xil_printf("nQuad_Deser_setup: LinkD configured\r\n");
	}


	// Enable links
	if( des->link_type & LINK_QUAD )
	{
		wr_data[0] = 0xFF;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0006, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);
	}
	else
	{	
		//Enable for the links connected
		wr_data[0] = nQuad_reg6_from_links(des->link_type);
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0006, 0x2, wr_data, 1);
		if (Status != XST_SUCCESS) DCT_ASSERT(0);		
	}
	


	// One shot reset links
	wr_data[0] = 0x00;
	Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0018, 0x2, wr_data, 1);
	if (Status != XST_SUCCESS) DCT_ASSERT(0);



	// Enable all I2C again
	wr_data[0] = 0x55;
	Status = HalWriteI2CReg(i2cBusId, Deser_addr, 0x0003, 0x2, wr_data, 1);
	if (Status != XST_SUCCESS) DCT_ASSERT(0);

	osSleep(10); // Why was this so long was 5000
	//xil_printf("nQuad_Deser_setup: exit\r\n");

	/* Run each active link's serializer init table (max9295_init). */
	nQuad_init_serializer(des);

	//nWrFmcDataTo_sharedmem();

	return Status;
}

/****************************************************************************
 *  nQuad_Deser_Enable / nQuad_Deser_Disable - drive carrier I/O expander
 *
 ****************************************************************************/
static int nQuad_Deser_Enable(u8 pos)
{
	int Status;

	// Nothing to do here 

	xil_printf("Enabled deserializer %d\r\n", pos + 1);
	return 0;
}

// Currently don't need this -- LT
static int nQuad_Deser_Disable(u8 pos)
{
	int Status;

	return 0;
}
