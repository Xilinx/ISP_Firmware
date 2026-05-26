/**************************************************************************
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
 *************************************************************************/
#include <fmc/max96716.h>

CREATE_TRACER(MAX_96716_INFO, "MAX_96716_INFO: ", INFO, 1);
CREATE_TRACER(MAX_96716_WARNING, "MAX_96716_WARNING: ", WARNING, 1);
CREATE_TRACER(MAX_96716_ERROR, "MAX_96716_ERROR: ", ERROR, 1);
CREATE_TRACER(MAX_96716_ALWAYS, "MAX_96716_ALWAYS: ", ALWAYS, 1);

static u32 fmcinitDone;
static u32 core_des_status;

IsiFmc_t g_fmc_max96716 = {
	.FmcName                       = "max96716_xylon_fmc",
	.pIsiIsiFmcSetup               =  max96716_Xylon_Fmc_Setup,
	.pIsiDeserSetup                =  max96716_Xylon_Deser_setup,
	.pIsiDeserEnable               =  max96716_Xylon_Deser_Enable,
	.pIsiDeserDisable              =  max96716_Xylon_Deser_Disable,
	.serializer_array              =  {
		&max9295_instance[IN_PIPE_0],
		&max9295_instance[IN_PIPE_1],
		&max9295_instance[IN_PIPE_2],
		&max9295_instance[IN_PIPE_3],
		&max9295_instance[IN_PIPE_4],
		&max9295_instance[IN_PIPE_5],
		&max9295_instance[IN_PIPE_6],
		&max9295_instance[IN_PIPE_7],
		&max9295_instance[IN_PIPE_8],
		&max9295_instance[IN_PIPE_9],
		&max9295_instance[IN_PIPE_10],
		&max9295_instance[IN_PIPE_11],
		&max9295_instance[IN_PIPE_12]
	},
	.sensor_array = {
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
	.accessiic_array = {
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
	},
};

desInterface max96716_des_arr[DS_MAX] = {
	{
		MAX96716_DS1_DEFAULT_ADDRESS,
		MAX96716_DS1_ALIAS_ADDRESS,
		PDB_DES_DES1,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_0_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_0_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_1_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_1_ALIAS_ADDR}
	},
	{
		MAX96716_DS2_DEFAULT_ADDRESS,
		MAX96716_DS2_ALIAS_ADDRESS,
		PDB_DES_DES2,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_2_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_2_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_3_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_3_ALIAS_ADDR}
	},
	{
		MAX96716_DS3_DEFAULT_ADDRESS,
		MAX96716_DS3_ALIAS_ADDRESS,
		PDB_DES_DES3,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_4_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_4_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_5_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_5_ALIAS_ADDR}
	},
	{
		MAX96716_DS4_DEFAULT_ADDRESS,
		MAX96716_DS4_ALIAS_ADDRESS,
		PDB_DES_DES4,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_6_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_6_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_7_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_7_ALIAS_ADDR}
	},
	{
		MAX96716_DS5_DEFAULT_ADDRESS,
		MAX96716_DS5_ALIAS_ADDRESS,
		PDB_DES_DES5,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_8_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_8_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_9_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_9_ALIAS_ADDR}
	},
	{
		MAX96716_DS6_DEFAULT_ADDRESS,
		MAX96716_DS6_ALIAS_ADDRESS,
		PDB_DES_DES6,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_10_ALIAS_ADDR,
			SENSOR_OX5B_ADDRESS,
			SENSOR_10_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_11_ALIAS_ADDR,
			SENSOR_OX5B_ADDRESS,
			SENSOR_11_ALIAS_ADDR}
	},
	{
		MAX96716_DS2_DEFAULT_ADDRESS,
		MAX96716_DS2_ALIAS_ADDRESS,
		PDB_DES_DES7,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_2_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_2_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR,
			SERIALIZER_3_ALIAS_ADDR,
			SENSOR_DEFAULT_ADDRSS,
			SENSOR_3_ALIAS_ADDR}
	},
};

/**
 * Core-to-Deserializer-to-I2C Bus Mapping Table
 *
 * This table defines the complete hardware topology:
 *   - Which CPU core manages which deserializers
 *   - Which I2C bus each deserializer uses
 *
 * Format: {core_id, num_des, {des_indices}, {i2cBusIds}}
 *
 * I2C Bus Mapping:
 *   - Bus 0: PS I2C (Processing System)
 *   - Bus 1: AXI I2C Controller 1
 *   - Bus 2: AXI I2C Controller 2
 *   - Bus 3: AXI I2C Controller 3
 *
 * Adjust the i2cBusIds array based on your hardware configuration.
 */
const core_des_mapping_t max96716_core_des_map[] = {
	/*
	 * Core 6: Manages 3 deserializers
	 * (DS_ONE, DS_TWO, DS_THREE) on I2C bus 1
	 */
	{6, 2, {DS_ONE, DS_TWO}, {1, 1}}, /* 4+1*/

	/* Core 7: Manages 1 deserializer (DS_FOUR) on I2C bus 2 */
	{7, 2, {DS_THREE, DS_FOUR}, {2, 2}}, /* 4+1*/

	/*
	 * Core 8: Manages 2 deserializers
	 * (DS_FIVE, DS_SIX) both on I2C bus 3
	 */
	{8, 2, {DS_FIVE, DS_SIX}, {3, 3}}, /* 1+1 */
};

u8 MAX96716_CORE_DES_MAP_SIZE =
	(sizeof(max96716_core_des_map)
	/ sizeof(core_des_mapping_t));

/**************************************************************************
 *          max96716_on_board_topology
 *
 * @brief   Display on-board sensor status and topology information
 *
 * @return  None
 *
 *************************************************************************/
void max96716_on_board_topology(void)
{
	u8 des_idx = 0;

	TRACE(MAX_96716_ALWAYS, "\n\r	New fmc - Max96716\n");
	TRACE(MAX_96716_ALWAYS, "\n\r	On-Board Sensor Status==>");
	TRACE(MAX_96716_ALWAYS,
		"\n\r	De-serializer	Sensor on link-a	Sensor on link-b\n\r ");

	for (des_idx = 0; des_idx < DS_MAX; des_idx++) {
		if (max96716_des_arr[des_idx].link_type == LINK_REVERSE_SPLITTER) {
			TRACE(MAX_96716_INFO, "\n\r %d				Y		Y\n\r ",
					des_idx + 1);
		} else if (max96716_des_arr[des_idx].link_type == NO_LINK) {
			TRACE(MAX_96716_INFO, "\n\r %d				N		N\n\r ",
					des_idx + 1);
		} else if (max96716_des_arr[des_idx].link_type == LINK_A) {
			TRACE(MAX_96716_INFO, "\n\r %d				Y		N\n\r ",
					des_idx + 1);
		} else {
			TRACE(MAX_96716_INFO, "\n\r %d				N		Y\n\r ",
					des_idx + 1);
		}
	}
}

/**************************************************************************
 *          max96716_WrFmcDataTo_sharedmem
 *
 * @brief   Write FMC data to shared memory
 *
 * @return  None
 *
 *************************************************************************/
void max96716_WrFmcDataTo_sharedmem(void)
{
	byte_memcpy(SHARED_DES_ARRAY_SRUCT,
		max96716_des_arr,
		sizeof(max96716_des_arr));
	TRACE(MAX_96716_ALWAYS,
		"Des-arr loaded by Core-%x for all cores\r\n",
		cpu_id);
}

/**************************************************************************
 *          max96716_RdFmcReadFrom_sharedmem
 *
 * @brief   Read FMC data from shared memory
 *
 * @return  None
 *
 *************************************************************************/
void max96716_RdFmcReadFrom_sharedmem(void)
{
	byte_memcpy(max96716_des_arr,
		SHARED_DES_ARRAY_SRUCT,
		sizeof(max96716_des_arr));
	TRACE(MAX_96716_ALWAYS,
		"Des-arr loaded for Core-%x \r\n",
		cpu_id);
}

/**************************************************************************
 *          max96716_Xylon_Fmc_Setup
 *
 * @brief   Setup FMC (FPGA Mezzanine Card) configuration for nXylon interface
 *
 * @param   ndes_arr_id    Deserializer array identifier
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Setup completed successfully
 * @retval  RET_FAILURE   Setup failed
 *
 *************************************************************************/
RESULT max96716_Xylon_Fmc_Setup(int ndes_arr_id)
{
	RESULT Status = RET_SUCCESS;

	(void)ndes_arr_id; /* Unused parameter */

	u16 expander_addr = 0;
	u32 register_addr = 0;
	u16 bytes_read = 1;

	u8 read_data[2] = {0};
	u8 wr_data[2] =	{0};
	HalI2cConfig_t i2cConfig;

	get_fmcinit_lock();

	if (*FMC_INIT_STATUS_REG == 1)
		fmcinitDone = 1;
	else
		fmcinitDone = 0;

	TRACE(MAX_96716_INFO, "FMC_INIT_STATUS_REG=%x fmcinitDone=%x cpuid=%x\n\r",
			*FMC_INIT_STATUS_REG, fmcinitDone, cpu_id);

	if (fmcinitDone == 0) {
		TRACE(MAX_96716_ALWAYS, "**********************************\r\n");
		TRACE(MAX_96716_ALWAYS, "FMC Init Started...\r\n");

		u8 potent_addr	= MAX96716_POTENTIOMETER_ADDR;

		register_addr	= MAX96716_POTENTIOMETER_REG;
		wr_data[0]	= MAX96716_POTENTIOMETER_WR_VAL;

		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, potent_addr,
			register_addr, MAX96716_FMC_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failedaddr=0x%x (err=%d)\n",
				 __func__, __LINE__, potent_addr, Status);
			return Status;
		}

		osSleep(MAX96716_PROBE_DELAY_MS);

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, potent_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failedaddr=0x%x (err=%d)\n",
				__func__, __LINE__, potent_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO,
			"potentiometer 2-4: Data Read is:[0]:0x%x\r\n", read_data[0]);

		u16 ldac_addr	= MAX96716_LDAC_ADDR;

		register_addr	= MAX96716_LDAC_DAC_REG;

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO,
			ldac_addr, register_addr,
			MAX96716_FMC_REG_ADDR_SIZE, read_data, 2);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ldac_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO,
			"ldac before: Data Read is:[0]:0x%x\r\n",
			read_data[0]);
		TRACE(MAX_96716_INFO,
			"ldac : Data Read is:[1]:0x%x\r\n",
			read_data[1]);

		wr_data[0]	= MAX96716_LDAC_WR_VAL;
		wr_data[1]	= MAX96716_LDAC_WR_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ldac_addr, register_addr,
				MAX96716_FMC_REG_ADDR_SIZE, wr_data, 2);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR,
				"%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__,
				ldac_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ldac_addr, register_addr,
				MAX96716_FMC_REG_ADDR_SIZE, read_data, 2);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR,
				"%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__,
				ldac_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO,
			"ldac : after Data Read is:[0]:0x%x\r\n",
			read_data[0]);
		TRACE(MAX_96716_INFO,
			"ldac : Data Read is:[1]:0x%x\r\n",
			read_data[1]);

	ldac_addr	= MAX96716_LDAC_ADDR;
	register_addr	= MAX96716_LDAC_MODE_REG;

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO,
			ldac_addr, register_addr,
			MAX96716_FMC_REG_ADDR_SIZE, read_data, 2);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ldac_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "before Data Read is:[0]:0x%x\r\n", read_data[0]);
		TRACE(MAX_96716_INFO, "Data Read is:[1]:0x%x\r\n", read_data[1]);

		wr_data[0]	= MAX96716_LDAC_WR_VAL;
		wr_data[1]	= MAX96716_LDAC_WR_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO,
			ldac_addr, register_addr,
			MAX96716_FMC_REG_ADDR_SIZE, wr_data, 2);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR,
				"%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__,
				ldac_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO,
			ldac_addr, register_addr,
			MAX96716_FMC_REG_ADDR_SIZE, read_data, 2);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ldac_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "after Data Read is:[0]:0x%x\r\n", read_data[0]);
		TRACE(MAX_96716_INFO, "Data Read is:[1]:0x%x\r\n", read_data[1]);

		expander_addr	= MAX96716_EXPANDER_ADDR;
		register_addr	= MAX96716_EXPANDER_OUT_PORT0_REG;

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x00: before :0x%x\n", read_data[0]);

		register_addr	= MAX96716_EXPANDER_OUT_PORT1_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x04: before :0x%x\n", read_data[0]);

		wr_data[0]	= MAX96716_EXPANDER_OUT_INIT_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO,
			expander_addr, register_addr,
			MAX96716_FMC_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR,
				"%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__,
				expander_addr, Status);
			return Status;
		}

		osSleep(MAX96716_REMAP_DELAY_MS);

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x04: after :0x%x\n", read_data[0]);

		register_addr	= MAX96716_EXPANDER_OUT_PORT0_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x00: after :0x%x\n", read_data[0]);

		register_addr	= MAX96716_EXPANDER_CONFIG_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x0c: before :0x%x\n", read_data[0]);

		wr_data[0]	= MAX96716_EXPANDER_CONFIG_ALL_OUT;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO,
			expander_addr, register_addr,
			MAX96716_FMC_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR,
				"%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__,
				expander_addr, Status);
			return Status;
		}

		osSleep(MAX96716_REMAP_DELAY_MS);

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x0c: after :0x%x\n", read_data[0]);

		register_addr	= MAX96716_EXPANDER_OUT_PORT1_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x0c: after :0x%x\n", read_data[0]);

		register_addr	= MAX96716_EXPANDER_OUT_PORT1_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x00: after :0x%x\n", read_data[0]);

		register_addr	= MAX96716_EXPANDER_OUT_PORT1_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "expander 0x04: before :0x%x\n", read_data[0]);

		expander_addr	= MAX96716_EXPANDER_ADDR;
		register_addr	= MAX96716_EXPANDER_OUT_PORT1_REG;

		wr_data[0]	= MAX96716_EXPANDER_DESER_EN_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO,
			expander_addr, register_addr,
			MAX96716_FMC_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR,
				"%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__,
				expander_addr, Status);
			return Status;
		}

		osSleep(MAX96716_REMAP_DELAY_MS);

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "expander 0x04-1: after :0x%x\n", read_data[0]);

		register_addr	= MAX96716_EXPANDER_OUT_PORT0_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
			 register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, expander_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "expander 0x00: before :0x%x\n", read_data[0]);

		Status |= max96716_Xylon_Deser_Disable(PDB_DES_DES1);
		Status |= max96716_Xylon_Deser_Disable(PDB_DES_DES2);
		Status |= max96716_Xylon_Deser_Disable(PDB_DES_DES3);
		Status |= max96716_Xylon_Deser_Disable(PDB_DES_DES4);
		Status |= max96716_Xylon_Deser_Disable(PDB_DES_DES5);
		Status |= max96716_Xylon_Deser_Disable(PDB_DES_DES6);

#if defined(I2C_FAST_MODE)
		TRACE(MAX_96716_ALWAYS, "Reverting  I2C Bus to 400KHz .....\r\n");
		revert_to_400khz();
		TRACE(MAX_96716_ALWAYS, "Running I2C Bus at  400KHz .....\r\n");
#else
		TRACE(MAX_96716_ALWAYS, "Running I2C Bus at  100KHz .....\r\n");
#endif

		*FMC_INIT_STATUS_REG = 1;
		fmcinitDone = 1;
	} else {
		if (core_des_status == 0) {
			max96716_RdFmcReadFrom_sharedmem();
			TRACE(MAX_96716_ALWAYS,
				"Des-arr loaded for Core-%x from Shared mem\r\n",
				cpu_id);
		} else {
			TRACE(MAX_96716_WARNING,
				"Des-arr structure Already loaded for Core-%x \r\n",
				cpu_id);
		}
	}

	TRACE(MAX_96716_ALWAYS, "Remapping I2c Address on FMC started...\r\n");

	if (core_des_status == 0) {
		u8 map_idx, des_iter;
		bool_t core_found = BOOL_FALSE;

		TRACE(MAX_96716_INFO, "Inside %s and Value of array: %d\n",
			__func__, MAX96716_CORE_DES_MAP_SIZE);

		for (map_idx = 0; map_idx < MAX96716_CORE_DES_MAP_SIZE; map_idx++) {
			if (max96716_core_des_map[map_idx].core_id == cpu_id) {
				core_found = BOOL_TRUE;
				TRACE(MAX_96716_INFO,
					"Core %d remapping %d deserializers\r\n",
					cpu_id,
					max96716_core_des_map[map_idx]
					.num_deserializers);

				u8 num_des =
					max96716_core_des_map[map_idx]
					.num_deserializers;
				for (des_iter = 0;
					des_iter < num_des;
					des_iter++) {
					u8 des_idx =
						max96716_core_des_map[map_idx]
						.des_indices[des_iter];
					u8 i2c_bus =
						max96716_core_des_map[map_idx]
						.i2cBusIds[des_iter];

					memset(&i2cConfig, 0, sizeof(HalI2cConfig_t));

					i2cConfig.i2cBusId = i2c_bus;
					i2cConfig.HalI2cMode = HAL_AXI_I2C_MODE;
					i2cConfig.hHalI2c = NULL;

					u8 result = HalI2cInit(&i2cConfig);

					if (result != RET_SUCCESS) {
						TRACE(MAX_96716_ERROR,
							"%s: Failed to init I2C bus %d (err=%d)\n",
							__func__,
							i2cConfig.i2cBusId,
							result);
						return RET_FAILURE;
					}

					TRACE(MAX_96716_INFO,
						"%s: I2C bus %d initialized successfully\n",
						__func__,
						i2cConfig.i2cBusId);

					u32 ndes_bit_mask = (1U << des_idx);

					if (*DES_REMAPPED_STATUS_REG & ndes_bit_mask) {
						TRACE(MAX_96716_WARNING,
							"Des DS_%d already remapped by another core\r\n",
							des_idx + 1);
						TRACE(MAX_96716_WARNING,
							"    [status: 0x%08x, mask: 0x%08x]\r\n",
							*DES_REMAPPED_STATUS_REG,
							ndes_bit_mask);
						continue;
					}

					TRACE(MAX_96716_INFO,
						"Remapping deser DS_%d (I2C bus %d)\r\n",
						des_idx + 1,
						i2cConfig.i2cBusId);
					max96716_Remapping_des_addr(&max96716_des_arr[des_idx]);

					*DES_REMAPPED_STATUS_REG |= ndes_bit_mask;
					TRACE(MAX_96716_INFO, "    [Updated shared status: 0x%08x]\r\n",
						*DES_REMAPPED_STATUS_REG);
				}
				break;
			}
		}

		if (!core_found) {
			TRACE(MAX_96716_INFO,
				"Warning: No deser mapping found for core %d\r\n",
				cpu_id);
		}

		core_des_status = 1;
		max96716_on_board_topology();

		max96716_WrFmcDataTo_sharedmem();
		TRACE(MAX_96716_ALWAYS,
			"Des-arr written to Shared memory for other cores\r\n");
	}

	TRACE(MAX_96716_ALWAYS, "Remapping I2c Address on FMC Done...\r\n");
	TRACE(MAX_96716_ALWAYS, "FMC Init Done\r\n");
	TRACE(MAX_96716_ALWAYS,
		"************************************\r\n");

	release_fmcinit_lock();

	TRACE(MAX_96716_ALWAYS, "FMC init Lock Released \r\n");
	return Status;
}

/**************************************************************************
 *          max96716_enable_link
 *
 * @brief   Enable specified link on the deserializer
 *
 * @param   i2c_bus_id    I2C bus ID to use for communication
 * @param   Deser_addr    Deserializer I2C address
 * @param   link_type     Type of link to enable
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 *************************************************************************/
RESULT max96716_enable_link(u8 i2c_bus_id, u8 Deser_addr, u8 link_type)
{
	RESULT Status = RET_SUCCESS;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};

	u16 reg_addr = CTRL0_REG;

	Status = HalReadI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2,
			read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, Deser_addr, Status);
		return Status;
	}

	read_data[0] = read_data[0] & (~(1 << AUTO_LINK));
	wr_data[0] = read_data[0];
	Status = HalWriteI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2,
			wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, Deser_addr, Status);
		return Status;
	}

	Status = HalReadI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2,
			read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, Deser_addr, Status);
		return Status;
	}

#if defined(READ_I2C_REG)
	TRACE(MAX_96716_INFO, "CTRL0 =%x\n\r", read_data[0]);
#endif

	reg_addr = GMSL1_EN;
	Status = HalReadI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2,
			read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, Deser_addr, Status);
		return Status;
	}

	read_data[0] = read_data[0] & (~(LINK_MASK));
	read_data[0] = read_data[0] | (link_type);
	wr_data[0] = read_data[0];
	Status = HalWriteI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2,
			wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, Deser_addr, Status);
		return Status;
	}

	Status = HalReadI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2,
			read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, Deser_addr, Status);
		return Status;
	}
	TRACE(MAX_96716_INFO, "GMSL1_EN =%x\n\r", read_data[0]);

	if (link_type == LINK_A) {
		TRACE(MAX_96716_INFO, "Reset link-A\n");

		reg_addr = CTRL0_REG;
		Status = HalReadI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "CTRL0 =%x\n\r", read_data[0]);

		read_data[0] = read_data[0] | (1 << RESET_ONE_SHOT);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2,
				wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
	} else {
		TRACE(MAX_96716_INFO, "Reset link-B\n");

		reg_addr = CTRL1_REG;
		Status = HalReadI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "CTRL1 =%x\n\r", read_data[0]);

		read_data[0] = read_data[0] | (1 << RESET_ONE_SHOT);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2c_bus_id, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "CTRL1 =%x\n\r", read_data[0]);
	}

	return Status;
}

/**************************************************************************
 *          max96716_probe_if_links
 *
 * @brief   Probe the serializers/sensors connected on board
 *
 * @param   Deser_addr    Deserializer I2C address
 * @param   desIface      Pointer to deserializer interface structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 *************************************************************************/
RESULT max96716_probe_if_links(u8 Deser_addr, desInterface *desIface)
{
	RESULT Status = RET_SUCCESS;

	u8 read_data[2] = {0};
	const TickType_t xDelay2 = MAX96716_PROBE_DELAY_MS;
	u16 reg_addr;
	u8 i2cBusId = GetI2cBusIdForDes(desIface->Port_DES_index);

	u8 dev_a = FALSE;
	u8 dev_b = FALSE;

	u8 itr_num = 1;

	TRACE(MAX_96716_INFO, "Inside %s and I2C busid: %d\n", __func__, i2cBusId);
	max96716_enable_link(i2cBusId, Deser_addr, LINK_A);

	while (1) {
#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO,
			"\n\rProbing sensor on link-A of Des%d ",
			desIface->Port_DES_index + 1);
#endif
		osSleep(xDelay2);

		reg_addr = CTRL3_REG;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr,
				0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO, "CTRL3_REG =%x\n\r", read_data[0]);
#endif
		if ((read_data[0] & (1 << LOCKED)) == (1 << LOCKED)) {
			dev_a = TRUE;
			break;
		}

		if (itr_num == NUM_INTERATION)
			break;
		itr_num++;
	}

	itr_num = 1;
	max96716_enable_link(i2cBusId, Deser_addr, LINK_B);

	while (1) {
#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO, "\n\rProbing sensor on link-B of Des%d ",
			desIface->Port_DES_index + 1);
#endif
		osSleep(xDelay2);

		reg_addr = CTRL9_REG;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr,
				0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO, "CTRL9_REG =%x\n\r", read_data[0]);
#endif
		if ((read_data[0] & (1 << LOCKED)) == (1 << LOCKED)) {
			dev_b = TRUE;
			break;
		}

		if (itr_num == NUM_INTERATION)
			break;
		itr_num++;
	}

	if (dev_a == TRUE && dev_b == TRUE)
		desIface->link_type = LINK_REVERSE_SPLITTER;
	else if (dev_a == FALSE && dev_b == FALSE)
		desIface->link_type = NO_LINK;
	else if (dev_a == TRUE && dev_b == FALSE)
		desIface->link_type = LINK_A;
	else
		desIface->link_type = LINK_B;

	return Status;
}

/**************************************************************************
 *          max96716_Remapping_des_addr
 *
 * @brief   Remap default I2C device addresses to new virtual addresses
 *          for deserializer, serializer and sensor as defined in isi.h
 *
 * @param   desIface      Pointer to deserializer interface structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 *************************************************************************/
static RESULT max96716_Remapping_des_addr(desInterface *desIface)
{
	RESULT Status = RET_SUCCESS;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};
	const TickType_t xDelay = MAX96716_REMAP_DELAY_MS;

	u8 i2cBusId = GetI2cBusIdForDes(desIface->Port_DES_index);
	u8 Deser_addr;
	u8 ser_addr;
	u16 reg_addr;
	dslink *link_ptr;

	Deser_addr = (desIface->des_actual_addr >> 1);
	TRACE(MAX_96716_INFO,
		"Inside %s and Default Des Addr: 0x%x and Alias: 0x%x\n",
		__func__, desIface->des_actual_addr, Deser_addr);
	TRACE(MAX_96716_INFO, "I2C bus ID passed: %d\n", i2cBusId);

	max96716_Xylon_Deser_Enable(desIface->Port_DES_index);
	TRACE(MAX_96716_INFO, "%s - des:%d\n", __func__, desIface->Port_DES_index);

	{
		Deser_addr = (desIface->des_actual_addr >> 1);
		reg_addr = DEV_ADDR_REG;

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "des_base before =%x\n", read_data[0]);

		wr_data[0] = desIface->des_alias_addr;
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Deser_addr = (desIface->des_alias_addr) >> 1;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "des_base after =%x\n", read_data[0]);
	}

	/**********************/

	max96716_probe_if_links(Deser_addr, desIface);

	if (desIface->link_type == NO_LINK) {
		TRACE(MAX_96716_ERROR, "%s: No Sensor Available on link a/B on Deserializer- %d "
			"so not re-mapping Address of Serializer and Sensor\r\n",
				__func__, desIface->Port_DES_index);
		return RET_FAILURE;
	}

	TRACE(MAX_96716_INFO,
		"Re-mapping Ser & Sensor Addr at Des-[0x%x]\r\n",
		desIface->des_alias_addr);

	if (desIface->link_type == LINK_REVERSE_SPLITTER) {
		TRACE(MAX_96716_INFO,
			"Configure De-serializer in reverse splitter mode \r\n");
		TRACE(MAX_96716_INFO, "Configure Link-A in reverse splitter mode \r\n");

		link_ptr = &(desIface->link_a);
		Deser_addr = (desIface->des_alias_addr)>>1;
		reg_addr = CTRL0_REG;

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "Rev Split before CTRL0: 0x%x\n", read_data[0]);

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		read_data[0] = read_data[0] & (~(1 << AUTO_LINK));
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO, "Rev Split CTRL0 =%x\n\r", read_data[0]);
#endif

		reg_addr = GMSL1_EN;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		read_data[0] = read_data[0] & (~(LINK_MASK));
		read_data[0] = read_data[0] | (LINK_A);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "A: GMSL1_EN =%x\n\r", read_data[0]);

		while (1) {
			TRACE(MAX_96716_INFO, "Poll for Lock...on link-A \r");
			osSleep(xDelay);

			reg_addr = CTRL3_REG;
			Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
					__func__, __LINE__, Deser_addr, Status);
				return Status;
			}

			TRACE(MAX_96716_INFO, "CTRL3_REG =%x \r\n", read_data[0]);

			if ((read_data[0] & (1<<LOCKED)) == (1<<LOCKED))
				break;
		}

		ser_addr = (link_ptr->serializer_default_addr)>>1;
		reg_addr = DEV_ADDR_REG;
		wr_data[0] = link_ptr->serializer_alias_addr;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_ALWAYS,
			"Serializer on link-A of Deserializer- %d"
			" is configured to Virtual Address-[0x%x]\r\n",
			desIface->Port_DES_index,
				link_ptr->serializer_alias_addr);

		ser_addr = (link_ptr->serializer_alias_addr)>>1;
		reg_addr = DEV_ADDR_REG;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO,
			"Re-verifying ser new addr :%x\r\n", read_data[0]);
#endif
		reg_addr = MAX96716_SER_GPIO_REG_A;
		wr_data[0] = MAX96716_SER_GPIO_DOUBLE_MODE_VAL;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		reg_addr = MAX96716_SER_GPIO_REG_B;
		wr_data[0] = MAX96716_SER_GPIO_DOUBLE_MODE_VAL;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		reg_addr = MAX96716_SER_GPIO_REG_C;
		wr_data[0] = MAX96716_SER_GPIO_DOUBLE_MODE_VAL;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		reg_addr = I2C_2;
		wr_data[0] = link_ptr->sensor_alias_addr;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		reg_addr = I2C_3;
		wr_data[0] = link_ptr->sensor_default_addr;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		reg_addr = I2C_2;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "I2C_3 :%x\r\n", read_data[0]);
#endif
		TRACE(MAX_96716_INFO,
			"Sensor on Ser-[0x%x] configured to sensor VA-[0x%x]\r\n",
			link_ptr->serializer_alias_addr,
			link_ptr->sensor_alias_addr);

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO,
			"Configure Link-B in reverse splitter mode \r\n");
#endif
		link_ptr = &(desIface->link_b);
		Deser_addr = (desIface->des_alias_addr)>>1;

		reg_addr = CTRL0_REG;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "Link B CTRL0 before : 0x%x\n", read_data[0]);

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		read_data[0] = read_data[0] & (~(1 << AUTO_LINK));
		read_data[0] = read_data[0] & (~(LINK_MASK));
		read_data[0] = read_data[0] | (LINK_REVERSE_SPLITTER);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO, "Rev Split B: CTRL0 =%x\n\r", read_data[0]);
#endif

		reg_addr = GMSL1_EN;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		read_data[0] = read_data[0] & (~(LINK_MASK));
		read_data[0] = read_data[0] | (LINK_REVERSE_SPLITTER);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "B: GMSL1_EN =%x\n\r", read_data[0]);

		reg_addr = CTRL1_REG;
		read_data[0] = read_data[0]|(1<<RESET_ONE_SHOT);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}
		TRACE(MAX_96716_INFO, "Link B after RST: 0x%x\n", read_data[0]);
		while (1) {
			TRACE(MAX_96716_INFO, "Poll for Lock...on link-B \r");
			osSleep(xDelay);

			reg_addr = CTRL9_REG;
			Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
					__func__, __LINE__, Deser_addr, Status);
				return Status;
			}

			TRACE(MAX_96716_INFO, "CTRL9_REG =%x \r\n", read_data[0]);

			if ((read_data[0]&(1<<LOCKED)) == (1<<LOCKED))
				break;
		}

		ser_addr = (link_ptr->serializer_default_addr)>>1;
		reg_addr = DEV_ADDR_REG;
		wr_data[0] = link_ptr->serializer_alias_addr;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "Link B Ser Before %x\n", read_data[0]);

		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_ALWAYS,
			"Serializer on link-B of Des- %d configured to VA-[0x%x]\r\n",
			desIface->Port_DES_index,
			link_ptr->serializer_alias_addr);

		ser_addr = (link_ptr->serializer_alias_addr)>>1;
		reg_addr = DEV_ADDR_REG;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_INFO,
			"Re-verifying ser new addr:%x\r\n", read_data[0]);
#endif

		reg_addr = I2C_2;
		wr_data[0] = link_ptr->sensor_alias_addr;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		reg_addr = I2C_3;
		wr_data[0] = link_ptr->sensor_default_addr;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		reg_addr = I2C_2;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, " I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, " I2C_3 :%x\r\n", read_data[0]);
#endif
		TRACE(MAX_96716_INFO,
			"Sensor on Ser-[0x%x] configured to sensor VA-[0x%x]\r\n",
			link_ptr->serializer_alias_addr,
			link_ptr->sensor_alias_addr);
	} else {
		if (desIface->link_type == LINK_A)
			link_ptr = &(desIface->link_a);
		else
			link_ptr = &(desIface->link_b);

		reg_addr = CTRL0_REG;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "before CTRL0 =%x \r\n", read_data[0]);

		read_data[0] = read_data[0]&(~(1<<AUTO_LINK));
		read_data[0] = read_data[0]&(~(LINK_MASK));
		read_data[0] = read_data[0]|(desIface->link_type);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "after CTRL0 =%x \r\n", read_data[0]);
		if (desIface->link_type == LINK_A)
			reg_addr = CTRL0_REG;
		else
			reg_addr = CTRL1_REG;

		read_data[0] = read_data[0]|(1<<RESET_ONE_SHOT);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		reg_addr = GMSL1_EN;
		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2,
				read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		read_data[0] = read_data[0] & (~(LINK_MASK));
		read_data[0] = read_data[0] | (desIface->link_type);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, Deser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "GMSL1_EN =%x\n\r", read_data[0]);

		while (1) {
#if defined(READ_I2C_REG)
			TRACE(MAX_96716_INFO, "Poll for Lock...on link-%c\r\n",
				(desIface->link_type == LINK_A)?'A':'B');
#endif
			osSleep(xDelay);

			if (desIface->link_type == LINK_A)
				reg_addr = CTRL3_REG;
			else
				reg_addr = CTRL9_REG;

			Status = HalReadI2CReg(i2cBusId, Deser_addr, reg_addr, 0x2, read_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
					__func__, __LINE__, Deser_addr, Status);
				return Status;
			}
#if defined(READ_I2C_REG)
			TRACE(MAX_96716_INFO, "CTRL%d_REG =%x \r\n",
				(desIface->link_type == LINK_A)?'3':'9', read_data[0]);
#endif
			if ((read_data[0]&(1<<LOCKED)) == (1<<LOCKED))
				break;
		}

		ser_addr = (link_ptr->serializer_default_addr)>>1;
		reg_addr = DEV_ADDR_REG;
		wr_data[0] = link_ptr->serializer_alias_addr;

#if defined(READ_I2C_REG)
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "Read Data:%x\r\n", read_data[0]);
#endif
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_ALWAYS, "Serializer on link-%c of
			Deserializer- %d is configured to Virtual Address-[0x%x]\r\n",
				(desIface->link_type == LINK_A)?'A':'B',
				desIface->Port_DES_index, link_ptr->serializer_alias_addr);
#endif
		ser_addr = (link_ptr->serializer_alias_addr)>>1;

#if defined(READ_I2C_REG)
		reg_addr = DEV_ADDR_REG;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "Read Data:%x\r\n", read_data[0]);
#endif
		reg_addr = I2C_2;
		wr_data[0] = link_ptr->sensor_alias_addr;

		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		reg_addr = I2C_3;
		wr_data[0] = link_ptr->sensor_default_addr;

		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		reg_addr = I2C_2;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
				__func__, __LINE__, ser_addr, Status);
			return Status;
		}

		TRACE(MAX_96716_INFO, "I2C_3 :%x\r\n", read_data[0]);
		TRACE(MAX_96716_INFO, "Sensor on Serializer-[0x%x] is
			configured to sensor Virtual Address-[0x%x]\r\n",
				link_ptr->serializer_alias_addr, link_ptr->sensor_alias_addr);
#endif

	}
	return Status;
}

/**************************************************************************
 *          get_max96716_des_array
 *
 * @brief   Get the deserializer initialization array based on port index
 *
 * @param   des           Pointer to deserializer interface structure
 * @param   max96716_des_arr       Pointer to pointer for deserializer array
 *
 * @return  Return the length of the array.
 * @retval  len           Number of elements in the initialization array
 *
 *************************************************************************/
u32 get_max96716_des_array(desInterface *des, RegI2CT **max96716_des_arr)
{
	if (des == NULL || max96716_des_arr == NULL) {
		TRACE(MAX_96716_ERROR,
			"%s:%d Invalid input params\n",
			__func__, __LINE__);
		return RET_NULL_POINTER;
	}

	int len = 0;

	if (des->Port_DES_index == PDB_DES_DES1) {
		*max96716_des_arr = max96716_Des1_init;
		len = (sizeof(max96716_Des1_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES2) {
		*max96716_des_arr = max96716_Des2_init;
		len = (sizeof(max96716_Des2_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES3) {
		*max96716_des_arr = max96716_Des3_init;
		len = (sizeof(max96716_Des3_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES4) {
		*max96716_des_arr = max96716_Des4_init;
		len = (sizeof(max96716_Des4_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES5) {
		*max96716_des_arr = max96716_Des5_init;
		len = (sizeof(max96716_Des5_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES6) {
		*max96716_des_arr = max96716_Des6_init;
		len = (sizeof(max96716_Des6_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES7) {
		*max96716_des_arr = max96716_Des7_init;
		len = (sizeof(max96716_Des7_init)) / (sizeof(RegI2CT));
	} else {
		return RET_INVALID_PARM;
	}

	return len;
}

/**************************************************************************
 *          max96716_Xylon_Deser_setup
 *
 * @brief   Setup and initialize the deserializer interface
 *
 * @param   des           Pointer to deserializer interface structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Setup completed successfully
 * @retval  RET_FAILURE   Setup failed
 *
 *************************************************************************/
static RESULT max96716_Xylon_Deser_setup(desInterface *des)
{
	RESULT Status = RET_SUCCESS;

	if (des == NULL)
		return RET_NULL_POINTER;

	u8 wr_data[4] = {0};
	u8 i2cBusId = GetI2cBusIdForDes(des->Port_DES_index);

	u32 len = 0, reg_idx = 0;
	RegI2CT *Deserializer_initialization = NULL;

	if (des->des_state == in_deinit) {
		u16 Deser_addr = (des->des_alias_addr) >> 1;

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_ALWAYS,
			"Init De-serializer at VA = 0x%x...",
			Deser_addr);
#endif
		len = get_max96716_des_array(des, &Deserializer_initialization);

		for (reg_idx = 0; reg_idx < len; reg_idx++) {
			if ((Deserializer_initialization + reg_idx)->addr == MAX929X_TABLE_END)
				break;

			if ((Deserializer_initialization + reg_idx)->addr == MAX929X_TABLE_WAIT) {
				osSleep((Deserializer_initialization + reg_idx)->val);
				continue;
			}

			wr_data[0] = ((Deserializer_initialization + reg_idx)->val);

			Status = HalWriteI2CReg(i2cBusId, Deser_addr,
					(Deserializer_initialization + reg_idx)->addr, 0x2, wr_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
					__func__, __LINE__, Deser_addr, Status);
				return Status;
			}

#if defined(READ_I2C_REG)
			Status = HalReadI2CReg(i2cBusId, Deser_addr,
					(Deserializer_initialization + reg_idx)->addr, 0x2, rd_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
					__func__, __LINE__, Deser_addr, Status);
				return Status;
			}

			TRACE(MAX_96716_INFO,
				"[%s] [%d] Addr 0x%x -> val 0x%x.\n",
				__func__, __LINE__,
				(Deserializer_initialization + reg_idx)->addr,
				rd_data[0]);
#endif
			if (reg_idx == 0)
				osSleep(MAX96716_REMAP_DELAY_MS);
		}

		des->des_state = in_init;
		des->des_state = in_running;

#if defined(READ_I2C_REG)
		TRACE(MAX_96716_ALWAYS, "\n\rInitialization Done...\n\r");
#endif
	} else {
		TRACE(MAX_96716_WARNING, "De-Serializer Already in Running state(%d)\n\r",
			des->des_state);
	}

	return Status;
}

/**************************************************************************
 *          max96716_Xylon_Deser_Enable
 *
 * @brief   Enable a specific deserializer by position
 *
 * @param   pos           Position/index of the deserializer to enable
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Success
 * @retval  RET_FAILURE   Operation failed
 *
 *************************************************************************/
static RESULT max96716_Xylon_Deser_Enable(u8 pos)
{
	RESULT Status = RET_SUCCESS;

	u8 expander_addr = 0;
	u32 register_addr = 0;
	u16 bytes_read = 1;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};

	expander_addr = MAX96716_EXPANDER_ADDR;
	wr_data[0] = 0x00;
	register_addr = MAX96716_EXPANDER_OUT_PORT1_REG;

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
		register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, expander_addr, Status);
		return Status;
	}
	wr_data[0] = read_data[0] & (~(1<<pos));
	expander_addr = MAX96716_EXPANDER_ADDR;
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr,
		register_addr, MAX96716_FMC_REG_ADDR_SIZE, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, expander_addr, Status);
		return Status;
	}

	osSleep(MAX96716_REMAP_DELAY_MS);

	expander_addr = MAX96716_EXPANDER_ADDR;
	wr_data[0] = read_data[0] | (1<<pos);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr,
		register_addr, MAX96716_FMC_REG_ADDR_SIZE, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, expander_addr, Status);
		return Status;
	}

	osSleep(MAX96716_REMAP_DELAY_MS);
#if defined(READ_I2C_REG)
	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
		register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, expander_addr, Status);
		return Status;
	}

	TRACE(MAX_96716_ALWAYS, "Enabled De-Serializer-%d\n", pos + 1);
	TRACE(MAX_96716_INFO, "expander:Register OutB=0x%x\n", read_data[0]);
#endif

	return Status;
}

/**************************************************************************
 *          max96716_Xylon_Deser_Disable
 *
 * @brief   Disable a specific deserializer by position
 *
 * @param   pos           Position/index of the deserializer to disable
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Success
 * @retval  RET_FAILURE   Operation failed
 *
 *************************************************************************/
static RESULT max96716_Xylon_Deser_Disable(u8 pos)
{
	RESULT Status = RET_SUCCESS;

	u8 expander_addr = 0;
	u32 register_addr = 0;
	u16 bytes_read = 1;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};

	expander_addr = MAX96716_EXPANDER_ADDR;
	register_addr = MAX96716_EXPANDER_OUT_PORT1_REG;

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
		register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, expander_addr, Status);
		return Status;
	}

	wr_data[0] = read_data[0] & (~(1<<pos));
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr,
		register_addr, MAX96716_FMC_REG_ADDR_SIZE, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C write failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, expander_addr, Status);
		return Status;
	}
#if defined(READ_I2C_REG)
	osSleep(MAX96716_REMAP_DELAY_MS);

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
		register_addr, MAX96716_FMC_REG_ADDR_SIZE, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_96716_ERROR, "%s:%d I2C read failed addr=0x%x (err=%d)\n",
			__func__, __LINE__, expander_addr, Status);
		return Status;
	}


	TRACE(MAX_96716_ALWAYS, "Disabled DeSerializer-%d \r\n", pos + 1);
#endif
	return Status;
}
