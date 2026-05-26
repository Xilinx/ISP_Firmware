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
 ************************************************************************/
#include <fmc/max9296.h>

CREATE_TRACER(MAX_9296_INFO, "MAX_9296_INFO: ", INFO, 1);
CREATE_TRACER(MAX_9296_WARNING, "MAX_9296_WARNING: ", WARNING, 1);
CREATE_TRACER(MAX_9296_ERROR, "MAX_9296_ERROR: ", ERROR, 1);
CREATE_TRACER(MAX_9296_ALWAYS, "MAX_9296_ALWAYS: ", ALWAYS, 1);

static u32 fmcinitDone;
static u32 core_des_status;

IsiFmc_t g_fmc_max9296 = {
	.FmcName                       = "max9296_xylon_fmc",
	.pIsiIsiFmcSetup               =  xylon_Fmc_Setup,
	.pIsiDeserSetup                =  xylon_Deser_setup,
	.pIsiDeserEnable               =  xylon_Deser_Enable,
	.pIsiDeserDisable              =  xylon_Deser_Disable,
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


desInterface des_arr[DS_MAX] = {
	{
		MAX9296_DS1_DEFAULT_ADDRESS,
		MAX9296_DS1_ALIAS_ADDRESS,
		PDB_DES_DES1,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_0_ALIAS_ADDR,
	    SENSOR_DEFAULT_ADDRSS, SENSOR_0_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_1_ALIAS_ADDR,
	    SENSOR_DEFAULT_ADDRSS, SENSOR_1_ALIAS_ADDR}
	},
	{
		MAX9296_DS2_DEFAULT_ADDRESS,
		MAX9296_DS2_ALIAS_ADDRESS,
		PDB_DES_DES2,
		in_deinit,
		NO_LINK,
#ifdef RGBIR_MODE
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_2_ALIAS_ADDR,
	SENSOR_OX5B_ADDRESS, SENSOR_2_ALIAS_ADDR},
#else
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_2_ALIAS_ADDR,
	 SENSOR_DEFAULT_ADDRSS, SENSOR_2_ALIAS_ADDR},
#endif
		{/*Link B*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_3_ALIAS_ADDR,
	 SENSOR_DEFAULT_ADDRSS, SENSOR_3_ALIAS_ADDR}
	},
	{
		MAX9296_DS3_DEFAULT_ADDRESS,
		MAX9296_DS3_ALIAS_ADDRESS,
		PDB_DES_DES3,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_4_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_4_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_5_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_5_ALIAS_ADDR}
	},
	{
		MAX9296_DS4_DEFAULT_ADDRESS,
		MAX9296_DS4_ALIAS_ADDRESS,
		PDB_DES_DES4,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_6_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_6_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_7_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_7_ALIAS_ADDR}
	},
	{
		MAX9296_DS5_DEFAULT_ADDRESS,
		MAX9296_DS5_ALIAS_ADDRESS,
		PDB_DES_DES5,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_8_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_8_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_9_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_9_ALIAS_ADDR}
	},
	{
		MAX9296_DS6_DEFAULT_ADDRESS,
		MAX9296_DS6_ALIAS_ADDRESS,
		PDB_DES_DES6,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_10_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_10_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_11_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_11_ALIAS_ADDR}
	},
	{
		MAX9296_DS2_DEFAULT_ADDRESS,
		MAX9296_DS2_ALIAS_ADDRESS,
		PDB_DES_DES7,
		in_deinit,
		NO_LINK,
		{/*Link A*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_2_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_2_ALIAS_ADDR},
		{/*Link B*/SERIALIZER_DEFAULT_ADDR, SERIALIZER_3_ALIAS_ADDR,
	SENSOR_DEFAULT_ADDRSS, SENSOR_3_ALIAS_ADDR}
	},
};

/*
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
 *
 * Adjust the i2cBusIds array based on your hardware configuration.
 */
const core_des_mapping_t max9296_core_des_map[] = {
	/* Core 6: Manages 3 deserializers*/
	/*(DS_ONE, DS_TWO, DS_THREE) all on I2C bus 1 */
	{6, 3, {DS_TWO, DS_THREE, DS_FOUR}, {0, 0, 0}}, /* 4+1*/

	/* Core 7: Manages 1 deserializer (DS_FOUR) on I2C bus 2 */
	{7, 0, {0, 0, 0}, {0, 0, 0}}, /* 4+1*/

	/* Core 8: Manages 2 deserializers (DS_FIVE, DS_SIX) both on I2C bus 3 */
	{8, 0, {0, 0, 0}, {0, 0, 0}}, /* 1+1 */
};

/**************************************************************************
 *          on_board_topology
 *
 * @brief   Display on-board sensor status and topology information
 *
 * @return  None
 *
 ************************************************************************/
void on_board_topology(void)
{
	u8 i = 0;

	TRACE(MAX_9296_INFO, "\n\r	On-Board Sensor Status==>");
	TRACE(MAX_9296_INFO, "\n\r\tDe-serializer"
		"\tSensor on link-a  Sensor on link-b\n\r ");

	for (i = 0; i < DS_MAX; i++) {
		if (des_arr[i].link_type == LINK_REVERSE_SPLITTER) {
			TRACE(MAX_9296_INFO, "\n\r %d				Y		Y\n\r ",
					i + 1);
		} else if (des_arr[i].link_type == NO_LINK) {
			TRACE(MAX_9296_INFO, "\n\r %d				N		N\n\r ",
					i + 1);
		} else if (des_arr[i].link_type == LINK_A) {
			TRACE(MAX_9296_INFO, "\n\r %d				Y		N\n\r ",
					i + 1);
		} else {
			TRACE(MAX_9296_INFO, "\n\r %d				N		Y\n\r ",
					i + 1);
		}
	}
}

/**************************************************************************
 *          WrFmcDataTo_sharedmem
 *
 * @brief   Write FMC data to shared memory
 *
 * @return  None
 *
 ************************************************************************/
void WrFmcDataTo_sharedmem(void)
{
	byte_memcpy(SHARED_DES_ARRAY_SRUCT, des_arr, sizeof(des_arr));
	TRACE(MAX_9296_ALWAYS, "Des-arr structure"
		"loaded by Core-%x for all other cores\r\n",
		cpu_id);
}

/**************************************************************************
 *          RdFmcReadFrom_sharedmem
 *
 * @brief   Read FMC data from shared memory
 *
 * @return  None
 *
 ************************************************************************/
void RdFmcReadFrom_sharedmem(void)
{
	byte_memcpy(des_arr, SHARED_DES_ARRAY_SRUCT, sizeof(des_arr));
	on_board_topology();
}

/**************************************************************************
 *          xylon_Fmc_Setup
 *
 * @brief   Setup FMC (FPGA Mezzanine Card) configuration for Xylon interface
 *
 * @param   des_arr_id    Deserializer array identifier
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Setup completed successfully
 * @retval  RET_FAILURE   Setup failed
 *
 ************************************************************************/
RESULT xylon_Fmc_Setup(int des_arr_id)
{
	(void)des_arr_id; /* Unused parameter */
	RESULT Status = RET_SUCCESS;

	u16 expander_addr = 0;
	u32 register_addr = 0;
	u16 bytes_read = 1;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};
	get_fmcinit_lock();

	if (*FMC_INIT_STATUS_REG == 1)
		fmcinitDone = 1;
	else
		fmcinitDone = 0;

	TRACE(MAX_9296_INFO, "FMC_INIT_STATUS_REG=%x fmcinitDone=%x cpuid=%x\n\r",
				*FMC_INIT_STATUS_REG, fmcinitDone, cpu_id);

	if (fmcinitDone == 0) {
		TRACE(MAX_9296_ALWAYS, "****************************************\r\n");
		TRACE(MAX_9296_ALWAYS, "FMC Init Started...\r\n");

		register_addr = MAX9296_POTENTIOMETER_REG;
		u8 potent_addr = MAX9296_POTENTIOMETER_ADDR;

		wr_data[0] = MAX9296_POTENTIOMETER_WR_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO,
				potent_addr, register_addr, 0x1, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

		osSleep(MAX9296_FMC_SETUP_DELAY_MS);

#if defined(READ_I2C_REG)
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, potent_addr,
				register_addr, 0x1, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
	    __func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, "potentiometer : Data Read is:[0]:0x%x\r\n",
	    read_data[0]);
#endif
		register_addr = MAX9296_LDAC_DAC_REG;
		u16 ldac_addr = MAX9296_LDAC_ADDR;

		wr_data[0] = MAX9296_LDAC_DAC_POWER_UP_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ldac_addr,
				register_addr, 0x1, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ldac_addr,
				register_addr, 0x1, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, "ldac : Data Read is:[0]:0x%x\r\n", read_data[0]);
#endif

		ldac_addr = MAX9296_LDAC_ADDR;
		register_addr = MAX9296_LDAC_MODE_REG;

		wr_data[0] = MAX9296_LDAC_MODE_6GBPS_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ldac_addr,
				register_addr, 0x1, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ldac_addr,
				register_addr, 0x1, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, "DAC mode 6Gbps done : \r\n");
		TRACE(MAX_9296_INFO, "Data Read is:[0]:0x%x\r\n", read_data[0]);
#endif
		register_addr = MAX9296_EXPANDER_IODIRB_REG;
		expander_addr = MAX9296_EXPANDER_ADDR;
		wr_data[0] = MAX9296_EXPANDER_IODIRB_ALL_OUT;

		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr,
				register_addr, 0x1, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

		osSleep(MAX9296_FMC_SETUP_DELAY_MS);

#if defined(READ_I2C_REG)
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
	register_addr, 0x1, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, "expander_Reg :IODIRB is:[0]:0x%x\r\n",
	    read_data[0]);
		TRACE(MAX_9296_INFO, "expander_addr : Data Read is:[0]:0x%x\r\n",
	    read_data[0]);
#endif
		register_addr = MAX9296_EXPANDER_OUTB_REG;

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
				register_addr, 0x1, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
		__func__, __LINE__, Status);
			return Status;
		}

		wr_data[0] = read_data[0] | ((1<<CAM_SUPPLY_EN));
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr,
				register_addr, 0x1, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				 __func__, __LINE__, Status);
			return Status;
		}

		osSleep(MAX9296_FMC_SETUP_DELAY_MS);

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
				register_addr, 0x1, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				 __func__, __LINE__, Status);
			return Status;
		}

		wr_data[0] = read_data[0] & (~(1<<CAM_SUPPLY_EN));
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr,
				register_addr, 0x1, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				 __func__, __LINE__, Status);
			return Status;
		}

		osSleep(MAX9296_PROBE_DELAY_MS);

#if defined(READ_I2C_REG)
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr,
				register_addr, 0x1, read_data, bytes_read);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}
#endif
		Status |= xylon_Deser_Disable(PDB_DES_DES1);
		Status |= xylon_Deser_Disable(PDB_DES_DES2);
		Status |= xylon_Deser_Disable(PDB_DES_DES3);
		Status |= xylon_Deser_Disable(PDB_DES_DES4);
		Status |= xylon_Deser_Disable(PDB_DES_DES5);
		Status |= xylon_Deser_Disable(PDB_DES_DES6);

		TRACE(MAX_9296_ALWAYS, "Remapping I2c Address on FMC started...\r\n");
		Remapping_I2C_addressess(&des_arr[DS_ONE]);
		Remapping_I2C_addressess(&des_arr[DS_TWO]);
		Remapping_I2C_addressess(&des_arr[DS_THREE]);
		Remapping_I2C_addressess(&des_arr[DS_FOUR]);
		Remapping_I2C_addressess(&des_arr[DS_FIVE]);
		Remapping_I2C_addressess(&des_arr[DS_SIX]);
//		Remapping_I2C_addressess(&des_arr[DS_SEVEN]);
		TRACE(MAX_9296_ALWAYS, "Remapping I2c Address on FMC Done...\r\n");

#if defined(I2C_FAST_MODE)
		TRACE(MAX_9296_ALWAYS, "Reverting I2C Bus to 400KHz .....\r\n");
		revert_to_400khz();

		TRACE(MAX_9296_ALWAYS, "Running I2C Bus at 400KHz .....\r\n");
#else

		TRACE(MAX_9296_ALWAYS, "Running I2C Bus at 100KHz .....\r\n");
#endif

		WrFmcDataTo_sharedmem();
		core_des_status = 1;
		*FMC_INIT_STATUS_REG = 1;
		fmcinitDone = 1;

		TRACE(MAX_9296_ALWAYS, "FMC Init Done\r\n");
		TRACE(MAX_9296_ALWAYS, "************************************\r\n");

		on_board_topology();
	} else {
		if (core_des_status == 0) {
			RdFmcReadFrom_sharedmem();
			core_des_status = 1;
			TRACE(MAX_9296_ALWAYS, "Des-arr structure"
				"loaded for Core-%x from Shared memory\r\n",
				cpu_id);
		} else {
			TRACE(MAX_9296_ALWAYS, "Des-arr structure"
				"already loaded for Core-%x \r\n",
				cpu_id);
		}
	}

	release_fmcinit_lock();
	TRACE(MAX_9296_ALWAYS, "FMC init Lock Released \r\n");
	return Status;
}

/**************************************************************************
 *          enable_link
 *
 * @brief   Enable specified link on the deserializer
 *
 * @param   Deser_addr    Deserializer I2C address
 * @param   link_type     Type of link to enable
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 ************************************************************************/
RESULT enable_link(u8 Deser_addr, u8 link_type)
{
	RESULT Status = RET_SUCCESS;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};
	u16 reg_addr = CTRL0_REG;

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr, MAX9296_REG_ADDR_SIZE,
			read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

	read_data[0] = read_data[0] & (~(1 << AUTO_LINK));
	read_data[0] = read_data[0] & (~(LINK_MASK));
	read_data[0] = read_data[0] | (link_type);
	wr_data[0] = read_data[0];

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr, MAX9296_REG_ADDR_SIZE,
			wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr, MAX9296_REG_ADDR_SIZE,
			read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

#if defined(READ_I2C_REG)
	TRACE(MAX_9296_INFO, "CTRL0 =%x\n\r", read_data[0]);
#endif

	read_data[0] = read_data[0] | (1 << RESET_ONE_SHOT);
	wr_data[0] = read_data[0];

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr, MAX9296_REG_ADDR_SIZE,
				wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

	return Status;
}

/**************************************************************************
 *          probe_if_links
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
 ************************************************************************/
RESULT probe_if_links(u8 Deser_addr, desInterface *desIface)
{
	RESULT Status = RET_SUCCESS;

	u8 read_data[2] = {0};
	u16 reg_addr;

	u8 dev_a = FALSE;
	u8 dev_b = FALSE;

	u8 itr_num = 1;

	enable_link(Deser_addr, LINK_A);

	while (1) {
#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, "\n\rProbing sensor on link-A of Des%d",
				desIface->Port_DES_index + 1);
#endif
		osSleep(MAX9296_PROBE_LINK_DELAY_MS);

		reg_addr = CTRL3_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, "CTRL3_REG =%x\n\r", read_data[0]);
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
	enable_link(Deser_addr, LINK_B);

	while (1) {
#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, "\n\rProbing sensor on link-B of Des%d",
				desIface->Port_DES_index + 1);
#endif
		osSleep(MAX9296_PROBE_LINK_DELAY_MS);

		reg_addr = CTRL3_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, "CTRL3_REG =%x\n\r", read_data[0]);
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
 *          Remapping_I2C_addressess
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
 ************************************************************************/
static RESULT Remapping_I2C_addressess(desInterface *desIface)
{
	RESULT Status = RET_SUCCESS;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};
	u8 Deser_addr;
	u8 ser_addr;
	u16 reg_addr;
	dslink *link_ptr;

	Deser_addr = (desIface->des_actual_addr)>>1;

	if (desIface->Port_DES_index == PDB_DES_DES7)
		Status |= xylon_Deser_Enable(PDB_DES_DES2);
	else
		Status |= xylon_Deser_Enable(desIface->Port_DES_index);

	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
			__func__, __LINE__, Status);
		return Status;
	}

	if (desIface->Port_DES_index == PDB_DES_DES1
		|| desIface->Port_DES_index == PDB_DES_DES3
		|| desIface->Port_DES_index == PDB_DES_DES5) {

		reg_addr = DEV_ADDR_REG;
		wr_data[0] = desIface->des_alias_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
				reg_addr, MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		Deser_addr = (desIface->des_alias_addr) >> 1;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
				reg_addr, MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		if (desIface->des_alias_addr != (read_data[0] & MAX9296_ADDR_MASK)) {
			TRACE(MAX_9296_ERROR, "%s:%d Addr mismatch:"
				"expected=0x%x, actual=0x%x\n",
				__func__, __LINE__,
				desIface->des_alias_addr,
				(read_data[0] & MAX9296_ADDR_MASK));
			return RET_FAILURE;
		}
	}

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
			CTRL0_REG, MAX9296_REG_ADDR_SIZE, read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

	wr_data[0] = read_data[0] | MAX9296_CTRL1_RESET_LINK_BIT;
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
			CTRL0_REG, MAX9296_REG_ADDR_SIZE, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

	osSleep(MAX9296_FMC_SETUP_DELAY_MS);

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
			CTRL0_REG, MAX9296_REG_ADDR_SIZE, read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

	probe_if_links(Deser_addr, desIface);

	if (desIface->link_type == NO_LINK) {
		TRACE(MAX_9296_ERROR, "%s: No Sensor"
			"Available on Des-[0x%x],"
			"skipping address remap\n",
			__func__, desIface->des_alias_addr);
#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO,
			"No Sensor Available on link a/B"
			"on Des- %d"
			" so not re-mapping Addr"
			"of Ser and Sensor\r\n",
			desIface->Port_DES_index);
#endif
		if (desIface->Port_DES_index == PDB_DES_DES7)
			Status |= xylon_Deser_Disable(PDB_DES_DES2);
		else
			Status |= xylon_Deser_Disable(desIface->Port_DES_index);

		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
	    __func__, __LINE__, Status);
			return Status;
		}

		return RET_NOTAVAILABLE;
	}

#if defined(READ_I2C_REG)
	TRACE(MAX_9296_INFO, "Re-mapping Serializer"
		"& Sensor Addresses at"
		"Deserializer-[0x%x]\r\n",
		desIface->des_alias_addr);
#endif

	if (desIface->link_type == LINK_REVERSE_SPLITTER) {
		TRACE(MAX_9296_ALWAYS, "Configure"
			"De-serializer in reverse"
			"splitter mode \r\n");

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, "Configure Link-A in reverse splitter mode \r\n");
#endif

		link_ptr = &(desIface->link_a);
		Deser_addr = (desIface->des_alias_addr)>>1;
		reg_addr = CTRL0_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		read_data[0] = read_data[0] & (~(1<<AUTO_LINK));
		read_data[0] = read_data[0]&(~(LINK_MASK));
		read_data[0] = read_data[0] | LINK_A;
		wr_data[0] = read_data[0];

		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		read_data[0] = read_data[0] | (1<<RESET_ONE_SHOT);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		while (1) {
			TRACE(MAX_9296_ALWAYS, "Poll for Lock...on link-A \r\n");
			osSleep(MAX9296_PROBE_DELAY_MS);
			reg_addr = CTRL3_REG;

			Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
					reg_addr, MAX9296_REG_ADDR_SIZE, read_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
					__func__, __LINE__, Status);
				return Status;
			}

			TRACE(MAX_9296_ALWAYS, "CTRL3_REG =%x \r\n", read_data[0]);
			if ((read_data[0] & (1<<LOCKED)) == (1<<LOCKED))
				break;
		}

		ser_addr = (link_ptr->serializer_default_addr)>>1;
		reg_addr = MAX9296_SER_DEV_ADDR_REG;
		wr_data[0] = link_ptr->serializer_alias_addr;

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_ALWAYS, "Ser on link-A"
			"of Des- %d is configured"
			"to Virtual Addr-[0x%x]\r\n",
			desIface->Port_DES_index,
			link_ptr->serializer_alias_addr);

		ser_addr = (link_ptr->serializer_alias_addr)>>1;
		reg_addr = MAX9296_SER_DEV_ADDR_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, " Just re verifying  serializer new addr :%x\r\n",
				read_data[0]);
#endif
		reg_addr = MAX9296_SER_GPIO_REG_A;
		wr_data[0] = MAX9296_SER_GPIO_DOUBLE_MODE_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		reg_addr = MAX9296_SER_GPIO_REG_B;
		wr_data[0] = MAX9296_SER_GPIO_DOUBLE_MODE_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		reg_addr = MAX9296_SER_GPIO_REG_C;
		wr_data[0] = MAX9296_SER_GPIO_DOUBLE_MODE_VAL;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		reg_addr = I2C_2;
		wr_data[0] = link_ptr->sensor_alias_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		reg_addr = I2C_3;
		wr_data[0] = link_ptr->sensor_default_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		reg_addr = I2C_2;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " I2C_3 :%x\r\n", read_data[0]);
#endif
		TRACE(MAX_9296_ALWAYS, "Sensor on Ser-[0x%x]"
			"is configured to sensor"
			"Virtual Addr-[0x%x]\r\n",
			link_ptr->serializer_alias_addr,
			link_ptr->sensor_alias_addr);

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, "Configure Link-B in reverse splitter mode \r\n");
#endif

		link_ptr = &(desIface->link_b);
		Deser_addr = (desIface->des_alias_addr)>>1;
		reg_addr = CTRL0_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		read_data[0] = read_data[0]&(~(1<<AUTO_LINK));
		read_data[0] = read_data[0]&(~(LINK_MASK));
		read_data[0] = read_data[0]|LINK_REVERSE_SPLITTER;
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		read_data[0] = read_data[0]|(1<<RESET_ONE_SHOT);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		while (1) {
			TRACE(MAX_9296_ALWAYS, "Poll for Lock...on link-B \r\n");
			osSleep(MAX9296_PROBE_DELAY_MS);

			reg_addr = CTRL3_REG;
			Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
				reg_addr, MAX9296_REG_ADDR_SIZE, read_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
					__func__, __LINE__, Status);
				return Status;
			}

			TRACE(MAX_9296_ALWAYS, "CTRL3_REG =%x \r\n", read_data[0]);
			if ((read_data[0]&(1<<LOCKED)) == (1<<LOCKED))
				break;
		}

		ser_addr = (link_ptr->serializer_default_addr)>>1;
		reg_addr = MAX9296_SER_DEV_ADDR_REG;
		wr_data[0] = link_ptr->serializer_alias_addr;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_ALWAYS, "Ser on link-B"
			"of Des- %d is configured"
			"to Virtual Addr-[0x%x]\r\n",
			desIface->Port_DES_index,
			link_ptr->serializer_alias_addr);

		ser_addr = (link_ptr->serializer_alias_addr)>>1;
		reg_addr = MAX9296_SER_DEV_ADDR_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, " Just re verifying"
			"serializer new addr:%x\r\n",
			read_data[0]);
#endif

		reg_addr = I2C_2;
		wr_data[0] = link_ptr->sensor_alias_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		reg_addr = I2C_3;
		wr_data[0] = link_ptr->sensor_default_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		reg_addr = I2C_2;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " I2C_3 :%x\r\n", read_data[0]);
#endif

		TRACE(MAX_9296_ALWAYS, "Sensor on Ser-[0x%x]"
			"is configured to sensor"
			"Virtual Addr-[0x%x]\r\n",
			link_ptr->serializer_alias_addr,
			link_ptr->sensor_alias_addr);
	} else {
		if (desIface->link_type == LINK_A)
			link_ptr = &(desIface->link_a);
		else
			link_ptr = &(desIface->link_b);

		reg_addr = CTRL0_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		read_data[0] = read_data[0]&(~(1<<AUTO_LINK));
		read_data[0] = read_data[0]&(~(LINK_MASK));
		read_data[0] = read_data[0]|(desIface->link_type);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_INFO, "CTRL0 =%x \r\n", read_data[0]);
#endif

		read_data[0] = read_data[0]|(1<<RESET_ONE_SHOT);
		wr_data[0] = read_data[0];
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		while (1) {
#if defined(READ_I2C_REG)
			TRACE(MAX_9296_ALWAYS, "Poll for Lock...on link-%c\r\n",
					(desIface->link_type == LINK_A)?'A':'B');
#endif

			osSleep(MAX9296_PROBE_DELAY_MS);

			reg_addr = CTRL3_REG;
			Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
				reg_addr, MAX9296_REG_ADDR_SIZE, read_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
					__func__, __LINE__, Status);
				return Status;
			}
#if defined(READ_I2C_REG)
			TRACE(MAX_9296_ALWAYS, "CTRL3_REG =%x \r\n", read_data[0]);
#endif
			if ((read_data[0]&(1<<LOCKED)) == (1<<LOCKED))
				break;
		}

		ser_addr = (link_ptr->serializer_default_addr)>>1;
		reg_addr = MAX9296_SER_DEV_ADDR_REG;

#if defined(READ_I2C_REG)
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " Read Data:%x\r\n", read_data[0]);
#endif

		wr_data[0] = link_ptr->serializer_alias_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_ALWAYS, "Ser on link-%c"
			"of Des- %d is configured"
			"to Virtual Addr-[0x%x]\r\n",
			(desIface->link_type == LINK_A)?'A':'B',
			desIface->Port_DES_index,
			link_ptr->serializer_alias_addr);
#endif
		ser_addr = (link_ptr->serializer_alias_addr)>>1;

#if defined(READ_I2C_REG)
		reg_addr = MAX9296_SER_DEV_ADDR_REG;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " Read Data:%x\r\n", read_data[0]);
#endif
		reg_addr = I2C_2;
		wr_data[0] = link_ptr->sensor_alias_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		reg_addr = I2C_3;
		wr_data[0] = link_ptr->sensor_default_addr;
		Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

#if defined(READ_I2C_REG)
		reg_addr = I2C_2;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, ser_addr, reg_addr,
				MAX9296_REG_ADDR_SIZE, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
				__func__, __LINE__, Status);
			return Status;
		}

		TRACE(MAX_9296_INFO, " I2C_3 :%x\r\n", read_data[0]);
		TRACE(MAX_9296_ALWAYS, "Sensor on Ser-[0x%x]"
			"is configured to sensor"
			"Virtual Addr-[0x%x]\r\n",
			link_ptr->serializer_alias_addr,
			link_ptr->sensor_alias_addr);
#endif

	}
	return Status;
}

/**************************************************************************
 *          get_des_array
 *
 * @brief   Get the deserializer initialization array based on port index
 *
 * @param   des           Pointer to deserializer interface structure
 * @param   Des_arr       Pointer to pointer for deserializer array
 *
 * @return  Return the length of the array.
 * @retval  len           Number of elements in the initialization array
 *
 ************************************************************************/
u32 get_des_array(desInterface *des, RegI2CT **Des_arr)
{
	if (des == NULL || Des_arr == NULL) {
		TRACE(MAX_9296_ERROR, "%s: NULL pointer (des=%p, Des_arr=%p)\n",
			__func__, des, Des_arr);
		return RET_NULL_POINTER;
	}

	int len = 0;

	if (des->Port_DES_index == PDB_DES_DES1) {
		*Des_arr = max9296_Des1_init;
		len = (sizeof(max9296_Des1_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES2) {
		*Des_arr = max9296_Des2_init;
		len = (sizeof(max9296_Des2_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES3) {
		*Des_arr = max9296_Des3_init;
		len = (sizeof(max9296_Des3_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES4) {
		*Des_arr = max9296_Des4_init;
		len = (sizeof(max9296_Des4_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES5) {
		*Des_arr = max9296_Des5_init;
		len = (sizeof(max9296_Des5_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES6) {
		*Des_arr = max9296_Des6_init;
		len = (sizeof(max9296_Des6_init)) / (sizeof(RegI2CT));
	} else if (des->Port_DES_index == PDB_DES_DES7) {
		*Des_arr = max9296_Des7_init;
		len = (sizeof(max9296_Des7_init)) / (sizeof(RegI2CT));
	} else {
		TRACE(MAX_9296_ERROR, "%s: Invalid Port_DES_index=%d\n",
			__func__, des->Port_DES_index);
		return RET_INVALID_PARM;
	}

	return len;
}

/**************************************************************************
 *          xylon_Deser_setup
 *
 * @brief   Setup and initialize the deserializer interface
 *
 * @param   des           Pointer to deserializer interface structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Setup completed successfully
 * @retval  RET_FAILURE   Setup failed
 *
 ************************************************************************/
static RESULT xylon_Deser_setup(desInterface *des)
{
	RESULT Status = RET_SUCCESS;

	u8 wr_data[4] = {0};
	u8 rd_data[4] = {0};

	u32 len = 0, reg_index = 0;
	RegI2CT *Deserializer_initialization = NULL;

	if (des == NULL) {
		TRACE(MAX_9296_ERROR, "%s: des is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (des->des_state == in_deinit) {
		u16 Deser_addr = (des->des_alias_addr) >> 1;

#if defined(READ_I2C_REG)
		TRACE(MAX_9296_ALWAYS, "Initializing"
			"De-serializer at Virtual"
			"Address = 0x%x...",
			Deser_addr);
#endif
		len = get_des_array(des, &Deserializer_initialization);

		for (reg_index = 0; reg_index < len; reg_index++) {
			if ((Deserializer_initialization + reg_index)->addr ==
				MAX929X_TABLE_END)
				break;

			if ((Deserializer_initialization+reg_index)->addr ==
				MAX929X_TABLE_WAIT) {
				osSleep((Deserializer_initialization + reg_index)->val);
				continue;
			}

			wr_data[0] = ((Deserializer_initialization + reg_index)->val);

			Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
					(Deserializer_initialization + reg_index)->addr,
					MAX9296_REG_ADDR_SIZE, wr_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
					__func__, __LINE__, Status);
				return Status;
			}

			Status = HalReadI2CReg(IIC_INSTANCE_ZERO, Deser_addr,
					(Deserializer_initialization + reg_index)->addr,
					MAX9296_REG_ADDR_SIZE, rd_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
					__func__, __LINE__, Status);
				return Status;
			}

#if defined(READ_I2C_REG)
			TRACE(MAX_9296_INFO, "[%s] [%d] Address 0x%x -> value 0x%x.\n",
	    __func__, __LINE__, (Deserializer_initialization+reg_index)->addr,
					rd_data[0]);
#endif
		}

		des->des_state = in_init;
		des->des_state = in_running;
#if defined(READ_I2C_REG)
		TRACE(MAX_9296_ALWAYS, "\n\rInitialization Done...\n\r");
#endif
	} else {
		TRACE(MAX_9296_ALWAYS, "De-Serializer Already in Running state(%d)\n\r",
				des->des_state);
	}

	return Status;
}

/**************************************************************************
 *          xylon_Deser_Enable
 *
 * @brief   Enable a specific deserializer by position
 *
 * @param   pos           Position/index of the deserializer to enable
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 ************************************************************************/
static RESULT xylon_Deser_Enable(u8 pos)
{
	RESULT Status = RET_SUCCESS;

	u16 expander_addr = 0;
	u32 register_addr = 0;
	u16 bytes_read = 1;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};

	expander_addr = MAX9296_EXPANDER_ADDR;
	register_addr = MAX9296_EXPANDER_OUTB_REG;

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr, register_addr,
			0x1, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
			__func__, __LINE__, Status);
		return Status;
	}

	wr_data[0] = read_data[0] & (~(1<<pos));
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr, register_addr,
			0x1, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
			__func__, __LINE__, Status);
		return Status;
	}

	osSleep(MAX9296_FMC_SETUP_DELAY_MS);

	wr_data[0] = read_data[0] | (1<<pos);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr, register_addr,
			0x1, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
			__func__, __LINE__, Status);
		return Status;
	}

	osSleep(MAX9296_FMC_SETUP_DELAY_MS);

#if defined(READ_I2C_REG)
	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr, register_addr,
			0x1, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
			__func__, __LINE__, Status);
		return Status;
	}

	TRACE(MAX_9296_INFO, "Enabled De-Serializer-%d\n", pos + 1);
	TRACE(MAX_9296_INFO, "expander:Register OutB=0x%x\n", read_data[0]);
#endif

	return Status;
}

/**************************************************************************
 *          xylon_Deser_Disable
 *
 * @brief   Disable a specific deserializer by position
 *
 * @param   pos           Position/index of the deserializer to disable
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 ************************************************************************/
static RESULT xylon_Deser_Disable(u8 pos)
{
	RESULT Status = RET_SUCCESS;

	u8 expander_addr = 0;
	u32 register_addr = 0;
	u16 bytes_read = 1;

	u8 read_data[2] = {0};
	u8 wr_data[2] = {0};

	expander_addr = MAX9296_EXPANDER_ADDR;
	register_addr = MAX9296_EXPANDER_OUTB_REG;

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr, register_addr,
				0x1, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
			__func__, __LINE__, Status);
		return Status;
	}

	wr_data[0] = read_data[0] & (~(1<<pos));
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, expander_addr, register_addr,
				0x1, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n",
			__func__, __LINE__, Status);
		return Status;
	}

	osSleep(MAX9296_FMC_SETUP_DELAY_MS);

#if defined(READ_I2C_REG)
	Status = HalReadI2CReg(IIC_INSTANCE_ZERO, expander_addr, register_addr,
				0x1, read_data, bytes_read);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9296_ERROR, "%s:%d Status=%d\n", __func__, __LINE__, Status);
		return Status;
	}

	TRACE(MAX_9296_INFO, "Disabled DeSerializer-%d \r\n", pos + 1);
#endif

	return Status;
}
