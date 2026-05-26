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

#include "isi_fmc.h"
#include "isi/isi.h"

#define FMC_ID_0    (0)
#define FMC_ID_1    (1)
#define MAX_FMC_COUNT (2)

#define INVALID_I2C_BUS_ID         (0xFF)
// Deserializer pipe status registers (MAX9296)
#define DES_REG_PIPE_X_STATUS      (0x108)
#define DES_REG_PIPE_Y_STATUS      (0x11a)
#define DES_REG_PIPE_Z_STATUS      (0x12c)
#define DES_REG_PIPE_U_STATUS      (0x13e)
// Serializer pipe status registers (MAX9295)
#define SER_REG_PIPE_X_STATUS      (0x102)
#define SER_REG_PIPE_Y_STATUS      (0x10a)
#define SER_REG_PIPE_Z_STATUS      (0x112)
#define SER_REG_PIPE_U_STATUS      (0x11a)

struct accessIIC *accessiic_handle[MAX_SENSOR_COUNT] = {NULL};
struct sensor_driver *sensor_handle[MAX_SENSOR_COUNT] = {NULL};

CREATE_TRACER(MS_FW_ALWAYS, "MS_FW_ALWAYS: ", ALWAYS, 1);
CREATE_TRACER(MS_FW_INFO, "MS_FW_INFO: ", INFO, 1);
CREATE_TRACER(MS_FW_WARNING, "MS_FW_WARNING: ", WARNING, 1);
CREATE_TRACER(MS_FW_ERROR, "MS_FW_ERROR: ", ERROR, 1);

int current_active_fmc = FMC_ID_0;

/**************************************************************************
 *          get_active_des_array
 *
 * @brief   Get the deserializer array based on active FMC
 *
 * @return  Pointer to des_arr or max96716_des_arr
 *
 ************************************************************************/
desInterface *get_active_des_array(void)
{
	if (current_active_fmc == FMC_ID_0)
		return des_arr;

	if (current_active_fmc == FMC_ID_1)
		return max96716_des_arr;

	TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, current_active_fmc);
	return NULL;
}

/**************************************************************************
 *          get_fmc_instance
 *
 * @brief   Get FMC instance based on FMC ID
 *
 * @param   fmc_id    FMC identifier
 *
 * @return  Pointer to FMC instance or NULL
 *
 ************************************************************************/
IsiFmc_t *get_fmc_instance(int fmc_id)
{
	switch (fmc_id) {
	case FMC_ID_0:
		return &g_fmc_max9296;
	case FMC_ID_1:
		return &g_fmc_max96716;
	default:
		TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, fmc_id);
		return NULL;
	}
}

/**************************************************************************
 *          get_active_fmc
 *
 * @brief   Get currently active FMC instance
 *
 * @return  Pointer to active FMC or NULL
 *
 ************************************************************************/
IsiFmc_t *get_active_fmc(void)
{
	if (current_active_fmc == -1) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected\r\n", __func__);
		return NULL;
	}
	return get_fmc_instance(current_active_fmc);
}

/**************************************************************************
 *          get_active_core_des_map
 *
 * @brief   Get active core deserializer map
 *
 * @return  Pointer to core_des_mapping_t or NULL
 *
 ************************************************************************/
core_des_mapping_t *get_active_core_des_map(void)
{
	if (current_active_fmc == FMC_ID_0)
		return max9296_core_des_map;

	if (current_active_fmc == FMC_ID_1)
		return max96716_core_des_map;

	TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, current_active_fmc);
	return NULL;
}

/**************************************************************************
 *          InitIIC
 *
 * @brief   Initialize PS I2C Instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Initialization successful
 * @retval  Other         Initialization failed
 *
 ************************************************************************/
RESULT InitIIC(void)
{
	RESULT Status = RET_SUCCESS;

	HalI2cConfig_t i2cConfig;

	i2cConfig.i2cBusId = 0;
	i2cConfig.HalI2cMode = HAL_PS_I2C_MODE;
	i2cConfig.hHalI2c = NULL;
	TRACE(MS_FW_INFO, "Inside %s and"
		"before PS INIT and Mode passed: %x\n",
		__func__, i2cConfig.HalI2cMode);

	Status = HalI2cInit(&i2cConfig);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: Failed to"
			"initialize I2C bus %d (error=%d)\n",
			__func__, i2cConfig.i2cBusId, Status);
		return Status;
	}
	TRACE(MS_FW_INFO, "%s: PS I2C bus %d"
		"initialized successfully\n",
		__func__, i2cConfig.i2cBusId);
	return Status;
}

/**************************************************************************/
/**
 * GetI2cBusIdForDes - Lookup I2C bus ID for a given deserializer
 *
 * @param   desId   Deserializer index (DS_ONE, DS_TWO, DS_THREE, etc.)
 *
 * @return  I2C bus ID (0-3) on success, 0xFF on error
 *
 * @note    This function searches the core_des_map table to find which
 *          I2C bus the specified deserializer is connected to.
 *
 *          This centralizes I2C bus topology configuration and provides
 *          a single point of access for I2C bus mapping.
 *
 * Example:
 *   u8 busId = GetI2cBusIdForDes(DS_ONE);  // Returns 1 (from core_des_map)
 *
 ************************************************************************/
u8 GetI2cBusIdForDes(u8 desId)
{
	if (current_active_fmc == FMC_ID_0) {
		/* FMC_ID_0 board has a single shared PS_I2C bus for all
		 * deserializers - no per-des bus mapping needed. */
		return IIC_INSTANCE_ZERO;
	} else if (current_active_fmc == FMC_ID_1) {
		u8 core_idx, des_idx, busId;

		core_des_mapping_t *active_core_des_map = get_active_core_des_map();

		if (!active_core_des_map) {
			TRACE(MS_FW_ERROR, "%s: No Core Des Map selected for des_index %d\r\n",
			__func__, desId);
			return INVALID_I2C_BUS_ID;
		}

		TRACE(MS_FW_INFO, "MAP SIZE : %u\n", MAX96716_CORE_DES_MAP_SIZE);

		for (core_idx = 0; core_idx < MAX96716_CORE_DES_MAP_SIZE; core_idx++) {
			for (des_idx = 0;
			     des_idx < active_core_des_map[core_idx].num_deserializers;
			     des_idx++) {
				if (active_core_des_map[core_idx].des_indices[des_idx] == desId) {
					busId = active_core_des_map[core_idx].i2cBusIds[des_idx];
					TRACE(MS_FW_INFO, "%s: desId=%d -> i2cBusId=%d (core=%d)\n",
							__func__, desId, busId, active_core_des_map[core_idx].core_id);
					return busId;
				}
			}
		}

		TRACE(MS_FW_ERROR, "ERROR: %s - desId %d not found in core_des_map!\n",
			__func__, desId);
		return INVALID_I2C_BUS_ID;
	}
	TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, current_active_fmc);
	return INVALID_I2C_BUS_ID;
}

/**************************************************************************
 *          init_des
 *
 * @brief   Initialize deserializer interface for the specified index
 *
 * @param   des_index     Index of the deserializer to initialize
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Initialization successful
 * @retval  RET_FAILURE   Initialization failed
 *
 ************************************************************************/
RESULT init_des(int des_index)
{
	RESULT Status = RET_SUCCESS;


	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected for des_index %d\r\n",
			__func__, des_index);
		return XST_INVALID_PARAM;
	}

	desInterface *active_des_array = get_active_des_array();

	if (!active_des_array) {
		TRACE(MS_FW_ERROR, "%s: No Des Array selected for des_index %d\r\n",
			__func__, des_index);
		return XST_INVALID_PARAM;
	}

	TRACE(MS_FW_INFO, "Initializing deserializer"
		"%d using FMC: %s with %s array\r\n",
			des_index, active_fmc->FmcName,
			(current_active_fmc == FMC_ID_0) ? "des_arr" : "max96716_des_arr");

	Status = InitIIC();
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: PS I2C Init Failed for des_index %d (err=%d)\r\n",
			__func__, des_index, Status);
		return Status;
	}

	Status = active_fmc->pIsiIsiFmcSetup(des_index);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: FMC Setup Failed for des_index %d (err=%d)\r\n",
			__func__, des_index, Status);
		return Status;
	}

	*I2C_INIT_STATUS_REG = 1;

	Status = active_fmc->pIsiDeserSetup(&active_des_array[des_index]);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: Deserializer Setup"
			" Failed for des_index %d (err=%d)\r\n",
			__func__, des_index, Status);
		return Status;
	}

	return RET_SUCCESS;
}

/**************************************************************************
 *          start_sensor
 *
 * @brief   Start streaming on the specified sensor pipeline
 *
 * @param   in_pipe       Input pipeline index to start streaming
 *
 * @return  None
 *
 ************************************************************************/
void start_sensor(int in_pipe)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected"
			"for sensor start on pipeline %d\r\n",
			__func__, in_pipe);
		return;
	}

	TRACE(MS_FW_INFO, "enter %s on pipeline"
		"%d using FMC: %s\r\n",
		__func__, in_pipe, active_fmc->FmcName);

	if ((active_fmc->sensor_array[in_pipe])->sensor_state == in_deinit) {
		TRACE(MS_FW_INFO, "%s: Sensor %s on"
			"Pipeline-0x%x , cannot stream"
			"as it is not initialized\r\n",
			__func__,
			(active_fmc->sensor_array[in_pipe])->name,
			in_pipe);
	} else if ((active_fmc->sensor_array[in_pipe])->sensor_state == in_running) {
		TRACE(MS_FW_INFO, "%s: Sensor %s on Pipeline-0x%x ,Already streaming \r\n",
				__func__, (active_fmc->sensor_array[in_pipe])->name, in_pipe);
	} else {
		/*
		 * 1) We reach here after init time,
		 * 2) We reach here after previous stop
		 */
		int ret = (active_fmc->sensor_array[in_pipe])->
				stream_on(active_fmc->sensor_array[in_pipe]);

		if (ret != RET_SUCCESS) {
			TRACE(MS_FW_ERROR, "%s: stream_on failed for pipe %d (error=%d)\n",
					__func__, in_pipe, ret);
			return;
		}
		(active_fmc->sensor_array[in_pipe])->sensor_state = in_running;

		TRACE(MS_FW_INFO, "Streaming Started on Sensor %s on Pipeline-0x%x...\r\n",
				(active_fmc->sensor_array[in_pipe])->name, in_pipe);
	}
}

/**************************************************************************
 *          access_iic_read
 *
 * @brief   Read data from I2C slave device register
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   slave_addr    Slave device address
 * @param   addr          Register address to read from
 * @param   regWidth      Register address width in bytes
 * @param   pValue        Pointer to store read value
 * @param   dataWidth     Data width in bytes
 *
 * @return  Return the result of the function call.
 *
 ************************************************************************/
int access_iic_read(u8 i2cBusId, u8 slave_addr, uint16_t addr,
		uint8_t regWidth, uint8_t *pValue, uint8_t dataWidth)
{
	return HalReadI2CReg(i2cBusId, slave_addr, addr, regWidth, pValue, dataWidth);
}

/**************************************************************************
 *          access_iic_write
 *
 * @brief   Write data to I2C slave device register
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   slave_addr    Slave device address
 * @param   addr          Register address to write to
 * @param   regWidth      Register address width in bytes
 * @param   value         Value to write
 * @param   dataWidth     Data width in bytes
 *
 * @return  Return the result of the function call.
 *
 ************************************************************************/
int access_iic_write(u8 i2cBusId, u8 slave_addr, uint16_t addr,
		uint8_t regWidth, uint8_t *pValue, uint8_t dataWidth)
{
	return HalWriteI2CReg(i2cBusId, slave_addr, addr,
		regWidth, pValue, dataWidth);
}


/**************************************************************************
 *          init_iic_access
 *
 * @brief   Initialize I2C access interface for the specified pipeline
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   in_pipe       Input pipeline index
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Initialization successful
 * @retval  RET_OUTOFMEM  Memory allocation failed
 * @retval  RET_FAILURE   No active FMC selected
 *
 ************************************************************************/
RESULT init_iic_access(u8 i2cBusId, int in_pipe)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected"
			" for IIC access init on pipe %d\r\n",
			__func__, in_pipe);
		return RET_FAILURE;
	}

	TRACE(MS_FW_INFO, "Enter %s for pipe %d"
		"using FMC: %s\r\n",
		__func__, in_pipe, active_fmc->FmcName);

	accessiic_handle[in_pipe] =
		(struct accessIIC *)osMalloc(
			sizeof(struct accessIIC));
	if (!accessiic_handle[in_pipe]) {
		TRACE(MS_FW_ERROR, "%s: osMalloc failed for accessIIC (pipe=%d)\n",
			__func__, in_pipe);
		return RET_OUTOFMEM;
	}
	
	active_fmc->accessiic_array[in_pipe] = accessiic_handle[in_pipe];
	active_fmc->accessiic_array[in_pipe]->i2cBusId = i2cBusId;
	active_fmc->accessiic_array[in_pipe]->readIIC  = access_iic_read;
	active_fmc->accessiic_array[in_pipe]->writeIIC = access_iic_write;

	return RET_SUCCESS;
}

/**************************************************************************
 *          init_sensor
 *
 * @brief   Initialize sensor driver for the specified
 *          pipeline and deserializer
 *
 * @param   in_pipe       Input pipeline index
 * @param   des_index     Deserializer index
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS    Initialization successful
 * @retval  RET_OUTOFMEM   Memory allocation failed
 * @retval  RET_FAILURE    No active FMC or Des Array selected
 *
 ************************************************************************/
RESULT init_sensor(int in_pipe, int des_index, int sensor_address)
{
	RESULT Status = RET_SUCCESS;
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s :No FMC selected"
			"for sensor init on pipe %d\r\n",
			__func__, in_pipe);
		return RET_FAILURE;
	}

	desInterface *active_des_array = get_active_des_array();

	if (!active_des_array) {
		TRACE(MS_FW_ERROR, "%s :No Des Array selected"
			"for sensor init on pipe %d\r\n",
			__func__, in_pipe);
		return RET_FAILURE;
	}

	TRACE(MS_FW_INFO, "enter %s for pipe %d using FMC: %s with %s array\r\n",
			__func__, in_pipe, active_fmc->FmcName,
			(current_active_fmc == FMC_ID_0) ? "des_arr" : "max96716_des_arr");

	sensor_handle[in_pipe] = (struct sensor_driver *)osMalloc
		(sizeof(struct sensor_driver));

	if (sensor_handle[in_pipe] == NULL) {
		TRACE(MS_FW_ERROR, "%s: osMalloc failed for sensor_driver (pipe=%d)\n",
			__func__, in_pipe);
		return RET_OUTOFMEM;
	}
	active_fmc->sensor_array[in_pipe] = sensor_handle[in_pipe];
	active_fmc->sensor_array[in_pipe]->pipe_no = in_pipe;


	if (active_des_array[des_index].link_type == NO_LINK) {
		TRACE(MS_FW_WARNING, "%s: Sensor Not Connected"
			",please connect the sensor\r\n",
			__func__);
		Status = XST_DEVICE_NOT_FOUND;
		goto cleanup_sensor;

	} else if (active_des_array[des_index].link_type == LINK_A) {
		TRACE(MS_FW_ALWAYS, "%s: Programming Link-A Device \r\n",
			__func__);
		active_fmc->serializer_array[in_pipe]->alias_addr =
			(active_des_array[des_index].link_a.serializer_alias_addr);
		active_fmc->serializer_array[in_pipe]->bus_num =
		GetI2cBusIdForDes(des_index);
		if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes failed for des_index %d\n",
				__func__, des_index);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		Status = (active_fmc->serializer_array[in_pipe])->
			init_serializer(active_fmc->serializer_array[in_pipe]);
		if (Status != RET_SUCCESS){
			TRACE(MS_FW_ERROR,
				"%s: init_serializer"
				"failed pipe %d (err=%d)\n",
				__func__, in_pipe, Status);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
			(active_des_array[des_index].link_a.sensor_alias_addr);

	} else if (active_des_array[des_index].link_type == LINK_B) {
		TRACE(MS_FW_ALWAYS, "%s: Programming Link-B Device \r\n",
			__func__);
		active_fmc->serializer_array[in_pipe]->alias_addr =
			(active_des_array[des_index].link_b.serializer_alias_addr);
		active_fmc->serializer_array[in_pipe]->bus_num =
		GetI2cBusIdForDes(des_index);
		if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes"
				"failed for des_index %d\n",
				__func__, des_index);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		Status = (active_fmc->serializer_array[in_pipe])->
			init_serializer(active_fmc->serializer_array[in_pipe]);
		if (Status != RET_SUCCESS){
			TRACE(MS_FW_ERROR,
				"%s: init_serializer"
				"failed pipe %d (err=%d)\n",
				__func__, in_pipe, Status);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
			(active_des_array[des_index].link_b.sensor_alias_addr);

	} else if (active_des_array[des_index].link_type == LINK_REVERSE_SPLITTER) {
		if ((in_pipe % 2) == 0) {
			TRACE(MS_FW_ALWAYS, "%s: Programming Link-A Device \r\n", __func__);
			active_fmc->serializer_array[in_pipe]->alias_addr =
				(active_des_array[des_index].link_a.serializer_alias_addr);
			active_fmc->serializer_array[in_pipe]->bus_num =
				GetI2cBusIdForDes(des_index);
			if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
				TRACE(MS_FW_ERROR,
					"%s: GetI2cBusIdForDes failed for des_index %d\n",
					__func__, des_index);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			Status = (active_fmc->serializer_array[in_pipe])->
			init_serializer(active_fmc->serializer_array[in_pipe]);
			if (Status != RET_SUCCESS){
				TRACE(MS_FW_ERROR,
					"%s: init_serializer"
					"failed pipe %d (err=%d)\n",
					__func__, in_pipe, Status);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
				(active_des_array[des_index].link_a.sensor_alias_addr);
		} else {
			TRACE(MS_FW_ALWAYS, "%s: Programming Link-B Device \r\n", __func__);
			active_fmc->serializer_array[in_pipe]->alias_addr =
				(active_des_array[des_index].link_b.serializer_alias_addr);
			active_fmc->serializer_array[in_pipe]->bus_num =
				GetI2cBusIdForDes(des_index);
			if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
				TRACE(MS_FW_ERROR, "%s: GetI2cBusIdForDes failed for des_index %d\n",
				__func__, des_index);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			Status = (active_fmc->serializer_array[in_pipe])->
				init_serializer(active_fmc->serializer_array[in_pipe]);
			if (Status != RET_SUCCESS){
				TRACE(MS_FW_ERROR,
					"%s: init_serializer"
					"failed pipe %d (err=%d)\n",
					__func__, in_pipe, Status);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
				(active_des_array[des_index].link_b.sensor_alias_addr);
		}
	}

	if (active_des_array[des_index].link_type != NO_LINK) {
		TRACE(MS_FW_INFO, "Sensor address passed: %x\n", sensor_address);
		u16 reg_addr;
		u8 ser_addr = active_fmc->serializer_array[in_pipe]->alias_addr >> 1;
		u8 i2cBusId = GetI2cBusIdForDes(des_index);

		if (i2cBusId == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes failed for des_index %d\n",
				__func__, des_index);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		u8 wr_data[2] = {0};
		u8 read_data[2] = {0};

		reg_addr = I2C_2;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_2 pipe=%d (err=%d)\n",
					__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "before I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_3 pipe=%d (err=%d)\n",
					__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "before I2C_3 :%x\r\n", read_data[0]);

		reg_addr = I2C_2;
		wr_data[0] = active_fmc->sensor_array[in_pipe]->sensor_alias_addr;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C write failed reg=I2C_2 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}

		reg_addr = I2C_3;
		wr_data[0] = sensor_address;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C write failed reg=I2C_3 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}

		reg_addr = I2C_2;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_2 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "After I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_3 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "After I2C_3 :%x\r\n", read_data[0]);
	}
	return RET_SUCCESS;

cleanup_sensor:
	osFree(sensor_handle[in_pipe]);
	sensor_handle[in_pipe] = NULL;
	active_fmc->sensor_array[in_pipe] = NULL;
	if (accessiic_handle[in_pipe]) {
		osFree(accessiic_handle[in_pipe]);
		accessiic_handle[in_pipe] = NULL;
		active_fmc->accessiic_array[in_pipe] = NULL;
	}
	return Status;
}

/**************************************************************************
 *          stop_sensor
 *
 * @brief   Stop streaming on the specified sensor pipeline
 *
 * @param   in_pipe       Input pipeline index to stop streaming
 *
 * @return  None
 *
 ************************************************************************/
void stop_sensor(int in_pipe)
{
	(void)in_pipe; // Suppress unused parameter warning
	TRACE(MS_FW_ALWAYS, "Skipping %s\n", __func__);
	return;
	/* TODO: stop_sensor is currently disabled. Remove early return to enable. */
}

/**************************************************************************
 *          sensor_status
 *
 * @brief   Display status information for the specified sensor
 *
 * @param   index         Sensor index to display status for
 *
 * @return  None
 *
 ************************************************************************/
void sensor_status(int index)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "Error: No active FMC selected for %s\r\n", __func__);
		return;
	}

	if (((active_fmc->sensor_array[index])->sensor_state == in_stop) ||
		((active_fmc->sensor_array[index])->sensor_state == in_running)) {

	} else {
		/*Yet to Impliment*/
	}
}

/**************************************************************************
 *          des_stats
 *
 * @brief   Display deserializer statistics for the specified index
 *
 * @param   des_index     Deserializer index to display statistics for
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 ************************************************************************/
RESULT des_stats(int des_index)
{
	u8 des_addr, i2cBusId;
	u8 pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	RESULT Status = RET_SUCCESS;
	desInterface *active_des_array = get_active_des_array();

	if (!active_des_array) {
		TRACE(MS_FW_ERROR, "%s: No Des Array selected for des_index %d\r\n",
			__func__, des_index);
		return RET_FAILURE;
	}

	if (active_des_array[des_index].des_state == in_running) {
		des_addr = (active_des_array[des_index].des_alias_addr) >> 1;
		i2cBusId = GetI2cBusIdForDes(active_des_array[des_index].Port_DES_index);
		if (i2cBusId == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes failed\r for des_index %d\n",
				__func__, des_index);
			return RET_FAILURE;
		}

		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_X_STATUS, 2, &pipe_x_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_X_STATUS, des_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_Y_STATUS, 2, &pipe_y_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_Y_STATUS, des_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_Z_STATUS, 2, &pipe_z_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_Z_STATUS, des_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_U_STATUS, 2, &pipe_u_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_U_STATUS, des_index, Status);
			return Status;
		}
		TRACE(MS_FW_INFO,
			"Des[%x] Pipe Status :"
				" pipe_x =\r 0x%x, pipe_y = 0x%x, pipe_z = 0x%x, pipe_u = 0x%x\r\n",
				active_des_array[des_index].Port_DES_index + 1, pipe_x_s,
				pipe_y_s, pipe_z_s, pipe_u_s);
	}

	return Status;
}

/**************************************************************************
 *          ser_stats
 *
 * @brief   Display serializer statistics for the specified index
 *
 * @param   ser_index     Serializer index to display statistics for
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 ************************************************************************/
RESULT ser_stats(int ser_index)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "Error: No active FMC selected for %s\r\n", __func__);
		return RET_FAILURE;
	}

	u8 ser_addr, i2cBusId;
	u8 pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	RESULT Status = RET_SUCCESS;

	if (active_fmc->serializer_array[ser_index]->ser_state == in_running) {
		ser_addr = (active_fmc->serializer_array[ser_index]->alias_addr) >> 1;
		i2cBusId = active_fmc->serializer_array[ser_index]->bus_num;

		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_X_STATUS, 2, &pipe_x_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_X_STATUS, ser_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_Y_STATUS, 2, &pipe_y_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_Y_STATUS, ser_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_Z_STATUS, 2, &pipe_z_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_Z_STATUS, ser_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_U_STATUS, 2, &pipe_u_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_U_STATUS, ser_index, Status);
			return Status;
		}
		TRACE(MS_FW_INFO, "ser[%x]: Px:0x%x, Py: 0x%x, Pz: 0x%x, Pu: 0x%x\n",
				ser_index + 1, pipe_x_s, pipe_y_s, pipe_z_s, pipe_u_s);
	}

	return Status;
}

/**************************************************************************
 *          Fmc_Sensor_Statustask
 *
 * @brief   Display status for all deserializers and serializers in the system
 *
 * @return  None
 *
 ************************************************************************/
void Fmc_Sensor_Statustask(void)
{
	int des_index, ser_index;

	for (des_index = 0; des_index < DS_MAX; des_index++)
		des_stats(des_index);
	for (ser_index = 0; ser_index < IN_PIPE_LAST; ser_index++)
		ser_stats(ser_index);
}
