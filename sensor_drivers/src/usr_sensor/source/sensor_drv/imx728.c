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
 ****************************************************************************/

#include <isi/isi_fmc.h>
#include "isi/isi.h"
#include "isi/isi_iss.h"
#include "sensor_drv/imx728_priv.h"

//#define IMX728_WITH_VTS	(1)
#define IMX728_WITH_HTS		(1)

CREATE_TRACER(IMX728_INFO, "IMX728: ", INFO, 1);
CREATE_TRACER(IMX728_WARN, "IMX728: ", WARNING, 1);
CREATE_TRACER(IMX728_ERROR, "IMX728: ", ERROR, 1);
CREATE_TRACER(IMX728_DEBUG, "IMX728: ", INFO, 1);

IsiSensorMode_t pimx728_mode_info[] = {
	{
	    .index     = 0,
	    .size = {
		.boundsWidth = 3840,
		.boundsHeight = 2160,
		.top = 0,
		.left = 0,
		.width = 3840,
		.height = 2160,
	    },
	    .aeInfo = {
		.intTimeRange[0].max = 0.03 * 1000000,
		.intTimeRange[0].min = 0.00001388 * 1 * 1000000,
		.aGainRange[0] = {1.0, 1.0, 1.0},
		.dGainRange[0] = {1.0, 1.0, 1.0f},
		.intTimeDelayFrame = 2,
		.gainDelayFrame = 2,
	    },
	    .fps = 30 * ISI_FPS_QUANTIZE,
	    .hdrMode = ISI_SENSOR_MODE_HDR_NATIVE,
	    .nativeMode = ISI_SENSOR_NATIVE_4DOL,
	    .bitWidth = 12,
	    .compress.enable = 1,
	    .compress.xBit = 24,
	    .compress.yBit = 12,
	    .bayerPattern = ISI_BPAT_RGGB,
	    .afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	    .dataType = ISI_MODE_BAYER,
	    .mipiLane = ISI_MIPI_4LANES,
	}
};

int imx728_mode_num = ARRAY_SIZE(pimx728_mode_info);

/*****************************************************************************
 * IMX728_IsiReadRegIss
 *
 * @brief   Read register value from IMX728 sensor via I2C.
 *
 * @param   handle      Sensor instance handle
 * @param   addr        Register address to read from
 * @param   pValue      Pointer to store the read value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *****************************************************************************/
static RESULT IMX728_IsiReadRegIss(IsiSensorHandle_t handle,
				    const uint16_t addr, uint16_t *pValue)
{
	RESULT Status = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *)handle;
	u8 slave_addr;
	uint8_t read_val = 0;

	if (pIMX728Ctx == NULL || pValue == NULL) {
		TRACE(IMX728_ERROR,
		      "%s: Invalid handle or null pointer!\n", __func__);
		return RET_NULL_POINTER;
	}

	osMutexLock(&pIMX728Ctx->registerLock);
	memset(pValue, 0, sizeof(uint16_t));
	if (!pIMX728Ctx->regAccessEnable) {
		TRACE(IMX728_ERROR,
		      "%s: register cannot be read!\n", __func__);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_WRONG_STATE;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(IMX728_ERROR, "%s: No FMC selected\n", __func__);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_INVALID_PARM;
	}

	slave_addr = (active_fmc->sensor_array[pIMX728Ctx->sensorDevId]
		      ->sensor_alias_addr) >> 1;

	Status = active_fmc->accessiic_array[pIMX728Ctx->sensorDevId]
		 ->readIIC(pIMX728Ctx->i2cId, slave_addr, addr, 0x2,
			   &read_val, 1);

	if (Status != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: I2C read failed at addr 0x%04x (err=%d)\n",
		      __func__, addr, Status);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_FAILURE;
	}
	*pValue = (uint16_t)(read_val & 0xff);

	osMutexUnlock(&pIMX728Ctx->registerLock);

	return RET_SUCCESS;
}

/*****************************************************************************
 * IMX728_IsiWriteRegIss
 *
 * @brief   Write register value to IMX728 sensor via I2C.
 *
 * @param   handle      Sensor instance handle
 * @param   addr        Register address to write to
 * @param   value       Value to write to the register
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *****************************************************************************/
static RESULT IMX728_IsiWriteRegIss(IsiSensorHandle_t handle,
				    const uint16_t addr, const uint16_t value)
{
	RESULT Status = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *)handle;
	u8 slave_addr;
	u8 write_val;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	osMutexLock(&pIMX728Ctx->registerLock);
	if (!pIMX728Ctx->regAccessEnable) {
		TRACE(IMX728_ERROR,
		      "%s: register cannot be written!\n", __func__);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_WRONG_STATE;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(IMX728_ERROR, "%s: No FMC selected\n", __func__);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_INVALID_PARM;
	}

	slave_addr = (active_fmc->sensor_array[pIMX728Ctx->sensorDevId]
		      ->sensor_alias_addr) >> 1;

	write_val = (u8)value;

	Status = active_fmc->accessiic_array[pIMX728Ctx->sensorDevId]
		 ->writeIIC(pIMX728Ctx->i2cId, slave_addr, addr, 0x2,
			    &write_val, 1);
	if (Status != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: I2C write failed at addr 0x%04x (err=%d)\n",
		      __func__, addr, Status);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_FAILURE;
	}
	if ((addr == IMX728_REG_STREAM) && (value == 0xff))
		pIMX728Ctx->regAccessEnable = BOOL_FALSE;

	osMutexUnlock(&pIMX728Ctx->registerLock);

	return RET_SUCCESS;
}

#ifdef ENABLE_I2C_GROUPING
/*****************************************************************************
 * IMX728_IsiWriteRegGroupIss
 *
 * @brief   Write register value to IMX728 sensor via I2C.
 *
 * @param   handle      Sensor instance handle
 * @param   addr        Register address to write to
 * @param   value       Value to write to the register
 * @param   datacount   Number of bytes to write to the continuous registers
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *****************************************************************************/
static RESULT IMX728_IsiWriteRegGroupIss(IsiSensorHandle_t handle,
					  const uint16_t addr, uint8_t *value,
					  uint8_t datacount)
{
	RESULT Status = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *)handle;
	u8 slave_addr;

	if (pIMX728Ctx == NULL || value == NULL) {
		TRACE(IMX728_ERROR,
		      "%s: Invalid handle or null pointer!\n", __func__);
		return RET_NULL_POINTER;
	}

	if (datacount > 4U)
		return RET_FAILURE;

	osMutexLock(&pIMX728Ctx->registerLock);
	if (!pIMX728Ctx->regAccessEnable) {
		TRACE(IMX728_ERROR,
		      "%s: register cannot be written!\n", __func__);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_WRONG_STATE;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(IMX728_ERROR, "%s: No FMC selected\n", __func__);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_INVALID_PARM;
	}

	slave_addr = (active_fmc->sensor_array[pIMX728Ctx->sensorDevId]
			->sensor_alias_addr) >> 1;

	Status = active_fmc->accessiic_array[pIMX728Ctx->sensorDevId]
		 ->writeIIC(pIMX728Ctx->i2cId, slave_addr, addr, 0x2,
			    value, datacount);

	if (Status != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: I2C group write failed at addr 0x%04x (err=%d)\n",
		      __func__, addr, Status);
		osMutexUnlock(&pIMX728Ctx->registerLock);
		return RET_FAILURE;
	}
	if ((addr == IMX728_REG_STREAM) && (value[0] == 0xff))
		pIMX728Ctx->regAccessEnable = BOOL_FALSE;

	osMutexUnlock(&pIMX728Ctx->registerLock);

	return RET_SUCCESS;
}
#endif

/*****************************************************************************
 *          IMX728_IsiUpdateRegIss
 *
 * @brief   Update specific bits in a register using mask and value.
 *
 * @param   handle      Sensor instance handle
 * @param   addr        Register address to update
 * @param   mask        Bit mask for the update
 * @param   value       Value to write (masked)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_IsiUpdateRegIss(IsiSensorHandle_t handle,
				     const uint16_t addr,
				     const uint8_t mask, const uint8_t value)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val16 = 0;

	result = IMX728_IsiReadRegIss(handle, addr, &reg_val16);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to read reg - %x\n", __func__, addr);
		return RET_FAILURE;
	}

	uint8_t reg_val = (uint8_t)(reg_val16 & 0xFF);

	reg_val = (mask & value) | (~mask & reg_val);

	result = IMX728_IsiWriteRegIss(handle, addr, reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to update reg - %x\n", __func__, addr);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX728_GetRemapMode
 *
 * @brief   Get the current remap mode from the sensor.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the remap mode.
 * @retval  RemapMode_e enumeration value
 * @retval  IMX728_REMAP_MODE_INVALID on error
 *
 *****************************************************************************/
IMX728_RemapMode_e IMX728_GetRemapMode(IsiSensorHandle_t handle)
{
	uint16_t remap_mode = 0;

	RESULT result = RET_SUCCESS;

	result = IMX728_IsiReadRegIss(handle, IMX728_REG_REGMAP,
				      &remap_mode);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to read remap mode!\n", __func__);
		return IMX728_REMAP_MODE_INVALID;
	}

	return (IMX728_RemapMode_e)remap_mode;
}

/*****************************************************************************
 *          IMX728_GetSensorState
 *
 * @brief   Get the current sensor state.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the sensor state.
 * @retval  SensorState_e enumeration value
 * @retval  IMX728_SENSOR_STATE_INVALID on error
 *
 *****************************************************************************/
IMX728_SensorState_e IMX728_GetSensorState(IsiSensorHandle_t handle)
{
	uint16_t sensor_state = 0;

	if (IMX728_IsiReadRegIss(handle, IMX728_REG_STATE,
				(uint16_t *)&sensor_state) != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to read sensor state!\n", __func__);
		return IMX728_SENSOR_STATE_INVALID;
	}

	return (IMX728_SensorState_e)sensor_state;
}

/*****************************************************************************
 *          IMX728_sensor_framecount
 *
 * @brief   Read and update the global sensor frame count.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 *****************************************************************************/
int imx728_sensor_framecount(IsiSensorHandle_t handle)
{
	uint16_t read_buf[4] = {0};
	int Status = RET_SUCCESS;

	Status = IMX728_IsiReadRegIss(handle, 0x8280, &read_buf[0]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	Status = IMX728_IsiReadRegIss(handle, 0x8281, &read_buf[1]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	Status = IMX728_IsiReadRegIss(handle, 0x8282, &read_buf[2]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}
	Status = IMX728_IsiReadRegIss(handle, 0x8283, &read_buf[3]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	g_Sensor_frame_count = (read_buf[3] << 24) | (read_buf[2] << 16) |
			       (read_buf[1] << 8) | (read_buf[0]);

	return Status;
}

/*****************************************************************************
 *          IMX728_IsiCreateIss
 *
 * @brief   Create Sensor Context for the given config
 *
 * @param   pConfig     Given Sensor Config
 * @param   pHandle     Return the Sensor Ctx
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig,
				  IsiSensorHandle_t *pHandle)
{
	RESULT result = RET_SUCCESS;
	uint32_t desId = 0, pipeId = 0;
	uint8_t busId;
	IMX728_Context_t *pIMX728Ctx;

	pIMX728Ctx = (IMX728_Context_t *) osMalloc(sizeof(IMX728_Context_t));
	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR,
		      "%s: Can't allocate IMX728 context\n", __func__);
		return RET_OUTOFMEM;
	}

	MEMSET(pIMX728Ctx, 0, sizeof(IMX728_Context_t));

	pIMX728Ctx->isiCtx.pSensor	= pConfig->pSensor;
	pIMX728Ctx->configured		= BOOL_FALSE;
	pIMX728Ctx->streaming		= BOOL_FALSE;
	pIMX728Ctx->testPattern		= BOOL_FALSE;
	pIMX728Ctx->sensorMode.index	= 0;
	pIMX728Ctx->i2cId		= 0;
	pIMX728Ctx->sensorDevId		= pConfig->halDevID;
	pIMX728Ctx->regAccessEnable	= BOOL_TRUE;
	osMutexInit(&pIMX728Ctx->registerLock);

	*pHandle = (IsiSensorHandle_t) pIMX728Ctx;
	pipeId = pIMX728Ctx->sensorDevId;

	if (pIMX728Ctx->sensorDevId >= IN_PIPE_LAST) {
		TRACE(IMX728_ERROR,
		      "%s: sensor device ID %d is not supported!\n",
		      __func__, pIMX728Ctx->sensorDevId);
		osFree(pIMX728Ctx);
		*pHandle = NULL;
		return RET_UNSUPPORT_ID;
	}

	desId = MAPPING_INPIPE_TO_DES_ID(pipeId);

	pIMX728Ctx->i2cId = GetI2cBusIdForDes(desId);
	busId = (uint8_t)pIMX728Ctx->i2cId;

	/* Validate I2C bus ID */
	if (busId == INVALID_I2C_BUS_ID) {
		TRACE(IMX728_ERROR,
		      "%s: Invalid I2C bus ID for desId %d\n",
		      __func__, desId);
		osFree(pIMX728Ctx);
		return RET_FAILURE;
	}
	TRACE(IMX728_INFO,
	      "%s: desId: %d, pipeId: %d, i2cBusId:%d\r\n",
	      __func__, desId, pipeId, busId);

	result = init_iic_access(busId, pipeId);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: init_iic_access failed for pipe %d\n",
		      __func__, pipeId);
		osFree(pIMX728Ctx);
		return result;
	}
	result = init_des(desId);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: init_des failed for desId %d\n", __func__, desId);
		osFree(pIMX728Ctx);
		return result;
	}
	result = init_sensor(pipeId, desId, SENSOR_IMX728_ADDRESS);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: init_sensor failed for pipe %d\n", __func__, pipeId);
		osFree(pIMX728Ctx);
		return result;
	}

	return result;
}

/*****************************************************************************
 *          IMX728_IsiEnumModeIss
 *
 * @brief   query sensor info.
 *
 * @param   handle                  sensor instance handle
 * @param   pEnumMode               sensor query mode
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiEnumModeIss(IsiSensorHandle_t handle,
				    IsiSensorEnumMode_t *pEnumMode)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	uint32_t mode_idx;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pEnumMode->index >= (ARRAY_SIZE(pimx728_mode_info)))
		return RET_OUTOFRANGE;

	for (mode_idx = 0; mode_idx < (ARRAY_SIZE(pimx728_mode_info)); mode_idx++) {
		if (pimx728_mode_info[mode_idx].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &pimx728_mode_info[mode_idx],
			       sizeof(IsiSensorMode_t));
			TRACE(IMX728_INFO, "%s (exit)\n", __func__);
			return RET_SUCCESS;
		}
	}

	return RET_NOTSUPP;
}

/*****************************************************************************
 *          IMX728_SetClknStartup
 *
 * @brief   Set clock frequency during sensor startup.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_SetClknSteadyby(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (IMX728_GetSensorState(handle) != IMX728_SENSOR_STATE_SLEEP) {
		TRACE(IMX728_ERROR,
		      "%s:INCK can be set in Sleep State only.\n", __func__);
		return RET_FAILURE;
	}
	/* Set clock (24MHz, 0x1B20=0x18) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_INCK_FREQ,
				       FMC_CLK_HZ / 1000000);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set input clock frequency!\n", __func__);
		return RET_FAILURE;
	}
	/* Enable input clock set by I2C register write (0x1B1C=0x01) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_INCK_EN, 0x01);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't enable INCK frequency!\n", __func__);
		return RET_FAILURE;
	}
	/* Set to standby state (0x1B05=0xFF) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_SLEEP, 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't set to standby state!\n", __func__);
		return RET_FAILURE;
	}
	osSleep(30);
	/* Check if sensor is in standby state */
	if (IMX728_GetSensorState(handle) != IMX728_SENSOR_STATE_STANDBY) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't transition from Sleep to Standby state!\n",
		      __func__);
		return RET_FAILURE;
	}
	/* Set Remap Mode to Standby (0xFFFF=0x00) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_REGMAP,
				       IMX728_REMAP_MODE_STANDBY);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't write regmap mode!\n", __func__);
		return RET_FAILURE;
	}

	osSleep(1);
	TRACE(IMX728_INFO, "%s (exit) result = %d\n", __func__, result);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetRevisionIss
 *
 * @brief   This function reads the sensor revision register and returns it.
 *
 * @param   handle      sensor instance handle
 * @param   pValue   pointer to revision
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetRevisionIss(IsiSensorHandle_t handle,
				       uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val;
	uint32_t sensor_id;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	reg_val   = 0;
	result    = IMX728_IsiReadRegIss(handle, IMX728_REG_CHIP_ID1, &reg_val);
	sensor_id = ((reg_val & 0xff) << 8);

	reg_val   = 0;
	result    |= IMX728_IsiReadRegIss(handle, IMX728_REG_CHIP_ID0, &reg_val);
	sensor_id |= (reg_val & 0xff);

	*pValue = sensor_id;

	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiCheckConnectionIss
 *
 * @brief   Checks the connection to the camera sensor, if possible.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiCheckConnectionIss(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	pIMX728Ctx->regAccessEnable = BOOL_TRUE;

	TRACE(IMX728_INFO, "========= IMX728 Latest With 2A =========\n");

	result = IMX728_SetClknSteadyby(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_WARN, "%s: Powerup and Standby Failed! (err=%d)\n", __func__, result);
		// return result;
	}

	uint32_t sensor_id = 0;
	uint32_t correct_id = IMX728_SENSOR_ID;

	result = IMX728_IsiGetRevisionIss(handle, &sensor_id);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Read Sensor ID Error! (err=%d)\n", __func__, result);
		return result;
	}

	if ((correct_id & 0xFFFF) != (sensor_id & 0xFFFF)) {
		TRACE(IMX728_ERROR,
		      "%s: ChipID =0x%x sensor_id=%x error!\n",
		      __func__, correct_id, sensor_id);
		return RET_FAILURE;
	}

	TRACE(IMX728_INFO,
	      "%s ChipID = 0x%08x, sensor_id = 0x%08x, success!\n",
	      __func__, correct_id, sensor_id);
	TRACE(IMX728_INFO, "%s: check connection done\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX728_IsiSetFpsIss
 *
 * @brief   set Sensor Fps Config.
 *
 * @param   handle                  sensor instance handle
 * @param   fps                     Setfps
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX728_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
	RESULT result = RET_SUCCESS;
	uint16_t fps_value;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_WRONG_HANDLE;
	}

	if (IMX728_GetSensorState(handle) != IMX728_SENSOR_STATE_STANDBY) {
		TRACE(IMX728_ERROR,
		      "%s: FPS can be set in Standby State only.\n",
		      __func__);
		return RET_WRONG_STATE;
	}

	if (fps > pIMX728Ctx->maxFps) {
		TRACE(IMX728_ERROR,
		      "%s: FPS cannot be greater than %d.\n",
		      __func__, pIMX728Ctx->maxFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}

	if (fps < pIMX728Ctx->minFps) {
		TRACE(IMX728_ERROR,
		      "%s: FPS cannot be less than %d.\n",
		      __func__, pIMX728Ctx->minFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}

	fps /= ISI_FPS_QUANTIZE;

#ifdef IMX728_WITH_HTS
	TRACE(IMX728_INFO, "Set fps with hts\n");
	fps_value = (30 * 6000) / fps;

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_L,
				       (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set FPS_L! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_H,
				       (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set FPS_H! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_APL_L,
				       (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set FPS_APL_L! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_APL_H,
				       (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set FPS_APL_H! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
#endif

#ifdef IMX728_WITH_VTS
	TRACE(IMX728_INFO, "Set fps with vts\n");
	fps_value = (30 * 2400) / fps;

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_L,
				       (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set VTS_FPS_L! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_H,
				       (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set VTS_FPS_H! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_APL_L,
				       (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set VTS_FPS_APL_L! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_APL_H,
				       (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set VTS_FPS_APL_H! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
#endif

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}
/*****************************************************************************
 *          IMX728_IsiGetFpsIss
 *
 * @brief   Get Sensor Fps Config.
 *
 * @param   handle                   sensor instance handle
 * @param   pFps                     current fps
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX728_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps)
{
	RESULT result = RET_SUCCESS;
	uint16_t fps_reg = 0;
	uint16_t value = 0;

	/* 0x9630 */
	result = IMX728_IsiReadRegIss(handle, IMX728_REG_FPS_APL_L, &value);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Get FPS! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	fps_reg = value;

	/* 0x9631 */
	result = IMX728_IsiReadRegIss(handle, IMX728_REG_FPS_APL_H, &value);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Get FPS! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	fps_reg = fps_reg | (value << 8);
	*pFps = (30 * 6000) / fps_reg;
	*pFps *= ISI_FPS_QUANTIZE;

	imx728_sensor_framecount(handle);
	Fmc_Sensor_Statustask();

	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX728_InitFixedExposure
 *
 * @brief   Initialize fixed exposure settings for HDR mode.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_InitFixedExposure(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	/* 10ms, 0x2710 */
	unsigned int exposure_sp1_sp2_us = 10000;
	/* 56us, 0x38 */
	unsigned int exposure_sp1vs_us = 56;
	unsigned int sp1h_gain = 240;	/* 0xf0 */
	unsigned int sp1l_gain = 75;	/* 0x4b */
	unsigned int sp1ec_gain = 21;	/* 0x15 */
	unsigned int sp2_gain = 33;	/* 0x21 */
	unsigned int sp1vs_gain = 84;	/* 0x54 */

	/* Exposure time of SP1 */
	result  = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_LL,
				       (exposure_sp1_sp2_us >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_LH,
				       (exposure_sp1_sp2_us >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_HL,
				       (exposure_sp1_sp2_us >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_HH,
				       (exposure_sp1_sp2_us >> 24) & 0xFF);
	/* Exposure time of SP2 */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_LL,
				       (exposure_sp1_sp2_us >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_LH,
				       (exposure_sp1_sp2_us >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_HL,
				       (exposure_sp1_sp2_us >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_HH,
				       (exposure_sp1_sp2_us >> 24) & 0xFF);
	/* Exposure time of SP1 VS */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_LL,
				       (exposure_sp1vs_us >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_LH,
				       (exposure_sp1vs_us >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_HL,
				       (exposure_sp1vs_us >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_HH,
				       (exposure_sp1vs_us >> 24) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set fixed exposure values!\n", __func__);
		return RET_FAILURE;
	}

	/* Analog Gain of SP1 H */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1H_L,
				       (sp1h_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1H_M,
				       (sp1h_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1H_H,
					0b00000001, (sp1h_gain >> 16) & 0xFF);
	/* Analog Gain of SP1 L */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1L_L,
				       (sp1l_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1L_M,
				       (sp1l_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1L_H,
					0b00000001, (sp1l_gain >> 16) & 0xFF);
	/* Analog Gain of SP1 EC */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1EC_L,
				       (sp1ec_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1EC_M,
				       (sp1ec_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1EC_H,
					0b00000001, (sp1ec_gain >> 16) & 0xFF);
	/* Analog Gain of SP2 */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP2_L,
				       (sp2_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP2_M,
				       (sp2_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP2_H,
					0b00000001, (sp2_gain >> 16) & 0xFF);
	/* Analog Gain of SP1 VS */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1VS_L,
				       (sp1vs_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1VS_M,
				       (sp1vs_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1VS_H,
					0b00000001, (sp1vs_gain >> 16) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set fixed gain values!\n", __func__);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX728_HDRConfigure
 *
 * @brief   Configure HDR settings with control points.
 *
 * @param   handle      Sensor instance handle
 * @param   points      Pointer to HDR control points structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_HDRConfigure(IsiSensorHandle_t handle,
					  struct imx728_ctrl_point *points)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	if (pIMX728Ctx == NULL || points == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	uint32_t hdr_norm_x0;
	uint32_t hdr_norm_x1;
	uint16_t hdr_norm_y0;
	uint16_t hdr_norm_y1;
	int ctrl_pt_idx;

	/* HDR 24bit settings */
	hdr_norm_x0 = 0x3000;
	hdr_norm_x1 = 0x5000;

	hdr_norm_y0 = 0x0;
	hdr_norm_y1 = 0xe000;

	/* Enable normal gain */
	/* NORM_GAIN_TH_LOW_X0 (0x9c60-0x9c63) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_LL,
				       (hdr_norm_x0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_LH,
					(hdr_norm_x0 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_HL,
					(hdr_norm_x0 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_HH,
					(hdr_norm_x0 >> 24) & 0xFF);
	/* NORM_GAIN_TH_LOW_X1 (0x9c6c-0x9c6f) */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_LL,
					(hdr_norm_x0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_LH,
					(hdr_norm_x0 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_HL,
					(hdr_norm_x0 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_HH,
					(hdr_norm_x0 >> 24) & 0xFF);
	/* NORM_GAIN_TH_HIGH_X0 (0x9c64-0x9c67) */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_LL,
					(hdr_norm_x1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_LH,
					(hdr_norm_x1 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_HL,
					(hdr_norm_x1 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_HH,
					(hdr_norm_x1 >> 24) & 0xFF);
	/* NORM_GAIN_TH_HIGH_X1 (0x9c70-0x9c73) */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_LL,
					(hdr_norm_x1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_LH,
					(hdr_norm_x1 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_HL,
					(hdr_norm_x1 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_HH,
					(hdr_norm_x1 >> 24) & 0xFF);
	/* NORM_GAIN_TH_LOW_Y0 (0x9c68-0x9c69) */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y0_L,
					(hdr_norm_y0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y0_H,
					(hdr_norm_y0 >> 8) & 0xFF);
	/* NORM_GAIN_TH_LOW_Y1 (0x9c74-0x9c75) */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y0_L,
					(hdr_norm_y0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y0_H,
					(hdr_norm_y0 >> 8) & 0xFF);
	/* NORM_GAIN_TH_HIGH_Y0 (0x9c6a-0x9c6b) */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y1_L,
					(hdr_norm_y1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y1_H,
					(hdr_norm_y1 >> 8) & 0xFF);
	/* NORM_GAIN_TH_HIGH_Y1 (0x9c76-0x9c77) */
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y1_L,
					(hdr_norm_y1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y1_H,
					(hdr_norm_y1 >> 8) & 0xFF);

	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed when setting HDR Normalization gains!\n",
		      __func__);
		return RET_FAILURE;
	}

	/* configure PWL curve */
	for (ctrl_pt_idx = 0; ctrl_pt_idx < 17; ctrl_pt_idx++) {
		result = IMX728_IsiWriteRegIss(handle,
				IMX728_REG_CTRL_POINT_X(ctrl_pt_idx),
				(points->x >> 0) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,
				IMX728_REG_CTRL_POINT_X(ctrl_pt_idx) + 1,
				(points->x >> 8) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,
				IMX728_REG_CTRL_POINT_X(ctrl_pt_idx) + 2,
				(points->x >> 16) & 0xFF);

		result |= IMX728_IsiWriteRegIss(handle,
				IMX728_REG_CTRL_POINT_Y(ctrl_pt_idx),
				(points->y >> 0) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,
				IMX728_REG_CTRL_POINT_Y(ctrl_pt_idx) + 1,
				(points->y >> 8) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,
				IMX728_REG_CTRL_POINT_Y(ctrl_pt_idx) + 2,
				(points->y >> 16) & 0xFF);

		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR,
			      "%s: Failed to write control point %d\n",
			      __func__, ctrl_pt_idx);
			return RET_FAILURE;
		}

		if ((points + 1)->x >= 0 && (points + 1)->y >= 0)
			points++;
	}

	/* init exp and gain */
	result = IMX728_InitFixedExposure(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set initial exposure values!\n", __func__);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX728_Configure
 *
 * @brief   Configure sensor with default settings for operation.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_Configure(IsiSensorHandle_t handle)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	IMX728_RawMode_e img_out_mode;
	IMX728_DriveMode_e mode_sel;

	img_out_mode = IMX728_IMG_MODE_HDR;
	mode_sel = IMX728_MODE_3856x2176_40_4LANE_RAW12_HDR;

	/* 0x98ac=0x03 */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_AE_MODE,
				       IMX728_AEMODE_FULL_ME);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't set full manual AE mode!\n", __func__);
		return RET_FAILURE;
	}

	/* 0xa248=0x00 */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_AWBMODE,
				       IMX728_AWBMODE_ATW);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't set full manual white balance mode!\n",
		      __func__);
		return RET_FAILURE;
	}

	/* 0x1808[0]=0 */
	result = IMX728_IsiUpdateRegIss(handle, IMX728_REG_AWB_EN,
					0b00000001, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't enable full manual WB mode!\n",
		      __func__);
		return RET_FAILURE;
	}


	/* Set Exposure time units to micro seconds for SP1, SP2, SP1 VS */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXP_UNIT_SP1,
				       IMX728_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't write SP1 exposure time unit!\n",
		      __func__);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXP_UNIT_SP2,
				       IMX728_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't write SP2 exposure time unit!\n",
		      __func__);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXP_UNIT_SP1VS,
				       IMX728_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't write SP1 VS exposure time unit!\n",
		      __func__);
		return RET_FAILURE;
	}

	/* HDR PWL curve and other settings */
	result = IMX728_HDRConfigure(handle, imx728_hdr_24bit);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Couldn't configure sensor for HDR mode!\n",
		      __func__);
		return RET_FAILURE;
	}

	/* Disabling Metadata (0x1708-0x170A, 0x1B40) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_MD_FEBD, 0x00);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_MD_REBD, 0x00);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_MD_APH, 0x00);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_MD_APF, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Error disabling metadata!\n", __func__);
		return RET_FAILURE;
	}
	/* Bitdepth (0x9728-9729) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_MODE_SEL_L,
				       (mode_sel >> 0) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_MODE_SEL_H,
					 0b01111111, (mode_sel >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set Bitdepth Mode!\n", __func__);
		return RET_FAILURE;
	}
	/* HDR/Linear Mode (0xEC7E=0x02, HDR mode) */
	result = IMX728_IsiUpdateRegIss(handle, IMX728_REG_FW_OUT_MODE,
					0b00000111, img_out_mode);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set Image Output mode!\n", __func__);
		return RET_FAILURE;
	}
	/* Disable Optical Black (0x28 - Disable, 0x2C Enable) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_OB_0_L,
				       (0X28 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_OB_0_H,
					(0x28 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_OB_1, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to disable Optical Black output!\n",
		      __func__);
		return RET_FAILURE;
	}
	/* Disable periodic skew calibration (0x1761=0x00) */
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_SKEW, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed disabling skew calibration!\n", __func__);
		return RET_FAILURE;
	}
	/* Set Sub pixels (0x9714[2:0]=0x07, 0xB684[2:0]=0x07) */
	result = IMX728_IsiUpdateRegIss(handle, IMX728_REG_SUBP_0,
					0b00000111, 0x7);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set Sub Pixel0 register!\n", __func__);
		return RET_FAILURE;
	}
	result = IMX728_IsiUpdateRegIss(handle, IMX728_REG_SUBP_1,
					0b00000111, 0x7);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: Failed to set Sub Pixel1 register!\n", __func__);
		return RET_FAILURE;
	}

	return result;
}

/**************************************************************************
 *          IMX728_SetModeInitParameters
 *
 * @brief   Set mode-specific init parameters
 *
 * @param   handle      Sensor handle
 * @param   pIMX728Ctx  Pointer to sensor context
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT IMX728_SetModeInitParameters(IsiSensorHandle_t handle,
					    IMX728_Context_t *pIMX728Ctx)
{
	RESULT result = RET_SUCCESS;
	uint16_t value = 0;
	uint32_t regVal = 0;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	pIMX728Ctx->aecIntegrationTimeIncrement = pIMX728Ctx->oneLineExpTime;
	pIMX728Ctx->aecGainIncrement            = 1.0f;
	/* oneLineExpTime, 13.88us */
	pIMX728Ctx->aecMinIntegrationTime       = 0.00001388;
	/* 20ms */
	pIMX728Ctx->aecMaxIntegrationTime       = 0.03;

	TRACE(IMX728_DEBUG, "%s: AecMaxIntegrationTime = %f\n",
	      __func__, pIMX728Ctx->aecMaxIntegrationTime);

	IMX728_IsiReadRegIss(handle, 0x98DF, &value);
	regVal = value << 24;
	IMX728_IsiReadRegIss(handle, 0x98DE, &value);
	regVal = regVal | ((value & 0xff) << 16);
	IMX728_IsiReadRegIss(handle, 0x98DD, &value);
	regVal = regVal | ((value & 0xff) << 8);
	IMX728_IsiReadRegIss(handle, 0x98DC, &value);
	regVal = regVal | (value & 0xff);
	/* unit us to unit s */
	pIMX728Ctx->aecCurIntTime.intTime[0] = regVal / 1000000.0;
	pIMX728Ctx->aecCurIntTime.intTime[1] = pIMX728Ctx->aecCurIntTime.intTime[0];
	pIMX728Ctx->aecCurIntTime.intTime[2] = pIMX728Ctx->aecCurIntTime.intTime[0];

	IMX728_IsiReadRegIss(handle, 0x98EF, &value);
	regVal = value << 24;
	IMX728_IsiReadRegIss(handle, 0x98EE, &value);
	regVal = regVal | ((value & 0xff) << 16);
	IMX728_IsiReadRegIss(handle, 0x98ED, &value);
	regVal = regVal | ((value & 0xff) << 8);
	IMX728_IsiReadRegIss(handle, 0x98EC, &value);
	regVal = regVal | (value & 0xff);
	/* unit us to unit s */
	pIMX728Ctx->aecCurIntTime.intTime[3] = regVal / 1000000.0;

	pIMX728Ctx->aecCurGain.gain[0]       = 1.0;
	pIMX728Ctx->aecCurGain.gain[1]       = 1.0;
	pIMX728Ctx->aecCurGain.gain[2]       = 1.0;
	pIMX728Ctx->aecCurGain.gain[3]       = 1.0;

	/* set initial wb 1.0x */
	IMX728_IsiWriteRegIss(handle, 0xa2a8, 0x00);
	IMX728_IsiWriteRegIss(handle, 0xa2a9, 0x1);
	IMX728_IsiWriteRegIss(handle, 0xa2aa, 0x00);
	IMX728_IsiWriteRegIss(handle, 0xa2ab, 0x1);
	IMX728_IsiWriteRegIss(handle, 0xa2ac, 0x00);
	IMX728_IsiWriteRegIss(handle, 0xa2ad, 0x1);
	IMX728_IsiWriteRegIss(handle, 0xa2ae, 0x00);
	IMX728_IsiWriteRegIss(handle, 0xa2af, 0x01);
	pIMX728Ctx->sensorWb.bGain  = 1.0;
	pIMX728Ctx->sensorWb.gbGain = 1.0;
	pIMX728Ctx->sensorWb.grGain = 1.0;
	pIMX728Ctx->sensorWb.rGain  = 1.0;

	/* initial bls */
	IMX728_IsiWriteRegIss(handle, 0xa690, 0x2);  /* obb clamp mode */
	IMX728_IsiWriteRegIss(handle, 0xa691, 0x1);  /* fixed clamp */
	IMX728_IsiWriteRegIss(handle, 0x0064, 0x0);  /* enable/disable user clamp */
	pIMX728Ctx->sensorBlc.blue = 0;
	pIMX728Ctx->sensorBlc.gb = 0;
	pIMX728Ctx->sensorBlc.gr = 0;
	pIMX728Ctx->sensorBlc.red = 0;
	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);

	return result;
}



/*****************************************************************************
 *          IMX728_IsiOpenIss
 *
 * @brief   Open of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   mode        Current work mode
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);
	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (pIMX728Ctx->streaming != BOOL_FALSE)
		return RET_WRONG_STATE;

	pIMX728Ctx->regAccessEnable = BOOL_TRUE;

	pIMX728Ctx->sensorMode.index   = mode;
	IsiSensorMode_t *SensorDefaultMode = NULL;

	for (uint32_t mode_idx = 0;
	     mode_idx < sizeof(pimx728_mode_info) / sizeof(IsiSensorMode_t);
	     mode_idx++) {
		if (pimx728_mode_info[mode_idx].index ==
		    pIMX728Ctx->sensorMode.index) {
			SensorDefaultMode = &(pimx728_mode_info[mode_idx]);
			break;
		}
	}

	if (SensorDefaultMode != NULL) {
		for (uint32_t reg_idx = 0;
		     reg_idx < ARRAY_SIZE(IMX728_3840x2160_init);
		     reg_idx++) {
			if (IMX728_3840x2160_init[reg_idx][0] == IMX728_TABLE_WAIT)
				osSleep(IMX728_3840x2160_init[reg_idx][1]);
			else if (IMX728_3840x2160_init[reg_idx][0] == IMX728_TABLE_END)
				break;
			else if (IMX728_3840x2160_init[reg_idx][0] == IMX728_TABLE_REMAP) {
				IMX728_IsiWriteRegIss(handle, IMX728_REG_REGMAP,
						      IMX728_REMAP_MODE_STANDBY);
				osSleep(10);
			} else {
				IMX728_IsiWriteRegIss(handle,
					IMX728_3840x2160_init[reg_idx][0],
					IMX728_3840x2160_init[reg_idx][1]);
			}
		}
		osSleep(1);

		memcpy(&(pIMX728Ctx->sensorMode), SensorDefaultMode,
		       sizeof(IsiSensorMode_t));
	} else {
		TRACE(IMX728_ERROR,
		      "%s: Invalid SensorDefaultMode\n", __func__);
		return RET_NULL_POINTER;
	}

	switch (pIMX728Ctx->sensorMode.index) {
	case 0:
		pIMX728Ctx->oneLineExpTime      = 0.00001388;
		pIMX728Ctx->frameLengthLines    = 0x960;
		pIMX728Ctx->curFrameLengthLines = pIMX728Ctx->frameLengthLines;
		pIMX728Ctx->maxIntegrationLine  =
			pIMX728Ctx->curFrameLengthLines - 24;
		pIMX728Ctx->minIntegrationLine  = 1;
		pIMX728Ctx->aecMaxGain          = 1.0;
		pIMX728Ctx->aecMinGain          = 1.0;
		break;
	default:
		TRACE(IMX728_INFO, "%s: not support sensor mode %d\n",
		      __func__, pIMX728Ctx->sensorMode.index);
		return RET_NOTSUPP;
	}

	MEMCPY(&pIMX728Ctx->aGain,
	       &pimx728_mode_info[pIMX728Ctx->sensorMode.index]
		.aeInfo.aGainRange[0], sizeof(IsiGainInfo_t));
	MEMCPY(&pIMX728Ctx->dGain,
	       &pimx728_mode_info[pIMX728Ctx->sensorMode.index]
		.aeInfo.dGainRange[0], sizeof(IsiGainInfo_t));
	pIMX728Ctx->maxFps  = pIMX728Ctx->sensorMode.fps;
	pIMX728Ctx->minFps  = 1 * ISI_FPS_QUANTIZE;
	pIMX728Ctx->currFps = pIMX728Ctx->maxFps;

	TRACE(IMX728_DEBUG, "%s: IMX728 System-Reset executed\n", __func__);

	result = IMX728_Configure(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Configuring Sensor failed. (err=%d)\n", __func__, result);
		return result;
	}
	result = IMX728_SetModeInitParameters(handle, pIMX728Ctx);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR,
		      "%s: IMX728_SetModeInitParameters failed. (err=%d)\n", __func__, result);
		return result;
	}

	pIMX728Ctx->configured = BOOL_TRUE;
	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetModeIss
 *
 * @brief   get cuurent sensor mode info.
 *
 * @param   handle      Sensor instance handle
 * @param   pMode       Sensor mode ptr
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetModeIss(IsiSensorHandle_t handle,
				   IsiSensorMode_t *pMode)
{
	TRACE(IMX728_INFO, "%s (enter)\n", __func__);
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;
	if (pMode == NULL)
		return RET_WRONG_HANDLE;

	memcpy(pMode, &(pIMX728Ctx->sensorMode), sizeof(pIMX728Ctx->sensorMode));
	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return  RET_SUCCESS;
}

/*****************************************************************************
 *          IMX728_IsiGetCapsIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   handle      Sensor instance handle
 * @param   pCaps       Sensor caps pointer
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *)handle;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);
	if (pIMX728Ctx == NULL || pCaps == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_WRONG_HANDLE;
	}

	pCaps->bitWidth			= pIMX728Ctx->sensorMode.bitWidth;
	pCaps->mode			= ISI_MODE_BAYER;
	pCaps->bayerPattern		= pIMX728Ctx->sensorMode.bayerPattern;
	pCaps->resolution.width		= pIMX728Ctx->sensorMode.size.width;
	pCaps->resolution.height	= pIMX728Ctx->sensorMode.size.height;
	pCaps->mipiLanes		= ISI_MIPI_4LANES;
	pCaps->vinType			= ISI_ITF_TYPE_MIPI;

	if (pCaps->bitWidth == 10)
		pCaps->mipiMode = ISI_FORMAT_RAW_10;
	else if (pCaps->bitWidth == 12)
		pCaps->mipiMode = ISI_FORMAT_RAW_12;
	else
		pCaps->mipiMode = ISI_MIPI_OFF;

	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX728_pIsiGetAeBaseInfoIss
 *
 * @brief   Returns the Ae base info of a sensor
 *          instance
 *
 * @param   handle        sensor instance handle
 * @param   pAeBaseInfo   Pointer to the sensor aebase info value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetAeBaseInfoIss(IsiSensorHandle_t handle,
					 IsiAeBaseInfo_t *pAeBaseInfo)
{
	const IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	RESULT result = RET_SUCCESS;
	uint16_t value = 0, regVal = 0;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pAeBaseInfo == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer received!!\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pIMX728Ctx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		/* based on HCG */
		/* get time limit and total gain limit */
		pAeBaseInfo->longGain.min        = pIMX728Ctx->aecMinGain;
		pAeBaseInfo->longGain.max        = pIMX728Ctx->aecMaxGain;
		pAeBaseInfo->longIntTime.min     = pIMX728Ctx->aecMinIntegrationTime;
		pAeBaseInfo->longIntTime.max     = pIMX728Ctx->aecMaxIntegrationTime;

		/* get again/dgain min/max/step info */
		pAeBaseInfo->aLongGain           = pIMX728Ctx->aGain;
		pAeBaseInfo->dLongGain           = pIMX728Ctx->dGain;

		/* get current intTime and gain */
		pAeBaseInfo->curGain = pIMX728Ctx->aecCurGain;
		pAeBaseInfo->curIntTime = pIMX728Ctx->aecCurIntTime;

		pAeBaseInfo->nativeMode      = pIMX728Ctx->sensorMode.nativeMode;

		IMX728_IsiReadRegIss(handle, 0x67e1, &value);
		regVal = value << 8;
		IMX728_IsiReadRegIss(handle, 0x67e0, &value);
		regVal = regVal | (value & 0xff);
		TRACE(IMX728_INFO,
		      "%s:normal gain from reg is %04x!\n", __func__, regVal);
		pAeBaseInfo->normGainSupport = true;
		/* U1.15 */
		pAeBaseInfo->normGain = regVal / 32768.0;
		TRACE(IMX728_INFO,
		      "%s:normal gain magnification factor is %f!\n",
		      __func__, pAeBaseInfo->normGain);

	} else {
		TRACE(IMX728_INFO,
		      "%s:not support this mode!\n", __func__);
		return RET_NOTSUPP;
	}

	pAeBaseInfo->aecGainStep     = pIMX728Ctx->aecGainIncrement;
	pAeBaseInfo->aecIntTimeStep  = pIMX728Ctx->aecIntegrationTimeIncrement;


	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          IMX728_IsiExcuteExpControlIss
 *
 * @brief   Execute exposure control
 *
 * @param   handle      Sensor handle
 * @param   pExpParam   Exposure parameters input
 * @param   pExpResult  Exposure parameters output
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT IMX728_IsiExcuteExpControlIss(IsiSensorHandle_t handle,
					    const IsiSensorExpParam_t *pExpParam,
					    IsiSensorExpParam_t *pExpResult)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pExpParam == NULL || pExpResult == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_WRONG_HANDLE;
	}

	TRACE(IMX728_INFO,
	      "%s: input intTime: %02f, again: %02f, dgain:%02f, ratio[0]: %f\n",
	      __func__, pExpParam->sensorExpTime[0], pExpParam->sensorAgain[0],
	      pExpParam->sensorDgain[0], pExpParam->sensorRatio[0]);

	/*
	 * The gain setting in IMX728 is fixed(1x),
	 * ISP gain is equal to real gain
	 */
	pExpResult->sensorAgain[0] = 1.0;
	pExpResult->sensorDgain[0] = 1.0;

	/* multip 10ms x, unit s */
	pExpResult->sensorExpTime[0] =
		(int)(pExpParam->sensorExpTime[0] * 100) / 100.0;
	TRACE(IMX728_INFO,
	      "%s: input intTime(s): %02f, output1 intTime(s): %02f.\n",
	      __func__, pExpParam->sensorExpTime[0],
	      pExpResult->sensorExpTime[0]);

	if (pExpResult->sensorExpTime[0] == 0)
		pExpResult->sensorExpTime[0] = 0.01; /* 0.01s = 10ms, min unit */

	pExpResult->ispGain = 1.0f;

	pExpResult->sensorAgain[1] = 1.0;
	pExpResult->sensorDgain[1] = 1.0;
	pExpResult->sensorExpTime[1] = pExpResult->sensorExpTime[0];

	pExpResult->sensorAgain[2] = 1.0;
	pExpResult->sensorDgain[2] = 1.0;
	pExpResult->sensorExpTime[2] = pExpResult->sensorExpTime[0];

	pExpResult->sensorAgain[3] = 1.0;
	pExpResult->sensorDgain[3] = 1.0;
	pExpResult->sensorExpTime[3] = pExpResult->sensorExpTime[0];
	TRACE(IMX728_INFO,
	      "%s: input intTime(s): %02f, output2 intTime(s): %02f.\n",
	      __func__, pExpParam->sensorExpTime[0],
	      pExpResult->sensorExpTime[0]);
	TRACE(IMX728_INFO,
	      "%s: realGain: %02f, ISPDigitalGain: %02f\n",
	      __func__, pExpParam->ispGain, pExpResult->ispGain);

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   mode        Streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT IMX728_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t mode)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pIMX728Ctx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	pIMX728Ctx->regAccessEnable = BOOL_TRUE;

	if (mode == true) {
		result = IMX728_IsiWriteRegIss(handle, IMX728_REG_STREAM, 0x5C);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR,
			      "%s: Failed to start sensor stream!\n", __func__);
			return RET_FAILURE;
		}
		result = IMX728_IsiWriteRegIss(handle, IMX728_REG_STREAM, 0xA3);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR,
			      "%s: Failed to start sensor stream!\n", __func__);
			return RET_FAILURE;
		}
	} else {
		result = IMX728_IsiWriteRegIss(handle, IMX728_REG_STREAM, 0xFF);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR,
			      "%s: Failed to Stop sensor stream!\n", __func__);
			return RET_FAILURE;
		}
	}

	pIMX728Ctx->streaming = mode;

	TRACE(IMX728_INFO,
	      "%s: streaming=%s mode=%d res=%dx%d fps=%d\n",
	      __func__, mode ? "ON" : "OFF",
	      pIMX728Ctx->sensorMode.index,
	      pIMX728Ctx->sensorMode.size.width,
	      pIMX728Ctx->sensorMode.size.height,
	      pIMX728Ctx->currFps / ISI_FPS_QUANTIZE);
	
	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiCloseIss
 *
 * @brief   Close the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiCloseIss(IsiSensorHandle_t handle)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;

	(void)IMX728_IsiSetStreamingIss(pIMX728Ctx, BOOL_FALSE);
	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX728_IsiReleaseIss
 *
 * @brief   Release the image sensor instance and free resources.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT IMX728_IsiReleaseIss(IsiSensorHandle_t handle)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;

	stop_sensor(pIMX728Ctx->sensorDevId);
	osMutexDestroy(&pIMX728Ctx->registerLock);

	MEMSET(pIMX728Ctx, 0, sizeof(IMX728_Context_t));
	osFree(pIMX728Ctx);
	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX728_IsiSetAGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *
 * @param   handle                  sensor instance handle
 * @param   pSensorAGain            pointer to sensor again to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiSetAGainIss(IsiSensorHandle_t handle,
				    IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pSensorAGain == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	/* set imx728 sensor again, fix again to 1x */
	result  = IMX728_IsiWriteRegIss(handle, 0x98F8, 0xf0); /* SP1H */
	result |= IMX728_IsiWriteRegIss(handle, 0x98FC, 0x4b); /* SP1L */
	result |= IMX728_IsiWriteRegIss(handle, 0x9900, 0x15); /* SP1EC */
	result |= IMX728_IsiWriteRegIss(handle, 0x9904, 0x21); /* SP2 */
	result |= IMX728_IsiWriteRegIss(handle, 0x9908, 0x54); /* SP1VS */
	pIMX728Ctx->curAgain.gain[0] = 1.0;
	pIMX728Ctx->curAgain.gain[1] = 1.0;
	pIMX728Ctx->curAgain.gain[2] = 1.0;
	pIMX728Ctx->curAgain.gain[3] = 1.0;

	TRACE(IMX728_INFO,
	      "%s: frame=%d aGain=[%.3f,%.3f,%.3f,%.3f]\n",
	      __func__, g_Sensor_frame_count,
	      pIMX728Ctx->curAgain.gain[0],
	      pIMX728Ctx->curAgain.gain[1],
	      pIMX728Ctx->curAgain.gain[2],
	      pIMX728Ctx->curAgain.gain[3]);

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiSetDGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorDGain             pointer to sensor dgain to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX728_IsiSetDGainIss(IsiSensorHandle_t handle,
			     IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;
	int gain_idx;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pSensorDGain == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	/* set imx728 sensor dgain 1x */
	result  = IMX728_IsiWriteRegIss(handle, 0x9918, 0x00);
	pIMX728Ctx->curDgain.gain[0] = 1.0;
	pIMX728Ctx->curDgain.gain[1] = 1.0;
	pIMX728Ctx->curDgain.gain[2] = 1.0;
	pIMX728Ctx->curDgain.gain[3] = 1.0;

	for (gain_idx = 0; gain_idx < 4; gain_idx++) {
		pIMX728Ctx->aecCurGain.gain[gain_idx] =
			pIMX728Ctx->curAgain.gain[gain_idx] *
			pIMX728Ctx->curDgain.gain[gain_idx];
	}

	TRACE(IMX728_INFO,
	      "%s: frame=%d dGain=[%.3f,%.3f,%.3f,%.3f] "
	      "totalGain=[%.3f,%.3f,%.3f,%.3f]\n",
	      __func__, g_Sensor_frame_count,
	      pIMX728Ctx->curDgain.gain[0],
	      pIMX728Ctx->curDgain.gain[1],
	      pIMX728Ctx->curDgain.gain[2],
	      pIMX728Ctx->curDgain.gain[3],
	      pIMX728Ctx->aecCurGain.gain[0],
	      pIMX728Ctx->aecCurGain.gain[1],
	      pIMX728Ctx->aecCurGain.gain[2],
	      pIMX728Ctx->aecCurGain.gain[3]);

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetAGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                    sensor instance handle
 * @param   pSensorAGain              pointer to sensor again to get
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetAGainIss(IsiSensorHandle_t handle,
					    IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	const IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorAGain) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	memcpy(pSensorAGain, &pIMX728Ctx->curAgain, sizeof(IsiSensorGain_t));

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetDGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorDGain             pointer to sensor dgain to get
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetDGainIss(IsiSensorHandle_t handle,
					    IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	const IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorDGain) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	memcpy(pSensorDGain, &pIMX728Ctx->curDgain, sizeof(IsiSensorGain_t));

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetIntTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorIntTime           pointer to integration time to get
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetIntTimeIss(IsiSensorHandle_t handle,
					      IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	const IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}
	if (!pSensorIntTime) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	memcpy(pSensorIntTime, &pIMX728Ctx->aecCurIntTime,
		sizeof(IsiSensorIntTime_t));

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiSetIntTimeIss
 * @brief   Writes integration time values to the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorIntTime           pointer to sensor integration time to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiSetIntTimeIss(IsiSensorHandle_t handle,
				      const IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;
	/* default value: 10000us=10ms */
	uint32_t expTimeus = 0x2710;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_WRONG_HANDLE;
	}

	if (pSensorIntTime->intTime[0] != 0) {
		TRACE(IMX728_INFO,
		      "%s:Set SensorIntTime->intTime[0] %f (s) .\n",
		      __func__, pSensorIntTime->intTime[0]);
		expTimeus = MIN(pIMX728Ctx->aecMaxIntegrationTime * 1000000,
				MAX(pIMX728Ctx->aecMinIntegrationTime * 1000000,
				    pSensorIntTime->intTime[0] * 1000000));
		TRACE(IMX728_INFO,
		      "%s: set expTimeus = %d\n", __func__, expTimeus);
	}

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];

	/* Set exposure time for first group (0x98DC-0x98DF) */
	group_values[0] = expTimeus & 0xFF;
	group_values[1] = (expTimeus & 0xFF00) >> 8;
	group_values[2] = (expTimeus & 0xFF0000) >> 16;
	group_values[3] = (expTimeus & 0xFF000000) >> 24;

#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX728_INFO,
	      "%s: Before write - Group1: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result = IMX728_IsiWriteRegGroupIss(handle, 0x98DC, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back Group1 values */
	uint32_t read_value = 0;
	uint8_t read_values[4] = {0};

	IMX728_IsiReadRegIss(handle, 0x98DC, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0x98DD, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0x98DE, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0x98DF, &read_value);
	read_values[3] = read_value & 0xFF;
	TRACE(IMX728_INFO,
	      "%s: After write - Group1: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	/* Set exposure time for second group (0xC198-0xC19B) */
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX728_INFO,
	      "%s: Before write - Group2: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= IMX728_IsiWriteRegGroupIss(handle, 0xC198, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back Group2 values */
	IMX728_IsiReadRegIss(handle, 0xC198, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0xC199, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0xC19A, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0xC19B, &read_value);
	read_values[3] = read_value & 0xFF;
	TRACE(IMX728_INFO,
	      "%s: After write - Group2: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	/* Set exposure time for third group (0x98E4-0x98E7) */
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX728_INFO,
	      "%s: Before write - Group3: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= IMX728_IsiWriteRegGroupIss(handle, 0x98E4, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back Group3 values */
	IMX728_IsiReadRegIss(handle, 0x98E4, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0x98E5, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0x98E6, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0x98E7, &read_value);
	read_values[3] = read_value & 0xFF;
	TRACE(IMX728_INFO,
	      "%s: After write - Group3: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	/* Set exposure time for fourth group (0xC1A0-0xC1A3) */
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX728_INFO,
	      "%s: Before write - Group4: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= IMX728_IsiWriteRegGroupIss(handle, 0xC1A0, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back Group4 values */
	IMX728_IsiReadRegIss(handle, 0xC1A0, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0xC1A1, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0xC1A2, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX728_IsiReadRegIss(handle, 0xC1A3, &read_value);
	read_values[3] = read_value & 0xFF;
	TRACE(IMX728_INFO,
	      "%s: After write - Group4: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif
#else
	result  = IMX728_IsiWriteRegIss(handle, 0x98DC, expTimeus & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, 0x98DD,
					(expTimeus & 0xFF00) >> 8);
	result |= IMX728_IsiWriteRegIss(handle, 0x98DE,
					(expTimeus & 0xFF0000) >> 16);
	result |= IMX728_IsiWriteRegIss(handle, 0x98DF,
					(expTimeus & 0xFF000000) >> 24);

	result |= IMX728_IsiWriteRegIss(handle, 0xC198, expTimeus & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, 0xC199,
					(expTimeus & 0xFF00) >> 8);
	result |= IMX728_IsiWriteRegIss(handle, 0xC19A,
					(expTimeus & 0xFF0000) >> 16);
	result |= IMX728_IsiWriteRegIss(handle, 0xC19B,
					(expTimeus & 0xFF000000) >> 24);

	result |= IMX728_IsiWriteRegIss(handle, 0x98E4, expTimeus & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, 0x98E5,
					(expTimeus & 0xFF00) >> 8);
	result |= IMX728_IsiWriteRegIss(handle, 0x98E6,
					(expTimeus & 0xFF0000) >> 16);
	result |= IMX728_IsiWriteRegIss(handle, 0x98E7,
					(expTimeus & 0xFF000000) >> 24);

	result |= IMX728_IsiWriteRegIss(handle, 0xC1A0, expTimeus & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, 0xC1A1,
					(expTimeus & 0xFF00) >> 8);
	result |= IMX728_IsiWriteRegIss(handle, 0xC1A2,
					(expTimeus & 0xFF0000) >> 16);
	result |= IMX728_IsiWriteRegIss(handle, 0xC1A3,
					(expTimeus & 0xFF000000) >> 24);
#endif

	pIMX728Ctx->aecCurIntTime.intTime[0] = expTimeus / 1000000.0;
	pIMX728Ctx->aecCurIntTime.intTime[1] = expTimeus / 1000000.0;
	pIMX728Ctx->aecCurIntTime.intTime[2] = expTimeus / 1000000.0;

	TRACE(IMX728_INFO,
	      "%s: frame=%d intTime=[%.6f,%.6f,%.6f] expTimeus=%u\n",
	      __func__, g_Sensor_frame_count,
	      pIMX728Ctx->aecCurIntTime.intTime[0],
	      pIMX728Ctx->aecCurIntTime.intTime[1],
	      pIMX728Ctx->aecCurIntTime.intTime[2],
	      expTimeus);

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetIspStatusIss
 *
 * @brief   Get sensor isp status.
 *
 * @param   handle              sensor instance handle
 * @param   pIspStatus          sensor isp status
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetIspStatusIss(IsiSensorHandle_t handle,
						IsiIspStatus_t *pIspStatus)
{
	RESULT result = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pIspStatus == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	pIspStatus->useSensorAE  = false;
	pIspStatus->useSensorBLC = false;
	pIspStatus->useAWBMode = ISI_USE_ISP_WB_GAIN;

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiSetWBIss
 *
 * @brief   set sensor linear mode white balance
 *          or hdr mode normal exp frame white balance
 *
 * @param   handle            sensor instance handle
 * @param   pWb               wb params pointer
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_IsiSetWBIss(IsiSensorHandle_t handle,
					 const IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pWb == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetWBIss
 *
 * @brief   set sensor linear mode white balance
 *          or hdr mode normal exp frame white balance
 *
 * @param   handle            sensor instance handle
 * @param   pWb               wb params point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetWBIss(IsiSensorHandle_t handle,  IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pWb == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}
	return result;
}

/*****************************************************************************
 *          IMX728_IsiSetBlcIss
 *
 * @brief   set sensor linear mode black level
 *
 *
 * @param   handle            sensor instance handle
 * @param   pBlc              blc params pointer
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_IsiSetBlcIss(IsiSensorHandle_t handle,
					  const IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pBlc == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	return result;
}
/*****************************************************************************
 *          IMX728_IsiGetBlcIss
 *
 * @brief   set sensor linear mode black level
 *
 *
 * @param   handle            sensor instance handle
 * @param   pBlc              blc params point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetBlcIss(IsiSensorHandle_t handle,
					  IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pBlc == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetTpgIss
 *
 * @brief   set sensor test pattern.
 *
 * @param   handle       Sensor instance handle
 * @param   pTpg         Sensor test pattern ptr
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX728_IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *pTpg)
{
	RESULT result = RET_SUCCESS;
	uint16_t value = 0;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pTpg == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pIMX728Ctx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	if (!IMX728_IsiReadRegIss(handle, 0xB58E, &value)) {
		pTpg->enable = ((value & 0x01) != 0) ? 1 : 0;
		if (pTpg->enable)
			pTpg->pattern = (0xff & value);
		pIMX728Ctx->testPattern = pTpg->enable;
	}

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiSetTpgIss
 *
 * @brief   set sensor test pattern.
 *
 * @param   handle      Sensor instance handle
 * @param   tpg         Sensor test pattern
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX728_IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pIMX728Ctx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	if (tpg.enable == 1) {
		result = IMX728_IsiWriteRegIss(handle, 0x1A28, 0x01);
		result = IMX728_IsiWriteRegIss(handle, 0x1A2A, 0x80);
		result = IMX728_IsiWriteRegIss(handle, 0x1A2B, 0x00);
		result = IMX728_IsiWriteRegIss(handle, 0x1A30, 0xFF);
		result = IMX728_IsiWriteRegIss(handle, 0x1A31, 0x0F);
		result = IMX728_IsiWriteRegIss(handle, 0x1A32, 0x00);
		result = IMX728_IsiWriteRegIss(handle, 0xB58E, 0x01);
		result = IMX728_IsiWriteRegIss(handle, 0xB58F, 0x02);
		result = IMX728_IsiWriteRegIss(handle, 0xB6C4, 0x01);
		result = IMX728_IsiWriteRegIss(handle, 0xB6C5, 0x02);
	}

	pIMX728Ctx->testPattern = tpg.enable;

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetExpandCurveIss
 *
 * @brief   get sensor expand curve
 *
 * @param   handle            sensor instance handle
 * @param   pCurve            expand curve pointer
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX728_IsiGetExpandCurveIss(IsiSensorHandle_t handle,
						IsiSensorCompandCurve_t *pCurve)
{
	RESULT result = RET_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL || pCurve == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	return result;
}

/*****************************************************************************
 *          IMX728_IsiGetSensorIss
 *
 * @brief   get sensor api's
 *
 * @param   pIsiSensor        sensor specific api's
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX728_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s (enter)\n", __func__);

	static const char SensorName[16] = "IMX728";

	if (pIsiSensor != NULL) {
		pIsiSensor->pszName = SensorName;
		pIsiSensor->pIsiCreateIss = IMX728_IsiCreateIss;
		pIsiSensor->pIsiOpenIss = IMX728_IsiOpenIss;
		pIsiSensor->pIsiCloseIss = IMX728_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss = IMX728_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss = IMX728_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss = IMX728_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss = IMX728_IsiGetModeIss;
		pIsiSensor->pIsiEnumModeIss = IMX728_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss = IMX728_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss =
			IMX728_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss = IMX728_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss = IMX728_IsiSetStreamingIss;
		pIsiSensor->pIsiGetAeBaseInfoIss = IMX728_IsiGetAeBaseInfoIss;
		pIsiSensor->pIsiExcuteExpCtrlIss =
			IMX728_IsiExcuteExpControlIss;
		pIsiSensor->pIsiGetAGainIss = IMX728_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss = IMX728_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss = IMX728_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss = IMX728_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss = IMX728_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss = IMX728_IsiSetIntTimeIss;
		pIsiSensor->pIsiGetFpsIss = IMX728_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = IMX728_IsiSetFpsIss;
		pIsiSensor->pIsiGetIspStatusIss = IMX728_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss = IMX728_IsiSetWBIss;
		pIsiSensor->pIsiGetWBIss = IMX728_IsiGetWBIss;
		pIsiSensor->pIsiSetBlcIss = IMX728_IsiSetBlcIss;
		pIsiSensor->pIsiGetBlcIss = IMX728_IsiGetBlcIss;
		pIsiSensor->pIsiSetTpgIss = IMX728_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss                       =
			IMX728_IsiGetTpgIss;
		pIsiSensor->pIsiGetExpandCurveIss               =
			IMX728_IsiGetExpandCurveIss;
		pIsiSensor->pIsiFocusCreateIss                  = NULL;
		pIsiSensor->pIsiFocusReleaseIss                 = NULL;
		pIsiSensor->pIsiFocusGetCalibrateIss            = NULL;
		pIsiSensor->pIsiFocusSetIss                     = NULL;
		pIsiSensor->pIsiFocusGetIss                     = NULL;
		pIsiSensor->pIsiSetIRLightExpIss                = NULL;
		pIsiSensor->pIsiGetIRLightExpIss                = NULL;
	} else {
		result = RET_NULL_POINTER;
		TRACE(IMX728_ERROR, "%s: NULL pointer detected\n", __func__);
	}

	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return  result;
}

/*
 * each sensor driver need declare this struct for isi load
 */
IsiCamDrvConfig_t IMX728_IsiCamDrvConfig = {
	.cameraDriverID = IMX728_SENSOR_ID,
	.pIsiGetSensorIss = IMX728_IsiGetSensorIss,
};
