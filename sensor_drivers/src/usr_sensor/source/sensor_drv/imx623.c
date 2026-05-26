/****************************************************************************
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
#include "sensor_drv/imx623_priv.h"

CREATE_TRACER(IMX623_INFO, "IMX623: ", INFO, 1);
CREATE_TRACER(IMX623_WARN, "IMX623: ", WARNING, 1);
CREATE_TRACER(IMX623_ERROR, "IMX623: ", ERROR, 1);
CREATE_TRACER(IMX623_DEBUG, "IMX623: ", INFO, 1);
CREATE_TRACER(IMX623_REG_INFO, "IMX623: ", INFO, 1);
CREATE_TRACER(IMX623_REG_DEBUG, "IMX623: ", INFO, 1);

/**
 * @brief IMX623 supported sensor modes configuration
 *
 * Mode 0: 1920x1080 cropped from 1936x1552 sensor area (1080p)
 * Mode 1: 1936x1552 full sensor resolution
 *
 * Both modes operate at 30 FPS with HDR native mode and 12-bit output.
 */
IsiSensorMode_t pimx623_mode_info[] = {

	/* Mode 0: 1080p (1920x1080) - Cropped from full sensor area */
	{
		.index     = 0,
		.size = {

			.boundsWidth  = 1936,
			.boundsHeight = 1552,
			.top          = 236,
			.left         = 8,
			.width        = 1920,
			.height       = 1080,
		},
		.aeInfo = {

			.intTimeDelayFrame = 2,
			.gainDelayFrame    = 2,
		},
		.fps          = 30 * ISI_FPS_QUANTIZE,
		.hdrMode      = ISI_SENSOR_MODE_HDR_NATIVE,
		.nativeMode   = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth     = 12,
		.compress = {

			.enable = 1,
			.xBit   = 24,
			.yBit   = 12,
		},
		.bayerPattern = ISI_BPAT_RGGB,
		.afMode       = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType     = ISI_MODE_BAYER,
		.mipiLane     = ISI_MIPI_4LANES,
	},
	/* Mode 1: Full Resolution (1936x1552) */
	{
		.index     = 1,
		.size = {

			.boundsWidth  = 1936,
			.boundsHeight = 1552,
			.top          = 0,
			.left         = 0,
			.width        = 1936,
			.height       = 1552,
		},
		.aeInfo = {

			.intTimeDelayFrame = 2,
			.gainDelayFrame    = 2,
		},
		.fps          = 30 * ISI_FPS_QUANTIZE,
		.hdrMode      = ISI_SENSOR_MODE_HDR_NATIVE,
		.nativeMode   = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth     = 12,
		.compress = {

			.enable = 1,
			.xBit   = 24,
			.yBit   = 12,
		},
		.bayerPattern = ISI_BPAT_RGGB,
		.afMode       = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType     = ISI_MODE_BAYER,
		.mipiLane     = ISI_MIPI_4LANES,
	}
};

int imx623_mode_num = ARRAY_SIZE(pimx623_mode_info);

/*****************************************************************************
 * @brief IMX623 camera driver configuration structure
 *
 * This structure is required by the ISI framework for sensor driver loading
 * and initialization. It provides the sensor ID for identification and the
 * function pointer to retrieve the sensor interface structure.
 *****************************************************************************/
IsiCamDrvConfig_t IMX623_IsiCamDrvConfig = {

	.cameraDriverID   = IMX623_SENSOR_ID,
	.pIsiGetSensorIss = IMX623_IsiGetSensorIss,
};

/*****************************************************************************
 *          IMX623_IsiReadRegIss
 *
 * @brief   Read register value from IMX623 sensor via I2C.
 *
 * @param   handle      Sensor instance handle
 * @param   addr        Register address to read from
 * @param   pValue      Pointer to store the read value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_IsiReadRegIss(IsiSensorHandle_t handle,
				   const uint16_t addr,
				   uint16_t *pValue)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	RESULT Status = RET_SUCCESS;
	uint8_t read_val = 0;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL || pValue == NULL) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_NULL_POINTER;
	}

	osMutexLock(&pIMX623Ctx->registerLock);
	memset(pValue, 0, sizeof(uint16_t));

	if (!pIMX623Ctx->regAccessEnable) {
		TRACE(IMX623_ERROR,
		      "%s: register cannot be read!\n", __func__);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_WRONG_STATE;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(IMX623_ERROR, "%s: No FMC selected\r\n", __func__);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pIMX623Ctx->sensorDevId]->sensor_alias_addr)
		>> 1;

	Status =
	active_fmc->accessiic_array[pIMX623Ctx->sensorDevId]->readIIC
	(pIMX623Ctx->i2cId, slave_addr, addr, 0x2, &read_val, 1);
	if (Status != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: I2C read failed for address 0x%04x (err=%d)\n",
			__func__, addr, Status);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_FAILURE;
	}
	*pValue = (uint16_t)read_val;

	osMutexUnlock(&pIMX623Ctx->registerLock);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX623_IsiWriteRegIss
 *
 * @brief   Write register value to IMX623 sensor via I2C.
 *
 * @param   handle      Sensor instance handle
 * @param   addr        Register address to write to
 * @param   value       Value to write to the register
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_IsiWriteRegIss(IsiSensorHandle_t handle,
		const uint16_t addr, const uint16_t value)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	RESULT Status = RET_SUCCESS;
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_NULL_POINTER;
	}

	osMutexLock(&pIMX623Ctx->registerLock);

	if (!pIMX623Ctx->regAccessEnable) {
		TRACE(IMX623_ERROR,
		      "%s: register cannot be written!\n", __func__);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_WRONG_STATE;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(IMX623_ERROR, "%s: No FMC selected\r\n", __func__);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pIMX623Ctx->sensorDevId]->sensor_alias_addr)
		>> 1;

	u8 wr_data[2];

	wr_data[0] = (u8)value;

	Status =
	active_fmc->accessiic_array[pIMX623Ctx->sensorDevId]->writeIIC
	(pIMX623Ctx->i2cId, slave_addr, addr, 0x2, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: I2C write failed for address 0x%04x (err=%d)\n",
			__func__, addr, Status);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_FAILURE;
	}

	if ((addr == IMX623_REG_MODE_SET_F_H) && (value == 0x00))
		pIMX623Ctx->regAccessEnable = BOOL_FALSE;

	osMutexUnlock(&pIMX623Ctx->registerLock);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

#ifdef ENABLE_I2C_GROUPING
/*****************************************************************************
 *          IMX623_IsiWriteRegGroupIss
 *
 * @brief   Write group register values to IMX623 sensor via I2C.
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
 *
 *****************************************************************************/
static RESULT IMX623_IsiWriteRegGroupIss(IsiSensorHandle_t handle,
		const uint16_t addr, uint8_t *value, uint8_t datacount)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	RESULT Status = RET_SUCCESS;
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_NULL_POINTER;
	}

	if (datacount > 4U) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid datacount %d (exceeds maximum of 4)\n",
		      __func__, datacount);
		return RET_FAILURE;
	}

	osMutexLock(&pIMX623Ctx->registerLock);

	if (!pIMX623Ctx->regAccessEnable) {
		TRACE(IMX623_ERROR,
		      "%s: register cannot be written!\n", __func__);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_WRONG_STATE;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(IMX623_ERROR, "%s: No FMC selected\r\n", __func__);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pIMX623Ctx->sensorDevId]->sensor_alias_addr)
		>> 1;
	Status =
	active_fmc->accessiic_array[pIMX623Ctx->sensorDevId]->writeIIC
	(pIMX623Ctx->i2cId, slave_addr, addr, 0x2, value, datacount);
	if (Status != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: I2C group write failed for address 0x%04x (err=%d)\n",
			__func__, addr, Status);
		osMutexUnlock(&pIMX623Ctx->registerLock);
		return RET_FAILURE;
	}

	if ((addr == IMX623_REG_MODE_SET_F_H) && (value == 0x00))
		pIMX623Ctx->regAccessEnable = BOOL_FALSE;

	osMutexUnlock(&pIMX623Ctx->registerLock);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}
#endif


/*****************************************************************************
 *          IMX623_IsiUpdateRegIss
 *
 * @brief   Update specific bits in a register using mask and value.
 *
 * @param   handle      Sensor instance handle
 * @param   addr        Register address to update
 * @param   mask        Bit mask for the update (1s indicate bits to update)
 * @param   value       Value to write (masked bits applied to register)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_IsiUpdateRegIss(IsiSensorHandle_t handle,
				      const uint16_t addr,
				      const uint8_t mask,
				      const uint8_t value)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;
	uint16_t reg_val16 = 0;

	/* Read the current register value */
	result = IMX623_IsiReadRegIss(handle, addr, &reg_val16);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to read register at address 0x%x (err=%d)\n",
		      __func__, addr, result);
		return RET_FAILURE;
	}

	uint8_t reg_val = (uint8_t)(reg_val16 & 0xFF);

	/* Apply mask: set bits where mask is 1 to value, keep others */
	reg_val = (mask & value) | (~mask & reg_val);

	/* Write the updated value back to the register */
	result = IMX623_IsiWriteRegIss(handle, addr, reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to write register value to address 0x%x (err=%d)\n",
		      __func__, addr, result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_GetSensorState
 *
 * @brief   Get the current sensor state.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the sensor state.
 * @retval  SensorState_e enumeration value
 * @retval  IMX623_SENSOR_STATE_INVALID on error
 *
 *****************************************************************************/
SensorState_e IMX623_GetSensorState(IsiSensorHandle_t handle)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	uint8_t sensor_state = 0;

	RESULT result = RET_SUCCESS;

	result = IMX623_IsiReadRegIss(handle, IMX623_REG_STATE,
				      (uint16_t *)&sensor_state);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read sensor state register! (err=%d)\n", __func__, result);
		return IMX623_SENSOR_STATE_INVALID;
	}

	TRACE(IMX623_DEBUG, "%s: IMX623 Sensor State: %d\n", __func__, sensor_state);
	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return (SensorState_e)sensor_state;
}

/*****************************************************************************
 *          imx623_sensor_framecount
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
int imx623_sensor_framecount(IsiSensorHandle_t handle)
{
	u32 frame_counter;
	uint16_t read_buf[4] = {0};
	int Status = RET_SUCCESS;

	Status = IMX623_IsiReadRegIss(handle, 0x7dc8, &read_buf[0]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}
	Status = IMX623_IsiReadRegIss(handle, 0x7dc9, &read_buf[1]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	Status = IMX623_IsiReadRegIss(handle, 0x7dca, &read_buf[2]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	Status = IMX623_IsiReadRegIss(handle, 0x7dcb, &read_buf[3]);
	if (Status != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s:%d Failed to read register (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}
	frame_counter = (read_buf[3] << 24) | (read_buf[2] << 16) |
		(read_buf[1] << 8) | (read_buf[0]);
	g_Sensor_frame_count = frame_counter;

	return Status;
}

/*****************************************************************************
 *          IMX623_IsiGetSensorIss
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
RESULT IMX623_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	static const char SensorName[16] = "IMX623";

	if (pIsiSensor != NULL) {
		/* Core sensor driver functions */
		pIsiSensor->pszName                  = SensorName;
		pIsiSensor->pIsiCreateIss            = IMX623_IsiCreateIss;
		pIsiSensor->pIsiOpenIss              = IMX623_IsiOpenIss;
		pIsiSensor->pIsiCloseIss             = IMX623_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss           = IMX623_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss           = IMX623_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss          = IMX623_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss           = IMX623_IsiGetModeIss;
		pIsiSensor->pIsiEnumModeIss          = IMX623_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss           = IMX623_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss   = IMX623_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss       = IMX623_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss      = IMX623_IsiSetStreamingIss;

		/* Auto Exposure Control (AEC) functions */
		pIsiSensor->pIsiGetAeBaseInfoIss     = IMX623_pIsiGetAeBaseInfoIss;
		pIsiSensor->pIsiGetAGainIss          = IMX623_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss          = IMX623_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss          = IMX623_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss          = IMX623_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss        = IMX623_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss        = IMX623_IsiSetIntTimeIss;

		/* Frame Rate Control functions */
		pIsiSensor->pIsiGetFpsIss            = IMX623_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss            = IMX623_IsiSetFpsIss;

		/* Sensor ISP functions */
		pIsiSensor->pIsiGetIspStatusIss      = IMX623_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss             = IMX623_IsiSetWBIss;
		pIsiSensor->pIsiGetWBIss             = IMX623_IsiGetWBIss;
		pIsiSensor->pIsiSetBlcIss            = IMX623_IsiSetBlcIss;
		pIsiSensor->pIsiGetBlcIss            = IMX623_IsiGetBlcIss;
		pIsiSensor->pIsiGetExpandCurveIss    = IMX623_IsiGetExpandCurveIss;

		/* Test Pattern Generator functions */
		pIsiSensor->pIsiSetTpgIss            = IMX623_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss            = IMX623_IsiGetTpgIss;

		/* Auto Focus functions (not supported) */
		pIsiSensor->pIsiFocusCreateIss       = NULL;
		pIsiSensor->pIsiFocusReleaseIss      = NULL;
		pIsiSensor->pIsiFocusGetCalibrateIss = NULL;
		pIsiSensor->pIsiFocusSetIss          = NULL;
		pIsiSensor->pIsiFocusGetIss          = NULL;
		pIsiSensor->pIsiSetIRLightExpIss     = NULL;
		pIsiSensor->pIsiGetIRLightExpIss     = NULL;
	} else {
		TRACE(IMX623_ERROR, "%s: NULL pointer provided for pIsiSensor\n",
			__func__);
		result = RET_NULL_POINTER;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiCreateIss
 *
 * @brief   Create Sensor Context for the given config
 *
 * @param   pConfig    Given Sensor Config
 * @param   pHandle    Return the Sensor Ctx
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX623_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig,
				  IsiSensorHandle_t *pHandle)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;
	uint32_t desId = 0, pipeId = 0;

	IMX623_Context_t *pIMX623Ctx;

	pIMX623Ctx = (IMX623_Context_t *)osMalloc(sizeof(IMX623_Context_t));

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to allocate memory for IMX623 context\n",
		      __func__);
		return RET_OUTOFMEM;
	}

	MEMSET(pIMX623Ctx, 0, sizeof(IMX623_Context_t));

	/* Initialize sensor context with default values */
	pIMX623Ctx->isiCtx.pSensor	= pConfig->pSensor;
	pIMX623Ctx->configured		= BOOL_FALSE;
	pIMX623Ctx->streaming		= BOOL_FALSE;
	pIMX623Ctx->testPattern		= BOOL_FALSE;
	pIMX623Ctx->isAfpsRun		= BOOL_FALSE;
	pIMX623Ctx->sensorMode.index	= 0;
	pIMX623Ctx->i2cId		= 0;
	pIMX623Ctx->sensorDevId		= pConfig->halDevID;
	pIMX623Ctx->regAccessEnable	= BOOL_TRUE;

	osMutexInit(&pIMX623Ctx->registerLock);

	pipeId = pIMX623Ctx->sensorDevId;
	*pHandle = (IsiSensorHandle_t)pIMX623Ctx;

	if (pipeId >= IN_PIPE_LAST) {
		TRACE(IMX623_ERROR,
		      "%s: Sensor device ID %d is not supported!\n",
		      __func__, pipeId);
		*pHandle = NULL;
		osFree(pIMX623Ctx);
		return RET_UNSUPPORT_ID;
	}

	desId = MAPPING_INPIPE_TO_DES_ID(pipeId);

	/*Assignment of  Bus ID */
	pIMX623Ctx->i2cId = GetI2cBusIdForDes(desId);
	uint8_t busId = (uint8_t)pIMX623Ctx->i2cId;

	/* Validate I2C bus ID */
	if (busId == INVALID_I2C_BUS_ID) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid I2C bus ID for desId %d\n",
		      __func__, desId);
		osFree(pIMX623Ctx);
		return RET_FAILURE;
	}

	TRACE(IMX623_INFO,
	      "%s: desId: %d, pipeId: %d, i2cBusId:%d\r\n",
	      __func__, desId, pipeId, busId);

	result = init_iic_access(busId, pipeId);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: init_iic_access failed for pipe %d (err=%d)\n",
		      __func__, pipeId, result);
		osFree(pIMX623Ctx);
		return result;
	}
	result = init_des(desId);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to initialize deserializer %d (err=%d)\n",
		      __func__, desId, result);
		osFree(pIMX623Ctx);
		return result;
	}
	result = init_sensor(pipeId, desId, SENSOR_IMX623_ADDRESS);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: init_sensor failed for pipe %d (err=%d)\n",
		      __func__, pipeId, result);
		osFree(pIMX623Ctx);
		return result;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiEnumModeIss
 *
 * @brief   query sensor info.
 *
 * @param   handle                sensor instance handle
 * @param   pEnumMode             sensor query mode
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX623_IsiEnumModeIss(IsiSensorHandle_t handle,
				    IsiSensorEnumMode_t *pEnumMode)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL || pEnumMode == NULL) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid handle or NULL pointer for pEnumMode\n",
		      __func__);
		return RET_NULL_POINTER;
	}

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (pEnumMode->index >= ARRAY_SIZE(pimx623_mode_info)) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid mode index %d, max supported is %d\n",
		      __func__, pEnumMode->index,
		      (int)(ARRAY_SIZE(pimx623_mode_info) - 1));
		return RET_OUTOFRANGE;
	}

	for (uint32_t mode_idx = 0;
	     mode_idx < ARRAY_SIZE(pimx623_mode_info); mode_idx++) {
		if (pimx623_mode_info[mode_idx].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &pimx623_mode_info[mode_idx],
			       sizeof(IsiSensorMode_t));
			TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
			return RET_SUCCESS;
		}
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_NOTSUPP;
}

/*****************************************************************************
 *          IMX623_SetClknStartup
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
static RESULT IMX623_SetClknStartup(IsiSensorHandle_t handle)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	RESULT result = RET_SUCCESS;

	/* Ensure the sensor is in a valid startup state */
	SensorState_e sensorState = IMX623_GetSensorState(handle);

	if (!(sensorState == IMX623_SENSOR_STATE_STARTUP_IMMEDIATE ||
	      sensorState == IMX623_SENSOR_STATE_STARTUP)) {
		TRACE(IMX623_ERROR,
		      "%s: INCK can only be set in Startup state. Current state: %d\n",
		      __func__, sensorState);
		return RET_FAILURE;
	}

	/* Set input clock frequency (INCK_FREQ) */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_LL,
				       (FMC_CLK_HZ >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_LH,
					(FMC_CLK_HZ >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_HL,
					(FMC_CLK_HZ >> 16) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_HH,
					(FMC_CLK_HZ >> 24) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set input clock frequency (INCK_FREQ)! (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_LL,
				       (FMC_CLK_HZ >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_LH,
					(FMC_CLK_HZ >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_HL,
					(FMC_CLK_HZ >> 16) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_HH,
					(FMC_CLK_HZ >> 24) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set input clock frequency (INCK_FREQ_EXC)! (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Enable input clock frequency */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EN, 0x1);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to enable input clock frequency! (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	osSleep(60);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetRevisionIss
 *
 * @brief   This function reads the sensor ID registers and returns it.
 *
 * @param   handle      sensor instance handle
 * @param   pValue      pointer to sensor ID
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT IMX623_IsiGetRevisionIss(IsiSensorHandle_t handle,
					uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val = 0;
	uint32_t sensor_id = 0;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (pValue == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer for pValue\n", __func__);
		return RET_NULL_POINTER;
	}

	/* Read CHIP_ID2 */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_CHIP_ID2, &reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read CHIP_ID2 (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	sensor_id = (reg_val & 0xFF) << 16;

	/* Read CHIP_ID1 */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_CHIP_ID1, &reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read CHIP_ID1 (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	sensor_id |= ((reg_val & 0xFF) << 8);

	/* Read CHIP_ID0 */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_CHIP_ID0, &reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read CHIP_ID0 (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	sensor_id |= (reg_val & 0xFF);

	*pValue = sensor_id;

	TRACE(IMX623_DEBUG, "%s: Sensor ID = 0x%08X\n", __func__, sensor_id);
	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX623_IsiCheckConnectionIss
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
static RESULT IMX623_IsiCheckConnectionIss(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	TRACE(IMX623_INFO, "============ IMX623 ============\n");

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer)\n", __func__);
		return RET_NULL_POINTER;
	}

	pIMX623Ctx->regAccessEnable = BOOL_TRUE;

	result = IMX623_SetClknStartup(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_WARN,
		      "%s: Sensor power-up and standby init failed (err=%d)\n",
		      __func__, result);
		// return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "IMX623 clock setup completed successfully\n");

	uint32_t sensor_id = 0;
	uint32_t correct_id = IMX623_SENSOR_ID;

	result = IMX623_IsiGetRevisionIss(handle, &sensor_id);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read sensor ID (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "IMX623 sensor ID read successfully\n");

	if (correct_id != sensor_id) {
		TRACE(IMX623_ERROR,
		      "%s: Sensor ID mismatch - Expected: 0x%08x, Read: 0x%08x\n",
		      __func__, correct_id, sensor_id);
		return RET_FAILURE;
	}

	TRACE(IMX623_INFO,
	      "%s: Sensor ID verification successful - ID: 0x%08x\n",
	      __func__, sensor_id);
	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}


/*****************************************************************************
 *          IMX623_IsiSetFpsIss
 *
 * @brief   set Sensor Fps Config.
 *
 * @param   handle                  sensor instance handle
 * @param   fps                     Setfps
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	SensorState_e sensorState = IMX623_GetSensorState(handle);

	if (!(sensorState == IMX623_SENSOR_STATE_STARTUP_IMMEDIATE ||
	      sensorState == IMX623_SENSOR_STATE_STARTUP)) {
		TRACE(IMX623_ERROR,
		      "%s: FPS can only be set in Startup state. Current state: %d\n",
		      __func__, sensorState);
		return RET_WRONG_STATE;
	}

	if (fps > pIMX623Ctx->maxFps) {
		TRACE(IMX623_ERROR,
		      "%s: FPS %u exceeds maximum allowed %u\n",
		      __func__, fps / ISI_FPS_QUANTIZE,
		      pIMX623Ctx->maxFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}
	if (fps < pIMX623Ctx->minFps) {
		TRACE(IMX623_ERROR,
		      "%s: FPS %u is below minimum allowed %u\n",
		      __func__, fps / ISI_FPS_QUANTIZE,
		      pIMX623Ctx->minFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}

	fps /= ISI_FPS_QUANTIZE;
	uint32_t vmax_offset = ((30 * IMX623_VMAX_30FPS) / fps) - IMX623_VMAX_30FPS;

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_VMAX_OFFSET_L,
				       (vmax_offset >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to write VMAX_OFFSET low byte (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_VMAX_OFFSET_M,
				       (vmax_offset >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to write VMAX_OFFSET mid byte (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_VMAX_OFFSET_H,
					0b00000011, (vmax_offset >> 16) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to update VMAX_OFFSET high byte (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetFpsIss
 *
 * @brief   Get Sensor Fps Config.
 *
 * @param   handle                   sensor instance handle
 * @param   pFps                     current fps
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	uint32_t vmax = 0;
	uint16_t reg_value = 0;

	/* Read VMAX low byte */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_L, &reg_value);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to read VMAX low byte register (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	vmax = reg_value & 0xFF;

	/* Read VMAX mid byte */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_M, &reg_value);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to read VMAX mid byte register (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	vmax |= (reg_value & 0xFF) << 8;

	/* Read VMAX high byte */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_H, &reg_value);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to read VMAX high byte register (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	vmax |= (reg_value & 0xFF) << 16;

	uint32_t vmax_offset = 0;

	/* Read VMAX_OFFSET low byte */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_OFFSET_L, &reg_value);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to read VMAX_OFFSET low byte (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	vmax_offset = reg_value & 0xFF;

	/* Read VMAX_OFFSET mid byte */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_OFFSET_M, &reg_value);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to read VMAX_OFFSET mid byte (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	vmax_offset |= (reg_value & 0xFF) << 8;

	/* Read VMAX_OFFSET high byte */
	result = IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_OFFSET_H, &reg_value);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to read VMAX_OFFSET high byte (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
	vmax_offset |= (reg_value & 0xFF) << 16;

	/* Calculate FPS: FPS = (30 * VMAX_30FPS) / (VMAX + VMAX_OFFSET) */
	*pFps = (float)((30.0f * IMX623_VMAX_30FPS) / (vmax + vmax_offset)) *
		ISI_FPS_QUANTIZE;

	/* Update frame counter and sensor status */
	imx623_sensor_framecount(handle);
	Fmc_Sensor_Statustask();

	TRACE(IMX623_DEBUG,
	      "%s: IMX623 FPS: %u\n", __func__, *pFps / ISI_FPS_QUANTIZE);
	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_ConfigureBLC
 *
 * @brief   Configures the sensor's Black Level Correction (BLC) module.
 *          When enabled, applies user-defined and sensor black level
 *          subtraction values. When disabled, BLC is bypassed.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_ConfigureBLC(IsiSensorHandle_t handle)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_NULL_POINTER;
	}

	enum imx623_blsmode_e blsMode = IMX623_BLSMODE_USER_SENSOR;

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_BLS_MODE_SELECT, blsMode);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_BLS_CONTROL_SELECT, 1);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set black level subtraction mode (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	IsiSensorBlc_t pBlc = {0, 0, 0, 0};

	result = IMX623_IsiSetBlcIss(handle, &pBlc);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_InitialExposure
 *
 * @brief   Initialize fixed exposure settings.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_InitialExposure(IsiSensorHandle_t handle)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_NULL_POINTER;
	}

	/* Set initial exposure time */
	const float initial_exposure_time_sec = pIMX623Ctx->aecMinIntegrationTime;

	result = IMX623_SetIntTime(handle, initial_exposure_time_sec);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set initial exposure time to %.2f seconds (err=%d)\n",
		      __func__, initial_exposure_time_sec, result);
		return RET_FAILURE;
	}

	/* Set initial analog gain to 7.94 (18dB) */
	IsiSensorGain_t sensor_a_gain;

	sensor_a_gain.gain[ISI_LINEAR_PARAS] = pIMX623Ctx->aGain.min;

	result = IMX623_IsiSetAGainIss(handle, &sensor_a_gain);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set initial analog gain to %.2f (err=%d)\n",
		      __func__, sensor_a_gain.gain[ISI_LINEAR_PARAS], result);
		return RET_FAILURE;
	}

	/* Set initial digital gain to 1.0 (0dB) */
	IsiSensorGain_t sensor_d_gain;

	sensor_d_gain.gain[ISI_LINEAR_PARAS] = pIMX623Ctx->dGain.min;

	result = IMX623_IsiSetDGainIss(handle, &sensor_d_gain);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set initial digital gain to %.2f (err=%d)\n",
		      __func__, sensor_d_gain.gain[ISI_LINEAR_PARAS], result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_Configure2A
 *
 * @brief   Configures the sensor's Auto Exposure (AE) and Auto White Balance
 *          (AWB) settings. Sets both modules to full manual mode and applies
 *          initial exposure and white balance values.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_Configure2A(IsiSensorHandle_t handle)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_NULL_POINTER;
	}

	/* Set exposure time units to microseconds for SP1 and SP2 sub-pixels */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_UNIT_SP1,
				       IMX623_FME_SHTVAL_UNIT_MICROSECONDS);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_UNIT_SP2,
					IMX623_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to configure exposure time units for SP1/SP2 (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Configure Auto Exposure to full manual mode */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AE_MODE,
				       IMX623_AEMODE_FULL_ME);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to configure AE to full manual mode (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Apply initial exposure settings */
	result = IMX623_InitialExposure(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to apply initial exposure settings (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Configure Auto White Balance to full manual mode */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AWB_MODE,
				       IMX623_AWBMODE_FULL_MWB);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to configure AWB to full manual mode (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Apply initial white balance gains with unity values */
	IsiSensorWb_t initialWB = {1.0f, 1.0f, 1.0f, 1.0f};

	result = IMX623_IsiSetWBIss(handle, &initialWB);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to apply initial white balance gains (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_ConfigureHDR
 *
 * @brief   Configures HDR settings, including composite region switching,
 *          companding curve control points, HDR compression enablement,
 *          and sub-pixel output selection.
 *
 * @param   handle      Sensor instance handle
 * @param   points      Pointer to HDR control points structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_ConfigureHDR(IsiSensorHandle_t handle,
				  struct imx623_ctrl_point *points)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	/*
	 * Configure composite region switching to pixel value switching.
	 * Use IMX623_LUMINANCE_SWITCHING if color shading avoidance
	 * is prioritized over SNR.
	 */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_SIGNAL_SWITCH_0,
				       IMX623_PIXEL_VALUE_SWITCHING);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_SIGNAL_SWITCH_1,
					IMX623_PIXEL_VALUE_SWITCHING);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_SIGNAL_SWITCH_2,
					IMX623_PIXEL_VALUE_SWITCHING);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to configure pixel value switching (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Configure PWL companding curve control points (X=input, Y=output) */
	for (int ctrl_pt_idx = 0; ctrl_pt_idx < 16; ctrl_pt_idx++) {
		/* Write X coordinate (24-bit, little-endian) */
		result = IMX623_IsiWriteRegIss(handle,
			IMX623_REG_CTRL_POINT_X(ctrl_pt_idx),
			(points->x >> 0) & 0xFF);
		result |= IMX623_IsiWriteRegIss(handle,
			IMX623_REG_CTRL_POINT_X(ctrl_pt_idx) + 1,
			(points->x >> 8) & 0xFF);
		result |= IMX623_IsiWriteRegIss(handle,
			IMX623_REG_CTRL_POINT_X(ctrl_pt_idx) + 2,
			(points->x >> 16) & 0xFF);

		/* Write Y coordinate (24-bit, little-endian) */
		result |= IMX623_IsiWriteRegIss(handle,
			IMX623_REG_CTRL_POINT_Y(ctrl_pt_idx),
			(points->y >> 0) & 0xFF);
		result |= IMX623_IsiWriteRegIss(handle,
			IMX623_REG_CTRL_POINT_Y(ctrl_pt_idx) + 1,
			(points->y >> 8) & 0xFF);
		result |= IMX623_IsiWriteRegIss(handle,
			IMX623_REG_CTRL_POINT_Y(ctrl_pt_idx) + 2,
			(points->y >> 16) & 0xFF);

		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR,
			      "%s: Failed to write control point %d. (err=%d)\n",
			      __func__, ctrl_pt_idx, result);
			return RET_FAILURE;
		}

		if ((points+1)->x >= 0 && (points+1)->y >= 0)
			points++;
	}

	/* Enable HDR compression mode */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_HDRON, IMX623_IMG_MODE_HDR);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_HDRON_APL,
					IMX623_IMG_MODE_HDR);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to enable HDR compression mode. (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Configure sub-pixel output selection to SP1 HCG */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTSEL_1,
				       IMX623_RAW_OUTSEL_SP1_HCG);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTSEL_1_APL,
					IMX623_RAW_OUTSEL_SP1_HCG);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to configure sub-pixel output selection. (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_Configure
 *
 * @brief   Configures the sensor with default settings for operation.
 *
 * @param   handle      Sensor instance handle.
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT IMX623_Configure(IsiSensorHandle_t handle)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	/* Set sensor drive mode register */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MODE_SET_F_L, 0x1D);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set drive mode register (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Unlock MODE_SET_F register for configuration */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MODE_SET_LOCK, 0x53);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to unlock drive mode register (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Configure Black Level Correction (BLC) module */
	result = IMX623_ConfigureBLC(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to configure Black Level Correction (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	/* Configure Auto Exposure (AE) and Auto White Balance (AWB) */
	result = IMX623_Configure2A(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to configure AE/AWB settings (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Configure HDR settings (composite regions, PWL curve) */
	result = IMX623_ConfigureHDR(handle, imx623_hdr_24bit);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to configure HDR settings (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Disable embedded metadata output (front and rear) */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MD_FEBD, 0x00);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_MD_REBD, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to disable embedded metadata (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Configure ADC to 12-bit resolution */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_ADBIT, IMX623_AD_12BIT);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set ADC bit depth to 12-bit (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Configure output mode to standard RAW */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTMODE,
				       IMX623_OUTMODE_RAW);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTMODE_APL,
					IMX623_OUTMODE_RAW);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set output mode to RAW (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Configure RAW output format to 12-bit */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_RAW_OUTMODE,
				       IMX623_RAW_OUTMODE_RAW12);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_RAW_OUTMODE_APL,
					IMX623_RAW_OUTMODE_RAW12);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set the RAW output mode to 12-bit (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

static RESULT IMX623_Crop(IsiSensorHandle_t handle, uint32_t boundsWidth,
			  uint32_t boundsHeight, uint32_t left, uint32_t top,
			  uint32_t width, uint32_t height)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	/* Validate bounds against sensor maximum resolution */
	if (boundsWidth != IMX623_MAX_WIDTH || boundsHeight != IMX623_MAX_HEIGHT) {
		TRACE(IMX623_ERROR,
		      "%s: Bounds %ux%u do not match sensor max %ux%u\n",
		      __func__, boundsWidth, boundsHeight,
		      IMX623_MAX_WIDTH, IMX623_MAX_HEIGHT);
		return RET_FAILURE;
	}

	/* Ensure output resolution meets minimum requirements */
	if (width < IMX623_MIN_WIDTH || height < IMX623_MIN_HEIGHT) {
		TRACE(IMX623_ERROR,
		      "%s: Resolution %ux%u below minimum %ux%u\n",
		      __func__, width, height,
		      IMX623_MIN_WIDTH, IMX623_MIN_HEIGHT);
		return RET_FAILURE;
	}

	/* Skip cropping if full resolution is requested */
	if (boundsWidth == width && boundsHeight == height)
		return result;
	/* Validate crop window fits within sensor bounds */
	if (boundsWidth < width || boundsHeight < height ||
	    (left + width) > boundsWidth || (top + height) > boundsHeight) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid crop region (left=%u, top=%u, w=%u, h=%u) bounds %ux%u\n",
		      __func__, left, top, width, height,
		      boundsWidth, boundsHeight);
		return RET_FAILURE;
	}

	uint16_t horizontal_offset = left;
	uint16_t vertical_offset = top;
	uint16_t horizontal_size = width;
	uint16_t vertical_size = height;

	/* Enable digital cropping */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_ON, 0x01);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_ON_APL, 0x01);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to enable digital cropping (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	/* Set horizontal crop offset (16-bit, little-endian) */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_L,
				       (horizontal_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_H,
					(horizontal_offset >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_L_APL,
					(horizontal_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_H_APL,
					(horizontal_offset >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set horizontal crop offset to %u (err=%d)\n",
		      __func__, horizontal_offset, result);
		return RET_FAILURE;
	}

	/* Set vertical crop offset (16-bit, little-endian) */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_L,
				       (vertical_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_H,
					(vertical_offset >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_L_APL,
					(vertical_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_H_APL,
					(vertical_offset >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set vertical crop offset to %u (err=%d)\n",
		      __func__, vertical_offset, result);
		return RET_FAILURE;
	}

	/* Set horizontal crop size (16-bit, little-endian) */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_L,
				       (horizontal_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_H,
					(horizontal_size >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_L_APL,
					(horizontal_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_H_APL,
					(horizontal_size >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set horizontal crop size to %u (err=%d)\n",
		      __func__, horizontal_size, result);
		return RET_FAILURE;
	}

	/* Set vertical crop size (16-bit, little-endian) */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_L,
				       (vertical_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_H,
					(vertical_size >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_L_APL,
					(vertical_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_H_APL,
					(vertical_size >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set vertical crop size to %u (err=%d)\n",
		      __func__, vertical_size, result);
		return RET_FAILURE;
	}

	/* Select custom digital crop data source */
	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_DCROP_DATA_SEL,
					0b00000001, 0x01);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to select custom digital crop data (err=%d)\n",
		      __func__, result);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiOpenIss
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
static RESULT IMX623_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer)\\n", __func__);
		return RET_NULL_POINTER;
	}


	if (pIMX623Ctx->streaming != BOOL_FALSE)
		return RET_WRONG_STATE;

	pIMX623Ctx->regAccessEnable = BOOL_TRUE;

	pIMX623Ctx->sensorMode.index = mode;
	IsiSensorMode_t *SensorDefaultMode = NULL;

	/* Find the matching sensor mode from the predefined modes */
	for (uint32_t mode_idx = 0;
	     mode_idx < ARRAY_SIZE(pimx623_mode_info); mode_idx++) {
		if (pimx623_mode_info[mode_idx].index == pIMX623Ctx->sensorMode.index) {
			SensorDefaultMode = &(pimx623_mode_info[mode_idx]);
			break;
		}
	}

	if (SensorDefaultMode != NULL) {
		/* Initialize sensor registers from the init table */
		for (uint32_t reg_idx = 0;
		     reg_idx < ARRAY_SIZE(IMX623_init); reg_idx++) {
			if (IMX623_init[reg_idx][0] == IMX623_TABLE_WAIT) {
				osSleep(IMX623_init[reg_idx][1]);
			} else if (IMX623_init[reg_idx][0] == IMX623_TABLE_END) {
				break;
			} else if (IMX623_init[reg_idx][0] == IMX623_TABLE_REMAP) {
				IMX623_IsiWriteRegIss(handle, IMX623_REG_REGMAP,
						    IMX623_REMAP_MODE_STARTUP);
				osSleep(10);
			} else {
				IMX623_IsiWriteRegIss(handle, IMX623_init[reg_idx][0],
						     IMX623_init[reg_idx][1]);
			}
		}
		osSleep(1);
		memcpy(&(pIMX623Ctx->sensorMode), SensorDefaultMode,
		       sizeof(IsiSensorMode_t));
	} else {
		TRACE(IMX623_ERROR,
		      "%s: No matching sensor mode found for index %u\n",
		      __func__, mode);
		return RET_NULL_POINTER;
	}

	/* Configure exposure time range (in seconds) */
	pIMX623Ctx->aecMinIntegrationTime  = IMX623_MIN_EXP_TIME;
	pIMX623Ctx->aecMaxIntegrationTime  = IMX623_MAX_EXP_TIME;
	pIMX623Ctx->aecIntegrationTimeStep = 1.0f / SEC_TO_MICROSEC;

	/* Configure analog gain range (SP1 HCG) */
	pIMX623Ctx->aGain.min  = IMX623_SP1_HCG_MIN_AGAIN;
	pIMX623Ctx->aGain.max  = 9.76f;
	pIMX623Ctx->aGain.step = 0.01f;

	/* Configure digital gain range */
	pIMX623Ctx->dGain.min  = IMX623_MIN_DGAIN;
	pIMX623Ctx->dGain.max  = IMX623_MAX_DGAIN;
	pIMX623Ctx->dGain.step = 0.001f;

	/* Calculate total gain range (analog * digital) */
	pIMX623Ctx->aecMinGain       = pIMX623Ctx->aGain.min * pIMX623Ctx->dGain.min;
	pIMX623Ctx->aecMaxGain       = pIMX623Ctx->aGain.max * pIMX623Ctx->dGain.max;
	pIMX623Ctx->aecGainIncrement = 0.001f;

	/* Configure white balance gain range (12-bit: 0x100 = 1.0) */
	pIMX623Ctx->minWBGain = 0x001;
	pIMX623Ctx->maxWBGain = 0xFFF;

	/* Configure frame rate range */
	pIMX623Ctx->maxFps  = pIMX623Ctx->sensorMode.fps;
	pIMX623Ctx->minFps  = 0.25f * ISI_FPS_QUANTIZE;
	pIMX623Ctx->currFps = pIMX623Ctx->maxFps;

	/* Configure sensor ISP usage flags */
	pIMX623Ctx->sensorConfig.useSensorAE = false;
	pIMX623Ctx->sensorConfig.useAWBMode = ISI_USE_ISP_WB_GAIN;
	/* can be enabled if bls mode is IMX623_BLSMODE_USER */
	pIMX623Ctx->sensorConfig.useSensorBLC = false;

	TRACE(IMX623_DEBUG,
	      "%s: IMX623 system reset and initialization completed\n",
	      __func__);

	result = IMX623_Configure(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Sensor configuration failed (err=%d)\n", __func__, result);
		return result;
	}

	result = IMX623_Crop(handle,
		SensorDefaultMode->size.boundsWidth,
		SensorDefaultMode->size.boundsHeight,
		SensorDefaultMode->size.left,
		SensorDefaultMode->size.top,
		SensorDefaultMode->size.width,
		SensorDefaultMode->size.height);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to configure sensor cropping (err=%d)\n", __func__, result);
		return result;
	}

	pIMX623Ctx->configured = BOOL_TRUE;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetModeIss
 *
 * @brief   Get current sensor mode information.
 *
 * @param   handle     Sensor instance handle
 * @param   pMode      Pointer to sensor mode structure to populate
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX623_IsiGetModeIss(IsiSensorHandle_t handle,
				    IsiSensorMode_t *pMode)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;

	if (pIMX623Ctx == NULL || pMode == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	memcpy(pMode, &(pIMX623Ctx->sensorMode), sizeof(pIMX623Ctx->sensorMode));

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX623_IsiGetCapsIss
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
static RESULT IMX623_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (pIMX623Ctx == NULL || pCaps == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	pCaps->bitWidth          = pIMX623Ctx->sensorMode.bitWidth;
	pCaps->mode              = ISI_MODE_BAYER;
	pCaps->bayerPattern      = pIMX623Ctx->sensorMode.bayerPattern;
	pCaps->resolution.width  = pIMX623Ctx->sensorMode.size.width;
	pCaps->resolution.height = pIMX623Ctx->sensorMode.size.height;
	pCaps->mipiLanes         = ISI_MIPI_4LANES;
	pCaps->vinType           = ISI_ITF_TYPE_MIPI;

	if (pCaps->bitWidth == 10)
		pCaps->mipiMode = ISI_FORMAT_RAW_10;
	else if (pCaps->bitWidth == 12)
		pCaps->mipiMode = ISI_FORMAT_RAW_12;
	else
		pCaps->mipiMode = ISI_MIPI_OFF;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetTpgIss
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
RESULT IMX623_IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *pTpg)
{

	RESULT result = RET_SUCCESS;
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL || pTpg == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}
	return result;
}

/**************************************************************************
 *          IMX623_IsiSetTpgIss
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
RESULT IMX623_IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg)
{
	RESULT result = RET_SUCCESS;
	(void)tpg;    /* Unused parameter */
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	return result;
}

/*****************************************************************************
 *          IMX623_pIsiGetAeBaseInfoIss
 *
 * @brief   Returns the Ae base info of a sensor
 *          instance
 *
 * @param   handle        sensor instance handle
 * @param   pAeBaseInfo   Pointer to the sensor aebase info value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX623_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle,
					  IsiAeBaseInfo_t *pAeBaseInfo)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	if (pIMX623Ctx == NULL || pAeBaseInfo == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	/* AE base info with minimum and maximum gain values */
	pAeBaseInfo->longGain.min = pIMX623Ctx->aecMinGain;
	pAeBaseInfo->longGain.max = pIMX623Ctx->aecMaxGain;

	/* AE base info with minimum and maximum integration time */
	pAeBaseInfo->longIntTime.min = pIMX623Ctx->aecMinIntegrationTime;
	pAeBaseInfo->longIntTime.max = pIMX623Ctx->aecMaxIntegrationTime;

	pAeBaseInfo->aLongGain = pIMX623Ctx->aGain;
	pAeBaseInfo->dLongGain = pIMX623Ctx->dGain;

	pAeBaseInfo->curIntTime = pIMX623Ctx->curIntTime;

	/* Calculate current total gain */
	pAeBaseInfo->curGain.gain[0] =
		pIMX623Ctx->curAgain.gain[0] * pIMX623Ctx->curDgain.gain[0];
	pAeBaseInfo->curGain.gain[1] =
		pIMX623Ctx->curAgain.gain[1] * pIMX623Ctx->curDgain.gain[1];
	pAeBaseInfo->curGain.gain[2] =
		pIMX623Ctx->curAgain.gain[2] * pIMX623Ctx->curDgain.gain[2];
	pAeBaseInfo->curGain.gain[3] =
		pIMX623Ctx->curAgain.gain[3] * pIMX623Ctx->curDgain.gain[3];

	pAeBaseInfo->aecIntTimeStep = pIMX623Ctx->aecIntegrationTimeStep;
	pAeBaseInfo->aecGainStep = pIMX623Ctx->aecGainIncrement;
	pAeBaseInfo->nativeMode = pIMX623Ctx->sensorMode.nativeMode;
	pAeBaseInfo->conversionGainDCG =
		(float)IMX623_SP1_HCG_MIN_AGAIN / IMX623_SP1_LCG_MIN_AGAIN;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   mode          new streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT IMX623_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t mode)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer)\n", __func__);
		return RET_NULL_POINTER;
	}


	if (pIMX623Ctx->configured != BOOL_TRUE) {
		TRACE(IMX623_ERROR,
		      "%s: Sensor not configured - cannot change streaming\n",
		      __func__);
		return RET_WRONG_STATE;
	}

	pIMX623Ctx->regAccessEnable = BOOL_TRUE;

	/* Unlock the MODE_SET_F register to allow modifications */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MODE_SET_LOCK, 0x53);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to unlock MODE_SET_F register\n", __func__);
		return RET_FAILURE;
	}

	/* Set streaming mode: bit 7 controls stream enable (1=start, 0=stop) */
	if (mode == true) {
		result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_MODE_SET_F_H,
						0b10000000, 0x80);
		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR,
			      "%s: Failed to start sensor streaming\n",
			      __func__);
			return RET_FAILURE;
		}
	} else {
		result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_MODE_SET_F_H,
						0b10000000, 0x00);
		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR,
			      "%s: Failed to stop sensor streaming\n",
			      __func__);
			return RET_FAILURE;
		}
	}

	pIMX623Ctx->streaming = mode;

	TRACE(IMX623_INFO,
	      "%s: streaming=%s mode=%d res=%dx%d fps=%d\n",
	      __func__, mode ? "ON" : "OFF",
	      pIMX623Ctx->sensorMode.index,
	      pIMX623Ctx->sensorMode.size.width,
	      pIMX623Ctx->sensorMode.size.height,
	      pIMX623Ctx->currFps / ISI_FPS_QUANTIZE);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiCloseIss
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
static RESULT IMX623_IsiCloseIss(IsiSensorHandle_t handle)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	result = IMX623_IsiSetStreamingIss(handle, BOOL_FALSE);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to stop streaming during close (err=%d)\n", __func__, result);
		return result;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiReleaseIss
 *
 * @brief   Release the image sensor instance and free resources.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT IMX623_IsiReleaseIss(IsiSensorHandle_t handle)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle provided\n", __func__);
		return RET_NULL_POINTER;
	}

	/* Stop the sensor stream */
	stop_sensor(pIMX623Ctx->sensorDevId);
	osMutexDestroy(&pIMX623Ctx->registerLock);


	/* Clear the context structure and free memory */
	MEMSET(pIMX623Ctx, 0, sizeof(IMX623_Context_t));
	osFree(pIMX623Ctx);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiSetWBIss
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
static RESULT IMX623_IsiSetWBIss(IsiSensorHandle_t handle,
				 const IsiSensorWb_t *pWb)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL || !pWb) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	/* Calculate and clamp white balance gains */
	uint16_t gain_r = (uint16_t)(pWb->rGain * IMX623_WB_SCALING_FACTOR);
	uint16_t gain_gr = (uint16_t)(pWb->grGain * IMX623_WB_SCALING_FACTOR);
	uint16_t gain_gb = (uint16_t)(pWb->gbGain * IMX623_WB_SCALING_FACTOR);
	uint16_t gain_b = (uint16_t)(pWb->bGain * IMX623_WB_SCALING_FACTOR);

	gain_r = MAX(MIN(gain_r, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);
	gain_gr = MAX(MIN(gain_gr, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);
	gain_gb = MAX(MIN(gain_gb, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);
	gain_b = MAX(MIN(gain_b, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];
	/* Set red channel gain */
	group_values[0] = (gain_r >> 0) & 0xFF;
	group_values[1] = (gain_r >> 8) & 0xFF;
	/* Set green-red channel gain */
	group_values[2] = (gain_gr >> 0) & 0xFF;
	group_values[3] = (gain_gr >> 8) & 0xFF;
	result = IMX623_IsiWriteRegGroupIss(handle, IMX623_REG_FULLMWBGAIN_R_L,
					    group_values, 2U);

	/* Set green-blue channel gain */
	group_values[0] = (gain_gb >> 0) & 0xFF;
	group_values[1] = (gain_gb >> 8) & 0xFF;
	/* Set blue channel gain */
	group_values[2] = (gain_b >> 0) & 0xFF;
	group_values[3] = (gain_b >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle, IMX623_REG_FULLMWBGAIN_GB_L,
					     group_values, 2U);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set white balance gains (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}
#else
	/* Set red channel gain */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_R_L,
				       (gain_r >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_R_H,
					(gain_r >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set WB gain for red channel\n", __func__);
		return RET_FAILURE;
	}

	/* Set green-red channel gain */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GR_L,
				       (gain_gr >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GR_H,
					(gain_gr >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set WB gain for green-red channel\n",
		      __func__);
		return RET_FAILURE;
	}

	/* Set green-blue channel gain */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GB_L,
				       (gain_gb >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GB_H,
					(gain_gb >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set WB gain for green-blue channel\n",
		      __func__);
		return RET_FAILURE;
	}

	/* Set blue channel gain */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_B_L,
				       (gain_b >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_B_H,
					(gain_b >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set WB gain for blue channel\n", __func__);
		return RET_FAILURE;
	}
#endif

	/* Update context with new white balance values */
	memcpy(&pIMX623Ctx->sensorWb, pWb, sizeof(IsiSensorWb_t));

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetWBIss
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
static RESULT IMX623_IsiGetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;

	if (pIMX623Ctx == NULL || !pWb) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	memcpy(pWb, &pIMX623Ctx->sensorWb, sizeof(IsiSensorWb_t));

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX623_IsiGetAGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                    sensor instance handle
 * @param   pSensorAGain              pointer to sensor again to get
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetAGainIss(IsiSensorHandle_t handle,
			     IsiSensorGain_t *pSensorAGain)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx || !pSensorAGain) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	*pSensorAGain = pIMX623Ctx->curAgain;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX623_IsiSetAGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *
 * @param   handle                  sensor instance handle
 * @param   pSensorAGain            pointer to sensor again to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiSetAGainIss(IsiSensorHandle_t handle,
			     IsiSensorGain_t *pSensorAGain)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx || !pSensorAGain) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] < pIMX623Ctx->aGain.min) {
		TRACE(IMX623_WARN,
		      "%s: Analog gain value too small, clamping to min\n",
		      __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->aGain.min;
	}

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] > pIMX623Ctx->aGain.max) {
		TRACE(IMX623_WARN,
		      "%s: Analog gain value too large, clamping to max\n",
		      __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->aGain.max;
	}

	float32_t sp1_hcg = pSensorAGain->gain[ISI_LINEAR_PARAS];
	float32_t sp1_lcg = sp1_hcg *
		IMX623_SP1_LCG_MIN_AGAIN / IMX623_SP1_HCG_MIN_AGAIN;
	float32_t sp2_h = sp1_hcg *
		IMX623_SP2_H_MIN_AGAIN / IMX623_SP1_HCG_MIN_AGAIN;
	float32_t sp2_l = sp1_hcg *
		IMX623_SP2_L_MIN_AGAIN / IMX623_SP1_HCG_MIN_AGAIN;

	uint16_t sp1_hcg_gain;
	uint16_t sp1_lcg_gain;
	uint16_t sp2_h_gain;
	uint16_t sp2_l_gain;

	sp1_hcg_gain = (uint16_t)(IMX623_LINEAR_TO_DB(sp1_hcg) *
				  IMX623_AG_DG_SCALING_FACTOR);
	sp1_lcg_gain = (uint16_t)(IMX623_LINEAR_TO_DB(sp1_lcg) *
				  IMX623_AG_DG_SCALING_FACTOR);
	sp2_h_gain = (uint16_t)(IMX623_LINEAR_TO_DB(sp2_h) *
				IMX623_AG_DG_SCALING_FACTOR);
	sp2_l_gain = (uint16_t)(IMX623_LINEAR_TO_DB(sp2_l) *
				IMX623_AG_DG_SCALING_FACTOR);

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];
	/* Write analog gains for SP1 HCG */
	group_values[0] = (sp1_hcg_gain >> 0) & 0xFF;
	group_values[1] = (sp1_hcg_gain >> 8) & 0x3;
	/* Write analog gains for SP1 LCG */
	group_values[2] = (sp1_lcg_gain >> 0) & 0xFF;
	group_values[3] = (sp1_lcg_gain >> 8) & 0x3;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX623_INFO,
	      "%s: Before write - SP1: 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
	      group_values[0], group_values[1], group_values[2], group_values[3]);
#endif
	result = IMX623_IsiWriteRegGroupIss(handle, IMX623_REG_AGAIN_SP1_HCG_L,
					    group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back SP1 values */
	uint16_t read_value = 0;
	uint8_t read_values[4] = {0};

	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP1_HCG_L, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP1_HCG_H, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP1_LCG_L, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP1_LCG_H, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(IMX623_INFO,
	      "%s: After write - SP1: 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
	      read_values[0], read_values[1], read_values[2], read_values[3]);
#endif

	/* Write analog gains for SP2 H */
	group_values[0] = (sp2_h_gain >> 0) & 0xFF;
	group_values[1] = (sp2_h_gain >> 8) & 0x3;
	/* Write analog gains for SP2 L */
	group_values[2] = (sp2_l_gain >> 0) & 0xFF;
	group_values[3] = (sp2_l_gain >> 8) & 0x3;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX623_INFO,
	      "%s: Before write - SP2: 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
	      group_values[0], group_values[1], group_values[2], group_values[3]);
#endif
	result |= IMX623_IsiWriteRegGroupIss(handle, IMX623_REG_AGAIN_SP2_H_L,
					     group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back SP2 values */
	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP2_H_L, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP2_H_H, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP2_L_L, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_AGAIN_SP2_L_H, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(IMX623_INFO,
	      "%s: After write - SP2: 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
	      read_values[0], read_values[1], read_values[2], read_values[3]);
#endif
#else
	/* Write analog gains for SP1 HCG */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP1_HCG_L,
				       (sp1_hcg_gain >> 0) & 0xFF);
	result |= IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP1_HCG_H,
					 0b00000011, (sp1_hcg_gain >> 8) & 0xFF);

	/* Write analog gains for SP1 LCG */
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP1_LCG_L,
					(sp1_lcg_gain >> 0) & 0xFF);
	result |= IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP1_LCG_H,
					 0b00000011, (sp1_lcg_gain >> 8) & 0xFF);

	/* Write analog gains for SP2 H */
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP2_H_L,
					(sp2_h_gain >> 0) & 0xFF);
	result |= IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP2_H_H,
					 0b00000011, (sp2_h_gain >> 8) & 0xFF);

	/* Write analog gains for SP2 L */
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP2_L_L,
					(sp2_l_gain >> 0) & 0xFF);
	result |= IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP2_L_H,
					 0b00000011, (sp2_l_gain >> 8) & 0xFF);
#endif

	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set analog gain values\n", __func__);
		return RET_FAILURE;
	}

	/* Update context with current analog gains */
	pIMX623Ctx->curAgain.gain[0] = sp1_hcg;
	pIMX623Ctx->curAgain.gain[1] = sp1_lcg;
	pIMX623Ctx->curAgain.gain[2] = sp2_h;
	pIMX623Ctx->curAgain.gain[3] = sp2_l;

	TRACE(IMX623_INFO,
	      "%s: frame=%d aGain=[%.3f,%.3f,%.3f,%.3f]\n",
	      __func__, g_Sensor_frame_count,
	      pIMX623Ctx->curAgain.gain[0],
	      pIMX623Ctx->curAgain.gain[1],
	      pIMX623Ctx->curAgain.gain[2],
	      pIMX623Ctx->curAgain.gain[3]);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetDGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorDGain             pointer to sensor dgain to get
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetDGainIss(IsiSensorHandle_t handle,
			      IsiSensorGain_t *pSensorDGain)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx || !pSensorDGain) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	*pSensorDGain = pIMX623Ctx->curDgain;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX623_IsiSetDGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorDGain             pointer to sensor dgain to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiSetDGainIss(IsiSensorHandle_t handle,
			      IsiSensorGain_t *pSensorDGain)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx || !pSensorDGain) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	/* Clamp digital gain to valid range */
	if (pSensorDGain->gain[ISI_LINEAR_PARAS] < pIMX623Ctx->dGain.min) {
		TRACE(IMX623_WARN,
		      "%s: Digital gain value too small, clamping to min\n",
		      __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->dGain.min;
	}

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] > pIMX623Ctx->dGain.max) {
		TRACE(IMX623_WARN,
		      "%s: Digital gain value too large, clamping to max\n",
		      __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->dGain.max;
	}

	uint16_t digital_gain;

	digital_gain = (uint16_t)(IMX623_LINEAR_TO_DB(
		pSensorDGain->gain[ISI_LINEAR_PARAS]) * IMX623_AG_DG_SCALING_FACTOR);

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[2U];
	/* Write digital gain registers (applies to all sub-pixels in HDR) */
	group_values[0] = (digital_gain >> 0) & 0xFF;
	group_values[1] = (digital_gain >> 8) & 0x3;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX623_INFO,
	      "%s: Before write: 0x%02X 0x%02X\n", __func__,
	      group_values[0], group_values[1]);
#endif
	result = IMX623_IsiWriteRegGroupIss(handle, IMX623_REG_DGAIN_L,
					    group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back values */
	uint16_t read_value = 0;
	uint8_t read_values[2] = {0};

	IMX623_IsiReadRegIss(handle, IMX623_REG_DGAIN_L, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_DGAIN_H, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(IMX623_INFO,
	      "%s: After write: 0x%02X 0x%02X\n", __func__,
	      read_values[0], read_values[1]);
#endif
#else
	/* Write digital gain registers (applies to all sub-pixels in HDR) */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DGAIN_L,
				       (digital_gain >> 0) & 0xFF);
	result |= IMX623_IsiUpdateRegIss(handle, IMX623_REG_DGAIN_H,
					 0b00000011, (digital_gain >> 8) & 0xFF);
#endif
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to write digital gain registers\n",
			__func__);
		return RET_FAILURE;
	}

	/* Update context with current digital gain values */
	pIMX623Ctx->curDgain.gain[0] = pSensorDGain->gain[ISI_LINEAR_PARAS];
	pIMX623Ctx->curDgain.gain[1] = pIMX623Ctx->curDgain.gain[0];
	pIMX623Ctx->curDgain.gain[2] = pIMX623Ctx->curDgain.gain[0];
	pIMX623Ctx->curDgain.gain[3] = pIMX623Ctx->curDgain.gain[0];

	TRACE(IMX623_INFO,
	      "%s: frame=%d dGain=[%.3f,%.3f,%.3f,%.3f]\n",
	      __func__, g_Sensor_frame_count,
	      pIMX623Ctx->curDgain.gain[0],
	      pIMX623Ctx->curDgain.gain[1],
	      pIMX623Ctx->curDgain.gain[2],
	      pIMX623Ctx->curDgain.gain[3]);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetIntTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorIntTime           pointer to integration time to get
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetIntTimeIss(IsiSensorHandle_t handle,
				IsiSensorIntTime_t *pSensorIntTime)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	RESULT result = RET_SUCCESS;
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *)handle;

	if (!pIMX623Ctx || !pSensorIntTime) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pIMX623Ctx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		*pSensorIntTime = pIMX623Ctx->curIntTime;
	} else {
		TRACE(IMX623_WARN, "%s: Not supported this HDR mode.\n", __func__);
		return RET_NOTSUPP;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiSetIntTimeIss
 *
 * @brief   Write integration time values to the image sensor module.
 *          Sets the exposure time for SP1 and SP2 sub-pixels.
 *
 * @param   handle                   Sensor instance handle
 * @param   pSensorIntTime           Pointer to sensor integration time to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT IMX623_IsiSetIntTimeIss(IsiSensorHandle_t handle,
				      const IsiSensorIntTime_t *pSensorIntTime)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx || !pSensorIntTime) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pIMX623Ctx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		result = IMX623_SetIntTime(handle,
					   pSensorIntTime->intTime[ISI_LINEAR_PARAS]);
		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR,
			      "%s: Failed to set sensor integration time\n",
			      __func__);
			return RET_FAILURE;
		}
	} else {
		TRACE(IMX623_INFO, "%s: Exposure frame type not supported\n", __func__);
		return RET_NOTSUPP;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_SetIntTime
 *
 * @brief   Set integration time (internal helper function).
 *
 * @param   handle                  Sensor instance handle
 * @param   newIntegrationTime      integration time settings
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT IMX623_SetIntTime(IsiSensorHandle_t handle,
				float newIntegrationTime)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer)\\n", __func__);
		return RET_NULL_POINTER;
	}

	/* Clamp integration time to valid range */
	newIntegrationTime = MAX(MIN(newIntegrationTime,
				     pIMX623Ctx->aecMaxIntegrationTime),
				 pIMX623Ctx->aecMinIntegrationTime);

	unsigned int exposure_sp1_us;
	unsigned int exposure_sp2_us;

	exposure_sp1_us = (unsigned int)(newIntegrationTime * SEC_TO_MICROSEC);
	exposure_sp2_us = exposure_sp1_us;

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];

	/* Set exposure time for SP1 */
	group_values[0] = (exposure_sp1_us >> 0) & 0xFF;
	group_values[1] = (exposure_sp1_us >> 8) & 0xFF;
	group_values[2] = (exposure_sp1_us >> 16) & 0xFF;
	group_values[3] = (exposure_sp1_us >> 24) & 0xFF;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX623_INFO,
	      "SetIntTime: Before write - SP1: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      group_values[0], group_values[1], group_values[2], group_values[3]);
#endif
	result = IMX623_IsiWriteRegGroupIss(handle, IMX623_REG_EXPOSURE_SP1_LL,
					    group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back SP1 values */
	uint16_t read_value = 0;
	uint8_t read_values[4] = {0};

	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP1_LL, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP1_LH, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP1_HL, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP1_HH, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(IMX623_INFO,
	      "SetIntTime: After write - SP1: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      read_values[0], read_values[1], read_values[2], read_values[3]);
#endif
	/* Set exposure time for SP2 */
	group_values[0] = (exposure_sp2_us >> 0) & 0xFF;
	group_values[1] = (exposure_sp2_us >> 8) & 0xFF;
	group_values[2] = (exposure_sp2_us >> 16) & 0xFF;
	group_values[3] = (exposure_sp2_us >> 24) & 0xFF;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(IMX623_INFO,
	      "SetIntTime: Before write - SP2: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      group_values[0], group_values[1], group_values[2], group_values[3]);
#endif
	result |= IMX623_IsiWriteRegGroupIss(handle, IMX623_REG_EXPOSURE_SP2_LL,
					     group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	/* Read back SP2 values */
	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP2_LL, &read_value);
	read_values[0] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP2_LH, &read_value);
	read_values[1] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP2_HL, &read_value);
	read_values[2] = read_value & 0xFF;
	IMX623_IsiReadRegIss(handle, IMX623_REG_EXPOSURE_SP2_HH, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(IMX623_INFO,
	      "SetIntTime: After write - SP2: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      read_values[0], read_values[1], read_values[2], read_values[3]);
#endif
#else
	/* Set exposure time for SP1 */
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_LL,
				       (exposure_sp1_us >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_LH,
					(exposure_sp1_us >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_HL,
					(exposure_sp1_us >> 16) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_HH,
					(exposure_sp1_us >> 24) & 0xFF);

	/* Set exposure time for SP2 */
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_LL,
					(exposure_sp2_us >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_LH,
					(exposure_sp2_us >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_HL,
					(exposure_sp2_us >> 16) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_HH,
					(exposure_sp2_us >> 24) & 0xFF);
#endif

	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to write exposure time registers\n", __func__);
		return RET_FAILURE;
	}

	/* Update context with current integration times */
	pIMX623Ctx->curIntTime.intTime[0] = (float)exposure_sp1_us / SEC_TO_MICROSEC;
	pIMX623Ctx->curIntTime.intTime[1] = (float)exposure_sp2_us / SEC_TO_MICROSEC;
	pIMX623Ctx->curIntTime.intTime[2] = pIMX623Ctx->curIntTime.intTime[0];
	pIMX623Ctx->curIntTime.intTime[3] = pIMX623Ctx->curIntTime.intTime[0];

	TRACE(IMX623_INFO,
	      "%s: frame=%d intTime=[%.6f,%.6f,%.6f,%.6f]\n",
	      __func__, g_Sensor_frame_count,
	      pIMX623Ctx->curIntTime.intTime[0],
	      pIMX623Ctx->curIntTime.intTime[1],
	      pIMX623Ctx->curIntTime.intTime[2],
	      pIMX623Ctx->curIntTime.intTime[3]);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetIspStatusIss
 *
 * @brief   Get sensor isp status.
 *
 * @param   handle                    sensor instance handle
 * @param   pIspStatus          sensor isp status
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetIspStatusIss(IsiSensorHandle_t handle,
				  IsiIspStatus_t *pIspStatus)
{
	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR,
		      "%s: Invalid sensor handle (NULL pointer)\n", __func__);
		return RET_NULL_POINTER;
	}

	pIspStatus->useSensorAE = pIMX623Ctx->sensorConfig.useSensorAE;
	pIspStatus->useAWBMode = pIMX623Ctx->sensorConfig.useAWBMode;
	pIspStatus->useSensorBLC = pIMX623Ctx->sensorConfig.useSensorBLC;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiSetBlcIss
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
static RESULT IMX623_IsiSetBlcIss(IsiSensorHandle_t handle,
				  const IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	/* Scale BLS values by 32 (5-bit left shift) for register format */
	uint16_t bls_r = pBlc->red << 5;
	uint16_t bls_gr = pBlc->gr << 5;
	uint16_t bls_gb = pBlc->gb << 5;
	uint16_t bls_b = pBlc->blue << 5;

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];

	/* SP1HCG - P0 (R) and P1 (GR) */
	group_values[0] = bls_r & 0xFF;
	group_values[1] = (bls_r >> 8) & 0xFF;
	group_values[2] = bls_gr & 0xFF;
	group_values[3] = (bls_gr >> 8) & 0xFF;
	result = IMX623_IsiWriteRegGroupIss(handle,
					    IMX623_REG_BLACK_LEVEL_SP1HCG_P0_L,
					    group_values, 4U);

	/* SP1HCG - P2 (GB) and P3 (B) */
	group_values[0] = bls_gb & 0xFF;
	group_values[1] = (bls_gb >> 8) & 0xFF;
	group_values[2] = bls_b & 0xFF;
	group_values[3] = (bls_b >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle,
					     IMX623_REG_BLACK_LEVEL_SP1HCG_P2_L,
					     group_values, 4U);

	/* SP1LCG - P0 (R) and P1 (GR) */
	group_values[0] = bls_r & 0xFF;
	group_values[1] = (bls_r >> 8) & 0xFF;
	group_values[2] = bls_gr & 0xFF;
	group_values[3] = (bls_gr >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle,
					     IMX623_REG_BLACK_LEVEL_SP1LCG_P0_L,
					     group_values, 4U);

	/* SP1LCG - P2 (GB) and P3 (B) */
	group_values[0] = bls_gb & 0xFF;
	group_values[1] = (bls_gb >> 8) & 0xFF;
	group_values[2] = bls_b & 0xFF;
	group_values[3] = (bls_b >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle,
					     IMX623_REG_BLACK_LEVEL_SP1LCG_P2_L,
					     group_values, 4U);

	/* SP2H - P0 (R) and P1 (GR) */
	group_values[0] = bls_r & 0xFF;
	group_values[1] = (bls_r >> 8) & 0xFF;
	group_values[2] = bls_gr & 0xFF;
	group_values[3] = (bls_gr >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle,
					     IMX623_REG_BLACK_LEVEL_SP2H_P0_L,
					     group_values, 4U);

	/* SP2H - P2 (GB) and P3 (B) */
	group_values[0] = bls_gb & 0xFF;
	group_values[1] = (bls_gb >> 8) & 0xFF;
	group_values[2] = bls_b & 0xFF;
	group_values[3] = (bls_b >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle,
					     IMX623_REG_BLACK_LEVEL_SP2H_P2_L,
					     group_values, 4U);

	/* SP2L - P0 (R) and P1 (GR) */
	group_values[0] = bls_r & 0xFF;
	group_values[1] = (bls_r >> 8) & 0xFF;
	group_values[2] = bls_gr & 0xFF;
	group_values[3] = (bls_gr >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle,
					     IMX623_REG_BLACK_LEVEL_SP2L_P0_L,
					     group_values, 4U);

	/* SP2L - P2 (GB) and P3 (B) */
	group_values[0] = bls_gb & 0xFF;
	group_values[1] = (bls_gb >> 8) & 0xFF;
	group_values[2] = bls_b & 0xFF;
	group_values[3] = (bls_b >> 8) & 0xFF;
	result |= IMX623_IsiWriteRegGroupIss(handle,
					     IMX623_REG_BLACK_LEVEL_SP2L_P2_L,
					     group_values, 4U);
#else
	result = IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P0_L, bls_r & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P0_H, (bls_r >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P1_L, bls_gr & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P1_H, (bls_gr >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P2_L, bls_gb & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P2_H, (bls_gb >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P3_L, bls_b & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P3_H, (bls_b >> 8) & 0xFF);

	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P0_L, bls_r & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P0_H, (bls_r >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P1_L, bls_gr & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P1_H, (bls_gr >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P2_L, bls_gb & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P2_H, (bls_gb >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P3_L, bls_b & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1LCG_P3_H, (bls_b >> 8) & 0xFF);

	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P0_L, bls_r & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P0_H, (bls_r >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P1_L, bls_gr & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P1_H, (bls_gr >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P2_L, bls_gb & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P2_H, (bls_gb >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P3_L, bls_b & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2H_P3_H, (bls_b >> 8) & 0xFF);

	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P0_L, bls_r & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P0_H, (bls_r >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P1_L, bls_gr & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P1_H, (bls_gr >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P2_L, bls_gb & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P2_H, (bls_gb >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P3_L, bls_b & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP2L_P3_H, (bls_b >> 8) & 0xFF);
#endif
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to set sensor black level subtraction\n",
		      __func__);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetBlcIss
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
static RESULT IMX623_IsiGetBlcIss(IsiSensorHandle_t handle,
				  IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	/* Read black level values (scaled by 32 in register format) */
	uint16_t reg_value = 0;
	uint16_t bls_r = 0;
	uint16_t bls_gr = 0;
	uint16_t bls_gb = 0;
	uint16_t bls_b = 0;

	result = IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P0_L, &bls_r);
	result |= IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P0_H, &reg_value);
	bls_r |= (reg_value & 0xFF) << 8;

	result |= IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P1_L, &bls_gr);
	result |= IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P1_H, &reg_value);
	bls_gr |= (reg_value & 0xFF) << 8;

	result |= IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P2_L, &bls_gb);
	result |= IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P2_H, &reg_value);
	bls_gb |= (reg_value & 0xFF) << 8;

	result |= IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P3_L, &bls_b);
	result |= IMX623_IsiReadRegIss(handle,
		IMX623_REG_BLACK_LEVEL_SP1HCG_P3_H, &reg_value);
	bls_b |= (reg_value & 0xFF) << 8;

	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR,
		      "%s: Failed to get sensor black level subtraction\n",
		      __func__);
		return RET_FAILURE;
	}

	pBlc->red = bls_r;
	pBlc->gr = bls_gr;
	pBlc->gb = bls_gb;
	pBlc->blue = bls_b;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetExpandCurveIss
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
static RESULT IMX623_IsiGetExpandCurveIss(IsiSensorHandle_t handle,
					  IsiSensorCompandCurve_t *pCurve)
{
	RESULT result = RET_SUCCESS;
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL || pCurve == NULL) {
		TRACE(IMX623_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	return result;
}
