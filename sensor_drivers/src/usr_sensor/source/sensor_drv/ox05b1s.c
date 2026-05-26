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

#include <oslayer/oslayer.h>
#include <isi/isi_fmc.h>
#include "isi/isi.h"
#include "isi/isi_iss.h"
#include "sensor_drv/ox05b1s_priv.h"

CREATE_TRACER(Ox05b1s_INFO, "Ox05b1s: ", INFO,      1);
CREATE_TRACER(Ox05b1s_WARN, "Ox05b1s: ", WARNING,   1);
CREATE_TRACER(Ox05b1s_ERROR, "Ox05b1s: ", ERROR,    1);
CREATE_TRACER(Ox05b1s_DEBUG, "Ox05b1s: ", INFO,     1);

#define IR_POWER_DEVICE_SLAVE_ADDRESS		(0x4a)
#define Ox05b1s_MIN_GAIN_STEP			(1.0f/1024.0f)
#define OX05B1S_IR_LIGHT_STRENGTH_MAX	(26)
#define OX05B1S_MAX_INSTANCES			(16)

typedef struct multiInstanceMutex_s {
	osMutex mut;
	bool initialized;
} multiInstanceMutex_t;

multiInstanceMutex_t ox05b1sABmodeMutex[OX05B1S_MAX_INSTANCES];

typedef enum {
	OX05B1S_AB_MODE_EXP_LINE = 0,
	OX05B1S_AB_MODE_A_GAIN,
	OX05B1S_AB_MODE_D_GAIN,
	OX05B1S_AB_MODE_IR_PARAMS
} OX05B1S_ABmode_Index_t;

/*****************************************************************************
 *Sensor Info
 *****************************************************************************/

IsiSensorMode_t pox05b1s_mode_info[] = {
	{
	.index     = 0,
		.size = {
		.boundsWidth   = 2592,
		.boundsHeight  = 1944,
		.top           = 0,
		.left          = 0,
		.width         = 2592,
		.height        = 1944,
	},
	.aeInfo    = {
		.intTimeDelayFrame = 2,
		.gainDelayFrame = 2,
	},
	.fps       = 30 * ISI_FPS_QUANTIZE,
	.hdrMode  = ISI_SENSOR_MODE_LINEAR,
	.bitWidth = 10,
	.bayerPattern = ISI_BPAT_BGGIR,
	.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType = ISI_MODE_BAYER,
		.mipiLane = ISI_MIPI_4LANES,
	},
	{
		.index     = 1,
		.size = {
			.boundsWidth   = 2592,
			.boundsHeight  = 1944,
			.top           = 0,
			.left          = 0,
			.width         = 2592,
			.height        = 1944,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 30 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.bitWidth = 10,
		.bayerPattern = ISI_BPAT_BGGIR,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType = ISI_MODE_BAYER,
		.mipiLane = ISI_MIPI_4LANES,
	 },
	{
		.index     = 2,
		.size = {
			.boundsWidth   = 2592,
			.boundsHeight  = 1944,
			.top           = 0,
			.left          = 0,
			.width         = 2592,
			.height        = 1944,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 30 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.bitWidth = 10,
		.bayerPattern = ISI_BPAT_BGGIR,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType = ISI_MODE_BAYER,
		.mipiLane = ISI_MIPI_4LANES,
	 },
};

int ox05b1s_mode_num =
	(int)(sizeof(pox05b1s_mode_info) / sizeof(IsiSensorMode_t));

/**************************************************************************
 *          enable_IR_power
 *
 * @brief   Enable IR power for sensor
 *
 * @param   handle      Sensor handle
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
RESULT enable_IR_power(IsiSensorHandle_t handle)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	uint32_t register_addr = 0x04;
	uint8_t wr_data[2];
	RESULT Status = RET_SUCCESS;

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(Ox05b1s_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}

	struct accessIIC *iic_access =
		active_fmc->accessiic_array
		[pOx05b1sCtx->sensorDevId];

	wr_data[0] = 0x0f;
	Status = iic_access->writeIIC(
		pOx05b1sCtx->i2cId,
		IR_POWER_DEVICE_SLAVE_ADDRESS,
		register_addr, 0x1, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s:%d IR write sensor register error! (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	register_addr = 0x02;
	wr_data[0] = 0xe1;
	Status = iic_access->writeIIC(
		pOx05b1sCtx->i2cId,
		IR_POWER_DEVICE_SLAVE_ADDRESS,
		register_addr, 0x1, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s:%d IR write reg 0x02 failed (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	register_addr = 0x05;
	wr_data[0] = 0x00;
	Status = iic_access->writeIIC(
		pOx05b1sCtx->i2cId,
		IR_POWER_DEVICE_SLAVE_ADDRESS,
		register_addr, 0x1, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s:%d IR write reg 0x05 failed (err=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	return RET_SUCCESS;
}

/*****************************************************************************
 *          Ox05b1s_IsiReadRegIss
 *
 * @brief   reads a given number of bytes from the image sensor device
 *
 * @param   handle              Handle to image sensor device
 * @param   addr                register address
 * @param   pValue              value to read
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT Ox05b1s_IsiReadRegIss(IsiSensorHandle_t handle,
				    const uint16_t addr,
				    uint16_t *pValue)
{
	RESULT result = RET_SUCCESS;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL || pValue == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx or pValue is NULL\n", __func__);
		return RET_NULL_POINTER;
	}
	memset(pValue, 0, sizeof(uint16_t));
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(Ox05b1s_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pOx05b1sCtx->sensorDevId]
		 ->sensor_alias_addr) >> 1;

	uint8_t read_val = 0;

	result = active_fmc->accessiic_array
		[pOx05b1sCtx->sensorDevId]->readIIC(
		pOx05b1sCtx->i2cId, slave_addr,
		addr, 0x2, &read_val, 1);
	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR, "%s: I2C read failed at addr 0x%04x (err=%d)\n",
			__func__, addr, result);
		return RET_FAILURE;
	}
	*pValue = (uint16_t)read_val;

	return result;
}

/*****************************************************************************
 *          Ox05b1s_IsiWriteRegIss
 *
 * @brief   writes a given number of bytes to the image sensor device by
 *          calling the corresponding sensor-function
 *
 * @param   handle              Handle to image sensor device
 * @param   addr                register address
 * @param   value               value to write
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NOTSUPP
 *
 ***************************************************************************/
static RESULT Ox05b1s_IsiWriteRegIss(IsiSensorHandle_t handle,
				     const uint16_t addr,
				     const uint16_t value)
{
	RESULT result = RET_SUCCESS;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(Ox05b1s_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pOx05b1sCtx->sensorDevId]
		 ->sensor_alias_addr) >> 1;

	u8 wr_data[2];

	wr_data[0] = (u8)value;
	result = active_fmc->accessiic_array
		[pOx05b1sCtx->sensorDevId]->writeIIC(
		pOx05b1sCtx->i2cId, slave_addr,
		addr, 0x2, wr_data, 1);

	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR, "%s: I2C write failed at addr 0x%04x (err=%d)\n",
			__func__, addr, result);
		return RET_FAILURE;
	}
	return result;
}

#ifdef ENABLE_I2C_GROUPING
/***************************************************************************
 *          Ox05b1s_IsiWriteRegGroupIss
 *
 * @brief   Write group register values to Ox05b1s sensor via I2C.
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
 ***************************************************************************/
static RESULT Ox05b1s_IsiWriteRegGroupIss(IsiSensorHandle_t handle,
					   const uint16_t addr,
					   uint8_t *value,
					   uint8_t datacount)
{
	RESULT result = RET_SUCCESS;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (datacount > 4U)
		return RET_FAILURE;

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(Ox05b1s_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pOx05b1sCtx->sensorDevId]
		 ->sensor_alias_addr) >> 1;

	result = active_fmc->accessiic_array
		[pOx05b1sCtx->sensorDevId]->writeIIC(
		pOx05b1sCtx->i2cId, slave_addr,
		addr, 0x2, value, datacount);

	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR, "%s: I2C group write failed at addr 0x%04x (err=%d)\n",
			__func__, addr, result);
		return RET_FAILURE;
	}

	return result;
}
#endif

/****************************************************************************
 *          Ox05b1s_IsiGetModeIss
 *
 * @brief   get cuurent sensor mode info.
 *
 * @param   handle      Sensor instance handle
 * @param   pMode       Sensor Mode Value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 ***************************************************************************/
static RESULT Ox05b1s_IsiGetModeIss(IsiSensorHandle_t handle,
				    IsiSensorMode_t *pMode)
{
	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_WRONG_HANDLE;
	}
	if (pMode == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pMode is NULL\n", __func__);
		return RET_WRONG_HANDLE;
	}

	memcpy(pMode, &(pOx05b1sCtx->sensorMode), sizeof(pOx05b1sCtx->sensorMode));

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          Ox05b1s_IsiEnumModeIss
 *
 * @brief   query sensor info.
 *
 * @param   handle                  sensor instance handle
 * @param   EnumModePtr             sensor query mode
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static  RESULT Ox05b1s_IsiEnumModeIss(IsiSensorHandle_t handle,
				      IsiSensorEnumMode_t *pEnumMode)
{
	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pEnumMode->index >= (ARRAY_SIZE(pox05b1s_mode_info))) {
		TRACE(Ox05b1s_ERROR,
		      "%s: index %d out of range\n", __func__, pEnumMode->index);
		return RET_OUTOFRANGE;
	}

	for (uint32_t mode_idx = 0;
	     mode_idx < (ARRAY_SIZE(pox05b1s_mode_info)); mode_idx++) {
		if (pox05b1s_mode_info[mode_idx].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &pox05b1s_mode_info[mode_idx],
			       sizeof(IsiSensorMode_t));
			TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
			return RET_SUCCESS;
		}
	}

	return RET_NOTSUPP;
}

/*****************************************************************************
 *          Ox05b1s_IsiGetCapsIss
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
static RESULT Ox05b1s_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_WRONG_HANDLE;
	}

	if (pCaps == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pCaps is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	pCaps->bitWidth          = pOx05b1sCtx->sensorMode.bitWidth;
	pCaps->mode              = ISI_MODE_BAYER;
	pCaps->bayerPattern      = pOx05b1sCtx->sensorMode.bayerPattern;
	pCaps->resolution.width  = pOx05b1sCtx->sensorMode.size.width;
	pCaps->resolution.height = pOx05b1sCtx->sensorMode.size.height;
	pCaps->mipiLanes         = ISI_MIPI_4LANES;
	pCaps->vinType           = ISI_ITF_TYPE_MIPI;

	if (pCaps->bitWidth == 10)
		pCaps->mipiMode      = ISI_FORMAT_RAW_10;
	else if (pCaps->bitWidth == 12)
		pCaps->mipiMode      = ISI_FORMAT_RAW_12;
	else
		pCaps->mipiMode      = ISI_MIPI_OFF;

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox05b1s_IsiCreateIss
 *
 * @brief   Create Sensor Context for the given config
 *
 * @param   pConfig	Given Sensor Config
 * @param   pHandle	Return the Sensor Ctx
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Ox05b1s_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig,
				   IsiSensorHandle_t *pHandle)
{
	RESULT result = RET_SUCCESS;
	uint32_t desId = 0, pipeId = 0;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx =
		(Ox05b1s_Context_t *) osMalloc(sizeof(Ox05b1s_Context_t));

	if (!pOx05b1sCtx) {
		TRACE(Ox05b1s_ERROR, "%s: Can't allocate ox05b1s context\n", __func__);
		return RET_OUTOFMEM;
	}

	MEMSET(pOx05b1sCtx, 0, sizeof(Ox05b1s_Context_t));

	pOx05b1sCtx->isiCtx.pSensor	= pConfig->pSensor;
	pOx05b1sCtx->groupHold		= BOOL_FALSE;
	pOx05b1sCtx->configured		= BOOL_FALSE;
	pOx05b1sCtx->streaming		= BOOL_FALSE;
	pOx05b1sCtx->testPattern	= BOOL_FALSE;
	pOx05b1sCtx->isAfpsRun		= BOOL_FALSE;
	pOx05b1sCtx->sensorMode.index	= 0;
	pOx05b1sCtx->sensorDevId	= pConfig->halDevID;

	pipeId = pOx05b1sCtx->sensorDevId;

	*pHandle = (IsiSensorHandle_t) pOx05b1sCtx;

	desId = MAPPING_INPIPE_TO_DES_ID(pipeId);

	pOx05b1sCtx->i2cId = GetI2cBusIdForDes(desId);
	uint8_t busId = (uint8_t)pOx05b1sCtx->i2cId;

	/* Validate I2C bus ID */
	if (busId == INVALID_I2C_BUS_ID) {
		TRACE(Ox05b1s_ERROR,
		      "%s: Invalid I2C bus ID for desId %d\n",
		      __func__, desId);
		osFree(pOx05b1sCtx);
		return RET_FAILURE;
	}

	TRACE(Ox05b1s_INFO,
	      "\r\n desId %d pipeId:%d i2cBusId:%d\r\n",
	      desId, pipeId, pOx05b1sCtx->i2cId);

	result = init_iic_access(busId, pipeId);
	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s: init_iic_access failed for pipe %d (err=%d)\n",
		      __func__, pipeId, result);
		osFree(pOx05b1sCtx);
		return result;
	}

	static int8_t mcmABmode_initCount;

	/* MCM A/B mode: init_des/init_sensor only needed once per GMSL link.
	 * With up to 2 deserializer links, skip init after the first 3 calls
	 * since the deserializer and serializer are already configured. */
	if (mcmABmode_initCount <= 2) {
		result = init_des(desId);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR,
			      "%s: init_des failed for desId %d (err=%d)\n",
			      __func__, desId, result);
			osFree(pOx05b1sCtx);
			return result;
		}
		result = init_sensor(pipeId, desId, SENSOR_OX5B_ADDRESS);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR,
			      "%s: init_sensor failed for pipe %d (err=%d)\n",
			      __func__, pipeId, result);
			osFree(pOx05b1sCtx);
			return result;
		}
	}

	mcmABmode_initCount++;

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          Ox05b1s_AecSetModeParameters
 *
 * @brief   Set AEC mode-specific parameters
 *
 * @param   handle       Sensor handle
 * @param   pOx05b1sCtx  Pointer to sensor context
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT Ox05b1s_AecSetModeParameters(IsiSensorHandle_t handle,
			Ox05b1s_Context_t *pOx05b1sCtx)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO,
	      "%s%s: (enter)\n", __func__, pOx05b1sCtx->isAfpsRun ? "(AFPS)" : "");
	uint32_t exp_line = 0, again = 0, dgain = 0, irLine;
	uint16_t value = 0;

	pOx05b1sCtx->aecMinIntegrationTime       = pOx05b1sCtx->oneLineExpTime
		* pOx05b1sCtx->minIntegrationLine;
	pOx05b1sCtx->aecMaxIntegrationTime       = pOx05b1sCtx->oneLineExpTime
		* pOx05b1sCtx->maxIntegrationLine;
	TRACE(Ox05b1s_DEBUG, "%s: AecMaxIntegrationTime = %f\n", __func__,
		 pOx05b1sCtx->aecMaxIntegrationTime);

	pOx05b1sCtx->aecGainIncrement = Ox05b1s_MIN_GAIN_STEP;
	pOx05b1sCtx->aecIntegrationTimeIncrement = pOx05b1sCtx->oneLineExpTime;

	pOx05b1sCtx->irLightInfo.irRangeInfo.minIrStrength    = 4;
	pOx05b1sCtx->irLightInfo.irRangeInfo.maxIrStrength    =
					OX05B1S_IR_LIGHT_STRENGTH_MAX;
	pOx05b1sCtx->irLightInfo.irRangeInfo.irStrengthStep   = 1;
	pOx05b1sCtx->irLightInfo.irDelayFrame      = 2;
	if (pOx05b1sCtx->sensorMode.index == 2)
		pOx05b1sCtx->irLightInfo.irSuppAeCtrl   = 1;
	else
		pOx05b1sCtx->irLightInfo.irSuppAeCtrl   = 0;

	Ox05b1s_IsiReadRegIss(handle, 0x3508, &value);
	again = (value & 0x0f) << 4;
	Ox05b1s_IsiReadRegIss(handle, 0x3509, &value);
	again = again | ((value & 0xf0) >> 4);

	Ox05b1s_IsiReadRegIss(handle, 0x350a, &value);
	dgain = (value & 0x0f) << 10;
	Ox05b1s_IsiReadRegIss(handle, 0x350b, &value);
	dgain = dgain | ((value & 0xff) << 2);
	Ox05b1s_IsiReadRegIss(handle, 0x350c, &value);
	dgain = dgain | ((value & 0xc0) >> 6);
	pOx05b1sCtx->aecCurGain = ((float)again/16.0) * ((float)dgain/1024.0);

	Ox05b1s_IsiReadRegIss(handle, 0x3500, &value);
	exp_line = (value & 0xff) << 16;
	Ox05b1s_IsiReadRegIss(handle, 0x3501, &value);
	exp_line = exp_line | ((value & 0xff) << 8);
	Ox05b1s_IsiReadRegIss(handle, 0x3502, &value);
	exp_line = exp_line | (value & 0xff);
	pOx05b1sCtx->aecCurIntegrationTime = exp_line * pOx05b1sCtx->oneLineExpTime;

	if (pOx05b1sCtx->sensorMode.index == 2) {
		pOx05b1sCtx->irLightExp.irOn = BOOL_TRUE;
		result |= Ox05b1s_IsiReadRegIss(handle, 0x3b25, &value);//strobe width
		irLine = (value & 0xff) << 24;
		result |= Ox05b1s_IsiReadRegIss(handle, 0x3b26, &value);//0x00
		irLine |= (value & 0xff) << 16;
		result |= Ox05b1s_IsiReadRegIss(handle, 0x3b27, &value);//0x00
		irLine |= (value & 0xff) << 8;
		result |= Ox05b1s_IsiReadRegIss(handle, 0x3b28, &value);//0x1a
		irLine |= value & 0xff;
		pOx05b1sCtx->irLightExp.irStrength = MAX(MIN(irLine,
			pOx05b1sCtx->irLightInfo.irRangeInfo.maxIrStrength),
			pOx05b1sCtx->irLightInfo.irRangeInfo.minIrStrength);
	} else {
		pOx05b1sCtx->irLightExp.irOn = BOOL_FALSE;
		pOx05b1sCtx->irLightExp.irStrength = 0;
	}

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*****************************************************************************
 *         ox05b1s_IsiOpenIss
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
static RESULT Ox05b1s_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (!pOx05b1sCtx) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
		__func__);
		return RET_WRONG_HANDLE;
	}

	if (pOx05b1sCtx->streaming != BOOL_FALSE) {
		TRACE(Ox05b1s_ERROR, "%s: sensor already streaming\n", __func__);
		return RET_WRONG_STATE;
	}

	pOx05b1sCtx->sensorMode.index   = mode;
	IsiSensorMode_t *SensorDefaultMode = NULL;

	for (uint32_t mode_idx = 0;
	     mode_idx < sizeof(pox05b1s_mode_info) / sizeof(IsiSensorMode_t);
	     mode_idx++) {
		if (pox05b1s_mode_info[mode_idx].index == pOx05b1sCtx->sensorMode.index) {
			SensorDefaultMode = &(pox05b1s_mode_info[mode_idx]);
			break;
		}
	}

	if (SensorDefaultMode != NULL) {
		switch (SensorDefaultMode->index) {
		case 0:
		for (uint32_t reg_idx = 0;
		     reg_idx < sizeof(Ox05b1s_mipi4lane_2592_1944_linear_init)
			/ sizeof(Ox05b1s_mipi4lane_2592_1944_linear_init[0]);
		     reg_idx++) {
			if (Ox05b1s_mipi4lane_2592_1944_linear_init
			    [reg_idx][0] == OX05B1S_TABLE_WAIT) {
				osSleep(Ox05b1s_mipi4lane_2592_1944_linear_init[reg_idx][1]);
				continue;
			}
			if (Ox05b1s_mipi4lane_2592_1944_linear_init[reg_idx][0] ==
				OX05B1S_TABLE_END)
				break;
			Ox05b1s_IsiWriteRegIss(handle,
					Ox05b1s_mipi4lane_2592_1944_linear_init[reg_idx][0],
					Ox05b1s_mipi4lane_2592_1944_linear_init[reg_idx][1]);
				}
				break;
		case 1:
		if (!(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].initialized)) {
			int32_t osRet = OSLAYER_OK;

			osRet = osMutexInit(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
			if (osRet != OSLAYER_OK)
				return RET_FAILURE;

			ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].initialized = true;
		}

		for (uint32_t reg_idx = 0;
		     reg_idx < sizeof(Ox05b1s_mipi4lane_2592_1944_linear_ABmode_init) /
			sizeof(Ox05b1s_mipi4lane_2592_1944_linear_ABmode_init[0]);
		     reg_idx++) {
			if (Ox05b1s_mipi4lane_2592_1944_linear_ABmode_init[reg_idx][0] ==
				OX05B1S_TABLE_WAIT) {
				osSleep(Ox05b1s_mipi4lane_2592_1944_linear_ABmode_init[reg_idx][1]);
				continue;
			}
			if (Ox05b1s_mipi4lane_2592_1944_linear_ABmode_init[reg_idx][0] ==
				OX05B1S_TABLE_END)
				break;
			Ox05b1s_IsiWriteRegIss(handle,
				       Ox05b1s_mipi4lane_2592_1944_linear_ABmode_init[reg_idx][0],
				       Ox05b1s_mipi4lane_2592_1944_linear_ABmode_init[reg_idx][1]);

		}
		break;
		case 2:
		if (!(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].initialized)) {
			int32_t osRet = OSLAYER_OK;

			osRet = osMutexInit(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
			if (osRet != OSLAYER_OK)
				return RET_FAILURE;

			ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].initialized = true;
		}

		for (uint32_t reg_idx = 0;
			reg_idx < sizeof(Ox05b1s_mipi4lane_2592_1944_linear_ABmode_IRframe_init) /
			sizeof(Ox05b1s_mipi4lane_2592_1944_linear_ABmode_IRframe_init[0]);
		     reg_idx++) {
			if (Ox05b1s_mipi4lane_2592_1944_linear_ABmode_IRframe_init
				[reg_idx][0] == OX05B1S_TABLE_WAIT) {
				osSleep(Ox05b1s_mipi4lane_2592_1944_linear_ABmode_IRframe_init
					[reg_idx][1]);
				continue;
			}
			if (Ox05b1s_mipi4lane_2592_1944_linear_ABmode_IRframe_init
				   [reg_idx][0] ==
				OX05B1S_TABLE_END)
				break;
			Ox05b1s_IsiWriteRegIss(handle,
				       Ox05b1s_mipi4lane_2592_1944_linear_ABmode_IRframe_init
					   [reg_idx][0],
				       Ox05b1s_mipi4lane_2592_1944_linear_ABmode_IRframe_init
					   [reg_idx][1]);

		}
		break;
		default:
		TRACE(Ox05b1s_INFO, "%s:not support sensor mode %d\n",
			__func__, pOx05b1sCtx->sensorMode.index);
		osFree(pOx05b1sCtx);
		return RET_NOTSUPP;
	}

		memcpy(&(pOx05b1sCtx->sensorMode), SensorDefaultMode,
		       sizeof(IsiSensorMode_t));
	} else {
		TRACE(Ox05b1s_ERROR, "%s: Invalid SensorDefaultMode\n", __func__);
		return RET_NULL_POINTER;
	}

		switch (pOx05b1sCtx->sensorMode.index) {
		case 0:
		pOx05b1sCtx->oneLineExpTime      = 0.000027;
		pOx05b1sCtx->frameLengthLines    = 0x850;
		pOx05b1sCtx->curFrameLengthLines = pOx05b1sCtx->frameLengthLines;
		pOx05b1sCtx->maxIntegrationLine  = pOx05b1sCtx->frameLengthLines - 30;
		pOx05b1sCtx->minIntegrationLine  = 4;
		pOx05b1sCtx->aecMaxGain          = 230;
		pOx05b1sCtx->aecMinGain          = 1.0;
		pOx05b1sCtx->aGain.min           = 1.0;
		pOx05b1sCtx->aGain.max           = 15.5;
		pOx05b1sCtx->aGain.step          = (1.0f/16.0f);
		pOx05b1sCtx->dGain.min           = 1.0;
		pOx05b1sCtx->dGain.max           = 15;
		pOx05b1sCtx->dGain.step          = (1.0f/1024.0f);
		break;
		case 1:
		case 2:
		pOx05b1sCtx->oneLineExpTime      = 0.00001601562f; /* HTS=0x02f0 */
		pOx05b1sCtx->frameLengthLines    = 0x850;
		pOx05b1sCtx->curFrameLengthLines = pOx05b1sCtx->frameLengthLines;
		pOx05b1sCtx->maxIntegrationLine  = pOx05b1sCtx->frameLengthLines - 30;
		pOx05b1sCtx->minIntegrationLine  = 4;
		pOx05b1sCtx->aecMaxGain          = 230;
		pOx05b1sCtx->aecMinGain          = 1.0;
		pOx05b1sCtx->aGain.min           = 1.0;
		pOx05b1sCtx->aGain.max           = 15.5;
		pOx05b1sCtx->aGain.step          = (1.0f/16.0f);
		pOx05b1sCtx->dGain.min           = 1.0;
		pOx05b1sCtx->dGain.max           = 15;
		pOx05b1sCtx->dGain.step          = (1.0f/1024.0f);
		break;
		default:
		TRACE(Ox05b1s_INFO, "%s:not support sensor mode %d\n",
				__func__, pOx05b1sCtx->sensorMode.index);
		return RET_NOTSUPP;
	}

	pOx05b1sCtx->maxFps				= pOx05b1sCtx->sensorMode.fps;
	pOx05b1sCtx->minFps				= 1 * ISI_FPS_QUANTIZE;
	pOx05b1sCtx->currFps				= pOx05b1sCtx->maxFps;
	pOx05b1sCtx->sensorWb.rGain			= 1.8;
	pOx05b1sCtx->sensorWb.gbGain			= 1.0;
	pOx05b1sCtx->sensorWb.grGain			= 1.0;
	pOx05b1sCtx->sensorWb.bGain			= 1.65;

	TRACE(Ox05b1s_DEBUG, "%s: Ox05b1s System-Reset executed\n", __func__);

	result = Ox05b1s_AecSetModeParameters(handle, pOx05b1sCtx);
	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR, "%s: SetupOutputWindow failed. (err=%d)\n", __func__, result);
		return result;
	}

	pOx05b1sCtx->configured = BOOL_TRUE;


	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return 0;
}

/****************************************************************************
 *         ox05b1s_IsiCloseIss
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
static RESULT Ox05b1s_IsiCloseIss(IsiSensorHandle_t handle)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_WRONG_HANDLE;
	}

	(void)Ox05b1s_IsiSetStreamingIss(pOx05b1sCtx, BOOL_FALSE);

	if (pOx05b1sCtx->sensorMode.index == 1) {
		int32_t osRet = OSLAYER_OK;

		osRet = osMutexDestroy(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
		if (osRet != OSLAYER_OK)
			return RET_FAILURE;

		ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].initialized = false;
	}
	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          Ox05b1s_IsiReleaseIss
 *
 * @brief   Release sensor instance
 *
 * @param   handle      Sensor handle
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT Ox05b1s_IsiReleaseIss(IsiSensorHandle_t handle)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_WRONG_HANDLE;
	}

	stop_sensor(pOx05b1sCtx->sensorDevId);

	MEMSET(pOx05b1sCtx, 0, sizeof(Ox05b1s_Context_t));
	osFree(pOx05b1sCtx);
	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          IsiCheckConnectionIss
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
static RESULT Ox05b1s_IsiCheckConnectionIss(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	uint32_t sensor_id = 0;
	uint32_t correct_id = OX05B1S_SENSOR_ID;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	result = Ox05b1s_IsiGetRevisionIss(handle, &sensor_id);
	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR, "%s: Read Sensor ID Error! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	if (correct_id != sensor_id) {
		TRACE(Ox05b1s_ERROR, "%s:ChipID =0x%x sensor_id=%x error!\n",
				__func__, correct_id, sensor_id);
		return RET_FAILURE;
	}

	TRACE(Ox05b1s_INFO, "%s ChipID = 0x%08x, sensor_id = 0x%08x, success!\n",
	      __func__, correct_id, sensor_id);
	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiGetRevisionIss
 *
 * @brief   This function reads the sensor revision register and returns it.
 *
 * @param   handle      sensor instance handle
 * @param   pRevision   pointer to revision
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT Ox05b1s_IsiGetRevisionIss(IsiSensorHandle_t handle,
					uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val;
	uint32_t sensor_id;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	reg_val   = 0;
	result    = Ox05b1s_IsiReadRegIss(handle, 0x300a, &reg_val);
	sensor_id = (reg_val & 0xff) << 8;

	reg_val   = 0;
	result    |= Ox05b1s_IsiReadRegIss(handle, 0x300b, &reg_val);
	sensor_id |= (reg_val & 0xff);

	*pValue = sensor_id;
	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          Ox05b1s_IsiSetGroupA_IR
 *
 * @brief   Set Group A IR exposure registers
 *
 * @param   handle      Sensor handle
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT Ox05b1s_IsiSetGroupA_IR(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	TRACE(Ox05b1s_DEBUG,
	      "%s: (enter) expLine=0x%x, again=0x%x, dgain=0x%x\n",
	      __func__, pOx05b1sCtx->expLine, pOx05b1sCtx->again,
	      pOx05b1sCtx->dgain);

	/* ---- Group 0 hold start ---- */
	result  = Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x00);

	/* Set VC0 for IR */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x4813, 0x00);

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];

	/* Exposure time */
	group_values[0] = (pOx05b1sCtx->expLine >> 16) & 0xFF;
	group_values[1] = (pOx05b1sCtx->expLine >> 8) & 0xFF;
	group_values[2] = pOx05b1sCtx->expLine & 0xFF;
	result |= Ox05b1s_IsiWriteRegGroupIss(handle, 0x3500, group_values, 3U);

	/* Analog gain */
	group_values[0] = (pOx05b1sCtx->again >> 4) & 0x0F;
	group_values[1] = (pOx05b1sCtx->again & 0x0F) << 4;
	result |= Ox05b1s_IsiWriteRegGroupIss(handle, 0x3508, group_values, 2U);

	/* Digital gain */
	group_values[0] = (pOx05b1sCtx->dgain >> 10) & 0x0F;
	group_values[1] = (pOx05b1sCtx->dgain >> 2) & 0xFF;
	group_values[2] = (pOx05b1sCtx->dgain & 0x03) << 6;
	result |= Ox05b1s_IsiWriteRegGroupIss(handle, 0x350a, group_values, 3U);
#else
	/* Exposure time */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3500,
					 (pOx05b1sCtx->expLine >> 16) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3501,
					 (pOx05b1sCtx->expLine >> 8) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3502,
					 pOx05b1sCtx->expLine & 0xff);

	/* Analog gain */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3508,
					 (pOx05b1sCtx->again >> 4) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3509,
					 (pOx05b1sCtx->again & 0x0f) << 4);

	/* Digital gain */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350a,
					 (pOx05b1sCtx->dgain >> 10) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350b,
					 (pOx05b1sCtx->dgain >> 2) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350c,
					 (pOx05b1sCtx->dgain & 0x03) << 6);
#endif

	/* ---- Group 0 hold end ---- */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x10);

	TRACE(Ox05b1s_DEBUG, "%s: (exit) result=%d\n", __func__, result);
	return result;
}

/**************************************************************************
 *          Ox05b1s_IsiSetGroupB_RGB
 *
 * @brief   Set Group B RGB exposure registers
 *
 * @param   handle      Sensor handle
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT Ox05b1s_IsiSetGroupB_RGB(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	TRACE(Ox05b1s_DEBUG,
	      "%s: (enter) expLine=0x%x, again=0x%x, dgain=0x%x\n",
	      __func__, pOx05b1sCtx->expLine, pOx05b1sCtx->again,
	      pOx05b1sCtx->dgain);

	/* ---- Group 1 hold start ---- */
	result  = Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x01);

	/* Set VC1 for RGB */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x4813, 0x01);

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];

	/* Exposure time */
	group_values[0] = (pOx05b1sCtx->expLine >> 16) & 0xFF;
	group_values[1] = (pOx05b1sCtx->expLine >> 8) & 0xFF;
	group_values[2] = pOx05b1sCtx->expLine & 0xFF;
	result |= Ox05b1s_IsiWriteRegGroupIss(handle, 0x3500, group_values, 3U);

	/* Analog gain */
	group_values[0] = (pOx05b1sCtx->again >> 4) & 0x0F;
	group_values[1] = (pOx05b1sCtx->again & 0x0F) << 4;
	result |= Ox05b1s_IsiWriteRegGroupIss(handle, 0x3508, group_values, 2U);

	/* Digital gain */
	group_values[0] = (pOx05b1sCtx->dgain >> 10) & 0x0F;
	group_values[1] = (pOx05b1sCtx->dgain >> 2) & 0xFF;
	group_values[2] = (pOx05b1sCtx->dgain & 0x03) << 6;
	result |= Ox05b1s_IsiWriteRegGroupIss(handle, 0x350a, group_values, 3U);
#else
	/* Exposure time */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3500,
					 (pOx05b1sCtx->expLine >> 16) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3501,
					 (pOx05b1sCtx->expLine >> 8) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3502,
					 pOx05b1sCtx->expLine & 0xff);

	/* Analog gain */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3508,
					 (pOx05b1sCtx->again >> 4) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3509,
					 (pOx05b1sCtx->again & 0x0f) << 4);

	/* Digital gain */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350a,
					 (pOx05b1sCtx->dgain >> 10) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350b,
					 (pOx05b1sCtx->dgain >> 2) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350c,
					 (pOx05b1sCtx->dgain & 0x03) << 6);
#endif

	/* ---- Group 1 hold end ---- */
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x11);

	TRACE(Ox05b1s_DEBUG, "%s: (exit) result=%d\n", __func__, result);
	return result;
}

/**************************************************************************
 *          Ox05b1s_IsiBufferABmodeParam
 *
 * @brief   Buffer AB mode exposure parameter
 *
 * @param   handle   Sensor handle
 * @param   paramId  AB mode parameter index
 * @param   value    Parameter value
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT Ox05b1s_IsiBufferABmodeParam(IsiSensorHandle_t handle,
					    OX05B1S_ABmode_Index_t paramId,
					    uint32_t value)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	TRACE(Ox05b1s_DEBUG,
	      "%s: sensorMode=%d, paramId=%d, value=0x%x (enter)\n",
	      __func__, pOx05b1sCtx->sensorMode.index, paramId, value);

	switch (paramId) {
	case OX05B1S_AB_MODE_EXP_LINE:
		pOx05b1sCtx->expLine = value;
		break;
	case OX05B1S_AB_MODE_A_GAIN:
		pOx05b1sCtx->again = value;
		break;
	case OX05B1S_AB_MODE_D_GAIN:
		pOx05b1sCtx->dgain = value;
		break;
	default:
		TRACE(Ox05b1s_ERROR, "%s: invalid paramId=%d\n", __func__, paramId);
		return RET_INVALID_PARM;
	}

	TRACE(Ox05b1s_DEBUG, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          Ox05b1s_IsiSetABmodeExp
 *
 * @brief   Set AB mode exposure
 *
 * @param   handle      Sensor handle
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT Ox05b1s_IsiSetABmodeExp(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	int32_t osRet = OSLAYER_OK;

	TRACE(Ox05b1s_INFO,
	      "%s: sensorMode=%d (enter)\n", __func__, pOx05b1sCtx->sensorMode.index);

	osRet = osMutexLock(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
	if (osRet != OSLAYER_OK)
		return RET_FAILURE;

	if (pOx05b1sCtx->sensorMode.index == 1) {
		/* RGB stream: program Group 1 (RGB) */
		result = Ox05b1s_IsiSetGroupB_RGB(handle);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: SetGroupB_RGB failed (%d)\n", __func__, result);
			osMutexUnlock(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
			return result;
		}
	} else if (pOx05b1sCtx->sensorMode.index == 2) {
		/* IR stream: program Group 0 (IR) */
		result = Ox05b1s_IsiSetGroupA_IR(handle);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: SetGroupA_IR failed (%d)\n", __func__, result);
			osMutexUnlock(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
			return result;
		}
	} else {
		osMutexUnlock(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
		TRACE(Ox05b1s_ERROR,
		      "%s: unsupported sensor mode %d\n",
		      __func__, pOx05b1sCtx->sensorMode.index);
		return RET_UNSUPPORT_ID;
	}

	osRet = osMutexUnlock(&(ox05b1sABmodeMutex[pOx05b1sCtx->sensorDevId].mut));
	if (osRet != OSLAYER_OK)
		return RET_FAILURE;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT Ox05b1s_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s (enter) on=%d\n", __func__, on);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pOx05b1sCtx->configured != BOOL_TRUE) {
		TRACE(Ox05b1s_ERROR, "%s: sensor not configured\n", __func__);
		return RET_WRONG_STATE;
	}

	if (pOx05b1sCtx->sensorMode.index != 2) {
		result = enable_IR_power(handle);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: enable_IR_power failed! (err=%d)\n", __func__, result);
			return RET_FAILURE;
		}
		result = Ox05b1s_IsiWriteRegIss(handle, 0x0100, on);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: set sensor streaming error! (err=%d)\n", __func__, result);
			return RET_FAILURE;
		}
	}

	if (on == BOOL_TRUE && pOx05b1sCtx->sensorMode.index == 1) {
		/* RGB stream ? initialize buffer for RGB frame (Group B) */
		pOx05b1sCtx->expLine = 0x200;
		pOx05b1sCtx->again = 0x80;
		pOx05b1sCtx->dgain = 0x400;
		pOx05b1sCtx->irLightExp.irOn = BOOL_FALSE;
		pOx05b1sCtx->irLightExp.irStrength = 0;

	} else if (on == BOOL_TRUE && pOx05b1sCtx->sensorMode.index == 2) {
		/* IR stream ? initialize buffer for IR frame (Group A) */
		pOx05b1sCtx->expLine = 0x100;
		pOx05b1sCtx->again = 0x10;
		pOx05b1sCtx->dgain = 0x400;
		pOx05b1sCtx->irLightExp.irOn = BOOL_TRUE;
		pOx05b1sCtx->irLightExp.irStrength = 0x1a;
	}

	pOx05b1sCtx->streaming = on;

	TRACE(Ox05b1s_INFO,
	      "%s: streaming=%s mode=%d res=%dx%d fps=%d\n",
	      __func__, on ? "ON" : "OFF",
	      pOx05b1sCtx->sensorMode.index,
	      pOx05b1sCtx->sensorMode.size.width,
	      pOx05b1sCtx->sensorMode.size.height,
	      pOx05b1sCtx->currFps / ISI_FPS_QUANTIZE);

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiGetAeBaseInfoIss
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
static RESULT Ox05b1s_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle,
					   IsiAeBaseInfo_t *pAeBaseInfo)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (pAeBaseInfo == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: NULL pointer received!!\n", __func__);
		return RET_NULL_POINTER;
	}

	pAeBaseInfo->gain.min        = pOx05b1sCtx->aecMinGain;
	pAeBaseInfo->gain.max        = pOx05b1sCtx->aecMaxGain;
	pAeBaseInfo->intTime.min     = pOx05b1sCtx->aecMinIntegrationTime;
	pAeBaseInfo->intTime.max     = pOx05b1sCtx->aecMaxIntegrationTime;

	pAeBaseInfo->aGain           = pOx05b1sCtx->aGain;
	pAeBaseInfo->dGain           = pOx05b1sCtx->dGain;

	pAeBaseInfo->aecCurGain      = pOx05b1sCtx->aecCurGain;
	pAeBaseInfo->aecCurIntTime   = pOx05b1sCtx->aecCurIntegrationTime;
	pAeBaseInfo->aecGainStep     = pOx05b1sCtx->aecGainIncrement;
	pAeBaseInfo->aecIntTimeStep  = pOx05b1sCtx->aecIntegrationTimeIncrement;

	pAeBaseInfo->aecIrLightExp    = pOx05b1sCtx->irLightExp;
	pAeBaseInfo->aecIrLightInfo   = pOx05b1sCtx->irLightInfo;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiSetAGainIss
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
RESULT Ox05b1s_IsiSetAGainIss(IsiSensorHandle_t handle,
			      IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);
	uint32_t again = 0;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] < pOx05b1sCtx->aGain.min) {
		TRACE(Ox05b1s_WARN, "%s: invalid too small again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->aGain.min;
	}

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] > pOx05b1sCtx->aGain.max) {
		TRACE(Ox05b1s_WARN, "%s: invalid too big again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->aGain.max;
	}

	again = (uint32_t)(pSensorAGain->gain[ISI_LINEAR_PARAS] * 16);

	TRACE(Ox05b1s_DEBUG,
	      "%s: in mode %d again %d\n",
	      __func__, pOx05b1sCtx->sensorMode.index, again);

	if (pOx05b1sCtx->sensorMode.index == 1 ||
		pOx05b1sCtx->sensorMode.index == 2) {
		result = Ox05b1s_IsiBufferABmodeParam(handle, OX05B1S_AB_MODE_A_GAIN, again);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: buffer again failed (%d)\n",
				__func__, result);
			return result;
		}
	} else {
		result = Ox05b1s_IsiWriteRegIss(handle, 0x3508, (again >> 4) & 0x0f);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3509, (again & 0x0f) << 4);
	}

	pOx05b1sCtx->curAgain = (float)again/16.0f;

	TRACE(Ox05b1s_INFO,
	      "%s: frame=%d mode=%d aGain=%.3f\n",
	      __func__, g_Sensor_frame_count,
	      pOx05b1sCtx->sensorMode.index,
	      pOx05b1sCtx->curAgain);

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiSetDGainIss
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
RESULT Ox05b1s_IsiSetDGainIss(IsiSensorHandle_t handle,
			      IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	uint32_t dgain = 0;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] < pOx05b1sCtx->dGain.min) {
		TRACE(Ox05b1s_WARN, "%s: invalid too small dgain parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->dGain.min;
	}

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] > pOx05b1sCtx->dGain.max) {
		TRACE(Ox05b1s_WARN, "%s: invalid too big dgain parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->dGain.max;
	}

	dgain = (uint32_t)(pSensorDGain->gain[ISI_LINEAR_PARAS] * 1024);
	TRACE(Ox05b1s_DEBUG,
	      "%s: in mode %d, dgain %d\n",
	      __func__, pOx05b1sCtx->sensorMode.index, dgain);

	if (pOx05b1sCtx->sensorMode.index == 1 ||
		pOx05b1sCtx->sensorMode.index == 2) {
		result = Ox05b1s_IsiBufferABmodeParam(handle,
			OX05B1S_AB_MODE_D_GAIN, dgain);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: buffer dgain failed (%d)\n",
				__func__, result);
			return result;
		}

		result = Ox05b1s_IsiSetABmodeExp(handle);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: CommitABmode failed (%d)\n", __func__, result);
			return result;
		}
	} else {
		result = Ox05b1s_IsiWriteRegIss(handle, 0x350a, (dgain >> 10) & 0x0f);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x350b, (dgain >> 2) & 0xff);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x350c, (dgain & 0x03) << 6);
	}

	pOx05b1sCtx->curDgain = (float)dgain/1024.0f;
	pOx05b1sCtx->aecCurGain = pOx05b1sCtx->curAgain * pOx05b1sCtx->curDgain;

	TRACE(Ox05b1s_INFO,
	      "%s: frame=%d mode=%d dGain=%.3f totalGain=%.3f\n",
	      __func__, g_Sensor_frame_count,
	      pOx05b1sCtx->sensorMode.index,
	      pOx05b1sCtx->curDgain,
	      pOx05b1sCtx->aecCurGain);

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          IsiGetAGainIss
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
RESULT Ox05b1s_IsiGetAGainIss(IsiSensorHandle_t handle,
			      IsiSensorGain_t *pSensorAGain)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_WRONG_HANDLE;
	}

	if (pSensorAGain == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pSensorAGain is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	pSensorAGain->gain[ISI_LINEAR_PARAS]       = pOx05b1sCtx->curAgain;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiGetDGainIss
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
RESULT Ox05b1s_IsiGetDGainIss(IsiSensorHandle_t handle,
			      IsiSensorGain_t *pSensorDGain)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_WRONG_HANDLE;
	}

	if (pSensorDGain == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pSensorDGain is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	pSensorDGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->curDgain;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiSetIntTimeIss
 *
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
RESULT Ox05b1s_IsiSetIntTimeIss(IsiSensorHandle_t handle,
				 const IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (!pOx05b1sCtx) {
		TRACE(Ox05b1s_ERROR,
		      "%s: Invalid sensor handle (NULL pointer detected)\n",
		      __func__);
		return RET_WRONG_HANDLE;
	}

	uint32_t expLine = 0;

	expLine = pSensorIntTime->intTime[ISI_LINEAR_PARAS] /
		  pOx05b1sCtx->oneLineExpTime;
	expLine = MIN(pOx05b1sCtx->maxIntegrationLine,
		      MAX(pOx05b1sCtx->minIntegrationLine, expLine));
	TRACE(Ox05b1s_DEBUG,
	      "%s: in mode %d, set expLine = 0x%04x\n",
	      __func__, pOx05b1sCtx->sensorMode.index, expLine);

	if (pOx05b1sCtx->sensorMode.index == 1 ||
		pOx05b1sCtx->sensorMode.index == 2) {
		result = Ox05b1s_IsiBufferABmodeParam(handle,
			OX05B1S_AB_MODE_EXP_LINE, expLine);
		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: buffer expLine failed (%d)\n",
				__func__, result);
			return result;
		}
	} else {
		result = Ox05b1s_IsiWriteRegIss(handle, 0x3500, (expLine >> 16) & 0xff);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3501, (expLine >> 8) & 0xff);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3502, (expLine & 0xff));
	}

	pOx05b1sCtx->aecCurIntegrationTime = expLine * pOx05b1sCtx->oneLineExpTime;

	TRACE(Ox05b1s_INFO,
	      "%s: frame=%d mode=%d intTime=%.6f expLine=0x%04x\n",
	      __func__, g_Sensor_frame_count,
	      pOx05b1sCtx->sensorMode.index,
	      pOx05b1sCtx->aecCurIntegrationTime,
	      expLine);

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiGetIntTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorIntTime           pointer to integration time to get
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT Ox05b1s_IsiGetIntTimeIss(IsiSensorHandle_t handle,
				IsiSensorIntTime_t *pSensorIntTime)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (!pOx05b1sCtx) {
		TRACE(Ox05b1s_ERROR,
			"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorIntTime) {
		TRACE(Ox05b1s_ERROR, "%s: pSensorIntTime is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	pSensorIntTime->intTime[ISI_LINEAR_PARAS] =
		pOx05b1sCtx->aecCurIntegrationTime;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          Ox05b1s_IsiSetIRLightExpIss
 *
 * @brief   Set IR light exposure parameters
 *
 * @param   handle       Sensor handle
 * @param   pIrExpParam  IR exposure parameters
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
RESULT Ox05b1s_IsiSetIRLightExpIss(IsiSensorHandle_t handle,
				   const IsiIrLightExp_t *pIrExpParam)
{
	RESULT result = RET_SUCCESS;
	uint32_t irStrobeLine = 0;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (!pOx05b1sCtx) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}
	if (pIrExpParam == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pIrExpParam is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pIrExpParam->irOn == BOOL_TRUE)
		irStrobeLine = MAX(MIN(pIrExpParam->irStrength,
				       pOx05b1sCtx->irLightInfo.irRangeInfo.maxIrStrength),
				   pOx05b1sCtx->irLightInfo.irRangeInfo.minIrStrength);
	else
		irStrobeLine = pOx05b1sCtx->irLightExp.irStrength;

	TRACE(Ox05b1s_DEBUG,
	      "%s: in mode %d, set irStrobeLine = 0x%04x\n",
	      __func__, pOx05b1sCtx->sensorMode.index, irStrobeLine);

	/* Strobe pulse width [31:0] */
	result  = Ox05b1s_IsiWriteRegIss(handle, 0x3b25, (irStrobeLine >> 24) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b26, (irStrobeLine >> 16) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b27, (irStrobeLine >> 8) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b28, irStrobeLine & 0xff);

	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s: Failed to set IR strobe width (%d)\n",
		      __func__, result);
		return result;
	}

	pOx05b1sCtx->irLightExp.irOn = pIrExpParam->irOn;
	pOx05b1sCtx->irLightExp.irStrength = irStrobeLine;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          Ox05b1s_IsiGetIRLightExpIss
 *
 * @brief   Get IR light exposure parameters
 *
 * @param   handle       Sensor handle
 * @param   pIrExpParam  IR exposure parameters output
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
RESULT Ox05b1s_IsiGetIRLightExpIss(IsiSensorHandle_t handle,
				   IsiIrLightExp_t *pIrExpParam)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (!pOx05b1sCtx) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pIrExpParam) {
		TRACE(Ox05b1s_ERROR, "%s: pIrExpParam is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	pIrExpParam->irOn = pOx05b1sCtx->irLightExp.irOn;
	pIrExpParam->irStrength = pOx05b1sCtx->irLightExp.irStrength;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          sensor_framecount_ox05b1s
 *
 * @brief   Read sensor frame counter
 *
 * @param   handle      Sensor handle
 *
 * @return  Frame count or error status
 *
 ************************************************************************/
int sensor_framecount_ox05b1s(IsiSensorHandle_t handle)
{
	u32 frame_counter;
	uint16_t read_buf[4] = {0};
	int Status;

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4613, &read_buf[0]);
	if (Status != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s:%d read failed (Status=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4612, &read_buf[1]);
	if (Status != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s:%d read failed (Status=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4611, &read_buf[2]);
	if (Status != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s:%d read failed (Status=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4610, &read_buf[3]);
	if (Status != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR,
		      "%s:%d read failed (Status=%d)\n",
		      __func__, __LINE__, Status);
		return Status;
	}
	frame_counter = ((read_buf[3] & 0xFF) << 24) | ((read_buf[2] & 0xFF) << 16) |
			((read_buf[1] & 0xFF) << 8) | (read_buf[0] & 0xFF);

	g_Sensor_frame_count = frame_counter;

	return RET_SUCCESS;
}

/*****************************************************************************
 *          Ox05b1s_IsiGetFpsIss
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
RESULT Ox05b1s_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	sensor_framecount_ox05b1s(handle);

	*pFps = pOx05b1sCtx->currFps;
	Fmc_Sensor_Statustask();

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox05b1s_IsiSetFpsIss
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
RESULT Ox05b1s_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
	RESULT result = RET_SUCCESS;
	int32_t NewVts = 0;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (fps > pOx05b1sCtx->maxFps) {
		TRACE(Ox05b1s_ERROR,
		      "%s: set fps(%d) out of range, correct to %d (%d, %d)\n",
		      __func__, fps, pOx05b1sCtx->maxFps,
		      pOx05b1sCtx->minFps, pOx05b1sCtx->maxFps);
		fps = pOx05b1sCtx->maxFps;
	}
	if (fps < pOx05b1sCtx->minFps) {
		TRACE(Ox05b1s_ERROR,
		      "%s: set fps(%d) out of range, correct to %d (%d, %d)\n",
		      __func__, fps, pOx05b1sCtx->minFps,
		      pOx05b1sCtx->minFps, pOx05b1sCtx->maxFps);
		fps = pOx05b1sCtx->minFps;
	}

	NewVts = pOx05b1sCtx->frameLengthLines *
		 pOx05b1sCtx->sensorMode.fps / fps;
	result  =  Ox05b1s_IsiWriteRegIss(handle, 0x380e, NewVts >> 8);
	result |=  Ox05b1s_IsiWriteRegIss(handle, 0x380f, NewVts & 0xff);
	pOx05b1sCtx->currFps              = fps;
	pOx05b1sCtx->curFrameLengthLines  = NewVts;
	pOx05b1sCtx->maxIntegrationLine   = pOx05b1sCtx->curFrameLengthLines - 30;
	pOx05b1sCtx->aecMaxIntegrationTime = pOx05b1sCtx->maxIntegrationLine *
					     pOx05b1sCtx->oneLineExpTime;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox05b1s_IsiGetIspStatusIss
 *
 * @brief   Get sensor isp status.
 *
 * @param   handle                    sensor instance handle
 * @param   pSensorIspStatus          sensor isp status
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT Ox05b1s_IsiGetIspStatusIss(IsiSensorHandle_t handle,
				  IsiIspStatus_t *pIspStatus)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_WRONG_HANDLE;
	}
	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	pIspStatus->useSensorAE  = false;
	pIspStatus->useSensorBLC = false;
	pIspStatus->useAWBMode = ISI_USE_ISP_WB_GAIN;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          Ox05b1s_IsiSetTpgIss
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
RESULT Ox05b1s_IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: pOx05b1sCtx is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pOx05b1sCtx->configured != BOOL_TRUE) {
		TRACE(Ox05b1s_ERROR, "%s: sensor not configured\n", __func__);
		return RET_WRONG_STATE;
	}

	if (tpg.enable == 0)
		result = Ox05b1s_IsiWriteRegIss(handle, 0x5100, 0x00);
	else
		result = Ox05b1s_IsiWriteRegIss(handle, 0x5100, 0x80);

	pOx05b1sCtx->testPattern = tpg.enable;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiGetTpgIss
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
RESULT Ox05b1s_IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *pTpg)
{
	RESULT result = RET_SUCCESS;
	uint16_t value = 0;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL || pTpg == NULL) {
		TRACE(Ox05b1s_ERROR,
		      "%s: NULL pointer (handle=%p, pTpg=%p)\n",
		      __func__, handle, pTpg);
		return RET_NULL_POINTER;
	}

	if (pOx05b1sCtx->configured != BOOL_TRUE) {
		TRACE(Ox05b1s_ERROR, "%s: sensor not configured\n", __func__);
		return RET_WRONG_STATE;
	}

	if (!Ox05b1s_IsiReadRegIss(handle, 0x5100, &value)) {
		pTpg->enable = ((value & 0x80) != 0) ? 1 : 0;
		if (pTpg->enable)
			pTpg->pattern = (0xff & value);
		pOx05b1sCtx->testPattern = pTpg->enable;
	}

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/****************************************************************************
 *          Ox05b1s_IsiGetSensorIss
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
RESULT Ox05b1s_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
	RESULT result = RET_SUCCESS;
	static const char SensorName[16] = "Ox05b1s";

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (pIsiSensor != NULL) {
		pIsiSensor->pszName                             = SensorName;
		pIsiSensor->pIsiCreateIss                       = Ox05b1s_IsiCreateIss;
		pIsiSensor->pIsiOpenIss                         = Ox05b1s_IsiOpenIss;
		pIsiSensor->pIsiCloseIss                        = Ox05b1s_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss                      = Ox05b1s_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss                      = Ox05b1s_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss                     = Ox05b1s_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss                      = Ox05b1s_IsiGetModeIss;
		pIsiSensor->pIsiEnumModeIss                     = Ox05b1s_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss                      = Ox05b1s_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss    = Ox05b1s_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss                  = Ox05b1s_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss                 = Ox05b1s_IsiSetStreamingIss;
		pIsiSensor->pIsiGetAeBaseInfoIss      = Ox05b1s_pIsiGetAeBaseInfoIss;
		pIsiSensor->pIsiGetAGainIss                     = Ox05b1s_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss                     = Ox05b1s_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss                     = Ox05b1s_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss                     = Ox05b1s_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss                   = Ox05b1s_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss                   = Ox05b1s_IsiSetIntTimeIss;
		pIsiSensor->pIsiGetFpsIss                       = Ox05b1s_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss                       = Ox05b1s_IsiSetFpsIss;
		pIsiSensor->pIsiGetIspStatusIss                 = Ox05b1s_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss                        = NULL;
		pIsiSensor->pIsiGetWBIss                        = NULL;
		pIsiSensor->pIsiSetBlcIss                       = NULL;
		pIsiSensor->pIsiGetBlcIss                       = NULL;
		pIsiSensor->pIsiSetTpgIss                       = Ox05b1s_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss                       = Ox05b1s_IsiGetTpgIss;
		pIsiSensor->pIsiGetExpandCurveIss               = NULL;
		pIsiSensor->pIsiFocusCreateIss                  = NULL;
		pIsiSensor->pIsiFocusReleaseIss                 = NULL;
		pIsiSensor->pIsiFocusGetCalibrateIss            = NULL;
		pIsiSensor->pIsiFocusSetIss                     = NULL;
		pIsiSensor->pIsiFocusGetIss                     = NULL;
		pIsiSensor->pIsiSetIRLightExpIss                =
			Ox05b1s_IsiSetIRLightExpIss;
		pIsiSensor->pIsiGetIRLightExpIss                =
			Ox05b1s_IsiGetIRLightExpIss;

	} else {
		result = RET_NULL_POINTER;
	}

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 ****************************************************************************/
IsiCamDrvConfig_t Ox05b1s_IsiCamDrvConfig = {
	.cameraDriverID      = OX05B1S_SENSOR_ID,
	.pIsiGetSensorIss    = Ox05b1s_IsiGetSensorIss,
};
