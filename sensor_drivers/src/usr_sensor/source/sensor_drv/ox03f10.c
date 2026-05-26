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
#include "sensor_drv/ox03f10_priv.h"

CREATE_TRACER(Ox03f10_INFO, "Ox03f10: ", INFO, 1);
CREATE_TRACER(Ox03f10_WARN, "Ox03f10: ", WARNING, 1);
CREATE_TRACER(Ox03f10_ERROR, "Ox03f10: ", ERROR, 1);
CREATE_TRACER(Ox03f10_DEBUG, "Ox03f10: ", INFO, 1);
CREATE_TRACER(Ox03f10_REG_INFO, "Ox03f10: ", INFO,    1);
CREATE_TRACER(Ox03f10_REG_DEBUG, "Ox03f10: ", INFO,    1);

#define Ox03f10_MIN_GAIN_STEP	(1.0f/1024.0f)

/*****************************************************************************
 *Sensor Info
 *****************************************************************************/

IsiSensorMode_t pox03f10_mode_info[] = {
	{
		.index     = 0,
		.size       = {
			.boundsWidth  = 1920,
			.boundsHeight = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.aeInfo    = {
			.intTimeRange[0].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* DCG 20ms */
			.intTimeRange[0].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[1].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* DCG 20ms */
			.intTimeRange[1].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[2].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* SPD 20ms */
			.intTimeRange[2].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[3].max =
				0.00004035 * 31 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* VS 20ms */
			.intTimeRange[3].min =
				0.00004035 * 1 * ISI_INTEGRATION_TIME_QUANTIZE,
			.aGainRange[0] = {1.0, 15.5, (1.0f/16.0f)},//HCG again
			.aGainRange[1] = {1.0, 15.5, (1.0f/16.0f)},//LCG again
			.aGainRange[2] = {4.25, 15.5, (1.0f/16.0f)},//SPD again
			.aGainRange[3] = {1.0, 15.5, (1.0f/16.0f)},//VS again
			.dGainRange[0] = {1.0, 1.0, (1.0f/1024.0f)},//HCG dgain
			.intTimeDelayFrame = 0,
			.gainDelayFrame = 0,
		},
		.fps       = 30 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_HDR_NATIVE,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 12,
		.compress.enable = 1,
		.compress.xBit  = 24,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType = ISI_MODE_BAYER,
		.mipiLane = ISI_MIPI_4LANES,
	},
	{
		.index     = 1,
		.size       = {
			.boundsWidth  = 1280,
			.boundsHeight = 720,
			.top           = 0,
			.left          = 0,
			.width         = 1280,
			.height        = 720,
		},
		.aeInfo    = {
			.intTimeRange[0].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
				/* DCG 20ms */
			.intTimeRange[0].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[1].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* DCG 20ms */
			.intTimeRange[1].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[2].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* SPD 20ms */
			.intTimeRange[2].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[3].max =
				0.00004035 * 31 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* VS 20ms */
			.intTimeRange[3].min =
				0.00004035 * 1 * ISI_INTEGRATION_TIME_QUANTIZE,
			.aGainRange[0] = {1.0, 15.5, (1.0f/16.0f)},//HCG again
			.aGainRange[1] = {1.0, 15.5, (1.0f/16.0f)},//LCG again
			.aGainRange[2] = {4.25, 15.5, (1.0f/16.0f)},//SPD again
			.aGainRange[3] = {1.0, 15.5, (1.0f/16.0f)},//VS again
			.dGainRange[0] = {1.0, 1.0, (1.0f/1024.0f)},//HCG dgain
			.intTimeDelayFrame = 0,
			.gainDelayFrame = 0,
		},
		.fps       = 30 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_HDR_NATIVE,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 12,
		.compress.enable = 1,
		.compress.xBit  = 24,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType = ISI_MODE_BAYER,
		.mipiLane = ISI_MIPI_4LANES,
	},
	{
		.index     = 2,
		.size       = {
			.boundsWidth  = 640,
			.boundsHeight = 480,
			.top           = 0,
			.left          = 0,
			.width         = 640,
			.height        = 480,
		},
		.aeInfo    = {
			.intTimeRange[0].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* DCG 20ms */
			.intTimeRange[0].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[1].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* DCG 20ms */
			.intTimeRange[1].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[2].max =
				0.00004035 * 496 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* SPD 20ms */
			.intTimeRange[2].min =
				0.00004035 * 2 * ISI_INTEGRATION_TIME_QUANTIZE,
			.intTimeRange[3].max =
				0.00004035 * 31 * ISI_INTEGRATION_TIME_QUANTIZE,
			/* VS 20ms */
			.intTimeRange[3].min =
				0.00004035 * 1 * ISI_INTEGRATION_TIME_QUANTIZE,
			.aGainRange[0] = {1.0, 15.5, (1.0f/16.0f)},//HCG again
			.aGainRange[1] = {1.0, 15.5, (1.0f/16.0f)},//LCG again
			.aGainRange[2] = {4.25, 15.5, (1.0f/16.0f)},//SPD again
			.aGainRange[3] = {1.0, 15.5, (1.0f/16.0f)},//VS again
			.dGainRange[0] = {1.0, 1.0, (1.0f/1024.0f)},//HCG dgain
			.intTimeDelayFrame = 0,
			.gainDelayFrame = 0,
		},
		.fps       = 30 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_HDR_NATIVE,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 12,
		.compress.enable = 1,
		.compress.xBit  = 24,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
		.dataType = ISI_MODE_BAYER,
		.mipiLane = ISI_MIPI_4LANES,
	}
};

int ox03f10_mode_num = (int)ARRAY_SIZE(pox03f10_mode_info);

/*****************************************************************************
 *          Ox03f10_IsiReadRegIss
 *
 * @brief   reads a given number of bytes from the image sensor device
 *
 * @param   handle              Handle to image sensor device
 * @param   addr                register address
 * @param   pValue              value to read
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiReadRegIss(IsiSensorHandle_t handle,
				    const uint16_t addr,
				    uint16_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint8_t read_val = 0;

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pValue == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid parameter (NULL pointer)\n",
				__func__);
		return RET_NULL_POINTER;
	}

	memset(pValue, 0, sizeof(uint16_t));
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(Ox03f10_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pOx03f10Ctx->sensorDevId]
		 ->sensor_alias_addr) >> 1;
	result = active_fmc->accessiic_array[pOx03f10Ctx->sensorDevId]
		->readIIC(pOx03f10Ctx->i2cId,
				slave_addr, addr, 0x2, &read_val, 1);


	if (result !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: I2C read failed at addr 0x%04x (err=%d)\n",
				__func__, addr, result);
		return RET_FAILURE;
	}
	*pValue = (uint16_t)read_val;
	*pValue &= 0xff;


	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiWriteRegIss
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
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiWriteRegIss(IsiSensorHandle_t handle,
				     const uint16_t addr,
				     const uint16_t value)
{
	RESULT result = RET_SUCCESS;

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(Ox03f10_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}

	u8 slave_addr =
		(active_fmc->sensor_array[pOx03f10Ctx->sensorDevId]
		 ->sensor_alias_addr) >> 1;
	u8 wr_data[2];

	wr_data[0] = (u8)value;
	result = active_fmc->accessiic_array[pOx03f10Ctx->sensorDevId]
		->writeIIC(pOx03f10Ctx->i2cId
				, slave_addr, addr, 0x2, wr_data, 1);


	if (result !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: hal write sensor register error! (err=%d)\n",
			__func__, result);
		return RET_FAILURE;
	}

	return result;
}


#ifdef ENABLE_I2C_GROUPING
/*****************************************************************************
 *          Ox03f10_IsiWriteRegIss
 *
 * @brief   writes a given number of bytes to the image sensor device by
 *          calling the corresponding sensor-function
 *
 * @param   handle              Handle to image sensor device
 * @param   addr                register address
 * @param   value               value to write
 * @param   datacount           number of bytes to write
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiWriteRegGroupIss(IsiSensorHandle_t handle,
		const uint16_t addr, uint8_t *value, uint8_t datacount)
{
	RESULT result = RET_SUCCESS;

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (datacount > 4U)
		return RET_FAILURE;

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(Ox03f10_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}


	u8 slave_addr =
		(active_fmc->sensor_array[pOx03f10Ctx->sensorDevId]
		 ->sensor_alias_addr) >> 1;
	result = active_fmc->accessiic_array[pOx03f10Ctx->sensorDevId]
		->writeIIC(pOx03f10Ctx->i2cId
				, slave_addr, addr, 0x2, value, datacount);


	if (result !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: I2C group write failed at addr 0x%04x (err=%d)\n",
				__func__, addr, result);
		return RET_FAILURE;
	}

	return result;
}
#endif


/*****************************************************************************
 *          Ox03f10_IsiGetModeIss
 *
 * @brief   get cuurent sensor mode info.
 *
 * @param   handle      Sensor instance handle
 * @param   pMode       Pointer to store current sensor mode info
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiGetModeIss(IsiSensorHandle_t handle,
		IsiSensorMode_t *pMode)
{
	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pMode == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid parameter (NULL pointer)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	memcpy(pMode, &(pOx03f10Ctx->sensorMode), sizeof(pOx03f10Ctx->sensorMode));

	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          Ox03f10_IsiEnumModeIss
 *
 * @brief   query sensor info.
 *
 * @param   handle                  sensor instance handle
 * @param   pEnumMode               sensor query mode
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFRANGE
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiEnumModeIss(IsiSensorHandle_t handle,
		IsiSensorEnumMode_t *pEnumMode)
{
	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pEnumMode == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid parameter (NULL pointer)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	if (pEnumMode->index >= ARRAY_SIZE(pox03f10_mode_info))
		return RET_OUTOFRANGE;

	for (uint32_t mode_idx = 0;
	     mode_idx < ARRAY_SIZE(pox03f10_mode_info); mode_idx++) {
		if (pox03f10_mode_info[mode_idx].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &pox03f10_mode_info[mode_idx],
			    sizeof(IsiSensorMode_t));
			TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
			return RET_SUCCESS;
		}
	}

	return RET_NOTSUPP;
}
/*****************************************************************************
 *          Ox03f10_IsiGetCapsIss
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
static RESULT Ox03f10_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	if (pOx03f10Ctx == NULL || pCaps == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid parameter (NULL pointer)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	pCaps->bitWidth          = pOx03f10Ctx->sensorMode.bitWidth;
	pCaps->mode              = ISI_MODE_BAYER;
	pCaps->bayerPattern      = pOx03f10Ctx->sensorMode.bayerPattern;
	pCaps->resolution.width  = pOx03f10Ctx->sensorMode.size.width;
	pCaps->resolution.height = pOx03f10Ctx->sensorMode.size.height;
	pCaps->mipiLanes         = ISI_MIPI_4LANES;
	pCaps->vinType           = ISI_ITF_TYPE_MIPI;

	if (pCaps->bitWidth == 10)
		pCaps->mipiMode      = ISI_FORMAT_RAW_10;
	else if (pCaps->bitWidth == 12)
		pCaps->mipiMode      = ISI_FORMAT_RAW_12;
	else
		pCaps->mipiMode      = ISI_MIPI_OFF;

	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiCreateIss
 *
 * @brief   Create Sensor Context for the given config
 *
 * @param   pConfig	Given Sensor Config
 * @param   pHandle	Return the Sensor Ctx
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 * @retval  RET_OUTOFMEM
 * @retval  RET_UNSUPPORT_ID
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig,
		IsiSensorHandle_t *pHandle)
{
	RESULT result = RET_SUCCESS;
	uint32_t desId = 0, pipeId = 0;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx =
		(Ox03f10_Context_t *)osMalloc(sizeof(Ox03f10_Context_t));

	if (pConfig == NULL || pHandle == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid parameter (NULL pointer)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	if (!pOx03f10Ctx) {
		TRACE(Ox03f10_ERROR, "%s: Can't allocate ox03f10 context\n", __func__);
		return RET_OUTOFMEM;
	}

	MEMSET(pOx03f10Ctx, 0, sizeof(Ox03f10_Context_t));

	pOx03f10Ctx->isiCtx.pSensor		= pConfig->pSensor;
	pOx03f10Ctx->configured			= BOOL_FALSE;
	pOx03f10Ctx->streaming			= BOOL_FALSE;
	pOx03f10Ctx->testPattern		= BOOL_FALSE;
	pOx03f10Ctx->sensorMode.index		= 0;
	pOx03f10Ctx->sensorDevId		= pConfig->halDevID;
	pOx03f10Ctx->instanceId			= pConfig->instanceID;

	pipeId = pOx03f10Ctx->sensorDevId;

	*pHandle = (IsiSensorHandle_t) pOx03f10Ctx;

	if (pipeId >= IN_PIPE_LAST) {
		TRACE(Ox03f10_ERROR,
		      "%s: sensor device ID %d is not supported!\n",
		      __func__, pipeId);
		osFree(pOx03f10Ctx);
		return RET_UNSUPPORT_ID;
	}
	desId = MAPPING_INPIPE_TO_DES_ID(pipeId);

	/* REFACTORING: Get I2C bus ID from centralized core_des_map
	 *
	 * EVOLUTION OF I2C BUS ASSIGNMENT:
	 *   V1 (OLD):  pOx03f10Ctx->i2cId = AXI_IIC_INSTANCE_ZERO;
	 *              ? Hard-coded, inflexible
	 *
	 *   V2 (OLD):  pOx03f10Ctx->i2cId = des_arr[desId].i2cBusId;
	 *              ? Better, but duplicated in des_arr structure
	 *
	 *   V3 (NEW):  pOx03f10Ctx->i2cId = GetI2cBusIdForDes(desId);
	 *              ? Centralized in core_des_map, single source of truth
	 *
	 * BENEFITS:
	 *   - All topology in one place (core_des_map in max9296.c)
	 *   - Easier hardware reconfiguration
	 *   - Better separation of concerns
	 */
	pOx03f10Ctx->i2cId = GetI2cBusIdForDes(desId);
	uint8_t busId = (uint8_t)pOx03f10Ctx->i2cId;

	/* Validate I2C bus ID */
	if (busId == INVALID_I2C_BUS_ID) {
		TRACE(Ox03f10_ERROR,
		      "%s: Invalid I2C bus ID for desId %d\n",
		      __func__, desId);
		osFree(pOx03f10Ctx);
		return RET_FAILURE;
	}

	TRACE(Ox03f10_INFO,
			"%s: desId: %d, pipeId: %d, i2cBusId:%d\r\n",
			__func__, desId, pipeId, busId);

	result = init_iic_access(busId, pipeId);
	if (result != RET_SUCCESS) {
		TRACE(Ox03f10_ERROR,
		      "%s: init_iic_access failed for pipe %d (err=%d)\n",
		      __func__, pipeId, result);
		osFree(pOx03f10Ctx);
		return result;
	}
	result = init_des(desId);
	if (result != RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: init_des failed! (err=%d)\n", __func__, result);
		osFree(pOx03f10Ctx);
		return RET_FAILURE;
	}
	result = init_sensor(pipeId, desId, SENSOR_OX3F10_ADDRESS);
	if (result != RET_SUCCESS) {
		TRACE(Ox03f10_ERROR,
		      "%s: init_sensor failed for pipe %d (err=%d)\n",
		      __func__, pipeId, result);
		osFree(pOx03f10Ctx);
		return result;
	}

	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_AecSetModeParameters
 *
 * @brief   Sets AEC mode parameters for the sensor.
 *
 * @param   handle                  sensor instance handle
 * @param   pOx03f10Ctx             sensor context
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT Ox03f10_AecSetModeParameters(IsiSensorHandle_t handle,
	Ox03f10_Context_t *pOx03f10Ctx)
{
	RESULT result = RET_SUCCESS;

	uint32_t exp_line = 0, again = 0, dgain = 0;
	uint16_t value = 0;

	pOx03f10Ctx->aecIntegrationTimeIncrement = pOx03f10Ctx->oneLineExpTime;
	pOx03f10Ctx->aecGainIncrement = Ox03f10_MIN_GAIN_STEP;
	pOx03f10Ctx->aecMinIntegrationTime       = pOx03f10Ctx->oneLineExpTime
		* pOx03f10Ctx->minDCGIntegrationLine;
	pOx03f10Ctx->aecMaxIntegrationTime       = pOx03f10Ctx->oneLineExpTime
		* pOx03f10Ctx->maxDCGIntegrationLine;
	TRACE(Ox03f10_DEBUG, "%s: AecMaxIntegrationTime = %f\n", __func__,
			pOx03f10Ctx->aecMaxIntegrationTime);
	/* reflects the state of the sensor registers, must equal default settings */
	/*get current dcg exp time*/
	result = Ox03f10_IsiReadRegIss(handle, 0x3501, &value);

	exp_line = (value & 0xff) << 8;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3502, &value);
	exp_line = exp_line | (value & 0xff);
	pOx03f10Ctx->aecCurIntTime.intTime[0] =
		exp_line * pOx03f10Ctx->oneLineExpTime;
	pOx03f10Ctx->aecCurIntTime.intTime[1] =
		exp_line * pOx03f10Ctx->oneLineExpTime;

	/*get current spd exp time*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x3541, &value);
	exp_line = (value & 0xff) << 8;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3542, &value);
	exp_line = exp_line | (value & 0xff);
	pOx03f10Ctx->aecCurIntTime.intTime[2] =
		exp_line * pOx03f10Ctx->oneLineExpTime;

	/*get current vs exp time*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c1, &value);
	exp_line = (value & 0xff) << 8;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c2, &value);
	exp_line = exp_line | (value & 0xff);
	pOx03f10Ctx->aecCurIntTime.intTime[3] =
		exp_line * pOx03f10Ctx->oneLineExpTime;

	/*get current hcg again*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x3508, &value);
	again = (value & 0x0f) << 4;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3509, &value);

	again = again | ((value & 0xf0) >> 4);
	pOx03f10Ctx->curAgain.gain[0] = (float32_t)again/16.0f;
	/*get current hcg dgain*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x350a, &value);
	dgain = (value & 0x0f) << 10;
	result |= Ox03f10_IsiReadRegIss(handle, 0x350b, &value);
	dgain = dgain | ((value & 0xff) << 2);
	result |= Ox03f10_IsiReadRegIss(handle, 0x350c, &value);
	dgain = dgain | ((value & 0xc0) >> 6);
	pOx03f10Ctx->curDgain.gain[0] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[0] =
		pOx03f10Ctx->curAgain.gain[0] * pOx03f10Ctx->curDgain.gain[0];

	/*get current lcg again*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x3588, &value);
	again = (value & 0x0f) << 4;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3589, &value);

	again = again | ((value & 0xf0) >> 4);
	pOx03f10Ctx->curAgain.gain[1] = (float32_t)again/16.0f;
	/*get current lcg dgain*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x358a, &value);
	dgain = (value & 0x0f) << 10;
	result |= Ox03f10_IsiReadRegIss(handle, 0x358b, &value);
	dgain = dgain | ((value & 0xff) << 2);
	result |= Ox03f10_IsiReadRegIss(handle, 0x358c, &value);
	dgain = dgain | ((value & 0xc0) >> 6);
	pOx03f10Ctx->curDgain.gain[1] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[1] =
		pOx03f10Ctx->curAgain.gain[1] * pOx03f10Ctx->curDgain.gain[1];

	/*get current spd again*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x3548, &value);
	again = (value & 0x0f) << 4;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3549, &value);

	again = again | ((value & 0xf0) >> 4);
	pOx03f10Ctx->curAgain.gain[2] = (float32_t)again/16.0f;
	/*get current spd dgain*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x354a, &value);
	dgain = (value & 0x0f) << 10;
	result |= Ox03f10_IsiReadRegIss(handle, 0x354b, &value);
	dgain = dgain | ((value & 0xff) << 2);
	result |= Ox03f10_IsiReadRegIss(handle, 0x354c, &value);
	dgain = dgain | ((value & 0xc0) >> 6);
	pOx03f10Ctx->curDgain.gain[2] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[2] =
		pOx03f10Ctx->curAgain.gain[2] * pOx03f10Ctx->curDgain.gain[2];

	/*get current vs again*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c8, &value);
	again = (value & 0x0f) << 4;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c9, &value);
	again = again | ((value & 0xf0) >> 4);
	pOx03f10Ctx->curAgain.gain[3] = (float32_t)again/16.0f;
	/*get current vs dgain*/
	result |= Ox03f10_IsiReadRegIss(handle, 0x35ca, &value);
	dgain = (value & 0x0f) << 10;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35cb, &value);
	dgain = dgain | ((value & 0xff) << 2);
	result |= Ox03f10_IsiReadRegIss(handle, 0x35cc, &value);
	dgain = dgain | ((value & 0xc0) >> 6);
	pOx03f10Ctx->curDgain.gain[3] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[3] =
		pOx03f10Ctx->curAgain.gain[3] * pOx03f10Ctx->curDgain.gain[3];

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiOpenIss
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
static RESULT Ox03f10_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	if (!pOx03f10Ctx) {
		TRACE(Ox03f10_ERROR,
				"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_NULL_POINTER;
	}

	if (pOx03f10Ctx->streaming !=  BOOL_FALSE)
		return RET_WRONG_STATE;

	pOx03f10Ctx->sensorMode.index   = mode;
	IsiSensorMode_t *SensorDefaultMode = NULL;

	for (unsigned int mode_idx = 0;
	    mode_idx < ARRAY_SIZE(pox03f10_mode_info); mode_idx++) {
		if (pox03f10_mode_info[mode_idx].index ==
				pOx03f10Ctx->sensorMode.index) {
			SensorDefaultMode = &(pox03f10_mode_info[mode_idx]);
			break;
		}
	}

	if (SensorDefaultMode != NULL) {
		switch (SensorDefaultMode->index) {
		case 0:
			for (unsigned int reg_idx = 0;
			     reg_idx < ARRAY_SIZE(Ox03f10_mipi4lane_1080p_native4dol_init);
			     reg_idx++) {
				if (Ox03f10_mipi4lane_1080p_native4dol_init[reg_idx][0] ==
						OX03F10_TABLE_WAIT) {
					osSleep(Ox03f10_mipi4lane_1080p_native4dol_init[reg_idx][1]);
					continue;
				}
				if (Ox03f10_mipi4lane_1080p_native4dol_init[reg_idx][0] ==
				    OX03F10_TABLE_END)
					break;

				result = Ox03f10_IsiWriteRegIss(handle,
					Ox03f10_mipi4lane_1080p_native4dol_init[reg_idx][0],
					Ox03f10_mipi4lane_1080p_native4dol_init[reg_idx][1]);
				if (result != RET_SUCCESS) {
					TRACE(Ox03f10_ERROR, "%s: Failed to write reg! (err=%d)\n",
					__func__, result);
					return result;
				}
			}
			break;
		case 1:
			for (unsigned int reg_idx = 0;
					reg_idx <
					ARRAY_SIZE(Ox03f10_mipi4lane_1080p_native4dol_1280_720_init);
					reg_idx++) {
				if (Ox03f10_mipi4lane_1080p_native4dol_1280_720_init[reg_idx][0] ==
						OX03F10_TABLE_WAIT) {
					osSleep(Ox03f10_mipi4lane_1080p_native4dol_1280_720_init[reg_idx][1]);
					continue;
				}
				if (Ox03f10_mipi4lane_1080p_native4dol_1280_720_init[reg_idx][0] ==
						OX03F10_TABLE_END)
					break;

				result = Ox03f10_IsiWriteRegIss(handle,
					Ox03f10_mipi4lane_1080p_native4dol_1280_720_init[reg_idx][0],
					Ox03f10_mipi4lane_1080p_native4dol_1280_720_init[reg_idx][1]);
				if (result != RET_SUCCESS) {
					TRACE(Ox03f10_ERROR, "%s: Failed to write reg! (err=%d)\n",
						__func__, result);
					return result;
				}
			}
			break;
		case 2:
			for (unsigned int reg_idx = 0;
					reg_idx <
					ARRAY_SIZE(Ox03f10_mipi4lane_1080p_native4dol_640_480_init);
					reg_idx++) {
				if (Ox03f10_mipi4lane_1080p_native4dol_640_480_init[reg_idx][0] ==
						OX03F10_TABLE_WAIT) {
					osSleep(Ox03f10_mipi4lane_1080p_native4dol_640_480_init[reg_idx][1]);
					continue;
				}
				if (Ox03f10_mipi4lane_1080p_native4dol_640_480_init[reg_idx][0] ==
						OX03F10_TABLE_END)
					break;

				result = Ox03f10_IsiWriteRegIss(handle,
					Ox03f10_mipi4lane_1080p_native4dol_640_480_init[reg_idx][0],
					Ox03f10_mipi4lane_1080p_native4dol_640_480_init[reg_idx][1]);
				if (result != RET_SUCCESS) {
					TRACE(Ox03f10_ERROR, "%s: Failed to write reg! (err=%d)\n",
						__func__, result);
						return result;
				}
			}
			break;
		default:
			TRACE(Ox03f10_INFO, "%s:not support sensor mode %d\n", __func__,
				pOx03f10Ctx->sensorMode.index);
			osFree(pOx03f10Ctx);
			return RET_NOTSUPP;
		}

		memcpy(&(pOx03f10Ctx->sensorMode), SensorDefaultMode,
				sizeof(IsiSensorMode_t));
	} else {
		TRACE(Ox03f10_ERROR, "%s: Invalid SensorDefaultMode\n", __func__);
		return RET_NULL_POINTER;
	}

	switch (pOx03f10Ctx->sensorMode.index) {
	case 0:
	case 1:
	case 2:
	pOx03f10Ctx->oneLineExpTime       = 0.00004035;
	//frame_length VTS
	pOx03f10Ctx->frameLengthLines     = 0x330;
	pOx03f10Ctx->curFrameLengthLines  = pOx03f10Ctx->frameLengthLines;
	//DCG_shutter
	pOx03f10Ctx->maxDCGIntegrationLine = 496;
	pOx03f10Ctx->minDCGIntegrationLine = 2;
	//SPD_shutter
	pOx03f10Ctx->maxSPDIntegrationLine = 496;
	pOx03f10Ctx->minSPDIntegrationLine = 2;
	//VS_shutter
	pOx03f10Ctx->maxVSIntegrationLine = 31;
	pOx03f10Ctx->minVSIntegrationLine = 1;
	//AE_totalGain
	pOx03f10Ctx->aecMaxGain           = 240;
	pOx03f10Ctx->aecMinGain           = 1.0;
	break;
	default:
		TRACE(Ox03f10_INFO,
			"%s:not support sensor mode %d\n",
			__func__, pOx03f10Ctx->sensorMode.index);
		return RET_NOTSUPP;
	}

	MEMCPY(&pOx03f10Ctx->aGainHCG,
			&pox03f10_mode_info[pOx03f10Ctx->sensorMode.index].aeInfo.aGainRange[0],
			sizeof(IsiGainInfo_t));
	MEMCPY(&pOx03f10Ctx->aGainLCG,
			&pox03f10_mode_info[pOx03f10Ctx->sensorMode.index].aeInfo.aGainRange[1],
			sizeof(IsiGainInfo_t));
	MEMCPY(&pOx03f10Ctx->aGainSPD,
			&pox03f10_mode_info[pOx03f10Ctx->sensorMode.index].aeInfo.aGainRange[2],
			sizeof(IsiGainInfo_t));
	MEMCPY(&pOx03f10Ctx->aGainVS,
			&pox03f10_mode_info[pOx03f10Ctx->sensorMode.index].aeInfo.aGainRange[3],
			sizeof(IsiGainInfo_t));
	MEMCPY(&pOx03f10Ctx->dGain,
			&pox03f10_mode_info[pOx03f10Ctx->sensorMode.index].aeInfo.dGainRange[0],
			sizeof(IsiGainInfo_t));

	pOx03f10Ctx->maxFps  = pOx03f10Ctx->sensorMode.fps;
	pOx03f10Ctx->minFps  = 0.25 * ISI_FPS_QUANTIZE;
	pOx03f10Ctx->currFps = pOx03f10Ctx->maxFps;

	TRACE(Ox03f10_DEBUG, "%s: Ox03f10 System-Reset executed\n", __func__);

	result = Ox03f10_AecSetModeParameters(handle, pOx03f10Ctx);
	if (result !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: Failed to set AEC mode parameters. (err=%d)\n",
			__func__, result);
		return result;
	}

	pOx03f10Ctx->configured = BOOL_TRUE;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          Ox03f10_IsiCloseIss
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
static RESULT Ox03f10_IsiCloseIss(IsiSensorHandle_t handle)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	result = Ox03f10_IsiSetStreamingIss(pOx03f10Ctx, BOOL_FALSE);
	if (result !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: Failed to set streaming. (err=%d)\n", __func__, result);
		return result;
	}

	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          Ox03f10_IsiReleaseIss
 *
 * @brief   Release sensor instance
 *
 * @param   handle      Sensor handle
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
static RESULT Ox03f10_IsiReleaseIss(IsiSensorHandle_t handle)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	stop_sensor(pOx03f10Ctx->sensorDevId);

	MEMSET(pOx03f10Ctx, 0, sizeof(Ox03f10_Context_t));
	osFree(pOx03f10Ctx);
	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiCheckConnectionIss
 *
 * @brief   Checks the connection to the camera sensor, if possible.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiCheckConnectionIss(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	uint32_t sensor_id = 0;
	uint32_t correct_id = OX03F10_SENSOR_ID;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	result = Ox03f10_IsiGetRevisionIss(handle, &sensor_id);
	if (result !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: Read Sensor ID Error! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	if (correct_id !=  sensor_id) {
		TRACE(Ox03f10_ERROR, "%s:ChipID  = 0x%x sensor_id = %x error!\n",
			__func__, correct_id, sensor_id);
			return RET_FAILURE;
	}

	TRACE(Ox03f10_INFO,
	    "%s ChipID = 0x%08x, sensor_id = 0x%08x, success!\n",
	    __func__, correct_id, sensor_id);
	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetRevisionIss
 *
 * @brief   This function reads the sensor revision register and returns it.
 *
 * @param   handle      sensor instance handle
 * @param   pRevision   pointer to revision
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiGetRevisionIss(IsiSensorHandle_t handle,
		uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val;
	uint32_t sensor_id;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pValue == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid parameter (NULL pointer)\n",
			__func__);
		return RET_NULL_POINTER;
}

	reg_val   = 0;
	result    = Ox03f10_IsiReadRegIss(handle, 0x300a, &reg_val);
	sensor_id = (reg_val & 0xff) << 16;

	reg_val   = 0;
	result    |=  Ox03f10_IsiReadRegIss(handle, 0x300b, &reg_val);
	sensor_id |=  ((reg_val & 0xff) << 8);

	reg_val   = 0;
	result    |=  Ox03f10_IsiReadRegIss(handle, 0x300c, &reg_val);
	sensor_id |=  (reg_val & 0xff);

	*pValue = sensor_id;
	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new streaming state (BOOL_TRUE = on, BOOL_FALSE = off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT Ox03f10_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pOx03f10Ctx->configured !=  BOOL_TRUE)
		return RET_WRONG_STATE;

	result = Ox03f10_IsiWriteRegIss(handle, 0x0100, on);
	if (result !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR, "%s: set sensor streaming error! (err=%d)\n", __func__, result);
		return RET_FAILURE;
	}

	pOx03f10Ctx->streaming = on;

	TRACE(Ox03f10_INFO,
	      "%s: streaming=%s mode=%d res=%dx%d fps=%d\n",
	      __func__, on ? "ON" : "OFF",
	      pOx03f10Ctx->sensorMode.index,
	      pOx03f10Ctx->sensorMode.size.width,
	      pOx03f10Ctx->sensorMode.size.height,
	      pOx03f10Ctx->currFps / ISI_FPS_QUANTIZE);

	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_pIsiGetAeBaseInfoIss
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
static RESULT Ox03f10_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle,
		IsiAeBaseInfo_t *pAeBaseInfo)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	if (pOx03f10Ctx == NULL || pAeBaseInfo == NULL) {
		TRACE(Ox03f10_ERROR,
			"%s: Invalid sensor handle or AeBaseInfo pointer (NULL)\n",
			__func__);
		return RET_NULL_POINTER;
	}

    //get time limit and total gain limit
	pAeBaseInfo->longGain.min        = pOx03f10Ctx->aecMinGain;
	pAeBaseInfo->longGain.max        = pOx03f10Ctx->aecMaxGain;
	pAeBaseInfo->longIntTime.min     = pOx03f10Ctx->aecMinIntegrationTime;
	pAeBaseInfo->longIntTime.max     = pOx03f10Ctx->aecMaxIntegrationTime;

    //get again/dgain min/max/step info
	pAeBaseInfo->aLongGain           = pOx03f10Ctx->aGainHCG;
	pAeBaseInfo->dLongGain           = pOx03f10Ctx->dGain;

    //get current intTime and gain
	pAeBaseInfo->curIntTime  = pOx03f10Ctx->aecCurIntTime;
	pAeBaseInfo->curGain     = pOx03f10Ctx->aecCurGain;

	pAeBaseInfo->aecGainStep = pOx03f10Ctx->aecGainIncrement;
	pAeBaseInfo->aecIntTimeStep = pOx03f10Ctx->aecIntegrationTimeIncrement;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/**************************************************************************
 *          Ox03f10_IsiExcuteExpControlIss
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
RESULT Ox03f10_IsiExcuteExpControlIss(
	IsiSensorHandle_t handle,
	const IsiSensorExpParam_t *pExpParam,
	IsiSensorExpParam_t *pExpResult)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pExpParam == NULL || pExpResult == NULL) {
		TRACE(Ox03f10_ERROR, "%s: Invalid parameter (NULL pointer)\n",
				__func__);
		return RET_NULL_POINTER;
	}
	TRACE(Ox03f10_INFO, "%s: input intTime: %02f,again: %02f ,"
		"dgain:%02f,ratio[0]: %f, ispDg: %f\n",
			__func__, pExpParam->sensorExpTime[0],
			pExpParam->sensorAgain[0], pExpParam->sensorDgain[0],
			pExpParam->sensorRatio[0], pExpParam->ispGain);

/*
 * Exposure stratage:
 * Daylight:
 *@exp: HCG = LCG = SPD = 10ms   VS = 0.0415ms
 *@gain:   HCG : LCG :  SPD : VS
 *	   3 :   1 : 4.25 :  1
 * Night:
 *@exp: HCG = LCG = SPD = 20ms   VS = 0.083ms
 *@gain:   HCG : LCG :  SPD : VS
 *	   3 :   1 : 4.25 :  1
 *
 * exp : 5~10ms Continuous change, 10~20ms Discrete change
 * Note: must use ae flicker mode, set "antiFlikerMode" to mode 1 in auto json
 */

	float32_t exp = pExpParam->sensorExpTime[0];

	float32_t exp_vs = exp / OVX3F_LCG_VS_EXP_RATIO;
	float32_t total_gain =
		pExpParam->sensorAgain[0] * pExpParam->sensorDgain[0] / 3;
	float32_t realGain, realAGain, realDGain = 1.0;

	/* calculate hcg line & gain */
	pExpResult->sensorExpTime[0] =
		MIN(pOx03f10Ctx->maxDCGIntegrationLine * pOx03f10Ctx->oneLineExpTime,
				MAX(pOx03f10Ctx->minDCGIntegrationLine * pOx03f10Ctx->oneLineExpTime,
					exp));

	realGain = total_gain * 3;
	realAGain = realGain;//total gain-> all Again
	pExpResult->sensorAgain[0] =
		MAX(pOx03f10Ctx->aGainHCG.min,
				MIN(realAGain, pOx03f10Ctx->aGainHCG.max));
	realDGain = realGain /  pExpResult->sensorAgain[0];//realDGain always 1x
	pExpResult->sensorDgain[0] =
		MAX(pOx03f10Ctx->dGain.min,
				MIN(realDGain, pOx03f10Ctx->dGain.max));

	/* calculate lcg gain */
	pExpResult->sensorExpTime[1] = pExpResult->sensorExpTime[0];//HCG LCG same Exp

	realGain = total_gain;//=HCG gain/3
	realAGain = realGain;
	pExpResult->sensorAgain[1] =
		MAX(pOx03f10Ctx->aGainLCG.min,
				MIN(realAGain, pOx03f10Ctx->aGainLCG.max));
	realDGain = realGain /  pExpResult->sensorAgain[1];
	pExpResult->sensorDgain[1] =
		MAX(pOx03f10Ctx->dGain.min,
				MIN(realDGain, pOx03f10Ctx->dGain.max));

	/* calculate spd line & gain */
	pExpResult->sensorExpTime[2] =
		MIN(pOx03f10Ctx->maxSPDIntegrationLine * pOx03f10Ctx->oneLineExpTime,
				MAX(pOx03f10Ctx->minSPDIntegrationLine * pOx03f10Ctx->oneLineExpTime,
					exp));
	//Exp same as HCG/LCG
	realGain = total_gain * 4.25;
	realAGain = realGain;
	pExpResult->sensorAgain[2] =
		MAX(pOx03f10Ctx->aGainSPD.min,
				MIN(realAGain, pOx03f10Ctx->aGainSPD.max));
	realDGain = realGain /  pExpResult->sensorAgain[2];
	pExpResult->sensorDgain[2] =
		MAX(pOx03f10Ctx->dGain.min,
				MIN(realDGain, pOx03f10Ctx->dGain.max));

	/* calculate vs line & gain*/
	pExpResult->sensorExpTime[3] =
		MIN(pOx03f10Ctx->maxVSIntegrationLine * pOx03f10Ctx->oneLineExpTime,
				MAX(pOx03f10Ctx->minVSIntegrationLine * pOx03f10Ctx->oneLineExpTime,
					exp_vs));
	/*
	 * Max(DCG_exp + VS_exp, SPD_exp) < VTS - 12 =
	 * pOx03f10Ctx->curFrameLengthLines - 12
	 */
	if (pExpResult->sensorExpTime[3] >=
			(pOx03f10Ctx->curFrameLengthLines - 12) *
			pOx03f10Ctx->oneLineExpTime - pExpParam->sensorExpTime[0])
		pExpResult->sensorExpTime[3] =
			(pOx03f10Ctx->curFrameLengthLines - 12) *
			pOx03f10Ctx->oneLineExpTime -
			pExpParam->sensorExpTime[0];

	realGain = total_gain;
	realAGain = realGain;
	pExpResult->sensorAgain[3] =
		MAX(pOx03f10Ctx->aGainVS.min,
				MIN(realAGain, pOx03f10Ctx->aGainVS.max));
	realDGain = realAGain / pExpResult->sensorAgain[3];
	pExpResult->sensorDgain[3] =
		MAX(pOx03f10Ctx->dGain.min,
				MIN(realDGain, pOx03f10Ctx->dGain.max));

	pExpResult->ispGain = 1.0f;

	TRACE(Ox03f10_INFO,
			"%s: output(HCG) expTime: %02f, AGain: %02f, DGain: %02f.\n",
			__func__, pExpResult->sensorExpTime[0],
			pExpResult->sensorAgain[0], pExpResult->sensorDgain[0]);
	TRACE(Ox03f10_INFO,
			"%s: output(LCG) expTime: %02f, AGain: %02f, DGain: %02f.\n",
			__func__, pExpResult->sensorExpTime[1],
			pExpResult->sensorAgain[1], pExpResult->sensorDgain[1]);
	TRACE(Ox03f10_INFO,
			"%s: output(SPD) expTime: %02f, AGain: %02f, DGain: %02f.\n",
			__func__, pExpResult->sensorExpTime[2],
			pExpResult->sensorAgain[2], pExpResult->sensorDgain[2]);
	TRACE(Ox03f10_INFO,
			"%s: output(VS) expTime: %02f, AGain: %02f, DGain: %02f.\n",
			__func__, pExpResult->sensorExpTime[3],
			pExpResult->sensorAgain[3], pExpResult->sensorDgain[3]);

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiSetAGainIss
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
RESULT Ox03f10_IsiSetAGainIss(IsiSensorHandle_t handle,
		IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pSensorAGain == NULL) {
		TRACE(Ox03f10_ERROR,
			"%s: Invalid sensor handle or SensorAGain pointer (NULL)\n",
			__func__);
		return RET_NULL_POINTER;
	}

	TRACE(Ox03f10_INFO,
		"%s: input again: %02f, %02f, %02f, %02f\n",
		__func__, pSensorAGain->gain[0], pSensorAGain->gain[1],
		pSensorAGain->gain[2], pSensorAGain->gain[3]);

	float32_t realAGain = 0;
	uint32_t again = 0;

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[2U];

	//set HCG analog gain
	realAGain = pSensorAGain->gain[0];
	realAGain = MAX(pOx03f10Ctx->aGainHCG.min,
			MIN(realAGain, pOx03f10Ctx->aGainHCG.max));
	again = (uint32_t)(realAGain * 16);
	group_values[0] = (again & 0xf0) >> 4;
	group_values[1] = (again & 0x0f) << 4;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - HCG: 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1]);
#endif
	result = Ox03f10_IsiWriteRegGroupIss(handle, 0x3508, group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back HCG values
	uint32_t read_value = 0;
	uint8_t read_values[2] = {0};

	result |= Ox03f10_IsiReadRegIss(handle, 0x3508, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3509, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - HCG: 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1]);
#endif
	pOx03f10Ctx->curAgain.gain[0] = (float32_t)again/16.0f;

	//set LCG analog gain
	realAGain = pSensorAGain->gain[1];
	realAGain = MAX(pOx03f10Ctx->aGainLCG.min,
			MIN(realAGain, pOx03f10Ctx->aGainLCG.max));
	again = (uint32_t)(realAGain * 16);
	group_values[0] = (again & 0xf0) >> 4;
	group_values[1] = (again & 0x0f) << 4;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - LCG: 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x3588, group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back LCG values
	result |= Ox03f10_IsiReadRegIss(handle, 0x3588, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3589, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - LCG: 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1]);
#endif

	pOx03f10Ctx->curAgain.gain[1] = (float32_t)again/16.0f;

	//set SPD analog gain
	realAGain = pSensorAGain->gain[2];
	realAGain = MAX(pOx03f10Ctx->aGainSPD.min,
			MIN(realAGain, pOx03f10Ctx->aGainSPD.max));
	again = (uint32_t)(realAGain * 16);
	group_values[0] = (again & 0xf0) >> 4;
	group_values[1] = (again & 0x0f) << 4;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - SPD: 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x3548, group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back SPD values
	result |= Ox03f10_IsiReadRegIss(handle, 0x3548, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3549, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - SPD: 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1]);
#endif

	pOx03f10Ctx->curAgain.gain[2] = (float32_t)again/16.0f;

	//set VS analog gain
	realAGain = pSensorAGain->gain[3];
	realAGain = MAX(pOx03f10Ctx->aGainVS.min,
			MIN(realAGain, pOx03f10Ctx->aGainVS.max));
	again = (uint32_t)(realAGain * 16);
	group_values[0] = (again & 0xf0) >> 4;
	group_values[1] = (again & 0x0f) << 4;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - VS: 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x35c8, group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back VS values
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c8, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c9, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - VS: 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1]);
#endif
	pOx03f10Ctx->curAgain.gain[3] = (float32_t)again/16.0f;
#else
	//set HCG analog gain
	realAGain = pSensorAGain->gain[0];
	realAGain = MAX(pOx03f10Ctx->aGainHCG.min,
			MIN(realAGain, pOx03f10Ctx->aGainHCG.max));
	again = (uint32_t)(realAGain * 16);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3508, (again & 0xf0) >> 4);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3509, (again & 0x0f) << 4);
	pOx03f10Ctx->curAgain.gain[0] = (float32_t)again/16.0f;

	//set LCG analog gain
	realAGain = pSensorAGain->gain[1];
	realAGain = MAX(pOx03f10Ctx->aGainLCG.min,
			MIN(realAGain, pOx03f10Ctx->aGainLCG.max));
	again = (uint32_t)(realAGain * 16);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3588, (again & 0xf0) >> 4);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3589, (again & 0x0f) << 4);
	pOx03f10Ctx->curAgain.gain[1] = (float32_t)again/16.0f;

	//set SPD analog gain
	realAGain = pSensorAGain->gain[2];
	realAGain = MAX(pOx03f10Ctx->aGainSPD.min,
			MIN(realAGain, pOx03f10Ctx->aGainSPD.max));
	again = (uint32_t)(realAGain * 16);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3548, (again & 0xf0) >> 4);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3549, (again & 0x0f) << 4);
	pOx03f10Ctx->curAgain.gain[2] = (float32_t)again/16.0f;

	//set VS analog gain
	realAGain = pSensorAGain->gain[3];
	realAGain = MAX(pOx03f10Ctx->aGainVS.min,
			MIN(realAGain, pOx03f10Ctx->aGainVS.max));
	again = (uint32_t)(realAGain * 16);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x35c8, (again & 0xf0) >> 4);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x35c9, (again & 0x0f) << 4);
	pOx03f10Ctx->curAgain.gain[3] = (float32_t)again/16.0f;
#endif

	TRACE(Ox03f10_INFO,
	      "%s: frame=%d aGain=[%.3f,%.3f,%.3f,%.3f]\n",
	      __func__, g_Sensor_frame_count,
	      pOx03f10Ctx->curAgain.gain[0],
	      pOx03f10Ctx->curAgain.gain[1],
	      pOx03f10Ctx->curAgain.gain[2],
	      pOx03f10Ctx->curAgain.gain[3]);

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiSetDGainIss
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
RESULT Ox03f10_IsiSetDGainIss(IsiSensorHandle_t handle,
		IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);
	uint32_t dgain = 0;
	float32_t realDGain = 0;

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pSensorDGain == NULL) {
		TRACE(Ox03f10_ERROR,
			"%s: Invalid sensor handle or SensorDGain pointer (NULL)\n",
			__func__);
		return RET_NULL_POINTER;
	}
#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[3U];

	//set HCG digital gain
	realDGain = pSensorDGain->gain[0];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	group_values[0] = (dgain >> 10) & 0x0f;
	group_values[1] = (dgain >> 2) & 0xff;
	group_values[2] = (dgain & 0x03) << 6;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - HCG: 0x%02X 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1],
		group_values[2]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x350a, group_values, 3U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back HCG values
	uint32_t read_value = 0;
	uint8_t read_values[3] = {0};

	result |= Ox03f10_IsiReadRegIss(handle, 0x350a, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x350b, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x350c, &read_value);
	read_values[2] = read_value & 0xFF;

	TRACE(Ox03f10_INFO, "%s: After write - HCG: 0x%02X 0x%02X 0x%02X\n",
		__func__, read_values[0], read_values[1],
		read_values[2]);
#endif

	pOx03f10Ctx->curDgain.gain[0] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[0] =
		pOx03f10Ctx->curAgain.gain[0] * pOx03f10Ctx->curDgain.gain[0];

	//set LCG digital gain
	realDGain = pSensorDGain->gain[1];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	group_values[0] = (dgain >> 10) & 0x0f;
	group_values[1] = (dgain >> 2) & 0xff;
	group_values[2] = (dgain & 0x03) << 6;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - LCG: 0x%02X 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1],
		group_values[2]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x358a, group_values, 3U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back LCG values
	result |= Ox03f10_IsiReadRegIss(handle, 0x358a, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x358b, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x358c, &read_value);
	read_values[2] = read_value & 0xFF;

	TRACE(Ox03f10_INFO, "%s: After write - LCG: 0x%02X 0x%02X 0x%02X\n",
		__func__, read_values[0], read_values[1],
		read_values[2]);
#endif

	pOx03f10Ctx->curDgain.gain[1] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[1] =
		pOx03f10Ctx->curAgain.gain[1] * pOx03f10Ctx->curDgain.gain[1];

	//set SPD digital gain
	realDGain = pSensorDGain->gain[2];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	group_values[0] = (dgain >> 10) & 0x0f;
	group_values[1] = (dgain >> 2) & 0xff;
	group_values[2] = (dgain & 0x03) << 6;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - SPD: 0x%02X 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1],
		group_values[2]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x354a, group_values, 3U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back SPD values
	result |= Ox03f10_IsiReadRegIss(handle, 0x354a, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x354b, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x354c, &read_value);
	read_values[2] = read_value & 0xFF;

	TRACE(Ox03f10_INFO, "%s: After write - SPD: 0x%02X 0x%02X 0x%02X\n",
		__func__, read_values[0], read_values[1],
		read_values[2]);
#endif

	pOx03f10Ctx->curDgain.gain[2] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[2] =
		pOx03f10Ctx->curAgain.gain[2] * pOx03f10Ctx->curDgain.gain[2];

	//set VS digital gain
	realDGain = pSensorDGain->gain[3];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	group_values[0] = (dgain >> 10) & 0x0f;
	group_values[1] = (dgain >> 2) & 0xff;
	group_values[2] = (dgain & 0x03) << 6;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - VS: 0x%02X 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1],
		group_values[2]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x35ca, group_values, 3U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back VS values
	result |= Ox03f10_IsiReadRegIss(handle, 0x35ca, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35cb, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35cc, &read_value);
	read_values[2] = read_value & 0xFF;

	TRACE(Ox03f10_INFO, "%s: After write - VS: 0x%02X 0x%02X 0x%02X\n",
		__func__, read_values[0], read_values[1],
		read_values[2]);
#endif
	pOx03f10Ctx->curDgain.gain[3] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[3] =
		pOx03f10Ctx->curAgain.gain[3] * pOx03f10Ctx->curDgain.gain[3];
#else
	//set HCG digital gain
	realDGain = pSensorDGain->gain[0];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x350a, (dgain >> 10) & 0x0f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x350b, (dgain >> 2) & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x350c, (dgain & 0x03) << 6);
	pOx03f10Ctx->curDgain.gain[0] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[0] =
		pOx03f10Ctx->curAgain.gain[0] * pOx03f10Ctx->curDgain.gain[0];

	//set LCG digital gain
	realDGain = pSensorDGain->gain[1];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x358a, (dgain >> 10) & 0x0f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x358b, (dgain >> 2) & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x358c, (dgain & 0x03) << 6);
	pOx03f10Ctx->curDgain.gain[1] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[1] =
		pOx03f10Ctx->curAgain.gain[1] * pOx03f10Ctx->curDgain.gain[1];

	//set SPD digital gain
	realDGain = pSensorDGain->gain[2];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x354a, (dgain >> 10) & 0x0f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x354b, (dgain >> 2) & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x354c, (dgain & 0x03) << 6);
	pOx03f10Ctx->curDgain.gain[2] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[2] =
		pOx03f10Ctx->curAgain.gain[2] * pOx03f10Ctx->curDgain.gain[2];

	//set VS digital gain
	realDGain = pSensorDGain->gain[3];
	realDGain = MAX(pOx03f10Ctx->dGain.min,
			MIN(realDGain, pOx03f10Ctx->dGain.max));
	dgain = (uint32_t)(realDGain * 1024);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x35ca, (dgain >> 10) & 0x0f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x35cb, (dgain >> 2) & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x35cc, (dgain & 0x03) << 6);
	pOx03f10Ctx->curDgain.gain[3] = (float32_t)dgain/1024.0f;
	pOx03f10Ctx->aecCurGain.gain[3] =
		pOx03f10Ctx->curAgain.gain[3] * pOx03f10Ctx->curDgain.gain[3];
#endif

	TRACE(Ox03f10_INFO,
	      "%s: frame=%d dGain=[%.3f,%.3f,%.3f,%.3f] "
	      "totalGain=[%.3f,%.3f,%.3f,%.3f]\n",
	      __func__, g_Sensor_frame_count,
	      pOx03f10Ctx->curDgain.gain[0],
	      pOx03f10Ctx->curDgain.gain[1],
	      pOx03f10Ctx->curDgain.gain[2],
	      pOx03f10Ctx->curDgain.gain[3],
	      pOx03f10Ctx->aecCurGain.gain[0],
	      pOx03f10Ctx->aecCurGain.gain[1],
	      pOx03f10Ctx->aecCurGain.gain[2],
	      pOx03f10Ctx->aecCurGain.gain[3]);

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetAGainIss
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
RESULT Ox03f10_IsiGetAGainIss(IsiSensorHandle_t handle,
		IsiSensorGain_t *pSensorAGain)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	if (pOx03f10Ctx == NULL || pSensorAGain == NULL) {
		TRACE(Ox03f10_ERROR,
				"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_NULL_POINTER;
	}

	memcpy(pSensorAGain, &pOx03f10Ctx->curAgain, sizeof(IsiSensorGain_t));

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetDGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorDGain             pointer to sensor dgain to get
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT Ox03f10_IsiGetDGainIss(IsiSensorHandle_t handle,
		IsiSensorGain_t *pSensorDGain)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	if (pOx03f10Ctx == NULL || pSensorDGain == NULL) {
		TRACE(Ox03f10_ERROR,
				"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_NULL_POINTER;
	}

	memcpy(pSensorDGain, &pOx03f10Ctx->curDgain, sizeof(IsiSensorGain_t));

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiSetIntTimeIss
 *
 * @brief   Writes integration time values to the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorIntTime           pointer to sensor integration time to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT Ox03f10_IsiSetIntTimeIss(IsiSensorHandle_t handle,
		const IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (!pOx03f10Ctx || !pSensorIntTime) {
		TRACE(Ox03f10_ERROR,
				"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_NULL_POINTER;
	}

	TRACE(Ox03f10_INFO,
		"%s: input integration time: %02f, %02f, %02f, %02f\n",
		__func__, pSensorIntTime->intTime[0],
		pSensorIntTime->intTime[1],
		pSensorIntTime->intTime[2],
		pSensorIntTime->intTime[3]);

	uint32_t exp_line = 0;

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[2U];

	//set DCG(HCG & LCG) exp
	exp_line = pSensorIntTime->intTime[0] / pOx03f10Ctx->oneLineExpTime;
	exp_line = MIN(pOx03f10Ctx->maxDCGIntegrationLine,
		MAX(pOx03f10Ctx->minDCGIntegrationLine, exp_line));
	TRACE(Ox03f10_DEBUG,
		"%s: set DCG exp_line = 0x%04x\n",
		__func__, exp_line);
	group_values[0] = (exp_line >> 8) & 0xff;
	group_values[1] = (exp_line & 0xff);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - DCG: 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1]);
#endif
	result |=  Ox03f10_IsiWriteRegGroupIss(handle, 0x3501, group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back DCG values
	uint32_t read_value = 0;
	uint8_t read_values[2] = {0};

	Ox03f10_IsiReadRegIss(handle, 0x3501, &read_value);
	read_values[0] = read_value & 0xFF;
	Ox03f10_IsiReadRegIss(handle, 0x3502, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(Ox03f10_INFO, "%s: After write - DCG: 0x%02X 0x%02X\n",
		__func__, read_values[0], read_values[1]);
#endif
	pOx03f10Ctx->aecCurIntTime.intTime[0] =
		exp_line * pOx03f10Ctx->oneLineExpTime;
	pOx03f10Ctx->aecCurIntTime.intTime[1] =
		exp_line * pOx03f10Ctx->oneLineExpTime;

	//set SPD exp
	exp_line = pSensorIntTime->intTime[2] / pOx03f10Ctx->oneLineExpTime;
	exp_line = MIN(pOx03f10Ctx->maxSPDIntegrationLine,
		MAX(pOx03f10Ctx->minSPDIntegrationLine, exp_line));
	TRACE(Ox03f10_DEBUG,
		"%s: set SPD exp_line = 0x%04x\n",
		__func__, exp_line);
	group_values[0] = (exp_line >> 8) & 0xff;
	group_values[1] = (exp_line & 0xff);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - SPD: 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1]);
#endif
	result |=  Ox03f10_IsiWriteRegGroupIss(handle, 0x3541, group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back SPD values
	result |= Ox03f10_IsiReadRegIss(handle, 0x3541, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x3542, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(Ox03f10_INFO, "%s: After write - SPD: 0x%02X 0x%02X\n",
		__func__, read_values[0], read_values[1]);
#endif
	pOx03f10Ctx->aecCurIntTime.intTime[2] =
		exp_line * pOx03f10Ctx->oneLineExpTime;

	//set VS exp
	exp_line = pSensorIntTime->intTime[3] /
		pOx03f10Ctx->oneLineExpTime - 0.5;
	exp_line = MIN(pOx03f10Ctx->maxVSIntegrationLine,
		MAX(pOx03f10Ctx->minVSIntegrationLine, exp_line));
	TRACE(Ox03f10_DEBUG,
		"%s: set VS exp_line = 0x%04x\n",
		__func__, exp_line);
	group_values[0] = (exp_line >> 8) & 0xff;
	group_values[1] = (exp_line & 0xff);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO, "%s: Before write - VS: 0x%02X 0x%02X\n",
		__func__, group_values[0], group_values[1]);
#endif
	result |=  Ox03f10_IsiWriteRegGroupIss(handle, 0x35c1, group_values, 2U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back VS values
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c1, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c2, &read_value);
	read_values[1] = read_value & 0xFF;

	TRACE(Ox03f10_INFO, "%s: After write - VS: 0x%02X 0x%02X\n",
		__func__, read_values[0], read_values[1]);
#endif
	pOx03f10Ctx->aecCurIntTime.intTime[3] =
		exp_line * pOx03f10Ctx->oneLineExpTime;
	pOx03f10Ctx->maxDCGIntegrationLine =
		pOx03f10Ctx->curFrameLengthLines - 13 - exp_line;
#else
	//set DCG(HCG & LCG) exp
	exp_line = pSensorIntTime->intTime[0] / pOx03f10Ctx->oneLineExpTime;
	exp_line = MIN(pOx03f10Ctx->maxDCGIntegrationLine,
		MAX(pOx03f10Ctx->minDCGIntegrationLine, exp_line));
	TRACE(Ox03f10_DEBUG,
		"%s: set DCG exp_line = 0x%04x\n",
		__func__, exp_line);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3501,
		(exp_line >> 8) & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3502,
		(exp_line & 0xff));
	pOx03f10Ctx->aecCurIntTime.intTime[0] =
		exp_line * pOx03f10Ctx->oneLineExpTime;
	pOx03f10Ctx->aecCurIntTime.intTime[1] =
		exp_line * pOx03f10Ctx->oneLineExpTime;

	//set SPD exp
	exp_line = pSensorIntTime->intTime[2] / pOx03f10Ctx->oneLineExpTime;
	exp_line = MIN(pOx03f10Ctx->maxSPDIntegrationLine,
		MAX(pOx03f10Ctx->minSPDIntegrationLine, exp_line));
	TRACE(Ox03f10_DEBUG,
		"%s: set SPD exp_line = 0x%04x\n",
		__func__, exp_line);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3541,
		(exp_line >> 8) & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x3542,
		(exp_line & 0xff));
	pOx03f10Ctx->aecCurIntTime.intTime[2] =
		exp_line * pOx03f10Ctx->oneLineExpTime;

	//set VS exp
	exp_line = pSensorIntTime->intTime[3] /
		pOx03f10Ctx->oneLineExpTime - 0.5;
	exp_line = MIN(pOx03f10Ctx->maxVSIntegrationLine,
		MAX(pOx03f10Ctx->minVSIntegrationLine, exp_line));
	TRACE(Ox03f10_DEBUG,
		"%s: set VS exp_line = 0x%04x\n",
		__func__, exp_line);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x35c1,
		(exp_line >> 8) & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x35c2,
		(exp_line & 0xff));
	pOx03f10Ctx->aecCurIntTime.intTime[3] =
		exp_line * pOx03f10Ctx->oneLineExpTime;
	pOx03f10Ctx->maxDCGIntegrationLine =
		pOx03f10Ctx->curFrameLengthLines - 13 - exp_line;
#endif

	TRACE(Ox03f10_INFO,
	      "%s: frame=%d intTime=[%.6f,%.6f,%.6f,%.6f]\n",
	      __func__, g_Sensor_frame_count,
	      pOx03f10Ctx->aecCurIntTime.intTime[0],
	      pOx03f10Ctx->aecCurIntTime.intTime[1],
	      pOx03f10Ctx->aecCurIntTime.intTime[2],
	      pOx03f10Ctx->aecCurIntTime.intTime[3]);

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}


/*****************************************************************************
 *          Ox03f10_IsiGetIntTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                   sensor instance handle
 * @param   pSensorIntTime           pointer to integration time to get
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT Ox03f10_IsiGetIntTimeIss(IsiSensorHandle_t handle,
		IsiSensorIntTime_t *pSensorIntTime)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	if (!pOx03f10Ctx || !pSensorIntTime) {
		TRACE(Ox03f10_ERROR,
				"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_NULL_POINTER;
	}

	memcpy(pSensorIntTime, &pOx03f10Ctx->aecCurIntTime,
	       sizeof(IsiSensorIntTime_t));

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          sensor_framecount
 *
 * @brief   Reads and processes the sensor frame counter.
 *
 * @param   handle                  sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 *****************************************************************************/
int sensor_framecount(IsiSensorHandle_t handle)
{
	u32 frame_counter;
	uint16_t read_buf[4];
	int Status;

	Status = Ox03f10_IsiReadRegIss(handle, 0x4623, &read_buf[0]);
	if (Status !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR,
				"%s:%d Failed to read register (err=%d)\n",
				__func__, __LINE__, Status);
		return Status;
	}
	Status = Ox03f10_IsiReadRegIss(handle, 0x4622, &read_buf[1]);
	if (Status !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR,
				"%s:%d Failed to read register (err=%d)\n",
				__func__, __LINE__, Status);
		return Status;
	}
	Status = Ox03f10_IsiReadRegIss(handle, 0x4621, &read_buf[2]);
	if (Status !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR,
				"%s:%d Failed to read register (err=%d)\n",
				__func__, __LINE__, Status);
		return Status;
	}
	Status = Ox03f10_IsiReadRegIss(handle, 0x4620, &read_buf[3]);
	if (Status !=  RET_SUCCESS) {
		TRACE(Ox03f10_ERROR,
				"%s:%d Failed to read register (err=%d)\n",
				__func__, __LINE__, Status);
		return Status;
	}
	frame_counter = (read_buf[3] << 24) |
		(read_buf[2] << 16) |
		(read_buf[1] << 8) |
		read_buf[0];

	g_Sensor_frame_count = frame_counter;

	return Status;
}

/*****************************************************************************
 *          Ox03f10_IsiGetFpsIss
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
RESULT Ox03f10_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR,
				"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_NULL_POINTER;
	}

	*pFps = pOx03f10Ctx->currFps;

	sensor_framecount(handle);
	Fmc_Sensor_Statustask();

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiSetFpsIss
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
RESULT Ox03f10_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
	RESULT result = RET_SUCCESS;
	int32_t NewVts = 0;
	int32_t vs_exp = 0;
	uint16_t data = 0;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR,
				"%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_NULL_POINTER;
	}
	if (fps > pOx03f10Ctx->maxFps) {
		TRACE(Ox03f10_ERROR,
				"%s: fps(%d) out of range, correct to %d (%d, %d)\n",
				__func__, fps, pOx03f10Ctx->maxFps, pOx03f10Ctx->minFps,
				pOx03f10Ctx->maxFps);
		fps = pOx03f10Ctx->maxFps;
	}
	if (fps < pOx03f10Ctx->minFps) {
		TRACE(Ox03f10_ERROR,
				"%s: set fps(%d) out of range, correct to %d (%d, %d)\n",
				__func__, fps, pOx03f10Ctx->minFps,
				pOx03f10Ctx->minFps, pOx03f10Ctx->maxFps);
		fps = pOx03f10Ctx->minFps;
	}

	NewVts = pOx03f10Ctx->frameLengthLines*pOx03f10Ctx->sensorMode.fps / fps;
	result  = Ox03f10_IsiWriteRegIss(handle, 0x380e, NewVts >> 8);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x380f, NewVts & 0xff);
	pOx03f10Ctx->currFps              = fps;
	pOx03f10Ctx->curFrameLengthLines  = NewVts;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c1, &data);
	vs_exp = (data & 0xff)  << 8;
	result |= Ox03f10_IsiReadRegIss(handle, 0x35c2, &data);
	vs_exp |= (data & 0xff);
	//		pOx03f10Ctx->maxDCGIntegrationLine =
	//			pOx03f10Ctx->curFrameLengthLines - 13 - vs_exp;
	//		pOx03f10Ctx->maxSPDIntegrationLine =
	//			pOx03f10Ctx->curFrameLengthLines - 13;
	//		pOx03f10Ctx->aecMaxIntegrationTime =
	//		pOx03f10Ctx->maxDCGIntegrationLine *
	//							pOx03f10Ctx->oneLineDCGExpTime;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetIspStatusIss
 *
 * @brief   Get sensor isp status.
 *
 * @param   handle                    sensor instance handle
 * @param   pSensorIspStatus          sensor isp status
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT Ox03f10_IsiGetIspStatusIss(IsiSensorHandle_t handle,
		IsiIspStatus_t *pIspStatus)
{
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}
	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	pIspStatus->useSensorAE  = false;
	pIspStatus->useSensorBLC = true;
	pIspStatus->useAWBMode = ISI_USE_ISP_AND_SENSOR_WB_GAIN;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/*****************************************************************************
 *          Ox03f10_IsiSetTpgIss
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
RESULT Ox03f10_IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pOx03f10Ctx->configured !=  BOOL_TRUE)
		return RET_WRONG_STATE;

	if (tpg.enable == 0) {
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5004, 0x1e);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5005, 0x1e);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5006, 0x1e);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5007, 0x1e);

		result |= Ox03f10_IsiWriteRegIss(handle, 0x5240, 0x00);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5440, 0x00);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5640, 0x00);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5840, 0x00);
	} else {
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5004, 0x1f);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5005, 0x1f);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5006, 0x1f);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5007, 0x1f);

		result |= Ox03f10_IsiWriteRegIss(handle, 0x5240, 0x01);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5440, 0x01);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5640, 0x01);
		result |= Ox03f10_IsiWriteRegIss(handle, 0x5840, 0x01);
	}

	pOx03f10Ctx->testPattern = tpg.enable;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetTpgIss
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
RESULT Ox03f10_IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *pTpg)
{
	RESULT result = RET_SUCCESS;
	uint16_t hcgValue = 0, lcgValue = 0, spdValue = 0, vsValue = 0;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL  || pTpg == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pOx03f10Ctx->configured !=  BOOL_TRUE)
		return RET_WRONG_STATE;

	if (!Ox03f10_IsiReadRegIss(handle, 0x5240, &hcgValue)
			&& !Ox03f10_IsiReadRegIss(handle, 0x5440, &lcgValue)
			&& !Ox03f10_IsiReadRegIss(handle, 0x5640, &spdValue)
			&& !Ox03f10_IsiReadRegIss(handle, 0x5840, &vsValue)) {
		pTpg->enable = (((hcgValue & 0x01) !=  0) && ((lcgValue & 0x01) !=  0)
				&& ((spdValue & 0x01) !=  0)
				&& ((vsValue & 0x01) !=  0)) ? 1 : 0;
		if (pTpg->enable)
			pTpg->pattern = (0xff & hcgValue);
		pOx03f10Ctx->testPattern = pTpg->enable;
	}

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

static void Ox03f10_IsiGetLPDSPDWbGain
(
	const IsiSensorWb_t *pSensorFillWBGainData,
	IsiSensorWb_t *pLpd,
	IsiSensorWb_t *pSpd
)
{
	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);
	float32_t awb_slope, factor, awb_distance;
	float32_t awb_d65_ratio[2] = {0.0};
	float32_t awb_cwf_ratio[2] = {0.0};
	float32_t awb_a_ratio[2] = {0.0};
	float32_t awb_cur_light[2] = {0.0};
	uint32_t awb_spd_b_gain, awb_spd_r_gain, awb_spd_g_gain;
	uint32_t awb_lpd_r_gain, awb_lpd_b_gain, awb_lpd_g_gain;

	/*
	 * AMD X3F module pre-AWB calibration parameters
	 * ------------------        LPD        -----------------
	 * CT     G/B（Bgain） G/R (Rgain)     CT(Rgain/Bgain)
	 * 6500   0.867152     1.144517        1.319857
	 * 4150   1.446735     1.214702        0.839616
	 * 2800   1.695229     0.639525        0.377250
	 *
	 * ------------------        SPD        -----------------
	 * CT     G/B（Bgain） G/R (Rgain)
	 * 6500   0.897887     1.078680
	 * 4150   1.493608     1.168333
	 * 2800   1.333959     0.659302
	 *
	 * ------------------     SPD / LPD     -----------------
	 * CT      G/B         G/R       nCT
	 * 6500 1.035444    0.942476     151
	 * 4150 1.032399    0.961827      D6
	 * 2800 0.786891    1.030924      60
	 */
	awb_d65_ratio[0] = 1.035444;
	awb_d65_ratio[1] = 0.942476;
	awb_cwf_ratio[0] = 1.032399;
	awb_cwf_ratio[1] = 0.961827;
	awb_a_ratio[0] = 0.786891;
	awb_a_ratio[1] = 1.030924;
	TRACE(Ox03f10_INFO,
			"awb_d65_ratio = [%f, %f], awb_cwf_ratio ="
			"[%f, %f], awb_a_ratio = [%f, %f]\n",
			awb_d65_ratio[0], awb_d65_ratio[1],
			awb_cwf_ratio[0], awb_cwf_ratio[1],
			awb_a_ratio[0], awb_a_ratio[1]);

	if (pSensorFillWBGainData == NULL)
		return;

	awb_lpd_b_gain = (uint32_t)(pSensorFillWBGainData->bGain * 1024.0);
	awb_lpd_g_gain = (uint32_t)(pSensorFillWBGainData->grGain * 1024.0);
	awb_spd_g_gain = (uint32_t)(pSensorFillWBGainData->gbGain * 1024.0);
	awb_lpd_r_gain = (uint32_t)(pSensorFillWBGainData->rGain * 1024.0);
	awb_spd_r_gain = 1024;
	awb_spd_b_gain = 1024;

	/* current_color_temper = ispAWB_rGain / ispAWB_bGain */
	float32_t color_temper =
		pSensorFillWBGainData->rGain /
		pSensorFillWBGainData->bGain;

	TRACE(Ox03f10_INFO,
		"color_temper %f = %f / %f\n",
		color_temper,
		pSensorFillWBGainData->rGain,
		pSensorFillWBGainData->bGain);

	/*
	 * The color temperature here can be represented using the CCT value.
	 * Therefore, the BR ratio is used as a substitute.
	 * ------------------        LPD        -----------------
	 * CT     G/B（Bgain） G/R (Rgain)     TEMPER(Rgain/Bgain)
	 * D65     6500   0.867152     1.144517        1.319857
	 * CWF     4150   1.446735     1.214702        0.839616
	 * A       2800   1.695229     0.639525        0.377250
	 */
	float32_t D65_TEMPER = 1.319857;
	float32_t CWF_TEMPER = 0.839616;
	float32_t A_TEMPER = 0.377250;

	TRACE(Ox03f10_INFO,
		"D65_TEMPER = %f, CWF_TEMPER = %f, A_TEMPER = %f\n",
		D65_TEMPER, CWF_TEMPER, A_TEMPER);

	if (color_temper >= D65_TEMPER) {
		awb_spd_b_gain = awb_lpd_b_gain * awb_d65_ratio[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_d65_ratio[1];
	} else if (color_temper < D65_TEMPER && color_temper > CWF_TEMPER) {
		for (uint8_t channel_idx = 0; channel_idx < 2; channel_idx++) {
			if (awb_d65_ratio[channel_idx] - awb_cwf_ratio[channel_idx] >= 0) {
				awb_slope =
					(float32_t)(D65_TEMPER - CWF_TEMPER) /
					(awb_d65_ratio[channel_idx] - awb_cwf_ratio[channel_idx]);
				awb_distance = color_temper - CWF_TEMPER;
				awb_cur_light[channel_idx] = awb_cwf_ratio[channel_idx] +
					awb_distance / awb_slope;
			} else {
				awb_slope =
					(float32_t)(D65_TEMPER - CWF_TEMPER) /
					(awb_cwf_ratio[channel_idx] - awb_d65_ratio[channel_idx]);
				awb_distance = color_temper - CWF_TEMPER;
				awb_cur_light[channel_idx] = awb_cwf_ratio[channel_idx] -
					awb_distance / awb_slope;
			}
		}
		awb_spd_b_gain = awb_lpd_b_gain * awb_cur_light[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_cur_light[1];
	} else if (color_temper == CWF_TEMPER) {
		awb_spd_b_gain = awb_lpd_b_gain * awb_cwf_ratio[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_cwf_ratio[1];
	} else if (color_temper < CWF_TEMPER && color_temper > A_TEMPER) {
		for (uint8_t channel_idx = 0; channel_idx < 2; channel_idx++) {
			if (awb_cwf_ratio[channel_idx] - awb_a_ratio[channel_idx] >= 0) {
				awb_slope =
					(float32_t)(CWF_TEMPER - A_TEMPER) /
					(awb_cwf_ratio[channel_idx] - awb_a_ratio[channel_idx]);
				awb_distance = color_temper - A_TEMPER;
				awb_cur_light[channel_idx] = awb_a_ratio[channel_idx] +
					awb_distance / awb_slope;
			} else {
				awb_slope =
					(float32_t)(CWF_TEMPER - A_TEMPER) /
					(awb_a_ratio[channel_idx] - awb_cwf_ratio[channel_idx]);
				awb_distance = color_temper - A_TEMPER;
				awb_cur_light[channel_idx] = awb_a_ratio[channel_idx] -
					awb_distance / awb_slope;
			}
		}
		awb_spd_b_gain = awb_lpd_b_gain * awb_cur_light[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_cur_light[1];
	} else if (color_temper <= A_TEMPER) {
		awb_spd_b_gain = awb_lpd_b_gain * awb_a_ratio[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_a_ratio[1];
	}

	/* sensor awb gain归一化 */
	if (awb_lpd_r_gain < 1024) {
		factor = (1024.0 / awb_lpd_r_gain);
		awb_lpd_b_gain = awb_lpd_b_gain * factor;
		awb_lpd_g_gain = awb_lpd_g_gain * factor;
		awb_lpd_r_gain = 1024;
	}
	if (awb_lpd_b_gain < 1024) {
		factor = (1024.0 / awb_lpd_b_gain);
		awb_lpd_r_gain = awb_lpd_r_gain * factor;
		awb_lpd_g_gain = awb_lpd_g_gain * factor;
		awb_lpd_b_gain = 1024;
	}
	if (awb_spd_r_gain < 1024) {
		factor = (1024.0 / awb_spd_r_gain);
		awb_spd_b_gain = awb_spd_b_gain * factor;
		awb_spd_g_gain = awb_spd_g_gain * factor;
		awb_spd_r_gain = 1024;
	}
	if (awb_spd_b_gain < 1024) {
		factor = (1024.0 / awb_spd_b_gain);
		awb_spd_r_gain = awb_spd_r_gain * factor;
		awb_spd_g_gain = awb_spd_g_gain * factor;
		awb_spd_b_gain = 1024;
	}

	pLpd->bGain = awb_lpd_b_gain;
	pLpd->grGain = awb_lpd_g_gain;
	pLpd->gbGain = awb_lpd_g_gain;
	pLpd->rGain = awb_lpd_r_gain;

	pSpd->bGain = awb_spd_b_gain;
	pSpd->grGain = awb_spd_g_gain;
	pSpd->gbGain = awb_spd_g_gain;
	pSpd->rGain = awb_spd_r_gain;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
}

/*****************************************************************************
 *          Ox03f10_IsiSetWBIss
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
static RESULT Ox03f10_IsiSetWBIss(IsiSensorHandle_t handle,
				 const IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	uint32_t bGainLpd, gbGainLpd, grGainLpd, rGainLpd;
	uint32_t bGainSpd, gbGainSpd, grGainSpd, rGainSpd;
	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	if (pWb == NULL) {
		TRACE(Ox03f10_ERROR,
			"%s %d pWb is NULL\n\r",
			__func__, __LINE__);
		return RET_NULL_POINTER;
	}
	TRACE(Ox03f10_DEBUG,
		"%s:set [%f,%f,%f,%f]\n", __func__,
		pWb->rGain, pWb->grGain,
		pWb->gbGain, pWb->bGain);

	IsiSensorWb_t lpdGain, spdGain;

	MEMSET(&lpdGain, 0, sizeof(IsiSensorWb_t));
	MEMSET(&spdGain, 0, sizeof(IsiSensorWb_t));

	Ox03f10_IsiGetLPDSPDWbGain(pWb, &lpdGain, &spdGain);

	bGainLpd = lpdGain.bGain;
	gbGainLpd = lpdGain.gbGain;
	grGainLpd = lpdGain.grGain;
	rGainLpd  = lpdGain.rGain;
	bGainSpd = spdGain.bGain;
	gbGainSpd = spdGain.gbGain;
	grGainSpd = spdGain.grGain;
	rGainSpd  = spdGain.rGain;

#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];

	//set HCG channel awb gain - B and GB
	group_values[0] = (bGainLpd >> 8) & 0x7f;
	group_values[1] = bGainLpd & 0xff;
	group_values[2] = (gbGainLpd >> 8) & 0x7f;
	group_values[3] = gbGainLpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
	      "%s: Before write - HCG B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5280, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back HCG B/GB values
	uint32_t read_value = 0;
	uint8_t read_values[4] = {0};

	result |= Ox03f10_IsiReadRegIss(handle, 0x5280, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5281, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5282, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5283, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - HCG B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	//set HCG channel awb gain - GR and R
	group_values[0] = (grGainLpd >> 8) & 0x7f;
	group_values[1] = grGainLpd & 0xff;
	group_values[2] = (rGainLpd >> 8) & 0x7f;
	group_values[3] = rGainLpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
	      "%s: Before write - HCG GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5284, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back HCG GR/R values
	result |= Ox03f10_IsiReadRegIss(handle, 0x5284, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5285, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5286, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5287, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - HCG GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	//set LCG channel awb gain - B and GB
	group_values[0] = (bGainLpd >> 8) & 0x7f;
	group_values[1] = bGainLpd & 0xff;
	group_values[2] = (gbGainLpd >> 8) & 0x7f;
	group_values[3] = gbGainLpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
	      "%s: Before write - LCG B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5480, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back LCG B/GB values
	result |= Ox03f10_IsiReadRegIss(handle, 0x5480, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5481, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5482, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5483, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - LCG B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	//set LCG channel awb gain - GR and R
	group_values[0] = (grGainLpd >> 8) & 0x7f;
	group_values[1] = grGainLpd & 0xff;
	group_values[2] = (rGainLpd >> 8) & 0x7f;
	group_values[3] = rGainLpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
	      "%s: Before write - LCG GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5484, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back LCG GR/R values
	result |= Ox03f10_IsiReadRegIss(handle, 0x5484, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5485, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5486, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5487, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - LCG GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	//set SPD channel awb gain - B and GB
	group_values[0] = (bGainSpd >> 8) & 0x7f;
	group_values[1] = bGainSpd & 0xff;
	group_values[2] = (gbGainSpd >> 8) & 0x7f;
	group_values[3] = gbGainSpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
	      "%s: Before write - SPD B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5680, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back SPD B/GB values
	result |= Ox03f10_IsiReadRegIss(handle, 0x5680, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5681, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5682, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5683, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - SPD B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	//set SPD channel awb gain - GR and R
	group_values[0] = (grGainSpd >> 8) & 0x7f;
	group_values[1] = grGainSpd & 0xff;
	group_values[2] = (rGainSpd >> 8) & 0x7f;
	group_values[3] = rGainSpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
	      "%s: Before write - SPD GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5684, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back SPD GR/R values
	result |= Ox03f10_IsiReadRegIss(handle, 0x5684, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5685, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5686, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5687, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - SPD GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	//set VS channel awb gain - B and GB
	group_values[0] = (bGainLpd >> 8) & 0x7f;
	group_values[1] = bGainLpd & 0xff;
	group_values[2] = (gbGainLpd >> 8) & 0x7f;
	group_values[3] = gbGainLpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
	      "%s: Before write - VS B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, group_values[0], group_values[1],
	      group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5880, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back VS B/GB values
	result |= Ox03f10_IsiReadRegIss(handle, 0x5880, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5881, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5882, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5883, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
	      "%s: After write - VS B/GB: 0x%02X 0x%02X 0x%02X 0x%02X\n",
	      __func__, read_values[0], read_values[1],
	      read_values[2], read_values[3]);
#endif

	//set VS channel awb gain - GR and R
	group_values[0] = (grGainLpd >> 8) & 0x7f;
	group_values[1] = grGainLpd & 0xff;
	group_values[2] = (rGainLpd >> 8) & 0x7f;
	group_values[3] = rGainLpd & 0xff;
#ifdef ENABLE_I2C_GROUPING_DEBUG
	TRACE(Ox03f10_INFO,
			"%s: Before write - VS GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
			__func__, group_values[0], group_values[1],
			group_values[2], group_values[3]);
#endif
	result |= Ox03f10_IsiWriteRegGroupIss(handle, 0x5884, group_values, 4U);
#ifdef ENABLE_I2C_GROUPING_DEBUG
	// Read back VS GR/R values
	result |= Ox03f10_IsiReadRegIss(handle, 0x5884, &read_value);
	read_values[0] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5885, &read_value);
	read_values[1] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5886, &read_value);
	read_values[2] = read_value & 0xFF;
	result |= Ox03f10_IsiReadRegIss(handle, 0x5887, &read_value);
	read_values[3] = read_value & 0xFF;

	TRACE(Ox03f10_INFO,
			"%s: After write - VS GR/R: 0x%02X 0x%02X 0x%02X 0x%02X\n",
			__func__, read_values[0], read_values[1],
			read_values[2], read_values[3]);
#endif
#else
	//set HCG channel awb gain
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5280, (bGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5281, bGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5282, (gbGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5283, gbGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5284, (grGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5285, grGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5286, (rGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5287, rGainLpd & 0xff);

	//set LCG channel awb gain
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5480, (bGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5481, bGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5482, (gbGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5483, gbGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5484, (grGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5485, grGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5486, (rGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5487, rGainLpd & 0xff);

	//set SPD channel awb gain
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5680, (bGainSpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5681, bGainSpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5682, (gbGainSpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5683, gbGainSpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5684, (grGainSpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5685, grGainSpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5686, (rGainSpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5687, rGainSpd & 0xff);

	//set VS channel awb gain
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5880, (bGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5881, bGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5882, (gbGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5883, gbGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5884, (grGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5885, grGainLpd & 0xff);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5886, (rGainLpd >> 8) & 0x7f);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x5887, rGainLpd & 0xff);
#endif

	memcpy(&pOx03f10Ctx->sensorWb, pWb, sizeof(IsiSensorWb_t));

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetWBIss
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
static RESULT Ox03f10_IsiGetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pWb == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}
	memcpy(pWb, &pOx03f10Ctx->sensorWb, sizeof(IsiSensorWb_t));

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiSetBlcIss
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
static RESULT Ox03f10_IsiSetBlcIss(IsiSensorHandle_t handle,
		const IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;
	uint16_t blcGain = 0;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pBlc == NULL) {
		TRACE(Ox03f10_ERROR,
			"%s %d pOx03f10Ctx or pBlc is NULL\n\r",
			__func__, __LINE__);
		return RET_NULL_POINTER;
	}

	blcGain = pBlc->red;
#ifdef ENABLE_I2C_GROUPING
	uint8_t group_values[4U];

	//set HCG blc
	group_values[0] = (blcGain >> 8) & 0x03;
	group_values[1] = blcGain & 0xff;
	//set LCG blc
	group_values[2] = (blcGain >> 8) & 0x03;
	group_values[3] = blcGain & 0xff;

	result = Ox03f10_IsiWriteRegGroupIss(handle, 0x4026, group_values, 4U);

	//set S blc
	group_values[0] = (blcGain >> 8) & 0x03;
	group_values[1] = blcGain & 0xff;
	//set VS blc
	group_values[2] = (blcGain >> 8) & 0x03;
	group_values[3] = blcGain & 0xff;

	result = Ox03f10_IsiWriteRegGroupIss(handle, 0x402a, group_values, 4U);
#else
	//set HCG blc
	result = Ox03f10_IsiWriteRegIss(handle, 0x4026, (blcGain >> 8) & 0x03);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x4027, blcGain & 0xff);
	//set LCG blc
	result |= Ox03f10_IsiWriteRegIss(handle, 0x4028, (blcGain >> 8) & 0x03);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x4029, blcGain & 0xff);
	//set S blc
	result |= Ox03f10_IsiWriteRegIss(handle, 0x402a, (blcGain >> 8) & 0x03);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x402b, blcGain & 0xff);
	//set VS blc
	result |= Ox03f10_IsiWriteRegIss(handle, 0x402c, (blcGain >> 8) & 0x03);
	result |= Ox03f10_IsiWriteRegIss(handle, 0x402d, blcGain & 0xff);
#endif

	pOx03f10Ctx->sensorBlc = *pBlc;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetBlcIss
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
static RESULT Ox03f10_IsiGetBlcIss(IsiSensorHandle_t handle,
		IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pBlc == NULL) {
		TRACE(Ox03f10_ERROR,
			"%s %d pOx03f10Ctx or pBlc is NULL\n\r",
			__func__, __LINE__);
		return RET_NULL_POINTER;
	}
	*pBlc = pOx03f10Ctx->sensorBlc;

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          Ox03f10_IsiGetExpandCurveIss
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
static RESULT Ox03f10_IsiGetExpandCurveIss(IsiSensorHandle_t handle,
		IsiSensorCompandCurve_t *pCurve)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox03f10_INFO, "%s: (enter)\n", __func__);

	Ox03f10_Context_t *pOx03f10Ctx = (Ox03f10_Context_t *) handle;

	if (pOx03f10Ctx == NULL || pCurve == NULL) {
		TRACE(Ox03f10_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	uint8_t expand_px[64] = {
		22, 20, 12, 20, 20, 20, 20, 19, 19, 19, 19,
		19, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
		18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 12,
	};
	memcpy(pCurve->compandPx, expand_px, sizeof(expand_px));

	pCurve->compandXData[0] = 0;
	pCurve->compandYData[0] = 0;
	for (int curve_idx = 1; curve_idx < 65; curve_idx++) {
		if (pCurve->compandXData[curve_idx - 1] == 0 &&
		    pCurve->compandPx[curve_idx - 1] > 0) {
			pCurve->compandXData[curve_idx] =
				pCurve->compandXData[curve_idx - 1] +
				((1 << pCurve->compandPx[curve_idx - 1]) - 1);
		} else if (pCurve->compandXData[curve_idx - 1] > 0 &&
			   pCurve->compandPx[curve_idx - 1] > 0) {
			pCurve->compandXData[curve_idx] =
				pCurve->compandXData[curve_idx - 1] +
				(1 << pCurve->compandPx[curve_idx - 1]);
		} else if (pCurve->compandXData[curve_idx - 1] > 0 &&
			   pCurve->compandPx[curve_idx - 1] == 0) {
			pCurve->compandXData[curve_idx] = pCurve->compandXData[curve_idx-1];
		} else {
			TRACE(Ox03f10_INFO, "%s: invalid paramter\n", __func__);
			return RET_INVALID_PARM;
		}
	}

	uint16_t expandXValue[34] = {
		0, 1023, 1279, 1279, 1535, 1791, 2047, 2303, 2431,
		2559, 2687, 2815, 2943, 3007, 3071, 3135, 3199,
		3263, 3327, 3391, 3455, 3519, 3583, 3647, 3711,
		3775, 3839, 3903, 3967, 3999, 4031, 4063, 4095,
		4095,
	};
	uint32_t expandYValue[34] = {
		0, 1023, 2047, 2047, 4095, 8191, 12287, 16383, 20479,
		24575, 32767, 40959, 49151, 57343, 65535, 81919,
		98303, 114687, 131071, 163839, 196607, 262143,
		393215, 524287, 786431, 1048575, 1572863, 2097151,
		3145727, 4194303, 8388607, 12582911, 16777215,
		16777215,
	};
	float slope[34] = {0};

	for (int seg_idx = 0; seg_idx < 34; seg_idx++)
		slope[seg_idx] =
			(expandYValue[seg_idx + 1] - expandYValue[seg_idx]) /
			(expandXValue[seg_idx + 1] - expandXValue[seg_idx]);

	for (int curve_idx = 1; curve_idx < 65; curve_idx++) {
		for (int seg_idx = 1; seg_idx < 34; seg_idx++) {
			if (pCurve->compandXData[curve_idx] >= expandXValue[seg_idx-1] &&
					pCurve->compandXData[curve_idx] < expandXValue[seg_idx]) {
				pCurve->compandYData[curve_idx] =
					expandYValue[seg_idx-1] + (pCurve->compandXData[curve_idx]
							- expandXValue[seg_idx-1]) * slope[seg_idx-1];
			}
		}
	}

	TRACE(Ox03f10_INFO, "%s: (exit)\n", __func__);
	return result;
}

/******************************************************************************
 *          Ox03f10_IsiGetSensorIss
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
RESULT Ox03f10_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
	RESULT result = RET_SUCCESS;
	static const char SensorName[16] = "Ox03f10";

	TRACE(Ox03f10_INFO, "%s (enter)\n", __func__);

	if (pIsiSensor != NULL) {
		pIsiSensor->pszName = SensorName;
		pIsiSensor->pIsiCreateIss = Ox03f10_IsiCreateIss;
		pIsiSensor->pIsiOpenIss = Ox03f10_IsiOpenIss;
		pIsiSensor->pIsiCloseIss = Ox03f10_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss = Ox03f10_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss = Ox03f10_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss = Ox03f10_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss = Ox03f10_IsiGetModeIss;
		pIsiSensor->pIsiEnumModeIss = Ox03f10_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss = Ox03f10_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss = Ox03f10_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss = Ox03f10_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss = Ox03f10_IsiSetStreamingIss;
		pIsiSensor->pIsiGetAeBaseInfoIss = Ox03f10_pIsiGetAeBaseInfoIss;
		pIsiSensor->pIsiExcuteExpCtrlIss = Ox03f10_IsiExcuteExpControlIss;
		pIsiSensor->pIsiGetAGainIss = Ox03f10_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss = Ox03f10_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss = Ox03f10_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss = Ox03f10_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss = Ox03f10_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss = Ox03f10_IsiSetIntTimeIss;
		pIsiSensor->pIsiGetFpsIss = Ox03f10_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss = Ox03f10_IsiSetFpsIss;
		pIsiSensor->pIsiGetIspStatusIss = Ox03f10_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss = Ox03f10_IsiSetWBIss;
		pIsiSensor->pIsiGetWBIss = Ox03f10_IsiGetWBIss;
		pIsiSensor->pIsiSetBlcIss = Ox03f10_IsiSetBlcIss;
		pIsiSensor->pIsiGetBlcIss = Ox03f10_IsiGetBlcIss;
		pIsiSensor->pIsiSetTpgIss = Ox03f10_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss = Ox03f10_IsiGetTpgIss;
		pIsiSensor->pIsiGetExpandCurveIss = Ox03f10_IsiGetExpandCurveIss;
		pIsiSensor->pIsiFocusCreateIss = NULL;
		pIsiSensor->pIsiFocusReleaseIss = NULL;
		pIsiSensor->pIsiFocusGetCalibrateIss = NULL;
		pIsiSensor->pIsiFocusSetIss = NULL;
		pIsiSensor->pIsiFocusGetIss = NULL;
		pIsiSensor->pIsiSetIRLightExpIss = NULL;
		pIsiSensor->pIsiGetIRLightExpIss = NULL;
	} else {
		result = RET_NULL_POINTER;
	}

	TRACE(Ox03f10_INFO, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t Ox03f10_IsiCamDrvConfig = {
	.cameraDriverID      = OX03F10_SENSOR_ID,
	.pIsiGetSensorIss    = Ox03f10_IsiGetSensorIss,
};
