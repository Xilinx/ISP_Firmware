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
 ******************************************************************************/

#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/misc.h>
#include <isi/isi_fmc.h>
#include "common/return_codes.h"
#include "isi/isi.h"
#include "isi/isi_iss.h"
#include "isi/isi_priv.h"
#include "sensor_drv/imx728_priv.h"

#define WITH_VTS		(1)
//#define WITH_HTS		(0)

extern int g_Sensor_frame_count;

CREATE_TRACER(IMX728_INFO, "IMX728: ", INFO, 0);
CREATE_TRACER(IMX728_WARN, "IMX728: ", WARNING, 0);
CREATE_TRACER(IMX728_ERROR, "IMX728: ", ERROR, 1);
CREATE_TRACER(IMX728_DEBUG, "IMX728: ", INFO, 0);
CREATE_TRACER(IMX728_REG_INFO, "IMX728: ", INFO, 0);
CREATE_TRACER(IMX728_REG_DEBUG, "IMX728: ", INFO, 0);

/*
 * Sensor Info
 */

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
		.intTimeDelayFrame = 2,
		.gainDelayFrame = 2,
	    },
	    .fps = 30 * ISI_FPS_QUANTIZE,
	    .hdrMode = ISI_SENSOR_MODE_HDR_NATIVE,
	    .nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
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

/*
 * each sensor driver need declare this struct for isi load
 */
IsiCamDrvConfig_t IMX728_IsiCamDrvConfig = {
	.cameraDriverID = 0x931801,
	.pIsiGetSensorIss = IMX728_IsiGetSensorIss,
};

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
				    const uint16_t addr,  uint16_t *pValue)
{
	int Status = XST_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *)handle;

	if (pIMX728Ctx == NULL)
		return RET_NULL_POINTER;

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO,
			       ((g_fmc_single.sensor_array[pIMX728Ctx->sensorDevId]->sensor_alias_addr) >> 1),
			       addr, 0x2, pValue, 1);
	if (Status != XST_SUCCESS)
		return RET_FAILURE;

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
static RESULT IMX728_IsiWriteRegIss(IsiSensorHandle_t handle, const uint16_t addr,
					const uint16_t value)
{
	int Status = XST_SUCCESS;
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *)handle;

	if (pIMX728Ctx == NULL)
		return RET_NULL_POINTER;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO,
			((g_fmc_single.sensor_array[pIMX728Ctx->sensorDevId]->sensor_alias_addr)
			>> 1), addr, 0x2, value, 1);
	if (Status != XST_SUCCESS)
		return RET_FAILURE;

	return RET_SUCCESS;
}

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
static RESULT IMX728_IsiUpdateRegIss(IsiSensorHandle_t handle, const uint16_t addr,
				const uint8_t mask, const uint8_t value)
{
	RESULT result = RET_SUCCESS;
	uint8_t reg_val = 0;

	result = IMX728_IsiReadRegIss(handle,  addr,  (uint16_t *)&reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to read reg - %x\n", __func__,   addr);
		return RET_FAILURE;
	}

	reg_val = (mask & value) | (~mask & reg_val);

	result = IMX728_IsiWriteRegIss(handle,  addr,  reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to update reg - %x\n", __func__,   addr);
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
	uint8_t remap_mode = 0;

	RESULT result = RET_SUCCESS;

	result = IMX728_IsiReadRegIss(handle,  IMX728_REG_REGMAP, (uint16_t *)&remap_mode);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to read remap mode!\n", __func__);
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
	uint8_t sensor_state = 0;

	if (IMX728_IsiReadRegIss(handle, IMX728_REG_STATE,
				(uint16_t *)&sensor_state) != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to read remap mode!\n", __func__);
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
 * @return  void
 *
 *****************************************************************************/
void imx728_sensor_framecount(IsiSensorHandle_t handle)
{
	u32 read_buf[4] = {0};
	int Status = XST_SUCCESS;

	Status = IMX728_IsiReadRegIss(handle,  0x8280, &read_buf[0]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = IMX728_IsiReadRegIss(handle,  0x8281, &read_buf[1]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = IMX728_IsiReadRegIss(handle,  0x8282, &read_buf[2]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = IMX728_IsiReadRegIss(handle,  0x8283, &read_buf[3]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	g_Sensor_frame_count = (read_buf[3] << 24) | (read_buf[2] << 16) | (read_buf[1] << 8) |
				(read_buf[0]);
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
		pIsiSensor->pszName                             = SensorName;
		pIsiSensor->pIsiCreateIss                       = IMX728_IsiCreateIss;
		pIsiSensor->pIsiOpenIss                         = IMX728_IsiOpenIss;
		pIsiSensor->pIsiCloseIss                        = IMX728_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss                      = IMX728_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss                      = IMX728_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss                     = IMX728_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss                      = IMX728_IsiGetModeIss;
		pIsiSensor->pIsiEnumModeIss                     = IMX728_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss                      = IMX728_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss              = IMX728_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss                  = IMX728_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss                 = IMX728_IsiSetStreamingIss;
		pIsiSensor->pIsiGetAeBaseInfoIss                = IMX728_pIsiGetAeBaseInfoIss;
		pIsiSensor->pIsiGetAGainIss                     = IMX728_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss                     = IMX728_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss                     = IMX728_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss                     = IMX728_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss                   = IMX728_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss                   = IMX728_IsiSetIntTimeIss;
		pIsiSensor->pIsiGetFpsIss                       = IMX728_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss                       = IMX728_IsiSetFpsIss;
		pIsiSensor->pIsiGetIspStatusIss                 = IMX728_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss                        = IMX728_IsiSetWBIss;
		pIsiSensor->pIsiGetWBIss                        = IMX728_IsiGetWBIss;
		pIsiSensor->pIsiSetBlcIss                       = IMX728_IsiSetBlcIss;
		pIsiSensor->pIsiGetBlcIss                       = IMX728_IsiGetBlcIss;
		pIsiSensor->pIsiSetTpgIss                       = IMX728_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss                       = IMX728_IsiGetTpgIss;
		pIsiSensor->pIsiGetExpandCurveIss               = IMX728_IsiGetExpandCurveIss;
		pIsiSensor->pIsiFocusCreateIss                  = NULL;
		pIsiSensor->pIsiFocusReleaseIss                 = NULL;
		pIsiSensor->pIsiFocusGetCalibrateIss            = NULL;
		pIsiSensor->pIsiFocusSetIss                     = NULL;
		pIsiSensor->pIsiFocusGetIss                     = NULL;
		pIsiSensor->pIsiSetIRLightExpIss                = NULL;
		pIsiSensor->pIsiGetIRLightExpIss                = NULL;
	} else
		result = RET_NULL_POINTER;

	TRACE(IMX728_INFO, "%s (exit)\n", __func__);
	return  result;
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
static RESULT IMX728_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig, IsiSensorHandle_t *pHandle)
{
	RESULT result = RET_SUCCESS;
	uint32_t desId = 0;

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) osMalloc(sizeof(IMX728_Context_t));

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Can't allocate IMX728 context\n", __func__);
		return RET_OUTOFMEM;
	}

	MEMSET(pIMX728Ctx, 0, sizeof(IMX728_Context_t));

	pIMX728Ctx->isiCtx.pSensor	= pConfig->pSensor;
	pIMX728Ctx->configured		= BOOL_FALSE;
	pIMX728Ctx->streaming		= BOOL_FALSE;
	pIMX728Ctx->testPattern		= BOOL_FALSE;
	pIMX728Ctx->isAfpsRun		= BOOL_FALSE;
	pIMX728Ctx->sensorMode.index	= 0;
	pIMX728Ctx->i2cId		= 0;
	pIMX728Ctx->sensorDevId		= pConfig->cameraDevId;

	IsiSensorSccbCfg_t sccbConfig;

	sccbConfig.slaveAddr = 0x6c >> 1;
	sccbConfig.addrByte  = 2;
	sccbConfig.dataByte  = 1;

	*pHandle = (IsiSensorHandle_t) pIMX728Ctx;

	if (pIMX728Ctx->sensorDevId >= IN_PIPE_LAST) {
		TRACE(IMX728_ERROR, "%s: sensor device ID %d is mot support!\n", __func__,
			pIMX728Ctx->sensorDevId);
		return RET_UNSUPPORT_ID;
	}

	desId = MAPPING_INPIPE_TO_DES_ID(pIMX728Ctx->sensorDevId);

	TRACE(IMX728_INFO, "desId %d pipeId:%d\n", desId, pIMX728Ctx->sensorDevId);

	init_des(desId);
	init_sensor(pIMX728Ctx->sensorDevId, desId);
	init_iic_access(pIMX728Ctx->i2cId, pIMX728Ctx->sensorDevId);

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
static RESULT IMX728_IsiEnumModeIss(IsiSensorHandle_t handle,  IsiSensorEnumMode_t *pEnumMode)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_NULL_POINTER;

	if (pEnumMode->index >= (ARRAY_SIZE(pimx728_mode_info)))
		return RET_OUTOFRANGE;

	for (uint32_t i = 0; i < (ARRAY_SIZE(pimx728_mode_info)); i++) {
		if (pimx728_mode_info[i].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &pimx728_mode_info[i], sizeof(IsiSensorMode_t));
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

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_NULL_POINTER;

	if (IMX728_GetSensorState(handle) != IMX728_SENSOR_STATE_SLEEP) {
		TRACE(IMX728_ERROR, "%s:INCK can be set in Sleep State. Other states not allowed.\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_INCK_FREQ, FMC_CLK_HZ / 1000000);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set input clock frequency!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_INCK_EN, 0x01);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't enable INCK frequency!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_SLEEP, 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't set to standby state!\n", __func__);
		return RET_FAILURE;
	}

	vTaskDelay(30);

	if (IMX728_GetSensorState(handle) != IMX728_SENSOR_STATE_STANDBY) {
		TRACE(IMX728_ERROR, "%s: Couldn't transition from Sleep to Standby state!\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_REGMAP, IMX728_REMAP_MODE_STANDBY);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't write regmap mode!\n", __func__);
		return RET_FAILURE;
	}

	vTaskDelay(1);
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
static RESULT IMX728_IsiGetRevisionIss(IsiSensorHandle_t handle,  uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val;
	uint32_t sensor_id;

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_NULL_POINTER;

	reg_val   = 0;
	result    = IMX728_IsiReadRegIss(handle,  IMX728_REG_CHIP_ID2, &reg_val);
	sensor_id = (reg_val & 0xff) << 16;

	reg_val   = 0;
	result    |= IMX728_IsiReadRegIss(handle,  IMX728_REG_CHIP_ID1, &reg_val);
	sensor_id |= ((reg_val & 0xff) << 8);

	reg_val   = 0;
	result    |= IMX728_IsiReadRegIss(handle,  IMX728_REG_CHIP_ID0, &reg_val);
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

	xil_printf("========= IMX728 Latest With 2A =========\n");

	result = IMX728_SetClknSteadyby(handle);
	if (result != RET_SUCCESS)
		TRACE(IMX728_ERROR, "%s: Powerup and Standby Failed!\n", __func__);

	uint32_t sensor_id = 0;
	uint32_t correct_id = 0x931801;

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_NULL_POINTER;

	result = IMX728_IsiGetRevisionIss(handle,  &sensor_id);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Read Sensor ID Error!\n", __func__);
		TRACE(IMX728_INFO, "%s: get revision fetched success\n", __func__);
	}

	if ((correct_id & 0xFFFF) != (sensor_id & 0xFFFF)) {
		TRACE(IMX728_ERROR, "%s: ChipID =0x%x sensor_id=%x error!\n", __func__,
			correct_id, sensor_id);
	}

	TRACE(IMX728_INFO, "%s: check connection done\n", __func__);
	return result;
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
RESULT IMX728_IsiSetFpsIss(IsiSensorHandle_t handle,  uint32_t fps)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (IMX728_GetSensorState(handle) != IMX728_SENSOR_STATE_STANDBY) {
		TRACE(IMX728_ERROR, "%s: FPS can be set in Standby State. Other states not allowed.\n",
			__func__);
		return RET_FAILURE;
	}

	if (fps > pIMX728Ctx->maxFps) {
		TRACE(IMX728_ERROR, "%s: FPS cannot be greater than %d.\n", __func__,
			pIMX728Ctx->maxFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}

	if (fps < pIMX728Ctx->minFps) {
		TRACE(IMX728_ERROR, "%s: FPS cannot be less than %d.\n", __func__,
			pIMX728Ctx->minFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}

	fps /= ISI_FPS_QUANTIZE;

uint16_t fps_value;

#ifdef WITH_HTS
	xil_printf("Set fps with hts\n");
	fps_value = (30 * 6000) / fps; 

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_L, (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_H, (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_APL_L, (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_FPS_APL_H, (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}
#endif

#ifdef WITH_VTS
	xil_printf("Set fps with vts\n");
	fps_value = (30 * 2400) / fps; 

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_L, (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_H, (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_APL_L, (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_VTS_FPS_APL_H, (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream! \n",__func__);
		return (RET_FAILURE);
	}
#endif


    return (result);
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
RESULT IMX728_IsiGetFpsIss(IsiSensorHandle_t handle,  uint32_t *pFps)
{
	RESULT result = RET_SUCCESS;
	uint16_t fps_reg = 0;
	uint8_t value = 0;

	result = IMX728_IsiReadRegIss(handle,  IMX728_REG_FPS_APL_L, &value);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Get FPS!\n", __func__);
		return RET_FAILURE;
	}

	fps_reg = value;

	result = IMX728_IsiReadRegIss(handle,  IMX728_REG_FPS_APL_H, &value);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Get FPS!\n", __func__);
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
 *          IMX728_InitialExposure
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
static RESULT IMX728_InitialExposure(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	float initial_exposure_time = 0.010f;

	result = IMX728_SetIntTime(handle,  initial_exposure_time);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set initial exposure time!\n", __func__);
		return RET_FAILURE;
	}

	IsiSensorGain_t pSensorAGain;

	pSensorAGain.gain[ISI_LINEAR_PARAS] = 15.85f;

	result = IMX728_IsiSetAGainIss(handle,  &pSensorAGain);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set initial analog gain values!\n", __func__);
		return RET_FAILURE;
	}

	IsiSensorGain_t pSensorDGain;

	pSensorDGain.gain[ISI_LINEAR_PARAS] = 1.0f;

	result = IMX728_IsiSetDGainIss(handle,  &pSensorDGain);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set initial digital gain values!\n", __func__);
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
static RESULT IMX728_HDRConfigure(IsiSensorHandle_t handle,  struct imx728_ctrl_point *points)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	uint32_t hdr_norm_x0;
	uint32_t hdr_norm_x1;
	uint16_t hdr_norm_y0;
	uint16_t hdr_norm_y1;

	hdr_norm_x0 = 0x3000;
	hdr_norm_x1 = 0x5000;

	hdr_norm_y0 = 0x0;
	hdr_norm_y1 = 0xe000;

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_LL, (hdr_norm_x0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_LH, (hdr_norm_x0 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_HL, (hdr_norm_x0 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X0_HH, (hdr_norm_x0 >> 24) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_LL, (hdr_norm_x0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_LH, (hdr_norm_x0 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_HL, (hdr_norm_x0 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X0_HH, (hdr_norm_x0 >> 24) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_LL, (hdr_norm_x1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_LH, (hdr_norm_x1 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_HL, (hdr_norm_x1 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_X1_HH, (hdr_norm_x1 >> 24) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_LL, (hdr_norm_x1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_LH, (hdr_norm_x1 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_HL, (hdr_norm_x1 >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_X1_HH, (hdr_norm_x1 >> 24) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y0_L, (hdr_norm_y0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y0_H, (hdr_norm_y0 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y0_L, (hdr_norm_y0 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y0_H, (hdr_norm_y0 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y1_L, (hdr_norm_y1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_LOW_Y1_H, (hdr_norm_y1 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y1_L, (hdr_norm_y1 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_NG_HIGH_Y1_H, (hdr_norm_y1 >> 8) & 0xFF);

	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed when setting HDR Normalization gains!\n", __func__);
		return RET_FAILURE;
	}

	int i;
	
	for (i = 0; i < 16; i++) {
		result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_CTRL_POINT_X(i),
						(points->x >> 0) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_CTRL_POINT_X(i) + 1,
						(points->x >> 8) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_CTRL_POINT_X(i) + 2,
						(points->x >> 16) & 0xFF);

		result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_CTRL_POINT_Y(i),
						(points->y >> 0) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_CTRL_POINT_Y(i) + 1,
						(points->y >> 8) & 0xFF);
		result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_CTRL_POINT_Y(i) + 2,
						(points->y >> 16) & 0xFF);

	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to write control point %i\n", __func__, i);
		return RET_FAILURE;
	}

	if ((points+1)->x >= 0 && (points+1)->y >= 0)
		points++;
	}
	
	result = IMX728_InitialExposure(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set initial exposure values!\n", __func__);
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

	IMX728_RawMode_e img_out_mode;
	IMX728_DriveMode_e mode_sel;

	img_out_mode = IMX728_IMG_MODE_HDR;
	mode_sel = IMX728_MODE_3856x2176_40_4LANE_RAW12_HDR;

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_AE_MODE, IMX728_AEMODE_FULL_ME);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't set full manual AE mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_AWBMODE, IMX728_AWBMODE_FULL_MWB);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't set full manual white balance mode!\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiUpdateRegIss(handle,  IMX728_REG_AWB_EN, 0b00000001, 0x01);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't enable full manual white balance mode!\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXP_UNIT_SP1,
					IMX728_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't write SP1 exposure time unit to microseconds!\n",
			__func__);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXP_UNIT_SP2,
					IMX728_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't write SP2 exposure time unit to microseconds!\n",
			__func__);
		return RET_FAILURE;
	}
	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXP_UNIT_SP1VS,
					IMX728_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't write SP1 VS exposure time unit to microseconds!\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_UNIT,
					IMX728_FMWB_AGAIN_UNIT_MAG_FACT);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't write analog gain unit to magnification factor!\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_DGAIN_UNIT,
					IMX728_FMWB_DGAIN_UNIT_MAG_FACT);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't write digital gain unit to magnification factor\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX728_HDRConfigure(handle,  imx728_hdr_24bit);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Couldn't configure sensor for HDR mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_MD_FEBD, 0x00);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_MD_REBD, 0x00);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_MD_APH, 0x00);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_MD_APF, 0x00);

	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Error disabling metadata!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_MODE_SEL_L, (mode_sel >> 0) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle,  IMX728_REG_MODE_SEL_H, 0b01111111,
						(mode_sel >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set Bitdepth Mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiUpdateRegIss(handle,  IMX728_REG_FW_OUT_MODE, 0b00000111, img_out_mode);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set Image Output mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_OB_0_L, (0X28 >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_OB_0_H, (0x28 >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_OB_1, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to disable Optical Black output!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_SKEW, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed disabling skew calibration from sensor to SER!\n",
			__func__);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX728_SerDeserSettings
 *
 * @brief   Configure serializer and deserializer settings.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the status of the operation.
 * @retval  XST_SUCCESS
 * @retval  XST_FAILURE
 *
 *****************************************************************************/
int IMX728_SerDeserSettings(IsiSensorHandle_t handle)
{
	int Status = XST_SUCCESS;

	TRACE(IMX728_INFO, "%s: ser-des config start\n", __func__);

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02be, 0x2, 0x80, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02bf, 0x2, 0x60, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0010, 0x2, 0x31, 1);
	if (Status != XST_SUCCESS)
		return Status;

	sleep(1);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0010, 0x2, 0x21, 1);
	if (Status != XST_SUCCESS)
		return Status;

	sleep(1);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0003, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0005, 0x2, 0x40, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0001, 0x2, 0x42, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0001, 0x2, 0x48, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0330, 0x2, 0x10, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0332, 0x2, 0xE4, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0333, 0x2, 0x44, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0331, 0x2, 0x31, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0311, 0x2, 0x20, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0308, 0x2, 0x62, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0314, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0316, 0x2, 0x6c, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0318, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x031A, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0002, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0330, 0x2, 0x04, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0333, 0x2, 0x4E, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0334, 0x2, 0xE4, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x040A, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x044A, 0x2, 0xd0, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x048A, 0x2, 0xd0, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x04CA, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x031D, 0x2, 0x26, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0320, 0x2, 0x2c, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0323, 0x2, 0x2c, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0326, 0x2, 0x26, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0050, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0051, 0x2, 0x01, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0052, 0x2, 0x02, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0053, 0x2, 0x03, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0332, 0x2, 0xF0, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02be, 0x2, 0x90, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02bf, 0x2, 0x60, 1);
	if (Status != XST_SUCCESS)
		return Status;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x03F1, 0x2, 0x89, 1);
	if (Status != XST_SUCCESS)
		return Status;

	TRACE(IMX728_INFO, "ser-des config sleep(1)\n");
	sleep(1);

	return Status;
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
static RESULT IMX728_IsiOpenIss(IsiSensorHandle_t handle,  uint32_t mode)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (pIMX728Ctx->streaming != BOOL_FALSE)
		return RET_WRONG_STATE;

	pIMX728Ctx->sensorMode.index   = mode;
	IsiSensorMode_t *SensorDefaultMode = NULL;

	for (int i = 0; i < sizeof(pimx728_mode_info) / sizeof(IsiSensorMode_t); i++) {
		if (pimx728_mode_info[i].index == pIMX728Ctx->sensorMode.index) {
			SensorDefaultMode = &(pimx728_mode_info[i]);
		break;
		}
	}

	if (SensorDefaultMode != NULL) {
		int Status = XST_SUCCESS;

	for (int i = 0; i < ARRAY_SIZE(IMX728_3840x2160_init); i++) {
		if (IMX728_3840x2160_init[i][0] == IMX728_TABLE_WAIT)
			vTaskDelay(IMX728_3840x2160_init[i][1]);
		else if (IMX728_3840x2160_init[i][0] == IMX728_TABLE_END)
			break;
		else if (IMX728_3840x2160_init[i][0] == IMX728_TABLE_REMAP) {
			IMX728_IsiWriteRegIss(handle, IMX728_REG_REGMAP, IMX728_REMAP_MODE_STANDBY);
			vTaskDelay(10);
		} else
			IMX728_IsiWriteRegIss(handle, IMX728_3840x2160_init[i][0],
					IMX728_3840x2160_init[i][1]);
	}
	sleep(1);

	memcpy(&(pIMX728Ctx->sensorMode), SensorDefaultMode, sizeof(IsiSensorMode_t));
	} else {
		TRACE(IMX728_ERROR, "%s: Invalid SensorDefaultMode\n", __func__);
		return RET_NULL_POINTER;
	}

	pIMX728Ctx->aecMinIntegrationTime	= 0.001;
	pIMX728Ctx->aecMaxIntegrationTime	= 0.030;
	pIMX728Ctx->aecIntegrationTimeStep	= 0.00001;

	pIMX728Ctx->aGain.min			= 7.94;
	pIMX728Ctx->aGain.max			= 15.85;
	pIMX728Ctx->aGain.step			= 0.01;

	pIMX728Ctx->dGain.min			= 1;
	pIMX728Ctx->dGain.max			= 32;
	pIMX728Ctx->dGain.step			= 0.01;

	pIMX728Ctx->aecMinGain			= pIMX728Ctx->aGain.min * pIMX728Ctx->dGain.min;
	pIMX728Ctx->aecMaxGain			= pIMX728Ctx->aGain.max * pIMX728Ctx->dGain.max;
	pIMX728Ctx->aecGainIncrement		= 0.1;

	pIMX728Ctx->minWBGain			= 0x001;
	pIMX728Ctx->maxWBGain			= 0xFFF;

	pIMX728Ctx->maxFps			= pIMX728Ctx->sensorMode.fps;
	pIMX728Ctx->minFps			= 0.25 * ISI_FPS_QUANTIZE;
	pIMX728Ctx->currFps			= pIMX728Ctx->maxFps;

	pIMX728Ctx->sensorWb.rGain		= 1.0;
	pIMX728Ctx->sensorWb.gbGain		= 1.0;
	pIMX728Ctx->sensorWb.grGain		= 1.0;
	pIMX728Ctx->sensorWb.bGain		= 1.0;

	TRACE(IMX728_DEBUG, "%s: IMX728 System-Reset executed\n", __func__);
	osSleep(100);

	result = IMX728_Configure(handle);	
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Configuring Sensor failed.\n", __func__);
		return result;
	}
	
	pIMX728Ctx->configured = BOOL_TRUE;
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
static RESULT IMX728_IsiGetModeIss(IsiSensorHandle_t handle,  IsiSensorMode_t *pMode)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;
	if (pMode == NULL)
		return RET_WRONG_HANDLE;

	memcpy(pMode, &(pIMX728Ctx->sensorMode), sizeof(pIMX728Ctx->sensorMode));
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
static RESULT IMX728_IsiGetCapsIss(IsiSensorHandle_t handle,  IsiCaps_t *pCaps)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *)handle;

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;

	if (pCaps == NULL)
		return RET_NULL_POINTER;

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

	return RET_SUCCESS;
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
RESULT IMX728_IsiGetTpgIss(IsiSensorHandle_t handle,  IsiSensorTpg_t *pTpg)
{
	return RET_SUCCESS;
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
RESULT IMX728_IsiSetTpgIss(IsiSensorHandle_t handle,  IsiSensorTpg_t tpg)
{
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
static RESULT IMX728_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle, IsiAeBaseInfo_t *pAeBaseInfo)
{
	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	if (pIMX728Ctx == NULL) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pAeBaseInfo == NULL) {
		TRACE(IMX728_ERROR, "%s: NULL pointer received!!\n", __func__);
		return RET_NULL_POINTER;
	}

	pAeBaseInfo->longGain.min		= pIMX728Ctx->aecMinGain;
	pAeBaseInfo->longGain.max		= pIMX728Ctx->aecMaxGain;
	pAeBaseInfo->longIntTime.min		= pIMX728Ctx->aecMinIntegrationTime;
	pAeBaseInfo->longIntTime.max		= pIMX728Ctx->aecMaxIntegrationTime;
	pAeBaseInfo->aLongGain			= pIMX728Ctx->aGain;
	pAeBaseInfo->dLongGain			= pIMX728Ctx->dGain;
	pAeBaseInfo->curIntTime			= pIMX728Ctx->curIntTime;
	pAeBaseInfo->curGain.gain[0]		= pIMX728Ctx->curAgain.gain[0] *
								pIMX728Ctx->curDgain.gain[0];
	pAeBaseInfo->curGain.gain[1]		= pIMX728Ctx->curAgain.gain[1] *
								pIMX728Ctx->curDgain.gain[1];
	pAeBaseInfo->curGain.gain[2]		= pIMX728Ctx->curAgain.gain[2] *
								pIMX728Ctx->curDgain.gain[2];
	pAeBaseInfo->curGain.gain[3]		= pIMX728Ctx->curAgain.gain[3] *
								pIMX728Ctx->curDgain.gain[3];
	pAeBaseInfo->aecIntTimeStep		= pIMX728Ctx->aecIntegrationTimeStep;
	pAeBaseInfo->aecGainStep		= pIMX728Ctx->aecGainIncrement;
	pAeBaseInfo->nativeMode			= pIMX728Ctx->sensorMode.nativeMode;
	pAeBaseInfo->conversionGainDCG		= (float)IMX728_SP1_HCG_MIN_AGAIN /
							IMX728_SP1_LCG_MIN_AGAIN;

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
static RESULT IMX728_IsiSetStreamingIss(IsiSensorHandle_t handle,  bool_t mode)
{
	RESULT result = RET_SUCCESS;

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_NULL_POINTER;

	if (pIMX728Ctx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	if (mode == true) {
		result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_STREAM, 0x5C);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR, "%s: Failed to start sensor stream!\n", __func__);
			return RET_FAILURE;
		}
		result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_STREAM, 0xA3);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR, "%s: Failed to start sensor stream!\n", __func__);
			return RET_FAILURE;
		}
	} else {
		result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_STREAM, 0xA3);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR, "%s: Failed to start sensor stream!\n", __func__);
			return RET_FAILURE;
		}
		result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_STREAM, 0xFF);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream!\n", __func__);
			return RET_FAILURE;
		}
		result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_SLEEP, 0x52);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream!\n", __func__);
			return RET_FAILURE;
		}
		result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_SLEEP, 0xAE);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR, "%s: Failed to Stop sensor stream!\n", __func__);
			return RET_FAILURE;
		}
	}

	pIMX728Ctx->streaming = mode;
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

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;

	(void)IMX728_IsiSetStreamingIss(pIMX728Ctx, BOOL_FALSE);
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

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;

	stop_sensor(pIMX728Ctx->sensorDevId);

	MEMSET(pIMX728Ctx, 0, sizeof(IMX728_Context_t));
	osFree(pIMX728Ctx);

	return RET_SUCCESS;
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
static RESULT IMX728_IsiSetWBIss(IsiSensorHandle_t handle,  IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;

	if (!pWb)
		return RET_NULL_POINTER;

	uint16_t gain_r = (uint16_t)(pWb->rGain * IMX728_WB_SCALING_FACTOR);
	uint16_t gain_gr = (uint16_t)(pWb->grGain * IMX728_WB_SCALING_FACTOR);
	uint16_t gain_gb = (uint16_t)(pWb->gbGain * IMX728_WB_SCALING_FACTOR);
	uint16_t gain_b = (uint16_t)(pWb->bGain * IMX728_WB_SCALING_FACTOR);

	gain_r = MAX(MIN(gain_r, pIMX728Ctx->maxWBGain), pIMX728Ctx->minWBGain);
	gain_gr = MAX(MIN(gain_gr, pIMX728Ctx->maxWBGain), pIMX728Ctx->minWBGain);
	gain_gb = MAX(MIN(gain_gb, pIMX728Ctx->maxWBGain), pIMX728Ctx->minWBGain);
	gain_b = MAX(MIN(gain_b, pIMX728Ctx->maxWBGain), pIMX728Ctx->minWBGain);

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_R_L, (gain_r >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_R_H, (gain_r >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Set White Balance Gain for Red Channel!\n",
				__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_GR_L, (gain_gr >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_GR_H, (gain_gr >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Set White Balance Gain for Green Red Channel!\n",
				__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_GB_L, (gain_gb >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_GB_H, (gain_gb >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Set White Balance Gain for Green Blue Channel!\n",
				__func__);
		return RET_FAILURE;
	}

	result = IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_B_L, (gain_b >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle,  IMX728_REG_FULLMWBGAIN_B_H, (gain_b >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to Set White Balance Gain for Blue Channel!\n",
				__func__);
		return RET_FAILURE;
	}

	memcpy(&pIMX728Ctx->sensorWb, pWb, sizeof(IsiSensorWb_t));

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
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

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (pIMX728Ctx == NULL)
		return RET_WRONG_HANDLE;

	if (!pWb)
		return RET_NULL_POINTER;

	memcpy(pWb, &pIMX728Ctx->sensorWb, sizeof(IsiSensorWb_t));

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
RESULT IMX728_IsiGetAGainIss(IsiSensorHandle_t handle,  IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorAGain)
		return RET_NULL_POINTER;

	*pSensorAGain = pIMX728Ctx->curAgain;

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
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
RESULT IMX728_IsiSetAGainIss(IsiSensorHandle_t handle,  IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorAGain)
		return RET_NULL_POINTER;

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] < pIMX728Ctx->aGain.min) {
		TRACE(IMX728_WARN, "%s: invalid too small again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pIMX728Ctx->aGain.min;
	}

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] > pIMX728Ctx->aGain.max) {
		TRACE(IMX728_WARN, "%s: invalid too big again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pIMX728Ctx->aGain.max;
	}

	uint32_t sp1h_gain = (uint32_t)(pSensorAGain->gain[ISI_LINEAR_PARAS] *
							IMX728_AG_DG_SCALING_FACTOR);
	float32_t gainFactor = (sp1h_gain - IMX728_SP1_HCG_MIN_AGAIN) / (IMX728_SP1_HCG_MAX_AGAIN -
							IMX728_SP1_HCG_MIN_AGAIN);
	uint32_t sp1l_gain = IMX728_SP1_LCG_MIN_AGAIN + (gainFactor * (IMX728_SP1_LCG_MAX_AGAIN -
							IMX728_SP1_LCG_MIN_AGAIN));
	uint32_t sp1ec_gain = IMX728_SP1_EC_MIN_AGAIN + (gainFactor * (IMX728_SP1_EC_MAX_AGAIN -
							IMX728_SP1_EC_MIN_AGAIN));
	uint32_t sp2_gain = IMX728_SP2_MIN_AGAIN + (gainFactor * (IMX728_SP2_MAX_AGAIN -
							IMX728_SP2_MIN_AGAIN));
	uint32_t sp1vs_gain = IMX728_SP1_VS_MIN_AGAIN + (gainFactor * (IMX728_SP1_VS_MAX_AGAIN -
							IMX728_SP1_VS_MIN_AGAIN));

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1H_L, (sp1h_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1H_M, (sp1h_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1H_H, 0b00000001,
						(sp1h_gain >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1L_L,
						(sp1l_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1L_M, (sp1l_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1L_H, 0b00000001,
						(sp1l_gain >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1EC_L,
						(sp1ec_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1EC_M,
						(sp1ec_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1EC_H, 0b00000001,
						(sp1ec_gain >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP2_L, (sp2_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP2_M, (sp2_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP2_H, 0b00000001,
						(sp2_gain >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1VS_L,
						(sp1vs_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_AGAIN_SP1VS_M,
						(sp1vs_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_AGAIN_SP1VS_H, 0b00000001,
						(sp1vs_gain >> 16) & 0xFF);

	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set fixed analog gain values!\n", __func__);
		return RET_FAILURE;
	}

	pIMX728Ctx->curAgain.gain[0] = (float)sp1h_gain / IMX728_AG_DG_SCALING_FACTOR;
	pIMX728Ctx->curAgain.gain[1] = (float)sp1l_gain / IMX728_AG_DG_SCALING_FACTOR;
	pIMX728Ctx->curAgain.gain[2] = (float)sp2_gain / IMX728_AG_DG_SCALING_FACTOR;
	pIMX728Ctx->curAgain.gain[3] = (float)sp1vs_gain / IMX728_AG_DG_SCALING_FACTOR;

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
RESULT IMX728_IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorDGain)
		return RET_NULL_POINTER;

	*pSensorDGain = pIMX728Ctx->curDgain;

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
RESULT IMX728_IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorDGain)
		return RET_NULL_POINTER;

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] < pIMX728Ctx->dGain.min) {
		TRACE(IMX728_WARN, "%s: invalid too small again parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pIMX728Ctx->dGain.min;
	}

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] > pIMX728Ctx->dGain.max) {
		TRACE(IMX728_WARN, "%s: invalid too big again parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pIMX728Ctx->dGain.max;
	}

	uint32_t digital_gain = (uint32_t)(pSensorDGain->gain[ISI_LINEAR_PARAS] *
								IMX728_AG_DG_SCALING_FACTOR);

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_DGAIN_L, (digital_gain >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_DGAIN_M, (digital_gain >> 8) & 0xFF);
	result |= IMX728_IsiUpdateRegIss(handle, IMX728_REG_DGAIN_H, 0b00000001,
						(digital_gain >> 16) & 0xFF);

	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set fixed digital gain values!\n", __func__);
		return RET_FAILURE;
	}

	pIMX728Ctx->curDgain.gain[0] = MAX((float)digital_gain / IMX728_AG_DG_SCALING_FACTOR,
								pIMX728Ctx->dGain.min);
	pIMX728Ctx->curDgain.gain[1] = pIMX728Ctx->curDgain.gain[0];
	pIMX728Ctx->curDgain.gain[2] = pIMX728Ctx->curDgain.gain[0];
	pIMX728Ctx->curDgain.gain[3] = pIMX728Ctx->curDgain.gain[0];

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
RESULT IMX728_IsiGetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}
	if (!pSensorIntTime)
		return RET_NULL_POINTER;

	if (pIMX728Ctx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		*pSensorIntTime = pIMX728Ctx->curIntTime;

	} else {
		TRACE(IMX728_INFO, "%s:not support this ExpoFrmType.\n", __func__);
		return RET_NOTSUPP;
	}

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
RESULT IMX728_IsiSetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
				__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorIntTime)
		return RET_NULL_POINTER;

	if (pIMX728Ctx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		result = IMX728_SetIntTime(handle,  pSensorIntTime->intTime[ISI_LINEAR_PARAS]);
		if (result != RET_SUCCESS) {
			TRACE(IMX728_ERROR, "%s: set sensor IntTime[ISI_LINEAR_PARAS] error!\n",
				__func__);
			return RET_FAILURE;
		}
	} else {
		TRACE(IMX728_INFO, "%s:not support this ExpoFrmType.\n", __func__);
		return RET_NOTSUPP;
	}

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX728_SetIntTime
 *
 * @brief   Set integration time (internal helper function).
 *
 * @param   handle              `       Sensor instance handle
 * @param   newIntegrationTime          Integration time settings
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT IMX728_SetIntTime(IsiSensorHandle_t handle, float newIntegrationTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	IMX728_Context_t *pIMX728Ctx = (IMX728_Context_t *) handle;

	if (!pIMX728Ctx) {
		TRACE(IMX728_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	newIntegrationTime = MAX(MIN(newIntegrationTime, pIMX728Ctx->aecMaxIntegrationTime),
					pIMX728Ctx->aecMinIntegrationTime);
	
	uint32_t exposure_sp1_sp2_us = newIntegrationTime * SEC_TO_MICROSEC;
	uint32_t exposure_sp1vs_us = exposure_sp1_sp2_us * IMX728_SP1_SP1VS_EXP_RATIO;

	result = IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_LL,
						(exposure_sp1_sp2_us >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_LH,
						(exposure_sp1_sp2_us >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_HL,
						(exposure_sp1_sp2_us >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1_HH,
						(exposure_sp1_sp2_us >> 24) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_LL,
						(exposure_sp1_sp2_us >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_LH,
						(exposure_sp1_sp2_us >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_HL,
						(exposure_sp1_sp2_us >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP2_HH,
						(exposure_sp1_sp2_us >> 24) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_LL,
						(exposure_sp1vs_us >> 0) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_LH,
						(exposure_sp1vs_us >> 8) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_HL,
						(exposure_sp1vs_us >> 16) & 0xFF);
	result |= IMX728_IsiWriteRegIss(handle, IMX728_REG_EXPOSURE_SP1VS_HH,
						(exposure_sp1vs_us >> 24) & 0xFF);

	if (result != RET_SUCCESS) {
		TRACE(IMX728_ERROR, "%s: Failed to set fixed exposure time values!\n", __func__);
		return RET_FAILURE;
	}

	pIMX728Ctx->curIntTime.intTime[0] = (float)exposure_sp1_sp2_us / SEC_TO_MICROSEC;
	pIMX728Ctx->curIntTime.intTime[1] = pIMX728Ctx->curIntTime.intTime[0];
	pIMX728Ctx->curIntTime.intTime[2] = pIMX728Ctx->curIntTime.intTime[0];
	pIMX728Ctx->curIntTime.intTime[3] = (float)exposure_sp1vs_us / SEC_TO_MICROSEC;

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
RESULT IMX728_IsiGetIspStatusIss(IsiSensorHandle_t handle, IsiIspStatus_t *pIspStatus)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX728_INFO, "%s: (enter)\n", __func__);

	pIspStatus->useSensorAE  = false;
	pIspStatus->useSensorBLC = false;
	pIspStatus->useSensorAWB = true;

	TRACE(IMX728_INFO, "%s: (exit)\n", __func__);
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
static RESULT IMX728_IsiSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
	return RET_SUCCESS;
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
static RESULT IMX728_IsiGetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
	return RET_SUCCESS;
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
	return RET_SUCCESS;
}
