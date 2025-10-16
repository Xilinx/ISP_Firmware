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

#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/misc.h>
#include <isi/isi_fmc.h>
#include "isi/isi.h"
#include "isi/isi_iss.h"
#include "isi/isi_priv.h"
#include "sensor_drv/imx623_priv.h"

extern int g_Sensor_frame_count;

CREATE_TRACER(IMX623_INFO, "IMX623: ", INFO,    0);
CREATE_TRACER(IMX623_WARN, "IMX623: ", WARNING, 0);
CREATE_TRACER(IMX623_ERROR, "IMX623: ", ERROR,   0);
CREATE_TRACER(IMX623_DEBUG, "IMX623: ", INFO, 0);
CREATE_TRACER(IMX623_REG_INFO, "IMX623: ", INFO, 0);
CREATE_TRACER(IMX623_REG_DEBUG, "IMX623: ", INFO, 0);

IsiSensorMode_t pimx623_mode_info[] = {
	{
	.index     = 0,
	.size = {
	    .boundsWidth  = 1920,
	    .boundsHeight = 1080,
	    .top           = 0,
	    .left          = 0,
	    .width         = 1920,
	    .height        = 1080,
	},
	.aeInfo    = {
	    .intTimeDelayFrame = 2,
	    .gainDelayFrame = 2,
	},
	.fps       = 30 * ISI_FPS_QUANTIZE,
	.hdrMode  = ISI_SENSOR_MODE_HDR_NATIVE,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
	.bitWidth = 12,
	.compress.enable = 1,
	.compress.xBit  = 24,
	.compress.yBit  = 12,
	.bayerPattern = ISI_BPAT_RGGB,
	.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	.dataType = ISI_MODE_BAYER,
	.mipiLane = ISI_MIPI_4LANES,
	}
};

int imx623_mode_num = (int)(sizeof(pimx623_mode_info) / sizeof(IsiSensorMode_t));

/*****************************************************************************
 * each sensor driver need declare this struct for isi load
 *****************************************************************************/
IsiCamDrvConfig_t IMX623_IsiCamDrvConfig = {
	.cameraDriverID      = 0x831101,
	.pIsiGetSensorIss    = IMX623_IsiGetSensorIss,
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
static RESULT IMX623_IsiReadRegIss(IsiSensorHandle_t handle, const uint16_t addr, uint16_t *pValue)
{
	int Status = XST_SUCCESS;
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_NULL_POINTER;

	Status = HalReadI2CReg(IIC_INSTANCE_ZERO,
			((g_fmc_single.sensor_array[pIMX623Ctx->sensorDevId]->sensor_alias_addr)
				>> 1),
			addr, 0x2, pValue, 1);
	if (Status != XST_SUCCESS)
		return RET_FAILURE;

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
static RESULT IMX623_IsiWriteRegIss(IsiSensorHandle_t handle, const uint16_t addr,
		const uint16_t value)
{
	int Status = XST_SUCCESS;
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_NULL_POINTER;

	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO,
			((g_fmc_single.sensor_array[pIMX623Ctx->sensorDevId]->sensor_alias_addr)
				>> 1),
			addr, 0x2, value, 1);
	if (Status != XST_SUCCESS)
		return RET_FAILURE;

	return RET_SUCCESS;
}

/*****************************************************************************
 *          IMX623_IsiUpdateRegIss
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
static RESULT IMX623_IsiUpdateRegIss(IsiSensorHandle_t handle, const uint16_t addr,
		const uint8_t mask, const uint8_t value)
{
	RESULT result = RET_SUCCESS;
	uint8_t reg_val = 0;

	result = IMX623_IsiReadRegIss(handle, addr, (uint16_t *)&reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read reg - %x\n", __func__, addr);
		return RET_FAILURE;
	}

	reg_val = (mask & value) | (~mask & reg_val);
	result = IMX623_IsiWriteRegIss(handle, addr, reg_val);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to update reg - %x\n", __func__, addr);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX623_GetRemapMode
 *
 * @brief   Get the current remap mode from the sensor.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the remap mode.
 * @retval  RemapMode_e enumeration value
 * @retval  IMX623_REMAP_MODE_INVALID on error
 *
 *****************************************************************************/
RemapMode_e IMX623_GetRemapMode(IsiSensorHandle_t handle)
{
	uint8_t remap_mode = 0;

	RESULT result = RET_SUCCESS;

	result = IMX623_IsiReadRegIss(handle, IMX623_REG_REGMAP, (uint16_t *)&remap_mode);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read remap mode!\n", __func__);
		return IMX623_REMAP_MODE_INVALID;
	}

	return (RemapMode_e)remap_mode;
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
	uint8_t sensor_state = 0;

	RESULT result = RET_SUCCESS;

	result = IMX623_IsiReadRegIss(handle, IMX623_REG_STATE, (uint16_t *)&sensor_state);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to read remap mode!\n", __func__);
		return IMX623_SENSOR_STATE_INVALID;
	}

	TRACE(IMX623_INFO, "Inside %s and IMX623 Sensor State: %d\n", __func__, sensor_state);
	return (SensorState_e)sensor_state;
}

/*****************************************************************************
 *          imx623_sensor_framecount
 *
 * @brief   Read and update the global sensor frame count.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  void
 *
 *****************************************************************************/
void imx623_sensor_framecount(IsiSensorHandle_t handle)
{
	u32 frame_counter;
	u32 read_buf[4] = {0};
	int Status = XST_SUCCESS;

	Status = IMX623_IsiReadRegIss(handle, 0x7dc8, &read_buf[0]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = IMX623_IsiReadRegIss(handle, 0x7dc9, &read_buf[1]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = IMX623_IsiReadRegIss(handle, 0x7dca, &read_buf[2]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = IMX623_IsiReadRegIss(handle, 0x7dcb, &read_buf[3]);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	frame_counter = (read_buf[3] << 24) | (read_buf[2] << 16) |
		(read_buf[1] << 8) | (read_buf[0]);
	g_Sensor_frame_count = frame_counter;
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
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_INFO, "%s (enter)\n", __func__);

	static const char SensorName[16] = "IMX623";

	if (pIsiSensor != NULL) {
		pIsiSensor->pszName                             = SensorName;
		pIsiSensor->pIsiCreateIss                       = IMX623_IsiCreateIss;
		pIsiSensor->pIsiOpenIss                         = IMX623_IsiOpenIss;
		pIsiSensor->pIsiCloseIss                        = IMX623_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss                      = IMX623_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss                      = IMX623_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss                     = IMX623_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss                      = IMX623_IsiGetModeIss;
		pIsiSensor->pIsiEnumModeIss                     = IMX623_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss                      = IMX623_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss              = IMX623_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss                  = IMX623_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss                 = IMX623_IsiSetStreamingIss;
		pIsiSensor->pIsiGetAeBaseInfoIss                = IMX623_pIsiGetAeBaseInfoIss;
		pIsiSensor->pIsiGetAGainIss                     = IMX623_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss                     = IMX623_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss                     = IMX623_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss                     = IMX623_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss                   = IMX623_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss                   = IMX623_IsiSetIntTimeIss;
		pIsiSensor->pIsiGetFpsIss                       = IMX623_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss                       = IMX623_IsiSetFpsIss;
		pIsiSensor->pIsiGetIspStatusIss                 = IMX623_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss                        = IMX623_IsiSetWBIss;
		pIsiSensor->pIsiGetWBIss                        = IMX623_IsiGetWBIss;
		pIsiSensor->pIsiSetBlcIss                       = IMX623_IsiSetBlcIss;
		pIsiSensor->pIsiGetBlcIss                       = IMX623_IsiGetBlcIss;
		pIsiSensor->pIsiSetTpgIss                       = IMX623_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss                       = IMX623_IsiGetTpgIss;
		pIsiSensor->pIsiGetExpandCurveIss               = IMX623_IsiGetExpandCurveIss;
		pIsiSensor->pIsiFocusCreateIss                  = NULL;
		pIsiSensor->pIsiFocusReleaseIss                 = NULL;
		pIsiSensor->pIsiFocusGetCalibrateIss            = NULL;
		pIsiSensor->pIsiFocusSetIss                     = NULL;
		pIsiSensor->pIsiFocusGetIss                     = NULL;
		pIsiSensor->pIsiSetIRLightExpIss                = NULL;
		pIsiSensor->pIsiGetIRLightExpIss                = NULL;
	} else {
		result = RET_NULL_POINTER;
	}

	TRACE(IMX623_INFO, "%s (exit)\n", __func__);
	return  result;
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
static RESULT IMX623_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig, IsiSensorHandle_t *pHandle)
{
	RESULT result = RET_SUCCESS;
	uint32_t desId = 0, pipeId = 0;

	TRACE(IMX623_INFO, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) osMalloc(sizeof(IMX623_Context_t));

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Can't allocate IMX623 context\n", __func__);
		return RET_OUTOFMEM;
	}

	MEMSET(pIMX623Ctx, 0, sizeof(IMX623_Context_t));

	pIMX623Ctx->isiCtx.pSensor     = pConfig->pSensor;
	pIMX623Ctx->configured         = BOOL_FALSE;
	pIMX623Ctx->streaming          = BOOL_FALSE;
	pIMX623Ctx->testPattern        = BOOL_FALSE;
	pIMX623Ctx->isAfpsRun          = BOOL_FALSE;
	pIMX623Ctx->sensorMode.index   = 0;
	pIMX623Ctx->i2cId                      = 0;
	pIMX623Ctx->sensorDevId        = pConfig->cameraDevId;

	uint8_t busId = (uint8_t)pIMX623Ctx->i2cId;

	pipeId = pIMX623Ctx->sensorDevId;

	*pHandle = (IsiSensorHandle_t) pIMX623Ctx;

	if (pipeId >= IN_PIPE_LAST) {
		TRACE(IMX623_ERROR, "%s: sensor device ID %d is not support!\n", __func__, pipeId);
		return RET_UNSUPPORT_ID;
	}
	desId = MAPPING_INPIPE_TO_DES_ID(pipeId);

	TRACE(IMX623_INFO, "%s: desId: %d, pipeId: %d\n", __func__, desId, pipeId);

	init_iic_access(pIMX623Ctx->i2cId, pipeId);
	init_des(desId);
	init_sensor(pipeId, desId);

	TRACE(IMX623_INFO, "%s (exit)\n", __func__);
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX623_IsiEnumModeIss(IsiSensorHandle_t handle, IsiSensorEnumMode_t *pEnumMode)
{
	TRACE(IMX623_INFO, "%s (enter)\n", __func__);
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_NULL_POINTER;

	if (pEnumMode->index >= (ARRAY_SIZE(pimx623_mode_info)))
		return RET_OUTOFRANGE;

	for (uint32_t i = 0; i < (ARRAY_SIZE(pimx623_mode_info)); i++) {
		if (pimx623_mode_info[i].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &pimx623_mode_info[i], sizeof(IsiSensorMode_t));
			TRACE(IMX623_INFO, "%s (exit)\n", __func__);
			return RET_SUCCESS;
		}
	}

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
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_INFO, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_NULL_POINTER;

	if (!(IMX623_GetSensorState(handle) == IMX623_SENSOR_STATE_STARTUP_IMMEDIATE ||
				IMX623_GetSensorState(handle) == IMX623_SENSOR_STATE_STARTUP)) {
		TRACE(IMX623_ERROR, "%s: INCK can only be set in Startup state. Other states are not allowed.\n",
					__func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_LL, (FMC_CLK_HZ >> 0) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_LH, (FMC_CLK_HZ >> 8) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_HL, (FMC_CLK_HZ >> 16) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_HH, (FMC_CLK_HZ >> 24) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set input clock frequency!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_LL,
			(FMC_CLK_HZ >> 0) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_LH,
			(FMC_CLK_HZ >> 8) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_HL,
			(FMC_CLK_HZ >> 16) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EXC_HH,
			(FMC_CLK_HZ >> 24) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set input clock frequency!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_INCK_FREQ_EN, 0x1);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set input clock frequency!\n", __func__);
		return RET_FAILURE;
	}

	vTaskDelay(60);

	TRACE(IMX623_DEBUG, "%s (exit) result = %d\n", __func__, result);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetRevisionIss
 *
 * @brief   This function reads the sensor revision register and returns it.
 *
 * @param   handle      sensor instance handle
 * @param   pValue      pointer to revision
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
static RESULT IMX623_IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val;
	uint32_t sensor_id;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_NULL_POINTER;

	reg_val   = 0;
	result    = IMX623_IsiReadRegIss(handle, IMX623_REG_CHIP_ID2, &reg_val);
	sensor_id = (reg_val & 0xff) << 16;

	reg_val   = 0;
	result    |= IMX623_IsiReadRegIss(handle, IMX623_REG_CHIP_ID1, &reg_val);
	sensor_id |= ((reg_val & 0xff) << 8);

	reg_val   = 0;
	result    |= IMX623_IsiReadRegIss(handle, IMX623_REG_CHIP_ID0, &reg_val);
	sensor_id |= (reg_val & 0xff);

	*pValue = sensor_id;
	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
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

	uint32_t sensor_id = 0;
	uint32_t correct_id = 0x831101;

	TRACE(IMX623_INFO, "========= IMX623 Latest With 2A =========\n");

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_NULL_POINTER;

	TRACE(IMX623_INFO, "ser-des bypassed\n");

	result = IMX623_SetClknStartup(handle);
	if (result != RET_SUCCESS)
		TRACE(IMX623_ERROR, "%s: Powerup and Standby Failed!\n", __func__);

	TRACE(IMX623_INFO, "set clk done\n");

	result = IMX623_IsiGetRevisionIss(handle, &sensor_id);
	if (result != RET_SUCCESS)
		TRACE(IMX623_ERROR, "%s: Read Sensor ID Error!\n", __func__);

	TRACE(IMX623_INFO, "get revision done\n");

	if (correct_id != sensor_id)
		TRACE(IMX623_ERROR, "%s : ChipID =0x%x sensor_id=%x error!\n", __func__, correct_id,
			sensor_id);

	TRACE(IMX623_INFO, "%s ChipID = 0x%08x, sensor_id = 0x%08x, success!\n",
			__func__, correct_id, sensor_id);
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s: (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!(IMX623_GetSensorState(handle) == IMX623_SENSOR_STATE_STARTUP_IMMEDIATE ||
				IMX623_GetSensorState(handle) == IMX623_SENSOR_STATE_STARTUP)) {
		TRACE(IMX623_ERROR, "%s: FPS set in Standby State. Other states not allowed.\n",
			__func__);
		return RET_FAILURE;
	}

	if (fps > pIMX623Ctx->maxFps) {
		TRACE(IMX623_ERROR, "%s: FPS cannot be greater than %d.\n", __func__,
				pIMX623Ctx->maxFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}
	if (fps < pIMX623Ctx->minFps) {
		TRACE(IMX623_ERROR, "%s: FPS cannot be less than %d.\n", __func__,
				pIMX623Ctx->minFps / ISI_FPS_QUANTIZE);
		return RET_FAILURE;
	}


	fps /= ISI_FPS_QUANTIZE;
	uint32_t fps_value = ((30 * IMX623_VMAX_30FPS) / fps) - IMX623_VMAX_30FPS;

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_VMAX_OFFSET_L, (fps_value >> 0) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Set FPS register (VMAX_OFFSET)!\n", __func__);
		return RET_FAILURE;
	}
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_VMAX_OFFSET_M, (fps_value >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Set FPS register (VMAX_OFFSET)!\n", __func__);
		return RET_FAILURE;
	}
	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_VMAX_OFFSET_H, 0b00000011,
			(fps_value >> 16) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Set FPS register (VMAX_OFFSET)!\n", __func__);
		return RET_FAILURE;
	}

	TRACE(IMX623_DEBUG, "%s: (exit)\n", __func__);
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps)
{
	RESULT result = RET_SUCCESS;

	uint32_t vmax = 0;
	uint8_t value = 0;

	IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_L, &value);
	vmax = value;
	value = 0;
	IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_M, &value);
	vmax |= (value << 8);
	value = 0;
	IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_H, &value);
	vmax |= (value << 16);
	value = 0;

	uint32_t vmax_offset = 0;

	IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_OFFSET_L, &value);
	vmax_offset = value;
	value = 0;
	IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_OFFSET_M, &value);
	vmax_offset |= (value << 8);
	value = 0;
	IMX623_IsiReadRegIss(handle, IMX623_REG_VMAX_OFFSET_H, &value);
	vmax_offset |= (value << 16);
	value = 0;

	*pFps = (30 * IMX623_VMAX_30FPS) / (vmax + vmax_offset);
	*pFps *= ISI_FPS_QUANTIZE;

	imx623_sensor_framecount(handle);
	Fmc_Sensor_Statustask();

	return result;
}
/*****************************************************************************
 *          IMX623_InitFixedExposure
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
static RESULT IMX623_InitialExposure(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	float exposure_sp1_sp2_us = 0.01;

	result = IMX623_SetIntTime(handle, exposure_sp1_sp2_us);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set initial exposure time!\n", __func__);
		return RET_FAILURE;
	}

	IsiSensorGain_t pSensorAGain;

	pSensorAGain.gain[ISI_LINEAR_PARAS] = 7.94f;

	result = IMX623_IsiSetAGainIss(handle, &pSensorAGain);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set initial analog gain!\n", __func__);
		return RET_FAILURE;
	}

	IsiSensorGain_t pSensorDGain;

	pSensorDGain.gain[ISI_LINEAR_PARAS] = 1.0f;

	result = IMX623_IsiSetDGainIss(handle, &pSensorDGain);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set initial digital gain!\n", __func__);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX623_HDRConfigure
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
static RESULT IMX623_HDRConfigure(IsiSensorHandle_t handle, struct imx623_ctrl_point *points)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	for (int i = 0; i < 16; i++) {
		result = IMX623_IsiWriteRegIss(handle, IMX623_REG_CTRL_POINT_X(i),
			(points->x >> 0) & 0xFF);
		result = IMX623_IsiWriteRegIss(handle, IMX623_REG_CTRL_POINT_X(i) + 1,
			(points->x >> 8) & 0xFF);
		result = IMX623_IsiWriteRegIss(handle, IMX623_REG_CTRL_POINT_X(i) + 2,
			(points->x >> 16) & 0xFF);

		result = IMX623_IsiWriteRegIss(handle, IMX623_REG_CTRL_POINT_Y(i),
			(points->y >> 0) & 0xFF);
		result = IMX623_IsiWriteRegIss(handle, IMX623_REG_CTRL_POINT_Y(i) + 1,
			(points->y >> 8) & 0xFF);
		result = IMX623_IsiWriteRegIss(handle, IMX623_REG_CTRL_POINT_Y(i) + 2,
			(points->y >> 16) & 0xFF);

		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR, "%s: Failed to write control point %i\n", __func__, i);
			return RET_FAILURE;
		}

		if ((points+1)->x >= 0 && (points+1)->y >= 0)
			points++;
	}

	result = IMX623_InitialExposure(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set initial exposure values!\n", __func__);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX623_Configure
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
static RESULT IMX623_Configure(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MODE_SET_F_L, 0x1D);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Couldn't set Drive mode!\n", __func__);
		return RET_FAILURE;
	}
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MODE_SET_LOCK, 0x53);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Couldn't unlock drive mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AE_MODE, IMX623_AEMODE_FULL_ME);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Couldn't set full manual AE mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AWBMODE, IMX623_AWBMODE_FULL_MWB);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Couldn't set full manual white balance mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_UNIT_SP1,
						IMX623_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Couldn't write SP1 exposure time unit to microseconds!\n",
				__func__);
		return RET_FAILURE;
	}
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_UNIT_SP2,
						IMX623_FME_SHTVAL_UNIT_MICROSECONDS);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Couldn't write SP2 exposure time unit to microseconds!\n",
				__func__);
		return RET_FAILURE;
	}

	result = IMX623_HDRConfigure(handle, imx623_hdr_24bit);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Couldn't configure sensor for HDR mode!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MD_FEBD, 0x00);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_MD_REBD, 0x00);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Error disabling metadata!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_ADBIT, IMX623_AD_12BIT);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set AD Bitdepth!\n", __func__);
		return RET_FAILURE;
	}
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTMODE, IMX623_OUTMODE_RAW);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTMODE_APL, IMX623_OUTMODE_RAW);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set outmode!\n", __func__);
		return RET_FAILURE;
	}
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_RAW_OUTMODE, IMX623_RAW_OUTMODE_RAW12);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_RAW_OUTMODE_APL,
						IMX623_RAW_OUTMODE_RAW12);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set output bitdepth!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_HDRON, IMX623_IMG_MODE_HDR);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_HDRON_APL, IMX623_IMG_MODE_HDR);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Enable HDR Compression!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTSEL_1, IMX623_RAW_OUTSEL_SP1_HCG);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_OUTSEL_1_APL,
						IMX623_RAW_OUTSEL_SP1_HCG);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set Sub Pixel register!\n", __func__);
		return RET_FAILURE;
	}

	uint16_t horizontal_offset = 8;
	uint16_t vertical_offset = 236;
	uint16_t horizontal_size = 1920;
	uint16_t vertical_size = 1080;

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_ON, 0x1);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_ON_APL, 0x1);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to enable digital crop!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_L,
			(horizontal_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_H,
			(horizontal_offset >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_L_APL,
			(horizontal_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HOFFSET_H_APL,
			(horizontal_offset >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set horizontal crop offset!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_L,
			(vertical_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_H,
			(vertical_offset >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_L_APL,
			(vertical_offset >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VOFFSET_H_APL,
			(vertical_offset >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set vertical crop offset!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_L,
			(horizontal_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_H,
			(horizontal_size >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_L_APL,
			(horizontal_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_HSIZE_H_APL,
			(horizontal_size >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set horizontal crop size!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_L,
			(vertical_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_H,
			(vertical_size >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_L_APL,
			(vertical_size >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_DCROP_VSIZE_H_APL,
			(vertical_size >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set vertical crop size!\n", __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_DCROP_DATA_SEL, 0b00000001, 0x1);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to select custom digital crop!\n", __func__);
		return RET_FAILURE;
	}

	return result;
}

/*****************************************************************************
 *          IMX623_SerDeserSettings
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
int IMX623_SerDeserSettings(IsiSensorHandle_t handle)
{
	int Status = XST_SUCCESS;

	TRACE(IMX623_INFO, "ser-des config start\n");
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02be, 0x2, 0x80, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02bf, 0x2, 0x60, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0010, 0x2, 0x31, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	sleep(1);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0010, 0x2, 0x21, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	sleep(1);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0003, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0005, 0x2, 0x40, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0001, 0x2, 0x42, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0001, 0x2, 0x48, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0330, 0x2, 0x10, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0332, 0x2, 0xE4, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0333, 0x2, 0x44, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0331, 0x2, 0x31, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0311, 0x2, 0x20, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0308, 0x2, 0x62, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0314, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0316, 0x2, 0x6c, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0318, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x031A, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x0002, 0x2, 0x22, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0330, 0x2, 0x04, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0333, 0x2, 0x4E, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0334, 0x2, 0xE4, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x040A, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x044A, 0x2, 0xd0, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x048A, 0x2, 0xd0, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x04CA, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x031D, 0x2, 0x26, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0320, 0x2, 0x26, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0323, 0x2, 0x26, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0326, 0x2, 0x26, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0050, 0x2, 0x00, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0051, 0x2, 0x01, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0052, 0x2, 0x02, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0053, 0x2, 0x03, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x4a, 0x0332, 0x2, 0xF0, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02be, 0x2, 0x90, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x02bf, 0x2, 0x60, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	Status = HalWriteI2CReg(IIC_INSTANCE_ZERO, 0x40, 0x03F1, 0x2, 0x89, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	TRACE(IMX623_INFO, "ser-des config end\n");
	sleep(1);

	return Status;
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
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pIMX623Ctx->streaming != BOOL_FALSE)
		return RET_WRONG_STATE;

	pIMX623Ctx->sensorMode.index   = mode;
	IsiSensorMode_t *SensorDefaultMode = NULL;

	for (int i = 0; i < sizeof(pimx623_mode_info) / sizeof(IsiSensorMode_t); i++) {
		if (pimx623_mode_info[i].index == pIMX623Ctx->sensorMode.index) {
			SensorDefaultMode = &(pimx623_mode_info[i]);
			break;
		}
	}

	if (SensorDefaultMode != NULL) {
		int Status = XST_SUCCESS;

		for (int i = 0; i < ARRAY_SIZE(IMX623_1920x1080_init); i++) {
			if (IMX623_1920x1080_init[i][0] == IMX623_TABLE_WAIT) {
				vTaskDelay(IMX623_1920x1080_init[i][1]);
			} else if (IMX623_1920x1080_init[i][0] == IMX623_TABLE_END) {
				break;
			} else if (IMX623_1920x1080_init[i][0] == IMX623_TABLE_REMAP) {
				IMX623_IsiWriteRegIss(handle, IMX623_REG_REGMAP,
						IMX623_REMAP_MODE_STARTUP);
				vTaskDelay(10);
			} else {
				IMX623_IsiWriteRegIss(handle, IMX623_1920x1080_init[i][0],
						IMX623_1920x1080_init[i][1]);
			}
		}
		sleep(1);
		memcpy(&(pIMX623Ctx->sensorMode), SensorDefaultMode, sizeof(IsiSensorMode_t));
	} else {
		TRACE(IMX623_ERROR, "%s: Invalid SensorDefaultMode\n", __func__);
		return RET_NULL_POINTER;
	}

	pIMX623Ctx->aecMinIntegrationTime	= 0.001;
	pIMX623Ctx->aecMaxIntegrationTime	= 0.030;
	pIMX623Ctx->aecIntegrationTimeStep	= 0.00001;

	pIMX623Ctx->aGain.min			= IMX623_SP1_HCG_MIN_AGAIN;
	pIMX623Ctx->aGain.max			= IMX623_SP1_HCG_MAX_AGAIN;
	pIMX623Ctx->aGain.step			= 0.01;

	pIMX623Ctx->dGain.min			= 1;
	pIMX623Ctx->dGain.max			= 32;
	pIMX623Ctx->dGain.step			= 0.01;

	pIMX623Ctx->aecMinGain			= pIMX623Ctx->aGain.min * pIMX623Ctx->dGain.min;
	pIMX623Ctx->aecMaxGain			= pIMX623Ctx->aGain.max * pIMX623Ctx->dGain.max;
	pIMX623Ctx->aecGainIncrement		= 0.1;

	pIMX623Ctx->minWBGain			= 0x001;
	pIMX623Ctx->maxWBGain			= 0xFFF;

	pIMX623Ctx->maxFps			= pIMX623Ctx->sensorMode.fps;
	pIMX623Ctx->minFps			= 0.25 * ISI_FPS_QUANTIZE;
	pIMX623Ctx->currFps			= pIMX623Ctx->maxFps;

	pIMX623Ctx->sensorWb.rGain		= 1.0;
	pIMX623Ctx->sensorWb.gbGain		= 1.0;
	pIMX623Ctx->sensorWb.grGain		= 1.0;
	pIMX623Ctx->sensorWb.bGain		= 1.0;

	TRACE(IMX623_DEBUG, "%s : IMX623 System-Reset executed\n", __func__);
	osSleep(100);

	result = IMX623_Configure(handle);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s : Configuring Sensor failed.\n", __func__);
		return result;
	}

	pIMX623Ctx->configured = BOOL_TRUE;

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetModeIss
 *
 * @brief   get cuurent sensor mode info.
 *
 * @param   handle     Sensor instance handle
 * @param   pMode      Sensor mode ptr
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT IMX623_IsiGetModeIss(IsiSensorHandle_t handle, IsiSensorMode_t *pMode)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_WRONG_HANDLE;
	if (pMode == NULL)
		return RET_WRONG_HANDLE;

	memcpy(pMode, &(pIMX623Ctx->sensorMode), sizeof(pIMX623Ctx->sensorMode));

	return  RET_SUCCESS;
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

	if (pIMX623Ctx == NULL)
		return RET_WRONG_HANDLE;

	if (pCaps == NULL)
		return RET_NULL_POINTER;

	pCaps->bitWidth          = pIMX623Ctx->sensorMode.bitWidth;
	pCaps->mode              = ISI_MODE_BAYER;
	pCaps->bayerPattern      = pIMX623Ctx->sensorMode.bayerPattern;
	pCaps->resolution.width  = pIMX623Ctx->sensorMode.size.width;
	pCaps->resolution.height = pIMX623Ctx->sensorMode.size.height;
	pCaps->mipiLanes         = ISI_MIPI_4LANES;
	pCaps->vinType           = ISI_ITF_TYPE_MIPI;

	if (pCaps->bitWidth == 10)
		pCaps->mipiMode      = ISI_FORMAT_RAW_10;
	else if (pCaps->bitWidth == 12)
		pCaps->mipiMode      = ISI_FORMAT_RAW_12;
	else
		pCaps->mipiMode      = ISI_MIPI_OFF;

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

	return result;
}

/*****************************************************************************
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

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	TRACE(IMX623_DEBUG, "%s (exit)\n", __func__);
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX623_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle, IsiAeBaseInfo_t *pAeBaseInfo)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	if (pIMX623Ctx == NULL) {
		TRACE(IMX623_ERROR, "%s : Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pAeBaseInfo == NULL) {
		TRACE(IMX623_ERROR, "%s : NULL pointer received !!\n", __func__);
		return RET_NULL_POINTER;
	}

	pAeBaseInfo->longGain.min		= pIMX623Ctx->aecMinGain;
	pAeBaseInfo->longGain.max		= pIMX623Ctx->aecMaxGain;
	pAeBaseInfo->longIntTime.min		= pIMX623Ctx->aecMinIntegrationTime;
	pAeBaseInfo->longIntTime.max		= pIMX623Ctx->aecMaxIntegrationTime;
	pAeBaseInfo->aLongGain			= pIMX623Ctx->aGain;
	pAeBaseInfo->dLongGain			= pIMX623Ctx->dGain;
	pAeBaseInfo->curIntTime			= pIMX623Ctx->curIntTime;

	pAeBaseInfo->curGain.gain[0]		= pIMX623Ctx->curAgain.gain[0] *
			pIMX623Ctx->curDgain.gain[0];
	pAeBaseInfo->curGain.gain[1]		= pIMX623Ctx->curAgain.gain[1] *
			pIMX623Ctx->curDgain.gain[1];
	pAeBaseInfo->curGain.gain[2]		= pIMX623Ctx->curAgain.gain[2] *
			pIMX623Ctx->curDgain.gain[2];
	pAeBaseInfo->curGain.gain[3]		= pIMX623Ctx->curAgain.gain[3] *
			pIMX623Ctx->curDgain.gain[3];

	pAeBaseInfo->aecIntTimeStep		= pIMX623Ctx->aecIntegrationTimeStep;
	pAeBaseInfo->aecGainStep		= pIMX623Ctx->aecGainIncrement;
	pAeBaseInfo->nativeMode			= pIMX623Ctx->sensorMode.nativeMode;
	pAeBaseInfo->conversionGainDCG		= (float)IMX623_SP1_HCG_MIN_AGAIN /
							IMX623_SP1_LCG_MIN_AGAIN;

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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT IMX623_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t mode)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_NULL_POINTER;

	if (pIMX623Ctx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_MODE_SET_LOCK, 0x53);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s : Couldn't unlock MODE_SET_F register!\n", __func__);
		return RET_FAILURE;
	}

	if (mode == true) {
		result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_MODE_SET_F_H, 0b10000000, 0x80);
		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR, "%s: Failed to start sensor stream!\n", __func__);
			return RET_FAILURE;
		}
	} else {
		result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_MODE_SET_F_H, 0b10000000, 0x00);
		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR, "%s: Failed to start sensor stream!\n", __func__);
			return RET_FAILURE;
		}
	}

	pIMX623Ctx->streaming = mode;

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
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (pIMX623Ctx == NULL)
		return RET_WRONG_HANDLE;

	(void)IMX623_IsiSetStreamingIss(pIMX623Ctx, BOOL_FALSE);

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
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT IMX623_IsiReleaseIss(IsiSensorHandle_t handle)
{
	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_DEBUG, "%s (enter)\n", __func__);

	if (pIMX623Ctx == NULL)
		return RET_WRONG_HANDLE;

	stop_sensor(pIMX623Ctx->sensorDevId);

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
static RESULT IMX623_IsiSetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_WRONG_HANDLE;

	if (!pWb)
		return RET_NULL_POINTER;

	uint16_t gain_r = (uint16_t)(pWb->rGain * IMX623_WB_SCALING_FACTOR);
	uint16_t gain_gr = (uint16_t)(pWb->grGain * IMX623_WB_SCALING_FACTOR);
	uint16_t gain_gb = (uint16_t)(pWb->gbGain * IMX623_WB_SCALING_FACTOR);
	uint16_t gain_b = (uint16_t)(pWb->bGain * IMX623_WB_SCALING_FACTOR);

	gain_r = MAX(MIN(gain_r, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);
	gain_gr = MAX(MIN(gain_gr, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);
	gain_gb = MAX(MIN(gain_gb, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);
	gain_b = MAX(MIN(gain_b, pIMX623Ctx->maxWBGain), pIMX623Ctx->minWBGain);

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_R_L, (gain_r >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_R_H, (gain_r >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Set White Balance Gain for Red Channel!\n",
			__func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GR_L, (gain_gr >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GR_H,
			(gain_gr >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Set White Balance Gain for Green Red Channel!\n",
				__func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GB_L, (gain_gb >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_GB_H,
			(gain_gb >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Set White Balance Gain for Green Blue Channel!\n"
				, __func__);
		return RET_FAILURE;
	}

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_B_L, (gain_b >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_FULLMWBGAIN_B_H, (gain_b >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to Set White Balance Gain for Blue Channel!\n",
				__func__);
		return RET_FAILURE;
	}

	memcpy(&pIMX623Ctx->sensorWb, pWb, sizeof(IsiSensorWb_t));

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
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
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (pIMX623Ctx == NULL)
		return RET_WRONG_HANDLE;

	if (!pWb)
		return RET_NULL_POINTER;

	memcpy(pWb, &pIMX623Ctx->sensorWb, sizeof(IsiSensorWb_t));

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
	return result;
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_INFO, "%s: (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorAGain)
		return RET_NULL_POINTER;

	*pSensorAGain = pIMX623Ctx->curAgain;

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
	return result;
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiSetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_INFO, "%s: (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorAGain)
		return RET_NULL_POINTER;

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] < pIMX623Ctx->aGain.min) {
		TRACE(IMX623_WARN,  "%s: invalid too small again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->aGain.min;
	}

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] > pIMX623Ctx->aGain.max) {
		TRACE(IMX623_WARN,  "%s: invalid too big again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->aGain.max;
	}

	float32_t sp1_hcg = pSensorAGain->gain[ISI_LINEAR_PARAS];
	float32_t gainFactor = (sp1_hcg - IMX623_SP1_HCG_MIN_AGAIN) / (IMX623_SP1_HCG_MAX_AGAIN -
					IMX623_SP1_HCG_MIN_AGAIN);
	float32_t sp1_lcg = IMX623_SP1_LCG_MIN_AGAIN
		+ (gainFactor * (IMX623_SP1_LCG_MAX_AGAIN - IMX623_SP1_LCG_MIN_AGAIN));
	float32_t sp2_h = IMX623_SP2_H_MIN_AGAIN
		+ (gainFactor * (IMX623_SP2_H_MAX_AGAIN - IMX623_SP2_H_MIN_AGAIN));
	float32_t sp2_l = IMX623_SP2_L_MIN_AGAIN
		+ (gainFactor * (IMX623_SP2_L_MAX_AGAIN - IMX623_SP2_L_MIN_AGAIN));

	printf("Analog gain: %f\ %f\ %f\ %f\n", sp1_hcg, sp1_lcg, sp2_h, sp2_l);

	uint16_t sp1_hcg_gain = (uint16_t)IMX623_LINEAR_TO_DB(sp1_hcg)
		* IMX623_AG_DG_SCALING_FACTOR;
	uint16_t sp1_lcg_gain = (uint16_t)IMX623_LINEAR_TO_DB(sp1_lcg)
		* IMX623_AG_DG_SCALING_FACTOR;
	uint16_t sp2_h_gain = (uint16_t)IMX623_LINEAR_TO_DB(sp2_h)
		* IMX623_AG_DG_SCALING_FACTOR;
	uint16_t sp2_l_gain = (uint16_t)IMX623_LINEAR_TO_DB(sp2_l)
		* IMX623_AG_DG_SCALING_FACTOR;

	printf("Analog gain: %d\ %d\ %d\ %d\n", sp1_hcg_gain, sp1_lcg_gain, sp2_h_gain, sp2_l_gain);

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP1_HCG_L,
			(sp1_hcg_gain >> 0) & 0xFF);
	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP1_HCG_H, 0b00000011,
			(sp1_hcg_gain >> 8) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP1_LCG_L,
			(sp1_lcg_gain >> 0) & 0xFF);
	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP1_LCG_H, 0b00000011,
			(sp1_lcg_gain >> 8) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP2_H_L, (sp2_h_gain >> 0) & 0xFF);
	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP2_H_H, 0b00000011,
			(sp2_h_gain >> 8) & 0xFF);
	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_AGAIN_SP2_L_L, (sp2_l_gain >> 0) & 0xFF);
	result = IMX623_IsiUpdateRegIss(handle, IMX623_REG_AGAIN_SP2_L_H, 0b00000011,
			(sp2_l_gain >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set analog gain values!\n", __func__);
		return RET_FAILURE;
	}

	pIMX623Ctx->curAgain.gain[0] = sp1_hcg;
	pIMX623Ctx->curAgain.gain[1] = sp1_lcg;
	pIMX623Ctx->curAgain.gain[2] = sp2_h;
	pIMX623Ctx->curAgain.gain[3] = sp2_l;

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiGetDGainIss
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
RESULT IMX623_IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorDGain)
		return RET_NULL_POINTER;

	*pSensorDGain = pIMX623Ctx->curDgain;

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
	return result;
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_INFO, "%s: (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorDGain)
		return RET_NULL_POINTER;

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] < pIMX623Ctx->dGain.min) {
		TRACE(IMX623_WARN,  "%s: invalid too small again parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->dGain.min;
	}

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] > pIMX623Ctx->dGain.max) {
		TRACE(IMX623_WARN,  "%s: invalid too big again parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pIMX623Ctx->dGain.max;
	}

	uint16_t digital_gain = (uint16_t)IMX623_LINEAR_TO_DB(pSensorDGain->gain[ISI_LINEAR_PARAS])
			* IMX623_AG_DG_SCALING_FACTOR;


	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_DGAIN_L, (digital_gain >> 0) & 0xFF);
	result |= IMX623_IsiUpdateRegIss(handle, IMX623_REG_DGAIN_H, 0b00000011,
			(digital_gain >> 8) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set digital gain values!\n", __func__);
		return RET_FAILURE;
	}

	pIMX623Ctx->curDgain.gain[0] = pSensorDGain->gain[ISI_LINEAR_PARAS];
	pIMX623Ctx->curDgain.gain[1] = pIMX623Ctx->curDgain.gain[0];
	pIMX623Ctx->curDgain.gain[2] = pIMX623Ctx->curDgain.gain[0];
	pIMX623Ctx->curDgain.gain[3] = pIMX623Ctx->curDgain.gain[0];

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_INFO, "%s: (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}
	if (!pSensorIntTime)
		return RET_NULL_POINTER;

	if (pIMX623Ctx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		*pSensorIntTime = pIMX623Ctx->curIntTime;

	} else {
		TRACE(IMX623_INFO, "%s : Not support this ExpoFrmType.\n", __func__);
		return RET_NOTSUPP;
	}

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*****************************************************************************
 *          IMX623_IsiSetIntTimeIss
 *
	 @brief   Writes integration time values to the image sensor module.
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
RESULT IMX623_IsiSetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (!pSensorIntTime)
		return RET_NULL_POINTER;

	if (pIMX623Ctx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		result = IMX623_SetIntTime(handle, pSensorIntTime->intTime[ISI_LINEAR_PARAS]);
		if (result != RET_SUCCESS) {
			TRACE(IMX623_ERROR, "%s: set sensor IntTime[ISI_LINEAR_PARAS] error!\n",
			__func__);
			return RET_FAILURE;
		}

	} else {
		TRACE(IMX623_INFO, "%s : Not support this ExpoFrmType.\n", __func__);
		return RET_NOTSUPP;
	}

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
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
static RESULT IMX623_SetIntTime(IsiSensorHandle_t handle, float newIntegrationTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(IMX623_INFO, "%s: (enter)\n", __func__);

	IMX623_Context_t *pIMX623Ctx = (IMX623_Context_t *) handle;

	if (!pIMX623Ctx) {
		TRACE(IMX623_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	newIntegrationTime = MAX(MIN(newIntegrationTime, pIMX623Ctx->aecMaxIntegrationTime),
			pIMX623Ctx->aecMinIntegrationTime);
	unsigned int exposure_sp1_sp2_us = newIntegrationTime * SEC_TO_MICROSEC;

	printf("Exp time: %d\ %d\n", exposure_sp1_sp2_us, exposure_sp1_sp2_us);

	result = IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_LL,
			(exposure_sp1_sp2_us >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_LH,
			(exposure_sp1_sp2_us >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_HL,
			(exposure_sp1_sp2_us >> 16) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP1_HH,
			(exposure_sp1_sp2_us >> 24) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_LL,
			(exposure_sp1_sp2_us >> 0) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_LH,
			(exposure_sp1_sp2_us >> 8) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_HL,
			(exposure_sp1_sp2_us >> 16) & 0xFF);
	result |= IMX623_IsiWriteRegIss(handle, IMX623_REG_EXPOSURE_SP2_HH,
			(exposure_sp1_sp2_us >> 24) & 0xFF);
	if (result != RET_SUCCESS) {
		TRACE(IMX623_ERROR, "%s: Failed to set exposure time!\n", __func__);
		return RET_FAILURE;
	}

	TRACE(IMX623_INFO, "%s: (exit)\n", __func__);
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
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT IMX623_IsiGetIspStatusIss(IsiSensorHandle_t handle, IsiIspStatus_t *pIspStatus)
{
	RESULT result = RET_SUCCESS;

	pIspStatus->useSensorAE  = false;
	pIspStatus->useSensorBLC = false;
	pIspStatus->useSensorAWB = false;

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
static RESULT IMX623_IsiSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;
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
static RESULT IMX623_IsiGetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;
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
	return result;
}
