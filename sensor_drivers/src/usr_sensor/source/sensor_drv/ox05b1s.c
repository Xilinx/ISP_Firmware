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
 **************************************************************************/
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/misc.h>
#include <oslayer/oslayer.h>
#include <isi/isi_fmc.h>
#include "isi/isi.h"
#include "isi/isi_iss.h"
#include "isi/isi_priv.h"
#include "sensor_drv/ox05b1s_priv.h"

CREATE_TRACER(Ox05b1s_INFO, "Ox05b1s: ", INFO,		1);
CREATE_TRACER(Ox05b1s_WARN, "Ox05b1s: ", WARNING,	1);
CREATE_TRACER(Ox05b1s_ERROR, "Ox05b1s: ", ERROR,	1);
CREATE_TRACER(Ox05b1s_DEBUG, "Ox05b1s: ", INFO,		1);
CREATE_TRACER(Ox05b1s_REG_INFO, "Ox05b1s: ", INFO,	1);
CREATE_TRACER(Ox05b1s_REG_DEBUG, "Ox05b1s: ", INFO,	1);

#define ONE_LINE_EXP_TIME			(0.000027)
#define FRAME_LENGTH_LINES			(0x1d00)
#define MIN_INTEGRATION_LINE			(1)
#define AEC_MAX_GAIN				(230)
#define AEC_MIN_GAIN				(1.0)
#define AGAIN_MIN				(1.0)
#define AGAIN_MAX				(15.5)
#define AGAIN_STEP				(1.0f / 16.0f)
#define DGAIN_MIN				(1.0)
#define DGAIN_MAX				(15)
#define DGAIN_STEP				(1.0f / 1024.0f)
#define MIN_FPS					(1 * ISI_FPS_QUANTIZE)

#define SENSORWB_RGAIN				(1.8)
#define SENSORWB_GBGAIN				(1.0)
#define SENSORWB_GRGAIN				(1.0)
#define SENSORWB_BGAIN				(1.65)

#define IR_POWER_DEVICE_SLAVE_ADDRESS		(0x4a)
#define Ox05b1s_MIN_GAIN_STEP			(1.0f / 1024.0f)
#define OS05B1S_IR_LIGHT_STRENGTH_MAX		(255)

osMutex os05b1sABmodeMutex;

typedef enum {
	OS05B1S_AB_MODE_EXP_LINE = 0,
	OS05B1S_AB_MODE_A_GAIN,
	OS05B1S_AB_MODE_D_GAIN,
	OS05B1S_AB_MODE_IR_PARAMS
} OX05B1S_ABmode_Index_t;

typedef struct {
	bool_t		irOn;
	uint32_t	irStrength;
} OX05B1S_IR_setting_t;

typedef struct {
	uint32_t		expLine;
	uint32_t		again;
	uint32_t		dgain;
	OX05B1S_IR_setting_t	irCfg;
} OX05B1S_ABmode_Setting_t;

OX05B1S_ABmode_Setting_t gOX05B1S_ABmode[2] = {0};

static IsiSensorMode_t pox05b1s_mode_info[] = {
	{
		.index			= 0,
		.size	= {
			.boundsWidth	= 2592,
			.boundsHeight	= 1944,
			.top		= 0,
			.left		= 0,
			.width		= 2592,
			.height		= 1944,
		},
		.aeInfo	= {
			.intTimeDelayFrame	= 2,
			.gainDelayFrame		= 2,
		},
		.fps			= 5 * ISI_FPS_QUANTIZE,
		.hdrMode		= ISI_SENSOR_MODE_LINEAR,
		.bitWidth		= 10,
		.bayerPattern		= ISI_BPAT_BGGIR,
		.afMode			= ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index	= 1,
		.size	= {
			.boundsWidth	= 2592,
			.boundsHeight	= 1944,
			.top		= 0,
			.left		= 0,
			.width		= 2592,
			.height		= 1944,
		},
		.aeInfo	= {
			.intTimeDelayFrame	= 2,
			.gainDelayFrame		= 2,
		},
		.fps			= 5 * ISI_FPS_QUANTIZE,
		.hdrMode		= ISI_SENSOR_MODE_LINEAR,
		.bitWidth		= 10,
		.bayerPattern		= ISI_BPAT_BGGIR,
		.afMode			= ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index			= 2,
		.size	= {
			.boundsWidth	= 2592,
			.boundsHeight	= 1944,
			.top		= 0,
			.left		= 0,
			.width		= 2592,
			.height		= 1944,
		},
		.aeInfo	= {
			.intTimeDelayFrame	= 2,
			.gainDelayFrame		= 2,
		},
		.fps			= 5 * ISI_FPS_QUANTIZE,
		.hdrMode		= ISI_SENSOR_MODE_LINEAR,
		.bitWidth		= 10,
		.bayerPattern		= ISI_BPAT_BGGIR,
		.afMode			= ISI_SENSOR_AF_MODE_NOTSUPP,
	},
};

void enable_IR_power(void)
{
	int Status = XST_SUCCESS;
	uint32_t register_addr = 0x04;
	uint8_t wr_data[2];
	uint8_t read_data[2] = {0};
	uint16_t bytes_read = 1;

	wr_data[0] = 0x0f;

	Status = HalXilWriteI2CReg(IIC_INSTANCE_ZERO, IR_POWER_DEVICE_SLAVE_ADDRESS,
			register_addr, 0x1, wr_data[0], 1);

	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	register_addr = 0x02;
	wr_data[0] = 0xe1;

	Status = HalXilWriteI2CReg(IIC_INSTANCE_ZERO, IR_POWER_DEVICE_SLAVE_ADDRESS,
			register_addr, 0x1, wr_data[0], 1);

	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	register_addr = 0x05;
	wr_data[0] = 0x00;

	Status = HalXilWriteI2CReg(IIC_INSTANCE_ZERO, IR_POWER_DEVICE_SLAVE_ADDRESS,
			register_addr, 0x1, wr_data[0], 1);

	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
}

/*******************************************************************************
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
static RESULT Ox05b1s_IsiReadRegIss(IsiSensorHandle_t handle, const uint16_t addr,
	uint16_t *pValue)
{
	RESULT result = RET_SUCCESS;
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	g_fmc_single.iic_array[pOx05b1sCtx->sensorDevId]->readIIC(pOx05b1sCtx->sensorDevId,
		addr, pValue);
	return result;
}

/*******************************************************************************
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
 *****************************************************************************/
static RESULT Ox05b1s_IsiWriteRegIss(IsiSensorHandle_t handle, const uint16_t addr,
				const uint16_t value)
{
	RESULT result = RET_SUCCESS;
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	g_fmc_single.iic_array[pOx05b1sCtx->sensorDevId]->writeIIC(pOx05b1sCtx->sensorDevId,
		addr, value);
	return result;
}

/*******************************************************************************
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
 *****************************************************************************/
static RESULT Ox05b1s_IsiGetModeIss(IsiSensorHandle_t handle, IsiSensorMode_t *pMode)
{
	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_WRONG_HANDLE;

	if (pMode == NULL)
		return RET_WRONG_HANDLE;

	memcpy(pMode, &(pOx05b1sCtx->sensorMode), sizeof(pOx05b1sCtx->sensorMode));

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

	return RET_SUCCESS;
}

/*******************************************************************************
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
static RESULT Ox05b1s_IsiEnumModeIss(IsiSensorHandle_t handle, IsiSensorEnumMode_t *pEnumMode)
{
	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	if (pEnumMode->index >= ARRAY_SIZE(pox05b1s_mode_info))
		return RET_OUTOFRANGE;

	for (uint32_t i = 0; i < ARRAY_SIZE(pox05b1s_mode_info); i++) {
		if (pox05b1s_mode_info[i].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &pox05b1s_mode_info[i], sizeof(IsiSensorMode_t));

			TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

			return RET_SUCCESS;
		}
	}

	return RET_NOTSUPP;
}

/*******************************************************************************
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
	RESULT result = RET_SUCCESS;
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (pOx05b1sCtx == NULL)
		return RET_WRONG_HANDLE;

	if (pCaps == NULL)
		return RET_NULL_POINTER;

	pCaps->bitWidth			= pOx05b1sCtx->sensorMode.bitWidth;
	pCaps->mode			= ISI_MODE_BAYER;
	pCaps->bayerPattern		= pOx05b1sCtx->sensorMode.bayerPattern;
	pCaps->resolution.width		= pOx05b1sCtx->sensorMode.size.width;
	pCaps->resolution.height	= pOx05b1sCtx->sensorMode.size.height;
	pCaps->mipiLanes		= ISI_MIPI_4LANES;
	pCaps->vinType			= ISI_ITF_TYPE_MIPI;

	if (pCaps->bitWidth == 10)
		pCaps->mipiMode = ISI_FORMAT_RAW_10;
	else if (pCaps->bitWidth == 12)
		pCaps->mipiMode = ISI_FORMAT_RAW_12;
	else
		pCaps->mipiMode = ISI_MIPI_OFF;

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
static RESULT Ox05b1s_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig, IsiSensorHandle_t *pHandle)
{
	RESULT result = RET_SUCCESS;
	uint32_t desId = 0, pipeId = 0;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)osMalloc(sizeof(Ox05b1s_Context_t));

	if (pOx05b1sCtx == NULL) {
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
	pOx05b1sCtx->i2cId		= 0;
	pOx05b1sCtx->sensorDevId	= pConfig->cameraDevId;

	uint8_t busId = (uint8_t)pOx05b1sCtx->i2cId;

	pipeId = pOx05b1sCtx->sensorDevId;

	*pHandle = (IsiSensorHandle_t) pOx05b1sCtx;

	result = init_MUX();

	if (result != XST_SUCCESS) {
		xil_printf("\n\rIIC Init Failed\n\r");
		return result;
	}

	desId = MAPPING_INPIPE_TO_DES_ID(pipeId);
	static int8_t mcmABmode_initCount;

	if (mcmABmode_initCount <= 0) {
		init_des(desId);
		init_sensor(pipeId, desId);
	}

	init_iic_access(pipeId, desId);

	mcmABmode_initCount++;

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

	return result;
}

static RESULT Ox05b1s_AecSetModeParameters(IsiSensorHandle_t handle,
		Ox05b1s_Context_t *pOx05b1sCtx)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s%s: (enter)\n", __func__, pOx05b1sCtx->isAfpsRun ? "(AFPS)" : "");

	uint32_t exp_line = 0, again = 0, dgain = 0, irLine;
	uint16_t value = 0;

	pOx05b1sCtx->aecMinIntegrationTime = pOx05b1sCtx->oneLineExpTime *
						pOx05b1sCtx->minIntegrationLine;
	pOx05b1sCtx->aecMaxIntegrationTime = pOx05b1sCtx->oneLineExpTime *
						pOx05b1sCtx->maxIntegrationLine;

	TRACE(Ox05b1s_DEBUG, "%s: AecMaxIntegrationTime = %f\n", __func__,
		pOx05b1sCtx->aecMaxIntegrationTime);

	pOx05b1sCtx->aecGainIncrement = Ox05b1s_MIN_GAIN_STEP;
	pOx05b1sCtx->aecIntegrationTimeIncrement = pOx05b1sCtx->oneLineExpTime;

	pOx05b1sCtx->irLightInfo.irRangeInfo.minIrStrength	= 1;
	pOx05b1sCtx->irLightInfo.irRangeInfo.maxIrStrength	= OS05B1S_IR_LIGHT_STRENGTH_MAX;
	pOx05b1sCtx->irLightInfo.irRangeInfo.irStrengthStep	= 1;
	pOx05b1sCtx->irLightInfo.irDelayFrame			= 0;

	if (pOx05b1sCtx->sensorMode.index == 2)
		pOx05b1sCtx->irLightInfo.irSuppAeCtrl = 1;
	else
		pOx05b1sCtx->irLightInfo.irSuppAeCtrl = 0;

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

	pOx05b1sCtx->aecCurGain = ((float)again / 16.0) * ((float)dgain / 1024.0);

	Ox05b1s_IsiReadRegIss(handle, 0x3500, &value);
	exp_line = (value & 0xff) << 16;
	Ox05b1s_IsiReadRegIss(handle, 0x3501, &value);
	exp_line = exp_line | ((value & 0xff) << 8);
	Ox05b1s_IsiReadRegIss(handle, 0x3502, &value);
	exp_line = exp_line | (value & 0xff);

	pOx05b1sCtx->aecCurIntegrationTime = exp_line * pOx05b1sCtx->oneLineExpTime;
	value = 0;

	result |= Ox05b1s_IsiReadRegIss(handle, 0x3b20, &value);

	if (value != 0)
		pOx05b1sCtx->irLightExp.irOn = BOOL_TRUE;
	else
		pOx05b1sCtx->irLightExp.irOn = BOOL_FALSE;

	result |= Ox05b1s_IsiReadRegIss(handle, 0x3b25, &value);
	irLine = (value & 0xff) << 24;
	result |= Ox05b1s_IsiReadRegIss(handle, 0x3b26, &value);
	irLine |= (value & 0xff) << 16;
	result |= Ox05b1s_IsiReadRegIss(handle, 0x3b27, &value);
	irLine |= (value & 0xff) << 8;
	result |= Ox05b1s_IsiReadRegIss(handle, 0x3b28, &value);
	irLine |= value & 0xff;

	pOx05b1sCtx->irLightExp.irStrength =
		MAX(MIN(irLine,
			pOx05b1sCtx->irLightInfo.irRangeInfo.maxIrStrength),
			pOx05b1sCtx->irLightInfo.irRangeInfo.minIrStrength);

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pOx05b1sCtx->streaming != BOOL_FALSE)
		return RET_WRONG_STATE;

	pOx05b1sCtx->sensorMode.index		= mode;
	IsiSensorMode_t *SensorDefaultMode	= NULL;

	for (int i = 0; i < ARRAY_SIZE(pox05b1s_mode_info); i++) {
		if (pox05b1s_mode_info[i].index == pOx05b1sCtx->sensorMode.index) {
			SensorDefaultMode = &(pox05b1s_mode_info[i]);
			break;
		}
	}

	if (pOx05b1sCtx->sensorMode.index == 1) {
		int32_t osRet = OSLAYER_OK;

		osRet = osMutexInit(&os05b1sABmodeMutex);

		if (osRet != OSLAYER_OK)
			return RET_FAILURE;
	}

	if (SensorDefaultMode != NULL) {
		int Status = XST_SUCCESS;

		switch (SensorDefaultMode->index) {
		case 0:
			for (int i = 0; i < ARRAY_SIZE(Ox05b1s_mipi4lane_linear_init); i++) {
				if (Ox05b1s_mipi4lane_linear_init[i][0] == OX05B1S_TABLE_WAIT)
					vTaskDelay(Ox05b1s_mipi4lane_linear_init[i][1]);
				else if (Ox05b1s_mipi4lane_linear_init[i][0] == OX05B1S_TABLE_END)
					break;
				else {
					g_fmc_single.iic_array[pOx05b1sCtx->sensorDevId]->writeIIC(
							pOx05b1sCtx->sensorDevId,
							Ox05b1s_mipi4lane_linear_init[i][0],
							Ox05b1s_mipi4lane_linear_init[i][1]);
				}
			}
			break;
		case 1:
			for (int i = 0; i < ARRAY__SIZE(Ox05b1s_mipi4lane_ABmode_init); i++) {
				if (Ox05b1s_mipi4lane_ABmode_init[i][0] ==
						OX05B1S_TABLE_WAIT) {
					vTaskDelay(Ox05b1s_mipi4lane_ABmode_init[i][1]);
				} else if (Ox05b1s_mipi4lane_ABmode_init[i][0] ==
						OX05B1S_TABLE_END) {
					break;
				} else {
					g_fmc_single.iic_array[pOx05b1sCtx->sensorDevId]->writeIIC(
						pOx05b1sCtx->sensorDevId,
						Ox05b1s_mipi4lane_ABmode_init[i][0],
						Ox05b1s_mipi4lane_ABmode_init[i][1]);
				}
			}
			break;
		case 2:
			for (int i = 0; i < ARRAY_SIZE(Ox05b1s_mipi4lane_ABmode_IRFRAME_init);
					i++) {
				if (Ox05b1s_mipi4lane_ABmode_IRFRAME_init[i][0] ==
						OX05B1S_TABLE_WAIT) {
					vTaskDelay(Ox05b1s_mipi4lane_ABmode_IRFRAME_init[i][1]);
				} else if (Ox05b1s_mipi4lane_ABmode_IRFRAME_init[i][0] ==
						OX05B1S_TABLE_END) {
					break;
				} else {
					g_fmc_single.iic_array[pOx05b1sCtx->sensorDevId]->writeIIC(
							pOx05b1sCtx->sensorDevId,
							Ox05b1s_mipi4lane_ABmode_IRFRAME_init[i][0],
							Ox05b1s_mipi4lane_ABmode_IRFRAME_init[i][1]);
				}
			}
			break;
		default:
			TRACE(Ox05b1s_INFO, "%s:not support sensor mode %d\n", __func__,
				pOx05b1sCtx->sensorMode.index);
			osFree(pOx05b1sCtx);
			return RET_NOTSUPP;
		}

		memcpy(&(pOx05b1sCtx->sensorMode), SensorDefaultMode, sizeof(IsiSensorMode_t));
	} else {
		TRACE(Ox05b1s_ERROR, "%s: Invalid SensorDefaultMode\n", __func__);
		return RET_NULL_POINTER;
	}

	switch (pOx05b1sCtx->sensorMode.index) {
	case 0:
		pOx05b1sCtx->oneLineExpTime		= ONE_LINE_EXP_TIME;
		pOx05b1sCtx->frameLengthLines		= FRAME_LENGTH_LINES;
		pOx05b1sCtx->curFrameLengthLines	= pOx05b1sCtx->frameLengthLines;
		pOx05b1sCtx->maxIntegrationLine		= pOx05b1sCtx->frameLengthLines - 30;
		pOx05b1sCtx->minIntegrationLine		= MIN_INTEGRATION_LINE;
		pOx05b1sCtx->aecMaxGain			= AEC_MAX_GAIN;
		pOx05b1sCtx->aecMinGain			= AEC_MIN_GAIN;
		pOx05b1sCtx->aGain.min			= AGAIN_MIN;
		pOx05b1sCtx->aGain.max			= AGAIN_MAX;
		pOx05b1sCtx->aGain.step			= AGAIN_STEP;
		pOx05b1sCtx->dGain.min			= DGAIN_MIN;
		pOx05b1sCtx->dGain.max			= DGAIN_MAX;
		pOx05b1sCtx->dGain.step			= DGAIN_STEP;
		break;
	case 1:
	case 2:
		pOx05b1sCtx->oneLineExpTime		= ONE_LINE_EXP_TIME;
		pOx05b1sCtx->frameLengthLines		= FRAME_LENGTH_LINES;
		pOx05b1sCtx->curFrameLengthLines	= pOx05b1sCtx->frameLengthLines;
		pOx05b1sCtx->maxIntegrationLine		= pOx05b1sCtx->frameLengthLines - 30;
		pOx05b1sCtx->minIntegrationLine		= MIN_INTEGRATION_LINE;
		pOx05b1sCtx->aecMaxGain			= AEC_MAX_GAIN;
		pOx05b1sCtx->aecMinGain			= AEC_MIN_GAIN;
		pOx05b1sCtx->aGain.min			= AGAIN_MIN;
		pOx05b1sCtx->aGain.max			= AGAIN_MAX;
		pOx05b1sCtx->aGain.step			= AGAIN_STEP;
		pOx05b1sCtx->dGain.min			= DGAIN_MIN;
		pOx05b1sCtx->dGain.max			= DGAIN_MAX;
		pOx05b1sCtx->dGain.step			= DGAIN_STEP;
		break;
	default:
		TRACE(Ox05b1s_INFO, "%s:not support sensor mode %d\n", __func__,
			pOx05b1sCtx->sensorMode.index);
		return RET_NOTSUPP;
	}

	pOx05b1sCtx->maxFps		= pOx05b1sCtx->sensorMode.fps;
	pOx05b1sCtx->minFps		= MIN_FPS;
	pOx05b1sCtx->currFps		= pOx05b1sCtx->maxFps;

	pOx05b1sCtx->sensorWb.rGain	= SENSORWB_RGAIN;
	pOx05b1sCtx->sensorWb.gbGain	= SENSORWB_GBGAIN;
	pOx05b1sCtx->sensorWb.grGain	= SENSORWB_GRGAIN;
	pOx05b1sCtx->sensorWb.bGain	= SENSORWB_BGAIN;

	TRACE(Ox05b1s_DEBUG, "%s: Ox05b1s System-Reset executed\n", __func__);

	osSleep(100);

	result = Ox05b1s_AecSetModeParameters(handle, pOx05b1sCtx);

	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
		return result;
	}

	pOx05b1sCtx->configured = BOOL_TRUE;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return 0;
}

/*******************************************************************************
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

	if (pOx05b1sCtx == NULL)
		return RET_WRONG_HANDLE;

	(void)Ox05b1s_IsiSetStreamingIss(pOx05b1sCtx, BOOL_FALSE);

	if (pOx05b1sCtx->sensorMode.index == 2) {
		int32_t osRet = OSLAYER_OK;

		osRet = osMutexDestroy(&os05b1sABmodeMutex);

		if (osRet != OSLAYER_OK)
			return RET_FAILURE;
	}

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

	return result;
}

static RESULT Ox05b1s_IsiReleaseIss(IsiSensorHandle_t handle)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	if (pOx05b1sCtx == NULL)
		return RET_WRONG_HANDLE;

	MEMSET(pOx05b1sCtx, 0, sizeof(Ox05b1s_Context_t));
	osFree(pOx05b1sCtx);

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
	uint32_t correct_id = 0x5805;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	result = Ox05b1s_IsiGetRevisionIss(handle, &sensor_id);

	if (result != RET_SUCCESS) {
		TRACE(Ox05b1s_ERROR, "%s: Read Sensor ID Error!\n", __func__);
		return RET_FAILURE;
	}

	if (correct_id != sensor_id) {
		TRACE(Ox05b1s_ERROR, "%s:ChipID =0x%x sensor_id=%x error!\n", __func__,
			correct_id, sensor_id);
		return RET_FAILURE;
	}

	TRACE(Ox05b1s_INFO, "%s ChipID = 0x%08x, sensor_id = 0x%08x, success!\n", __func__,
		correct_id, sensor_id);

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
static RESULT Ox05b1s_IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val;
	uint32_t sensor_id;

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	reg_val = 0;
	result = Ox05b1s_IsiReadRegIss(handle, 0x300a, &reg_val);

	sensor_id = (reg_val & 0xff) << 8;

	reg_val = 0;
	result |= Ox05b1s_IsiReadRegIss(handle, 0x300b, &reg_val);

	sensor_id |= (reg_val & 0xff);
	*pValue = sensor_id;

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

	return result;
}

static RESULT Ox05b1s_IsiSetABmodeGroup(IsiSensorHandle_t handle, const uint8_t mode,
		OX05B1S_ABmode_Setting_t setting)
{
	RESULT result = RET_SUCCESS;
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	int32_t osRet = OSLAYER_OK;

	TRACE(Ox05b1s_INFO, "%s: mode: %d (enter)\n", __func__, pOx05b1sCtx->sensorMode.index);
	osRet = osMutexLock(&os05b1sABmodeMutex);

	if (osRet != OSLAYER_OK)
		return RET_FAILURE;

	if (pOx05b1sCtx->sensorMode.index == 1) {
		if ((mode & 0x1) != 0)
			gOX05B1S_ABmode[0].expLine = setting.expLine;

		if ((mode & 0x2) != 0)
			gOX05B1S_ABmode[0].again = setting.again;

		if ((mode & 0x4) != 0)
			gOX05B1S_ABmode[0].dgain = setting.dgain;

		if ((mode & 0x8) != 0) {
			gOX05B1S_ABmode[0].irCfg.irOn = setting.irCfg.irOn;
			gOX05B1S_ABmode[0].irCfg.irStrength = setting.irCfg.irStrength;
		}
	} else if (pOx05b1sCtx->sensorMode.index == 2) {
		if ((mode & 0x1) != 0)
			gOX05B1S_ABmode[1].expLine = setting.expLine;

		if ((mode & 0x2) != 0)
			gOX05B1S_ABmode[1].again = setting.again;

		if ((mode & 0x4) != 0)
			gOX05B1S_ABmode[1].dgain = setting.dgain;

		if ((mode & 0x8) != 0) {
			gOX05B1S_ABmode[1].irCfg.irOn = setting.irCfg.irOn;
			gOX05B1S_ABmode[1].irCfg.irStrength = setting.irCfg.irStrength;
		}
	} else {
		return RET_UNSUPPORT_ID;
	}

	result = Ox05b1s_IsiWriteRegIss(handle, 0x320a, 0x01);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x320b, 0x01);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x320c, 0x00);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x320d, 0x00);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x00);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x431c, 0x00);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x4813, 0x01);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b20, 0x00);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3500, (gOX05B1S_ABmode[0].expLine >> 16) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3501, (gOX05B1S_ABmode[0].expLine >> 8) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3502, (gOX05B1S_ABmode[0].expLine & 0xff));
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3508, (gOX05B1S_ABmode[0].again >> 4) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3509, (gOX05B1S_ABmode[0].again & 0x0f) << 4);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350a, (gOX05B1S_ABmode[0].dgain >> 10) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350b, (gOX05B1S_ABmode[0].dgain >> 2) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350c, (gOX05B1S_ABmode[0].dgain & 0x03) << 6);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x10);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x01);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x431c, 0x09);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x4813, 0x04);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3500, (gOX05B1S_ABmode[1].expLine >> 16) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3501, (gOX05B1S_ABmode[1].expLine >> 8) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3502, (gOX05B1S_ABmode[1].expLine & 0xff));
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3508, (gOX05B1S_ABmode[1].again >> 4) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3509, (gOX05B1S_ABmode[1].again & 0x0f) << 4);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350a, (gOX05B1S_ABmode[1].dgain >> 10) & 0x0f);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350b, (gOX05B1S_ABmode[1].dgain >> 2) & 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x350c, (gOX05B1S_ABmode[1].dgain & 0x03) << 6);

	if (gOX05B1S_ABmode[1].irCfg.irOn == BOOL_TRUE)
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b20, 0xff);
	else
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b20, 0x00);

	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b1e, 0x00);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b25, (gOX05B1S_ABmode[1].irCfg.irStrength >> 24)
			& 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b26, (gOX05B1S_ABmode[1].irCfg.irStrength >> 16)
			& 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b27, (gOX05B1S_ABmode[1].irCfg.irStrength >> 8)
			& 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b28, gOX05B1S_ABmode[1].irCfg.irStrength
			& 0xff);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b2f, 0x4a);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0x11);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3211, 0x30);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x3208, 0xa0);

	osRet = osMutexUnlock(&os05b1sABmodeMutex);

	if (osRet != OSLAYER_OK)
		return RET_FAILURE;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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

	TRACE(Ox05b1s_INFO, "%s (enter)\n", __func__);
	TRACE(Ox05b1s_INFO, "%s Enabling IR Power...!\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	if (pOx05b1sCtx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	if (pOx05b1sCtx->sensorMode.index != 2) {
		enable_IR_power();
		result = Ox05b1s_IsiWriteRegIss(handle, 0x0100, on);

		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: set sensor streaming error!\n", __func__);
			return RET_FAILURE;
		}
	}

	if (pOx05b1sCtx->sensorMode.index == 1) {
		uint8_t mode = 0;

		OX05B1S_ABmode_Setting_t setting = {0, 0, 0, {0, 0}};

		gOX05B1S_ABmode[0].expLine		= 0x50;
		gOX05B1S_ABmode[0].again		= 0x00;
		gOX05B1S_ABmode[0].dgain		= 0x400;
		gOX05B1S_ABmode[0].irCfg.irOn		= BOOL_FALSE;

		memcpy(&setting, gOX05B1S_ABmode, sizeof(OX05B1S_ABmode_Setting_t));

		gOX05B1S_ABmode[1].expLine		= 0x50;
		gOX05B1S_ABmode[1].again		= 0x00;
		gOX05B1S_ABmode[1].dgain		= 0x400;
		gOX05B1S_ABmode[1].irCfg.irOn		= BOOL_TRUE;
		gOX05B1S_ABmode[1].irCfg.irStrength	= 0x10;

		mode = ((1 << OS05B1S_AB_MODE_EXP_LINE) | (1 << OS05B1S_AB_MODE_A_GAIN) |
			(1 << OS05B1S_AB_MODE_D_GAIN) | (1 << OS05B1S_AB_MODE_IR_PARAMS));

		result = Ox05b1s_IsiSetABmodeGroup(handle, mode, setting);

		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: set init Ox05b1s_IsiSetABmodeGroup error! %d\n",
				__func__, result);
			return result;
		}

		pOx05b1sCtx->irLightExp.irOn = BOOL_FALSE;
		pOx05b1sCtx->irLightExp.irStrength = 0;

	} else if (pOx05b1sCtx->sensorMode.index == 2) {
		pOx05b1sCtx->irLightExp.irOn = BOOL_TRUE;
		pOx05b1sCtx->irLightExp.irStrength = 0x10;
	}

	pOx05b1sCtx->streaming = on;

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
static RESULT Ox05b1s_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle, IsiAeBaseInfo_t *pAeBaseInfo)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle\n", __func__);
		return RET_WRONG_HANDLE;
	}

	if (pAeBaseInfo == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: NULL pointer received!!\n");
		return RET_NULL_POINTER;
	}

	pAeBaseInfo->gain.min		= pOx05b1sCtx->aecMinGain;
	pAeBaseInfo->gain.max		= pOx05b1sCtx->aecMaxGain;
	pAeBaseInfo->intTime.min	= pOx05b1sCtx->aecMinIntegrationTime;
	pAeBaseInfo->intTime.max	= pOx05b1sCtx->aecMaxIntegrationTime;
	pAeBaseInfo->aGain		= pOx05b1sCtx->aGain;
	pAeBaseInfo->dGain		= pOx05b1sCtx->dGain;
	pAeBaseInfo->aecCurGain		= pOx05b1sCtx->aecCurGain;
	pAeBaseInfo->aecCurIntTime	= pOx05b1sCtx->aecCurIntegrationTime;
	pAeBaseInfo->aecGainStep	= pOx05b1sCtx->aecGainIncrement;
	pAeBaseInfo->aecIntTimeStep	= pOx05b1sCtx->aecIntegrationTimeIncrement;
	pAeBaseInfo->aecIrLightExp	= pOx05b1sCtx->irLightExp;
	pAeBaseInfo->aecIrLightInfo	= pOx05b1sCtx->irLightInfo;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	return result;
}

/*******************************************************************************
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
RESULT Ox05b1s_IsiSetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	uint32_t again = 0;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] < pOx05b1sCtx->aGain.min) {
		TRACE(Ox05b1s_WARN, "%s: invalid too small again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->aGain.min;
	}

	if (pSensorAGain->gain[ISI_LINEAR_PARAS] > pOx05b1sCtx->aGain.max) {
		TRACE(Ox05b1s_WARN, "%s: invalid too big again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->aGain.max;
	}

	again = (uint32_t)(pSensorAGain->gain[ISI_LINEAR_PARAS] * 16);

	TRACE(Ox05b1s_DEBUG, "%s: in mode %d again %d\n", __func__,
		pOx05b1sCtx->sensorMode.index, again);

	if (pOx05b1sCtx->sensorMode.index == 1 || pOx05b1sCtx->sensorMode.index == 2) {
		uint8_t mode = 0;
		OX05B1S_ABmode_Setting_t setting = {0, 0, 0, {0, 0}};

		mode = (1 << OS05B1S_AB_MODE_A_GAIN);
		setting.again = again;
		result = Ox05b1s_IsiSetABmodeGroup(handle, mode, setting);

		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: set Ox05b1s_IsiSetABmodeGroup error! %d\n",
				__func__, result);
			return result;
		}
	} else {
		result = Ox05b1s_IsiWriteRegIss(handle, 0x3508, (again >> 4) & 0x0f);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3509, (again & 0x0f) << 4);
	}

	pOx05b1sCtx->curAgain = (float)again / 16.0f;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
RESULT Ox05b1s_IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	uint32_t dgain = 0;

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] < pOx05b1sCtx->dGain.min) {
		TRACE(Ox05b1s_WARN, "%s: invalid too small dgain parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->dGain.min;
	}

	if (pSensorDGain->gain[ISI_LINEAR_PARAS] > pOx05b1sCtx->dGain.max) {
		TRACE(Ox05b1s_WARN, "%s: invalid too big dgain parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->dGain.max;
	}

	dgain = (uint32_t)(pSensorDGain->gain[ISI_LINEAR_PARAS] * 1024);

	TRACE(Ox05b1s_DEBUG, "%s: in mode %d, dgain %d\n", __func__, pOx05b1sCtx->sensorMode.index,
		dgain);

	if (pOx05b1sCtx->sensorMode.index == 1 || pOx05b1sCtx->sensorMode.index == 2) {
		uint8_t mode = 0;
		OX05B1S_ABmode_Setting_t setting = {0, 0, 0, {0, 0}};

		mode = (1 << OS05B1S_AB_MODE_D_GAIN);
		setting.dgain = dgain;
		result = Ox05b1s_IsiSetABmodeGroup(handle, mode, setting);

		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: set Ox05b1s_IsiSetABmodeGroup error! %d\n",
				__func__, result);
			return result;
		}
	} else {
		result = Ox05b1s_IsiWriteRegIss(handle, 0x350a, (dgain >> 10) & 0x0f);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x350b, (dgain >> 2) & 0xff);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x350c, (dgain & 0x03) << 6);
	}

	pOx05b1sCtx->curDgain = (float)dgain / 1024.0f;
	pOx05b1sCtx->aecCurGain = pOx05b1sCtx->curAgain * pOx05b1sCtx->curDgain;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
RESULT Ox05b1s_IsiGetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle\n", __func__);
		return RET_WRONG_HANDLE;
	}

	if (pSensorAGain == NULL)
		return RET_NULL_POINTER;

	pSensorAGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->curAgain;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
RESULT Ox05b1s_IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle\n", __func__);
		return RET_WRONG_HANDLE;
	}

	if (pSensorDGain == NULL)
		return RET_NULL_POINTER;

	pSensorDGain->gain[ISI_LINEAR_PARAS] = pOx05b1sCtx->curDgain;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
RESULT Ox05b1s_IsiSetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	uint32_t expLine = 0;

	expLine = pSensorIntTime->intTime[ISI_LINEAR_PARAS] / pOx05b1sCtx->oneLineExpTime;
	expLine = MIN(pOx05b1sCtx->maxIntegrationLine, MAX(pOx05b1sCtx->minIntegrationLine,
			expLine));

	TRACE(Ox05b1s_DEBUG, "%s: in mode %d, set expLine = 0x%04x\n", __func__,
		pOx05b1sCtx->sensorMode.index, expLine);

	if (pOx05b1sCtx->sensorMode.index == 1 || pOx05b1sCtx->sensorMode.index == 2) {
		uint8_t mode = 0;
		OX05B1S_ABmode_Setting_t setting = {0, 0, 0, {0, 0}};

		mode = (1 << OS05B1S_AB_MODE_EXP_LINE);
		setting.expLine = expLine;
		result = Ox05b1s_IsiSetABmodeGroup(handle, mode, setting);

		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: set Ox05b1s_IsiSetABmodeGroup error! %d\n",
				__func__, result);
			return result;
		}
	} else {
		result = Ox05b1s_IsiWriteRegIss(handle, 0x3500, (expLine >> 16) & 0xff);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3501, (expLine >> 8) & 0xff);
		result |= Ox05b1s_IsiWriteRegIss(handle, 0x3502, (expLine & 0xff));
	}

	pOx05b1sCtx->aecCurIntegrationTime = expLine * pOx05b1sCtx->oneLineExpTime;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
RESULT Ox05b1s_IsiGetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pSensorIntTime == NULL)
		return RET_NULL_POINTER;

	pSensorIntTime->intTime[ISI_LINEAR_PARAS] = pOx05b1sCtx->aecCurIntegrationTime;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

RESULT Ox05b1s_IsiSetIRLightExpIss(IsiSensorHandle_t handle, const IsiIrLightExp_t *pIrExpParam)
{
	RESULT result = RET_SUCCESS;
	uint32_t irStrobeLine = 0, irStrobeShift = 0, expLine = 0;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pIrExpParam == NULL)
		return RET_NULL_POINTER;

	if (pIrExpParam->irOn == BOOL_TRUE) {
		irStrobeLine = MAX(MIN(pIrExpParam->irStrength,
				pOx05b1sCtx->irLightInfo.irRangeInfo.maxIrStrength),
				pOx05b1sCtx->irLightInfo.irRangeInfo.maxIrStrength);
	} else {
		irStrobeLine = pOx05b1sCtx->irLightExp.irStrength;
	}

	TRACE(Ox05b1s_DEBUG, "%s: in mode %d, set irStrobeLine = 0x%04x\n", __func__,
		pOx05b1sCtx->sensorMode.index, irStrobeLine);

	if (pOx05b1sCtx->sensorMode.index == 1 || pOx05b1sCtx->sensorMode.index == 2) {
		uint8_t mode = 0;
		OX05B1S_ABmode_Setting_t setting = {0, 0, 0, {0, 0}};

		mode = (1 << OS05B1S_AB_MODE_IR_PARAMS);
		setting.irCfg.irOn = pIrExpParam->irOn;
		setting.irCfg.irStrength = irStrobeLine;
		result = Ox05b1s_IsiSetABmodeGroup(handle, mode, setting);

		if (result != RET_SUCCESS) {
			TRACE(Ox05b1s_ERROR, "%s: set Ox05b1s_IsiSetABmodeGroup error! %d\n",
				__func__, result);
			return result;
		}
	} else {
		if (pIrExpParam->irOn == BOOL_TRUE) {
			result = Ox05b1s_IsiWriteRegIss(handle, 0x3b20, 0xff);
			result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b1e, 0);
			result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b25, (irStrobeLine >> 24) &&
					0xff);
			result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b26, (irStrobeLine >> 16) &&
					0xff);
			result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b27, (irStrobeLine >> 8) &&
					0xff);
			result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b28, irStrobeLine & 0xff);
			result |= Ox05b1s_IsiWriteRegIss(handle, 0x3b2f, 0x4a);
		} else {
			result = Ox05b1s_IsiWriteRegIss(handle, 0x3b20, 0);
		}
	}

	pOx05b1sCtx->irLightExp.irOn = pIrExpParam->irOn;
	pOx05b1sCtx->irLightExp.irOn = pIrExpParam->irOn;
	pOx05b1sCtx->irLightExp.irStrength = irStrobeLine;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

RESULT Ox05b1s_IsiGetIRLightExpIss(IsiSensorHandle_t handle, IsiIrLightExp_t *pIrExpParam)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	if (pOx05b1sCtx == NULL) {
		TRACE(Ox05b1s_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n",
			__func__);
		return RET_WRONG_HANDLE;
	}

	if (pIrExpParam == NULL)
		return RET_NULL_POINTER;

	pIrExpParam->irOn = pOx05b1sCtx->irLightExp.irOn;
	pIrExpParam->irStrength = pOx05b1sCtx->irLightExp.irStrength;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

void sensor_framecount_ox05b1s(IsiSensorHandle_t handle)
{
	u32 frame_counter;
	u8 read_buf[4];
	int Status;

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4613, &read_buf[0]);

	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4612, &read_buf[1]);

	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4611, &read_buf[2]);

	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	Status = Ox05b1s_IsiReadRegIss(handle, 0x4610, &read_buf[3]);

	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);

	frame_counter = (read_buf[3] << 24) | (read_buf[2] << 16) | (read_buf[1] << 8) |
		(read_buf[0]);

	g_Sensor_frame_count = frame_counter;
}

/*******************************************************************************
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

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*******************************************************************************
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
		TRACE(Ox05b1s_ERROR, "%s: set fps(%d) out of range, correct to %d (%d, %d)\n",
			__func__, fps, pOx05b1sCtx->maxFps, pOx05b1sCtx->minFps,
			pOx05b1sCtx->maxFps);
		fps = pOx05b1sCtx->maxFps;
	}

	if (fps < pOx05b1sCtx->minFps) {
		TRACE(Ox05b1s_ERROR, "%s: set fps(%d) out of range, correct to %d (%d, %d)\n",
			__func__, fps, pOx05b1sCtx->minFps, pOx05b1sCtx->minFps,
			pOx05b1sCtx->maxFps);
		fps = pOx05b1sCtx->minFps;
	}

	NewVts = pOx05b1sCtx->frameLengthLines * (pOx05b1sCtx->sensorMode.fps) / fps;
	result = Ox05b1s_IsiWriteRegIss(handle, 0x380e, NewVts >> 8);
	result |= Ox05b1s_IsiWriteRegIss(handle, 0x380f, NewVts & 0xff);

	pOx05b1sCtx->currFps			= fps;
	pOx05b1sCtx->curFrameLengthLines	= NewVts;
	pOx05b1sCtx->maxIntegrationLine		= pOx05b1sCtx->curFrameLengthLines - 30;
	pOx05b1sCtx->aecMaxIntegrationTime	=
				pOx05b1sCtx->maxIntegrationLine * pOx05b1sCtx->oneLineExpTime;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);

	return result;
}

/*******************************************************************************
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
RESULT Ox05b1s_IsiGetIspStatusIss(IsiSensorHandle_t handle, IsiIspStatus_t *pIspStatus)
{
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_WRONG_HANDLE;

	TRACE(Ox05b1s_INFO, "%s: (enter)\n", __func__);

	pIspStatus->useSensorAE = false;
	pIspStatus->useSensorBLC = false;
	pIspStatus->useSensorAWB = false;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/*******************************************************************************
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

	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL)
		return RET_NULL_POINTER;

	if (pOx05b1sCtx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	if (tpg.enable == 0)
		result = Ox05b1s_IsiWriteRegIss(handle, 0x5100, 0x00);
	else
		result = Ox05b1s_IsiWriteRegIss(handle, 0x5100, 0x80);

	pOx05b1sCtx->testPattern = tpg.enable;

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*******************************************************************************
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
	Ox05b1s_Context_t *pOx05b1sCtx = (Ox05b1s_Context_t *)handle;

	if (pOx05b1sCtx == NULL || pTpg == NULL)
		return RET_NULL_POINTER;

	if (pOx05b1sCtx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	if (!Ox05b1s_IsiReadRegIss(handle, 0x5100, &value)) {
		pTpg->enable = ((value & 0x80) != 0) ? 1 : 0;
		if (pTpg->enable)
			pTpg->pattern = (0xff & value);
		pOx05b1sCtx->testPattern = pTpg->enable;
	}

	TRACE(Ox05b1s_INFO, "%s: (exit)\n", __func__);
	return result;
}

/*******************************************************************************
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
		pIsiSensor->pszName			= SensorName;
		pIsiSensor->pIsiCreateIss		= Ox05b1s_IsiCreateIss;
		pIsiSensor->pIsiOpenIss			= Ox05b1s_IsiOpenIss;
		pIsiSensor->pIsiCloseIss		= Ox05b1s_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss		= Ox05b1s_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss		= Ox05b1s_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss		= Ox05b1s_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss		= Ox05b1s_IsiGetModeIss;
		pIsiSensor->pIsiEnumModeIss		= Ox05b1s_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss		= Ox05b1s_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss	= Ox05b1s_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss		= Ox05b1s_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss		= Ox05b1s_IsiSetStreamingIss;
		pIsiSensor->pIsiGetAeBaseInfoIss	= Ox05b1s_pIsiGetAeBaseInfoIss;
		pIsiSensor->pIsiGetAGainIss		= Ox05b1s_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss		= Ox05b1s_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss		= Ox05b1s_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss		= Ox05b1s_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss		= Ox05b1s_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss		= Ox05b1s_IsiSetIntTimeIss;
		pIsiSensor->pIsiGetFpsIss		= Ox05b1s_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss		= Ox05b1s_IsiSetFpsIss;
		pIsiSensor->pIsiGetIspStatusIss		= Ox05b1s_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss		= NULL;
		pIsiSensor->pIsiGetWBIss		= NULL;
		pIsiSensor->pIsiSetBlcIss		= NULL;
		pIsiSensor->pIsiGetBlcIss		= NULL;
		pIsiSensor->pIsiSetTpgIss		= Ox05b1s_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss		= Ox05b1s_IsiGetTpgIss;
		pIsiSensor->pIsiGetExpandCurveIss	= NULL;
		pIsiSensor->pIsiSetIRLightExpIss	= Ox05b1s_IsiSetIRLightExpIss;
		pIsiSensor->pIsiGetIRLightExpIss	= Ox05b1s_IsiGetIRLightExpIss;
	} else {
		result = RET_NULL_POINTER;
	}

	TRACE(Ox05b1s_INFO, "%s (exit)\n", __func__);
	return result;
}

IsiCamDrvConfig_t Ox05b1s_IsiCamDrvConfig = {
	.cameraDriverID		= 0x5805,
	.pIsiGetSensorIss	= Ox05b1s_IsiGetSensorIss,
};
