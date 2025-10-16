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
#ifndef __SEMU_PRIV_H__
#define __SEMU_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include "isi/isi_priv.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
	IsiSensorContext_t		isiCtx;
	IsiSensorMode_t			sensorMode;

	uint32_t			maxFps;
	uint32_t			minFps;
	uint32_t			currFps;

	bool_t				configured;
	bool_t				streaming;
	bool_t				testPattern;
	bool_t				isAfpsRun;

	float				oneLineDCGExpTime;
	float				oneLineSPDExpTime;
	float				oneLineVSExpTime;
	uint16_t			maxDCGIntegrationLine;
	uint16_t			minDCGIntegrationLine;
	uint16_t			maxSPDIntegrationLine;
	uint16_t			minSPDIntegrationLine;
	uint16_t			maxVSIntegrationLine;
	uint16_t			minVSIntegrationLine;

	uint16_t			frameLengthLines;
	uint16_t			curFrameLengthLines;

	float				aecMinGain;
	float				aecMaxGain;
	float				aecMinIntegrationTime;
	float				aecMaxIntegrationTime;

	float				aecIntegrationTimeIncrement;
	float				aecGainIncrement;

	IsiSensorGain_t			curGain;
	IsiSensorGain_t			curAgain;
	IsiSensorGain_t			curDgain;
	IsiSensorIntTime_t		curIntTime;

	bool				groupHold;
	uint32_t			oldGain;
	uint32_t			oldIntegrationTime;

	IsiGainInfo_t			aGain;
	IsiGainInfo_t			aVSGain;
	IsiGainInfo_t			dGain;
	IsiGainInfo_t			dVSGain;

	IsiSensorBlc_t			sensorBlc;
	IsiSensorWb_t			sensorWb;

	uint32_t			sensorDevId;
} Semu_Context_t;

/*****************************************************************************/
/**
 *          Semu_IsiCreateIss
 *
 * @brief   Create sensor emulation instance.
 *
 * @param   pConfig   pointer to sensor instance configuration
 * @param   pHandle   pointer to sensor handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig,
					IsiSensorHandle_t *pHandle);

/*****************************************************************************/
/**
 *          Semu_IsiOpenIss
 *
 * @brief   Open sensor emulation instance with specified mode.
 *
 * @param   handle      sensor instance handle
 * @param   mode        sensor operation mode
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode);

/*****************************************************************************/
/**
 *          Semu_IsiCloseIss
 *
 * @brief   Close sensor emulation instance.
 *
 * @param   handle      sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiCloseIss(IsiSensorHandle_t handle);

/*****************************************************************************/
/**
 *          Semu_IsiReleaseIss
 *
 * @brief   Release sensor emulation instance resources.
 *
 * @param   handle      sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiReleaseIss(IsiSensorHandle_t handle);

/*****************************************************************************/
/**
 *          Semu_IsiGetCapsIss
 *
 * @brief   Get sensor emulation capabilities.
 *
 * @param   handle      sensor instance handle
 * @param   pCaps     pointer to sensor capabilities structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps);

/*****************************************************************************/
/**
 *          Semu_IsiSetStreamingIss
 *
 * @brief   Enable or disable sensor emulation streaming.
 *
 * @param   handle      sensor instance handle
 * @param   on          streaming state (true=on, false=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on);

/*****************************************************************************/
/**
 *          Semu_IsiGetRevisionIss
 *
 * @brief   Get sensor emulation revision information.
 *
 * @param   handle      sensor instance handle
 * @param   pValue    pointer to revision value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue);

/*****************************************************************************/
/**
 *          Semu_pIsiGetAeBaseInfoIss
 *
 * @brief   Get auto exposure base information for sensor emulation.
 *
 * @param   handle          sensor instance handle
 * @param   pAeBaseInfo   pointer to AE base information structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle,
					IsiAeBaseInfo_t *pAeBaseInfo);

/*****************************************************************************/
/**
 *          Semu_IsiGetAGainIss
 *
 * @brief   Get analog gain values from sensor emulation.
 *
 * @param   handle              sensor instance handle
 * @param   pSensorAGain      pointer to sensor analog gain structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiGetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain);

/*****************************************************************************/
/**
 *          Semu_IsiGetDGainIss
 *
 * @brief   Get digital gain values from sensor emulation.
 *
 * @param   handle              sensor instance handle
 * @param   pSensorDGain      pointer to sensor digital gain structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain);

/*****************************************************************************/
/**
 *          Semu_IsiSetAGainIss
 *
 * @brief   Set analog gain values for sensor emulation.
 *
 * @param   handle              sensor instance handle
 * @param   pSensorAGain      pointer to sensor analog gain structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiSetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain);

/*****************************************************************************/
/**
 *          Semu_IsiSetDGainIss
 *
 * @brief   Set digital gain values for sensor emulation.
 *
 * @param   handle              sensor instance handle
 * @param   pSensorDGain      pointer to sensor digital gain structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain);

/*****************************************************************************/
/**
 *          Semu_IsiGetIntTimeIss
 *
 * @brief   Get integration time values from sensor emulation.
 *
 * @param   handle              sensor instance handle
 * @param   pSensorIntTime    pointer to sensor integration time structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiGetIntTimeIss(IsiSensorHandle_t handle,
					IsiSensorIntTime_t *pSensorIntTime);

/*****************************************************************************/
/**
 *          Semu_IsiSetIntTimeIss
 *
 * @brief   Set integration time values for sensor emulation.
 *
 * @param   handle              sensor instance handle
 * @param   pSensorIntTime    pointer to sensor integration time structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_IsiSetIntTimeIss(IsiSensorHandle_t handle,
					IsiSensorIntTime_t *pSensorIntTime);

/*****************************************************************************/
/**
 *          Semu_SetIntTime
 *
 * @brief   Set integration time for sensor emulation.
 *
 * @param   handle                  sensor instance handle
 * @param   newIntegrationTime      new integration time value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_SetIntTime(IsiSensorHandle_t handle, float newIntegrationTime);

/*****************************************************************************/
/**
 *          Semu_AecSetModeParameters
 *
 * @brief   Set AEC mode parameters for sensor emulation.
 *
 * @param   handle          sensor instance handle
 * @param   pSemuCtx      pointer to sensor emulation context
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT Semu_AecSetModeParameters(IsiSensorHandle_t handle, Semu_Context_t *pSemuCtx);

#ifdef __cplusplus
}
#endif

#endif
