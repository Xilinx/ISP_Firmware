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

typedef struct Semu_Context_s
{
    IsiSensorContext_t     isiCtx;                 /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */
    IsiSensorMode_t        sensorMode;

    uint32_t               maxFps;
    uint32_t               minFps;
    uint32_t               currFps;

    bool_t                 configured;             /**< flags that config was applied to sensor */
    bool_t                 streaming;              /**< flags that csensor is streaming data */
    bool_t                 testPattern;            /**< flags that sensor is streaming test-pattern */
    bool_t                 isAfpsRun;              /**< if true, just do anything required for Afps parameter calculation, but DON'T access SensorHW! */

    float                  oneLineDCGExpTime;
    float                  oneLineSPDExpTime;
    float                  oneLineVSExpTime;
    uint16_t               maxDCGIntegrationLine;
    uint16_t               minDCGIntegrationLine;
    uint16_t               maxSPDIntegrationLine;
    uint16_t               minSPDIntegrationLine;
    uint16_t               maxVSIntegrationLine;
    uint16_t               minVSIntegrationLine;

    uint16_t               frameLengthLines;       /**< frame line length */
    uint16_t               curFrameLengthLines;

    float                  aecMinGain;
    float                  aecMaxGain;
    float                  aecMinIntegrationTime;
    float                  aecMaxIntegrationTime;

    float                  aecIntegrationTimeIncrement; /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */
    float                  aecGainIncrement;            /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */

    IsiSensorGain_t        curGain;
    IsiSensorGain_t        curAgain;
    IsiSensorGain_t        curDgain;
    IsiSensorIntTime_t     curIntTime;

    bool                   groupHold;
    uint32_t               oldGain;
    uint32_t               oldIntegrationTime;

    IsiGainInfo_t          aGain;
    IsiGainInfo_t          aVSGain;
    IsiGainInfo_t          dGain;
    IsiGainInfo_t          dVSGain;

    IsiSensorBlc_t         sensorBlc;
    IsiSensorWb_t          sensorWb;

    uint32_t			   sensorDevId;

} Semu_Context_t;

static RESULT Semu_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig, IsiSensorHandle_t *pHandle);
static RESULT Semu_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode);
static RESULT Semu_IsiCloseIss(IsiSensorHandle_t handle);
static RESULT Semu_IsiReleaseIss(IsiSensorHandle_t handle);
static RESULT Semu_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t * pCaps);
static RESULT Semu_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on);
static RESULT Semu_IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue);
static RESULT Semu_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle, IsiAeBaseInfo_t *pAeBaseInfo);
static RESULT Semu_IsiGetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain);
static RESULT Semu_IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain);
static RESULT Semu_IsiSetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain);
static RESULT Semu_IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain);
static RESULT Semu_IsiGetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime);
static RESULT Semu_IsiSetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime);
static RESULT Semu_SetIntTime(IsiSensorHandle_t handle, float newIntegrationTime);
static RESULT Semu_AecSetModeParameters(IsiSensorHandle_t handle, Semu_Context_t *pSemuCtx);

#ifdef __cplusplus
}
#endif

#endif    /* __SEMUPRIV_H__ */
