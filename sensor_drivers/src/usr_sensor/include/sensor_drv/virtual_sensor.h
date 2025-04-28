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
#ifndef __VIRTUAL_SENSOR_PRIV_H__
#define __VIRTUAL_SENSOR_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_i2c.h>
#include <isi/isi_common.h>
#include <isi/isi_vvsensor.h>
#include <isi/isi_priv.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t dest_cpu_id;
extern uint32_t cookieRpu;

typedef struct {
	IsiSensorContext_t isiCtx;
	uint8_t rpu_id;
	uint32_t instanceId;
} VirtualSensor_Context_t;

static RESULT virtualSensor_IsiCreateIss(IsiSensorInstanceConfig_t *ConfigPtr,
					IsiSensorHandle_t *HandlePtr);
static RESULT virtualSensor_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode);
static RESULT virtualSensor_IsiCloseIss(IsiSensorHandle_t handle);
static RESULT virtualSensor_IsiReleaseIss(IsiSensorHandle_t handle);
static RESULT virtualSensor_IsiEnumModeIss(IsiSensorHandle_t handle,
					IsiSensorEnumMode_t *EnumModePtr);
static RESULT virtualSensor_IsiCheckConnectionIss(IsiSensorHandle_t handle);
static RESULT virtualSensor_IsiGetModeIss(IsiSensorHandle_t handle, IsiSensorMode_t *ModePtr);
static RESULT virtualSensor_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *CapsPtr);
static RESULT virtualSensor_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on);
static RESULT virtualSensor_IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *ValuePtr);

static RESULT virtualSensor_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle,
					IsiAeBaseInfo_t *AeBaseInfoPtr);
static RESULT virtualSensor_IsiGetAGainIss(IsiSensorHandle_t handle,
					IsiSensorGain_t *SensorAGainPtr);
static RESULT virtualSensor_IsiGetDGainIss(IsiSensorHandle_t handle,
					IsiSensorGain_t *SensorDGainPtr);
static RESULT virtualSensor_IsiSetAGainIss(IsiSensorHandle_t handle,
					IsiSensorGain_t *SensorAGainPtr);
static RESULT virtualSensor_IsiSetDGainIss(IsiSensorHandle_t handle,
					IsiSensorGain_t *SensorDGainPtr);
static RESULT virtualSensor_IsiGetIntTimeIss(IsiSensorHandle_t handle,
					IsiSensorIntTime_t *SensorIntTimePtr);
static RESULT virtualSensor_IsiSetIntTimeIss(IsiSensorHandle_t handle,
					IsiSensorIntTime_t *SensorIntTimePtr);
static RESULT virtualSensor_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *FpsPtr);
static RESULT virtualSensor_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps);

static RESULT virtualSensor_IsiGetIspStatusIss(IsiSensorHandle_t handle,
					IsiIspStatus_t *IspStatusPtr);
static RESULT virtualSensor_IsiSetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *WbPtr);
static RESULT virtualSensor_IsiGetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *WbPtr);
static RESULT virtualSensor_IsiSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *BlcPtr);
static RESULT virtualSensor_IsiGetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *BlcPtr);

static RESULT virtualSensor_IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg);
static RESULT virtualSensor_IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *TpgPtr);
static RESULT virtualSensor_IsiGetExpandCurveIss(IsiSensorHandle_t handle,
					IsiSensorCompandCurve_t *CurvePtr);

static RESULT virtualSensor_IsiWriteRegIss(IsiSensorHandle_t handle, const uint16_t addr,
					const uint16_t value);
static RESULT virtualSensor_IsiReadRegIss(IsiSensorHandle_t handle, const uint16_t addr,
					uint16_t *ValuePtr);

#ifdef __cplusplus
}
#endif

#endif
