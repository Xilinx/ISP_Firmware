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
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/misc.h>
#include <isi/isi_fmc.h>
#include "isi/isi.h"
#include "isi/isi_iss.h"
#include "isi/isi_priv.h"
#include "sensor_drv/semu_priv.h"

CREATE_TRACER( Semu_INFO , "semu: ", INFO,    1 );
CREATE_TRACER( Semu_WARN , "semu: ", WARNING, 1 );
CREATE_TRACER( Semu_ERROR, "semu: ", ERROR,   1 );
CREATE_TRACER( Semu_DEBUG,     "semu: ", INFO, 0 );
CREATE_TRACER( Semu_REG_INFO , "semu: ", INFO, 1 );
CREATE_TRACER( Semu_REG_DEBUG, "semu: ", INFO, 1 );

#define SEMU_ID_MAX 5
#define VTPG_OFFSET 0x1000

const uint32_t semuss_baseaddr[SEMU_ID_MAX] = {
        /*    0x800D0000,
            0x800F0000,
            0x80100000,
            0x80110000,
            0x80120000*/
            0xB1020000,
            0XB1080000
        };

#define Semu_MIN_GAIN_STEP    ( 1.0f/1024.0f )

/*****************************************************************************
 *Semu Info
*****************************************************************************/

static IsiSensorMode_t psemu_mode_info[] = {
	{
		.index     = 0,
		.size      = {
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
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 12,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 1,
		.size      = {
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
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 10,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 2,
		.size      = {
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
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 16,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 3,
		.size      = {
			.boundsWidth  = 1280,
			.boundsHeight = 720,
			.top           = 0,
			.left          = 0,
			.width         = 1280,
			.height        = 720,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 12,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 4,
		.size      = {
			.boundsWidth  = 1280,
			.boundsHeight = 720,
			.top           = 0,
			.left          = 0,
			.width         = 1280,
			.height        = 720,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 10,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 5,
		.size      = {
			.boundsWidth  = 1280,
			.boundsHeight = 720,
			.top           = 0,
			.left          = 0,
			.width         = 1280,
			.height        = 720,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 16,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 6,
		.size      = {
			.boundsWidth  = 640,
			.boundsHeight = 480,
			.top           = 0,
			.left          = 0,
			.width         = 640,
			.height        = 480,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 12,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 7,
		.size      = {
			.boundsWidth  = 640,
			.boundsHeight = 480,
			.top           = 0,
			.left          = 0,
			.width         = 640,
			.height        = 480,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 10,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	},
	{
		.index     = 8,
		.size      = {
			.boundsWidth  = 640,
			.boundsHeight = 480,
			.top           = 0,
			.left          = 0,
			.width         = 640,
			.height        = 480,
		},
		.aeInfo    = {
			.intTimeDelayFrame = 2,
			.gainDelayFrame = 2,
		},
		.fps       = 10 * ISI_FPS_QUANTIZE,
		.hdrMode  = ISI_SENSOR_MODE_LINEAR,
		.nativeMode = ISI_SENSOR_NATIVE_DCG_SPD_VS,
		.bitWidth = 16,
		.compress.enable = 0,
		.compress.xBit  = 12,
		.compress.yBit  = 12,
		.bayerPattern = ISI_BPAT_GRBG,
		.afMode = ISI_SENSOR_AF_MODE_NOTSUPP,
	}
};

static RESULT Semu_IsValidSemuId(uint32_t sensorDevId)
{
    if (sensorDevId >= SEMU_ID_MAX) return RET_NOTSUPP;
    return RET_SUCCESS;
}

static RESULT Semu_IsiReadRegIss(IsiSensorHandle_t handle, const uint32_t addr, uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

    if (Semu_IsValidSemuId(pSemuCtx->sensorDevId)) {
        TRACE(Semu_ERROR, "%s: Can't Support this sensor_id:%d\n", __func__, pSemuCtx->sensorDevId);
        return RET_NOTSUPP;
    }

    *pValue = Xil_In32(semuss_baseaddr[pSemuCtx->sensorDevId] + VTPG_OFFSET + addr);
//    *pValue = Xil_In32(semuss_baseaddr[pSemuCtx->sensorDevId] + addr);

	TRACE(Semu_INFO, "%s (exit) result = %d\n", __func__, result);
	return (result);
}

static RESULT Semu_IsiWriteRegIss(IsiSensorHandle_t handle, const uint32_t addr, const uint32_t value)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

    if (Semu_IsValidSemuId(pSemuCtx->sensorDevId)) {
        TRACE(Semu_ERROR, "%s: Can't Support this sensor_id:%d\n", __func__, pSemuCtx->sensorDevId);
        return RET_NOTSUPP;
    }

    Xil_Out32(semuss_baseaddr[pSemuCtx->sensorDevId] + VTPG_OFFSET + addr, value);
//    Xil_Out32(semuss_baseaddr[pSemuCtx->sensorDevId] + addr, value);

	TRACE(Semu_INFO, "%s (exit) result = %d\n", __func__, result);
	return (result);
}

static RESULT Semu_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig, IsiSensorHandle_t *pHandle)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) osMalloc(sizeof(Semu_Context_t));
	if (!pSemuCtx) {
		TRACE(Semu_ERROR, "%s: Can't allocate ox03f10 context\n", __func__);
		return (RET_OUTOFMEM);
	}

	xil_printf("semu create start\n");

	MEMSET(pSemuCtx, 0, sizeof(Semu_Context_t));

	pSemuCtx->isiCtx.pSensor     = pConfig->pSensor;
	pSemuCtx->configured         = BOOL_FALSE;
	pSemuCtx->streaming          = BOOL_FALSE;
	pSemuCtx->sensorDevId        = pConfig->cameraDevId;
	xil_printf("sensorDevId:%d\n", pSemuCtx->sensorDevId);
	pSemuCtx->groupHold          = BOOL_FALSE;
	pSemuCtx->oldGain            = 0;
	pSemuCtx->oldIntegrationTime = 0;
	pSemuCtx->testPattern        = BOOL_FALSE;
	pSemuCtx->isAfpsRun          = BOOL_FALSE;
	pSemuCtx->sensorMode.index   = 0;
#if 1
	IsiSensorMode_t *SensorDefaultMode = NULL;
	for (int i=0; i < sizeof(psemu_mode_info) / sizeof(IsiSensorMode_t); i++) {
		if (psemu_mode_info[i].index == pSemuCtx->sensorMode.index) {
			SensorDefaultMode = &(psemu_mode_info[i]);
			break;
		}
	}

	if (SensorDefaultMode != NULL) {
		memcpy(&(pSemuCtx->sensorMode), SensorDefaultMode, sizeof(IsiSensorMode_t));
	} else {
		TRACE(Semu_ERROR,"%s: Invalid SensorDefaultMode\n", __func__);
		return (RET_NULL_POINTER);
	}
#endif

	*pHandle = (IsiSensorHandle_t) pSemuCtx;

	xil_printf("semu create done\n");

	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	return (result);
}

static RESULT semu_config(IsiSensorHandle_t handle)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;
	
    xil_printf("middha start semu config\n");

	if (!pSemuCtx) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n",__func__);
		return (RET_WRONG_HANDLE);
	}

    if (Semu_IsValidSemuId(pSemuCtx->sensorDevId)) {
        TRACE(Semu_ERROR, "%s: Can't Support this sensor_id:%d\n", __func__, pSemuCtx->sensorDevId);
        return RET_NOTSUPP;
    }

	xil_printf("middha - sensorDevId: %d\n", pSemuCtx->sensorDevId);

	const uint32_t vtpg_addr = semuss_baseaddr[pSemuCtx->sensorDevId];
	const uint32_t semu_addr = semuss_baseaddr[pSemuCtx->sensorDevId] + VTPG_OFFSET;
//	const uint32_t semu_addr = semuss_baseaddr[pSemuCtx->sensorDevId];
//	const uint32_t vtpg_addr = semuss_baseaddr[pSemuCtx->sensorDevId] + VTPG_OFFSET;

	// tpg default settings
    xil_printf("middha - 1\n");
    Xil_Out32(vtpg_addr + 0x0020, 0xB        );  // tpg pattern
    xil_printf("middha - 2\n");
    Xil_Out32(vtpg_addr + 0x0028, 0x0        );
    xil_printf("middha - 3\n");
    Xil_Out32(vtpg_addr + 0x0038, 0x2        );
    xil_printf("middha - 4\n");
//    Xil_Out32(vtpg_addr + 0x0040, 0x0        );
    Xil_Out32(vtpg_addr + 0x0078, 0x50       );
    xil_printf("middha - 5\n");
//    Xil_Out32(vtpg_addr + 0x0080, 0x255      );  // box control
//    Xil_Out32(vtpg_addr + 0x0088, 0x0        );
//    Xil_Out32(vtpg_addr + 0x0090, 0x0        );    

	// tpg config settings
    Xil_Out32(vtpg_addr + 0x0010, pSemuCtx->sensorMode.size.height);
    xil_printf("middha - 6\n");
    Xil_Out32(vtpg_addr + 0x0018, pSemuCtx->sensorMode.size.width);
    xil_printf("middha - 7\n");

	// semu default settings
	Xil_Out32(semu_addr + 0x0008, 0x1        );	// bayer pattern
    xil_printf("middha - 8\n");

	// semu config settings
	Xil_Out32(semu_addr + 0x0010, ((pSemuCtx->sensorMode.bitWidth)<<16)|0x4); // bpc and ppc
    xil_printf("middha - 9\n");
	Xil_Out32(semu_addr + 0x000C, ((pSemuCtx->sensorMode.size.width)<<16)|(pSemuCtx->sensorMode.size.height));
    xil_printf("middha - 10\n");

	Xil_Out32(semu_addr + 0x0020, /*0x2625A0*//*0x3D090*/ 0XE4E1C0   );	// 10 fps
	//Xil_Out32(semu_addr + 0x0020, 0x1312d0   );	// 20 fps
	//Xil_Out32(semu_addr + 0x0020, 0xCB735    );	// 30 fps

	xil_printf("middha - width:%d, height:%d\n",pSemuCtx->sensorMode.size.width, pSemuCtx->sensorMode.size.height);
	xil_printf("middha - bitwidth:%d\n",pSemuCtx->sensorMode.bitWidth);

	xil_printf("middha end semu config\n");
	return result;
}

static RESULT Semu_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Semu_INFO, "%s (enter)\n", __func__);
	xil_printf("middha - Semu_IsiOpenIss\n");

	if (!pSemuCtx) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n",__func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSemuCtx->streaming != BOOL_FALSE) {
		return RET_WRONG_STATE;
	}

	pSemuCtx->sensorMode.index   = mode;

    IsiSensorMode_t *SensorDefaultMode = NULL;
	for (int i=0; i < sizeof(psemu_mode_info) / sizeof(IsiSensorMode_t); i++) {
		if (psemu_mode_info[i].index == pSemuCtx->sensorMode.index) {
			SensorDefaultMode = &(psemu_mode_info[i]);
			break;
		}
	}

	if (SensorDefaultMode != NULL) {
		memcpy(&(pSemuCtx->sensorMode), SensorDefaultMode, sizeof(IsiSensorMode_t));
	} else {
		TRACE(Semu_ERROR,"%s: Invalid SensorDefaultMode\n", __func__);
		return (RET_NULL_POINTER);
	}

	pSemuCtx->minFps  = 1;
	pSemuCtx->maxFps  = 10;
	pSemuCtx->currFps = pSemuCtx->sensorMode.fps;

	// semu configuration
    semu_config(handle);

	// default configuration
	pSemuCtx->oneLineDCGExpTime    = 0.0000319;
	pSemuCtx->oneLineSPDExpTime    = 0.0000158;
	pSemuCtx->oneLineVSExpTime     = 0.0000158;
	pSemuCtx->frameLengthLines     = 0x330;
	pSemuCtx->curFrameLengthLines  = pSemuCtx->frameLengthLines;
	pSemuCtx->maxDCGIntegrationLine= 1569;
	pSemuCtx->minDCGIntegrationLine= 2;
	pSemuCtx->maxSPDIntegrationLine= 1571;
	pSemuCtx->minSPDIntegrationLine= 2;
	pSemuCtx->maxVSIntegrationLine = 52;
	pSemuCtx->minVSIntegrationLine = 1;
	pSemuCtx->aecMaxGain           = 240;
	pSemuCtx->aecMinGain           = 1.0;
	pSemuCtx->aGain.min            = 1.0;
	pSemuCtx->aGain.max            = 15.5;
	pSemuCtx->aGain.step           = (1.0f/16.0f);
	pSemuCtx->dGain.min            = 1.0;
	pSemuCtx->dGain.max            = 15.99;
	pSemuCtx->dGain.step           = (1.0f/1024.0f);
	pSemuCtx->sensorWb.rGain = 1.8;
	pSemuCtx->sensorWb.gbGain = 1.0;
	pSemuCtx->sensorWb.grGain = 1.0;
	pSemuCtx->sensorWb.bGain = 1.65;

	result = Semu_AecSetModeParameters(handle, pSemuCtx);
	if (result != RET_SUCCESS) {
		TRACE(Semu_ERROR, "%s: SetupOutputWindow failed.\n", __func__);
		return (result);
	}

	pSemuCtx->configured = BOOL_TRUE;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return result;
}

static RESULT Semu_IsiCloseIss(IsiSensorHandle_t handle)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	if (pSemuCtx == NULL) return (RET_WRONG_HANDLE);

	(void)Semu_IsiSetStreamingIss(handle, BOOL_FALSE);

	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	return (result);
}

static RESULT Semu_IsiReleaseIss(IsiSensorHandle_t handle)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	if (pSemuCtx == NULL) return (RET_WRONG_HANDLE);

	MEMSET(pSemuCtx, 0, sizeof(Semu_Context_t));
	osFree(pSemuCtx);
	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	return (result);
}

static RESULT Semu_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	xil_printf("%s: (enter)\n", __func__);
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

	if (pSemuCtx->configured != BOOL_TRUE)
		return RET_WRONG_STATE;

	// START/STOP TPG

//	result = Semu_IsiWriteRegIss(handle, 0x0000, 0x80|on);
	const uint32_t vtpg_addr = semuss_baseaddr[pSemuCtx->sensorDevId];
    Xil_Out32(vtpg_addr + 0x0000, 0x80|on);

	if (result != RET_SUCCESS) {
		TRACE(Semu_ERROR, "%s: set sensor streaming error! \n",__func__);
		return (RET_FAILURE);
	}

	// START/STOP SEMU
	result = Semu_IsiWriteRegIss(handle, 0x0004, on);
	if (result != RET_SUCCESS) {
		TRACE(Semu_ERROR, "%s: set sensor streaming error! \n",__func__);
		return (RET_FAILURE);
	}

	pSemuCtx->streaming = on;

	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	xil_printf("%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;
//	xil_printf("%s: (enter)\n", __func__);

	if (pSemuCtx == NULL) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
		return (RET_WRONG_HANDLE);
	}

	*pFps = pSemuCtx->currFps;

//	xil_printf("%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
	RESULT result = RET_SUCCESS;
	uint32_t int_fps_period;

	xil_printf("%s: (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
		return (RET_WRONG_HANDLE);
	}

    fps /= ISI_FPS_QUANTIZE;

	int_fps_period = 25000000 / fps;

//	result = Semu_IsiWriteRegIss(handle, 0x20, int_fps_period);
	if (result != RET_SUCCESS) {
		TRACE(Semu_ERROR, "%s: set fps failed! \n",__func__);
		return (RET_FAILURE);
	} else {
		pSemuCtx->currFps = fps * ISI_FPS_QUANTIZE;
	}

	xil_printf("%s: (exit)\n", __func__);
	return (result);
}

static RESULT Semu_IsiGetModeIss(IsiSensorHandle_t handle, IsiSensorMode_t *pMode)
{
    TRACE(Semu_INFO, "%s (enter)\n", __func__);
    Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
    if (pSemuCtx == NULL) {
        return (RET_WRONG_HANDLE);
    }
    if (pMode == NULL) {
        return (RET_WRONG_HANDLE);
    }

    memcpy(pMode, &(pSemuCtx->sensorMode), sizeof(IsiSensorMode_t));

    TRACE(Semu_INFO, "%s (exit)\n", __func__);
    return ( RET_SUCCESS );
}

/*static RESULT Semu_IsiSetModeIss(IsiSensorHandle_t handle, IsiSensorMode_t *pMode)
{
	TRACE(Semu_INFO, "%s (enter)\n", __func__);
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;

	xil_printf("setmode start\n");

	if (pSemuCtx == NULL) {
		return (RET_WRONG_HANDLE);
	}

	if (pMode == NULL) {
		return (RET_WRONG_HANDLE);
	}

	memcpy(&(pSemuCtx->sensorMode), pMode, sizeof(IsiSensorMode_t));

	xil_printf("width:%d, height:%d\n",pSemuCtx->sensorMode.size.width, pSemuCtx->sensorMode.size.height);

	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	xil_printf("setmode end\n");

	return ( RET_SUCCESS );
}
*/

static RESULT Semu_IsiEnumModeIss(IsiSensorHandle_t handle, IsiSensorEnumMode_t *pEnumMode)
{
	TRACE(Semu_INFO, "%s (enter)\n", __func__);
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

	if (pEnumMode->index >= (sizeof(psemu_mode_info)/sizeof(psemu_mode_info[0])))
		return RET_OUTOFRANGE;

	for (uint32_t i = 0; i < (sizeof(psemu_mode_info)/sizeof(psemu_mode_info[0])); i++) {
		if (psemu_mode_info[i].index == pEnumMode->index) {
			memcpy(&pEnumMode->mode, &psemu_mode_info[i], sizeof(IsiSensorMode_t));
			TRACE(Semu_INFO, "%s (exit)\n", __func__);
			return RET_SUCCESS;
		}
	}

	return RET_NOTSUPP;
}

static RESULT Semu_AecSetModeParameters(IsiSensorHandle_t handle, Semu_Context_t *pSemuCtx)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s%s: (enter)\n", __func__, pSemuCtx->isAfpsRun ? "(AFPS)" : "");
	uint32_t exp_line = 0, again = 0, dgain = 0;
	uint16_t value = 0;

	pSemuCtx->aecMinIntegrationTime       = pSemuCtx->oneLineDCGExpTime * pSemuCtx->minDCGIntegrationLine;
	pSemuCtx->aecMaxIntegrationTime       = pSemuCtx->oneLineDCGExpTime * pSemuCtx->maxDCGIntegrationLine;
	TRACE(Semu_DEBUG, "%s: AecMaxIntegrationTime = %f \n", __func__, pSemuCtx->aecMaxIntegrationTime);

	pSemuCtx->aecGainIncrement = Semu_MIN_GAIN_STEP;
	pSemuCtx->aecIntegrationTimeIncrement = pSemuCtx->oneLineDCGExpTime;
	pSemuCtx->oldGain               = 0;
	pSemuCtx->oldIntegrationTime    = 0;

#if 0
	//reflects the state of the sensor registers, must equal default settings
	//get again
	Semu_IsiReadRegIss(handle, 0x3508, &value);
	again = (value & 0x0f) << 4;
	Semu_IsiReadRegIss(handle, 0x3509, &value);
	again = again | ((value & 0xf0) >> 4);

	//get dgain
	Semu_IsiReadRegIss(handle, 0x350a, &value);
	dgain = (value & 0x0f) << 10;
	Semu_IsiReadRegIss(handle, 0x350b, &value);
	dgain = dgain | ((value & 0xff) << 2);
	Semu_IsiReadRegIss(handle, 0x350c, &value);
	dgain = dgain | ((value & 0xc0) >> 6);
#endif

	pSemuCtx->curGain.gain[0] = ((float)again/16.0) * ((float)dgain/1024.0);

#if 0
	//get exp_line
	Semu_IsiReadRegIss(handle, 0x3501, &value);
	exp_line = (value & 0xff) << 8;
	Semu_IsiReadRegIss(handle, 0x3502, &value);
	exp_line = exp_line | (value & 0xff);
#endif
	pSemuCtx->curIntTime.intTime[0] = exp_line * pSemuCtx->oneLineDCGExpTime;


	TRACE(Semu_INFO, "%s: (exit)\n", __func__);

	return (result);
}

static RESULT Semu_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;

	RESULT result = RET_SUCCESS;

	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	if (pSemuCtx == NULL) return (RET_WRONG_HANDLE);

	if (pCaps == NULL) {
		return (RET_NULL_POINTER);
	}

	pCaps->bitWidth          = pSemuCtx->sensorMode.bitWidth;
	pCaps->mode              = ISI_MODE_BAYER;
	pCaps->bayerPattern      = pSemuCtx->sensorMode.bayerPattern;
	pCaps->resolution.width  = pSemuCtx->sensorMode.size.width;
	pCaps->resolution.height = pSemuCtx->sensorMode.size.height;
	pCaps->mipiLanes         = ISI_MIPI_4LANES;
	pCaps->vinType           = ISI_ITF_TYPE_MIPI;

	if (pCaps->bitWidth == 10) {
		pCaps->mipiMode      = ISI_FORMAT_RAW_10;
	} else if (pCaps->bitWidth == 12) {
		pCaps->mipiMode      = ISI_FORMAT_RAW_12;
	} else {
		pCaps->mipiMode      = ISI_MIPI_OFF;
	}

	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	return (result);
}

static RESULT Semu_IsiCheckConnectionIss(IsiSensorHandle_t handle)
{
	RESULT result = RET_SUCCESS;

	uint32_t sensor_id = 0;
	uint32_t correct_id = 0x5801;

	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

	result = Semu_IsiGetRevisionIss(handle, &sensor_id);
	if (result != RET_SUCCESS) {
		TRACE(Semu_ERROR, "%s: Read Sensor ID Error! \n",__func__);
		return (RET_FAILURE);
	}

	if (correct_id != sensor_id) {
		TRACE(Semu_ERROR, "%s:ChipID =0x%x sensor_id=%x error! \n", __func__, correct_id, sensor_id);
		return (RET_FAILURE);
	}

	TRACE(Semu_INFO,"%s ChipID = 0x%08x, sensor_id = 0x%08x, success! \n", __func__, correct_id, sensor_id);
	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	return (result);
}

static RESULT Semu_IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
	RESULT result = RET_SUCCESS;
	uint16_t reg_val;
	uint32_t sensor_id = 0x5801;

	TRACE(Semu_INFO, "%s (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

#if 0
	reg_val   = 0;
	result    = Semu_IsiReadRegIss(handle, 0x300a, &reg_val);
	sensor_id = (reg_val & 0xff) << 16;

	reg_val   = 0;
	result    |= Semu_IsiReadRegIss(handle, 0x300b, &reg_val);
	sensor_id |= ((reg_val & 0xff) << 8);

	reg_val   = 0;
	result    |= Semu_IsiReadRegIss(handle, 0x300c, &reg_val);
	sensor_id |= (reg_val & 0xff);
#endif

	*pValue = sensor_id;
	TRACE(Semu_INFO, "%s (exit)\n", __func__);
	return (result);
}

static RESULT Semu_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle, IsiAeBaseInfo_t *pAeBaseInfo)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;

	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	if (pSemuCtx == NULL) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n",__func__);
		return (RET_WRONG_HANDLE);
	}

	if (pAeBaseInfo == NULL) {
		TRACE(Semu_ERROR, "%s: NULL pointer received!!\n");
		return (RET_NULL_POINTER);
	}

	//get time limit and total gain limit
	pAeBaseInfo->longGain.min        = pSemuCtx->aecMinGain;
	pAeBaseInfo->longGain.max        = pSemuCtx->aecMaxGain;
	pAeBaseInfo->longIntTime.min     = pSemuCtx->aecMinIntegrationTime;
	pAeBaseInfo->longIntTime.max     = pSemuCtx->aecMaxIntegrationTime;

	//get again/dgain info
	pAeBaseInfo->aLongGain           = pSemuCtx->aGain;
	pAeBaseInfo->dLongGain           = pSemuCtx->dGain;

	//get current intTime and gain
	pAeBaseInfo->curIntTime  = pSemuCtx->curIntTime;
	pAeBaseInfo->curGain     = pSemuCtx->curGain;

	pAeBaseInfo->aecGainStep = pSemuCtx->aecGainIncrement;
	pAeBaseInfo->aecIntTimeStep = pSemuCtx->aecIntegrationTimeIncrement;
	pAeBaseInfo->nativeMode  = pSemuCtx->sensorMode.nativeMode;
	pAeBaseInfo->nativeHdrRatio[0] = 14.4;
	pAeBaseInfo->nativeHdrRatio[1] = 29.76;
	pAeBaseInfo->nativeHdrRatio[2] = 4.48;
	pAeBaseInfo->conversionGainDCG = 7.2;

	TRACE(Semu_INFO, "%s: (enter)\n", __func__);
	return (result);
}

RESULT Semu_IsiSetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);
	uint32_t again = 0;

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

	if(pSensorAGain->gain[ISI_LINEAR_PARAS] < pSemuCtx->aGain.min){
		TRACE(Semu_WARN, "%s: invalid too small again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pSemuCtx->aGain.min;
	}

	if(pSensorAGain->gain[ISI_LINEAR_PARAS] > pSemuCtx->aGain.max){
		TRACE(Semu_WARN, "%s: invalid too big again parameter!\n", __func__);
		pSensorAGain->gain[ISI_LINEAR_PARAS] = pSemuCtx->aGain.max;
	}

	//set HCG analog gain, base exp, conversion gain = HCG/LCG = 7.2
	again = (uint32_t)(pSensorAGain->gain[ISI_LINEAR_PARAS] * 16);
//	result |= Semu_IsiWriteRegIss(handle, 0x3508, (again & 0xf0) >> 4);
//	result |= Semu_IsiWriteRegIss(handle, 0x3509, (again & 0x0f) << 4);
	pSemuCtx->curAgain.gain[0] = (float)again/16.0f;

	//set LCG analog gain
	again = (uint32_t)(pSensorAGain->gain[ISI_LINEAR_PARAS] / 2 * 16);
//	result |= Semu_IsiWriteRegIss(handle, 0x3588, (again & 0xf0) >> 4);
//	result |= Semu_IsiWriteRegIss(handle, 0x3589, (again & 0x0f) << 4);
	pSemuCtx->curAgain.gain[1] = (float)again/16.0f;

	//set S analog gain, base exp
	again = (uint32_t)(pSensorAGain->gain[ISI_LINEAR_PARAS] * 16);
//	result = Semu_IsiWriteRegIss(handle, 0x3548, (again & 0xf0) >> 4);
//	result |= Semu_IsiWriteRegIss(handle, 0x3549, (again & 0x0f) << 4);
	pSemuCtx->curAgain.gain[2] = (float)again/16.0f;

	//set VS analog gain
	again = (uint32_t)(pSensorAGain->gain[ISI_LINEAR_PARAS] / 4 * 16);
//	result |= Semu_IsiWriteRegIss(handle, 0x35c8, (again & 0xf0) >> 4);
//	result |= Semu_IsiWriteRegIss(handle, 0x35c9, (again & 0x0f) << 4);
	pSemuCtx->curAgain.gain[3] = (float)again/16.0f;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);
	uint32_t dgain = 0;

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

	if(pSensorDGain->gain[ISI_LINEAR_PARAS] < pSemuCtx->dGain.min){
		TRACE(Semu_WARN, "%s: invalid too small dgain parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pSemuCtx->dGain.min;
	}

	if(pSensorDGain->gain[ISI_LINEAR_PARAS] > pSemuCtx->dGain.max){
		TRACE(Semu_WARN, "%s: invalid too big dgain parameter!\n", __func__);
		pSensorDGain->gain[ISI_LINEAR_PARAS] = pSemuCtx->dGain.max;
	}

	//set HCG digital gain, base exp
	dgain = (uint32_t)(pSensorDGain->gain[ISI_LINEAR_PARAS] * 1024);
//	result = Semu_IsiWriteRegIss(handle, 0x350a, (dgain >> 10) & 0x0f);
//	result |= Semu_IsiWriteRegIss(handle,0x350b, (dgain >> 2) & 0xff);
//	result |= Semu_IsiWriteRegIss(handle,0x350c, (dgain & 0x03) << 6);
	pSemuCtx->curDgain.gain[0] = (float)dgain/1024.0f;
	pSemuCtx->curGain.gain[0] = pSemuCtx->curAgain.gain[0] * pSemuCtx->curDgain.gain[0];

	//set LCG digital gain
	dgain = (uint32_t)(pSensorDGain->gain[ISI_LINEAR_PARAS] * 1024);
//	result = Semu_IsiWriteRegIss(handle, 0x358a, (dgain >> 10) & 0x0f);
//	result |= Semu_IsiWriteRegIss(handle,0x358b, (dgain >> 2) & 0xff);
//	result |= Semu_IsiWriteRegIss(handle,0x358c, (dgain & 0x03) << 6);
	pSemuCtx->curDgain.gain[1] = (float)dgain/1024.0f;
	pSemuCtx->curGain.gain[1] = pSemuCtx->curAgain.gain[1] * pSemuCtx->curDgain.gain[1];

	//set S digital gain
	dgain = (uint32_t)(pSensorDGain->gain[ISI_LINEAR_PARAS] * 1024);
//	result = Semu_IsiWriteRegIss(handle, 0x354a, (dgain >> 10) & 0x0f);
//	result |= Semu_IsiWriteRegIss(handle,0x354b, (dgain >> 2) & 0xff);
//	result |= Semu_IsiWriteRegIss(handle,0x354c, (dgain & 0x03) << 6);
	pSemuCtx->curDgain.gain[2] = (float)dgain/1024.0f;
	pSemuCtx->curGain.gain[2] = pSemuCtx->curAgain.gain[2] * pSemuCtx->curDgain.gain[2];

	//set VS digital gain
	dgain = (uint32_t)(pSensorDGain->gain[ISI_LINEAR_PARAS] * 1024);
//	result = Semu_IsiWriteRegIss(handle, 0x35ca, (dgain >> 10) & 0x0f);
//	result |= Semu_IsiWriteRegIss(handle,0x35cb, (dgain >> 2) & 0xff);
//	result |= Semu_IsiWriteRegIss(handle,0x35cc, (dgain & 0x03) << 6);
	pSemuCtx->curDgain.gain[3] = (float)dgain/1024.0f;
	pSemuCtx->curGain.gain[3] = pSemuCtx->curAgain.gain[3] * pSemuCtx->curDgain.gain[3];

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiGetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	if (pSemuCtx == NULL) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n",__func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSensorAGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSensorAGain       = pSemuCtx->curAgain;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	if (pSemuCtx == NULL) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n",__func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSensorDGain == NULL) {
		return (RET_NULL_POINTER);
	}

	*pSensorDGain = pSemuCtx->curDgain;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiSetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;

	if (!pSemuCtx) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n", __func__);
		return (RET_WRONG_HANDLE);
	}

	if (pSemuCtx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		result = Semu_SetIntTime(handle, pSensorIntTime->intTime[ISI_LINEAR_PARAS]);
		if (result != RET_SUCCESS) {
			TRACE(Semu_INFO, "%s: set sensor IntTime[ISI_LINEAR_PARAS] error!\n", __func__);
			return RET_FAILURE;
		}

	} else {
		TRACE(Semu_INFO, "%s:not support this ExpoFrmType.\n", __func__);
		return RET_NOTSUPP;
	}

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

static RESULT Semu_SetIntTime(IsiSensorHandle_t handle, float newIntegrationTime)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	uint32_t exp_line = 0;
	float sIntegrationTime = 0, vsIntegrationTime = 0;

	if (!pSemuCtx) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n",__func__);
		return (RET_WRONG_HANDLE);
	}

	//set DCG(HCG & LCG) exp
	exp_line = newIntegrationTime / pSemuCtx->oneLineDCGExpTime;
	exp_line = MIN(pSemuCtx->maxDCGIntegrationLine, MAX(pSemuCtx->minDCGIntegrationLine, exp_line));
	TRACE(Semu_DEBUG, "%s: set DCG exp_line = 0x%04x\n", __func__, exp_line);
//	result =  Semu_IsiWriteRegIss(handle, 0x3501,(exp_line >> 8) & 0xff);
//	result |= Semu_IsiWriteRegIss(handle, 0x3502,(exp_line & 0xff));
	pSemuCtx->curIntTime.intTime[0] = exp_line * pSemuCtx->oneLineDCGExpTime;
	pSemuCtx->curIntTime.intTime[1] = exp_line * pSemuCtx->oneLineDCGExpTime;

	//set SPD exp
	sIntegrationTime = newIntegrationTime;
	exp_line = sIntegrationTime / pSemuCtx->oneLineSPDExpTime;
	exp_line = MIN(pSemuCtx->maxSPDIntegrationLine, MAX(pSemuCtx->minSPDIntegrationLine, exp_line));
	TRACE(Semu_DEBUG, "%s: set SPD exp_line = 0x%04x\n", __func__, exp_line);
//	result = Semu_IsiWriteRegIss(handle, 0x3541,(exp_line >> 8) & 0xff);
//	result |= Semu_IsiWriteRegIss(handle, 0x3542,(exp_line & 0xff));
	pSemuCtx->curIntTime.intTime[2] = exp_line * pSemuCtx->oneLineSPDExpTime;

	//set VS exp
	vsIntegrationTime = newIntegrationTime/4/16;
	exp_line = vsIntegrationTime / pSemuCtx->oneLineVSExpTime;
	exp_line = MIN(pSemuCtx->maxVSIntegrationLine,MAX(pSemuCtx->minVSIntegrationLine, exp_line));
	TRACE(Semu_DEBUG, "%s: set VS exp_line = 0x%04x\n", __func__, exp_line);
//	result |= Semu_IsiWriteRegIss(handle, 0x35c1,(exp_line >> 8) & 0xff);
//	result |= Semu_IsiWriteRegIss(handle, 0x35c2,(exp_line & 0xff));
	pSemuCtx->curIntTime.intTime[3] = exp_line * pSemuCtx->oneLineVSExpTime;
	pSemuCtx->maxDCGIntegrationLine   = pSemuCtx->curFrameLengthLines - 13 - exp_line;

	TRACE(Semu_DEBUG, "%s: set IntTime = %f\n", __func__, pSemuCtx->curIntTime.intTime[0]);
	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiGetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	if (!pSemuCtx) {
		TRACE(Semu_ERROR,"%s: Invalid sensor handle (NULL pointer detected)\n",__func__);
		return (RET_WRONG_HANDLE);
	}

	if (!pSensorIntTime) return (RET_NULL_POINTER);


	if (pSemuCtx->sensorMode.hdrMode == ISI_SENSOR_MODE_HDR_NATIVE) {
		*pSensorIntTime = pSemuCtx->curIntTime;

	} else {
		TRACE(Semu_INFO, "%s:not support this ExpoFrmType.\n", __func__);
		return RET_NOTSUPP;
	}

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiGetIspStatusIss(IsiSensorHandle_t handle, IsiIspStatus_t *pIspStatus)
{
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_WRONG_HANDLE;
	}
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	pIspStatus->useSensorAE  = false;
	pIspStatus->useSensorBLC = true;
	pIspStatus->useSensorAWB = true;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

RESULT Semu_IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_NULL_POINTER;
	}

	if (pSemuCtx->configured != BOOL_TRUE)  return RET_WRONG_STATE;

#if 0
	if (tpg.enable == 0) {
		result = Semu_IsiWriteRegIss(handle, 0x5004, 0x1e);
		result = Semu_IsiWriteRegIss(handle, 0x5005, 0x1e);
		result = Semu_IsiWriteRegIss(handle, 0x5006, 0x1e);
		result = Semu_IsiWriteRegIss(handle, 0x5007, 0x1e);

		result = Semu_IsiWriteRegIss(handle, 0x5240, 0x00);
		result = Semu_IsiWriteRegIss(handle, 0x5440, 0x00);
		result = Semu_IsiWriteRegIss(handle, 0x5640, 0x00);
		result = Semu_IsiWriteRegIss(handle, 0x5840, 0x00);
	} else {
		result = Semu_IsiWriteRegIss(handle, 0x5004, 0x1f);
		result = Semu_IsiWriteRegIss(handle, 0x5005, 0x1f);
		result = Semu_IsiWriteRegIss(handle, 0x5006, 0x1f);
		result = Semu_IsiWriteRegIss(handle, 0x5007, 0x1f);

		result = Semu_IsiWriteRegIss(handle, 0x5240, 0x01);
		result = Semu_IsiWriteRegIss(handle, 0x5440, 0x01);
		result = Semu_IsiWriteRegIss(handle, 0x5640, 0x01);
		result = Semu_IsiWriteRegIss(handle, 0x5840, 0x01);
	}
#endif
	pSemuCtx->testPattern = tpg.enable;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

RESULT Semu_IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *pTpg)
{
	RESULT result = RET_SUCCESS;
	uint16_t hcgValue = 0, lcgValue = 0, spdValue = 0, vsValue = 0;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL || pTpg== NULL) {
		return RET_NULL_POINTER;
	}

	if (pSemuCtx->configured != BOOL_TRUE)  return RET_WRONG_STATE;

		pTpg->enable = (((hcgValue & 0x01) != 0) && ((lcgValue & 0x01) != 0) && ((spdValue & 0x01) != 0) && ((vsValue & 0x01) != 0)) ? 1 : 0;
		if (pTpg->enable) {
			pTpg->pattern = (0xff & hcgValue);
		}
		pSemuCtx->testPattern = pTpg->enable;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

static RESULT Semu_IsiSetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	uint32_t b_gain, gb_gain, gr_gain, r_gain;
	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_WRONG_HANDLE;
	}

	if (pWb == NULL)   return RET_NULL_POINTER;

	b_gain = (uint32_t)(pWb->bGain  * 1024);
	gb_gain = (uint32_t)(pWb->gbGain * 1024);
	gr_gain = (uint32_t)(pWb->grGain * 1024);
	r_gain  = (uint32_t)(pWb->rGain  * 1024);

#if 0
	//set HCG channel awb gain
	result |= Semu_IsiWriteRegIss(handle, 0x5280, (b_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5281, b_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5282, (gb_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5283, gb_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5284, (gr_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5285, gr_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5286, (r_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5287, r_gain & 0xff);

	//set LCG channel awb gain
	result |= Semu_IsiWriteRegIss(handle, 0x5480, (b_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5481, b_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5482, (gb_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5483, gb_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5484, (gr_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5485, gr_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5486, (r_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5487, r_gain & 0xff);

	//set S channel awb gain
	result |= Semu_IsiWriteRegIss(handle, 0x5680, (b_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5681, b_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5682, (gb_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5683, gb_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5684, (gr_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5685, gr_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5686, (r_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5687, r_gain & 0xff);

	//set VS channel awb gain
	result |= Semu_IsiWriteRegIss(handle, 0x5880, (b_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5881, b_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5882, (gb_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5883, gb_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5884, (gr_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5885, gr_gain & 0xff);
	result |= Semu_IsiWriteRegIss(handle, 0x5886, (r_gain >> 8) & 0x7f);
	result |= Semu_IsiWriteRegIss(handle, 0x5887, r_gain & 0xff);
#endif

	memcpy(&pSemuCtx->sensorWb, pWb, sizeof(IsiSensorWb_t));

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return result;
}

static RESULT Semu_IsiGetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL ) {
		return RET_WRONG_HANDLE;
	}

	if (pWb == NULL)  return RET_NULL_POINTER;

	memcpy(pWb, &pSemuCtx->sensorWb, sizeof(IsiSensorWb_t));

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return (result);
}

static RESULT Semu_IsiSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;
	uint16_t blcGain = 0;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_WRONG_HANDLE;
	}

	if (pBlc == NULL)  return RET_NULL_POINTER;

	blcGain = pBlc->red;

#if 0
	//set HCG blc
	result = Semu_IsiWriteRegIss(handle, 0x4026, (blcGain >> 8) & 0x03);
	result |= Semu_IsiWriteRegIss(handle, 0x4027, blcGain & 0xff);
	//set LCG blc
	result |= Semu_IsiWriteRegIss(handle, 0x4028, (blcGain >> 8) & 0x03);
	result |= Semu_IsiWriteRegIss(handle, 0x4029, blcGain & 0xff);
	//set S blc
	result |= Semu_IsiWriteRegIss(handle, 0x402a, (blcGain >> 8) & 0x03);
	result |= Semu_IsiWriteRegIss(handle, 0x402b, blcGain & 0xff);
	//set VS blc
	result |= Semu_IsiWriteRegIss(handle, 0x402c, (blcGain >> 8) & 0x03);
	result |= Semu_IsiWriteRegIss(handle, 0x402d, blcGain & 0xff);
#endif

	pSemuCtx->sensorBlc = *pBlc;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return result;
}

static RESULT Semu_IsiGetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
	RESULT result = RET_SUCCESS;
	TRACE(Semu_INFO, "%s: (enter)\n", __func__);

	Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
	if (pSemuCtx == NULL) {
		return RET_WRONG_HANDLE;
	}

	if (pBlc == NULL)  return RET_NULL_POINTER;

	*pBlc = pSemuCtx->sensorBlc;

	TRACE(Semu_INFO, "%s: (exit)\n", __func__);
	return result;
}


static RESULT Semu_IsiGetExpandCurveIss(IsiSensorHandle_t handle, IsiSensorCompandCurve_t *pCurve)
{
    RESULT result = RET_SUCCESS;
    TRACE(Semu_INFO, "%s: (enter)\n", __func__);

    Semu_Context_t *pSemuCtx = (Semu_Context_t *) handle;
    if (pSemuCtx == NULL) {
        return RET_NULL_POINTER;
    }

    //suppose isp pipeline is 24bit, expand_px left shift 12bit
    uint8_t expand_px[64] = {22, 20, 12, 20, 20, 20, 20, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18,
                            18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 12};//dx_exp[i], index number, x[i]=x[i-1]+2^dx_exp[i]
    memcpy(pCurve->compandPx, expand_px, sizeof(expand_px));

    pCurve->compandXData[0] = 0;//x value in expand curve
    pCurve->compandYData[0] = 0;//y value in expand curve
    for(int i=1; i<65; i++){
        if (pCurve->compandXData[i-1] == 0 && pCurve->compandPx[i-1] > 0){
            pCurve->compandXData[i] = pCurve->compandXData[i-1] + ((1 << pCurve->compandPx[i-1]) - 1);
        } else if (pCurve->compandXData[i-1] > 0 && pCurve->compandPx[i-1] > 0){
            pCurve->compandXData[i] = pCurve->compandXData[i-1] + (1 << pCurve->compandPx[i-1]);
        } else if (pCurve->compandXData[i-1] > 0 && pCurve->compandPx[i-1] == 0){
            pCurve->compandXData[i] = pCurve->compandXData[i-1];
        } else {
            TRACE(Semu_INFO, "%s: invalid paramter\n", __func__);
            return RET_INVALID_PARM;
        }
    }

    uint16_t expandXValue[34]={0, 1023, 1279, 1279, 1535, 1791, 2047, 2303, 2431, 2559, 2687, 2815, 2943, 3007, 3071,
                            3135, 3199, 3263, 3327, 3391, 3455, 3519, 3583, 3647, 3711, 3775, 3839, 3903, 3967, 3999,
                            4031, 4063, 4095, 4095};
    uint32_t expandYValue[34]={0, 1023, 2047, 2047, 4095, 8191, 12287, 16383, 20479, 24575, 32767, 40959, 49151, 57343,
                            65535, 81919, 98303, 114687, 131071, 163839, 196607, 262143, 393215, 524287, 786431, 1048575,
                            1572863, 2097151, 3145727, 4194303, 8388607, 12582911, 16777215, 16777215};
    float slope[34] = {0};
    for(int i=0; i<34; i++){
        slope[i] = (expandYValue[i+1] - expandYValue[i])/(expandXValue[i+1] - expandXValue[i]);
    }

    for(int i=1; i<65; i++){
        for(int j=1; j<34; j++){
            if(pCurve->compandXData[i] >= expandXValue[j-1] && pCurve->compandXData[i] < expandXValue[j]){
                pCurve->compandYData[i] = expandYValue[j-1] + (pCurve->compandXData[i] - expandXValue[j-1]) * slope[j-1];
            }
        }
    }

    TRACE(Semu_INFO, "%s: (exit)\n", __func__);
    return (result);
}

RESULT Semu_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
	RESULT result = RET_SUCCESS;
	static const char SensorName[16] = "semu";
	TRACE( Semu_INFO, "%s (enter)\n", __func__);

	if ( pIsiSensor != NULL ) {
		pIsiSensor->pszName                             = SensorName;
		pIsiSensor->pIsiCreateIss                       = Semu_IsiCreateIss;
		pIsiSensor->pIsiOpenIss                         = Semu_IsiOpenIss;
		pIsiSensor->pIsiCloseIss                        = Semu_IsiCloseIss;
		pIsiSensor->pIsiReleaseIss                      = Semu_IsiReleaseIss;
		pIsiSensor->pIsiReadRegIss                      = Semu_IsiReadRegIss;
		pIsiSensor->pIsiWriteRegIss                     = Semu_IsiWriteRegIss;
		pIsiSensor->pIsiGetModeIss                      = Semu_IsiGetModeIss;
//		pIsiSensor->pIsiSetModeIss                      = Semu_IsiSetModeIss;
		pIsiSensor->pIsiEnumModeIss                     = Semu_IsiEnumModeIss;
		pIsiSensor->pIsiGetCapsIss                      = Semu_IsiGetCapsIss;
		pIsiSensor->pIsiCheckConnectionIss              = Semu_IsiCheckConnectionIss;
		pIsiSensor->pIsiGetRevisionIss                  = Semu_IsiGetRevisionIss;
		pIsiSensor->pIsiSetStreamingIss                 = Semu_IsiSetStreamingIss;

		/* AEC */
		pIsiSensor->pIsiGetAeBaseInfoIss                = Semu_pIsiGetAeBaseInfoIss;
		pIsiSensor->pIsiGetAGainIss                     = Semu_IsiGetAGainIss;
		pIsiSensor->pIsiSetAGainIss                     = Semu_IsiSetAGainIss;
		pIsiSensor->pIsiGetDGainIss                     = Semu_IsiGetDGainIss;
		pIsiSensor->pIsiSetDGainIss                     = Semu_IsiSetDGainIss;
		pIsiSensor->pIsiGetIntTimeIss                   = Semu_IsiGetIntTimeIss;
		pIsiSensor->pIsiSetIntTimeIss                   = Semu_IsiSetIntTimeIss;
		pIsiSensor->pIsiGetFpsIss                       = Semu_IsiGetFpsIss;
		pIsiSensor->pIsiSetFpsIss                       = Semu_IsiSetFpsIss;

		/* SENSOR ISP */
		pIsiSensor->pIsiGetIspStatusIss                 = Semu_IsiGetIspStatusIss;
		pIsiSensor->pIsiSetWBIss                        = Semu_IsiSetWBIss;
		pIsiSensor->pIsiGetWBIss                        = Semu_IsiGetWBIss;
		pIsiSensor->pIsiSetBlcIss                       = Semu_IsiSetBlcIss;
		pIsiSensor->pIsiGetBlcIss                       = Semu_IsiGetBlcIss;

		/* SENSOE OTHER FUNC*/
		pIsiSensor->pIsiSetTpgIss                       = Semu_IsiSetTpgIss;
		pIsiSensor->pIsiGetTpgIss                       = Semu_IsiGetTpgIss;
		pIsiSensor->pIsiGetExpandCurveIss               = Semu_IsiGetExpandCurveIss;

		/* AF */
		pIsiSensor->pIsiFocusCreateIss                  = NULL;
		pIsiSensor->pIsiFocusReleaseIss                 = NULL;
		pIsiSensor->pIsiFocusGetCalibrateIss            = NULL;
		pIsiSensor->pIsiFocusSetIss                     = NULL;
		pIsiSensor->pIsiFocusGetIss                     = NULL;

	} else {
		result = RET_NULL_POINTER;
	}

	TRACE( Semu_INFO, "%s (exit)\n", __func__);
	return ( result );
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t Semu_IsiCamDrvConfig = {
	.cameraDriverID      = 0x5801,
	.pIsiGetSensorIss    = Semu_IsiGetSensorIss,
};
