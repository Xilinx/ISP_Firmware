/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 VeriSilicon Holdings Co., Ltd. ("VeriSilicon")
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
/* VeriSilicon 2022 */

/**
 * @file isi_vvsensor.h
 *
 * @brief
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup isi_vvsensor CamerIc Driver API
 * @{
 *
 */
#ifndef __ISI_VVSENSOR_H__
#define __ISI_VVSENSOR_H__

#include <isi/isi_common.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ISI_SENSOR_HIST_BIN_NUM_MAX	(256)
#define ISI_SENSOR_EXP_ROI_NUM_MAX	(64)

typedef enum {
	ISI_SENSOR_MODE_LINEAR,
	ISI_SENSOR_MODE_HDR_STITCH,
	ISI_SENSOR_MODE_HDR_NATIVE,
	DUMMY_ISI_010 = 0xdeadfeed
} IsiSensorHdrMode_t;

typedef enum {
	ISI_SENSOR_AF_MODE_NOTSUPP,
	ISI_SENSOR_AF_MODE_CDAF,
	ISI_SENSOR_AF_MODE_PDAF,
	DUMMY_ISI_011 = 0xdeadfeed
} IsiSensorAfMode_t;

typedef enum {
	ISI_SENSOR_STITCHING_DUAL_DCG		= 0,
	ISI_SENSOR_STITCHING_3DOL		= 1,
	ISI_SENSOR_STITCHING_LINEBYLINE		= 2,
	ISI_SENSOR_STITCHING_16BIT_COMPRESS	= 3,
	ISI_SENSOR_STITCHING_DUAL_DCG_NOWAIT	= 4,
	ISI_SENSOR_STITCHING_2DOL		= 5,
	ISI_SENSOR_STITCHING_L_AND_S		= 6,
	ISI_SENSOR_STITCHING_4DOL		= 7,
	ISI_SENSOR_STITCHING_MAX		= 8,
	DUMMY_ISI_012				= 0xdeadfeed
} IsiSensorStitchingMode_t;

typedef enum {
	ISI_SENSOR_NATIVE_DCG			= 0,
	ISI_SENSOR_NATIVE_L_AND_S		= 1,
	ISI_SENSOR_NATIVE_3DOL			= 2,
	ISI_SENSOR_NATIVE_4DOL			= 3,
	ISI_SENSOR_NATIVE_DCG_SPD_VS		= 4,
	ISI_SENSOR_NATIVE_MAX			= 5,
	DUMMY_ISI_0013				= 0xdeadfeed
} IsiSensorNativeMode_t;

typedef struct {
	uint8_t				slaveAddr;
	uint8_t				addrByte;
	uint8_t				dataByte;
} IsiSensorSccbCfg_t;

typedef struct {
	uint32_t			enable;
	uint32_t			pattern;
} IsiSensorTpg_t;

typedef struct {
	uint32_t			xBit;
	uint32_t			yBit;
	uint8_t				compandPx[64];
	uint32_t			compandXData[65];
	uint32_t			compandYData[65];
} IsiSensorCompandCurve_t;

typedef struct {
	uint32_t			enable;
	uint32_t			xBit;
	uint32_t			yBit;
} IsiSensorCompress_t;

typedef struct {
	uint32_t			boundsWidth;
	uint32_t			boundsHeight;
	uint32_t			top;
	uint32_t			left;
	uint32_t			width;
	uint32_t			height;
} IsiSensorSize_t;

typedef struct {
	uint32_t			red;
	uint32_t			gr;
	uint32_t			gb;
	uint32_t			blue;
} IsiSensorBlc_t;

typedef struct {
	float32_t			rGain;
	float32_t			grGain;
	float32_t			gbGain;
	float32_t			bGain;
} IsiSensorWb_t;

typedef struct {
	uint32_t			min;
	uint32_t			max;
} IsiSensorRange_t;

typedef struct {
	IsiSensorRange_t		afpsRange;
	uint32_t			maxGain;
} IsiSensorAutoFps_t;

typedef struct {
	uint8_t				num;
	IsiSensorWin_t			roi;
	uint32_t			bins[ISI_SENSOR_HIST_BIN_NUM_MAX];
} IsiSensorHist_t;

typedef struct {
	uint8_t				roiNum;
	uint32_t			meanLuma[ISI_SENSOR_EXP_ROI_NUM_MAX];
	IsiSensorWin_t			roiWin[ISI_SENSOR_EXP_ROI_NUM_MAX];
} IsiSensorMeanLuma_t;

typedef struct {
	uint32_t			defaultVts;
	uint32_t			currentVts;
	uint32_t			oneLineExpTimeNs;
	uint32_t			maxIntegrationTime;
	uint32_t			minIntegrationTime;
	uint32_t			integrationAccuracy;
	uint32_t			maxGain;
	uint32_t			minGain;
	uint32_t			gainAccuracy;
	uint32_t			startExpValue;
	uint32_t			gainStep;
	uint32_t			maxFps;
	uint32_t			minFps;
	uint32_t			curFps;
	IsiSensorAutoFps_t		afpsInfo;
	uint32_t			intTimeDelayFrame;
	uint32_t			gainDelayFrame;
} IsiSensorAeInfo_t;

typedef struct {
	uint32_t			index;
	IsiSensorSize_t			size;
	IsiSensorHdrMode_t		hdrMode;
	IsiSensorStitchingMode_t	stitchingMode;
	IsiSensorNativeMode_t		nativeMode;
	IsiSensorCompress_t		compress;
	IsiBayerPattern_t		bayerPattern;
	IsiSensorAeInfo_t		aeInfo;
	IsiSensorAfMode_t		afMode;
	uint32_t			dataType;
	uint32_t			mipiLane;
	uint32_t			fps;
	uint32_t			bitWidth;
	void				*regDataPtr;
	uint32_t			regCount;
} IsiSensorMode_t;

typedef struct {
	uint32_t			index;
	IsiSensorMode_t			mode;
} IsiSensorEnumMode_t;

#ifdef __cplusplus
}
#endif

#endif
