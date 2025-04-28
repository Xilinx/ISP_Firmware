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
#ifndef __ISI_METADATA_H__
#define __ISI_METADATA_H__

#include <ebase/types.h>
#include <isi/isi_vvsensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ISI_METADATA_WIN_NUM_MAX	(3)

typedef union {
	struct {
		uint32_t	support		: 1;
		uint32_t	regInfo		: 1;
		uint32_t	expTime		: 1;
		uint32_t	again		: 1;
		uint32_t	dgain		: 1;
		uint32_t	bls		: 1;
		uint32_t	hist		: 1;
		uint32_t	meanLuma	: 1;
		uint32_t	reservedEnable	: 23;
	} subAttr;
	uint32_t		mainAttr;
} IsiMetadataAttr_t;

typedef struct {
	uint8_t			winNum;
	IsiSensorWin_t		metaWin[ISI_METADATA_WIN_NUM_MAX];
} IsiMetadataWinInfo_t;

typedef struct {
	IsiMetadataAttr_t	validMask;
	uint32_t		regNum;
	IsiSensorReg_t		*RegPtr;
	uint8_t			expFrmNum;
	IsiSensorIntTime_t	expTime;
	IsiSensorGain_t		aGain;
	IsiSensorGain_t		dGain;
	IsiSensorBlc_t		blc;
	IsiSensorHist_t		hist;
	IsiSensorMeanLuma_t	meanLuma;
} IsiMetadataParserInfo_t;

typedef struct {
	uint32_t		chipId;
	uint32_t		frmCount;
	IsiMetadataParserInfo_t	data;
} IsiSensorMetadata_t;

#ifdef __cplusplus
}
#endif

#endif
