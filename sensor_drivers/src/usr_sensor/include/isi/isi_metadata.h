/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2022 Vivante Corporation
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

#ifndef ISI_METADATA_H
#define ISI_METADATA_H
#include "types.h"
#include <isi_vvsensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup 01_cam_isi VsCamISI E05C01 ISI_sensorCtrl Definitions
 * @brief The ISI_sensorCtrl API mainly includes operations for sensor such as
 * open/close, create/release, stream on/off, check connection status, obtain
 * sensor configuration, and set/get the WB/BLS test pattern of sensors. it is
 * mainly controlled and called by the upper layer E01 VsCamDevice.
 * @{
 */

#define ISI_METADATA_WIN_NUM_MAX	(3)

/******************************************************************************/
/**
 * @brief   VsCamISI union type of metadata attribute.
 *
 *****************************************************************************/
typedef union IsiMetadataAttr_s {
	struct {
		uint32_t support  : 1;	/**< bit 0: 0-disable 1-enable */
		uint32_t regInfo  : 1;	/**< bit 1 */
		uint32_t expTime  : 1;	/**< bit 2 */
		uint32_t again    : 1;	/**< bit 3 */
		uint32_t dgain    : 1;	/**< bit 4 */
		uint32_t bls      : 1;	/**< bit 5 */
		uint32_t hist     : 1;	/**< bit 6 */
		uint32_t meanLuma : 1;	/**< bit 7 */
		uint32_t frameCRC : 1;	/**< bit 8 */
		uint32_t reservedEnable : 23; /**< bit 9:31, reserved */
	} subAttr;	/**< Sub attribution */
	uint32_t mainAttr;	/**< Main attribution */
} IsiMetadataAttr_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the metadata window information.
 *
 *****************************************************************************/
typedef struct IsiMetadataWinInfo_s {
	uint8_t winNum;	/**< Number of metadata window */
	IsiSensorWin_t metaWin[ISI_METADATA_WIN_NUM_MAX]; /**< Metadata window info */
} IsiMetadataWinInfo_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the metadata parser information.
 *
 *****************************************************************************/
typedef struct IsiMetadataParserInfo_s {
	IsiMetadataAttr_t validMask;	/**< Sensor metadata enabling situation */
	uint32_t regNum;	/**< Number of registers to be parsed */
	IsiSensorReg_t *pReg; /**< Start address of regs to parse */
	uint8_t expFrmNum;	/**< Number of exposure frames */
	uint32_t frmCRC;	/**< Frame CRC value */
	IsiSensorIntTime_t expTime;	/**< Sensor exposure time */
	IsiSensorGain_t	aGain;	/**< Sensor analog gain */
	IsiSensorGain_t	dGain;	/**< Sensor digital gain */
	IsiSensorBlc_t	blc;	/**< sensor BLC */
	IsiSensorHist_t	hist;	/**< sensor HIST */
	IsiSensorMeanLuma_t meanLuma;	/**< Meanluma in all ROI windows */
} IsiMetadataParserInfo_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor metadata information.
 *
 *****************************************************************************/
typedef struct IsiSensorMetadata_s {
	uint32_t chipId;	/**< sensor version id */
	uint32_t frmCount;	/**< Sensor frame count */
	IsiMetadataParserInfo_t data; /**< Sensor metadata parser info */
} IsiSensorMetadata_t;

/** @} 01_cam_isi */

#ifdef __cplusplus
}
#endif

#endif /* ISI_METADATA_H */
