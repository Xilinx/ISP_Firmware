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

/* VeriSilicon 2022 */

#ifndef ISI_VVSENSOR_H
#define ISI_VVSENSOR_H

#include <isi_common.h>

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

#define ISI_SENSOR_HIST_BIN_NUM_MAX	(256)
#define ISI_SENSOR_EXP_ROI_NUM_MAX	(64)
#define ISI_EXP_NUM_MAX			(4)

/******************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor HDR mode.
 *
 *****************************************************************************/
typedef enum IsiSensorHdrMode_e {
	ISI_SENSOR_MODE_LINEAR,        /**< Linear mode */
	ISI_SENSOR_MODE_HDR_STITCH,    /**< Stitching HDR mode */
	ISI_SENSOR_MODE_HDR_NATIVE,    /**< Native HDR mode */
} IsiSensorHdrMode_t;

/******************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor AF mode.
 *
 *****************************************************************************/
typedef enum IsiSensorAfMode_e {
	ISI_SENSOR_AF_MODE_NOTSUPP,    /**< Auto focus not supported */
	ISI_SENSOR_AF_MODE_CDAF,       /**< CDAF mode */
	ISI_SENSOR_AF_MODE_PDAF,       /**< PDAF mode */
} IsiSensorAfMode_t;

/******************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor frame
 *          combination in stitch HDR mode.
 *
 *****************************************************************************/
typedef enum IsiSensorStitchingMode_e {
	ISI_SENSOR_STITCHING_DUAL_DCG           = 0,  /**< dual DCG mode 3x12-bit */
	ISI_SENSOR_STITCHING_3DOL               = 1,  /**< dol3 frame 3x12-bit */
	ISI_SENSOR_STITCHING_LINEBYLINE         = 2,  /**< 3x12-bit line by line */
	ISI_SENSOR_STITCHING_16BIT_COMPRESS     = 3,  /**< 16-bit comp+12 */
	ISI_SENSOR_STITCHING_DUAL_DCG_NOWAIT    = 4,  /**< 2x12 dual DCG */
	ISI_SENSOR_STITCHING_2DOL               = 5,  /**< dol2 or 1 CG+VS 12-bit */
	ISI_SENSOR_STITCHING_L_AND_S            = 6,  /**< L+S 2x12-bit RAW */
	ISI_SENSOR_STITCHING_4DOL               = 7,  /**< dol4 frame 3x12-bit */
	ISI_SENSOR_STITCHING_MAX
} IsiSensorStitchingMode_t;

/******************************************************************************/
/**
 * @brief   VsCamISI enum of sensor frame
 *          combination in native HDR mode.
 *
 *****************************************************************************/
typedef enum IsiSensorNativeMode_e {
	ISI_SENSOR_NATIVE_DCG		= 0,    /**< hcg and lcg combine in sensor*/
	ISI_SENSOR_NATIVE_L_AND_S	= 1,    /**< L+S combine in sensor*/
	ISI_SENSOR_NATIVE_3DOL		= 2,    /**< 3dol combine in sensor*/
	ISI_SENSOR_NATIVE_4DOL		= 3,    /**< 4dol combine in sensor*/
	ISI_SENSOR_NATIVE_DCG_SPD_VS	= 4,    /**< 4dol in sensor */
	ISI_SENSOR_NATIVE_MAX
} IsiSensorNativeMode_t;

/******************************************************************************/
/**
 * @brief VsCamISI structure to define the control information
 *        transferred through the SCCB bus
 *	to the sensor.

 *
 *****************************************************************************/
typedef struct IsiSensorSccbCfg_s {
	uint8_t slaveAddr;	/**< Sensor slave address */
	uint8_t addrByte;	/**< Width of sensor register address */
	uint8_t dataByte;	/**< Width of sensor register value */
} IsiSensorSccbCfg_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor TPG configuration.
 *
 *****************************************************************************/
typedef struct IsiSensorTpg_s {
	uint32_t enable;	/**< Whether to enable sensor test pattern */
	uint32_t pattern;	/**< Sensor test pattern information. */
} IsiSensorTpg_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define compand curve parameters.
 *
 *****************************************************************************/
typedef struct IsiSensorCompandCurve_s {
	uint32_t xBit;	/**< Compand curve input data bit width */
	uint32_t yBit;	/**< Compand curve output data bit width */
	uint8_t	compandPx[64]; /**< X axis interval */
	uint32_t compandXData[65];	/**< Compand curve X axis - 65 points */
	uint32_t compandYData[65];	/**< Compand curve Y axis - 65 points */
} IsiSensorCompandCurve_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define expand curve parameters.
 *
 *****************************************************************************/
typedef struct IsiSensorCompress_s {
	uint32_t enable;	/**< Whether to compress the sensor data */
	uint32_t xBit;	/**< Bit width of the compress curve input data */
	uint32_t yBit;	/**< Bit width of the compress curve output data */
} IsiSensorCompress_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor resolution.
 *
 *****************************************************************************/
typedef struct IsiSensorSize_s {
	uint32_t boundsWidth;	/**< Whole image width */
	uint32_t boundsHeight;	/**< Whole image height */
	uint32_t top;	/**< Effective data offset height pixel */
	uint32_t left;	/**< Effective data offset width pixel */
	uint32_t width;	/**< Effective data width */
	uint32_t height;	/**< Effective data height */
} IsiSensorSize_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure of sensor BLC configuration.
 *
 *****************************************************************************/
typedef struct IsiSensorBlc_s {
	uint32_t red;	/**< BLC for the R channel */
	uint32_t gr;	/**< BLC for the GR channel */
	uint32_t gb;	/**< BLC for the GB channel */
	uint32_t blue;	/**< BLC for the B channel */
} IsiSensorBlc_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure of sensor WB configuration.
 *
 *****************************************************************************/
typedef struct IsiSensorWb_s {
	float32_t rGain;	/**< WB gain for the R channel */
	float32_t grGain;	/**< WB gain for the GR channel */
	float32_t gbGain;	/**< WB gain for the GB channel */
	float32_t bGain;	/**< WB gain for the B channel */
} IsiSensorWb_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the range of sensor parameters.
 *
 *****************************************************************************/
typedef struct IsiSensorRange_s {
	uint32_t min;	/**< Minimum value */
	uint32_t max;	/**< Maximum value */
} IsiSensorRange_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the auto fps configuration.
 *
 *****************************************************************************/
typedef struct IsiSensorAutoFps_s {
	IsiSensorRange_t afpsRange;	/**< Auto fps range of the sensor */
	uint32_t	maxGain;	/**< Maximum gain of the sensor */
} IsiSensorAutoFps_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define sensor HIST parameters.
 *
 *****************************************************************************/
typedef struct IsiSensorHist_s {
	uint8_t num;	/**< Number of HIST bins */
	IsiSensorWin_t roi;	/**< ROI window parameters */
	/**< Values of sensor HIST bins */
	uint32_t bins[ISI_SENSOR_HIST_BIN_NUM_MAX];
} IsiSensorHist_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor meanluma of all ROI windows.
 *
 *****************************************************************************/
typedef struct IsiSensorMeanLuma_s {
	uint8_t roiNum;	/**< Number of ROIs */
	uint32_t meanLuma[ISI_SENSOR_EXP_ROI_NUM_MAX]; /**< Meanluma of ROI */
	IsiSensorWin_t roiWin[ISI_SENSOR_EXP_ROI_NUM_MAX]; /**< ROI windows */
} IsiSensorMeanLuma_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor AE information.
 *
 *****************************************************************************/
typedef struct IsiSensorAeInfo_s {
	IsiRange_t intTimeRange[ISI_EXP_NUM_MAX]; /**< Int time range */
	IsiGainInfo_t aGainRange[ISI_EXP_NUM_MAX]; /**< Again range */
	IsiGainInfo_t dGainRange[ISI_EXP_NUM_MAX]; /**< Dgain range */
	uint32_t intTimeDelayFrame;	/**< Delay frame of Integration time */
	uint32_t gainDelayFrame;	/**< Delay frame of Again/Dgain */
} IsiSensorAeInfo_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define sensor mode information.
 *
 *****************************************************************************/
typedef struct IsiSensorMode_s {
	uint32_t	index;	/**< Sensor mode index */
	IsiSensorSize_t	size;	/**< Sensor resolution information */
	IsiSensorHdrMode_t	hdrMode;	/**< Sensor HDR mode */
	IsiSensorStitchingMode_t stitchingMode;	/**< Sensor stitching mode */
	IsiSensorNativeMode_t	nativeMode;	/**< Sensor native HDR mode */
	IsiSensorCompress_t	compress;	/**< Sensor compress information */
	IsiBayerPattern_t	bayerPattern;	/**< Sensor bayer pattern. */
	IsiSensorAeInfo_t	aeInfo;	/**< Sensor AE information */
	IsiSensorAfMode_t	afMode;	/**< Sensor AF mode */
	uint32_t	dataType;	/**< Sensor data type */
	uint32_t	mipiLane;	/**< Sensor mipilane number */
	uint32_t	fps;	/**< Sensor fps info */
	uint32_t	bitWidth;	/**< Sensor output bitwidth */
	void *pregData;	/**< Sensor register initial array pointer */
	uint32_t	regCount;	/**< Sensor register data count */
} IsiSensorMode_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to enum the sensor mode.
 *
 *****************************************************************************/
typedef struct IsiSensorEnumMode_s {
	uint32_t	index;	/**< Sensor mode index */
	IsiSensorMode_t mode;	/**< Sensor mode information */
} IsiSensorEnumMode_t;

/** @} 01_cam_isi */

#ifdef __cplusplus
}
#endif

#endif /* ISI_VVSENSOR_H */
