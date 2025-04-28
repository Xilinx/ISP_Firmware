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
#ifndef __ISI_COMMON_H__
#define __ISI_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/*!
 * interface version
 * =================
 * please increment the version if you add something new to the interface.
 * This helps upper layer software to deal with different interface versions.
 */
/*****************************************************************************/
#define ISI_INTERFACE_VERSION	(6)

/*****************************************************************************/
/* capabilities / configuration
 *****************************************************************************/
#define ISI_BUSWIDTH_8BIT_ZZ	(0x00000001)
#define ISI_BUSWIDTH_8BIT_EX	(0x00000002)
#define ISI_BUSWIDTH_10BIT_EX	(0x00000004)
#define ISI_BUSWIDTH_10BIT_ZZ	(0x00000008)
#define ISI_BUSWIDTH_12BIT	(0x00000010)
#define ISI_BUSWIDTH_10BIT	(ISI_BUSWIDTH_10BIT_EX)
#define ISI_ITF_TYPE_MIPI	(0x00000001)
#define ISI_ITF_TYPE_LVDS	(0x00000002)
#define ISI_ITF_TYPE_DVP	(0x00000004)
#define ISI_ITF_TYPE_BT601	(0x00000008)
#define ISI_ITF_TYPE_BT656	(0x00000010)
#define ISI_ITF_TYPE_BT1120	(0x00000020)
#define ISI_MODE_YUV		(0x00000001)
#define ISI_MODE_MONO		(0x00000002)
#define ISI_MODE_BAYER		(0x00000004)
#define ISI_MODE_DATA		(0x00000008)
#define ISI_MIPI_OFF		(0x80000000)
#define ISI_MIPI_1LANES		(0x00000001)
#define ISI_MIPI_2LANES		(0x00000002)
#define ISI_MIPI_4LANES		(0x00000004)
#define ISI_MIPI_APHY		(0x00000008)
#define ISI_MIPI_CPHY		(0x00000010)
#define ISI_MIPI_DPHY		(0x00000020)
#define ISI_PDAF_TYPE3		(0x00000001)
#define ISI_MOTOR_VCM		(0x00000001)
#define ISI_MOTOR_STEP		(0x00000002)
#define ISI_FPS_QUANTIZE	(1000)

/*****************************************************************************/
/**
 * IsiSensorInputFormat_t
 *
 * @brief mode of awb control to handle whitebalance during integration of AR082x
 */
/*****************************************************************************/
typedef enum {
	ISI_FORMAT_YUV420P_8		= 0,
	ISI_FORMAT_YUV420SP_8		= 1,
	ISI_FORMAT_YUV420P_10		= 2,
	ISI_FORMAT_YUV420SP_10		= 3,
	ISI_FORMAT_YUV422P_8		= 4,
	ISI_FORMAT_YUV422SP_8		= 5,
	ISI_FORMAT_YUV422P_10		= 6,
	ISI_FORMAT_YUV422SP_10		= 7,
	ISI_FORMAT_RAW_8		= 8,
	ISI_FORMAT_RAW_10		= 9,
	ISI_FORMAT_RAW_12		= 10,
	ISI_FORMAT_RAW_14		= 11,
	ISI_FORMAT_RAW_16		= 12,
	ISI_FORMAT_RAW_20		= 13,
	ISI_FORMAT_RAW_24		= 14,
	ISI_FORMAT_MONO_8		= 15,
	DUMMY_ISI_FORMAT		= 0xdeadfeed
} IsiSensorInputFormat_t;

/*****************************************************************************/
/**
 * IsiSensorAwbMode_t
 *
 * @brief mode of awb control to handle whitebalance during integration of AR082x
 */
/*****************************************************************************/
typedef enum {
	ISI_SENSOR_AWB_MODE_NORMAL	= 0,
	ISI_SENSOR_AWB_MODE_SENSOR	= 1,
	DUMMY_ISI_002			= 0xdeadfeed
} IsiSensorAwbMode_t;

/*****************************************************************************/
/**
 * IsiColorComponent_t
 *
 * @brief color components
 */
/*****************************************************************************/
typedef enum {
	ISI_COLOR_COMPONENT_RED		= 0,
	ISI_COLOR_COMPONENT_GREENR	= 1,
	ISI_COLOR_COMPONENT_GREENB	= 2,
	ISI_COLOR_COMPONENT_BLUE	= 3,
	ISI_COLOR_COMPONENT_MAX		= 4,
	DUMMY_ISI_003			= 0xdeadfeed
} IsiColorComponent_t;

/*****************************************************************************/
/**
 * IsiBayerPattern_t
 *
 * @brief Bayer pattern of sensor
 */
/*****************************************************************************/
typedef enum {
	ISI_BPAT_RGGB			= 0x00,
	ISI_BPAT_GRBG			= 0x01,
	ISI_BPAT_GBRG			= 0x02,
	ISI_BPAT_BGGR			= 0x03,
	ISI_BPAT_BGGIR			= 0x10,
	ISI_BPAT_GRIRG			= 0x11,
	ISI_BPAT_RGGIR			= 0x12,
	ISI_BPAT_GBIRG			= 0x13,
	ISI_BPAT_GIRRG			= 0x14,
	ISI_BPAT_IRGGB			= 0x15,
	ISI_BPAT_GIRBG			= 0x16,
	ISI_BPAT_IRGGR			= 0x17,
	ISI_BPAT_RGIRB			= 0x18,
	ISI_BPAT_GRBIR			= 0x19,
	ISI_BPAT_IRBRG			= 0x20,
	ISI_BPAT_BIRGR			= 0x21,
	ISI_BPAT_BGIRR			= 0x22,
	ISI_BPAT_GBRIR			= 0x23,
	ISI_BPAT_IRRBG			= 0x24,
	ISI_BPAT_RIRGB			= 0x25,
	ISI_BPAT_RCCC			= 0x30,
	ISI_BPAT_RCCB			= 0x40,
	ISI_BPAT_RYYCY			= 0x50,
	DUMMY_ISI_004			= 0xdeadfeed
} IsiBayerPattern_t;

/*****************************************************************************/
/**
 * IsiSyncSignalPolarity_t
 *
 * @brief sensor dvp output H/V sync polarity
 */
/*****************************************************************************/
typedef enum {
	ISI_SYNC_POL_HIGH_ACIVE		= 0,
	ISI_SYNC_POL_LOW_ACIVE		= 1,
	DUMMY_ISI_005			= 0xdeadfeed
} IsiSyncSignalPolarity_t;

/*****************************************************************************/
/**
 * IsiSyncSignalPolarity_t
 *
 * @brief sensor output sample edge polarity
 */
/*****************************************************************************/
typedef enum {
	ISI_SAMPLE_EDGE_POL_NEGATIVE	= 0,
	ISI_SAMPLE_EDGE_POL_POSITIVE	= 1,
	DUMMY_ISI_006			= 0xdeadfeed
} IsiSampleEdgePolarity_t;

/*****************************************************************************/
/**
 * IsiSensorCCIRequence_t
 *
 * @brief sensor output ccir sequence.
 */
/*****************************************************************************/
typedef enum {
	ISI_SENSOR_CCIR_SEQUENCE_YCbYCr	= 0,
	ISI_SENSOR_CCIR_SEQUENCE_YCrYCb	= 1,
	ISI_SENSOR_CCIR_SEQUENCE_CbYCrY	= 2,
	ISI_SENSOR_CCIR_SEQUENCE_CrYCbY	= 3,
	DUMMY_ISI_007			= 0xdeadfeed
} IsiSensorCCIRSequence_t;

/*****************************************************************************/
/**
 * IsiI2cBitWidth_t
 *
 * @brief sensor I2C width information
 */
/*****************************************************************************/
typedef enum {
	ISI_I2C_NONE			= 0,
	ISI_I2C_8BIT			= 1,
	ISI_I2C_16BIT			= 2,
	DUMMY_ISI_008			= 0xdeadfeed
} IsiI2cBitWidth_t;

typedef struct {
	float32_t	min;
	float32_t	max;
	float32_t	step;
} IsiGainInfo_t;

typedef struct {
	float32_t	min;
	float32_t	max;
} IsiRange_t;

typedef struct {
	uint16_t	max;
	uint16_t	min;
} IsiExpLineRange_t;

typedef struct {
	float32_t	gain[4];
} IsiSensorGain_t;

typedef struct {
	float32_t	intTime[4];
} IsiSensorIntTime_t;

typedef struct {
	uint16_t	hStart;
	uint16_t	vStart;
	uint16_t	hSize;
	uint16_t	vSize;
} IsiSensorWin_t;

typedef struct {
	uint16_t	regAddr;
	uint16_t	regVal;
} IsiSensorReg_t;

typedef struct IsiSensorBuffer_s {
	uint8_t		*dataPtr;
	uint32_t	dataSize;
} IsiSensorBuffer_t;

#ifdef __cplusplus
}
#endif

#endif
