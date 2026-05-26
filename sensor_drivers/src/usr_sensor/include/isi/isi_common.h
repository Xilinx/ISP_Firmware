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

#ifndef ISI_COMMON_H
#define ISI_COMMON_H

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

/**
 * @details Please increment the version if you add something
 * new to the interface.
 * This helps upper layer software to deal with different interface versions
*/

/** @brief ISI interface version*/
#define ISI_INTERFACE_VERSION			(6)

/** @brief Buswidth information*/
#define ISI_BUSWIDTH_8BIT_ZZ			(0x00000001)
#define ISI_BUSWIDTH_8BIT_EX			(0x00000002)
#define ISI_BUSWIDTH_10BIT_EX			(0x00000004)
#define ISI_BUSWIDTH_10BIT_ZZ			(0x00000008)
#define ISI_BUSWIDTH_12BIT			(0x00000010)
#define ISI_BUSWIDTH_10BIT			(ISI_BUSWIDTH_10BIT_EX)

/** @brief ISI interface type*/
#define ISI_ITF_TYPE_MIPI			(0x00000001U)
#define ISI_ITF_TYPE_LVDS			(0x00000002U)
#define ISI_ITF_TYPE_DVP			(0x00000004U)
#define ISI_ITF_TYPE_BT601			(0x00000008U)
#define ISI_ITF_TYPE_BT656			(0x00000010U)
#define ISI_ITF_TYPE_BT1120			(0x00000020U)

/** @brief Sensor output image type*/
#define ISI_MODE_YUV				(0x00000001U)
#define ISI_MODE_MONO				(0x00000002U)
#define ISI_MODE_BAYER				(0x00000004U)
#define ISI_MODE_DATA				(0x00000008U)

/** @brief MIPI lanes and type*/
#define ISI_MIPI_OFF				(0x80000000U)
#define ISI_MIPI_1LANES				(0x00000001U)
#define ISI_MIPI_2LANES				(0x00000002U)
#define ISI_MIPI_4LANES				(0x00000004U)
#define ISI_MIPI_APHY				(0x00000008U)
#define ISI_MIPI_CPHY				(0x00000010U)
#define ISI_MIPI_DPHY				(0x00000020U)

/** @brief Sensor PDAF type*/
#define ISI_PDAF_TYPE3				(0x00000001)

/** @brief AF sensor motor type*/
#define ISI_MOTOR_VCM				(0x00000001)
#define ISI_MOTOR_STEP				(0x00000002)

/** @brief Sensor fps and intergration time quantization*/
#define ISI_FPS_QUANTIZE			(1000U)
#define ISI_INTEGRATION_TIME_QUANTIZE		(1000000U)

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor input format.
 */
/*****************************************************************************/
typedef enum IsiSensorInputFormat_e {
	ISI_FORMAT_YUV420P_8 = 0,      /**< YUV 420P  8-bit */
	ISI_FORMAT_YUV420SP_8,         /**< YUV 420SP 8-bit */
	ISI_FORMAT_YUV420P_10,         /**< YUV 420P  10-bit */
	ISI_FORMAT_YUV420SP_10,        /**< YUV 420SP 10-bit */
	ISI_FORMAT_YUV422P_8,          /**< YUV 422P  8-bit */
	ISI_FORMAT_YUV422SP_8,         /**< YUV 422SP 8-bit */
	ISI_FORMAT_YUV422P_10,         /**< YUV 422P  10-bit */
	ISI_FORMAT_YUV422SP_10,        /**< YUV 422SP 10-bit */
	ISI_FORMAT_RAW_8,              /**< RAW_8 */
	ISI_FORMAT_RAW_10,             /**< RAW_10 */
	ISI_FORMAT_RAW_12,             /**< RAW_12 */
	ISI_FORMAT_RAW_14,             /**< RAW_14 */
	ISI_FORMAT_RAW_16,             /**< RAW_16 */
	ISI_FORMAT_RAW_20,             /**< RAW_20 */
	ISI_FORMAT_RAW_24,             /**< RAW_24 */
	ISI_FORMAT_MONO_8              /**< MONO_8 */
} IsiSensorInputFormat_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor AWB mode.
 */
/*****************************************************************************/
typedef enum IsiSensorAwbMode_e {
	ISI_SENSOR_AWB_MODE_NORMAL = 0,    /**< ISP AWB */
	ISI_SENSOR_AWB_MODE_SENSOR,         /**< Sensor AWB */
} IsiSensorAwbMode_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of color components.
 */
/*****************************************************************************/
typedef enum IsiColorComponent_e {
	ISI_COLOR_COMPONENT_RED		= 0,    /**< Color component R */
	ISI_COLOR_COMPONENT_GREENR	= 1,    /**< Color component GR */
	ISI_COLOR_COMPONENT_GREENB	= 2,    /**< Color component GB */
	ISI_COLOR_COMPONENT_BLUE	= 3,    /**< Color component B */
	ISI_COLOR_COMPONENT_MAX		= 4     /**< Number of Color components */
} IsiColorComponent_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor bayer pattern.
 */
/*****************************************************************************/
typedef enum IsiBayerPattern_e {
	ISI_BPAT_RGGB      = 0x00,    /**< RGGB */
	ISI_BPAT_GRBG      = 0x01,    /**< GRBG */
	ISI_BPAT_GBRG      = 0x02,    /**< GBRG */
	ISI_BPAT_BGGR      = 0x03,    /**< BGGR */
	ISI_BPAT_BGGIR     = 0x10,    /**< BGGIR */
	ISI_BPAT_GRIRG     = 0x11,    /**< GRIRG */
	ISI_BPAT_RGGIR     = 0x12,    /**< RGGIR */
	ISI_BPAT_GBIRG     = 0x13,    /**< GBIRG */
	ISI_BPAT_GIRRG     = 0x14,    /**< GIRRG */
	ISI_BPAT_IRGGB     = 0x15,    /**< IRGGB */
	ISI_BPAT_GIRBG     = 0x16,    /**< GIRBG */
	ISI_BPAT_IRGGR     = 0x17,    /**< IRGGR */
	ISI_BPAT_RGIRB     = 0x18,    /**< RGIRB */
	ISI_BPAT_GRBIR     = 0x19,    /**< GRBIR */
	ISI_BPAT_IRBRG     = 0x20,    /**< IRBRG */
	ISI_BPAT_BIRGR     = 0x21,    /**< BIRGR */
	ISI_BPAT_BGIRR     = 0x22,    /**< BGIRR */
	ISI_BPAT_GBRIR     = 0x23,    /**< GBRIR */
	ISI_BPAT_IRRBG     = 0x24,    /**< IRRBG */
	ISI_BPAT_RIRGB     = 0x25,    /**< RIRGB */
	ISI_BPAT_RCCC      = 0x30,    /**< RCCC  */
	ISI_BPAT_RCCB      = 0x40,    /**< RCCB  */
	ISI_BPAT_RYYCY     = 0x50,    /**< RYYCY */
} IsiBayerPattern_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define sensor gain information.
 */
/*****************************************************************************/
typedef struct IsiGainInfo_s {
	float32_t min;	/**< Minimum gain */
	float32_t max;	/**< Maximun gain */
	float32_t step;	/**< Gain step */
} IsiGainInfo_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the range of variables.
 */
/*****************************************************************************/
typedef struct IsiRange_s {
	float32_t min;	/**< Minimum value */
	float32_t max;	/**< Maximum value */
} IsiRange_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the range of exposure line.
 */
/*****************************************************************************/
typedef struct IsiExpLineRange_s {
	uint16_t max;	/**< Maximum exposure line */
	uint16_t min;	/**< Minimum exposure line */
} IsiExpLineRange_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor gain
 *          of different exposures.
 */
/*****************************************************************************/
typedef struct IsiSensorGain_s {
	float32_t      gain[4];	/**< Gain of different exposures */
} IsiSensorGain_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor
 *          integration time of different exposures.
 */
/*****************************************************************************/
typedef struct IsiSensorIntTime_s {
	float32_t      intTime[4];	/**< Integration time of different exposures */
} IsiSensorIntTime_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the size and position
 *          of a sensor metadata window.
 */
/*****************************************************************************/
typedef struct IsiSensorWin_s {
	uint16_t hStart;	/**< Horizontal start */
	uint16_t vStart;	/**< Vertical start */
	uint16_t hSize;	/**< Width */
	uint16_t vSize;	/**< Height */
} IsiSensorWin_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the address
 *          and value of sensor registers.
 */
/*****************************************************************************/
typedef struct IsiSensorReg_s {
	uint16_t regAddr;	/**< Register address */
	uint16_t regVal;	/**< Register value */
} IsiSensorReg_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define a buffer.
 */
/*****************************************************************************/
typedef struct IsiSensorBuffer_s {
	uint8_t *pdata;	/**< Pointer to the buffer data */
	uint32_t dataSize;	/**< The size of the data */
} IsiSensorBuffer_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor sync signal polarity.
 */
/*****************************************************************************/
typedef enum IsiSyncSignalPolarity_e {
	ISI_SYNC_POL_HIGH_ACIVE   = 0,    /**< High active */
	ISI_SYNC_POL_LOW_ACIVE    = 1     /**< Low active */
} IsiSyncSignalPolarity_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor sample edge polarity.
 */
/*****************************************************************************/
typedef enum IsiSampleEdgePolarity_e {
	ISI_SAMPLE_EDGE_POL_NEGATIVE    = 0,  /**< Negative sample edge polarity */
	ISI_SAMPLE_EDGE_POL_POSITIVE    = 1   /**< Positive sample edge polarity */
} IsiSampleEdgePolarity_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor CCIR sequence type.
 */
/*****************************************************************************/
typedef enum IsiSensorCCIRSequence_e {
	ISI_SENSOR_CCIR_SEQUENCE_YCbYCr   = 0,    /**< YCbYCr */
	ISI_SENSOR_CCIR_SEQUENCE_YCrYCb   = 1,    /**< YCrYCb */
	ISI_SENSOR_CCIR_SEQUENCE_CbYCrY   = 2,    /**< CbYCrY */
	ISI_SENSOR_CCIR_SEQUENCE_CrYCbY   = 3,    /**< CrYCbY */
} IsiSensorCCIRSequence_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of I2C bit width.
 */
/*****************************************************************************/
typedef enum IsiI2cBitWidth_e {
	ISI_I2C_NONE    = 0,    /**< None I2C */
	ISI_I2C_8BIT    = 1,    /**< 8bit width*/
	ISI_I2C_16BIT   = 2,    /**< 16bit width*/
} IsiI2cBitWidth_t;

/** @} 01_cam_isi */

#ifdef __cplusplus
}
#endif

#endif /* ISI_COMMON_H */
