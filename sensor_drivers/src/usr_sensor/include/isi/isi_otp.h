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

#ifndef ISI_OTP_H
#define ISI_OTP_H
#include "types.h"
#include <isi_common.h>
#define ISI_OTP_LSC_TABLE_NUM		(33)
#define PDAF_OTP_FOCAL_SIZE		(48)

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

/******************************************************************************/
/**
 * @brief   VsCamISI enumeration type of color temperature.
 *
 *****************************************************************************/
typedef enum IsiColorTemperature_e {
	ISI_COLOR_TEMPERATURE_3100K	= 0,    /**< Color temperature of 3100K */
	ISI_COLOR_TEMPERATURE_4000K	= 1,    /**< Color temperature of 4000K */
	ISI_COLOR_TEMPERATURE_5800K	= 2,    /**< Color temperature of 5800K */
	ISI_COLOR_TEMPERATURE_MAX	= 3     /**< Number of color temperatures */
} IsiColorTemperature_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the OTP LSC calibration data.
 *
 *****************************************************************************/
typedef struct IsiOTPLSC_s {
	IsiColorTemperature_t colorTemperature;	/**< LSC color temp */
	uint16_t r[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];	/**< R channel */
	uint16_t gr[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];	/**< GR channel */
	uint16_t gb[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];	/**< GB channel */
	uint16_t b[ISI_OTP_LSC_TABLE_NUM][ISI_OTP_LSC_TABLE_NUM];	/**< B channel */
} IsiOTPLSC_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the OTP AWB calibration data.
 *
 *****************************************************************************/
typedef struct IsiOTPAWB_s {
	IsiColorTemperature_t colorTemperature;	/**< AWB color temp */
	uint16_t r;	/**< AWB R channel */
	uint16_t gr;	/**< AWB GR channel */
	uint16_t gb;	/**< AWB GB channel */
	uint16_t b;	/**< AWB B channel */
	uint16_t rgRatio;	/**< R/G ratio */
	uint16_t bgRatio;	/**< B/G ratio */
} IsiOTPAWB_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the OTP light source calibration data.
 *
 *****************************************************************************/
typedef struct IsiOTPLightSource_s {
	IsiColorTemperature_t colorTemperature;	/**< Light source color temp */
	uint16_t xCIE;	/**< X-coordinate in CIE */
	uint16_t yCIE;	/**< Y-coordinate in CIE */
	uint16_t intensity;	/**< Light source intensity */
} IsiOTPLightSource_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the CDAF focus information.
 *
 *****************************************************************************/
typedef struct IsiOTPCdaf_s {
	uint16_t minFocal;	/**< Minimum focus data */
	uint16_t maxFocal;	/**< Maximum focus data */
} IsiOTPCdaf_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the PDAF focus information.
 *
 *****************************************************************************/
typedef struct IsiOTPPdaf_s {
	int pdFocal[PDAF_OTP_FOCAL_SIZE];	/**< PD slope calibration data */
} IsiOTPPdaf_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the OTP AF calibration data.
 *
 *****************************************************************************/
typedef struct IsiOTPAFData_s {
	uint32_t	otpVersion;	/**< OTP version */
	bool	otpFocusEnable;	/**< Whether to enable OTP AF */
	IsiOTPCdaf_t cdafOtp;	/**< OTP focus data range of CDAF */
	IsiOTPPdaf_t pdafOtp;	/**< PD slope calibration data of PDAF */
} IsiOTPAFData_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the OTP module information.
 *
 *****************************************************************************/
typedef struct IsiOTPModuleInformation_s {
	uint16_t hwVersion;	/**< Hardware version */
	uint16_t eepromRevision;	/**< EEPROM revision */
	uint16_t sensorRevision;	/**< Image sensor revision */
	uint16_t tlensRevision;	/**< TLens revision */
	uint16_t ircfRevision;	/**< IRCF revision */
	uint16_t lensRevision;	/**< Lens revision */
	uint16_t caRevision;	/**< Contact assembly revision */
	uint16_t moduleInteID;	/**< Module integrator ID */
	uint16_t factoryID;	/**< Factory ID */
	uint16_t mirrorFlip;	/**< Image mirror and flip */
	uint16_t tlensSlaveID;	/**< TLens slave ID */
	uint16_t eepromSlaveID;	/**< EEPROM slave ID */
	uint16_t sensorSlaveID;	/**< Image sensor slave ID */
	uint8_t	sensorID[11];	/**< Image sensor ID */
	uint16_t manuDateYear;	/**< Manufacture date (Year) */
	uint16_t manuDateMonth;	/**< Manufacture date (Month) */
	uint16_t manuDateDay;	/**< Manufacture date (Date) */
	uint8_t	barcodeModuleSN[12];	/**< Barcode-Module SN */
	uint16_t mapTotalSize;	/**< Total EEPROM map size (bytes) */
} IsiOTPModuleInformation_t;

/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the OTP information.
 *
 *****************************************************************************/
typedef struct IsiOTP_s {
	IsiOTPModuleInformation_t otpInformation;	/**< OTP module info */
	bool	otpLSCEnable;	/**< Enable OTP LSC calibration */
	bool	otpAwbEnable;	/**< Enable OTP AWB calibration */
	bool	otpLightSourceEnable;	/**< Enable OTP light source cal */
	bool	otpFocusEnable;	/**< Enable OTP focus calibration */
	uint8_t	lscNum;	/**< LSC color temp count */
	uint8_t	awbNum;	/**< AWB color temp count */
	uint8_t	goldenAwbNum;	/**< Golden AWB color temp count */
	uint8_t	lightSourceNum;	/**< Light source color temp count */
	IsiOTPLSC_t *pLscData;	/**< OTP LSC calibration data */
	IsiOTPAWB_t *pAwbData;	/**< OTP AWB calibration data */
	IsiOTPAWB_t *pGoldenAwbData;	/**< OTP golden AWB cal data */
	IsiOTPLightSource_t *pLightSourceData;	/**< OTP light source data */
	IsiOTPAFData_t	focus;	/**< OTP focus calibration data */
} IsiOTP_t;

/** @} 01_cam_isi */

#ifdef __cplusplus
}
#endif

#endif
