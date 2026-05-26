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

#ifndef ISI_ISS_H
#define ISI_ISS_H

#include "types.h"
#include "return_codes.h"
#include "isi.h"

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

typedef RESULT(*IsiCreateIss_t) (IsiSensorInstanceConfig_t *pConfig,
				IsiSensorHandle_t *pHandle);
typedef RESULT(*IsiOpenIss_t) (IsiSensorHandle_t handle, uint32_t mode);
typedef RESULT(*IsiCloseIss_t) (IsiSensorHandle_t handle);
typedef RESULT(*IsiReleaseIss_t) (IsiSensorHandle_t handle);
typedef RESULT(*IsiReadRegIss_t) (IsiSensorHandle_t handle, const uint16_t addr,
				uint16_t *pValue);
typedef RESULT(*IsiWriteRegIss_t) (IsiSensorHandle_t handle,
		const uint16_t addr, const uint16_t value);
typedef RESULT(*IsiGetModeIss_t) (IsiSensorHandle_t handle,
				IsiSensorMode_t *pMode);
typedef RESULT(*IsiEnumModeIss_t) (IsiSensorHandle_t handle,
				IsiSensorEnumMode_t *pEnumMode);
typedef RESULT(*IsiGetCapsIss_t) (IsiSensorHandle_t handle, IsiCaps_t *pCaps);
typedef RESULT(*IsiCheckConnectionIss_t) (IsiSensorHandle_t handle);
typedef RESULT(*IsiGetRevisionIss_t) (IsiSensorHandle_t handle,
				uint32_t *pRevision);
typedef RESULT(*IsiSetStreamingIss_t) (IsiSensorHandle_t handle, bool_t on);

/* AEC */
typedef RESULT(*IsiGetAeBaseInfoIss_t) (IsiSensorHandle_t handle,
				IsiAeBaseInfo_t *pAeBaseInfo);
typedef RESULT(*IsiExcuteExpCtrlIss_t) (IsiSensorHandle_t handle,
			const IsiSensorExpParam_t *pExpParam,
		IsiSensorExpParam_t *pExpResult);
typedef RESULT(*IsiGetAGainIss_t) (IsiSensorHandle_t handle,
				IsiSensorGain_t *pSensorAGain);
typedef RESULT(*IsiSetAGainIss_t) (IsiSensorHandle_t handle,
				IsiSensorGain_t *pSensorAGain);
typedef RESULT(*IsiGetDGainIss_t) (IsiSensorHandle_t handle,
				IsiSensorGain_t *pSensorDGain);
typedef RESULT(*IsiSetDGainIss_t) (IsiSensorHandle_t handle,
				IsiSensorGain_t *pSensorDGain);
typedef RESULT(*IsiGetIntTimeIss_t) (IsiSensorHandle_t handle,
				IsiSensorIntTime_t *pSensorIntTime);
typedef RESULT(*IsiSetIntTimeIss_t) (IsiSensorHandle_t handle,
				const IsiSensorIntTime_t *pSensorIntTime);
typedef RESULT(*IsiGetFpsIss_t) (IsiSensorHandle_t handle, uint32_t *pFps);
typedef RESULT(*IsiSetFpsIss_t) (IsiSensorHandle_t handle, uint32_t fps);

/* SENSOR ISP */
typedef RESULT(*IsiGetIspStatusIss_t) (IsiSensorHandle_t handle,
				IsiIspStatus_t *pIspStatus);
typedef RESULT(*IsiSetBlcIss_t) (IsiSensorHandle_t handle,
				const IsiSensorBlc_t *pBlc);
typedef RESULT(*IsiGetBlcIss_t) (IsiSensorHandle_t handle,
				IsiSensorBlc_t *pBlc);
typedef RESULT(*IsiSetWBIss_t) (IsiSensorHandle_t handle,
				const IsiSensorWb_t *pWb);
typedef RESULT(*IsiGetWBIss_t) (IsiSensorHandle_t handle, IsiSensorWb_t *pWb);

/* SENSOE OTHER FUNC*/
typedef RESULT(*IsiSetTpgIss_t) (IsiSensorHandle_t handle, IsiSensorTpg_t tpg);
typedef RESULT(*IsiGetTpgIss_t) (IsiSensorHandle_t handle,
				IsiSensorTpg_t *pTpg);
typedef RESULT(*IsiGetExpandCurveIss_t) (IsiSensorHandle_t handle,
				IsiSensorCompandCurve_t *pCurve);
typedef RESULT(*IsiGetOtpDataIss_t) (IsiSensorHandle_t handle,
				IsiOTP_t *pOtpData);

/* AF */
typedef RESULT(*IsiFocusCreateIss_t) (IsiSensorHandle_t handle);
typedef RESULT(*IsiFocusReleaseIss_t) (IsiSensorHandle_t handle);
typedef RESULT(*IsiFocusGetCalibrateIss_t) (IsiSensorHandle_t handle,
					IsiFocusCalibAttr_t *pFocusCalib);
typedef RESULT(*IsiFocusSetIss_t) (IsiSensorHandle_t handle,
				const IsiFocusPos_t *pPos);
typedef RESULT(*IsiFocusGetIss_t) (IsiSensorHandle_t handle,
				IsiFocusPos_t *pPos);

/* Infrared Light */
typedef RESULT(*IsiSetIRLightExpIss_t) (IsiSensorHandle_t handle,
					const IsiIrLightExp_t *pIrExpParam);
typedef RESULT(*IsiGetIRLightExpIss_t) (IsiSensorHandle_t handle,
				IsiIrLightExp_t *pIrExpParam);

typedef RESULT(*IsiQueryMetadataAttrIss_t) (IsiSensorHandle_t handle,
				IsiMetadataAttr_t *pAttr);
typedef RESULT(*IsiSetMetadataAttrEnableIss_t) (IsiSensorHandle_t handle,
				IsiMetadataAttr_t attr);
typedef RESULT(*IsiGetMetadataAttrEnableIss_t) (IsiSensorHandle_t handle,
				IsiMetadataAttr_t *pAttr);
typedef RESULT(*IsiGetMetadataWindowIss_t) (IsiSensorHandle_t handle,
					IsiMetadataWinInfo_t *pMetaWin);
typedef RESULT(*IsiParserMetadataIss_t) (IsiSensorHandle_t handle,
				const MetadataBufInfo_t *pMetaBuf,
		IsiSensorMetadata_t *pMetaInfo);


/******************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor attributes.
 *
 *****************************************************************************/
struct IsiSensor_s {
	const char	*pszName;	/**< Name of the camera-sensor */
	const IsiCaps_t	*pIsiCaps;	/**< Pointer to sensor capabilities */
	IsiCreateIss_t	pIsiCreateIss;	/**< Create a sensor handle */
	IsiOpenIss_t	pIsiOpenIss;	/**< Open sensor */
	IsiCloseIss_t	pIsiCloseIss;	/**< Close sensor */
	IsiReleaseIss_t	pIsiReleaseIss;	/**< Release a sensor handle */
	IsiReadRegIss_t	pIsiReadRegIss;	/**< Read sensor register */
	IsiWriteRegIss_t	pIsiWriteRegIss;	/**< Write sensor register */
	IsiGetModeIss_t	pIsiGetModeIss;	/**< Get mode information */
	IsiEnumModeIss_t	pIsiEnumModeIss;	/**< Enumerate sensor mode */
	IsiGetCapsIss_t	pIsiGetCapsIss;	/**< Get sensor capabilities */
	/**< Check sensor connection status */
	IsiCheckConnectionIss_t	pIsiCheckConnectionIss;
	IsiGetRevisionIss_t	pIsiGetRevisionIss;	/**< Read sensor chipid register */
	/**< Enable/disable sensor streaming */
	IsiSetStreamingIss_t	pIsiSetStreamingIss;
	/**< Get AE related information */
	IsiGetAeBaseInfoIss_t	pIsiGetAeBaseInfoIss;
	/**< Execute exposure control */
	IsiExcuteExpCtrlIss_t	pIsiExcuteExpCtrlIss;
	IsiGetAGainIss_t	pIsiGetAGainIss;	/**< Get sensor analog again */
	IsiSetAGainIss_t	pIsiSetAGainIss;	/**< Set sensor analog again */
	IsiGetDGainIss_t	pIsiGetDGainIss;	/**< Get sensor digital again */
	IsiSetDGainIss_t	pIsiSetDGainIss;	/**< Set sensor digital again */
	IsiGetIntTimeIss_t	pIsiGetIntTimeIss;	/**< Get sensor integration time */
	IsiSetIntTimeIss_t	pIsiSetIntTimeIss;	/**< Set sensor integration time */
	IsiGetFpsIss_t	pIsiGetFpsIss;	/**< Get sensor fps */
	IsiSetFpsIss_t	pIsiSetFpsIss;	/**< Set sensor fps */
	IsiGetIspStatusIss_t	pIsiGetIspStatusIss;	/**< Get isp status */
	IsiSetBlcIss_t	pIsiSetBlcIss;	/**< Set sensor BLC */
	IsiGetBlcIss_t	pIsiGetBlcIss;	/**< Get sensor BLC */
	IsiSetWBIss_t	pIsiSetWBIss;	/**< Set sensor WB */
	IsiGetWBIss_t	pIsiGetWBIss;	/**< Get sensor WB */
	IsiSetTpgIss_t	pIsiSetTpgIss;	/**< Set sensor test pattern */
	IsiGetTpgIss_t	pIsiGetTpgIss;	/**< Get sensor test pattern */
	/**< Get sensor expand curve */
	IsiGetExpandCurveIss_t	pIsiGetExpandCurveIss;
	IsiGetOtpDataIss_t	pIsiGetOtpDataIss;	/**< Get sensor OTP data */
	IsiFocusCreateIss_t	pIsiFocusCreateIss;	/**< Create VCM I2C bus */
	IsiFocusReleaseIss_t	pIsiFocusReleaseIss;	/**< Release VCM I2C bus */
	/**< Get focus calibration data */
	IsiFocusGetCalibrateIss_t	pIsiFocusGetCalibrateIss;
	IsiFocusSetIss_t	pIsiFocusSetIss;	/**< Set sensor focus position */
	IsiFocusGetIss_t	pIsiFocusGetIss;	/**< Get sensor focus position */
	/**< Set infrared light exposure */
	IsiSetIRLightExpIss_t	pIsiSetIRLightExpIss;
	/**< Get infrared light exposure */
	IsiGetIRLightExpIss_t	pIsiGetIRLightExpIss;
	/**< Query metadata attr */
	IsiQueryMetadataAttrIss_t	pIsiQueryMetadataAttrIss;
	/**< Enable metadata attr */
	IsiSetMetadataAttrEnableIss_t	pIsiSetMetadataAttrEnableIss;
	/**< Get metadata attr status */
	IsiGetMetadataAttrEnableIss_t	pIsiGetMetadataAttrEnableIss;
	/**< Get metadata window */
	IsiGetMetadataWindowIss_t	pIsiGetMetadataWinIss;
	IsiParserMetadataIss_t	pIsiParserMetadataIss;	/**< Parser metadata */
};

typedef RESULT(*IsiGetSensorIss_t) (IsiSensor_t *pIsiSensor);


/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the camera sensor configuration.
 */
/*****************************************************************************/
typedef struct IsiCamDrvConfig_s {
	uint32_t	cameraDriverID;	/**< ID of camera sensor driver */
	IsiGetSensorIss_t	pIsiGetSensorIss;	/**< Pointer to a function */
	uint8_t	i2cBusId;	/**< The I2C bus ID */
	/**< ID of camera sensor driver that maps the port link information*/
	uint32_t	sensorDevId;
	uint32_t	instanceID;	/**< for virtual sensor use>*/
} IsiCamDrvConfig_t;


/*****************************************************************************/
/**
 * @brief   This function registers a sensor driver handle.
 * @startuml IsiSensorDrvHandleRegisterIss
 * !include E05_External/IsiSensorDrvHandleRegisterIss.plantuml
 * @enduml
 * @param[in]       pCamDrvConfig  configuration of the isi camera driver
 * @param[inout]    pSensorHandle  produced sensor handle
 * @details This function calls: \ref [sensor_drv]_IsiGetSensorIss,
				 \ref [sensor_drv]_IsiCreateIss
 * @details This function is called by: User application,
				 \ref CamDeviceSensorDrvHandleRegister
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         Operation succeeded
 * @retval  RET_FAILURE         Operation failed
 * @retval  RET_NULL_POINTER    Operation failed due to invalid pointer(s)
 * @retval  RET_OUTOFMEM        Operation failed due to out of memory
 * @retval  RET_WRONG_STATE     Operation failed due to wrong state
 * @retval  RET_WRONG_CONFIG    Operation failed due to given
 *                              configuration is invalid
 *
 *****************************************************************************/
RESULT IsiSensorDrvHandleRegisterIss(IsiCamDrvConfig_t *pCamDrvConfig,
				IsiSensorHandle_t *pSensorHandle);

/*****************************************************************************/
/**
 * @brief   This function unregisters a sensor driver handle.
 * @startuml IsiSensorDrvHandleUnRegisterIss
 * !include E05_External/IsiSensorDrvHandleUnRegisterIss.plantuml
 * @enduml
 * @param[inout]    handle  isi sensor handle
 * @details This function calls: \ref [sensor_drv]_IsiReleaseIss
 * @details This function called by: User application,
				 \ref CamDeviceSensorDrvHandleUnRegister
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         Operation succeeded
 * @retval  RET_FAILURE         Operation failed
 * @retval  RET_WRONG_HANDLE    Operation failed due to wrong handle
 * @retval  RET_NOTSUPP         Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSensorDrvHandleUnRegisterIss(IsiSensorHandle_t handle);

/** @} 01_cam_isi */

#ifdef __cplusplus
}
#endif

#endif /* ISI_ISS_H */
