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

#ifndef ISI_H
#define ISI_H

#include "types.h"
#include "return_codes.h"
#include "buf_defs.h"
#include <isi_vvsensor.h>
#include <isi_otp.h>
#include <isi_metadata.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ISI_LINEAR_PARAS			(0)
#define ISI_DUAL_EXP_L_PARAS			(0)
#define ISI_DUAL_EXP_S_PARAS			(1)
#define ISI_TRI_EXP_L_PARAS			(0)
#define ISI_TRI_EXP_S_PARAS			(1)
#define ISI_TRI_EXP_VS_PARAS			(2)
#define ISI_QUAD_EXP_L_PARAS			(0)
#define ISI_QUAD_EXP_S_PARAS			(1)
#define ISI_QUAD_EXP_VS_PARAS			(2)
#define ISI_QUAD_EXP_VVS_PARAS			(3)

typedef void *IsiSensorHandle_t;
typedef struct IsiSensor_s IsiSensor_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor resolution.
 */
/*****************************************************************************/
typedef struct IsiResolution_s {
	uint16_t	width;	/**< Width of output image */
	uint16_t	height;	/**< Height of output image */
} IsiResolution_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor capabilities.
 */
/*****************************************************************************/
typedef struct IsiCaps_s {
	uint32_t	bitWidth;	/**< Supported bit width */
	uint32_t	mode;	/**< Output image type */
	uint32_t	bayerPattern;	/**< Bayer pattern */
	IsiResolution_t	resolution;	/**< Supported resolution */
	uint32_t	vinType;	/**< Video input type */
	uint32_t	mipiMode;	/**< MIPI transfer data format*/
	uint32_t	mipiLanes;	/**< Number of MIPI data lanes */
	IsiSyncSignalPolarity_t	hSyncPol;	/**< Hsync polarity */
	IsiSyncSignalPolarity_t	vSyncPol;	/**< Vsync polarity */
	IsiSampleEdgePolarity_t	sampleEdge;	/**< Sample edge polarity */
	IsiSensorCCIRSequence_t	ccirSeq;	/**< CCIR sequence */
} IsiCaps_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of WB gain.
 */
/*****************************************************************************/
typedef enum IsiAWBModeSelection_e {
	ISI_USE_ISP_WB_GAIN	= 0,    /**< Use ISP WB gain */
	ISI_USE_SENSOR_WB_GAIN	= 1,    /**< Use Sensor WB gain */
	ISI_USE_ISP_AND_SENSOR_WB_GAIN	= 2,    /**< Use ISP and sensor WB gain */
} IsiAWBModeSelection_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the AE/BLC/WB gain type to be used.
 */
/*****************************************************************************/
typedef struct IsiIspStatus_s {
	bool_t	useSensorAE;	/**< Whether to use sensor AE */
	bool_t	useSensorBLC;	/**< Whether to use sensor BLC */
	IsiAWBModeSelection_t	useAWBMode;	/**< Type of AWB gain */
} IsiIspStatus_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the IR light exposure configuration.
 */
/*****************************************************************************/
typedef struct IsiIrLightExp_s {
	bool_t	irOn;	/**< Whether to enable IR light*/
	uint8_t	irStrength;	/**< IR light strength */
} IsiIrLightExp_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the IR light strength configuration.
 */
/*****************************************************************************/
typedef struct IsiIrLightRange_s {
	uint8_t	minIrStrength;	/**< Minimum IR light strength */
	uint8_t	maxIrStrength;	/**< Maximum IR light strength */
	uint8_t	irStrengthStep;	/**< Step of IR light strength */
} IsiIrLightRange_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor IR light information.
 */
/*****************************************************************************/
typedef struct IsiIrLightInfo_s {
	bool_t	irSuppAeCtrl;	/**< Whether sensor IR light supports AE control */
	IsiIrLightRange_t	irRangeInfo;	/**< Range of IR light strength */
	uint8_t	irDelayFrame;	/**< Delay frame of IR light */
} IsiIrLightInfo_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of sensor focus position.
 */
/*****************************************************************************/
typedef enum IsiFocusPosMode_enum {
	ISI_FOCUS_POS_ABSOLUTE	= 0,    /**< Absolute focus position */
	ISI_FOCUS_POS_RELATIVE,                 /**< Relative focus position */
} IsiFocusPosMode_e;

/*****************************************************************************/
/**
 * @brief   VsCamISI enumeration type of PDAF sensor.
 */
/*****************************************************************************/
typedef enum IsiPdafSensorType_enum {
	ISI_PDAF_SENSOR_DUAL_PIXEL	= 0,    /**< Dual pixel type */
	ISI_PDAF_SENSOR_OCL2X1	= 1,        /**< OCL2X1 type */
	ISI_PDAF_SENSOR_TYPE_MAX,          /**< Number of PDAF sensor type */
} IsiPdafSensorType_e;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the focus position information.
 */
/*****************************************************************************/
typedef struct IsiFocusPosInfo_s {
	uint32_t	minPos;	/**< Minimum position */
	uint32_t	maxPos;	/**< Maximum position */
	uint32_t	minStep;	/**< Minimum step of position */
} IsiFocusPosInfo_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the information
 *          of sensor focus PD points.
 */
/*****************************************************************************/
typedef struct IsiFocusPDInfo_s {
	IsiPdafSensorType_e	sensorType;	/**< PDAF sensor type */
	IsiBayerPattern_t	bayerPattern;	/**< Sensor Bayer pattern. */
	bool_t	ocl2x1Shield;	/**< Whether to use the ocl2x1Shield mode */
	uint8_t	bitWidth;	/**< Output bit width */
	uint32_t	imageWidth;	/**< Output image width */
	uint32_t	imageHeight;	/**< Output image height */
	uint16_t	pdArea[4];	/**< PD area of the PDAF sensor */
	uint32_t	correctRect[4];	/**< Correction rectangle */
	uint8_t	pdNumPerArea[2];	/**< PD number per PD area */
	uint8_t	pdShiftL2R[2];	/**< PD shift LR type */
	uint8_t	pdShiftMark[32];	/**< PD shift mark */
	uint8_t	pdFocalHeigh;	/**< Height of PD focal */
	uint8_t	pdFocalWidth;	/**< Width of PD focal */
	uint8_t	pdDistance;	/**< Distance of two adjacent PD points */
	int	pdFocal[48];	/**< PD focal */
} IsiFocusPDInfo_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor
 *          focus calibration information.
 */
/*****************************************************************************/
typedef struct IsiFocusCalibAttr_s {
	IsiFocusPosInfo_t	posInfo;	/**< Focus position information */
	IsiFocusPDInfo_t	pdInfo;	/**< Focus PD information */
} IsiFocusCalibAttr_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor focus information.
 */
/*****************************************************************************/
typedef struct IsiFocusPos_s {
	IsiFocusPosMode_e	posType;	/**< Sensor focus position mode*/
	uint32_t	pos;	/**< Sensor focus position */
} IsiFocusPos_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the i2c bus information.
 */
/*****************************************************************************/
typedef struct IsiBus_s {
	uint8_t	type;
	struct I2C {
		uint8_t		i2cBusNum;	/**< The I2C bus the sensor is connected to */
		/**< The I2C slave addr the sensor is configured to */
		uint16_t	slaveAddr;
	uint8_t		addrWidth;	/**< Sensor register address width */
		uint8_t		dataWidth;	/**< Sensor register data width */
	} i2c;
} IsiBus_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor instance configuration.
 */
/*****************************************************************************/
typedef struct IsiSensorInstanceConfig_s {
	uint32_t	halDevID;	/**< HAL device ID of the sensor */
	IsiBus_t	sensorBus;	/**< BUS information of the sensor */
	IsiBus_t	motorBus;	/**< BUS information of the sensor motor */
	IsiSensor_t	*pSensor;	/**< Pointer to the sensor driver interface */
	uint32_t	cameraDriverID;	/**< ID of camera sensor driver */
	uint32_t	instanceID;	/**< for virtual sensor use>*/
} IsiSensorInstanceConfig_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the AE base information.
 */
/*****************************************************************************/
typedef struct IsiAeBaseInfo_s {
	float32_t	aecCurGain;	/*<Current gain in linear mode*/
	float32_t	aecCurIntTime;	/*<Current integration time in linear mode*/
	IsiSensorGain_t	curGain;	/**<gain: L/S/VS/VVS */
	IsiSensorIntTime_t	curIntTime;	/*<intTime: L/S/VS/VVS */
	float32_t	aecGainStep;	/*<Gain step */
	float32_t	aecIntTimeStep;	/*<Integration time step */
	IsiSensorStitchingMode_t	stitchingMode;	/*<Stitching HDR mode*/
	float32_t	conversionGainDCG;	/*<Conversion gain between HCG and LCG*/
	IsiSensorNativeMode_t	nativeMode;	/*<Native HDR mode*/
	float32_t	nativeHdrRatio[3];	/* 0: L/S, 1: S/VS, 2:VS/VVS*/
	IsiRange_t	longGain;	/*Long frame gain limit */
	IsiRange_t	shortGain;	/*Short frame gain limit */
	IsiRange_t	vsGain;	/*VS frame gain limit */
	IsiRange_t	vvsGain;	/*VVS frame gain limit */
	IsiRange_t	gain;	/*Linear frame gain limit */
	IsiRange_t	longIntTime;	/*Long frame intTime limit */
	IsiRange_t	shortIntTime;	/*Short frame intTime limit */
	IsiRange_t	vsIntTime;	/*VS frame intTime limit */
	IsiRange_t	vvsIntTime;	/*VVS frame intTime limit */
	IsiRange_t	intTime;	/*Linear frame intTime limit */
	IsiGainInfo_t	aLongGain;	/*Analog gain: long */
	IsiGainInfo_t	aShortGain;	/*Analog gain: short */
	IsiGainInfo_t	aVSGain;	/*Analog gain: VS */
	IsiGainInfo_t	aVVSGain;	/*Analog gain: VVS */
	IsiGainInfo_t	aGain;	/*Analog gain: linear */
	IsiGainInfo_t	dLongGain;	/*Digital gain: long */
	IsiGainInfo_t	dShortGain;	/*Digital gain: short */
	IsiGainInfo_t	dVSGain;	/*Digital gain: VS */
	IsiGainInfo_t	dVVSGain;	/*Digital gain: VVS */
	IsiGainInfo_t	dGain;	/*Digital gain: linear */
	IsiIrLightExp_t	aecIrLightExp;	/*IR light exposure */
	IsiIrLightInfo_t	aecIrLightInfo;	/*IR light information */
	bool_t	normGainSupport;	/* Whether to support normal gain */
	float32_t	normGain;	/*Normal gain value*/
} IsiAeBaseInfo_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the exposure decomposition parameters.
 */
/*****************************************************************************/
typedef struct IsiExpDecomposeParam_s {
	float32_t	gain;	/**< Decomposed gain value */
	float32_t	integrationTime;	/**< Decomposed integration time value */
	/**< Ratio between different frame */
	float32_t	ispRatio[ISI_EXP_NUM_MAX-1];
} IsiExpDecomposeParam_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the exposure decomposition results.
 */
/*****************************************************************************/
typedef struct IsiExpDecomposeResult_s {
	/**< Decomposed sensor analog gain */
	float32_t	sensorAgain[ISI_EXP_NUM_MAX];
	/**< Decomposed sensor digital gain */
	float32_t	sensorDgain[ISI_EXP_NUM_MAX];
	/**< Sensor exp time */
	float32_t	sensorExpTime[ISI_EXP_NUM_MAX];
	/**< Ratio between different frame */
	float32_t	sensorRatio[ISI_EXP_NUM_MAX-1];
} IsiExpDecomposeResult_t;

/*****************************************************************************/
/**
 * @brief   VsCamISI structure to define the sensor exposure parameters.
 */
/*****************************************************************************/
typedef struct IsiSensorExpParam_s {
	float32_t	sensorAgain[ISI_EXP_NUM_MAX];	/**< Sensor analog gain */
	float32_t	sensorDgain[ISI_EXP_NUM_MAX];	/**< Sensor digital gain */
	float32_t	sensorExpTime[ISI_EXP_NUM_MAX];	/**< Sensor exposure time */
	/**< Ratio between different frame */
	float32_t	sensorRatio[ISI_EXP_NUM_MAX-1];
	float32_t	ispGain;	/**< ISP digital gain */
} IsiSensorExpParam_t;

/*****************************************************************************/
/**
 * @brief   This function opens a image sensor.
 * @startuml IsiOpenIss
 * !include E05_External/IsiOpenIss.plantuml
 * @enduml
 * @param[inout]    handle  Sensor instance handle
 * @param[in]       mode    Current work mode
 * @details This function calls: \ref [sensor_drv]_IsiOpenIss
 * @details This function is called by: \ref CamDeviceSensorIsiOpen
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS             Operation succeeded
 * @retval  RET_WRONG_HANDLE        Operation failed due to wrong handle
 * @retval  RET_NOTSUPP             Feature not supported
 * @retval  RET_WRONG_STATE         Operation failed due to wrong state
 * @retval  RET_NULL_POINTER        Operation failed due to invalid pointer(s)
 *
 *****************************************************************************/
RESULT IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode);

/*****************************************************************************/
/**
 * @brief   This function closes a image sensor.
 * @startuml IsiCloseIss
 * !include E05_External/IsiCloseIss.plantuml
 * @enduml
 * @param[inout]    handle  Sensor instance handle
 * @details This function calls: \ref [sensor_drv]_IsiCloseIss
 * @details This function is called by: \ref CamDeviceSensorIsiClose
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS             Operation succeeded
 * @retval  RET_WRONG_HANDLE        Operation failed due to wrong handle
 * @retval  RET_NOTSUPP             Feature not supported
 *
 *****************************************************************************/
RESULT IsiCloseIss(IsiSensorHandle_t handle);

/*****************************************************************************/
/**
 * @brief   This function reads a sensor register.
 * @startuml IsiReadRegIss
 * !include E05_External/IsiReadRegIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       addr    Register address
 * @param[inout]    value   Register value read from the register
 * @details This function calls: \ref [sensor_drv]_IsiReadRegIss
 * @details This function is called by: \ref CamDeviceSensorIsiReadRegister
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 * @retval  RET_FAILURE            Operation failed
 *
 *****************************************************************************/
RESULT IsiReadRegIss(
	IsiSensorHandle_t handle,
	const uint16_t addr, uint16_t *pValue);

/*****************************************************************************/
/**
 * @brief   This function writes a sensor register.
 * @startuml IsiWriteRegIss
 * !include E05_External/IsiWriteRegIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       addr    Register address
 * @param[in]       value   Register value to write
 * @details This function calls: \ref [sensor_drv]_IsiWriteRegIss
 * @details This function is called by: \ref CamDeviceSensorIsiWriteRegister
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 * @retval  RET_FAILURE            Operation failed
 *
 *****************************************************************************/
RESULT IsiWriteRegIss(
	IsiSensorHandle_t handle,
	const uint16_t addr, const uint16_t value);

/*****************************************************************************/
/**
 * @brief   This function gets sensor mode information of current mode index.
 * @startuml IsiGetModeIss
 * !include E05_External/IsiGetModeIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pMode   Pointer to the current work mode
 * @details This function calls: \ref [sensor_drv]_IsiGetModeIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetModeInfo,
 * \ref CamDeviceSensorIsiGetStatus, \ref CamEngineGetSensorModeInfo,
 * \ref AwbCtrlSetAttribute, \ref AeCtrlSetExpTable, \ref AeCtrlSetAttribute,
 * \ref AeCtrlMetaDataSetAttribute
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetModeIss(IsiSensorHandle_t handle, IsiSensorMode_t *pMode);

/*****************************************************************************/
/**
 * @brief   This function enumerates mode information of all sensor modes.
 * @startuml IsiEnumModeIss
 * !include E05_External/IsiEnumModeIss.plantuml
 * @enduml
 * @param[in]       handle     Sensor instance handle
 * @param[inout]    pEnumMode  Pointer to the sensor mode information
 * @details This function calls: \ref [sensor_drv]_IsiEnumModeIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetQuery
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 * @retval  RET_OUTOFRANGE         Parameter/variable out of range
 *
 *****************************************************************************/
RESULT IsiEnumModeIss(IsiSensorHandle_t handle, IsiSensorEnumMode_t *pEnumMode);

/*****************************************************************************/
/**
 * @brief   This function gets sensor capabilities.
 * @startuml IsiGetCapsIss
 * !include E05_External/IsiGetCapsIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pCaps   Pointer to the sensor capabilities
 * @details This function calls: \ref [sensor_drv]_IsiGetCapsIss
 * @details This function is called by: \ref CamDeviceSensorIsiOpen
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 *
 *****************************************************************************/
RESULT IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps);

/*****************************************************************************/
/**
 * @brief   This function checks sensor connection status.
 * @startuml IsiCheckConnectionIss
 * !include E05_External/IsiCheckConnectionIss.plantuml
 * @enduml
 * @param[in]    handle  Sensor instance handle
 * @details This function calls: \ref [sensor_drv]_IsiCheckConnectionIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetStatus
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 * @retval  RET_FAILURE            Operation failed
 *
 *****************************************************************************/
RESULT IsiCheckConnectionIss(IsiSensorHandle_t handle);


/*****************************************************************************/
/**
 * @brief   This function reads sensor chipid registers.
 * @startuml IsiGetRevisionIss
 * !include E05_External/IsiGetRevisionIss.plantuml
 * @enduml
 * @param[in]       handle     Sensor instance handle
 * @param[inout]    pRevision  Pointer to the sensor revision
 * @details This function calls: \ref [sensor_drv]_IsiGetRevisionIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetInfo
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *pRevision);


/*****************************************************************************/
/**
 * @brief   This function enables/disables sensor streaming.
 * @startuml IsiSetStreamingIss
 * !include E05_External/IsiSetStreamingIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       on      New streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 * @details This function calls: \ref [sensor_drv]_IsiSetStreamingIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetStreamEnable
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval  RET_WRONG_STATE        Operation failed due to wrong state
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 * @retval  RET_FAILURE            Operation failed
 *
 *****************************************************************************/
RESULT IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on);

/*****************************************************************************/
/**
 * @brief   This function executes the sensor gain split.
 * @startuml IsiGainExecuteIss
 * !include E05_External/IsiGainExecuteIss.plantuml
 * @enduml
 * @param[in]       totalGain     The sensor total gain need to execute
 * @param[in]       aGain         The limit of sensor analog gain
 * @param[in]       dGain         The limit of sensor digital gain
 * @param[inout]    splitAgain    The pointer of split analog gain
 * @param[inout]    splitDgain    The pointer of split digital gain
 * @details This function is called by: \ref AeCtrlhdrDualExpDecompose,
 * \ref AehdrTriExpDecompose, \ref AehdrQuadExpDecompose
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 *
 *****************************************************************************/
RESULT IsiGainExecuteIss(
	float32_t totalGain,
	IsiGainInfo_t aGain, IsiGainInfo_t dGain,
				float32_t *splitAgain, float32_t *splitDgain);

/*****************************************************************************/
/**
 * @brief   This function implements exposure decomposition.
 * @startuml IsiSensorExecuteExposureControl
 * !include E05_External/IsiSensorExecuteExposureControl.plantuml
 * @enduml
 * @param[in]       handle      Sensor instance handle
 * @param[in]       pExpParam   Pointer to the input exposure parammeters
 * @param[inout]    pExpResult  Pointer to the calculated result of exposure
 * @details This function calls: \ref [sensor_drv]_IsiExcuteExpCtrlIss
 * @details This function is called by: \ref AeCtrlSensorDecompose
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSensorExecuteExposureControl(
	IsiSensorHandle_t handle,
	const IsiSensorExpParam_t *pExpParam,
	IsiSensorExpParam_t *pExpResult);

/*****************************************************************************/
/**
 * @brief   This function gets AE related base information.
 * @startuml IsiGetAeBaseInfoIss
 * !include E05_External/IsiGetAeBaseInfoIss.plantuml
 * @enduml
 * @param[in]       handle       Sensor instance handle
 * @param[inout]    pAeBaseInfo  Pointer to the sensor AE base information
 * @details This function calls: \ref [sensor_drv]_IsiGetAeBaseInfoIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetInfo,
 * \ref AeCtrlSetExpTable, \ref AeCtrlSetAttributeSensor,
 * \ref AeCtrlMetaDataSetAttribute,
 * \ref AeCtrlGetPar, \ref AwbCtrlGetEachFrameParm,
 * \ref CamEngineMeasureCbForExpV2Event,
 * \ref CamEngineGetSensorInfo
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetAeBaseInfoIss(
	IsiSensorHandle_t handle,
	IsiAeBaseInfo_t *pAeBaseInfo);

/*****************************************************************************/
/**
 * @brief   This function gets sensor analog gain.
 * @startuml IsiGetAGainIss
 * !include E05_External/IsiGetAGainIss.plantuml
 * @enduml
 * @param[in]       handle        Sensor instance handle
 * @param[inout]    pSensorAGain  Pointer to sensor again
 * @details This function calls: \ref [sensor_drv]_IsiGetAGainIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetStatus,
 * \ref CamDeviceSensorIsiGetGain, \ref CamEngineGetSensorInfo
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain);

/*****************************************************************************/
/**
 * @brief   This function gets sensor digital gain.
 * @startuml IsiGetDGainIss
 * !include E05_External/IsiGetDGainIss.plantuml
 * @enduml
 * @param[in]       handle        Sensor instance handle
 * @param[inout]    pSensorDGain  Pointer to sensor dgain
 * @details This function calls: \ref [sensor_drv]_IsiGetDGainIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetStatus,
 * \ref CamDeviceSensorIsiGetGain, \ref CamEngineGetSensorInfo
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain);

/*****************************************************************************/
/**
 * @brief   This function sets sensor analog gain.
 * @startuml IsiSetAGainIss
 * !include E05_External/IsiSetAGainIss.plantuml
 * @enduml
 * @param[in]       handle        Sensor instance handle
 * @param[in]       pSensorAGain  Pointer to sensor again to write
 * @details This function calls: \ref [sensor_drv]_IsiSetAGainIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetGain,
 * \ref AeCtrlSetResult
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain);

/*****************************************************************************/
/**
 * @brief   This function sets sensor digital gain.
 * @startuml IsiSetDGainIss
 * !include E05_External/IsiSetDGainIss.plantuml
 * @enduml
 * @param[in]       handle        Sensor instance handle
 * @param[in]       pSensorDGain  Pointer to sensor dgain to write
 * @details This function calls: \ref [sensor_drv]_IsiSetDGainIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetGain,
 * \ref AeCtrlSetResult
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain);


/*****************************************************************************/
/**
 * @brief   This function gets sensor integration time.
 * @startuml IsiGetIntTimeIss
 * !include E05_External/IsiGetIntTimeIss.plantuml
 * @enduml
 * @param[in]       handle          Sensor instance handle
 * @param[inout]    pSensorIntTime  Pointer to integration time
 * @details This function calls: \ref [sensor_drv]_IsiGetIntTimeIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetStatus,
 * \ref CamDeviceSensorIsiGetExpControl, \ref CamEngineGetSensorInfo
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetIntTimeIss(
	IsiSensorHandle_t handle,
	IsiSensorIntTime_t *pSensorIntTime);


/*****************************************************************************/
/**
 * @brief   This function sets sensor integration time.
 * @startuml IsiSetIntTimeIss
 * !include E05_External/IsiSetIntTimeIss.plantuml
 * @enduml
 * @param[in]       handle            Sensor instance handle
 * @param[in]       pSensorIntTime    Pointer to integration time
 * @details This function calls: \ref [sensor_drv]_IsiSetIntTimeIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetExpControl,
 * \ref AeCtrlSetResult
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSetIntTimeIss(
	IsiSensorHandle_t handle,
	const IsiSensorIntTime_t *pSensorIntTime);

/*****************************************************************************/
/**
 * @brief   This function gets sensor fps.
 * @startuml IsiGetFpsIss
 * !include E05_External/IsiGetFpsIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pFps    Pointer to the current fps
 * @details This function calls: \ref [sensor_drv]_IsiGetFpsIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetFrameRate
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps);

/*****************************************************************************/
/**
 * @brief   This function sets sensor fps.
 * @startuml IsiSetFpsIss
 * !include E05_External/IsiSetFpsIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       fps     Pointer to the current fps
 * @details This function calls: \ref [sensor_drv]_IsiSetFpsIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetFrameRate
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps);

/*****************************************************************************/
/**
 * @brief   This function gets sensor isp status.
 * @startuml IsiGetIspStatusIss
 * !include E05_External/IsiGetIspStatusIss.plantuml
 * @enduml
 * @param[in]       handle      Sensor instance handle
 * @param[inout]    pIspStatus  Pointer to AE, BLC,
 *                              and WB gain types
 * @details This function calls: \ref [sensor_drv]_IsiGetIspStatusIss
 * @details This function is called by: \ref AwbCtrlSetAttribute,
 * \ref ABlsSetCurrentConfigure, \ref ABlsGetCurrentConfigure,
 * \ref CamEngineWbSetConfig, \ref CamEngineWbGetCurentConfig
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetIspStatusIss(IsiSensorHandle_t handle, IsiIspStatus_t *pIspStatus);

/*****************************************************************************/
/**
 * @brief   This function sets sensor black level.
 * @startuml IsiSetBlcIss
 * !include E05_External/IsiSetBlcIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       pBlc    Pointer to the BLC parameters
 * @details This function calls: \ref [sensor_drv]_IsiSetBlcIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetBls,
 * \ref ABlsSetCurrentConfigure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 * @retval  RET_FAILURE            Operation failed
 *
 *****************************************************************************/
RESULT IsiSetBlcIss(IsiSensorHandle_t handle, const IsiSensorBlc_t *pBlc);

/*****************************************************************************/
/**
 * @brief   This function gets sensor black level.
 * @startuml IsiGetBlcIss
 * !include E05_External/IsiGetBlcIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pBlc    Pointer to the BLC parameters
 * @details This function calls: \ref [sensor_drv]_IsiGetBlcIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetBls,
 * \ref ABlsGetCurrentConfigure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc);

/*****************************************************************************/
/**
 * @brief   This function sets sensor white balance gain.
 * @startuml IsiSetWBIss
 * !include E05_External/IsiSetWBIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       pWb     Pointer to the WB parameters
 * @details This function calls: \ref [sensor_drv]_IsiSetWBIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetWb,
 * \ref AwbCtrlSetSensorWbGain, \ref CamEngineSetNativeHdrWbGains
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 * @retval  RET_FAILURE            Operation failed
 *
 *****************************************************************************/
RESULT IsiSetWBIss(IsiSensorHandle_t handle, const IsiSensorWb_t *pWb);

/*****************************************************************************/
/**
 * @brief   This function gets sensor white balance gain.
 * @startuml IsiGetWBIss
 * !include E05_External/IsiGetWBIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pWb     Pointer to the WB parameters
 * @details This function calls: \ref [sensor_drv]_IsiGetWBIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetWb,
 * \ref AwbCtrlGetWbGainFromSensor, \ref CamEngineGetSensorModeInfo,
 * \ref CamEngineGetNativeHdrWbCurentGains
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb);

/*****************************************************************************/
/**
 * @brief   This function sets sensor test pattern.
 * @startuml IsiSetTpgIss
 * !include E05_External/IsiSetTpgIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       tpg     Sensor test pattern
 * @details This function calls: \ref [sensor_drv]_IsiSetTpgIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetTestPattern
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval  RET_WRONG_STATE        Operation failed due to wrong state
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg);

/*****************************************************************************/
/**
 * @brief   This function gets sensor test pattern.
 * @startuml IsiGetTpgIss
 * !include E05_External/IsiGetTpgIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pTpg    Pointer to the sensor test pattern
 * @details This function calls: \ref [sensor_drv]_IsiGetTpgIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetInfo
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval  RET_WRONG_STATE        Operation failed due to wrong state
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *pTpg);

/*****************************************************************************/
/**
 * @brief   This function gets sensor otp data.
 * @startuml IsiGetOtpDataIss
 * !include E05_External/IsiGetOtpDataIss.plantuml
 * @enduml
 * @param[in]       handle    Sensor instance handle
 * @param[inout]    pOtpData  Pointer to the sensor OTP data
 * @details This function calls: \ref [sensor_drv]_IsiGetOtpDataIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetOtpInfo,
 * \ref AwbCtrlSetAttribute, \ref AfCtrlSetAttribute
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetOtpDataIss(IsiSensorHandle_t handle, IsiOTP_t *pOtpData);

/*****************************************************************************/
/**
 * @brief   This function creates VCM I2C bus handle.
 * @startuml IsiFocusCreateIss
 * !include E05_External/IsiFocusCreateIss.plantuml
 * @enduml
 * @param[in]    handle  Sensor instance handle
 * @details This function calls: \ref [sensor_drv]_IsiFocusCreateIss
 * @details This function is called by: \ref CamDeviceSensorIsiOpen
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiFocusCreateIss(IsiSensorHandle_t handle);

/*****************************************************************************/
/**
 * @brief   This function releases VCM I2C bus handle.
 * @startuml IsiFocusReleaseIss
 * !include E05_External/IsiFocusReleaseIss.plantuml
 * @enduml
 * @param[in]    handle  Sensor instance handle
 * @details This function calls: \ref [sensor_drv]_IsiFocusReleaseIss
 * @details This function is called by: \ref CamDeviceSensorIsiClose
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiFocusReleaseIss(IsiSensorHandle_t handle);

/*****************************************************************************/
/**
 * @brief   This function gets focus calibration data.
 * @startuml IsiFocusGetCalibrateIss
 * !include E05_External/IsiFocusGetCalibrateIss.plantuml
 * @enduml
 * @param[in]       handle       Sensor instance handle
 * @param[inout]    pFocusCalib  Pointer to the sensor focus calibration data
 * @details This function calls: \ref [sensor_drv]_IsiFocusGetCalibrateIss
 * @details Called by:
 * \ref CamDeviceSensorIsiGetFocusCalibData,
 * \ref CamDeviceSensorIsiSetFocusPos, \ref AfCtrlSetAttribute
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiFocusGetCalibrateIss(
	IsiSensorHandle_t handle,
	IsiFocusCalibAttr_t *pFocusCalib);

/*****************************************************************************/
/**
 * @brief   This function sets sensor focus position.
 * @startuml IsiFocusSetIss
 * !include E05_External/IsiFocusSetIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       pPos    Pointer to the focus position
 * @details This function calls: \ref [sensor_drv]_IsiFocusSetIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetFocusPos,
 * \ref AfCtrlSetResultToSensor
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiFocusSetIss(IsiSensorHandle_t handle, const IsiFocusPos_t *pPos);

/*****************************************************************************/
/**
 * @brief   This function gets sensor focus position.
 * @startuml IsiFocusGetIss
 * !include E05_External/IsiFocusGetIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pPos    Pointer to the focus position
 * @details This function calls: \ref [sensor_drv]_IsiFocusGetIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetFocusPos
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos);

/*****************************************************************************/
/**
 * @brief   This function sets infrared light exposure parameters.
 * @startuml IsiSetInfraredLightExpParamIss
 * !include E05_External/IsiSetInfraredLightExpParamIss.plantuml
 * @enduml
 * @param[in]       handle       Sensor instance handle
 * @param[in]       pIrExpParam  Pointer to IR light
 *                               exposure parameters
 * @details This function calls:
 * \ref [sensor_drv]_IsiSetInfraredLightExpParamIss
 * @details This function is called by: \ref AeCtrlSetOutputParam
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSetInfraredLightExpParamIss(
	IsiSensorHandle_t handle,
	IsiIrLightExp_t *pIrExpParam);

/*****************************************************************************/
/**
 * @brief   This function gets infrared light exposure parameters.
 * @startuml IsiGetInfraredLightExpParamIss
 * !include E05_External/IsiGetInfraredLightExpParamIss.plantuml
 * @enduml
 * @param[in]       handle       Sensor instance handle
 * @param[inout]    pIrExpParam  Pointer to IR light
 *                               exposure parameters
 * @details This function calls:
 * \ref [sensor_drv]_IsiGetInfraredLightExpParamIss
 * @details This function is called by: \ref AeCtrlGetEachFrameParm
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetInfraredLightExpParamIss(
	IsiSensorHandle_t handle,
	IsiIrLightExp_t *pIrExpParam);

/*****************************************************************************/
/**
 * @brief   This function queries the metadata attribution.
 * @startuml IsiQueryMetadataAttrIss
 * !include E05_External/IsiQueryMetadataAttrIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pAttr   Pointer to the attributes of metadata
 * @details This function calls: \ref [sensor_drv]_IsiQueryMetadataAttrIss
 * @details This function is called by: \ref CamDeviceSensorIsiOpen,
 * \ref CamDeviceSensorIsiGetInfo, \ref CamDeviceSensorIsiGetMetadataAttrMask
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiQueryMetadataAttrIss(
	IsiSensorHandle_t handle,
	IsiMetadataAttr_t *pAttr);

/*****************************************************************************/
/**
 * @brief   This function specifies whether to enable metadata attribution.
 * @startuml IsiSetMetadataAttrEnableIss
 * !include E05_External/IsiSetMetadataAttrEnableIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[in]       attr    The attributes of sensor meta data
 * @details This function calls: \ref [sensor_drv]_IsiSetMetadataAttrEnableIss
 * @details This function is called by: \ref CamDeviceSensorIsiSetMetadataAttr
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiSetMetadataAttrEnableIss(
	IsiSensorHandle_t handle,
	IsiMetadataAttr_t attr);

/*****************************************************************************/
/**
 * @brief   This function gets the enabling status of metadata attribution.
 * @startuml IsiGetMetadataAttrEnableIss
 * !include E05_External/IsiGetMetadataAttrEnableIss.plantuml
 * @enduml
 * @param[in]       handle  Sensor instance handle
 * @param[inout]    pAttr   Pointer to the attributes of sensor metadata
 * @details This function calls: \ref [sensor_drv]_IsiGetMetadataAttrEnableIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetMetadataAttr
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetMetadataAttrEnableIss(
	IsiSensorHandle_t handle,
	IsiMetadataAttr_t *pAttr);

/*****************************************************************************/
/**
 * @brief   This function gets metadata window information.
 * @startuml IsiGetMetadataWindowIss
 * !include E05_External/IsiGetMetadataWindowIss.plantuml
 * @enduml
 * @param[in]       handle    Sensor instance handle
 * @param[inout]    pMetaWin  Pointer to the metadata window information
 * @details This function calls: \ref [sensor_drv]_IsiGetMetadataWindowIss
 * @details This function is called by: \ref CamDeviceSensorIsiGetMetadataWindow
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiGetMetadataWindowIss(
	IsiSensorHandle_t handle,
	IsiMetadataWinInfo_t *pMetaWin);

/*****************************************************************************/
/**
 * @brief   This function parsers sensor metadata.
 * @startuml IsiParserMetadataIss
 * !include E05_External/IsiParserMetadataIss.plantuml
 * @enduml
 * @param[in]       handle      Sensor instance handle
 * @param[in]       pMetaBuf    Pointer to the metadata buffer information
 * @param[inout]    pMetaInfo   Pointer to the sensor metadata information
 * @details This function calls: \ref [sensor_drv]_IsiParserMetadataIss
 * @details This function is called by: \ref CamDeviceSensorIsiParserMetadata
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS            Operation succeeded
 * @retval  RET_WRONG_HANDLE       Operation failed due to wrong handle
 * @retval  RET_NULL_POINTER       Operation failed due to invalid pointer(s)
 * @retval RET_NOTSUPP            Operation aborted due to feature not supported
 *
 *****************************************************************************/
RESULT IsiParserMetadataIss(
	IsiSensorHandle_t handle,
	const MetadataBufInfo_t *pMetaBuf,
	IsiSensorMetadata_t *pMetaInfo);

/** @} 01_cam_isi */

#ifdef __cplusplus
}
#endif

#endif /* ISI_H */
