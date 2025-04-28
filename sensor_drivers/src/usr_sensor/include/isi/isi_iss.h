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
#ifndef __ISI_ISS_H__
#define __ISI_ISS_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include "isi/isi.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef RESULT (*IsiCreateIss_t)(IsiSensorInstanceConfig_t *pConfig, IsiSensorHandle_t *HandlePtr);
typedef RESULT (*IsiOpenIss_t)(IsiSensorHandle_t handle, uint32_t mode);
typedef RESULT (*IsiCloseIss_t)(IsiSensorHandle_t handle);
typedef RESULT (*IsiReleaseIss_t)(IsiSensorHandle_t handle);
typedef RESULT (*IsiReadRegIss_t)(IsiSensorHandle_t handle, const uint16_t addr,
					uint16_t *ValuePtr);
typedef RESULT (*IsiWriteRegIss_t)(IsiSensorHandle_t handle, const uint16_t addr,
					const uint16_t value);
typedef RESULT (*IsiGetModeIss_t)(IsiSensorHandle_t handle, IsiSensorMode_t *ModePtr);
typedef RESULT (*IsiEnumModeIss_t)(IsiSensorHandle_t handle, IsiSensorEnumMode_t *EnumModePtr);
typedef RESULT (*IsiGetCapsIss_t)(IsiSensorHandle_t handle, IsiCaps_t *CapsPtr);
typedef RESULT (*IsiCheckConnectionIss_t)(IsiSensorHandle_t handle);
typedef RESULT (*IsiGetRevisionIss_t)(IsiSensorHandle_t handle, uint32_t *RevisionPtr);
typedef RESULT (*IsiSetStreamingIss_t)(IsiSensorHandle_t handle, bool_t on);
typedef RESULT (*IsiGetAeBaseInfoIss_t)(IsiSensorHandle_t handle, IsiAeBaseInfo_t *AeBaseInfoPtr);
typedef RESULT (*IsiExpDecomposeCtrlIss_t)(IsiSensorHandle_t handle,
					const IsiExpDecomposeParam_t *DecParamPtr,
					IsiExpDecomposeResult_t *DecResultPtr);
typedef RESULT (*IsiGetAGainIss_t)(IsiSensorHandle_t handle, IsiSensorGain_t *SensorAGainPtr);
typedef RESULT (*IsiSetAGainIss_t)(IsiSensorHandle_t handle, IsiSensorGain_t *SensorAGainPtr);
typedef RESULT (*IsiGetDGainIss_t)(IsiSensorHandle_t handle, IsiSensorGain_t *SensorDGainPtr);
typedef RESULT (*IsiSetDGainIss_t)(IsiSensorHandle_t handle, IsiSensorGain_t *SensorDGainPtr);
typedef RESULT (*IsiGetIntTimeIss_t)(IsiSensorHandle_t handle,
					IsiSensorIntTime_t *SensorIntTimePtr);
typedef RESULT (*IsiSetIntTimeIss_t)(IsiSensorHandle_t handle,
					const IsiSensorIntTime_t *SensorIntTimePtr);
typedef RESULT (*IsiGetFpsIss_t)(IsiSensorHandle_t handle, uint32_t *FpsPtr);
typedef RESULT (*IsiSetFpsIss_t)(IsiSensorHandle_t handle, uint32_t fps);
typedef RESULT (*IsiGetIspStatusIss_t)(IsiSensorHandle_t handle, IsiIspStatus_t *IspStatusPtr);
typedef RESULT (*IsiSetBlcIss_t)(IsiSensorHandle_t handle, const IsiSensorBlc_t *BlcPtr);
typedef RESULT (*IsiGetBlcIss_t)(IsiSensorHandle_t handle, IsiSensorBlc_t *BlcPtr);
typedef RESULT (*IsiSetWBIss_t)(IsiSensorHandle_t handle, const IsiSensorWb_t *WbPtr);
typedef RESULT (*IsiGetWBIss_t)(IsiSensorHandle_t handle, IsiSensorWb_t *WbPtr);
typedef RESULT (*IsiSetTpgIss_t)(IsiSensorHandle_t handle, IsiSensorTpg_t tpg);
typedef RESULT (*IsiGetTpgIss_t)(IsiSensorHandle_t handle, IsiSensorTpg_t *TpgPtr);
typedef RESULT (*IsiGetExpandCurveIss_t)(IsiSensorHandle_t handle,
					IsiSensorCompandCurve_t *CurvePtr);
typedef RESULT (*IsiGetCompressCurveIss_t)(IsiSensorHandle_t handle,
					IsiSensorCompandCurve_t *CurvePtr);
typedef RESULT (*IsiExtendFuncIss_t)(IsiSensorHandle_t handle, void *UserDataPtr);
typedef RESULT (*IsiGetOtpDataIss_t)(IsiSensorHandle_t handle, IsiOTP_t *OtpDataPtr);
typedef RESULT (*IsiFocusCreateIss_t)(IsiSensorHandle_t handle);
typedef RESULT (*IsiFocusReleaseIss_t)(IsiSensorHandle_t handle);
typedef RESULT (*IsiFocusGetCalibrateIss_t)(IsiSensorHandle_t handle,
					IsiFocusCalibAttr_t *FocusCalibPtr);
typedef RESULT (*IsiFocusSetIss_t)(IsiSensorHandle_t handle, const IsiFocusPos_t *PosPtr);
typedef RESULT (*IsiFocusGetIss_t)(IsiSensorHandle_t handle, IsiFocusPos_t *PosPtr);
typedef RESULT (*IsiSetIRLightExpIss_t)(IsiSensorHandle_t handle, const IsiIrLightExp_t
					*IrExpParamPtr);
typedef RESULT (*IsiGetIRLightExpIss_t)(IsiSensorHandle_t handle, IsiIrLightExp_t *IrExpParamPtr);
typedef RESULT (*IsiQueryMetadataAttrIss_t)(IsiSensorHandle_t handle, IsiMetadataAttr_t *AttrPtr);
typedef RESULT (*IsiSetMetadataAttrEnableIss_t)(IsiSensorHandle_t handle, IsiMetadataAttr_t attr);
typedef RESULT (*IsiGetMetadataAttrEnableIss_t)(IsiSensorHandle_t handle, IsiMetadataAttr_t
					*AttrPtr);
typedef RESULT (*IsiGetMetadataWindowIss_t)(IsiSensorHandle_t handle, IsiMetadataWinInfo_t
					*MetaWinPtr);
typedef RESULT (*IsiParserMetadataIss_t)(IsiSensorHandle_t handle, const MetadataBufInfo_t
					*MetaBufPtr, IsiSensorMetadata_t *MetaInfoPtr);

struct IsiSensor_s {
	const char			*pszName;
	const IsiCaps_t			*pIsiCaps;
	IsiCreateIss_t			pIsiCreateIss;
	IsiOpenIss_t			pIsiOpenIss;
	IsiCloseIss_t			pIsiCloseIss;
	IsiReleaseIss_t			pIsiReleaseIss;
	IsiReadRegIss_t			pIsiReadRegIss;
	IsiWriteRegIss_t		pIsiWriteRegIss;
	IsiGetModeIss_t			pIsiGetModeIss;
	IsiEnumModeIss_t		pIsiEnumModeIss;
	IsiGetCapsIss_t			pIsiGetCapsIss;
	IsiCheckConnectionIss_t		pIsiCheckConnectionIss;
	IsiGetRevisionIss_t		pIsiGetRevisionIss;
	IsiSetStreamingIss_t		pIsiSetStreamingIss;
	IsiGetAeBaseInfoIss_t		pIsiGetAeBaseInfoIss;
	IsiExpDecomposeCtrlIss_t	pIsiExpDecomposeCtrlIss;
	IsiGetAGainIss_t		pIsiGetAGainIss;
	IsiSetAGainIss_t		pIsiSetAGainIss;
	IsiGetDGainIss_t		pIsiGetDGainIss;
	IsiSetDGainIss_t		pIsiSetDGainIss;
	IsiGetIntTimeIss_t		pIsiGetIntTimeIss;
	IsiSetIntTimeIss_t		pIsiSetIntTimeIss;
	IsiGetFpsIss_t			pIsiGetFpsIss;
	IsiSetFpsIss_t			pIsiSetFpsIss;
	IsiGetIspStatusIss_t		pIsiGetIspStatusIss;
	IsiSetBlcIss_t			pIsiSetBlcIss;
	IsiGetBlcIss_t			pIsiGetBlcIss;
	IsiSetWBIss_t			pIsiSetWBIss;
	IsiGetWBIss_t			pIsiGetWBIss;
	IsiSetTpgIss_t			pIsiSetTpgIss;
	IsiGetTpgIss_t			pIsiGetTpgIss;
	IsiGetExpandCurveIss_t		pIsiGetExpandCurveIss;
	IsiGetCompressCurveIss_t	pIsiGetCompressCurveIss;
	IsiExtendFuncIss_t		pIsiExtendFuncIss;
	IsiGetOtpDataIss_t		pIsiGetOtpDataIss;
	IsiFocusCreateIss_t		pIsiFocusCreateIss;
	IsiFocusReleaseIss_t		pIsiFocusReleaseIss;
	IsiFocusGetCalibrateIss_t	pIsiFocusGetCalibrateIss;
	IsiFocusSetIss_t		pIsiFocusSetIss;
	IsiFocusGetIss_t		pIsiFocusGetIss;
	IsiSetIRLightExpIss_t		pIsiSetIRLightExpIss;
	IsiGetIRLightExpIss_t		pIsiGetIRLightExpIss;
	IsiQueryMetadataAttrIss_t	pIsiQueryMetadataAttrIss;
	IsiSetMetadataAttrEnableIss_t	pIsiSetMetadataAttrEnableIss;
	IsiGetMetadataAttrEnableIss_t	pIsiGetMetadataAttrEnableIss;
	IsiGetMetadataWindowIss_t	pIsiGetMetadataWinIss;
	IsiParserMetadataIss_t		pIsiParserMetadataIss;
};

typedef RESULT (*IsiGetSensorIss_t)(IsiSensor_t *IsiSensorPtr);

typedef struct {
	uint32_t		cameraDriverID;
	IsiGetSensorIss_t	pIsiGetSensorIss;
	uint32_t		cameraDevId;
	uint32_t		instanceId;
} IsiCamDrvConfig_t;

/*****************************************************************************/
/**
 *          IsiSensorDrvHandleRegisterIss
 *
 * @brief   Sensor deiver handle register.
 *
 * @param   pCamDrvConfig     configuration of the isi camera drv
 * @param   pSensorHandle     produced sensor handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
RESULT IsiSensorDrvHandleRegisterIss(IsiCamDrvConfig_t *pCamDrvConfig,
					IsiSensorHandle_t *pSensorHandle);

/*****************************************************************************/
/**
 *          IsiSensorDrvHandleUnRegisterIss
 *
 * @brief   Sensor deiver handle register.
 *
 * @param   handle          isi sensor handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
RESULT IsiSensorDrvHandleUnRegisterIss(IsiSensorHandle_t handle);

#ifdef __cplusplus
}
#endif

#endif
