/****************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Advanced Micro Devices, Inc. All right reserved.
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

#include "sensor_drv/sensor_drv.h"
#include <string.h>
#include <isi/isi_fmc.h>

CREATE_TRACER(SENSOR_DRV_INFO, "SENSOR_INFO: ", INFO, 1);
CREATE_TRACER(SENSOR_DRV_ERROR, "SENSOR_ERROR: ", ERROR, 1);

#define SENSOR_NAME_MAX_LEN      50
#define VIRTUAL_PREFIX_LEN       7
#define VIRTUAL_MIN_NAME_LEN     8
#define MAX_SENSOR_ID_REGS       3
#define I2C_REG_ADDR_WIDTH_2     0x2
#define DEFAULT_I2C_BUS_ID       8

extern int current_active_fmc;

SensorDrvConfigList_t sensorCfgList[] = {
	{"ox03f10", SENSOR_OX3F10_ADDRESS, REGWIDTH, DATAWIDTH,
		{SC_CTRL_A, SC_CTRL_B, SC_CTRL_C}, OX03F10_SENSOR_ID},
	{"ox08b40", SENSOR_OX8B40_ADDRESS, REGWIDTH, DATAWIDTH,
		{SC_CTRL_A, SC_CTRL_B, SC_CTRL_C}, OX08B40_SENSOR_ID},
	{"ox05b1s", SENSOR_OX5B_ADDRESS, REGWIDTH, DATAWIDTH,
		{SC_CTRL_A, SC_CTRL_B}, OX05B1S_SENSOR_ID},
	{"imx728", SENSOR_IMX728_ADDRESS, REGWIDTH, DATAWIDTH,
		{CHIP_ID_1, CHIP_ID_0}, IMX728_SENSOR_ID},
	{"imx623", SENSOR_IMX623_ADDRESS, REGWIDTH, DATAWIDTH,
		{CHIP_ID_1, CHIP_ID_0}, IMX623_SENSOR_ID}
};

/****************************************************************************
 *          SensorDrvConfigMapping
 *
 * @brief   Structure that maps sensor driver configuration parameters.
 *
 * @details This structure holds the mapping between
 *			configuration IDs and their
 *          corresponding configuration data for the sensor driver.
 *
 ***************************************************************************/
RESULT SensorDrvConfigMapping(const CamDeviceSensorModuleMapCfg_t
	*pModuleInfo, IsiCamDrvConfig_t **pSensorConfig
)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	uint8_t start = 0, end = VIRTUAL_PREFIX_LEN;
	char newSensorName[SENSOR_NAME_MAX_LEN] = "";
	char *sptr, *eptr;
	bool_t apuSensor = BOOL_FALSE;

	if (pModuleInfo == NULL) {
		TRACE(SENSOR_DRV_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	strncpy(newSensorName, pModuleInfo->moduleName,
		SENSOR_NAME_MAX_LEN - 1);
	newSensorName[SENSOR_NAME_MAX_LEN - 1] = '\0';
	if (strlen(newSensorName) > VIRTUAL_MIN_NAME_LEN) {
		sptr = newSensorName + start;
		eptr = newSensorName + end;

		*eptr = '\0';
		if (strcmp(sptr, "virtual") == 0) {
			TRACE(SENSOR_DRV_INFO,
					"%s: register sensor driver from APU.\n",
					__func__);
			sptr = eptr + 1;
			apuSensor = BOOL_TRUE;
		}

	}
	SensorDrvConfig_t sensorConfig[] = {
		{"ox03f10", &Ox03f10_IsiCamDrvConfig},
		{"ox08b40", &Ox08b40_IsiCamDrvConfig},
		{"ox05b1s", &Ox05b1s_IsiCamDrvConfig},
		{"imx728", &IMX728_IsiCamDrvConfig},
		{"imx623", &IMX623_IsiCamDrvConfig},
		{"semu",    &Semu_IsiCamDrvConfig   }
	};

	if (apuSensor) {
		*pSensorConfig = &virtualSensor_IsiCamDrvConfig;
		int cfg_cnt = (int)(ARRAY_SIZE(sensorConfig));

		for (int cfg_idx = 0; cfg_idx < cfg_cnt; cfg_idx++) {
			if (strcmp(sptr, sensorConfig[cfg_idx].pSensorName) == 0) {
				(*pSensorConfig)->cameraDriverID =
					sensorConfig[cfg_idx].pSensorConfig->cameraDriverID;
				(*pSensorConfig)->sensorDevId = pModuleInfo->sensorDevId;
				TRACE(SENSOR_DRV_INFO,
						"%s: cfg_idx=%d, match: %s success!!\n",
						__func__, cfg_idx,
						sensorConfig[cfg_idx].pSensorName);
				return RET_SUCCESS;
			}
		}

	} else {
		int cfg_cnt = (int)(ARRAY_SIZE(sensorConfig));

		for (int cfg_idx = 0; cfg_idx < cfg_cnt; cfg_idx++) {
			const char *modName = pModuleInfo->moduleName;

			if (strcmp(modName, sensorConfig[cfg_idx].pSensorName) == 0) {
				*pSensorConfig = sensorConfig[cfg_idx].pSensorConfig;
				(*pSensorConfig)->sensorDevId = pModuleInfo->sensorDevId;
				TRACE(SENSOR_DRV_INFO,
						"%s: cfg_idx=%d, match: %s success!!\n",
						__func__, cfg_idx,
						sensorConfig[cfg_idx].pSensorName);
				return RET_SUCCESS;
			}
		}
	}


	TRACE(SENSOR_DRV_ERROR, "%s: Unsupported sensor %s !\n",
			__func__, pModuleInfo->moduleName);
	return RET_NOTSUPP;
}

/****************************************************************************
 *          SensorDrvGetSensorNumber
 *
 * @brief   Retrieves the number of sensors supported by the driver.
 *
 * @param   pNumber	Pointer to the variable to store the number of sensors.
 *
 * @return  Number of supported sensors.
 *
 ***************************************************************************/
RESULT SensorDrvGetSensorNumber(uint16_t    *pNumber)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	if (pNumber == NULL) {
		TRACE(SENSOR_DRV_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	*pNumber = (ARRAY_SIZE(sensorCfgList));

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/***************************************************************************
 *          SensorDrvGetConfigList
 *
 * @brief   Provides the list of available sensor configurations.
 *
 * @param   sensorNum          Number of sensors to retrieve
 *								configurations for.
 * @param   pSensorDrvList     Pointer to the sensor driver list to be filled.
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         On success.
 * @retval  RET_NULL_POINTER    If a null pointer is provided.
 *
 ****************************************************************************/
RESULT SensorDrvGetConfigList(const uint16_t sensorNum,
		SensorDrvList_t *pSensorDrvList)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	if (pSensorDrvList == NULL){
		TRACE(SENSOR_DRV_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	SensorDrvList_t sensorConfig[] = {
		{"ox03f10", pox03f10_mode_info},
		{"ox05b1s", pox05b1s_mode_info},
		{"ox08b40", pox08b40_mode_info},
		{"imx728", pimx728_mode_info},
		{"imx623", pimx623_mode_info},
	};

	if (sensorNum > (ARRAY_SIZE(sensorConfig))) {
		TRACE(SENSOR_DRV_ERROR,
				"%s sensorNum is bigger than supported sensor number!\n",
				__func__);
		return RET_FAILURE;
	}

	for (uint8_t sensor_idx = 0; sensor_idx < sensorNum; sensor_idx++) {
		SensorDrvList_t *pDst = &pSensorDrvList[sensor_idx];
		SensorDrvList_t *pSrc = &sensorConfig[sensor_idx];

		strncpy(pDst->name, pSrc->name, sizeof(pDst->name) - 1);
		pDst->name[sizeof(pDst->name) - 1] = '\0';
		pDst->pSensorMode = pSrc->pSensorMode;
	}

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/****************************************************************************
 *          SensorDrvGetPortInfo
 *
 * @brief   Retrieves port information for a specific sensor.
 *
 * @param   sensorDevId		ID of the sensor.
 * @param   pPortInfo		Pointer to the structure to be
			    filled with port information.
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         On success.
 * @retval  RET_INVALID_PARM    If an invalid parameter is provided.
 *
 ***************************************************************************/
RESULT SensorDrvGetPortInfo(sensorPortInfo_t *pPortInfo, uint32_t sensorDevId)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	if (pPortInfo == NULL){
		TRACE(SENSOR_DRV_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}

	uint8_t pipeId = sensorDevId;
	uint8_t desId = MAPPING_INPIPE_TO_DES_ID(pipeId);
	uint8_t found = BOOL_FALSE;
	
	init_iic_access(0, pipeId);
	RESULT Status = init_des(desId);

	if (Status != RET_SUCCESS)
		return Status;
	init_sensor(pipeId, desId, 0);
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(SENSOR_DRV_ERROR, "%s: No FMC selected\r\n", __func__);
		return RET_INVALID_PARM;
	}

	for (uint8_t index = 0; index < (ARRAY_SIZE(sensorCfgList)); index++) {
		uint8_t regVal[3] = {0};
		uint32_t sensorId = 0;

		for (uint8_t reg_idx = 0; reg_idx < MAX_SENSOR_ID_REGS; reg_idx++) {
			uint16_t regAddr = sensorCfgList[index].regAddr[reg_idx];

			if (regAddr == 0) {
				continue;
			} else {
				u8 slave_addr =
					(active_fmc->sensor_array
						[sensorDevId]->sensor_alias_addr) >> 1;

				active_fmc->accessiic_array[sensorDevId]->readIIC(
					0, slave_addr, regAddr,
					I2C_REG_ADDR_WIDTH_2, &regVal[reg_idx], 1);
			}
		}

		if ((sensorCfgList[index].regAddr[0] != 0)
				&& (sensorCfgList[index].regAddr[1] == 0)
				&& (sensorCfgList[index].regAddr[2] == 0)) {
			sensorId |= (regVal[0] & 0xffff);
		} else if ((sensorCfgList[index].regAddr[0] != 0)
				&& (sensorCfgList[index].regAddr[1] != 0)
				&& (sensorCfgList[index].regAddr[2] == 0)) {
			sensorId = (regVal[0] & 0xff) << 8;
			sensorId |= (regVal[1] & 0xff);
		} else if ((sensorCfgList[index].regAddr[0] != 0)
				&& (sensorCfgList[index].regAddr[1] != 0)
				&& (sensorCfgList[index].regAddr[2] != 0)) {
			sensorId = (regVal[0] & 0xff) << 16;
			sensorId |= ((regVal[1] & 0xff) << 8);
			sensorId |= (regVal[2] & 0xff);
		}

		if (sensorCfgList[index].sensorId == sensorId) {
			TRACE(SENSOR_DRV_INFO,
				"%s: sensorId=%x found in sensorDevId-%u!\n",
				__func__, sensorId, sensorDevId);
			const char *name = sensorCfgList[index].pSensorName;
			found = BOOL_TRUE;
			strncpy(pPortInfo->name, name,
				sizeof(pPortInfo->name) - 1);
			pPortInfo->name[sizeof(pPortInfo->name) - 1] = '\0';
			pPortInfo->chipId = sensorId;
			break;
		}
	}

	if (!found) {
		TRACE(SENSOR_DRV_ERROR,
			"%s: sensorId not found in sensorDevId-%u!\n",
			__func__, sensorDevId);
		stop_sensor(pipeId);
		return RET_FAILURE;
	}


	stop_sensor(pipeId);

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}



static RESULT CamDeviceSensorIsiGetListInfo
(
	CamDeviceSensorListInfo_t *pSensorListInfo,
	const uint16_t sensorNum
)
{
	RESULT result = RET_SUCCESS;


	if (pSensorListInfo == NULL){
		TRACE(SENSOR_DRV_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_INVALID_PARM;
	}

	size_t allocSize = sizeof(SensorDrvList_t) * sensorNum;
	SensorDrvList_t *pSensorDrvList = (SensorDrvList_t *)osMalloc(allocSize);

	if (pSensorDrvList == NULL) {
		TRACE(SENSOR_DRV_ERROR, "%s (allocating sensor driver list failed)\n",
				__func__);
		return RET_OUTOFMEM;
	}
	memset(pSensorDrvList, 0, sensorNum * sizeof(SensorDrvList_t));
	result = SensorDrvGetConfigList(sensorNum, pSensorDrvList);
	if (result != RET_SUCCESS) {
		TRACE(SENSOR_DRV_ERROR, "%s SensorDrvGetConfigList failed:%d\n",
				__func__, result);
		osFree(pSensorDrvList);
		return result;
	}

	IsiSensorMode_t *pModeInfo = NULL;

	for (uint8_t num = 0; num < sensorNum; num++) {
		CamDeviceSensorListInfo_t *pInfo = &pSensorListInfo[num];

		strncpy(pInfo->name, pSensorDrvList[num].name,
			sizeof(pInfo->name) - 1);
		pInfo->name[sizeof(pInfo->name) - 1] = '\0';
		pModeInfo = pSensorDrvList[num].pSensorMode;

		/* SENSOR_DRV_NUM is the max modes per sensor (not sensor count).
		 * Loop breaks early on sentinel (index==0) after first entry. */
		for (uint8_t index = 0; index < SENSOR_DRV_NUM; index++) {
			if ((index > 0) && (pModeInfo->index == 0))
				break;
			CamDeviceSensorDrvModeInfo_t *pMode =
				&(pInfo->sensorModeInfo[index]);

			pInfo->number++;

			pMode->index = pModeInfo->index;
			pMode->width = pModeInfo->size.width;
			pMode->height = pModeInfo->size.height;
			pMode->bitWidth = pModeInfo->bitWidth;
			pMode->maxFps = (pModeInfo->fps)/ISI_FPS_QUANTIZE;
			pMode->sensorType = pModeInfo->hdrMode;
			pMode->stitchingMode = pModeInfo->stitchingMode;
			pMode->nativeMode = pModeInfo->nativeMode;
			pMode->bayerPattern = pModeInfo->bayerPattern;
			pMode->afMode = pModeInfo->afMode;
			pMode->dataType = pModeInfo->dataType;
			pMode->itfType = pModeInfo->mipiLane;
			memcpy(&pMode->aeInfo, &pModeInfo->aeInfo,
				sizeof(IsiSensorAeInfo_t));

			pModeInfo++;
		}
	}
	osFree(pSensorDrvList);

	return RET_SUCCESS;
}

/**************************************************************************
 *          VsiCamDeviceSensorGetNumber
 *
 * @brief   Get number of sensors
 *
 * @param   pNumber     Pointer to store sensor count
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
RESULT VsiCamDeviceSensorGetNumber(uint16_t *pNumber)
{
	RESULT result = RET_SUCCESS;

	TRACE(SENSOR_DRV_INFO, "%s (enter)\n", __func__);

	if (pNumber == NULL) {
		TRACE(SENSOR_DRV_ERROR, "%s: pNumber is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	result = SensorDrvGetSensorNumber(pNumber);
	if (result != RET_SUCCESS) {
		TRACE(SENSOR_DRV_ERROR, "%s CamDeviceSensorIsiGetNumber failed:%d\n",
				__func__, result);
		return RET_FAILURE;
	}

	TRACE(SENSOR_DRV_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          VsiCamDeviceSensorGetListInfo
 *
 * @brief   Get sensor list information
 *
 * @param   pSensorListInfo  Sensor list info output
 * @param   sensorNum        Sensor number
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
RESULT VsiCamDeviceSensorGetListInfo(CamDeviceSensorListInfo_t
	*pSensorListInfo, const uint16_t sensorNum)
{
	RESULT result = RET_SUCCESS;

	TRACE(SENSOR_DRV_INFO, "%s (enter)\n", __func__);

	if (pSensorListInfo == NULL) {
		TRACE(SENSOR_DRV_ERROR, "%s: pSensorListInfo is NULL\n",
				__func__);
		return RET_NULL_POINTER;
	}

	result = CamDeviceSensorIsiGetListInfo(pSensorListInfo, sensorNum);
	if (result != RET_SUCCESS) {
		TRACE(SENSOR_DRV_ERROR, "%s CamDeviceSensorIsiGetListInfo failed:%d\n",
				__func__, result);
		return RET_FAILURE;
	}

	TRACE(SENSOR_DRV_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          VsiCamDeviceSensorGetConnectPortInfo
 *
 * @brief   Get sensor connection port info
 *
 * @param   sensorDrvId  Sensor driver ID
 * @param   pPortInfo    Port info output
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
RESULT VsiCamDeviceSensorGetConnectPortInfo(SensorDrvId_t sensorDrvId,
						    CamDeviceSensorConnectPortInfo_t *pPortInfo)
{
	RESULT result = RET_SUCCESS;
	uint8_t desId = MAPPING_INPIPE_TO_DES_ID(sensorDrvId);

	TRACE(SENSOR_DRV_INFO, "%s (enter)\n", __func__);

	if (pPortInfo == NULL) {
		TRACE(SENSOR_DRV_ERROR, "%s: pPortInfo is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (sensorDrvId >= SENSOR_DRV_ID_MAX) {
		TRACE(SENSOR_DRV_ERROR, "%s: sensor drv index out of range: %d\n",
				__func__, sensorDrvId);
		return RET_OUTOFRANGE;
	}

	uint8_t busId = GetI2cBusIdForDes(desId);
	if(busId == INVALID_I2C_BUS_ID) {
		TRACE(SENSOR_DRV_ERROR, "%s: Invalid I2C bus ID for desId %d\n", __func__, desId);
		return RET_INVALID_PARM;
	}
	sensorPortInfo_t portInfo;

	memset(&portInfo, 0, sizeof(sensorPortInfo_t));
	result = SensorDrvGetPortInfo(&portInfo, busId);
	if (result != RET_SUCCESS) {
		TRACE(SENSOR_DRV_ERROR, "%s SensorDrvGetPortInfo failed:%d\n",
				__func__, result);
		return RET_FAILURE;
	}
	strncpy(pPortInfo->name, portInfo.name, sizeof(pPortInfo->name) - 1);
	pPortInfo->name[sizeof(pPortInfo->name) - 1] = '\0';
	pPortInfo->chipId = portInfo.chipId;


	TRACE(SENSOR_DRV_INFO, "%s (exit)\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          VsiCamDeviceSensorMapping
 *
 * @brief   Map sensor module configuration
 *
 * @param   pModuleInfo       Module map config
 * @param   pSensorDrvhandle  Sensor driver handle output
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/
RESULT VsiCamDeviceSensorMapping(const CamDeviceSensorModuleMapCfg_t
	*pModuleInfo, CamDeviceSensorDrvHandle_t *pSensorDrvhandle)
{
	RESULT result = RET_SUCCESS;

	TRACE(SENSOR_DRV_INFO, "%s (enter)\n", __func__);

	if (pModuleInfo == NULL || pSensorDrvhandle == NULL){
		TRACE(SENSOR_DRV_ERROR, "%s: NULL pointer detected\n", __func__);
		return RET_NULL_POINTER;
	}
	*pSensorDrvhandle = NULL;
	result = SensorDrvConfigMapping(pModuleInfo,
				    (IsiCamDrvConfig_t **)pSensorDrvhandle);
	if (result != RET_SUCCESS) {
		TRACE(SENSOR_DRV_ERROR, "%s IsiSensorCongigMapping failed:%d\n",
				__func__, result);
		return result;
	}

	TRACE(SENSOR_DRV_INFO, "%s (exit)\n", __func__);

	return RET_SUCCESS;
}

/**************************************************************************
 *          SelectFMCID
 *
 * @brief   Map FMC ID of current active FMC
 *
 * @param   fmc_id       FMC ID
 *
 * @return  RET_SUCCESS or error
 *
 ************************************************************************/

RESULT SelectFMCID(int fmc_id)
{
    RESULT result = RET_SUCCESS;

    TRACE(SENSOR_DRV_INFO, "%s (enter)\n", __func__);
    TRACE(SENSOR_DRV_INFO, "FMC ID Passed: %d\n",fmc_id);

    if(fmc_id == 0 || fmc_id == 1){
    	current_active_fmc = fmc_id;
		TRACE(SENSOR_DRV_INFO, "FMC ID updated as: %d ----- %d\n",current_active_fmc,__LINE__);
	} else {
		TRACE(SENSOR_DRV_ERROR, "INVALID FMC ID Passed\n");
		return RET_FAILURE;

	}

    TRACE(SENSOR_DRV_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}