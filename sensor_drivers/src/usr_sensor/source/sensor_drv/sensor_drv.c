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
#include <ebase/types.h>
#include <ebase/trace.h>
#include <common/return_codes.h>
#include <string.h>
#include <isi/isi_fmc.h>
#include <common/misc.h>


CREATE_TRACER(SENSOR_DRV_INFO, "SENSOR_INFO: ", INFO, 1);
CREATE_TRACER(SENSOR_DRV_ERROR, "SENSOR_ERROR: ", ERROR, 1);


extern IsiCamDrvConfig_t Ox03f10_IsiCamDrvConfig;
extern IsiCamDrvConfig_t Ox08b40_IsiCamDrvConfig;
extern IsiCamDrvConfig_t Ox05b1s_IsiCamDrvConfig;
extern IsiCamDrvConfig_t IMX623_IsiCamDrvConfig;
extern IsiCamDrvConfig_t IMX728_IsiCamDrvConfig;
extern IsiCamDrvConfig_t virtualSensor_IsiCamDrvConfig;
extern IsiCamDrvConfig_t Semu_IsiCamDrvConfig;

extern IsiSensorMode_t pox03f10_mode_info[];
extern IsiSensorMode_t pox08b40_mode_info[];
extern IsiSensorMode_t pox05b1s_mode_info[];

extern int ox03f10_mode_num;
extern int ox08b40_mode_num;
extern int ox05b1s_mode_num;

SensorDrvConfigList_t sensorCfgList[] = {
	{"ox03f10", {0x300a, 0x300b, 0x300c}, 0x580346},
	{"ox08b40", {0x300a, 0x300b, 0x300c}, 0x580841},
	{"ox05b1s", {0x300a, 0x300b}, 0x5805},
};

/*******************************************************************************
 *          SensorDrvConfigMapping
 *
 * @brief   Structure that maps sensor driver configuration parameters.
 *
 * @details This structure holds the mapping between configuration IDs and their
 *          corresponding configuration data for the sensor driver.
 *
 ******************************************************************************/
RESULT SensorDrvConfigMapping(const char *pSensorName, IsiCamDrvConfig_t **pSensorConfig)
{

	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	uint8_t i = 0, start = 0, end = 7;
	char newSensorName[50] = "";
	char *sptr, *eptr;
	bool_t apuSensor = BOOL_FALSE;

	if (pSensorName == NULL)
		return RET_NULL_POINTER;

	for (i = 0; pSensorName[i] != '\0'; ++i)
		newSensorName[i] = pSensorName[i];
	newSensorName[i] = '\0';
	if (i > 8) {
		sptr = newSensorName + start*sizeof(char);
		eptr = newSensorName + end*sizeof(char);

		*eptr = '\0';
		if (strcmp(sptr, "virtual") == 0) {
			TRACE(SENSOR_DRV_INFO, "%s: register sensor driver from APU.\n", __func__);
			sptr = eptr + sizeof(char);
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
		for (int i = 0; i < (int)(ARRAY_SIZE(sensorConfig)); i++) {
			if (strcmp(sptr, sensorConfig[i].pSensorName) == 0) {
				(*pSensorConfig)->cameraDriverID =
					sensorConfig[i].pSensorConfig->cameraDriverID;

				TRACE(SENSOR_DRV_INFO, "%s: i=%d, match sensor name: %s success!!\n",
						__func__, i, sensorConfig[i].pSensorName);
				return RET_SUCCESS;
			}
		}

	} else {

		for (int i = 0; i < (int)(ARRAY_SIZE(sensorConfig)); i++) {
			if (strcmp(pSensorName, sensorConfig[i].pSensorName) == 0) {
				*pSensorConfig = sensorConfig[i].pSensorConfig;

				TRACE(SENSOR_DRV_INFO, "%s: i=%d, match sensor name: %s success!!\n",
						__func__, i, sensorConfig[i].pSensorName);
				return RET_SUCCESS;
			}
		}
	}


	TRACE(SENSOR_DRV_ERROR, "%s: Unsupport sensor %s !\n", __func__, pSensorName);
	return RET_NOTSUPP;
}

/*******************************************************************************
 *          SensorDrvGetSensorNumber
 *
 * @brief   Retrieves the number of sensors supported by the driver.
 *
 * @param   pNumber            Pointer to the variable to store the number of sensors.
 *
 * @return  Number of supported sensors.
 *
 ******************************************************************************/
RESULT SensorDrvGetSensorNumber(uint16_t    *pNumber)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	*pNumber = (ARRAY_SIZE(sensorCfgList));

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/*******************************************************************************
 *          SensorDrvGetConfigList
 *
 * @brief   Provides the list of available sensor configurations.
 *
 * @param   sensorNum          Number of sensors to retrieve configurations for.
 * @param   pSensorDrvList     Pointer to the sensor driver list to be filled.
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         On success.
 * @retval  RET_NULL_POINTER    If a null pointer is provided.
 *
 ******************************************************************************/
RESULT SensorDrvGetConfigList(const uint16_t sensorNum, SensorDrvList_t *pSensorDrvList)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	if (pSensorDrvList == NULL)
		return (RET_NULL_POINTER);

	SensorDrvList_t sensorConfig[] = {
		{"ox03f10", pox03f10_mode_info, ox03f10_mode_num},
		{"ox05b1s", pox05b1s_mode_info, ox05b1s_mode_num},
		{"ox08b40", pox08b40_mode_info, ox08b40_mode_num},
	};

	if (sensorNum > (ARRAY_SIZE(sensorConfig))) {
		TRACE(SENSOR_DRV_ERROR, "%s sensorNum is bigger than supported sensor number!\n",
				__func__);
		return RET_FAILURE;
	}

	for (uint8_t i = 0; i < sensorNum; i++) {
		strcpy(pSensorDrvList[i].name, sensorConfig[i].name);
		pSensorDrvList[i].pSensorMode = sensorConfig[i].pSensorMode;
		pSensorDrvList[i].mode_num = sensorConfig[i].mode_num;
	}

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

/*******************************************************************************
 *          SensorDrvGetPortInfo
 *
 * @brief   Retrieves port information for a specific sensor.
 *
 * @param   sensorDevId            ID of the sensor.
 * @param   pPortInfo           Pointer to the structure to be filled with port information.
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         On success.
 * @retval  RET_INVALID_PARM    If an invalid parameter is provided.
 *
 ******************************************************************************/
RESULT SensorDrvGetPortInfo(sensorPortInfo_t *pPortInfo, uint32_t sensorDevId)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	if (pPortInfo == NULL)
		return (RET_NULL_POINTER);

	uint8_t pipeId = sensorDevId;
	uint8_t desId = MAPPING_INPIPE_TO_DES_ID(pipeId);

	init_iic_access(0, pipeId);
	int Status = init_des(desId);

	if (Status != XST_SUCCESS)
		return Status;
	init_sensor(pipeId, desId);

	for (uint8_t index = 0; index < (ARRAY_SIZE(sensorCfgList)); index++) {

		uint16_t regVal[3];
		uint32_t sensorId = 0;

		for (uint8_t i = 0; i < 3; i++) {
			if (sensorCfgList[index].regAddr[i] == 0) {
				continue;
			} else {
				regVal[i] = 0;
				u8 slave_addr =
					(g_fmc_single.sensor_array[sensorDevId]->sensor_alias_addr)
						>> 1;

				g_fmc_single.accessiic_array[sensorDevId]->readIIC(0, slave_addr,
						sensorCfgList[index].regAddr[i], 0x2, &regVal[i],
						1);
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
			TRACE(SENSOR_DRV_INFO, "%s: sensorId=%x found in sensorDevId-%u!\n",
					__func__, sensorId, sensorDevId);
			strcpy(pPortInfo->name, sensorCfgList[index].pSensorName);
			pPortInfo->chipId = sensorId;
			break;
		}
	}

	stop_sensor(pipeId);

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}
