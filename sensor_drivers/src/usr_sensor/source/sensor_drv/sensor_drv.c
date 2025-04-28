/****************************************************************************
 *
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

CREATE_TRACER(SENSOR_DRV_INFO, "SENSOR_INFO: ", INFO, 1);
CREATE_TRACER(SENSOR_DRV_ERROR, "SENSOR_ERROR: ", ERROR, 1);

SensorDrvConfigList_t sensorCfgList[] = {
	{"0x03f10", 0x6c, 2, 1, {0x300a, 0x300b}, 0x5308},
};

RESULT SensorDrvConfigMapping(const char *pSensorName, IsiCamDrvConfig_t **pSensorConfig)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	uint8_t i = 0, start = 0, end = 7;
	char newSensorName[50] = "";
	char *sptr, *eptr;
	bool_t apuSensor = BOOL_FALSE;

	if (pSensorName == NULL)
		return (RET_NULL_POINTER);

	for (i = 0; pSensorName[i] != '\0'; ++i)
		newSensorName[i] = pSensorName[i];
	newSensorName[i] = '\0';

	if (i > 8) {
		sptr = newSensorName + start * sizeof(char);
		eptr = newSensorName + end * sizeof(char);
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
		{"semu", &Semu_IsiCamDrvConfig}
	};

	if (apuSensor) {
		*pSensorConfig = &virtualSensor_IsiCamDrvConfig;
		for (int i = 0; i < ARRAY_SIZE(sensorConfig); i++) {
			if (strcmp(sptr, sensorConfig[i].pSensorName) == 0) {
				(*pSensorConfig)->cameraDriverID =
						sensorConfig[i].pSensorConfig->cameraDriverID;

				TRACE(SENSOR_DRV_INFO, "%s: i=%d, match sensor name: %s\n",
					 __func__, i, sensorConfig[i].pSensorName);
				return RET_SUCCESS;
			}
		}
	} else {
		for (int i = 0; i < ARRAY_SIZE(sensorConfig); i++) {
			if (strcmp(pSensorName, sensorConfig[i].pSensorName) == 0) {
				*pSensorConfig = sensorConfig[i].pSensorConfig;

				TRACE(SENSOR_DRV_INFO, "%s: i=%d, match sensorname: %s success!\n",
						__func__, i, sensorConfig[i].pSensorName);
				return RET_SUCCESS;
			}
		}
	}

	TRACE(SENSOR_DRV_ERROR, "%s: Unsupport sensor %s !\n", __func__, pSensorName);
	return RET_NOTSUPP;
}

RESULT SensorDrvGetSensorNumber(uint16_t *pNumber)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	*pNumber = ARRAY_SIZE(sensorCfgList);

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

RESULT SensorDrvGetConfigList(SensorDrvList_t *pSensorDrvList)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);

	if (pSensorDrvList == NULL)
		return (RET_NULL_POINTER);

	strcpy(pSensorDrvList[0].name, "0x03f10");
	pSensorDrvList[0].pSensorMode = pox03f10_mode_info;

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}

RESULT SensorDrvGetPortInfo(sensorPortInfo_t *pPortInfo, uint32_t sensorDevId)
{
	TRACE(SENSOR_DRV_INFO, "%s: (enter)\n", __func__);
	RESULT result = RET_SUCCESS;

	if (pPortInfo == NULL)
		return (RET_NULL_POINTER);

	int32_t fd = -1;
	uint8_t busId = (uint8_t)sensorDevId;
	char pathDevName[32] = { 0 };

	sprintf(pathDevName, "/dev/i2c-%u", busId);

	fd = HalOpenI2cDevice(pathDevName);
	if (fd < 0) {
		TRACE(SENSOR_DRV_INFO, "%s:Can't open %s!\n", __func__, pathDevName);
		return RET_FAILURE;
	}

	for (uint8_t index = 0; index < ARRAY_SIZE(sensorCfgList); index++) {
		result = HalIoCtl(fd, (sensorCfgList[index].slaveAddr >> 1));
		if (result < 0) {
			TRACE(SENSOR_DRV_INFO, "%s:I2C_SLAVE_FORCE error!\n", __func__);
			continue;
		}

		uint16_t regVal[3];
		uint32_t sensorId = 0;

		for (uint8_t i = 0; i < 3; i++) {
			if (sensorCfgList[index].regAddr[i] == 0) {
				continue;
			} else {
				regVal[i] = 0;
				result	= HalSensorDrvReadI2CReg(sensorCfgList[index].slaveAddr,
						sensorCfgList[index].regWidth,
						sensorCfgList[index].dataWidth,
						sensorCfgList[index].regAddr[i], &regVal[i], fd);
			}
		}

		if ((sensorCfgList[index].regAddr[0] != 0) &&
			(sensorCfgList[index].regAddr[1] == 0) &&
			(sensorCfgList[index].regAddr[2] == 0)) {
			sensorId |= (regVal[0] & 0xffff);
		} else if ((sensorCfgList[index].regAddr[0] != 0)
			&& (sensorCfgList[index].regAddr[1] != 0) &&
			(sensorCfgList[index].regAddr[2] == 0)) {
			sensorId = (regVal[0] & 0xff) << 8;
			sensorId |= (regVal[1] & 0xff);
		} else if ((sensorCfgList[index].regAddr[0] != 0) &&
			(sensorCfgList[index].regAddr[1] != 0) &&
			(sensorCfgList[index].regAddr[2] != 0)) {
			sensorId = (regVal[0] & 0xff) << 16;
			sensorId |= ((regVal[1] & 0xff) << 8);
			sensorId |= (regVal[2] & 0xff);
		}

		if (sensorCfgList[index].sensorId == sensorId) {
			TRACE(SENSOR_DRV_INFO, "%s: sensorId = %x found in i2c-%u\n",
				__func__, sensorId, busId);
			strcpy(pPortInfo->name, sensorCfgList[index].pSensorName);
			pPortInfo->chipId = sensorId;
			break;
		}
	}

	HalCloseI2cDevice(fd);

	TRACE(SENSOR_DRV_INFO, "%s: (exit)\n", __func__);
	return RET_SUCCESS;
}
