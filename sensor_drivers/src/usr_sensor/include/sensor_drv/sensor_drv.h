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

#ifndef __SENSOR_DRV_H__
#define __SENSOR_DRV_H__

#include "isi/isi_iss.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
	const char		*pSensorName;
	IsiCamDrvConfig_t	*pSensorConfig;
} SensorDrvConfig_t;

typedef struct {
	char			name[20];
	IsiSensorMode_t		*pSensorMode;
	int			mode_num;
} SensorDrvList_t;

typedef struct {
	uint32_t	chipId;
	char		name[20];
} sensorPortInfo_t;

typedef struct {
	char		*pSensorName;
	uint16_t	regAddr[3];
	uint32_t	sensorId;
} SensorDrvConfigList_t;

/*****************************************************************************/
/**
 *          SensorDrvConfigMapping
 *
 * @brief   sensor config mapping.
 *
 * @param   pSensorName      Pointer to the sensor name
 * @param   pSensorConfig    Pointer to the isi sensor driver config
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
RESULT SensorDrvConfigMapping(const char *pSensorName, IsiCamDrvConfig_t **pSensorConfig);

/*****************************************************************************/
/**
 *          SensorDrvGetSensorNumber
 *
 * @brief   Get the number of available sensors.
 *
 * @param   pNumber         Pointer to store the sensor number
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT SensorDrvGetSensorNumber(uint16_t *pNumber);

/*****************************************************************************/
/**
 *          SensorDrvGetConfigList
 *
 * @brief   Get sensor driver configuration list.
 *
 * @param   sensorNum           Sensor number
 * @param   pSensorDrvList      Pointer to sensor driver list structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT SensorDrvGetConfigList(const uint16_t sensorNum, SensorDrvList_t *pSensorDrvList);

/*****************************************************************************/
/**
 *          SensorDrvGetPortInfo
 *
 * @brief   Get sensor port information.
 *
 * @param   pPortInfo       Pointer to sensor port information structure
 * @param   sensorDevId     Sensor device ID
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT SensorDrvGetPortInfo(sensorPortInfo_t *pPortInfo, uint32_t sensorDevId);

#ifdef __cplusplus
}
#endif

#endif
