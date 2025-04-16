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
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/misc.h>
#include <isi/isi_fmc.h>
#include "isi/isi.h"
#include "isi/isi_iss.h"
#include "isi/isi_priv.h"
#include "sensor_drv/virtual_sensor.h"
#include "mbox/sensor_cmd.h"

CREATE_TRACER( virtualSensor_INFO , "virtualSensor: ", INFO,    1 );
CREATE_TRACER( virtualSensor_WARN , "virtualSensor: ", WARNING, 1 );
CREATE_TRACER( virtualSensor_ERROR, "virtualSensor: ", ERROR,   1 );
CREATE_TRACER( virtualSensor_DEBUG,     "virtualSensor: ", INFO, 0 );
CREATE_TRACER( virtualSensor_REG_INFO , "virtualSensor: ", INFO, 1 );
CREATE_TRACER( virtualSensor_REG_DEBUG, "virtualSensor: ", INFO, 1 );

/*****************************************************************************
 *Sensor Info
*****************************************************************************/

extern uint32_t dest_cpu_id;
extern uint32_t cookieRpu;


static RESULT virtualSensor_IsiGetModeIss(IsiSensorHandle_t handle, IsiSensorMode_t *pMode)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pMode == NULL)
		return (RET_NULL_POINTER);


	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pMode, sizeof(IsiSensorMode_t));
	packet.payload_size += sizeof(IsiSensorMode_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetModeIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pMode, p_data, sizeof(IsiSensorMode_t));

	xil_printf( "RPU return pMode->fps: %d \r\n", pMode->fps);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static  RESULT virtualSensor_IsiEnumModeIss(IsiSensorHandle_t handle, IsiSensorEnumMode_t *pEnumMode)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pEnumMode == NULL)
		return (RET_NULL_POINTER);

    xil_printf( "RPU pEnumMode->index: %d \r\n", pEnumMode->index);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pEnumMode, sizeof(IsiSensorEnumMode_t));
	packet.payload_size += sizeof(IsiSensorEnumMode_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiEnumModeIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pEnumMode, p_data, sizeof(IsiSensorEnumMode_t));

	xil_printf( "RPU return pEnumMode->index: %d \r\n", pEnumMode->index);
	xil_printf( "RPU return pEnumMode->mode.fps: %d \r\n", pEnumMode->mode.fps);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetCapsIss(IsiSensorHandle_t handle, IsiCaps_t *pCaps)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pCaps == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiCaps_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetCapsIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pCaps, p_data, sizeof(IsiCaps_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiCreateIss(IsiSensorInstanceConfig_t *pConfig, IsiSensorHandle_t *pHandle)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t *) osMalloc(sizeof(VirtualSensor_Context_t));
    if (!pVirtualSensorCtx) {
        TRACE(virtualSensor_ERROR, "%s: Can't allocate virtual sensor context\n", __func__);
        return (RET_OUTOFMEM);
    }

    MEMSET(pVirtualSensorCtx, 0, sizeof(VirtualSensor_Context_t));

    if( XPAR_CPU_ID == 0){  //RPU0
		pVirtualSensorCtx->rpu_id = MBOX_CORE_RPU0;
	}

	if(XPAR_CPU_ID == 1){  //RPU1
		pVirtualSensorCtx->rpu_id = MBOX_CORE_RPU1;
	}

    pVirtualSensorCtx->isiCtx.pSensor = pConfig->pSensor;
    pVirtualSensorCtx->instanceId = pConfig->instanceID;

    *pHandle = (IsiSensorHandle_t) pVirtualSensorCtx;


    // mailbox
    Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pConfig, sizeof(IsiSensorInstanceConfig_t));
	packet.payload_size += sizeof(IsiSensorInstanceConfig_t);


	result = Send_Command (RPU_2_APU_MB_CMD_IsiCreateIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);

    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

//static RESULT virtualSensor_AecSetModeParameters(IsiSensorHandle_t handle, Ox03f10_Context_t *pOx03f10Ctx)
//{
//    RESULT result = RET_SUCCESS;
//    TRACE(virtualSensor_INFO, "%s%s: (enter)\n", __func__, pOx03f10Ctx->isAfpsRun ? "(AFPS)" : "");
//
//
//    TRACE(virtualSensor_INFO, "%s: (exit)\n", __func__);
//
//    return (result);
//}

static RESULT virtualSensor_IsiOpenIss(IsiSensorHandle_t handle, uint32_t mode)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, &mode, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiOpenIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiCloseIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiCloseIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiReleaseIss(IsiSensorHandle_t handle)
{

    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    // free virtual context
    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

    MEMSET(pVirtualSensorCtx, 0, sizeof(VirtualSensor_Context_t));
	osFree(pVirtualSensorCtx);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiReleaseIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiCheckConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiCheckConnectionIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pValue == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetRevisionIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pValue, p_data, sizeof(uint32_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetStreamingIss(IsiSensorHandle_t handle, bool_t on)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, &on, sizeof(bool_t));
	p_data += sizeof(bool_t);
	packet.payload_size += sizeof(bool_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetStreamingIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_pIsiGetAeBaseInfoIss(IsiSensorHandle_t handle, IsiAeBaseInfo_t *pAeBaseInfo)
{

    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pAeBaseInfo == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiAeBaseInfo_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetAeBaseInfoIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pAeBaseInfo, p_data, sizeof(IsiAeBaseInfo_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pSensorAGain == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pSensorAGain, sizeof(IsiSensorGain_t));
	p_data += sizeof(IsiSensorGain_t);
	packet.payload_size += sizeof(IsiSensorGain_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetAGainIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pSensorDGain == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pSensorDGain, sizeof(IsiSensorGain_t));
	p_data += sizeof(IsiSensorGain_t);
	packet.payload_size += sizeof(IsiSensorGain_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetDGainIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetAGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorAGain)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pSensorAGain == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiSensorGain_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetAGainIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pSensorAGain, p_data, sizeof(IsiSensorGain_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetDGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pSensorDGain)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pSensorDGain == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiSensorGain_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetDGainIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pSensorDGain, p_data, sizeof(IsiSensorGain_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pSensorIntTime == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pSensorIntTime, sizeof(IsiSensorIntTime_t));
	p_data += sizeof(IsiSensorIntTime_t);
	packet.payload_size += sizeof(IsiSensorIntTime_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetIntTimeIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}


static RESULT virtualSensor_IsiGetIntTimeIss(IsiSensorHandle_t handle, IsiSensorIntTime_t *pSensorIntTime)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pSensorIntTime == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiSensorIntTime_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetIntTimeIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pSensorIntTime, p_data, sizeof(IsiSensorIntTime_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetFpsIss(IsiSensorHandle_t handle, uint32_t *pFps)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pFps == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetFpsIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pFps, p_data, sizeof(uint32_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, &fps, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetFpsIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetIspStatusIss(IsiSensorHandle_t handle, IsiIspStatus_t *pIspStatus)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pIspStatus == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiIspStatus_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetIspStatusIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pIspStatus, p_data, sizeof(IsiIspStatus_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t tpg)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, &tpg, sizeof(IsiSensorTpg_t));
	p_data += sizeof(IsiSensorTpg_t);
	packet.payload_size += sizeof(IsiSensorTpg_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetTpgIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetTpgIss(IsiSensorHandle_t handle, IsiSensorTpg_t *pTpg)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pTpg == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiSensorTpg_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetTpgIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pTpg, p_data, sizeof(IsiSensorTpg_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pWb == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pWb, sizeof(IsiSensorWb_t));
	p_data += sizeof(IsiSensorWb_t);
	packet.payload_size += sizeof(IsiSensorWb_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetWBIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetWBIss(IsiSensorHandle_t handle, IsiSensorWb_t *pWb)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pWb == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiSensorWb_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetWBIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pWb, p_data, sizeof(IsiSensorWb_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pBlc == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, pBlc, sizeof(IsiSensorBlc_t));
	p_data += sizeof(IsiSensorBlc_t);
	packet.payload_size += sizeof(IsiSensorBlc_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiSetBlcIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiGetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t *pBlc)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pBlc == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiSensorBlc_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetBlcIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pBlc, p_data, sizeof(IsiSensorBlc_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}


static RESULT virtualSensor_IsiGetExpandCurveIss(IsiSensorHandle_t handle, IsiSensorCompandCurve_t *pCurve)
{
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pCurve == NULL)
		return (RET_NULL_POINTER);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	packet.payload_size += sizeof(IsiSensorCompandCurve_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiGetExpandCurveIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pCurve, p_data, sizeof(IsiSensorCompandCurve_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiWriteRegIss(IsiSensorHandle_t handle, const uint16_t addr, const uint16_t value){
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);

	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, &addr, sizeof(uint16_t));
	p_data += sizeof(uint16_t);
	packet.payload_size += sizeof(uint16_t);
	memcpy(p_data, &value, sizeof(uint16_t));
	p_data += sizeof(uint16_t);
	packet.payload_size += sizeof(uint16_t);

	result = Send_Command (RPU_2_APU_MB_CMD_IsiWriteRegIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_ACK(packet.cookie);


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

static RESULT virtualSensor_IsiReadRegIss(IsiSensorHandle_t handle, const uint16_t addr, uint16_t *pValue){
    RESULT result = RET_SUCCESS;
    TRACE(virtualSensor_INFO, "%s (enter)\n", __func__);

    VirtualSensor_Context_t *pVirtualSensorCtx = (VirtualSensor_Context_t*)handle;
    if (pVirtualSensorCtx == NULL)
    	return (RET_WRONG_HANDLE);
    if (pValue == NULL)
		return (RET_NULL_POINTER);


	// mbox cmd
	Payload_packet packet;
	memset(&packet, 0, sizeof(Payload_packet));

	packet.cookie = cookieRpu++;
	packet.type = CMD;
	packet.payload_size = 0;

	uint8_t *p_data = packet.payload_data;
	memcpy(p_data, &pVirtualSensorCtx->instanceId, sizeof(uint32_t));
	p_data += sizeof(uint32_t);
	packet.payload_size += sizeof(uint32_t);
	memcpy(p_data, &addr, sizeof(uint16_t));
	p_data += sizeof(uint16_t);
	packet.payload_size += sizeof(uint16_t)*2;

	result = Send_Command (RPU_2_APU_MB_CMD_IsiReadRegIss, &packet, packet.payload_size + payload_extra_size, dest_cpu_id, pVirtualSensorCtx->rpu_id);
	if(0 != result) {
		return result;
	}

	packet.resp_field.error_subcode_t = rpu_wait_for_mb_data(packet.cookie, packet.payload_data);
	memcpy(pValue, p_data, sizeof(uint16_t));


    TRACE(virtualSensor_INFO, "%s (exit)\n", __func__);
    return (packet.resp_field.error_subcode_t);
}

RESULT virtualSensor_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    RESULT result = RET_SUCCESS;
    static const char SensorName[16] = "virtualSensor";
    TRACE( virtualSensor_INFO, "%s (enter)\n", __func__);

    if ( pIsiSensor != NULL ) {
        pIsiSensor->pszName                             = SensorName;
        pIsiSensor->pIsiCreateIss                       = virtualSensor_IsiCreateIss;
        pIsiSensor->pIsiOpenIss                         = virtualSensor_IsiOpenIss;
        pIsiSensor->pIsiCloseIss                        = virtualSensor_IsiCloseIss;
        pIsiSensor->pIsiReleaseIss                      = virtualSensor_IsiReleaseIss;
        pIsiSensor->pIsiReadRegIss                      = virtualSensor_IsiReadRegIss;
        pIsiSensor->pIsiWriteRegIss                     = virtualSensor_IsiWriteRegIss;
        pIsiSensor->pIsiGetModeIss                      = virtualSensor_IsiGetModeIss;
        pIsiSensor->pIsiEnumModeIss                     = virtualSensor_IsiEnumModeIss;
        pIsiSensor->pIsiGetCapsIss                      = virtualSensor_IsiGetCapsIss;
        pIsiSensor->pIsiCheckConnectionIss              = virtualSensor_IsiCheckConnectionIss;
        pIsiSensor->pIsiGetRevisionIss                  = virtualSensor_IsiGetRevisionIss;
        pIsiSensor->pIsiSetStreamingIss                 = virtualSensor_IsiSetStreamingIss;

        /* AEC */
        pIsiSensor->pIsiGetAeBaseInfoIss                = virtualSensor_pIsiGetAeBaseInfoIss;
        pIsiSensor->pIsiGetAGainIss                     = virtualSensor_IsiGetAGainIss;
        pIsiSensor->pIsiSetAGainIss                     = virtualSensor_IsiSetAGainIss;
        pIsiSensor->pIsiGetDGainIss                     = virtualSensor_IsiGetDGainIss;
        pIsiSensor->pIsiSetDGainIss                     = virtualSensor_IsiSetDGainIss;
        pIsiSensor->pIsiGetIntTimeIss                   = virtualSensor_IsiGetIntTimeIss;
        pIsiSensor->pIsiSetIntTimeIss                   = virtualSensor_IsiSetIntTimeIss;
        pIsiSensor->pIsiGetFpsIss                       = virtualSensor_IsiGetFpsIss;
        pIsiSensor->pIsiSetFpsIss                       = virtualSensor_IsiSetFpsIss;

        /* SENSOR ISP */
        pIsiSensor->pIsiGetIspStatusIss                 = virtualSensor_IsiGetIspStatusIss;
        pIsiSensor->pIsiSetWBIss                        = virtualSensor_IsiSetWBIss;
        pIsiSensor->pIsiGetWBIss                        = virtualSensor_IsiGetWBIss;
        pIsiSensor->pIsiSetBlcIss                       = virtualSensor_IsiSetBlcIss;
        pIsiSensor->pIsiGetBlcIss                       = virtualSensor_IsiGetBlcIss;

        /* SENSOE OTHER FUNC*/
        pIsiSensor->pIsiSetTpgIss                       = virtualSensor_IsiSetTpgIss;
        pIsiSensor->pIsiGetTpgIss                       = virtualSensor_IsiGetTpgIss;
        pIsiSensor->pIsiGetExpandCurveIss               = virtualSensor_IsiGetExpandCurveIss;

        /* AF */
        pIsiSensor->pIsiFocusCreateIss                  = NULL;
        pIsiSensor->pIsiFocusReleaseIss                 = NULL;
        pIsiSensor->pIsiFocusGetCalibrateIss            = NULL;
        pIsiSensor->pIsiFocusSetIss                     = NULL;
        pIsiSensor->pIsiFocusGetIss                     = NULL;

        pIsiSensor->pIsiSetIRLightExpIss                = NULL;
        pIsiSensor->pIsiGetIRLightExpIss                = NULL;
    } else {
        result = RET_NULL_POINTER;
    }

    TRACE( virtualSensor_INFO, "%s (exit)\n", __func__);
    return ( result );
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t virtualSensor_IsiCamDrvConfig = {
    .cameraDriverID      = 0x001,
    .pIsiGetSensorIss    = virtualSensor_IsiGetSensorIss,
};
