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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "isi/isi_fmc.h"
#include "ebase/dct_assert.h"
#include "isi/isi.h"
#include "sensor_drv/ox03f10_priv.h"

#if defined(AMP_COMMON_I2C_CONTROLLER) || defined(AMP_MULTIPLE_I2C_CONTROLLER)
#include "spinlock_address_map.h"
extern u32 *I2C_INIT_STATUS_REG;
#endif
#define mux_to_s4

u32 i2cdriver_init_Status;
extern desInterface des_arr[];
Sensor_device Sensor_dev[MAX_SENSOR_COUNT];
struct sensor_driver *sensor_handle[MAX_SENSOR_COUNT];
struct accessIIC *accessiic_handle[MAX_SENSOR_COUNT];
static int get_i2cdriver_status(void);

void InitIIC(void)
{
	int Status = XST_SUCCESS;

	if (!(get_i2cdriver_status())) {
		Status = i2c_driver_init();
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
	}
}

int init_des(int des_index)
{
#if !defined(AMP_COMMON_I2C_CONTROLLER)
	static count;

	if (count == 0)
		InitIIC();

	g_fmc_single.pIsiIsiFmcSetup(des_index);
	count++;
	g_fmc_single.pIsiDeserSetup(&des_arr[des_index]);
#else
	InitIIC();
	g_fmc_single.pIsiIsiFmcSetup(des_index);
	g_fmc_single.pIsiDeserSetup(&des_arr[des_index]);
#endif
	return XST_SUCCESS;
}

void start_sensor(int in_pipe)
{
	xil_printf("sensor: enter %s\n", __func__);

	if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_deinit) {
		xil_printf("Sensor %s on Pipeline-0x%x ,cannot stream as it is not initialized\n",
			(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_running) {
		xil_printf("Sensor %s on Pipeline-0x%x ,Already streaming\n",
			(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	} else {
		(g_fmc_single.sensor_array[in_pipe])->stream_on(g_fmc_single.sensor_array[in_pipe]);
		(g_fmc_single.sensor_array[in_pipe])->sensor_state = in_running;

		xil_printf("Streaming Started on Sensor %s on Pipeline-0x%x.\n",
			(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	}
}

int access_iic_read(uint16_t busId, uint16_t addr, uint16_t *pValue)
{
	u8 chipAddress = (g_fmc_single.sensor_array[busId]->sensor_alias_addr) >> 1;

	HalXilReadI2CReg(IIC_INSTANCE_ZERO, chipAddress, addr, 0x2, pValue, 1);
}

int access_iic_write(uint16_t busId, uint16_t addr, uint16_t value)
{
	u8 chipAddress = (g_fmc_single.sensor_array[busId]->sensor_alias_addr) >> 1;

	HalXilWriteI2CReg(IIC_INSTANCE_ZERO, chipAddress, addr, 0x2, value, 1);
}

void init_iic_access(int in_pipe, int des_index)
{
	xil_printf("enter %s Malloc added\n", __func__);

	accessiic_handle[in_pipe] = (struct accessIIC *)osMalloc(sizeof(struct accessIIC));
	g_fmc_single.iic_array[in_pipe] = accessiic_handle[in_pipe];

	xil_printf("g_fmc_single.iic_array[in_pipe]->readIIC 0x%x, 0x%x\n",
		g_fmc_single.iic_array[in_pipe]->readIIC,
		&g_fmc_single.iic_array[in_pipe]->readIIC);
	g_fmc_single.iic_array[in_pipe]->readIIC = access_iic_read;

	xil_printf("g_fmc_single.iic_array[in_pipe]->writeIIC 0x%x, 0x%x\n",
		g_fmc_single.iic_array[in_pipe]->writeIIC,
		&g_fmc_single.iic_array[in_pipe]->writeIIC);
	g_fmc_single.iic_array[in_pipe]->writeIIC = access_iic_write;
}

void init_sensor(int in_pipe, int des_index)
{
	xil_printf("sensor: enter %s\n", __func__);

	if (Sensor_dev[in_pipe].Sensortype == SENSOR_3MP) {
		xil_printf("3MP Sensor Selected\n");
		sensor_handle[in_pipe] =
			(struct sensor_driver *)osMalloc(sizeof(struct sensor_driver));
		g_fmc_single.sensor_array[in_pipe] = sensor_handle[in_pipe];
		g_fmc_single.sensor_array[in_pipe]->pipe_no = in_pipe;
	} else if (Sensor_dev[in_pipe].Sensortype == SENSOR_5MP) {
		xil_printf("5MP Sensor Selected\n");
		sensor_handle[in_pipe] =
			(struct sensor_driver *)osMalloc(sizeof(struct sensor_driver));
		g_fmc_single.sensor_array[in_pipe] = sensor_handle[in_pipe];
		g_fmc_single.sensor_array[in_pipe]->pipe_no = in_pipe;
	} else if (Sensor_dev[in_pipe].Sensortype == SENSOR_8MP) {
		xil_printf("8MP Sensor Selected\n");
		sensor_handle[in_pipe] =
			(struct sensor_driver *)osMalloc(sizeof(struct sensor_driver));
		g_fmc_single.sensor_array[in_pipe] = sensor_handle[in_pipe];
		g_fmc_single.sensor_array[in_pipe]->pipe_no = in_pipe;
	} else {
		xil_printf("Not Supported Sensor\n");
		DCT_ASSERT(0);
	}

	if (des_arr[des_index].link_type == NO_LINK) {
		xil_printf("Sensor Not Connected ,please connect the sensor\n");
	} else if (des_arr[des_index].link_type == LINK_A) {
		xil_printf("Programming Link-A Device\n", __func__, __LINE__);
		g_fmc_single.serializer_array[in_pipe]->alias_addr =
				(des_arr[des_index].link_a.serializer_alias_addr);
		(g_fmc_single.serializer_array[in_pipe])->
				init_serializer(g_fmc_single.serializer_array[in_pipe]);
		g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
				(des_arr[des_index].link_a.sensor_alias_addr);
	} else if (des_arr[des_index].link_type == LINK_B) {
		xil_printf("Programming Link-B Device\n", __func__, __LINE__);
		g_fmc_single.serializer_array[in_pipe]->alias_addr =
				(des_arr[des_index].link_b.serializer_alias_addr);
		(g_fmc_single.serializer_array[in_pipe])->
				init_serializer(g_fmc_single.serializer_array[in_pipe]);
		g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
				(des_arr[des_index].link_b.sensor_alias_addr);
	} else if (des_arr[des_index].link_type == LINK_REVERSE_SPLITTER) {
		if ((in_pipe % 2) == 0) {
			xil_printf("Programming Link-A Device\n", __func__, __LINE__);
			g_fmc_single.serializer_array[in_pipe]->alias_addr =
					(des_arr[des_index].link_a.serializer_alias_addr);
			(g_fmc_single.serializer_array[in_pipe])->
					init_serializer(g_fmc_single.serializer_array[in_pipe]);
			g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
					(des_arr[des_index].link_a.sensor_alias_addr);
		} else {
			xil_printf("Programming Link-B Device\n", __func__, __LINE__);
			g_fmc_single.serializer_array[in_pipe]->alias_addr =
				(des_arr[des_index].link_b.serializer_alias_addr);
			(g_fmc_single.serializer_array[in_pipe])->
					init_serializer(g_fmc_single.serializer_array[in_pipe]);
			g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
					(des_arr[des_index].link_b.sensor_alias_addr);
		}
	}
}

void stop_sensor(int in_pipe)
{
	xil_printf("sensor: enter %s\n", __func__);

	if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_deinit) {
		xil_printf("Sensor %s on Pipeline-0x%x ,cannot stop as it is not initialized\n",
			(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_init) {
		xil_printf("Sensor %s on Pipeline-0x%x ,cannot stop as it is not running\n",
			(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_stop) {
		xil_printf("Sensor %s on Pipeline-0x%x ,Already in stop state\n",
			(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	} else {
		(g_fmc_single.sensor_array[in_pipe])->
			stream_off(g_fmc_single.sensor_array[in_pipe]);
		(g_fmc_single.sensor_array[in_pipe])->sensor_state = in_stop;

		xil_printf("Streaming off on Sensor %s on Pipeline-0x%x\n",
			(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	}
}

void des_stats(int i)
{
	int Status = XST_SUCCESS;
	u8 des_addr;
	int pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;

	if (des_arr[i].des_state == in_running) {
		des_addr = (des_arr[i].des_alias_addr) >> 1;

		Status = HalXilReadI2CReg(0, des_addr, 0x108, 2, &pipe_x_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		Status = HalXilReadI2CReg(0, des_addr, 0x11a, 2, &pipe_y_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		Status = HalXilReadI2CReg(0, des_addr, 0x12c, 2, &pipe_z_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		Status = HalXilReadI2CReg(0, des_addr, 0x13e, 2, &pipe_u_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		xil_printf("Des[%x]: pipe_x = 0x%x, pipe_y = 0x%x, pipe_z = 0x%x, pipe_u = 0x%x\n",
			des_arr[i].Port_DES_index + 1, pipe_x_s, pipe_y_s, pipe_z_s, pipe_u_s);
	}
}

void ser_stats(int i)
{
	int Status = XST_SUCCESS;
	u8 ser_addr;
	int pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;

	if (g_fmc_single.serializer_array[i]->ser_state == in_running) {
		ser_addr = (g_fmc_single.serializer_array[i]->alias_addr) >> 1;

		Status = HalXilReadI2CReg(0, ser_addr, 0x102, 2, &pipe_x_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		Status = HalXilReadI2CReg(0, ser_addr, 0x10a, 2, &pipe_y_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		Status = HalXilReadI2CReg(0, ser_addr, 0x112, 2, &pipe_z_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);

		Status = HalXilReadI2CReg(0, ser_addr, 0x11a, 2, &pipe_u_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
	}
}

void Fmc_Sensor_Statustask(void)
{
	int i;

	for (i = 0; i < DS_MAX; i++)
		des_stats(i);

	for (i = 0; i < IN_PIPE_LAST; i++)
		ser_stats(i);
}

static int get_i2cdriver_status(void)
{
	if (i2cdriver_init_Status == 0)
		return 0;
	else
		return 1;
}

int i2c_driver_init(void)
{
	int Status = XST_SUCCESS;
	const TickType_t xDelay2 = 5000;

	xil_printf("i2c: %s start\n", __func__);

	Status = init_MUX();
	if (Status != XST_SUCCESS) {
		xil_printf("IIC Init Failed\n");
		return XST_FAILURE;
	} else {
#if defined(AMP_COMMON_I2C_CONTROLLER)
		u32 *ptr = (u32 *)(I2C_INIT_STATUS_REG);

		if (get_cpu_id() == CORE_0)
			*ptr = SH_VAR_INIT;
#endif
		i2cdriver_init_Status = 1;
	}

	xil_printf("i2c: %s done\n", __func__);

	return Status;
}

static int sensor_stop(int pipe)
{
	stop_sensor(pipe);
	return XST_SUCCESS;
}
