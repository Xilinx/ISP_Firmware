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
 ******************************************************************************/

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
#include "amp.h"


extern unsigned int *I2C_INIT_STATUS_REG;

#define mux_to_s4

extern desInterface des_arr[];
Sensor_device Sensor_dev[MAX_SENSOR_COUNT];
struct accessIIC     *accessiic_handle[MAX_SENSOR_COUNT];

extern struct sensor_driver ox08b40_driver;
extern struct sensor_driver ox3f10_driver;

struct sensor_driver *sensor_handle[MAX_SENSOR_COUNT];

/*******************************************************************************
 *          InitIIC
 *
 * @brief   Initialize I2C driver interface
 *
 * @return  None
 *
 *****************************************************************************/
void InitIIC(void)
{
	int Status;
	/*
	 * i2c_driver_init can be moved to sequence which is part of  Board init
	 */
	if (!(get_i2cdriver_status())) {
		Status = i2c_driver_init();
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
	}
}

/*******************************************************************************
 *          init_des
 *
 * @brief   Initialize deserializer interface for the specified index
 *
 * @param   des_index     Index of the deserializer to initialize
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Initialization successful
 * @retval  XST_FAILURE   Initialization failed
 *
 *****************************************************************************/
int  init_des(int des_index)
{
	/* FMC setup */

	int Status = g_fmc_single.pIsiIsiFmcSetup(des_index);

	if (Status != XST_SUCCESS)
		return Status;

	*I2C_INIT_STATUS_REG = 1;

	g_fmc_single.pIsiDeserSetup(&des_arr[des_index]);

#if 0
	/*
	 * Count is repetition for what we are doing with
	 * i2cdriver_init_Status
	 */


	/*
	 * i2cdriver_init_Status : Even though we have a shared variable (I2C_INIT_STATUS_REG)
	 * across the core which tells us about	i2c core Init status.
	 * But we still require this .
	 * If we only use I2C_INIT_STATUS_REG,then in case of multi-core i2c structure for second
	 * core wont get init.
	 * So even in case of multi-core ,role exist for this variable ,although it is split.
	 */

	/*
	 * First time when InitIIC is called on any core
	 * Core 0   : It init   the I2c hardware and create required Driver stuctures
	 * Core1-n  : It doesn't init the I2c(prohibited by I2C_INIT_STATUS_REG ,set by core -0)
	 * and create required Driver stuctures
	 * Second time when this function is called again
	 * core 0-n : Function just return as nothing is required.
	 *
	 * Question can be why we can't we move i2c init at init.
	 * But idea here is init i2c only if required.
	 */

	int Status = g_fmc_single.pIsiIsiFmcSetup(des_index);

	if (Status != XST_SUCCESS)
		return Status;

	/* DES setup */
	g_fmc_single.pIsiDeserSetup(&des_arr[des_index]);
#endif
	return XST_SUCCESS;
}
/*
 * Here we can come from 4 stages
 * 1) Without init
 * 2) after init
 * 3) after stop
 * 4) Already running
 */

/*******************************************************************************
 *          start_sensor
 *
 * @brief   Start streaming on the specified sensor pipeline
 *
 * @param   in_pipe       Input pipeline index to start streaming
 *
 * @return  None
 *
 *****************************************************************************/
void start_sensor(int in_pipe)
{
	xil_printf("enter start_sensor \r\n");
	if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_deinit) {

		xil_printf("Sensor %s on Pipeline-0x%x ,cannot stream as it is not initialized\r\n",
				(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_running) {

		xil_printf("Sensor %s on Pipeline-0x%x ,Already streaming \r\n",
				(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else {
		/*
		 * 1) We reach here after init time,
		 * 2) We reach here after previous stop
		 */
		(g_fmc_single.sensor_array[in_pipe])->stream_on(g_fmc_single.sensor_array[in_pipe]);
		(g_fmc_single.sensor_array[in_pipe])->sensor_state = in_running;

		xil_printf("Streaming Started on Sensor %s on Pipeline-0x%x...\r\n",
				(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	}
}

/*******************************************************************************
 *          access_iic_read
 *
 * @brief   Read data from I2C slave device register
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   slave_addr    Slave device address
 * @param   addr          Register address to read from
 * @param   regWidth      Register address width in bytes
 * @param   pValue        Pointer to store read value
 * @param   dataWidth     Data width in bytes
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Read operation successful
 * @retval  XST_FAILURE   Read operation failed
 *
 *****************************************************************************/
int access_iic_read(u8 i2cBusId, u8 slave_addr, uint16_t addr, uint8_t regWidth, uint16_t *pValue,
			uint8_t dataWidth)
{
	HalReadI2CReg(i2cBusId, slave_addr, addr, regWidth, pValue, dataWidth);
}

/*******************************************************************************
 *          access_iic_write
 *
 * @brief   Write data to I2C slave device register
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   slave_addr    Slave device address
 * @param   addr          Register address to write to
 * @param   regWidth      Register address width in bytes
 * @param   value         Value to write
 * @param   dataWidth     Data width in bytes
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Write operation successful
 * @retval  XST_FAILURE   Write operation failed
 *
 *****************************************************************************/
int access_iic_write(u8 i2cBusId, u8 slave_addr, uint16_t addr, uint8_t regWidth, uint16_t value,
			uint8_t dataWidth)
{
	HalWriteI2CReg(i2cBusId, slave_addr, addr, regWidth, value, dataWidth);
}


/*******************************************************************************
 *          init_iic_access
 *
 * @brief   Initialize I2C access interface for the specified pipeline
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   in_pipe       Input pipeline index
 *
 * @return  None
 *
 *****************************************************************************/
void init_iic_access(u8 i2cBusId, int in_pipe)
{
	xil_printf("enter init_iic_access \r\n");
	accessiic_handle[in_pipe] = (struct accessIIC *)osMalloc(sizeof(struct accessIIC));
	g_fmc_single.accessiic_array[in_pipe] = accessiic_handle[in_pipe];

	g_fmc_single.accessiic_array[in_pipe]->i2cBusId = i2cBusId;
	g_fmc_single.accessiic_array[in_pipe]->readIIC  = access_iic_read;
	g_fmc_single.accessiic_array[in_pipe]->writeIIC = access_iic_write;

}

/*******************************************************************************
 *          init_sensor
 *
 * @brief   Initialize sensor driver for the specified pipeline and deserializer
 *
 * @param   in_pipe       Input pipeline index
 * @param   des_index     Deserializer index
 *
 * @return  None
 *
 *****************************************************************************/
void init_sensor(int in_pipe, int des_index)
{
	xil_printf("enter init_sensor \r\n");
	/*
	 * Select the type of sensor,update that structure
	 * Todo :
	 * In case if multiple entry to this function
	 * Check the driver state ,load if in de-init state
	 */

	if (Sensor_dev[in_pipe].Sensortype == SENSOR_3MP) {
		sensor_handle[in_pipe] = (struct sensor_driver *)osMalloc
			(sizeof(struct sensor_driver));

		g_fmc_single.sensor_array[in_pipe] = sensor_handle[in_pipe];
		g_fmc_single.sensor_array[in_pipe]->pipe_no = in_pipe;

	} else if (Sensor_dev[in_pipe].Sensortype == SENSOR_5MP) {
		sensor_handle[in_pipe] = (struct sensor_driver *)osMalloc
			(sizeof(struct sensor_driver));

		g_fmc_single.sensor_array[in_pipe] = sensor_handle[in_pipe];
		g_fmc_single.sensor_array[in_pipe]->pipe_no = in_pipe;
	} else if (Sensor_dev[in_pipe].Sensortype == SENSOR_8MP) {
		sensor_handle[in_pipe] = (struct sensor_driver *)osMalloc
			(sizeof(struct sensor_driver));

		g_fmc_single.sensor_array[in_pipe] = sensor_handle[in_pipe];
		g_fmc_single.sensor_array[in_pipe]->pipe_no = in_pipe;

	} else {
		xil_printf("Not Supported Sensor \r\n");
		DCT_ASSERT(0);
	}

	if (des_arr[des_index].link_type == NO_LINK) {
		xil_printf("Sensor Not Connected ,please connect the sensor\r\n");

	} else if (des_arr[des_index].link_type == LINK_A) {
		xil_printf("Programming Link-A Device \r\n", __func__, __LINE__);
		g_fmc_single.serializer_array[in_pipe]->alias_addr =
			(des_arr[des_index].link_a.serializer_alias_addr);
		(g_fmc_single.serializer_array[in_pipe])->init_serializer(
				g_fmc_single.serializer_array[in_pipe]);

		g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
			(des_arr[des_index].link_a.sensor_alias_addr);

	} else if (des_arr[des_index].link_type == LINK_B) {
		xil_printf("Programming Link-B Device \r\n", __func__, __LINE__);
		g_fmc_single.serializer_array[in_pipe]->alias_addr =
			(des_arr[des_index].link_b.serializer_alias_addr);
		(g_fmc_single.serializer_array[in_pipe])->init_serializer(
				g_fmc_single.serializer_array[in_pipe]);

		g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
			(des_arr[des_index].link_b.sensor_alias_addr);

	} else if (des_arr[des_index].link_type == LINK_REVERSE_SPLITTER) {

		if ((in_pipe % 2) == 0) {
			xil_printf("Programming Link-A Device \r\n", __func__, __LINE__);
			g_fmc_single.serializer_array[in_pipe]->alias_addr =
				(des_arr[des_index].link_a.serializer_alias_addr);
			(g_fmc_single.serializer_array[in_pipe])->init_serializer(
					g_fmc_single.serializer_array[in_pipe]);

			g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
				(des_arr[des_index].link_a.sensor_alias_addr);

		} else {
			xil_printf("Programming Link-B Device \r\n", __func__, __LINE__);
			g_fmc_single.serializer_array[in_pipe]->alias_addr =
				(des_arr[des_index].link_b.serializer_alias_addr);
			(g_fmc_single.serializer_array[in_pipe])->init_serializer(
					g_fmc_single.serializer_array[in_pipe]);

			g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr =
				(des_arr[des_index].link_b.sensor_alias_addr);
		}
	}
}

/*******************************************************************************
 *          stop_sensor
 *
 * @brief   Stop streaming on the specified sensor pipeline
 *
 * @param   in_pipe       Input pipeline index to stop streaming
 *
 * @return  None
 *
 *****************************************************************************/
void stop_sensor(int in_pipe)
{
	xil_printf("Skipping stop sensor\n");
	return;
	xil_printf("enter stop_sensor \r\n");
	if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_deinit) {
		g_fmc_single.serializer_array[in_pipe]->ser_state = in_deinit;
		(g_fmc_single.serializer_array[in_pipe])->deinit_serializer(
				g_fmc_single.serializer_array[in_pipe]);

		xil_printf("Sensor %s on Pipeline-0x%x ,cannot stop as it is not initialized\r\n",
				(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

		des_arr[in_pipe/2].des_state = in_deinit;
	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_init) {
		g_fmc_single.serializer_array[in_pipe]->ser_state = in_deinit;

		des_arr[in_pipe/2].des_state = in_deinit;

		(g_fmc_single.serializer_array[in_pipe])->deinit_serializer(
				g_fmc_single.serializer_array[in_pipe]);
		xil_printf("Sensor %s on Pipeline-0x%x ,cannot stop as it is not running\r\n",
				(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_stop) {
		g_fmc_single.serializer_array[in_pipe]->ser_state = in_deinit;
		des_arr[in_pipe/2].des_state = in_deinit;
		(g_fmc_single.serializer_array[in_pipe])->deinit_serializer(
				g_fmc_single.serializer_array[in_pipe]);
		xil_printf("Sensor %s on Pipeline-0x%x ,Already in stop state \r\n",
				(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else {
		(g_fmc_single.sensor_array[in_pipe])->stream_off(
				g_fmc_single.sensor_array[in_pipe]);
		(g_fmc_single.sensor_array[in_pipe])->sensor_state = in_stop;

		g_fmc_single.serializer_array[in_pipe]->ser_state = in_deinit;
		des_arr[in_pipe/2].des_state = in_deinit;

		(g_fmc_single.serializer_array[in_pipe])->deinit_serializer(
				g_fmc_single.serializer_array[in_pipe]);
		xil_printf("Streaming off on Sensor %s on Pipeline-0x%x \r\n",
				(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);
	}
}

/*******************************************************************************
 *          sensor_status
 *
 * @brief   Display status information for the specified sensor
 *
 * @param   index         Sensor index to display status for
 *
 * @return  None
 *
 *****************************************************************************/
void sensor_status(int index)
{
	if (((g_fmc_single.sensor_array[index])->sensor_state == in_stop) ||
			(g_fmc_single.sensor_array[index])->sensor_state == in_running) {

	} else {
		// nothing to do.
	}
}

/*******************************************************************************
 *          des_stats
 *
 * @brief   Display deserializer statistics for the specified index
 *
 * @param   i             Deserializer index to display statistics for
 *
 * @return  None
 *
 *****************************************************************************/
void des_stats(int i)
{
	u8 des_addr;
	int pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	int Status;

	if (des_arr[i].des_state == in_running) {
		des_addr = (des_arr[i].des_alias_addr) >> 1;

		Status = HalReadI2CReg(0, des_addr, 0x108, 2, &pipe_x_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		Status = HalReadI2CReg(0, des_addr, 0x11a, 2, &pipe_y_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		Status = HalReadI2CReg(0, des_addr, 0x12c, 2, &pipe_z_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		Status = HalReadI2CReg(0, des_addr, 0x13e, 2, &pipe_u_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		xil_printf("Des[%x] Pipe Status : pipe_x = 0x%x, pipe_y = 0x%x, pipe_z = 0x%x, pipe_u = 0x%x\r\n",
				des_arr[i].Port_DES_index + 1, pipe_x_s, pipe_y_s, pipe_z_s, pipe_u_s);
	}
}

int i2c_driver_init(void);
/*******************************************************************************
 *          ser_stats
 *
 * @brief   Display serializer statistics for the specified index
 *
 * @param   i             Serializer index to display statistics for
 *
 * @return  None
 *
 *****************************************************************************/
void ser_stats(int i)
{
	u8 ser_addr;
	int pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	int Status;

	if (g_fmc_single.serializer_array[i]->ser_state == in_running) {
		ser_addr = (g_fmc_single.serializer_array[i]->alias_addr) >> 1;

		Status = HalReadI2CReg(0, ser_addr, 0x102, 2, &pipe_x_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		Status = HalReadI2CReg(0, ser_addr, 0x10a, 2, &pipe_y_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		Status = HalReadI2CReg(0, ser_addr, 0x112, 2, &pipe_z_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		Status = HalReadI2CReg(0, ser_addr, 0x11a, 2, &pipe_u_s, 1);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		if (Status != XST_SUCCESS)
			DCT_ASSERT(0);
		xil_printf("middha - ser[%x]: Px:0x%x, Py: 0x%x, Pz: 0x%x, Pu: 0x%x\n", i + 1, pipe_x_s, pipe_y_s, pipe_z_s, pipe_u_s);
	}
}

/*******************************************************************************
 *          Fmc_Sensor_Statustask
 *
 * @brief   Display status for all deserializers and serializers in the system
 *
 * @return  None
 *
 *****************************************************************************/
void Fmc_Sensor_Statustask(void)
{
	int i;

	for (i = 0; i < DS_MAX; i++)
		des_stats(i);
	for (i = 0; i < IN_PIPE_LAST; i++)
		ser_stats(i);
}

u32 i2cdriver_init_Status;

/*******************************************************************************
 *          get_i2cdriver_status
 *
 * @brief   Get the current I2C driver initialization status
 *
 * @return  Return the initialization status.
 * @retval  0             I2C driver not initialized
 * @retval  1             I2C driver initialized
 *
 *****************************************************************************/
int get_i2cdriver_status(void)
{
	if (i2cdriver_init_Status == 0)
		return 0;
	else
		return 1;
}

/*******************************************************************************
 *          i2c_driver_init
 *
 * @brief   Initialize I2C driver and set up shared memory for multi-core systems
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   I2C driver initialization successful
 * @retval  XST_FAILURE   I2C driver initialization failed
 *
 *****************************************************************************/
int i2c_driver_init(void)
{
	int Status = XST_SUCCESS;
	const TickType_t xDelay2 = 5000;

	if (Status != XST_SUCCESS) {
		xil_printf("\n\rIIC Init Failed \r\n");
		return XST_FAILURE;
	} else
		i2cdriver_init_Status = 1;
	xil_printf("\n\r %s done... \r\n", __func__);

	return XST_SUCCESS;
}

/*******************************************************************************
 *          sensor_stop
 *
 * @brief   Stop sensor streaming on the specified pipeline
 *
 * @param   pipe          Pipeline index to stop
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Stop operation successful
 *
 *****************************************************************************/
int sensor_stop(int pipe)
{
	stop_sensor(pipe);
	return XST_SUCCESS;
}
