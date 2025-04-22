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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "isi/isi_fmc.h"

#include "ebase/dct_assert.h"

#include "isi/isi.h"
#include "sensor_drv/ox03f10_priv.h"

//#include "cpu_info.h"
#if defined (AMP_COMMON_I2C_CONTROLLER) || defined (AMP_MULTIPLE_I2C_CONTROLLER)
#include "spinlock_address_map.h"
extern u32 *I2C_INIT_STATUS_REG;
#endif

#define mux_to_s4
//#define s1_mux_to_s0

extern desInterface des_arr[];
Sensor_device Sensor_dev[MAX_SENSOR_COUNT];
extern struct sensor_driver ox08b40_driver;
extern struct sensor_driver ox3f10_driver;

struct sensor_driver *sensor_handle[MAX_SENSOR_COUNT];
struct accessIIC     *accessiic_handle[MAX_SENSOR_COUNT];

void InitIIC(void){
	int Status;
	/*
	 * i2c_driver_init can be moved to sequence which is part of  Board init
	 */
	if(!(get_i2cdriver_status())){
		Status=i2c_driver_init();
		 if (Status != XST_SUCCESS) {
			 DCT_ASSERT(0);
				 }
		}
}

int  init_des(int des_index)
{
	//FMC setup
#if !defined (AMP_COMMON_I2C_CONTROLLER)
	static count = 0;
	if(count == 0){
		InitIIC();
	}
	g_fmc_single.pIsiIsiFmcSetup(des_index);
	count++;
	//DES setup
	g_fmc_single.pIsiDeserSetup(&des_arr[des_index]);
#else

	/*
	 * Count is repetition for what we are doing with
	 * i2cdriver_init_Status
	 */


	/*
	 * i2cdriver_init_Status : Even though we have a shared variable (I2C_INIT_STATUS_REG)across the core
	 * 						   which tells us about	i2c core Init status.
	 * 						   But we still require this .
	 * 						   If we only use I2C_INIT_STATUS_REG,then in case of multi-core i2c structure for second
	 * 						   core wont get init.
	 * 						   So even in case of multi-core ,role exist for this variable ,although it is split.
	 *
	 */
	/*
	 * First time when InitIIC is called on any core
	 * Core 0   : It init   the I2c hardware and and create required Driver stuctures
	 * Core1-n  : It doesn't init the I2c(prohibited by I2C_INIT_STATUS_REG ,set by core -0)
	 * 			  and and create required Driver stuctures
	 * Second time when this function is called again
	 * core 0-n : Function just return as nothing is required.
	 *
	 * Question can be why we can't we move i2c init at init.
	 * But idea here is init i2c only if required.
	 */

	InitIIC();
	g_fmc_single.pIsiIsiFmcSetup(des_index);

	//DES setup
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
void start_sensor(int in_pipe)
{
	xil_printf("enter start_sensor \r\n");
	if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_deinit) {

		xil_printf ("Sensor %s on Pipeline-0x%x ,cannot stream as it is not initialized\r\n",(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_running) {

		xil_printf ("Sensor %s on Pipeline-0x%x ,Already streaming \r\n",(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else {
		/*
		 * 1) We reach here after init time,
		 * 2) We reach here after previous stop
		 */
		(g_fmc_single.sensor_array[in_pipe])->stream_on(g_fmc_single.sensor_array[in_pipe]);
		(g_fmc_single.sensor_array[in_pipe])->sensor_state = in_running;

		xil_printf ("Streaming Started on Sensor %s on Pipeline-0x%x...\r\n",(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	}
}

int access_iic_read(uint16_t busId , uint16_t addr, uint16_t *pValue)
{
	// HalXilReadI2CReg(IIC_DEVICE_ID, pHalCtx->slaveAddr, addr, pHalCtx->regWidth, pValue, pHalCtx->dataWidth);

	// u8 chipAddress = (sensor_inst->sensor_alias_addr) >> 1;
	u8 chipAddress = (g_fmc_single.sensor_array[busId]->sensor_alias_addr) >> 1;

	HalXilReadI2CReg(  IIC_INSTANCE_ZERO /*IIC_DEVICE_ID*/, chipAddress, addr, 0x2, pValue, 1);

}

int access_iic_write(uint16_t busId,  uint16_t addr, uint16_t value)
{
    // HalXilWriteI2CReg(pHalCtx->devId, pHalCtx->slaveAddr, addr, pHalCtx->regWidth, value, pHalCtx->dataWidth);
	u8 chipAddress = (g_fmc_single.sensor_array[busId]->sensor_alias_addr) >> 1;
	HalXilWriteI2CReg(IIC_INSTANCE_ZERO  /*IIC_DEVICE_ID*/, chipAddress,addr, 0x2, value, 1);
}


void init_iic_access(int in_pipe, int des_index)
{
	xil_printf("enter init_iic_access Malloc added \r\n");
    
    accessiic_handle[in_pipe]=(struct accessIIC *)osMalloc(sizeof(struct accessIIC));

    g_fmc_single.accessiic_array[in_pipe] = accessiic_handle[in_pipe];
    
    xil_printf("g_fmc_single.accessiic_array[in_pipe]->readIIC 0x%x, 0x%x \n",g_fmc_single.accessiic_array[in_pipe]->readIIC, &g_fmc_single.accessiic_array[in_pipe]->readIIC );
	g_fmc_single.accessiic_array[in_pipe]->readIIC  = access_iic_read;


    xil_printf("g_fmc_single.accessiic_array[in_pipe]->writeIIC  0x%x, 0x%x \n",g_fmc_single.accessiic_array[in_pipe]->writeIIC, &g_fmc_single.accessiic_array[in_pipe]->writeIIC );
	g_fmc_single.accessiic_array[in_pipe]->writeIIC = access_iic_write;


}

void init_sensor(int in_pipe, int des_index)
{
	xil_printf("enter init_sensor \r\n");
	/*
	 * Select the type of sensor,update that structure
	 * Todo :
	 * In case if multiple entry to this function
	 * Check the driver state ,load if in de-init state
	 */

	if(Sensor_dev[in_pipe].Sensortype==SENSOR_3MP)
	{
		xil_printf("3MP Sensor Selected \r\n");

		sensor_handle[in_pipe]=(struct sensor_driver *)osMalloc(sizeof(struct sensor_driver));
//		memcpy(sensor_handle[in_pipe],&ox3f10_driver,sizeof(struct sensor_driver));  //wait for driver

		g_fmc_single.sensor_array[in_pipe]=sensor_handle[in_pipe];
	    	g_fmc_single.sensor_array[in_pipe]->pipe_no=in_pipe;

	}
	else if(Sensor_dev[in_pipe].Sensortype==SENSOR_5MP)
	{
		xil_printf("5MP Sensor Selected \r\n");

		sensor_handle[in_pipe]=(struct sensor_driver *)osMalloc(sizeof(struct sensor_driver));
//		memcpy(sensor_handle[in_pipe],&ox3f10_driver,sizeof(struct sensor_driver));  //wait for driver

		g_fmc_single.sensor_array[in_pipe]=sensor_handle[in_pipe];
		g_fmc_single.sensor_array[in_pipe]->pipe_no=in_pipe;
	}
	else if (Sensor_dev[in_pipe].Sensortype==SENSOR_8MP)
	{
	
		xil_printf("8MP Sensor Selected \r\n");

		sensor_handle[in_pipe]=(struct sensor_driver *)osMalloc(sizeof(struct sensor_driver));
		// memcpy(sensor_handle[in_pipe],&ox08b40_driver,sizeof(struct sensor_driver));  //wait for driver

		g_fmc_single.sensor_array[in_pipe]= sensor_handle[in_pipe];
	    	g_fmc_single.sensor_array[in_pipe]->pipe_no=in_pipe;
	
	}
	else
	{
		xil_printf("Not Supported Sensor \r\n");
		DCT_ASSERT(0);
	}

	if ((des_arr[des_index].link_type == NO_LINK)) {

		xil_printf ("Sensor Not Connected ,please connect the sensor\r\n");

	}
	else if (des_arr[des_index].link_type == LINK_A) {

		xil_printf ("Programming Link-A Device \r\n", __func__, __LINE__);
		g_fmc_single.serializer_array[in_pipe]->alias_addr = (des_arr[des_index].link_a.serializer_alias_addr);
		(g_fmc_single.serializer_array[in_pipe])->init_serializer(g_fmc_single.serializer_array[in_pipe]);

		//SENSOR setup

		g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr = (des_arr[des_index].link_a.sensor_alias_addr);
		//Move below function to new function called stream_on
		// (g_fmc_single.sensor_array[in_pipe])->init_sensor(g_fmc_single.sensor_array[in_pipe]);


	}
	else if (des_arr[des_index].link_type == LINK_B) {
		//SER setup
		xil_printf ("Programming Link-B Device \r\n", __func__, __LINE__);
		g_fmc_single.serializer_array[in_pipe]->alias_addr = (des_arr[des_index].link_b.serializer_alias_addr);
		(g_fmc_single.serializer_array[in_pipe])->init_serializer(g_fmc_single.serializer_array[in_pipe]);

		//SENSOR setup
		g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr = (des_arr[des_index].link_b.sensor_alias_addr);
//		(g_fmc_single.sensor_array[in_pipe])->init_sensor(g_fmc_single.sensor_array[in_pipe]);

	} else if (des_arr[des_index].link_type == LINK_REVERSE_SPLITTER) {


		if ((in_pipe % 2) == 0) {
			//SER setup
			xil_printf ("Programming Link-A Device \r\n", __func__, __LINE__);
			g_fmc_single.serializer_array[in_pipe]->alias_addr = (des_arr[des_index].link_a.serializer_alias_addr);
			(g_fmc_single.serializer_array[in_pipe])->init_serializer(g_fmc_single.serializer_array[in_pipe]);

			//SENSOR setup
			g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr = (des_arr[des_index].link_a.sensor_alias_addr);
			// (g_fmc_single.sensor_array[in_pipe])->init_sensor(g_fmc_single.sensor_array[in_pipe]);

		} else {

			//SER setup
			xil_printf ("Programming Link-B Device \r\n", __func__, __LINE__);
			g_fmc_single.serializer_array[in_pipe]->alias_addr = (des_arr[des_index].link_b.serializer_alias_addr);
			(g_fmc_single.serializer_array[in_pipe])->init_serializer(g_fmc_single.serializer_array[in_pipe]);

			//SENSOR setup
			g_fmc_single.sensor_array[in_pipe]->sensor_alias_addr = (des_arr[des_index].link_b.sensor_alias_addr);
			// (g_fmc_single.sensor_array[in_pipe])->init_sensor(g_fmc_single.sensor_array[in_pipe]);

		}
	}
}


void stop_sensor(int in_pipe)
{
	xil_printf("enter stop_sensor \r\n");
	if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_deinit) {

		xil_printf ("Sensor %s on Pipeline-0x%x ,cannot stop as it is not initialized\r\n",(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_init) {

		xil_printf ("Sensor %s on Pipeline-0x%x ,cannot stop as it is not running\r\n",(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else if ((g_fmc_single.sensor_array[in_pipe])->sensor_state == in_stop) {

		xil_printf ("Sensor %s on Pipeline-0x%x ,Already in stop state \r\n",(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	} else {
		(g_fmc_single.sensor_array[in_pipe])->stream_off(g_fmc_single.sensor_array[in_pipe]);
		(g_fmc_single.sensor_array[in_pipe])->sensor_state = in_stop;

		xil_printf ("Streaming off on Sensor %s on Pipeline-0x%x \r\n",(g_fmc_single.sensor_array[in_pipe])->name, in_pipe);

	}
}

void sensor_status(int index) {
	if (((g_fmc_single.sensor_array[index])->sensor_state == in_stop) || (g_fmc_single.sensor_array[index])->sensor_state == in_running) {

//		sensor_framecount(g_fmc_single.sensor_array[index]);

	} else {
		// nothing to do.
	}
}

void des_stats(int i) {
	u8 des_addr;
	int pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	int Status;

	if (des_arr[i].des_state == in_running) {
		des_addr = (des_arr[i].des_alias_addr) >> 1;

		Status = HalXilReadI2CReg(0/*bus*/, des_addr/*Deser*/, 0x108/*reg addr*/, 2/*reg size*/, &pipe_x_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		Status = HalXilReadI2CReg(0/*bus*/, des_addr/*Deser*/, 0x11a/*reg addr*/, 2/*reg size*/, &pipe_y_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		Status = HalXilReadI2CReg(0/*bus*/, des_addr/*Deser*/, 0x12c/*reg addr*/, 2/*reg size*/, &pipe_z_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		Status = HalXilReadI2CReg(0/*bus*/, des_addr/*Deser*/, 0x13e/*reg addr*/, 2/*reg size*/, &pipe_u_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		xil_printf("Desearlizer[%x] Pipe Status : pipe_x = 0x%x, pipe_y = 0x%x, pipe_z = 0x%x, pipe_u = 0x%x\r\n", des_arr[i].Port_DES_index + 1, pipe_x_s, pipe_y_s, pipe_z_s, pipe_u_s);
	}
}

int i2c_driver_init(void);
#if 0
void Fmc_Sensor_Statustask(void *pvParameters) {
	int i,Status;
	const TickType_t xDelay2 = 5000;

	do {
		for (i = 0; i < SENSOR_MAX; i++) {
			sensor_status(i);
		}

		for (i = 0; i < DS_MAX; i++) {
			des_stats(i);
		}

		vTaskDelay(xDelay2);
	} while (1);
}
#endif
//DEBUG _ADDED

void ser_stats(int i) {
	u8 ser_addr;
	int pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	int Status;

	if (g_fmc_single.serializer_array[i]->ser_state == in_running) {
		ser_addr = (g_fmc_single.serializer_array[i]->alias_addr) >> 1;

		Status = HalXilReadI2CReg(0/*bus*/, ser_addr/*ser*/, 0x102/*reg addr*/, 2/*reg size*/, &pipe_x_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		Status = HalXilReadI2CReg(0/*bus*/, ser_addr/*ser*/, 0x10a/*reg addr*/, 2/*reg size*/, &pipe_y_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		Status = HalXilReadI2CReg(0/*bus*/, ser_addr/*ser*/, 0x112/*reg addr*/, 2/*reg size*/, &pipe_z_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		Status = HalXilReadI2CReg(0/*bus*/, ser_addr/*ser*/, 0x11a/*reg addr*/, 2/*reg size*/, &pipe_u_s, 1/*datasize*/);
		if (Status != XST_SUCCESS) {
			DCT_ASSERT(0);
		}

		//xil_printf("middha - ser[%x]: Px:0x%x, Py: 0x%x, Pz: 0x%x, Pu: 0x%x\n", i + 1, pipe_x_s, pipe_y_s, pipe_z_s, pipe_u_s);
	}
}

void Fmc_Sensor_Statustask() {
	int i;
//    int Status;
//	const TickType_t xDelay2 = 5000;

//	do {
/*		for (i = 0; i < SENSOR_MAX; i++) {
			sensor_status(i);
		}
*/
//    xil_printf("middha - before des_stats\n");
		for (i = 0; i < DS_MAX; i++) {
			des_stats(i);
		}
		for (i = 0; i < IN_PIPE_LAST; i++) {
			ser_stats(i);
		}
//    xil_printf("middha - after des_stats\n");

//		vTaskDelay(xDelay2);
//	} while (1);
}


u32 i2cdriver_init_Status = 0;

int get_i2cdriver_status(void) {
	if (i2cdriver_init_Status == 0)
		return 0;
	else
		return 1;
}

int i2c_driver_init(void) {
	int Status = XST_SUCCESS;
	const TickType_t xDelay2 = 5000;

	Status = init_MUX();
	if (Status != XST_SUCCESS) {
		xil_printf("\n\rIIC Init Failed \r\n");
		return XST_FAILURE;
	}
		else{

#if defined (AMP_COMMON_I2C_CONTROLLER)
	u32 *ptr;
	ptr=(u32*) (I2C_INIT_STATUS_REG);
		if(get_cpu_id()==CORE_0){
			*ptr=SH_VAR_INIT;				//Setting i2c Init status to True ,this to be done from core-0 in this release
		}

#endif

	i2cdriver_init_Status = 1;
		}
	xil_printf("\n\r %s done... \r\n",__func__);

	return XST_SUCCESS;
}

int sensor_stop(int pipe)
{
	stop_sensor(pipe);
	return XST_SUCCESS;
}




