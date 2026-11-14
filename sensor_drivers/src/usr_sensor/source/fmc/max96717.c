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
#include <fmc/max96717.h>
#include "isi/isi_fmc.h"

/* sensor_lib has no ebase/dct_assert.h (telluride/BSP only); provide a local
 * fail-loud assert instead of pulling in telluride headers. */
#ifndef DCT_ASSERT
#define DCT_ASSERT(x) do { if (!(x)) { \
	xil_printf("DCT_ASSERT failed: %s:%d\r\n", __func__, __LINE__); } } while (0)
#endif

#define SER_ADDR 0x40

struct serializer_driver max96717_instance[MAX_SER_COUNT] = {
	{
		.name = "max96717-0",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser0_init_96717,
		.init_array_len = (sizeof(Ser0_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-1",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser1_init_96717,
		.init_array_len = (sizeof(Ser1_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-2",
		.link_lane = LINK_C,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser2_init_96717,
		.init_array_len = (sizeof(Ser2_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-3",
		.link_lane = LINK_D,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser3_init_96717,
		.init_array_len = (sizeof(Ser3_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-4",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x2,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser4_init_96717,
		.init_array_len = (sizeof(Ser4_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-5",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x2,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser5_init_96717,
		.init_array_len = (sizeof(Ser5_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-6",
		.link_lane = LINK_C,
		.i2c_addr = 0x40,
		.bus_num = 0x2,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser6_init_96717,
		.init_array_len = (sizeof(Ser6_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-7",
		.link_lane = LINK_D,
		.i2c_addr = 0x40,
		.bus_num = 0x2,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser7_init_96717,
		.init_array_len = (sizeof(Ser7_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-8",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x3,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser8_init_96717,
		.init_array_len = (sizeof(Ser8_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-9",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x3,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser9_init_96717,
		.init_array_len = (sizeof(Ser9_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-10",
		.link_lane = LINK_C,
		.i2c_addr = 0x40,
		.bus_num = 0x3,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser10_init_96717,
		.init_array_len = (sizeof(Ser10_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-11",
		.link_lane = LINK_D,
		.i2c_addr = 0x40,
		.bus_num = 0x3,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser11_init_96717,
		.init_array_len = (sizeof(Ser11_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
	{
		.name = "max96717-2",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser2_init_96717,
		.init_array_len = (sizeof(Ser2_init_96717))/(sizeof(RegI2CT)),
		.init_serializer = max96717_init,
		.deinit_serializer = max96717_deinit,
		.set_alias_addr = max96717_set_alias,
		.enable_link = max96717_enable_link,
		.disable_link = max96717_disable_link,
		.reset_links = max96717_reset_links,
		.phy_lanes_enable = max96717_phy_lane_enable,
		.set_virt_ch_map = max96717_set_virt_id_map,
		.set_sensor_addr_map = max96717_set_sensor_addr_map,
		.set_broadcast_addr = max96717_set_broadcast_addr,
		.send_broadcast = max96717_send_broadcast,
		.i2c_write = max96717_i2c_write,
		.i2c_read = max96717_i2c_read,
	},
};

/*******************************************************************************
 *          max96717_init
 *
 * @brief   Initialize MAX96717 serializer with configuration data
 *
 * @param   ser_inst      Pointer to serializer driver instance
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Initialization successful
 * @retval  XST_FAILURE   Initialization failed
 *
 *****************************************************************************/
int max96717_init(struct serializer_driver *ser_inst)
{
	int Status = XST_SUCCESS;

	if (ser_inst->ser_state == in_deinit) {

		u8 ser_addr = ser_inst->alias_addr >> 1;
		u8 wr_data[4] = {0};
		RegI2CT *Serializer_initialization;
		u32 len;

//#if defined(READ_I2C_REG)
		u8 rd_data[4] = { 0 };
//#endif
		osSleep(1000);

		Serializer_initialization = ser_inst->init_array;
		len = ser_inst->init_array_len;

#if !defined(READ_I2C_REG)
		xil_printf("Initializing Serializer(%s) at Virtual Address = 0x%x on Serial Link %c...\r\n",
					ser_inst->name, ser_addr,
				(ser_inst->link_lane == LINK_A)?'A':'B');
#endif
		for (int j = 0; j < len; j++) {
			if ((Serializer_initialization + j)->addr == MAX929X_TABLE_END)
				break;

			if ((Serializer_initialization + j)->addr == MAX929X_TABLE_WAIT) {
				osSleep((Serializer_initialization+j)->val);
				continue;
			}

			wr_data[0] = ((Serializer_initialization + j)->val);

			Status = HalWriteI2CReg(ser_inst->bus_num /*IIC_DEVICE_ID*/, ser_addr,
					(Serializer_initialization + j)->addr, 0x2, wr_data, 1);

			// Status = HalWriteI2CReg(IIC_INSTANCE_ZERO /*IIC_DEVICE_ID*/, ser_addr,
			// 		(Serializer_initialization + j)->addr, 0x2, wr_data, 1);
			if (Status != XST_SUCCESS)
				DCT_ASSERT(0);
//#if defined(READ_I2C_REG)
			Status = HalReadI2CReg(ser_inst->bus_num /*IIC_DEVICE_ID*/, ser_addr,
				(Serializer_initialization + j)->addr, 0x2, rd_data, 1);

			// Status = HalReadI2CReg(IIC_INSTANCE_ZERO /*IIC_DEVICE_ID*/, ser_addr,
			// 		(Serializer_initialization + j)->addr, 0x2, rd_data, 1);
			if (Status != XST_SUCCESS)
				DCT_ASSERT(0);

			xil_printf("%s Address=%x Read Data:%x\r\n", __func__,
					(Serializer_initialization + j)->addr, rd_data[0]);
//#endif
		}

		ser_inst->ser_state = in_init;
		ser_inst->ser_state = in_running;

#if !defined(READ_I2C_REG)
		xil_printf("Initialization Done...\r\n");
#endif
		osSleep(1000);
	} else {
		xil_printf("Serializer Already in Running state(%d)\r\n", ser_inst->ser_state);
	}

	return Status;
}

/*******************************************************************************
 *          max96717_deinit
 *
 * @brief   Deinitialize MAX96717 serializer and reset state
 *
 * @param   ser_inst      Pointer to serializer driver instance
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Deinitialization successful
 * @retval  XST_FAILURE   Deinitialization failed
 *
 *****************************************************************************/
int max96717_deinit(struct serializer_driver *ser_inst)
{
	int Status = XST_SUCCESS;
	u8 ser_addr = ser_inst->alias_addr >> 1;
	u8 wr_data[4] = {0};
  	wr_data[0] = 0x80;
	Status = HalWriteI2CReg(ser_inst->bus_num /*IIC_DEVICE_ID*/, ser_addr, 0x10, 0x2, wr_data, 1);
	// Status = HalWriteI2CReg(IIC_INSTANCE_ZERO /*IIC_DEVICE_ID*/, ser_addr, 0x10, 0x2, wr_data, 1);
	if (Status != XST_SUCCESS)
		DCT_ASSERT(0);
	ser_inst->ser_state = in_deinit;
	return Status;
}

/*******************************************************************************
 *          max96717_set_alias
 *
 * @brief   Set alias address for MAX96717 serializer on specified link
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   link_number   Link number to configure
 * @param   i2c_addr      Original I2C address
 * @param   alias_addr    Alias address to set
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Alias address set successfully
 * @retval  XST_FAILURE   Failed to set alias address
 *
 *****************************************************************************/
int max96717_set_alias(struct serializer_driver *ser_inst, int link_number,
		int i2c_addr, int alias_addr)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_enable_link
 *
 * @brief   Enable link on MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   link_mode     Link mode to enable
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Link enabled successfully
 * @retval  XST_FAILURE   Failed to enable link
 *
 *****************************************************************************/
int max96717_enable_link(struct serializer_driver *ser_inst, int link_mode)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_disable_link
 *
 * @brief   Disable link on MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   link_mode     Link mode to disable
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Link disabled successfully
 * @retval  XST_FAILURE   Failed to disable link
 *
 *****************************************************************************/
int max96717_disable_link(struct serializer_driver *ser_inst, int link_mode)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_reset_links
 *
 * @brief   Reset all links on MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Links reset successfully
 * @retval  XST_FAILURE   Failed to reset links
 *
 *****************************************************************************/
int max96717_reset_links(struct serializer_driver *ser_inst)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_phy_lane_enable
 *
 * @brief   Enable specified number of PHY lanes on MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   phy_count     Number of PHY lanes to enable
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   PHY lanes enabled successfully
 * @retval  XST_FAILURE   Failed to enable PHY lanes
 *
 *****************************************************************************/
int max96717_phy_lane_enable(struct serializer_driver *ser_inst, int phy_count)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_set_virt_id_map
 *
 * @brief   Set virtual channel ID mapping on MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   map           Pointer to mapping structure array
 * @param   map_count     Number of mapping entries
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Virtual ID mapping set successfully
 * @retval  XST_FAILURE   Failed to set virtual ID mapping
 *
 *****************************************************************************/
int max96717_set_virt_id_map(struct serializer_driver *ser_inst,
		struct map_struct *map, int map_count)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_set_sensor_addr_map
 *
 * @brief   Set sensor address mapping on MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   map           Pointer to mapping structure array
 * @param   map_count     Number of mapping entries
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Sensor address mapping set successfully
 * @retval  XST_FAILURE   Failed to set sensor address mapping
 *
 *****************************************************************************/
int max96717_set_sensor_addr_map(struct serializer_driver *ser_inst, struct map_struct *map,
				int map_count)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_set_broadcast_addr
 *
 * @brief   Set broadcast address for MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   broadcast_addr Broadcast address to set
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Broadcast address set successfully
 * @retval  XST_FAILURE   Failed to set broadcast address
 *
 *****************************************************************************/
int max96717_set_broadcast_addr(struct serializer_driver *ser_inst, int broadcast_addr)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_send_broadcast
 *
 * @brief   Send broadcast message via MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   data          Pointer to data buffer to broadcast
 * @param   size          Size of data to broadcast
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   Broadcast sent successfully
 * @retval  XST_FAILURE   Failed to send broadcast
 *
 *****************************************************************************/
int max96717_send_broadcast(struct serializer_driver *ser_inst, char *data, int size)
{
	return XST_SUCCESS;
}

/******************************************************************************
 *          max96717_i2c_write
 *
 * @brief   Write data via I2C through MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   data          Pointer to data buffer to write
 * @param   size          Size of data to write
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   I2C write successful
 * @retval  XST_FAILURE   I2C write failed
 *
 *****************************************************************************/
int max96717_i2c_write(struct serializer_driver *ser_inst, char *data, int size)
{
	return XST_SUCCESS;
}

/*******************************************************************************
 *          max96717_i2c_read
 *
 * @brief   Read data via I2C through MAX96717 serializer
 *
 * @param   ser_inst      Pointer to serializer driver instance
 * @param   data          Pointer to data buffer to store read data
 * @param   size          Size of data to read
 *
 * @return  Return the result of the function call.
 * @retval  XST_SUCCESS   I2C read successful
 * @retval  XST_FAILURE   I2C read failed
 *
 *****************************************************************************/
int max96717_i2c_read(struct serializer_driver *ser_inst, char *data, int size)
{
	return XST_SUCCESS;
}
