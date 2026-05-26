/**************************************************************************
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
 ************************************************************************/
#include <fmc/max9295.h>

CREATE_TRACER(MAX_9295_INFO, "MAX_9295_INFO: ", INFO, 1);
CREATE_TRACER(MAX_9295_WARNING, "MAX_9295_WARNING: ", WARNING, 1);
CREATE_TRACER(MAX_9295_ERROR, "MAX_9295_ERROR: ", ERROR, 1);
CREATE_TRACER(MAX_9295_ALWAYS, "MAX_9295_ALWAYS: ", ALWAYS, 1);

struct serializer_driver max9295_instance[MAX_SER_COUNT] = {
	{
		.name = "max9295-0",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x0,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser0_init,
		.init_array_len = (sizeof(Ser0_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-1",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x0,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser1_init,
		.init_array_len = (sizeof(Ser1_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-2",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser2_init,
		.init_array_len = (sizeof(Ser2_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-3",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser3_init,
		.init_array_len = (sizeof(Ser3_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-4",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser4_init,
		.init_array_len = (sizeof(Ser4_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-5",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser5_init,
		.init_array_len = (sizeof(Ser5_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-6",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser6_init,
		.init_array_len = (sizeof(Ser6_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-7",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser7_init,
		.init_array_len = (sizeof(Ser7_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-8",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser8_init,
		.init_array_len = (sizeof(Ser8_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-9",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser9_init,
		.init_array_len = (sizeof(Ser9_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-10",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser10_init,
		.init_array_len = (sizeof(Ser10_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-11",
		.link_lane = LINK_B,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser11_init,
		.init_array_len = (sizeof(Ser11_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
	{
		.name = "max9295-2",
		.link_lane = LINK_A,
		.i2c_addr = 0x40,
		.bus_num = 0x1,
		.alias_addr = 0x0,
		.broadcast_addr = 0x0,
		.enable = 1,
		.alias_en = 0,
		.broadcast_en = 0,
		.ser_state = in_deinit,
		.init_array = Ser12_init,
		.init_array_len = (sizeof(Ser12_init))/(sizeof(RegI2CT)),
		.init_serializer = max9295_init,
		.deinit_serializer = max9295_deinit,
	},
};

/**************************************************************************
 *          max9295_init
 *
 * @brief   Initialize MAX9295 serializer with configuration data
 *
 * @param   ser_inst      Pointer to serializer driver instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Initialization successful
 * @retval  RET_FAILURE   Initialization failed
 *
 ************************************************************************/
RESULT max9295_init(struct serializer_driver *ser_inst)
{
	RESULT Status = RET_SUCCESS;

	if (ser_inst == NULL) {
		TRACE(MAX_9295_ERROR, "%s: ser_inst is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	if (ser_inst->ser_state == in_deinit) {
		u8 ser_addr = ser_inst->alias_addr >> 1;
		u8 wr_data[4] = {0};
		RegI2CT *Serializer_initialization = NULL;
		u32 len = 0;

#if defined(READ_I2C_REG)
		u8 rd_data[4] = { 0 };
#endif
		osSleep(100);

		Serializer_initialization = ser_inst->init_array;
		len = ser_inst->init_array_len;

#if defined(READ_I2C_REG)
		TRACE(MAX_9295_ALWAYS, "Initializing Serializer(%s) at
				Virtual Address = 0x%x on Serial Link %c...\n\r",
				ser_inst->name, ser_addr,
				(ser_inst->link_lane == LINK_A)?'A':'B');
#endif

		for (u32 reg_index = 0; reg_index < len; reg_index++) {
			if ((Serializer_initialization + reg_index)->addr ==
		    MAX929X_TABLE_END)
				break;

			if ((Serializer_initialization + reg_index)->addr ==
		    MAX929X_TABLE_WAIT) {
				osSleep((Serializer_initialization+reg_index)->val);
				continue;
			}

			wr_data[0] = ((Serializer_initialization + reg_index)->val);
			Status = HalWriteI2CReg(ser_inst->bus_num, ser_addr,
					(Serializer_initialization + reg_index)->addr,
		    0x2, wr_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_9295_ERROR,
				      "%s: HalWriteI2CReg failed at reg_index %lu, addr 0x%x (error=%d)\n",
				      __func__,
				      (unsigned long)reg_index,
				      (Serializer_initialization
				       + reg_index)->addr,
				      Status);
				return Status;
			}

#if defined(READ_I2C_REG)
			Status = HalReadI2CReg(ser_inst->bus_num, ser_addr,
					(Serializer_initialization + reg_index)->addr,
		    0x2, rd_data, 1);
			if (Status != RET_SUCCESS) {
				TRACE(MAX_9295_ERROR,
				      "%s: HalReadI2CReg failed at reg_index %lu, addr 0x%x (error=%d)\n",
				      __func__,
				      (unsigned long)reg_index,
				      (Serializer_initialization
				       + reg_index)->addr,
				      Status);
				return Status;
			}

			TRACE(MAX_9295_INFO, "%s Address=%x Read Data:%x\n", __func__,
					(Serializer_initialization + reg_index)->addr, rd_data[0]);
#endif

		}
		ser_inst->ser_state = in_init;
		ser_inst->ser_state = in_running;

#if defined(READ_I2C_REG)
		TRACE(MAX_9295_ALWAYS, "Initialization Done...\n\r");
#endif
		osSleep(100);
	} else {
		TRACE(MAX_9295_ALWAYS,
		      "Serializer Already in Running state(%d)\n\r",
		      ser_inst->ser_state);
	}

	return Status;
}

/**************************************************************************
 *          max9295_deinit
 *
 * @brief   Deinitialize MAX9295 serializer and reset state
 *
 * @param   ser_inst      Pointer to serializer driver instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Deinitialization successful
 * @retval  RET_FAILURE   Deinitialization failed
 *
 ************************************************************************/
RESULT max9295_deinit(struct serializer_driver *ser_inst)
{
	RESULT Status = RET_SUCCESS;

	if (ser_inst == NULL) {
		TRACE(MAX_9295_ERROR, "%s: ser_inst is NULL\n", __func__);
		return RET_NULL_POINTER;
	}

	u8 ser_addr = ser_inst->alias_addr >> 1;
	u8 wr_data[4] = {0};

	wr_data[0] = 0x80;
	Status = HalWriteI2CReg(ser_inst->bus_num, ser_addr, 0x10, 0x2, wr_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MAX_9295_ERROR,
			 "%s: HalWriteI2CReg failed for deinit reset (error=%d)\n",
			 	__func__, Status);
		return Status;
	}

	ser_inst->ser_state = in_deinit;

	return Status;
}
