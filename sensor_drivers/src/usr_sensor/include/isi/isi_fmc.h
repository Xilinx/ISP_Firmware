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

#ifndef __ISI_FMC_H_
#define __ISI_FMC_H_

#include <sleep.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "xparameters.h"
#include "trace.h"
#include "return_codes.h"
#include "xiicps.h"
#include "xil_types.h"
#include "amp.h"
#include "hal_i2c.h"
#include <ebase/builtins.h>
#include <common/misc.h>
#include "types.h"

#if !defined(ISP_PRESI)
#include "xgpiops.h"
#endif

#define IN_PIPE_0				(0)
#define IN_PIPE_1				(1)
#define IN_PIPE_2				(2)
#define IN_PIPE_3				(3)
#define IN_PIPE_4				(4)
#define IN_PIPE_5				(5)
#define IN_PIPE_6				(6)
#define IN_PIPE_7				(7)
#define IN_PIPE_8				(8)
#define IN_PIPE_9				(9)
#define IN_PIPE_10				(10)
#define IN_PIPE_11				(11)
#define IN_PIPE_12				(12)

#define IN_PIPE_LAST				(IN_PIPE_12)
#define MAPPING_INPIPE_TO_DES_ID(pipeId)	((pipeId) / 2)

#define MAX_SER_COUNT				(IN_PIPE_LAST+1+1)
#define MAX_SENSOR_COUNT			(IN_PIPE_LAST+1+1)

#define I2C_2					(0x0042)
#define I2C_3					(0x0043)
#define I2C_4					(0x0044)
#define I2C_5					(0x0045)

#define LINK_MASK				(0x3)
#define LINK_A					(0x01)
#define LINK_B					(0x02)
#define LINK_REVERSE_SPLITTER			(0x03)
#define NO_LINK					(0x04)
#define AUTO_LINK				(0x4)
#define RESET_ONE_SHOT				(0x5)
#define LOCKED					(0x3)

#define DEV_ADDR_REG				(0x00)
#define CTRL0_REG				(0x10)
#define CTRL1_REG				(0x12)
#define CTRL3_REG				(0x13)
#define CTRL9_REG				(0x5009)
#define GMSL1_EN				(0xF00)

#define PDB_DES_DES1				(0)
#define PDB_DES_DES2				(1)
#define PDB_DES_DES3				(2)
#define PDB_DES_DES4				(3)
#define PDB_DES_DES5				(4)
#define PDB_DES_DES6				(5)
#define PDB_DES_DES7				(6)
#define CAM_SUPPLY_EN				(7)

#define DS_ONE					(0)
#define DS_TWO					(1)
#define DS_THREE				(2)
#define DS_FOUR					(3)
#define DS_FIVE					(4)
#define DS_SIX					(5)
#define DS_SEVEN				(6)
#define DS_MAX					(6+1)

#define OX03F10_SENSOR_ID			(0x580346)
#define OX08B40_SENSOR_ID			(0x580841)
#define OX05B1S_SENSOR_ID			(0x5805)
#define IMX728_SENSOR_ID			(0x1801)
#define IMX623_SENSOR_ID			(0x831101)

#define INVALID_I2C_BUS_ID         (0xFF)

#define SERIALIZER_DEFAULT_ADDR			(0xC4)
#define SERIALIZER_0_ALIAS_ADDR			(0xC2)
#define SERIALIZER_1_ALIAS_ADDR			(0xC6)
#define SERIALIZER_2_ALIAS_ADDR			(0xC8)
#define SERIALIZER_3_ALIAS_ADDR			(0xCA)
#define SERIALIZER_4_ALIAS_ADDR			(0xCC)
#define SERIALIZER_5_ALIAS_ADDR			(0xCE)
#define SERIALIZER_6_ALIAS_ADDR			(0x80)
#define SERIALIZER_7_ALIAS_ADDR			(0x82)
#define SERIALIZER_8_ALIAS_ADDR			(0x84)
#define SERIALIZER_9_ALIAS_ADDR			(0x86)
#define SERIALIZER_10_ALIAS_ADDR		(0x88)
#define SERIALIZER_11_ALIAS_ADDR		(0x8A)

#define SENSOR_OX3F10_ADDRESS			(0x6C)
#define SENSOR_OX8B40_ADDRESS			(0x6C)
#define SENSOR_OX5B_ADDRESS			(0x20)
#define SENSOR_IMX728_ADDRESS			(0x36)
#define SENSOR_IMX623_ADDRESS			(0x36)

#define SENSOR_DEFAULT_ADDRSS			(SENSOR_OX5B_ADDRESS)
#define SENSOR_0_ALIAS_ADDR			(SENSOR_DEFAULT_ADDRSS)
#define SENSOR_1_ALIAS_ADDR			(0x30)
#define SENSOR_2_ALIAS_ADDR			(0x32)
#define SENSOR_3_ALIAS_ADDR			(0x34)
#define SENSOR_4_ALIAS_ADDR			(0x26)
#define SENSOR_5_ALIAS_ADDR			(0x38)
#define SENSOR_6_ALIAS_ADDR			(0x3A)
#define SENSOR_7_ALIAS_ADDR			(0x3C)
#define SENSOR_8_ALIAS_ADDR			(0x3E)
#define SENSOR_9_ALIAS_ADDR			(0x20)
#define SENSOR_10_ALIAS_ADDR			(0x22)
#define SENSOR_11_ALIAS_ADDR			(0x24)

#define MAX929X_TABLE_END			(0xffff)
#define MAX929X_TABLE_WAIT			(0xfffe)
#define MAX929X_TABLE_WAIT_MS			(50)
#define TABLE_WAIT				(0xfffe)
#define TABLE_END				(0xffff)
#define SENSOR_MAX				(12)
#define NUM_INTERATION				(3)

extern unsigned int *FMC_INIT_STATUS_REG;
extern unsigned int *SHARED_DES_ARRAY_SRUCT;
extern unsigned int *DES_REMAPPED_STATUS_REG;
extern u8 cpu_id;

/*****************************************************************************
 *          transition_state
 *
 * @brief   State machine transitions for FMC device init and operation.
 *****************************************************************************/
typedef enum {
	in_deinit	= 0,
	in_progress,
	in_init,
	in_running,
	in_stop
} transition_state;

typedef struct {
	u8	serializer_default_addr;
	u8	serializer_alias_addr;
	u8	sensor_default_addr;
	u8	sensor_alias_addr;
} dslink;

typedef struct {
	u8	des_actual_addr;
	u8	des_alias_addr;
	u8	Port_DES_index;
	transition_state	des_state;
	u8	link_type;
	dslink	link_a;
	dslink	link_b;
} desInterface;

/**
 * core_des_mapping_t - Core to Deserializer to I2C Bus Mapping
 *
 * This structure defines the complete mapping from CPU core to deserializers
 * and their associated I2C bus controllers. This centralizes hardware topology
 * configuration in one place.
 *
 * @core_id:            CPU core ID (e.g., 6, 7, 8)
 * @num_deserializers:  Number of deserializers managed by this core
 * @des_indices:        Array of deserializer indices (DS_ONE, DS_TWO, etc.)
 * @i2cBusIds:          Array of I2C bus IDs (0-3) for each deserializer
 *
 * Example:
 *   {6, 3, {DS_ONE, DS_TWO, DS_THREE}, {1, 1, 1}}
 *   Core 6 manages 3 deserializers, all on I2C bus 1
 */
typedef struct {
	u8 core_id;		/* CPU core ID */
	u8 num_deserializers;	/* Number of deserializers on this core */
	u8 des_indices[3];	/* Deserializer indices (max 3 per core) */
	u8 i2cBusIds[3];	/* I2C bus ID for each deserializer */
} core_des_mapping_t;

typedef int (*IsiCreateFmcIss_t) ();
typedef int (*IsiFmcSetup) (int);
typedef int (*IsiDeserSetup) (desInterface *);
typedef int (*IsiDeserEnable) (u8);
typedef int (*IsiDeserDisable) (u8);

struct IsiFmc_s {
	char			FmcName[30];
	IsiCreateFmcIss_t	pIsiCreateFmcIss;
	IsiFmcSetup		pIsiIsiFmcSetup;
	IsiDeserSetup		pIsiDeserSetup;
	IsiDeserEnable		pIsiDeserEnable;
	IsiDeserDisable		pIsiDeserDisable;
	struct serializer_driver	*serializer_array[MAX_SER_COUNT];
	struct sensor_driver		*sensor_array[MAX_SENSOR_COUNT];
	struct accessIIC		*accessiic_array[MAX_SENSOR_COUNT];
};

typedef struct IsiFmc_s IsiFmc_t;
extern IsiFmc_t g_fmc_max9296;
extern IsiFmc_t g_fmc_max96716;
extern int g_Sensor_frame_count;

typedef struct {
	u16	addr;
	u8	val;
} RegI2CT;

typedef struct {
	u16	address;
	u8	value;
} reg_8;

typedef struct {
	u16	address;
	u16	value;
	int	delay;
} RegWrite;

struct map_struct {
	int	orig;
	int	alias;
};

struct serializer_driver {
	char	name[30];
	u8	link_lane;
	int	i2c_addr;
	int	bus_num;
	int	alias_addr;
	int	broadcast_addr;
	int	enable;
	int	alias_en;
	int	broadcast_en;
	transition_state	ser_state;
	RegI2CT	*init_array;
	u32	init_array_len;
	int (*init_serializer)(struct serializer_driver *ser_inst);
	int (*deinit_serializer)(struct serializer_driver *ser_inst);
	int (*set_alias_addr)(struct serializer_driver *ser_inst,
				int link_number, int addr,
				int alias_addr);
	int (*enable_link)(struct serializer_driver *ser_inst, int link);
	int (*disable_link)(struct serializer_driver *ser_inst, int link);
};

struct accessIIC {
	u8	i2cBusId;
	int (*readIIC)(u8 i2cBusId, u8 slave_addr, uint16_t addr,
			uint8_t regWidth, uint8_t *pValue, uint8_t dataWidth);
	int (*writeIIC)(u8 i2cBusId, u8 slave_addr, uint16_t addr,
			uint8_t regWidth, uint8_t *pValue, uint8_t dataWidth);
};

struct sensor_driver {
	char	name[30];
	u8	pipe_no;
	reg_8	*init_array;
	u32	init_array_len;
	reg_8	*streamon_array;
	u32	streamon_array_len;
	reg_8	*streamoff_array;
	u32	streamoff_array_len;
	reg_8	*fps_array;
	u32	fps_array_len;
	u8	sensor_alias_addr;
	int (*init_sensor)(struct sensor_driver *);
	int (*deinit_sensor)(struct sensor_driver *);
	int (*stream_on)(struct sensor_driver *);
	int (*stream_off)(struct sensor_driver *);
	transition_state sensor_state;
};

/*****************************************************************************
 * GetI2cBusIdForDes - Lookup I2C bus ID for a deserializer
 *
 * @param   desId   Deserializer index (DS_ONE, DS_TWO, etc.)
 *
 * @return  I2C bus ID (0-3) or 0xFF if not found
 *
 * @note    Queries the core_des_map table to determine which I2C bus
 *          controller is used by the specified deserializer.
 *****************************************************************************/
u8 GetI2cBusIdForDes(u8 desId);

/*****************************************************************************
 * Multi-Sensor Framework Functions
 * Common declarations used by all sensor drivers
 *****************************************************************************/

/**
 * @brief Initialize I2C access interface for the specified pipeline
 */
extern int init_iic_access(u8 i2cBusId, int in_pipe);

/**
 * @brief Initialize deserializer interface for the specified index
 */
extern int init_des(int des_index);

/**
 * @brief Initialize sensor driver for the specified pipeline and deserializer
 */
extern int init_sensor(int in_pipe, int des_index, int sensor_address);

/**
 * @brief Stop streaming on the specified sensor pipeline
 */
extern void stop_sensor(int in_pipe);

/**
 * @brief Display status for all deserializers and serializers
 */
extern void Fmc_Sensor_Statustask(void);

/**
 * @brief Get currently active FMC instance
 */
extern IsiFmc_t *get_active_fmc(void);

extern struct serializer_driver max9295_instance[];
extern void byte_memcpy(void *dest, const void *src, u32 size);
extern void revert_to_400khz(void);

extern desInterface des_arr[];
extern desInterface max96716_des_arr[];

extern const core_des_mapping_t max9296_core_des_map[];
extern const core_des_mapping_t max96716_core_des_map[];
extern u8 GetI2cBusIdForDes(u8 desId);
extern unsigned int *I2C_INIT_STATUS_REG;
extern u8 MAX96716_CORE_DES_MAP_SIZE;

extern IsiFmc_t g_fmc_max9296;
extern IsiFmc_t g_fmc_max96716;
#endif
