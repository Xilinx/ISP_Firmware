/****************************************************************************
 * *
 * * The MIT License (MIT)
 * *
 * * Copyright (c) 2025 Advanced Micro Devices, Inc. All right reserved.
 * *
 * * Permission is hereby granted, free of charge, to any person obtaining a
 * * copy of this software and associated documentation files (the "Software"),
 * * to deal in the Software without restriction, including without limitation
 * * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * * and/or sell copies of the Software, and to permit persons to whom the
 * * Software is furnished to do so, subject to the following conditions:
 * *
 * * The above copyright notice and this permission notice shall be included in
 * * all copies or substantial portions of the Software.
 * *
 * * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * * DEALINGS IN THE SOFTWARE.
 * *
 * ****************************************************************************/
#ifndef _HALI2CAPI_H_
#define _HALI2CAPI_H_

#include <common/return_codes.h>
#include <ebase/types.h>
#include <oslayer/oslayer.h>
#include <ebase/dct_assert.h>
#include <common/buf_defs.h>

#ifdef FMC_PS_IIC
#ifndef SDT
#define IIC_DEVICE_ID		XPAR_XIICPS_1_DEVICE_ID
#else
#define IIC_INSTANCE_ZERO	0
#define IIC_INTR_ID		128
#define FMC_I2C_PS_BASEADDRESS	0xf1940000
#endif
#define IIC_MUX_ADDR		0x74
#define IIC_MUX_FMC_CHANNEL	0x6
#endif

#ifdef FMC_AXI_IIC
#ifndef SDT
#define IIC_DEVICE_ID		(XPAR_IIC_1_DEVICE_ID - 1)
#define IIC_INSTANCE_ZERO	0
#define IIC_INTR_ID		128
#else
#define IIC_INSTANCE_ZERO	0
#define IIC_INTR_ID		128
#endif
#define AXI_IIC_MUX_ADDR	0x71
#define AXI_IIC_MUX_FMC_CHANNEL	0x2
#endif

typedef enum HalI2cMode_e {
	HAL_AXI_I2C_MODE	= 0x0000,
	HAL_PS_I2C_MODE		= 0x0001
} HalI2cMode_t;

typedef struct HalI2cConfig_s {
	void		*hHalI2c;
	uint8_t		i2cBusId;
	HalI2cMode_t	HalI2cMode;
} HalI2cConfig_t;

typedef struct HalI2cApiOps_s HalI2cApiOps_t;

typedef void *HalI2cHandle_t;

typedef struct HalI2cContext_s {
	uint32_t		refCount;
	uint8_t			devId;
	int32_t			fd;
	osMutex			refMutex;
	uint8_t			slaveAddr;
	uint8_t			regWidth;
	uint8_t			dataWidth;
} HalI2cContext_t;

typedef RESULT (*HalI2cInit_t)(uint8_t bus_num);
typedef RESULT (*HalI2cDeInit_t)(HalI2cHandle_t handle);
typedef RESULT (*HalI2cReadReg_t)(u8 bus_num, u8 slave_addr, u32 reg_address, u8 reg_addr_size,
					void *preg_value, u8 datacount);
typedef RESULT (*HalI2cWriteReg_t)(u8 bus_num, u8 slave_addr, u16 reg_address, u8 reg_addr_size,
					u8 reg_value, u32 databytes);

struct HalI2cApiOps_s {
	const char		*pi2cName;
	HalI2cInit_t		pHalI2cInit;
	HalI2cDeInit_t		pHalI2cDeInit;
	HalI2cReadReg_t		pHalI2cReadReg;
	HalI2cWriteReg_t	pHalI2cWriteReg;
};

RESULT HalI2cInit(HalI2cConfig_t *pConfig, HalI2cHandle_t *pI2cHandle);
RESULT HalI2cDeInit(HalI2cHandle_t hHalI2c);
RESULT HalReadI2CReg(HalI2cHandle_t hHalI2c, uint8_t slaveAddr, uint16_t regAddr,
			uint8_t regWidth, uint16_t *data, uint8_t dataWidth);
RESULT HalWriteI2CReg(HalI2cHandle_t hHalI2c, uint8_t slaveAddr, uint16_t regAddr,
			uint8_t regWidth, uint16_t data, uint8_t dataWidth);
RESULT HalXilReadI2CReg(u8 bus_num, u8 slave_addr, u32 reg_address, u8 reg_addr_size,
			void *preg_value, u8 datacount);
RESULT HalXilWriteI2CReg(u8 bus_num, u8 slave_addr, u16 reg_address, u8 reg_addr_size,
			u8 reg_value, u32 databytes);

#endif
