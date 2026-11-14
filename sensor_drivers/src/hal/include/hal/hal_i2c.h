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
 *****************************************************************************/

#ifndef _HALI2CAPI_H_
#define _HALI2CAPI_H_

#include <common/return_codes.h>
#include <ebase/types.h>
#include <oslayer/oslayer.h>
#include <common/buf_defs.h>

#ifndef SDT
#define	IIC_DEVICE_ID				(XPAR_XIICPS_1_DEVICE_ID)
#else
#define	IIC_INSTANCE_ZERO			(0)
#define	IIC_INTR_ID				(128)
#define	FMC_I2C_PS_BASEADDRESS			(0xf1940000)
#endif

#define	IIC_MUX_ADDR				(0x74)
#define	IIC_MUX_FMC_CHANNEL			(0x6)

#define	AXI1_IIC_INSTANCE			(1)
#define	AXI1_FMC_I2C_BASEADDRESS		(0x80000000)

#define	AXI2_IIC_INSTANCE			(2)
#define	AXI2_FMC_I2C_BASEADDRESS		(0x80010000)

#define	AXI3_IIC_INSTANCE_ZERO			(3)
#define	AXI3_FMC_I2C_BASEADDRESS		(0x80020000)

#define	AXI_IIC_MUX_ADDR			(0x71)
#define	AXI_IIC_MUX_FMC_CHANNEL			(0x2)

/*****************************************************************************
 * @enum HalI2cMode_e
 * @brief HAL I2C mode enumeration
 *
 * This enumeration defines the different I2C controller modes supported
 * by the HAL I2C interface, including AXI I2C and PS I2C modes.
 ****************************************************************************/
typedef enum {
	HAL_AXI_I2C_MODE	= 0x0000,
	HAL_PS_I2C_MODE		= 0x0001,
	DUMMY_HAL_I2C_MODE	= 0xDEADFEED
} HalI2cMode_t;

typedef struct {
	void		*hHalI2c;
	uint8_t		i2cBusId;
	HalI2cMode_t	HalI2cMode;
} HalI2cConfig_t;

typedef struct HalI2cApiOps_s HalI2cApiOps_t;

typedef void *HalI2cHandle_t;

/*****************************************************************************
 * @brief hal I2C configuration struct
 *****************************************************************************/
typedef struct {
	uint8_t		i2cBusId;
	uint8_t		slaveAddr;
	uint8_t		regWidth;
	uint8_t		dataWidth;
	int32_t		fd;
	uint32_t	refCount;
	osMutex		refMutex;
	HalI2cApiOps_t	*pHalI2c;
	HalI2cApiOps_t	*pHalI2cApiOps;
	void		*pPrivateCtx;
} HalI2cContext_t;

typedef RESULT(*HalI2cInit_t) (uint8_t i2cBusId);
typedef RESULT(*HalI2cDeInit_t) (uint8_t i2cBusId);
typedef RESULT(*HalI2cReadReg_t) (uint8_t i2cBusId, uint8_t slaveAddr,
	uint16_t regAddr, uint8_t regWidth, uint8_t *data, uint8_t dataWidth);
typedef RESULT(*HalI2cWriteReg_t) (uint8_t i2cBusId, uint8_t slaveAddr,
	uint16_t regAddr, uint8_t regWidth, const uint8_t *data,
	uint8_t dataWidth);

struct HalI2cApiOps_s {
	const char		*pi2cName;
	HalI2cInit_t		pHalI2cInit;
	HalI2cDeInit_t		pHalI2cDeInit;
	HalI2cReadReg_t		pHalI2cReadReg;
	HalI2cWriteReg_t	pHalI2cWriteReg;
};

RESULT HalI2cApuInit(const HalI2cConfig_t *pConfig);
RESULT HalI2cInit(const HalI2cConfig_t *pConfig);
RESULT HalI2cDeInit(uint8_t i2cBusId);
RESULT HalReadI2CReg(uint8_t i2cBusId, uint8_t slaveAddr, uint16_t regAddr,
	uint8_t regWidth, uint8_t *data, uint8_t dataWidth);
RESULT HalWriteI2CReg(uint8_t i2cBusId, uint8_t slaveAddr, uint16_t regAddr,
	uint8_t regWidth, const uint8_t *data, uint8_t dataWidth);

/*****************************************************************************
 * @struct HalI2cBusConfig_t
 * @brief  Runtime I2C controller configuration for one bus.
 *
 * Populated from the FMC .bin (which is derived from the Vivado .xsa) by the
 * sensor_lib config loader. Lets the HAL select an I2C controller's base
 * address and GIC interrupt ID at run time instead of relying on the
 * compile-time IIC_INTR_ID / *_BASEADDRESS macros.
 ****************************************************************************/
typedef struct {
	uint8_t		bus_id;		/* 0 = PS I2C, 1..N = AXI I2C instance      */
	uint8_t		mode;		/* HAL_AXI_I2C_MODE(0) / HAL_PS_I2C_MODE(1) */
	uint16_t	intr_id;	/* GIC interrupt ID                         */
	uint32_t	base_addr;	/* controller base address                  */
	uint32_t	high_addr;	/* controller high address                  */
	uint32_t	clock_khz;	/* controller input clock (kHz)             */
	uint8_t		valid;		/* 1 = sourced from the .bin, 0 = default   */
} HalI2cBusConfig_t;

/*****************************************************************************
 * @brief  Return the runtime config for an I2C bus.
 *
 * If the .bin supplied a matching controller entry, its values are returned
 * (valid == 1). Otherwise AXI buses get only their default base address
 * (valid == 0), used by the HAL as the BSP XIic_LookupConfig() key.
 *
 * @param  bus_id  I2C bus id (0 = PS, 1..N = AXI instance)
 * @return Pointer to a static, caller-read-only config record (never NULL).
 ****************************************************************************/
const HalI2cBusConfig_t *HalI2cGetBusConfig(uint8_t bus_id);

#endif
