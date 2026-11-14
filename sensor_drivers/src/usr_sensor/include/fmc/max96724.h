/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc. All right reserved.
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
#ifndef __nQuad_FMC_96724_H__
#define __nQuad_FMC_96724_H__

#include <isi/isi_fmc.h>
#include "hal/hal_i2c.h"

#if defined(AMP_COMMON_I2C_CONTROLLER) || defined(AMP_MULTIPLE_I2C_CONTROLLER)
#include "spinlock_address_map.h"
extern u32 *FMC_INIT_STATUS_REG;
extern u32 *SHARED_DES_ARRAY_SRUCT;
#endif

extern struct serializer_driver max9295_instance[];
extern struct sensor_driver sensor_instance[];

#ifndef MAX929X_TABLE_END
#define MAX929X_TABLE_END	(0xffff)
#define MAX929X_TABLE_WAIT	(0xfffe)
#endif
/* Defined unconditionally: isi_fmc.h supplies MAX929X_TABLE_END (so the
 * guard above is skipped) but not this MAX96724-specific wait value. */
#ifndef MAX96724_TABLE_WAIT_MS
#define MAX96724_TABLE_WAIT_MS	(10) //(100) // (210)
#endif
#ifndef NUM_INTERATION
#define NUM_INTERATION		(3)
#endif

/*****************************************************************************
 * MAX96724 quad-link GMSL deserializer port mapping.
 *
 * Each MAX96724 chip exposes 4 GMSL links (A/B/C/D). The framework's
 * desInterface now carries link_a/link_b/link_c/link_d directly, so one
 * physical chip == one desInterface entry.
 *
 *     entry chip_idx  -> Links A, B, C, D of that chip
 *
 * 3 chips * 4 links = 12 sensor pipes, with chip_idx == pipe_idx / 4.
 ****************************************************************************/

/* MAX96724 hardware bit values for the GMSL2_LINK_EN register (datasheet
 * 0xF00). These are physical register bits and are intentionally distinct
 * from the framework's LINK_A/B/C/D bitmask flags in isi_fmc.h (which use
 * 0x08/0x10 for C/D to avoid collision with the NO_LINK sentinel). */
#define MAX96724_LINK_C				(0x04)
#define MAX96724_LINK_D				(0x08)
#define MAX96724_LINK_MASK_ALL			(0x0F)

/* All four framework link-presence flags set (see LINK_A/B/C/D in isi_fmc.h).
 * Used as a desInterface.link_type bitmask by the MAX96724 quad driver. */
#define LINK_QUAD				(LINK_A | LINK_B | LINK_C | LINK_D)

/* MAX96724 register additions for Link C/D (placeholders - confirm with datasheet) */
#define CTRL5_REG				(0x14)  /* REMOVE */
#define CTRL11_REG				(0x500B) /* REMOVE */
#define CTRL15_REG				(0x500D) /* REMOVE */


#define M724_CTRL3_REG                    (0x1A) /*Link A status*/
#define CTRL12_REG                   (0x0A) /*Link B status*/
#define CTRL13_REG                   (0x0B) /*Link C status*/
#define CTRL14_REG                   (0x0C) /*Link D status*/

#define LINK_STATUS_BIT                 (3) /* bit 3 */


/* 3 physical MAX96724 chips, one desInterface entry per chip. */
#define MAX96724_NUM_CHIPS			(3)
#define MAX96724_LINKS_PER_CHIP			(4)
#define MAX96724_NUM_ENTRIES			(MAX96724_NUM_CHIPS)  /* 3 */

/* I2C addresses for the 3 MAX96724 chips. Tune to your board layout. */
#define M724_DS1_DEFAULT_ADDRESS		(0x4E)
#define M724_DS1_ALIAS_ADDRESS			(0x4E)
#define M724_DS2_DEFAULT_ADDRESS		(0x4E)
#define M724_DS2_ALIAS_ADDRESS			(0x4E)
#define M724_DS3_DEFAULT_ADDRESS		(0x4E)
#define M724_DS3_ALIAS_ADDRESS			(0x4E)

typedef struct {
	int num_of_serailizer;
	int num_of_deserializer;
	int SlaveAddress;
} nQuad_Context_t;

/* Optional explicit 4-link view used by helpers inside max96724.c.
 * desInterface itself now carries all 4 links, so this type is mainly
 * for code clarity when iterating links by index. */
typedef struct {
	u8			des_actual_addr;
	u8			des_alias_addr;
	u8			port_des_index;		/* chip index 0..MAX96724_NUM_CHIPS-1 */
	transition_state	des_state;
	u8			link_mask;		/* bitmask of LINK_A | LINK_B | LINK_C | LINK_D (framework values) */
	dslink			link[MAX96724_LINKS_PER_CHIP]; /* idx 0=A, 1=B, 2=C, 3=D */
} desInterface4_t;

/****************************************************************************
 *  Per-chip register initialization tables.
 *  The values below are derived from the MAX96716 tables in the original
 *  driver. They are placeholders for MAX96724; review against datasheet
 *  (especially the Link C/D pipe enables at 0x04CB..0x050B range).
 ****************************************************************************/

static RegI2CT m724_des1_init[] = {
	{MAX929X_TABLE_WAIT, MAX96724_TABLE_WAIT_MS},
    {0x0001,0xD0},
    {0x0003,0x55},
    {0x0007,0xF0},
    {0x040B,0x00},
    {0x0006,0xFF},
    {0x00F0,0x40}, // Pipeline select
    {0x00F1,0xC8}, // Pipeline select
    {0x00F4,0x0F},
    {0x090B,0x07},
    {0x090C,0x00},
    {0x090D,0x2C},
    {0x090E,0x2C},
    {0x090F,0x00},
    {0x0910,0x00},
    {0x0911,0x01},
    {0x0912,0x01},
    {0x092D,0x15},
    {0x094B,0x07},
    {0x094C,0x00},
    {0x094D,0x2C},
    {0x094E,0x6C},
    {0x094F,0x00},
    {0x0950,0x40},
    {0x0951,0x01},
    {0x0952,0x41},
    {0x096D,0x15},
    {0x098B,0x07},
    {0x098C,0x00},
    {0x098D,0x2C},
    {0x098E,0xAC},
    {0x098F,0x00},
    {0x0990,0x80},
    {0x0991,0x01},
    {0x0992,0x81},
    {0x09AD,0x15},
    {0x09CB,0x07},
    {0x09CC,0x00},
    {0x09CD,0x2C},
    {0x09CE,0xEC},
    {0x09CF,0x00},
    {0x09D0,0xC0},
    {0x09D1,0x01},
    {0x09D2,0xC1},
    {0x09ED,0x15},

    //{0x0973,0x01}, // Double mode, Needed for Quanta turn off for Leopard

    {0x08A0,0x04},
    // {0x094A,0xC0}, //VCX disabled
    {0x08A3,0xE4},
    {0x08A5,0x00},

    {0x0903,0x87}, // 2.5g
    {0x0904,0x87}, // 2.5g
    {0x0943,0x87}, // 2.5g
    {0x0944,0x87}, // 2.5g
    {0x08A1,0xB0}, // 2.5g

    {0x1D00,0xF4},
    {0x1D00,0xF4},

    {0x0418,0x39}, //2.5g
    {0x0415,0x39}, //2.5g
    {0x0418,0x39}, //2.5g
    {0x041B,0x39}, //2.5g
    {0x041E,0x39}, //2.5g

    {0x1D00,0xF5},
    {0x08A2,0x34},
    {0x08CA,0x55},
    {0x040B,0x02},  
	{MAX929X_TABLE_WAIT, MAX96724_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
};


static RegI2CT m724_des2_init[] = {
	{MAX929X_TABLE_WAIT, MAX96724_TABLE_WAIT_MS},
    {0x0001,0xD0},
    {0x0003,0x55},
    {0x0007,0xF0},
    {0x040B,0x00},
    {0x0006,0xFF},
    {0x00F0,0x40},
    {0x00F1,0xC8},
    {0x00F4,0x0F},
    {0x090B,0x07},
    {0x090C,0x00},
    {0x090D,0x2C},
    {0x090E,0x2C},
    {0x090F,0x00},
    {0x0910,0x00},
    {0x0911,0x01},
    {0x0912,0x01},
    {0x092D,0x15},
    {0x094B,0x07},
    {0x094C,0x00},
    {0x094D,0x2C},
    {0x094E,0x6C},
    {0x094F,0x00},
    {0x0950,0x40},
    {0x0951,0x01},
    {0x0952,0x41},
    {0x096D,0x15},
    {0x098B,0x07},
    {0x098C,0x00},
    {0x098D,0x2C},
    {0x098E,0xAC},
    {0x098F,0x00},
    {0x0990,0x80},
    {0x0991,0x01},
    {0x0992,0x81},
    {0x09AD,0x15},
    {0x09CB,0x07},
    {0x09CC,0x00},
    {0x09CD,0x2C},
    {0x09CE,0xEC},
    {0x09CF,0x00},
    {0x09D0,0xC0},
    {0x09D1,0x01},
    {0x09D2,0xC1},
    {0x09ED,0x15},

    //{0x0973,0x01}, // Double mode, Needed for Quanta turn off for Leopard

    {0x08A0,0x04},
    // {0x094A,0xC0}, //VCX disabled
    {0x08A3,0xE4},
    {0x08A5,0x00},
    {0x0903,0x87}, // 2.5g
    {0x0904,0x87}, // 2.5g
    {0x0943,0x87}, // 2.5g
    {0x0944,0x87}, // 2.5g
    {0x08A1,0xB0}, // 2.5g
    {0x1D00,0xF4},
    {0x1D00,0xF4},
    

    {0x0418,0x39}, //2.5g
    {0x0415,0x39}, //2.5g
    {0x0418,0x39}, //2.5g
    {0x041B,0x39}, //2.5g
    {0x041E,0x39}, //2.5g
    {0x1D00,0xF5},
    {0x08A2,0x34},
    {0x08CA,0x55},
    {0x040B,0x02},  
	{MAX929X_TABLE_WAIT, MAX96724_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
};

static RegI2CT m724_des3_init[] = {
	{MAX929X_TABLE_WAIT, MAX96724_TABLE_WAIT_MS},
    {0x0001,0xD0},
    {0x0003,0x55},
    {0x0007,0xF0},
    {0x040B,0x00},
    {0x0006,0xFF},
    {0x0003,0xAA}, // CPHY ADD - remote channels
    {0x00F0,0x40},
    {0x00F1,0xC8},
    {0x00F4,0x0F},
    {0x090B,0x07},
    {0x090C,0x00},
    {0x090D,0x2C},
    {0x090E,0x2C},
    {0x090F,0x00},
    {0x0910,0x00},
    {0x0911,0x01},
    {0x0912,0x01},
    {0x092D,0x2A}, // CPHY Change
    {0x094B,0x07},
    {0x094C,0x00},
    {0x094D,0x2C},
    {0x094E,0x6C},
    {0x094F,0x00},
    {0x0950,0x40},
    {0x0951,0x01},
    {0x0952,0x41},
    {0x096D,0x2A}, // CPHY Change
    {0x098B,0x07},
    {0x098C,0x00},
    {0x098D,0x2C},
    {0x098E,0xAC},
    {0x098F,0x00},
    {0x0990,0x80},
    {0x0991,0x01},
    {0x0992,0x81},
    {0x09AD,0x2A}, // CPHY Change
    {0x09CB,0x07},
    {0x09CC,0x00},
    {0x09CD,0x2C},
    {0x09CE,0xEC},
    {0x09CF,0x00},
    {0x09D0,0xC0},
    {0x09D1,0x01},
    {0x09D2,0xC1},
    {0x09ED,0x2A}, // CPHY Change

    //{0x0973,0x01}, // Double mode, Needed for Quanta turn off for Leopard

    //{0x08A0,0x04},
    //{0x094A,0xC0}, //VCX disabled //{0x094A,0xC0}, //VCX disabled
    {0x094A,0xB0},
    {0x090A,0xB0},
    {0x098A,0xB0},
    {0x09CA,0xB0},

    {0x098A,0xB0},   // MIPI_TX__2 : MIPI_TX10 | CSI2_LANE_CNT (Port B - Lane Count): 3
    {0x08A3,0xE4},
    //{0x08A5,0x00},
    {0x08A4,0xE4},   // MIPI_PHY : MIPI_PHY4 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1 | (Default) phy3_lane_map (Lane Map - PHY3 D0): Lane 2 | (Default) phy3_lane_map (Lane Map - PHY3 D1): Lane 3
    {0x08A6,0x00},   // MIPI_PHY : MIPI_PHY6 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 0): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 1): Normal
    {0x0983,0x07},   // MIPI_TX__2 : MIPI_TX3 | DESKEW_INIT (Controller 2 Auto Initial Deskew): Disabled
    {0x0984,0x01},   // MIPI_TX__2 : MIPI_TX4 | DESKEW_PER (Controller 2 Periodic Deskew): Disabled
    {0x1E00,0xF4},   //  (config_soft_rst_n - PHY2): 0x0
    {0x1E00,0xF4}, 
    
    // 1G = 0x2A
    {0x041B,0x2F},   // CSI Phy 2 is 1500 Mbps/lane (0x2F)
    {0x0415,0x2F}, // Line rate  1000Mbps/lane (0x2A)
    {0x0418,0x2F}, // Line rate 
    {0x041B,0x2F}, // Line rate
    {0x041E,0x2F}, // Line rate  ( 0x2E 1 frame, 0x2F = 3FS, )

    {0x1E00,0xF5},   // (config_soft_rst_n - PHY2): 0x1
    {0x08A2,0xC4},   // MIPI_PHY : MIPI_PHY2 | phy_Stdby_n (phy_Stdby_0): Put PHY0 in standby mode | phy_Stdby_n (phy_Stdby_1): Put PHY1 in standby mode
    {0x08CA,0x55}, // Check TODOTODO
    {0x040B,0x02},   //  (VID_TX_EN_X): Enabled
	{MAX929X_TABLE_WAIT, MAX96724_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
};

#if 0

static RegI2CT m724_des3_init[] = {
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
    {0x0001,0xD0},
    {0x0003,0x55},
    {0x0007,0xF0},
    {0x040B,0x00},
    {0x0006,0xFF},
    {0x0003,0xAA}, // CPHY ADD - remote channels
    {0x00F0,0x40},
    {0x00F1,0xC8},
    {0x00F4,0x0F},
    {0x090B,0x07},
    {0x090C,0x00},
    {0x090D,0x2C},
    {0x090E,0x2C},
    {0x090F,0x00},
    {0x0910,0x00},
    {0x0911,0x01},
    {0x0912,0x01},
    {0x092D,0x2A}, // CPHY Change
    {0x094B,0x07},
    {0x094C,0x00},
    {0x094D,0x2C},
    {0x094E,0x6C},
    {0x094F,0x00},
    {0x0950,0x40},
    {0x0951,0x01},
    {0x0952,0x41},
    {0x096D,0x2A}, // CPHY Change
    {0x098B,0x07},
    {0x098C,0x00},
    {0x098D,0x2C},
    {0x098E,0xAC},
    {0x098F,0x00},
    {0x0990,0x80},
    {0x0991,0x01},
    {0x0992,0x81},
    {0x09AD,0x2A}, // CPHY Change
    {0x09CB,0x07},
    {0x09CC,0x00},
    {0x09CD,0x2C},
    {0x09CE,0xEC},
    {0x09CF,0x00},
    {0x09D0,0xC0},
    {0x09D1,0x01},
    {0x09D2,0xC1},
    {0x09ED,0x2A}, // CPHY Change
    {0x08A0,0x04},
    // {0x094A,0xC0}, //VCX disabled
    //{0x090A,0xB0}, //0xE0
    //{0x094A,0xB0},
    //{0x098A,0xB0},
    //{0x09CA,0xB0},

    {0x098A,0xB0},   // MIPI_TX__2 : MIPI_TX10 | CSI2_LANE_CNT (Port B - Lane Count): 3
    {0x08A3,0xE4},
    {0x08A5,0x00},

    /*
    //{0x0903,0x87}, // 2.5g
    //{0x0904,0x87}, // 2.5g
    //{0x0943,0x87}, // 2.5g
    //{0x0944,0x87}, // 2.5g
    //{0x08A1,0xB0}, // 2.5g
    {0x1D00,0xF4},
    {0x1D00,0xF4},
    {0x0418,0x38}, //1.5g
    //{0x0418,0x39}, //2.5g
    //{0x0415,0x39}, //2.5g
    //{0x0418,0x39}, //2.5g
    //{0x041B,0x39}, //2.5g
    //{0x041E,0x39}, //2.5g
    
    {0x1D00,0xF5},
    {0x08A2,0x34},
    {0x08CA,0x55},
    {0x040B,0x02},*/
    
    {0x08A4,0xE4},   // MIPI_PHY : MIPI_PHY4 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1 | (Default) phy3_lane_map (Lane Map - PHY3 D0): Lane 2 | (Default) phy3_lane_map (Lane Map - PHY3 D1): Lane 3
    {0x08A6,0x00},   // MIPI_PHY : MIPI_PHY6 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 0): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 1): Normal
    {0x0983,0x07},   // MIPI_TX__2 : MIPI_TX3 | DESKEW_INIT (Controller 2 Auto Initial Deskew): Disabled
    {0x0984,0x01},   // MIPI_TX__2 : MIPI_TX4 | DESKEW_PER (Controller 2 Periodic Deskew): Disabled
    //{0x1E00,0xF4},   //  (config_soft_rst_n - PHY2): 0x0
    //{0x1E00,0xF4}, 
    //{0x041B,0x39},   // CSI Phy 2 is 2500 Mbps/lane
    //{0x0415,0x39}, // Line rate
    //{0x0418,0x39}, // Line rate
    //{0x041B,0x39}, // Line rate
    //{0x041E,0x39}, // Line rate

    {0x1E00,0xF5},   // (config_soft_rst_n - PHY2): 0x1
    {0x08A2,0xC4},   // MIPI_PHY : MIPI_PHY2 | phy_Stdby_n (phy_Stdby_0): Put PHY0 in standby mode | phy_Stdby_n (phy_Stdby_1): Put PHY1 in standby mode
    //{0x08CA,0x55}, // Check
    {0x040B,0x02},   //  (VID_TX_EN_X): Enabled
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
};


#endif
/*
{0x098A,0xB0}   // MIPI_TX__2 : MIPI_TX10 | CSI2_LANE_CNT (Port B - Lane Count): 3
{0x08A4,0xE4}   // MIPI_PHY : MIPI_PHY4 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1 | (Default) phy3_lane_map (Lane Map - PHY3 D0): Lane 2 | (Default) phy3_lane_map (Lane Map - PHY3 D1): Lane 3
{0x08A6,0x00}   // MIPI_PHY : MIPI_PHY6 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 0): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 1): Normal
{0x0983,0x07}   // MIPI_TX__2 : MIPI_TX3 | DESKEW_INIT (Controller 2 Auto Initial Deskew): Disabled
{0x0984,0x01}   // MIPI_TX__2 : MIPI_TX4 | DESKEW_PER (Controller 2 Periodic Deskew): Disabled
{0x1E00,0xF4}   //  (config_soft_rst_n - PHY2): 0x0
{0x1E00,0xF4} 
{0x041B,0x39}   // CSI Phy 2 is 2500 Mbps/lane
{0x0415,0x39} // Line rate
{0x0418,0x39} // Line rate
{0x041B,0x39} // Line rate
{0x041E,0x39} // Line rate
{0x1E00,0xF5}   // (config_soft_rst_n - PHY2): 0x1
{0x08A2,0xC4}   // MIPI_PHY : MIPI_PHY2 | phy_Stdby_n (phy_Stdby_0): Put PHY0 in standby mode | phy_Stdby_n (phy_Stdby_1): Put PHY1 in standby mode
{0x040B,0x02}   //  (VID_TX_EN_X): Enabled

*/
#endif /* __nQuad_FMC_96724_H__ */
