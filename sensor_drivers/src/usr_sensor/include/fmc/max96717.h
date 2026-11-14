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
#ifndef __XYLON_MAX_96717_H__
#define __XYLON_MAX_96717_H__

#include <isi/isi_fmc.h>

/* MAX929X_TABLE_END / _WAIT / _WAIT_MS come from isi_fmc.h. */


RegI2CT Ser0_init_96717[] = {
#if 0
	{0x0002, 0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0, 0x52},
	{0x0003, 0x03},
	{0x0570, 0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330, 0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383, 0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331, 0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332, 0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333, 0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334, 0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335, 0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308, 0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311, 0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318, 0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313, 0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E, 0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B, 0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002, 0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
#endif
//MAX96717 Settings from MAX9295 RAVE2
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},  // MIPI-RX pipe data-type (was 0x00)
	{0x0316, 0x52},  // (was 0x6C)
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},  // pipe start (was 0x25)
	{0x0002, 0x33},  // video pipe enable (was 0xF3)
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	//{0x02D6, 0x1E},  // 25.2 added
	//{0x02D8, 0x08},  // 25.2 added
	{0x02D6, 0x84}, //MFP8 : FSYNC - Leopard
	{0x02D6, 0x24},
	{0x02D8, 0x48},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
};

RegI2CT Ser1_init_96717[] = {
	#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
	#endif
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},  // MIPI-RX pipe data-type (was 0x00)
	{0x0316, 0x52},  // (was 0x6C)
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},  // pipe start (was 0x25)
	{0x0002, 0x33},  // video pipe enable (was 0xF3)
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	//{0x02D6, 0x1E},  // 25.2 added
	//{0x02D8, 0x08},  // 25.2 added
	{0x02D6, 0x84}, //MFP8 : FSYNC - Leopard
	{0x02D6, 0x24},
	{0x02D8, 0x48},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};


RegI2CT Ser2_init_96717[] = {
	#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
	#endif
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},  // MIPI-RX pipe data-type (was 0x00)
	{0x0316, 0x52},  // (was 0x6C)
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},  // pipe start (was 0x25)
	{0x0002, 0x33},  // video pipe enable (was 0xF3)
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	//{0x02D6, 0x1E},  // 25.2 added
	//{0x02D8, 0x08},  // 25.2 added
	{0x02D6, 0x84}, //MFP8 : FSYNC - Leopard
	{0x02D6, 0x24},
	{0x02D8, 0x48},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

RegI2CT Ser3_init_96717[] = {
	#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
	#endif
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},  // MIPI-RX pipe data-type (was 0x00)
	{0x0316, 0x52},  // (was 0x6C)
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},  // pipe start (was 0x25)
	{0x0002, 0x33},  // video pipe enable (was 0xF3)
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	//{0x02D6, 0x1E},  // 25.2 added
	//{0x02D8, 0x08},  // 25.2 added
	{0x02D6, 0x84}, //MFP8 : FSYNC - Leopard
	{0x02D6, 0x24},
	{0x02D8, 0x48},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

RegI2CT Ser4_init_96717[] = {
#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * 26.1 had left these at generic defaults (0x0314=0x00, 0x0316=0x6C,
	 * 0x0311=0x25, 0x0002=0xF3) -> serializer forwarded no valid pixels ->
	 * pipes 4-7 "launched" but produced no video. Aligned with Ser0_init. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

RegI2CT Ser5_init_96717[] = {
#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * Aligned with Ser1_init so pipe 5 forwards valid pixels. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x12},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x11},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};


RegI2CT Ser6_init_96717[] = {
#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * Aligned with Ser2_init so pipe 6 forwards valid pixels. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

RegI2CT Ser7_init_96717[] = {
#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * Aligned with Ser3_init so pipe 7 forwards valid pixels. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x12},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x11},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
};

RegI2CT Ser8_init_96717[] = {
#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * Aligned with Ser0_init so pipe 8 forwards valid pixels. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

RegI2CT Ser9_init_96717[] = {
	#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
	#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * Aligned with Ser1_init so pipe 9 forwards valid pixels. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x12},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x11},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

RegI2CT Ser10_init_96717[] = {
	#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
	#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * Converted from the old 0x4b (RGBIR-style) base to the proven 0x4C
	 * IMX623 base and aligned with Ser2_init so pipe 10 forwards pixels. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x11},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x12},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

RegI2CT Ser11_init_96717[] = {
	#if 0
	{0x0002,0x03}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
	{0x03F0,0x52},
	{0x0003,0x03},
	{0x0570,0x0C},
	{0x03F1, 0x89},
	{0x03F0, 0x59},
	{0x0006, 0xB0},
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0330,0x00}, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
	{0x0383,0x00}, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
	{0x0331,0x30}, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
	{0x0332,0xE0}, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
	{0x0333,0x04}, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
	{0x0334,0x00}, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
	{0x0335,0x00}, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
	{0x0308,0x64}, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
	{0x0311,0x40}, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
	{0x0318,0x6C}, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
	{0x0313,0x40}, // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
	{0x031E,0x38}, // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 | soft_bppz_en (soft_bppz_en): Software override enabled
	{0x005B,0x00}, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
	{0x0002,0x43}, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03F1, 0x89},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}
	#endif
	/* Restored 25.2 working IMX623 video-pipe config (see Ser0_init note).
	 * Converted from the old 0x4b (RGBIR-style) base to the proven 0x4C
	 * IMX623 base and aligned with Ser3_init so pipe 11 forwards pixels. */
	{0x02be, 0x80},
	{0x02bf, 0x60},
	{0x0053, 0x10},
	//{0x0057, 0x12},  // 25.2 leaves stream-id regs unwritten
	//{0x005B, 0x11},
	//{0x005F, 0x13},
	{0x0101, 0x4C},
	{0x0109, 0x0C},
	{0x0111, 0x4C},
	{0x0119, 0x4C},
	{0x0001, 0x48},
	{0x0330, 0x00},
	{0x0331, 0x33},
	{0x0332, 0xe0},
	{0x0333, 0x04},
	{0x0119, 0x4C},
	{0x0314, 0x6C},
	{0x0316, 0x52},
	{0x0318, 0x6C},
	{0x031A, 0x00},
	{0x0308, 0x7F},
	{0x0311, 0xF0},
	{0x0002, 0x33},
	{0x02be, 0x90},
	{0x02bf, 0x60},
	{0x03f1, 0x89},
	{0x02D6, 0x1E},
	{0x02D8, 0x08},
	{MAX929X_TABLE_WAIT, MAX929X_TABLE_WAIT_MS},
	{MAX929X_TABLE_END, 0}

};

int max96717_init(struct serializer_driver *ser_inst);
int max96717_deinit(struct serializer_driver *ser_inst);
int max96717_set_alias(struct serializer_driver *ser_inst, int link_number, int i2c_addr,
			int alias_addr);
int max96717_enable_link(struct serializer_driver *ser_inst, int link_mode);
int max96717_disable_link(struct serializer_driver *ser_inst, int link_mode);
int max96717_reset_links(struct serializer_driver *ser_inst);
int max96717_phy_lane_enable(struct serializer_driver *ser_inst, int phy_count);
int max96717_set_virt_id_map(struct serializer_driver *ser_inst, struct map_struct *map,
			int map_count);
int max96717_set_sensor_addr_map(struct serializer_driver *ser_inst, struct map_struct *map,
			int map_count);
int max96717_set_broadcast_addr(struct serializer_driver *ser_inst, int broadcast_addr);
int max96717_send_broadcast(struct serializer_driver *ser_inst, char *data, int size);
int max96717_i2c_write(struct serializer_driver *ser_inst, char *data, int size);
int max96717_i2c_read(struct serializer_driver *ser_inst, char *data, int size);

#endif
