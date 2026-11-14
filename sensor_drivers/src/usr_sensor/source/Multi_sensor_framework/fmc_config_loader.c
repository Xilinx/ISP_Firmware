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

/*
 * fmc_config_loader.c — Runtime FMC configuration loader.
 *
 * Parses the .bin produced by the FMC Config Generator (derived from the
 * Vivado .xsa) and overwrites the sensor_lib runtime tables that were
 * previously fixed by the compile-time macros:
 *
 *   - des_arr[] / max96716_des_arr[]  (DES actual/alias addresses, PDB GPIO,
 *                          link info) - the array of the ACTIVE FMC is filled,
 *                          chosen from the FMC selection id in the .bin
 *                          (FMC_ID_MAX9296 = MAX9296 = des_arr; the GMSL2
 *                          family MAX96716 / MAX96724 / MAX96726 =
 *                          max96716_des_arr)
 *   - max9295_instance[]   (serializer i2c/alias address, bus, link lane;
 *                          shared by both FMCs; the selected serializer model
 *                          per pipe is exposed via fmc_pipe_info_t)
 *   - g_runtime_core_map[] (core -> DES -> I2C bus)
 *   - g_des_bus[]          (I2C bus id per DES, consumed by GetI2cBusIdForDes)
 *   - g_i2c_bus[]          (I2C controller base address + GIC interrupt id,
 *                           exposed to the HAL via HalI2cGetBusConfig())
 *
 * Everything the previous macros hard-coded now comes from the .bin.
 */

#include <stdint.h>

#include <isi/isi_fmc.h>
#include <fmc/fmc_config.h>
#include <hal/hal_i2c.h>

CREATE_TRACER(FMC_CFG_INFO,    "FMC_CFG_INFO: ",    INFO,    1);
CREATE_TRACER(FMC_CFG_WARNING, "FMC_CFG_WARNING: ", WARNING, 1);
CREATE_TRACER(FMC_CFG_ERROR,   "FMC_CFG_ERROR: ",   ERROR,   1);
CREATE_TRACER(FMC_CFG_ALWAYS,  "FMC_CFG: ",         ALWAYS,  1);

/* ── Runtime tables populated from the .bin ────────────── */

static fmc_des_info_t     g_des_info[DS_MAX];
static core_des_mapping_t g_runtime_core_map[DS_MAX];
static fmc_pipe_info_t    g_pipe_info[FMC_MAX_PIPES];
static fmc_i2c_bus_info_t g_i2c_bus[FMC_MAX_I2C_BUS];
static u8                 g_des_bus[DS_MAX];

static int g_des_count;
static int g_sensor_count;
static int g_core_count;
static int g_bus_count;
static int g_loaded;
static int g_fmc_id = -1;   /* FMC selection id parsed from the .bin (-1=none) */

/* Source used by the address-based loader (fmc_config_apply_from_memory). */
static const u8 *g_src_ptr = (const u8 *)(FMC_CONFIG_BIN_BASE_ADDR);
static u32       g_src_len;

/**************************************************************************
 *          fmc_bin_len_from_header
 *
 * @brief   Derive the total .bin length from its header so the address-based
 *          loader can work from a base pointer alone.
 *
 * @param   data   Candidate buffer
 * @param   cap    Maximum number of bytes that may be read
 *
 * @return  Byte length declared by the header (<= cap), or 0 if invalid.
 ************************************************************************/
static u32 fmc_bin_len_from_header(const u8 *data, u32 cap)
{
	u32 des_off, sen_off, i2c_off, bus_off, end, e, magic;
	u16 des_cnt, sen_cnt, i2c_cnt, bus_cnt;

	if ((data == NULL) || (cap < FMC_HEADER_SIZE))
		return 0U;

	TRACE(FMC_CFG_ALWAYS,
	      "%s: about to READ .bin header at 0x%08X - if the log STOPS right\r\n"
	      "         here, that address is NOT readable by the R5 (MPU/region\r\n"
	      "         not mapped) => fix the MPU or use a mapped address.\r\n",
	      __func__, (unsigned)(uintptr_t)data);

	magic = ((u32)data[0] << 24) | ((u32)data[1] << 16)
	      | ((u32)data[2] << 8)  |  (u32)data[3];

	TRACE(FMC_CFG_ALWAYS, "%s: READ OK - magic=0x%08X (expect 0x%08X)\r\n",
	      __func__, (unsigned)magic, (unsigned)FMC_BIN_MAGIC);

	if (magic != FMC_BIN_MAGIC) {
		TRACE(FMC_CFG_ERROR,
		      "%s: bad magic at 0x%08X - not a valid .bin (wrong addr, not\r\n"
		      "         loaded yet, or overwritten by Linux)\r\n",
		      __func__, (unsigned)(uintptr_t)data);
		return 0U;
	}

	des_cnt = fmc_rd16(&data[6]);
	sen_cnt = fmc_rd16(&data[8]);
	i2c_cnt = fmc_rd16(&data[10]);
	des_off = fmc_rd32(&data[12]);
	sen_off = fmc_rd32(&data[16]);
	i2c_off = fmc_rd32(&data[20]);
	bus_cnt = fmc_rd16(&data[24]);
	bus_off = fmc_rd32(&data[28]);

	end = FMC_HEADER_SIZE;
	e = des_off + ((u32)des_cnt * FMC_DES_ENTRY_SIZE); if (e > end) end = e;
	e = sen_off + ((u32)sen_cnt * FMC_SEN_ENTRY_SIZE); if (e > end) end = e;
	e = i2c_off + ((u32)i2c_cnt * FMC_I2C_ENTRY_SIZE); if (e > end) end = e;
	if (bus_cnt != 0U) {
		e = bus_off + ((u32)bus_cnt * FMC_BUS_ENTRY_SIZE);
		if (e > end) end = e;
	}

	/* v3+ appends a CRC-32 trailer; include it so the returned length is the
	 * full file size that fmc_config_verify_crc() expects. */
	if (fmc_rd16(&data[4]) >= FMC_BIN_VERSION_CRC)
		end += FMC_CRC_TRAILER_SIZE;

	TRACE(FMC_CFG_ALWAYS,
	      "%s: version=%u des_cnt=%u sen_cnt=%u i2c_cnt=%u bus_cnt=%u total_len=%u\r\n",
	      __func__, (unsigned)fmc_rd16(&data[4]), (unsigned)des_cnt,
	      (unsigned)sen_cnt, (unsigned)i2c_cnt, (unsigned)bus_cnt, (unsigned)end);

	if (end > cap) {
		TRACE(FMC_CFG_ERROR, "%s: declared len %u > cap %u\r\n",
		      __func__, (unsigned)end, (unsigned)cap);
		return 0U;
	}

	return end;
}

/**************************************************************************
 *          fmc_pick_des_array
 *
 * @brief   Choose which DES array the .bin should populate:
 *            - MAX9296             = des_arr
 *            - MAX96716 / MAX96726 = max96716_des_arr (GMSL2 dual-link)
 *            - MAX96724            = ndes_arr (quad, one entry per chip)
 *
 *          The active FMC (if already selected) decides, since that is the
 *          array the rest of the stack will use. When no FMC is selected yet
 *          the choice falls back to the FMC selection id (header byte [26],
 *          derived from the DES chip name) carried in the .bin.
 *
 * @return  Pointer to the DES array to populate (never NULL).
 ************************************************************************/
static desInterface *fmc_pick_des_array(const u8 *data, u32 len)
{
	IsiFmc_t *active = get_active_fmc();
	int fmc_id;

	if (active == &g_fmc_max96716)
		return max96716_des_arr;
	if (active == &g_fmc_max9296)
		return des_arr;
	if (active == &g_fmc_rave)
		return ndes_arr;

	/* No FMC selected yet: route by the FMC selection id in the .bin.
	 * MAX9296 = des_arr; MAX96716 / MAX96726 = max96716_des_arr (dual);
	 * MAX96724 = ndes_arr (quad deserializer, one entry per chip). */
	fmc_id = fmc_config_get_fmc_id(data, len);
	if (fmc_id == FMC_ID_MAX96724)
		return ndes_arr;
	if ((fmc_id == FMC_ID_MAX96716) ||
	    (fmc_id == FMC_ID_MAX96726))
		return max96716_des_arr;

	return des_arr;
}

/**************************************************************************
 *          fmc_config_apply_bin
 *
 * @brief   Parse a .bin buffer and overwrite every runtime table.
 *
 * @param   data   Raw .bin buffer
 * @param   len    Size of the buffer in bytes
 *
 * @return  0 on success, negative fmc_config_load_bin() error otherwise.
 ************************************************************************/
int fmc_config_apply_bin(const u8 *data, u32 len)
{
	desInterface *des_target = fmc_pick_des_array(data, len);
	/* ndes_arr (MAX96724 quad) has fewer slots than DS_MAX; cap every
	 * des_target[] index by the picked array's real entry count so the
	 * shorter array is never overrun. */
	u32 des_cap = (des_target == ndes_arr)
			? (u32)MAX96724_CORE_DES_MAP_SIZE : (u32)DS_MAX;
	int rc;
	int i;
	/* MAX9296 deserializers pair on a shared native address (DS_1/DS_2 at 0xD0,
	 * etc.); the loader zeroes slots the .bin omits, but the odd partner must
	 * keep its default so FMC setup remaps it off the shared address, freeing
	 * it for the configured even deserializer. Only MAX9296 needs this - the
	 * GMSL2 FMCs give every deserializer a unique alias and skip unconfigured
	 * slots, so they are left untouched. */
	int restore_partners = (des_target == des_arr) ? 1 : 0;
	u8 saved_actual[DS_MAX];
	u8 saved_alias[DS_MAX];
	u8 saved_pdb[DS_MAX];

	if (restore_partners) {
		for (i = 0; i < DS_MAX; i++) {
			saved_actual[i] = des_target[i].des_actual_addr;
			saved_alias[i]  = des_target[i].des_alias_addr;
			saved_pdb[i]    = des_target[i].Port_DES_index;
		}
	}

	rc = fmc_config_load_bin(data, len,
				 des_target, g_des_info, &g_des_count,
				 g_runtime_core_map, &g_core_count,
				 max9295_instance,
				 g_pipe_info, &g_sensor_count);
	if (rc < 0) {
		TRACE(FMC_CFG_ERROR, "%s: fmc_config_load_bin failed (rc=%d)\r\n",
		      __func__, rc);
		return rc;
	}

	/* Restore compile-time defaults for MAX9296 slots the .bin did not
	 * configure (address 0), so each pair partner is remapped off 0xD0/0x94. */
	if (restore_partners) {
		for (i = 0; i < DS_MAX; i++) {
			if (des_target[i].des_actual_addr == 0U) {
				des_target[i].des_actual_addr = saved_actual[i];
				des_target[i].des_alias_addr  = saved_alias[i];
				des_target[i].Port_DES_index  = saved_pdb[i];
				des_target[i].des_state       = in_deinit;
				des_target[i].link_type       = NO_LINK;
			}
		}
	}

	/* Per-DES I2C bus map, indexed by the physical DES slot (des_no) so
	 * GetI2cBusIdForDes(desId) lines up with active_des_array[desId]. */
	for (i = 0; i < DS_MAX; i++)
		g_des_bus[i] = INVALID_I2C_BUS_ID;
	for (i = 0; i < g_des_count; i++) {
		u8 dno = g_des_info[i].des_no;
		if (dno < DS_MAX)
			g_des_bus[dno] = g_des_info[i].i2c_bus;
	}

	/* I2C controller (interrupt / base) table for the HAL. */
	rc = fmc_config_load_i2c_buses(data, len, g_i2c_bus, &g_bus_count);
	if (rc < 0) {
		TRACE(FMC_CFG_ERROR,
		      "%s: fmc_config_load_i2c_buses failed (rc=%d)\r\n",
		      __func__, rc);
		return rc;
	}

	g_loaded = 1;
	g_fmc_id = fmc_config_get_fmc_id(data, len);
	TRACE(FMC_CFG_ALWAYS,
	      "%s: applied .bin - fmc_id=%d %d DES, %d sensors, %d cores, %d I2C buses\r\n",
	      __func__, g_fmc_id, g_des_count, g_sensor_count, g_core_count, g_bus_count);

	/* Dump everything fetched from the .bin so the read can be verified
	 * against what the generator wrote (every field, same order/units). */
	for (i = 0; i < g_des_count; i++) {
		u8 dno = g_des_info[i].des_no;

		if (dno >= des_cap)
			continue;

		TRACE(FMC_CFG_ALWAYS,
		      "  DES[%d] slot=%u id=%u en=%u chip=%s core=%u i2c_bus=%u actual=0x%02X alias=0x%02X link_type=%u\r\n",
		      i, (unsigned)dno, (unsigned)g_des_info[i].id, (unsigned)g_des_info[i].enabled,
		      g_des_info[i].chip, (unsigned)g_des_info[i].core_id,
		      (unsigned)g_des_info[i].i2c_bus,
		      (unsigned)des_target[dno].des_actual_addr,
		      (unsigned)des_target[dno].des_alias_addr,
		      (unsigned)des_target[dno].link_type);
		TRACE(FMC_CFG_ALWAYS,
		      "         name=%s lane_rate=%u lane_count=%u des_no=%u pdb=%u link_a=%u link_b=%u\r\n",
		      g_des_info[i].alias, (unsigned)g_des_info[i].lane_rate,
		      (unsigned)g_des_info[i].lane_count, (unsigned)g_des_info[i].des_no,
		      (unsigned)g_des_info[i].pdb_gpio,
		      (unsigned)g_des_info[i].link_a, (unsigned)g_des_info[i].link_b);
		TRACE(FMC_CFG_ALWAYS,
		      "         linkA ser=0x%02X/0x%02X sen=0x%02X/0x%02X  linkB ser=0x%02X/0x%02X sen=0x%02X/0x%02X\r\n",
		      (unsigned)des_target[dno].link_a.serializer_default_addr,
		      (unsigned)des_target[dno].link_a.serializer_alias_addr,
		      (unsigned)des_target[dno].link_a.sensor_default_addr,
		      (unsigned)des_target[dno].link_a.sensor_alias_addr,
		      (unsigned)des_target[dno].link_b.serializer_default_addr,
		      (unsigned)des_target[dno].link_b.serializer_alias_addr,
		      (unsigned)des_target[dno].link_b.sensor_default_addr,
		      (unsigned)des_target[dno].link_b.sensor_alias_addr);
	}
	for (i = 0; i < g_bus_count; i++) {
		TRACE(FMC_CFG_ALWAYS,
		      "  BUS[%d] id=%u mode=%s intr=%u base=0x%08X high=0x%08X clock=%uKHz\r\n",
		      i, (unsigned)g_i2c_bus[i].bus_id,
		      (g_i2c_bus[i].mode == FMC_I2C_MODE_PS) ? "PS" : "AXI",
		      (unsigned)g_i2c_bus[i].intr_id,
		      (unsigned)g_i2c_bus[i].base_addr,
		      (unsigned)g_i2c_bus[i].high_addr,
		      (unsigned)g_i2c_bus[i].clock_khz);
	}
	for (i = 0; i < g_sensor_count; i++) {
		u8 pn = g_pipe_info[i].pipe_no;
		int ser_base  = (pn < FMC_MAX_PIPES) ? max9295_instance[pn].i2c_addr   : 0;
		int ser_alias = (pn < FMC_MAX_PIPES) ? max9295_instance[pn].alias_addr : 0;

		TRACE(FMC_CFG_ALWAYS,
		      "  PIPE[%d] pipe=%u en=%u lane=%u des_link=%u model=%s ser=%s\r\n",
		      i, (unsigned)pn, (unsigned)g_pipe_info[i].enabled,
		      (unsigned)g_pipe_info[i].link_lane, (unsigned)g_pipe_info[i].des_link,
		      g_pipe_info[i].sensor_model, g_pipe_info[i].serializer_model);
		TRACE(FMC_CFG_ALWAYS,
		      "         driverID=0x%08X sen_alias=0x%02X ser=0x%02X/0x%02X bus=%u\r\n",
		      (unsigned)g_pipe_info[i].driver_id,
		      (unsigned)g_pipe_info[i].sensor_alias,
		      (unsigned)ser_base, (unsigned)ser_alias,
		      (unsigned)g_pipe_info[i].bus_num);
	}
	for (i = 0; i < g_core_count; i++) {
		TRACE(FMC_CFG_ALWAYS,
		      "  CORE[%d] core_id=%u num_des=%u\r\n",
		      i, (unsigned)g_runtime_core_map[i].core_id,
		      (unsigned)g_runtime_core_map[i].num_deserializers);
	}
	return 0;
}

/**************************************************************************
 *          fmc_config_set_source
 *
 * @brief   Override the address/length used by the address-based loader.
 ************************************************************************/
void fmc_config_set_source(const u8 *data, u32 len)
{
	g_src_ptr = data;
	g_src_len = len;
}

/**************************************************************************
 *          fmc_config_apply_from_memory
 *
 * @brief   Apply the .bin found at the configured memory address.
 *
 * @return  0 on success, negative on parse error, +1 if no source configured.
 ************************************************************************/
int fmc_config_apply_from_memory(void)
{
	u32 len;

	TRACE(FMC_CFG_ALWAYS, "%s: enter - source addr=0x%08X len=%u\r\n",
	      __func__, (unsigned)(uintptr_t)g_src_ptr, (unsigned)g_src_len);

	if (g_src_ptr == NULL) {
		TRACE(FMC_CFG_WARNING,
		      "%s: no source configured (FMC_CONFIG_BIN_BASE_ADDR=0)\r\n",
		      __func__);
		return 1;
	}

	len = g_src_len;
	if (len == 0U)
		len = fmc_bin_len_from_header(g_src_ptr, FMC_CONFIG_BIN_MAX_LEN);
	if (len == 0U) {
		TRACE(FMC_CFG_WARNING,
		      "%s: no valid .bin at 0x%08X\r\n",
		      __func__, (unsigned)(uintptr_t)g_src_ptr);
		return 1;
	}

	TRACE(FMC_CFG_ALWAYS, "%s: parsing .bin at 0x%08X len=%u\r\n",
	      __func__, (unsigned)(uintptr_t)g_src_ptr, (unsigned)len);
	return fmc_config_apply_bin(g_src_ptr, len);
}

/**************************************************************************
 *          fmc_config_ensure_loaded
 *
 * @brief   Apply the configured source exactly once (idempotent).
 *
 * @return  0 if loaded, negative on parse error, +1 if no source configured.
 ************************************************************************/
int fmc_config_ensure_loaded(void)
{
	if (g_loaded)
		return 0;
	return fmc_config_apply_from_memory();
}

/**************************************************************************
 *          Accessors
 ************************************************************************/
int fmc_config_is_loaded(void)
{
	return g_loaded;
}

const fmc_pipe_info_t *fmc_config_get_pipe(int pipe_no)
{
	int i;

	if (!g_loaded)
		return NULL;
	for (i = 0; i < g_sensor_count; i++) {
		if ((int)g_pipe_info[i].pipe_no == pipe_no)
			return &g_pipe_info[i];
	}
	return NULL;
}

const fmc_des_info_t *fmc_config_get_des_info(int des_index)
{
	if (!g_loaded || (des_index < 0) || (des_index >= g_des_count))
		return NULL;
	return &g_des_info[des_index];
}

u8 fmc_config_get_des_bus(u8 des_index)
{
	if (!g_loaded || (des_index >= (u8)DS_MAX))
		return INVALID_I2C_BUS_ID;
	return g_des_bus[des_index];
}

const core_des_mapping_t *fmc_config_get_core_map(int *count_out)
{
	if (count_out != NULL)
		*count_out = g_loaded ? g_core_count : 0;
	if (!g_loaded)
		return NULL;
	return g_runtime_core_map;
}

int fmc_config_get_des_count(void)
{
	return g_loaded ? g_des_count : 0;
}

int fmc_config_get_sensor_count(void)
{
	return g_loaded ? g_sensor_count : 0;
}

/* FMC selection id parsed from the loaded .bin (0=MAX9296, 1=MAX96716,
 * 2=MAX96724, 3=MAX96726); -1 when no .bin has been applied. */
int fmc_config_get_active_fmc_id(void)
{
	return g_loaded ? g_fmc_id : -1;
}

/* Serializer model from the .bin, taken from the first pipe whose
 * serializer_model names a known part; -1 when unloaded/unknown. */
int fmc_config_get_serializer_id(void)
{
	int i;

	if (!g_loaded)
		return -1;
	for (i = 0; i < g_sensor_count; i++) {
		const char *model = g_pipe_info[i].serializer_model;

		if (strstr(model, "96717") != NULL)
			return SER_ID_MAX96717;
		if (strstr(model, "9295") != NULL)
			return SER_ID_MAX9295;
	}
	return -1;
}

/* I2C controller base address for a bus id from the loaded .bin, or 0 when the
 * bin has no entry for it (caller then uses its compile-time default). */
u32 fmc_config_get_i2c_base(u8 bus_id)
{
	int i;

	if (!g_loaded)
		return 0U;
	for (i = 0; i < g_bus_count; i++) {
		if (g_i2c_bus[i].valid && (g_i2c_bus[i].bus_id == bus_id))
			return g_i2c_bus[i].base_addr;
	}
	return 0U;
}

/**************************************************************************
 *          HalI2cGetBusConfig
 *
 * @brief   Return the runtime config (base address + GIC interrupt id) for an
 *          I2C bus. Prefers the .bin record (valid=1). Without one, AXI buses
 *          get only their default base address (valid=0) - the key the HAL
 *          uses to fetch the rest from the BSP XIic_ConfigTable[].
 *
 * @param   bus_id   I2C bus id (0 = PS, 1..N = AXI instance)
 *
 * @return  Pointer to a static, read-only config record (never NULL).
 ************************************************************************/
const HalI2cBusConfig_t *HalI2cGetBusConfig(uint8_t bus_id)
{
	static HalI2cBusConfig_t cfg;
	int i;

	(void)memset(&cfg, 0, sizeof(cfg));
	cfg.bus_id = bus_id;

	/* 1. Prefer a controller record parsed from the .bin. */
	if (g_loaded) {
		for (i = 0; i < g_bus_count; i++) {
			if (g_i2c_bus[i].valid && (g_i2c_bus[i].bus_id == bus_id)) {
				cfg.mode      = g_i2c_bus[i].mode;
				cfg.intr_id   = g_i2c_bus[i].intr_id;
				cfg.base_addr = g_i2c_bus[i].base_addr;
				cfg.high_addr = g_i2c_bus[i].high_addr;
				cfg.clock_khz = g_i2c_bus[i].clock_khz;
				cfg.valid     = 1U;
				return &cfg;
			}
		}
	}

	/* 2. No .bin record: default AXI base only (BSP lookup key). */
	switch (bus_id) {
	case 1U:
		cfg.mode      = FMC_I2C_MODE_AXI;
		cfg.base_addr = (uint32_t)AXI1_FMC_I2C_BASEADDRESS;
		break;
	case 2U:
		cfg.mode      = FMC_I2C_MODE_AXI;
		cfg.base_addr = (uint32_t)AXI2_FMC_I2C_BASEADDRESS;
		break;
	case 3U:
		cfg.mode      = FMC_I2C_MODE_AXI;
		cfg.base_addr = (uint32_t)AXI3_FMC_I2C_BASEADDRESS;
		break;
	default:
		/* PS bus (0) / unknown: leave zeroed, valid stays 0. */
		break;
	}

	return &cfg;
}
