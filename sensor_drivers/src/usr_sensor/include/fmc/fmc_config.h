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
 * fmc_config.h — Runtime FMC configuration front-end for sensor_lib.
 *
 * Instead of relying on the compile-time macros in isi_fmc.h / max9296.h /
 * max9295.h, the sensor_lib pulls every board-specific value (deserializer
 * addresses, serializer/sensor addresses, I2C bus per DES, core mapping, and
 * the I2C controller interrupt IDs / base addresses) from the .bin produced
 * by the FMC Config Generator (which is derived from the Vivado .xsa).
 *
 * Two entry points are provided:
 *
 *   1. Explicit pointer API — the application hands the loader the buffer:
 *
 *        extern const unsigned char my_bin[];
 *        fmc_config_apply_bin(my_bin, my_bin_len);
 *
 *   2. Address API — the loader reads from a fixed memory location. The
 *      default address is FMC_CONFIG_BIN_BASE_ADDR (overridable at build
 *      time with -DFMC_CONFIG_BIN_BASE_ADDR=0x...), or set at runtime with
 *      fmc_config_set_source():
 *
 *        fmc_config_set_source((const u8 *)0x20000000, 4096);
 *        fmc_config_apply_from_memory();
 *
 * The bin is applied ONLY when the application explicitly calls
 * fmc_config_apply_bin() (or fmc_config_apply_from_memory()). The library does
 * NOT read any address on its own, so a mis-set base address can never fault
 * the RPU. Call fmc_config_apply_bin(ptr, len) once, before init_des() / the
 * sensor create, from a context where the buffer is known to be valid (e.g. an
 * APU->RPU mailbox command that passes the bin address after the APU has copied
 * the bin into RPU-accessible memory).
 */

#ifndef FMC_CONFIG_H
#define FMC_CONFIG_H

#include <isi/isi_fmc.h>
#include "fmc_config_bin.h"

/*
 * Default location the address-based loader reads the .bin from. A value of 0
 * means "not configured" (the loader then relies on the pointer set through
 * fmc_config_set_source(), or leaves the compile-time defaults in place).
 */
#ifndef FMC_CONFIG_BIN_BASE_ADDR
#define FMC_CONFIG_BIN_BASE_ADDR   (0U)
#endif

/* Upper bound scanned when only a base address (no explicit length) is known. */
#ifndef FMC_CONFIG_BIN_MAX_LEN
#define FMC_CONFIG_BIN_MAX_LEN     (8192U)
#endif

/*
 * Apply a .bin buffer: parses it and overwrites des_arr[], max9295_instance[],
 * the runtime core map, the per-DES I2C bus table and the I2C controller
 * (interrupt/base) table.
 *
 * @return 0 on success, negative fmc_config_load_bin() error code otherwise.
 */
int fmc_config_apply_bin(const u8 *data, u32 len);

/*
 * Apply the .bin found at the configured memory address (see
 * FMC_CONFIG_BIN_BASE_ADDR / fmc_config_set_source()).
 *
 * @return 0 on success, negative on error, +1 if no source is configured.
 */
int fmc_config_apply_from_memory(void);

/* Override the address/length used by the address-based loader. */
void fmc_config_set_source(const u8 *data, u32 len);

/* Non-zero once a valid .bin has been applied. */
int fmc_config_is_loaded(void);

/*
 * Idempotent helper: applies the configured address source once. It is a no-op
 * after the first successful load and when no source is configured. NOT called
 * automatically anywhere - the application must invoke it (or fmc_config_apply_bin)
 * explicitly, only when the bin buffer is known to be valid.
 *
 * @return 0 if loaded (now or previously), negative on parse error, +1 when
 *         no source is configured (compile-time defaults remain in effect).
 */
int fmc_config_ensure_loaded(void);

/* ── Accessors (valid after a successful load) ── */

/* Per-pipe sensor info, or NULL if pipe_no is out of range / not loaded. */
const fmc_pipe_info_t *fmc_config_get_pipe(int pipe_no);

/* Extended DES metadata, or NULL if des_index is out of range / not loaded. */
const fmc_des_info_t *fmc_config_get_des_info(int des_index);

/* I2C bus id for a DES index, or INVALID_I2C_BUS_ID (0xFF) if unknown. */
u8 fmc_config_get_des_bus(u8 des_index);

/*
 * Runtime core -> DES -> I2C-bus map built from the .bin, or NULL if no .bin
 * has been applied. *count_out receives the number of core entries. Used by
 * the FMC setup to place deserializers on the I2C bus the .bin specifies.
 */
const core_des_mapping_t *fmc_config_get_core_map(int *count_out);

int fmc_config_get_des_count(void);
int fmc_config_get_sensor_count(void);

/*
 * FMC selection id parsed from the loaded .bin (0=MAX9296, 1=MAX96716,
 * 2=MAX96724, 3=MAX96726); -1 when no .bin has been applied.
 */
int fmc_config_get_active_fmc_id(void);

/*
 * Serializer model selected by the loaded .bin, derived from the pipe
 * serializer_model strings: SER_ID_MAX96717 / SER_ID_MAX9295, or -1 when no
 * .bin is loaded or no pipe names a known serializer (caller keeps default).
 */
int fmc_config_get_serializer_id(void);

/* I2C controller base address for a bus id from the loaded .bin, or 0 when the
 * bin has no entry (caller then uses its compile-time default). */
u32 fmc_config_get_i2c_base(u8 bus_id);

#endif /* FMC_CONFIG_H */
