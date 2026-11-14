/**
 * fmc_config_bin.h — Consolidated FMC Binary Config Loader for sensor_lib
 *
 * Drop this header into sensor_lib and call fmc_config_load_bin() to parse
 * the .bin file produced by the FMC Config Generator. The function directly
 * populates sensor_lib's native structures:
 *
 *   - desInterface[]          (DES addresses, PDB GPIO, link addresses)
 *   - serializer_driver[]     (bus_num, i2c_addr, alias_addr, link_lane)
 *   - core_des_mapping_t[]    (core-to-DES-to-I2C-bus mapping)
 *   - fmc_pipe_info_t[]       (per-pipe sensor model, driver_id, alias)
 *
 * Prerequisites:
 *   #include "isi_fmc.h"   // desInterface, dslink, core_des_mapping_t,
 *                          // serializer_driver, transition_state,
 *                          // DS_MAX, LINK_A, LINK_B, NO_LINK, etc.
 *
 * Usage:
 *   Within sensor_lib, prefer the high-level front-end in fmc_config.h
 *   (fmc_config_apply_bin() / fmc_config_apply_from_memory()), which calls
 *   the two parsers below and populates des_arr[]/max96716_des_arr[],
 *   max9295_instance[], the runtime core map and the I2C bus table.
 *
 *   Direct use of the low-level parsers:
 *
 *   extern desInterface            des_arr[];
 *   extern struct serializer_driver max9295_instance[];
 *
 *   fmc_des_info_t     des_info[DS_MAX];
 *   core_des_mapping_t core_map[DS_MAX];
 *   fmc_pipe_info_t    pipes[FMC_MAX_PIPES];
 *   fmc_i2c_bus_info_t buses[FMC_MAX_I2C_BUS];
 *   int des_cnt, sensor_cnt, core_cnt, bus_cnt;
 *
 *   const u8 *bin = <read .bin from flash / SD / memory>;
 *   u32 bin_len   = <size in bytes>;
 *
 *   int rc = fmc_config_load_bin(bin, bin_len,
 *                                des_arr, des_info, &des_cnt,
 *                                core_map,          &core_cnt,
 *                                max9295_instance,
 *                                pipes,             &sensor_cnt);
 *   rc |= fmc_config_load_i2c_buses(bin, bin_len, buses, &bus_cnt);
 *   if (rc < 0) { // handle error }
 *
 * Binary layout (little-endian):
 *   HEADER (32 B) -> DES_ENTRY[] (32 B each) -> SENSOR_ENTRY[] (64 B each)
 *                 -> I2C_ENTRY[] (16 B each) -> BUS_ENTRY[] (16 B each, v2+)
 *                 -> CRC-32 (4 B, v3+)
 *
 * HEADER (32 bytes):
 *   [0-3]   magic "FMCB"       [4-5]   version (1, 2 or 3)
 *   [6-7]   des_cnt (u16)      [8-9]   sen_cnt (u16)
 *   [10-11] i2c_cnt (u16)      [12-15] des_off (u32)
 *   [16-19] sen_off (u32)      [20-23] i2c_off (u32)
 *   [24-25] bus_cnt (u16, v2)  [26] fmc_id (u8, v3) [27] reserved
 *   [28-31] bus_off (u32, v2)
 *
 *   fmc_id selects the FMC board / DES family so the loader can pick the
 *   matching runtime array: 0=MAX9296, 1=MAX96716, 2=MAX96724, 3=MAX96726.
 *   The DES chip name (DES_ENTRY [24-31]) stays authoritative; fmc_id is a
 *   fast, explicit hint (and a fallback when the chip field is blank).
 *
 * CRC-32 TRAILER (4 bytes, v3+ - integrity check appended at end of file):
 *   Little-endian CRC-32 (IEEE 802.3, poly 0xEDB88320, init/final 0xFFFFFFFF)
 *   computed over every preceding byte [0 .. file_len-4). v1/v2 bins have no
 *   trailer and are still accepted (the loader skips the CRC check for them).
 *
 * BUS_ENTRY (16 bytes, v2 - one per I2C controller extracted from the XSA):
 *   [0]     bus_id (u8: 0=PS, 1..N = AXI instance)
 *   [1]     mode (u8: 0=AXI I2C, 1=PS I2C)
 *   [2-3]   interrupt_id (u16 LE, GIC IRQ ID)
 *   [4-7]   base_addr (u32 LE)
 *   [8-11]  high_addr (u32 LE)
 *   [12-15] clock_khz (u32 LE)
 *
 * DES_ENTRY (32 bytes):
 *   [0]     id (u8)           [1]     enabled (u8)
 *   [2]     core_id (u8)      [3]     i2c_bus (u8)
 *   [4-5]   default_addr (u16 LE)
 *   [6-7]   alias_addr (u16 LE)
 *   [8]     pdb_gpio (u8)     [9]     lane_rate (u8: 0=none, 1=3G, 2=6G)
 *   [10]    link_a (u8)       [11]    link_b (u8)
 *   [12-19] alias (char[8])   [20]    lane_count (u8)
 *   [21]    des_no (u8)       [22-23] reserved
 *   [24-31] chip (char[8])
 *
 * SENSOR_ENTRY (64 bytes):
 *   [0]     id (u8)           [1]     enabled (u8)
 *   [2]     pipe_no (u8)      [3]     link_lane (u8: 1=A, 2=B)
 *   [4-5]   sensor_base_addr (u16 LE)
 *   [6-7]   sensor_alias_addr (u16 LE)
 *   [8-9]   ser_base_addr (u16 LE)
 *   [10-11] ser_alias_addr (u16 LE)
 *   [12-15] driver_id (u32 LE)
 *   [16]    bus_num (u8)      [17-22] reserved
 *   [23]    des_link (u8, 0xFF = none)
 *   [24-35] sensor_model (char[12])
 *   [36-47] serializer_chip (char[12])
 *   [48-63] reserved
 */

#ifndef FMC_CONFIG_BIN_H
#define FMC_CONFIG_BIN_H

#include <string.h>

/* ── Binary format constants ───────────────────────────── */

#define FMC_BIN_MAGIC       0x464D4342  /* ASCII "FMCB" */
#define FMC_BIN_VERSION     3   /* v3 appends a CRC-32 integrity trailer     */
#define FMC_BIN_VERSION_MIN 1   /* v1 bins stay loadable (no bus section)    */
#define FMC_BIN_VERSION_CRC 3   /* first version carrying the CRC-32 trailer */

#define FMC_HEADER_SIZE     32
#define FMC_DES_ENTRY_SIZE  32
#define FMC_SEN_ENTRY_SIZE  64
#define FMC_I2C_ENTRY_SIZE  16
#define FMC_BUS_ENTRY_SIZE  16
#define FMC_CRC_TRAILER_SIZE 4  /* v3+ little-endian CRC-32 appended at EOF   */

#define FMC_MAX_PIPES       14  /* matches MAX_SER_COUNT / MAX_SENSOR_COUNT */
#define FMC_MAX_I2C_BUS     8   /* PS + AXI I2C controllers carried in bin   */

/* I2C controller modes carried in the bus section */
#define FMC_I2C_MODE_AXI    0
#define FMC_I2C_MODE_PS     1

/* FMC selection id (header byte [26]) — picks the DES board / family */
#define FMC_ID_MAX9296      0
#define FMC_ID_MAX96716     1
#define FMC_ID_MAX96724     2
#define FMC_ID_MAX96726     3

/*
 * Dual-link value (both Link A and Link B carry a sensor). sensor_lib models
 * this as LINK_REVERSE_SPLITTER; a standalone build may define LINK_DUAL.
 * Resolve to whichever the including project provides so this header compiles
 * unchanged in either environment.
 */
#if !defined(FMC_LINK_DUAL)
#  if defined(LINK_REVERSE_SPLITTER)
#    define FMC_LINK_DUAL LINK_REVERSE_SPLITTER
#  elif defined(LINK_DUAL)
#    define FMC_LINK_DUAL LINK_DUAL
#  else
#    define FMC_LINK_DUAL 0x03
#  endif
#endif

/* ── Per-pipe sensor info (returned to caller) ─────────── */

typedef struct {
    u8   pipe_no;           /* IN_PIPE_0 .. IN_PIPE_12             */
    u8   enabled;           /* 1 = active                          */
    u8   des_link;          /* global link index (0xFF = none)     */
    u8   link_lane;         /* LINK_A (0x01) or LINK_B (0x02)     */
    u32  driver_id;         /* sensor driver ID from config tool   */
    u8   sensor_alias;      /* sensor I2C alias address            */
    u8   bus_num;           /* I2C bus number for this pipe        */
    char sensor_model[13];  /* e.g. "IMX623", "OX03F10"           */
    char serializer_model[13]; /* e.g. "MAX9295", "MAX96717"       */
} fmc_pipe_info_t;

/* ── Per-DES extended info (returned alongside desInterface) ── */

typedef struct {
    u8   id;                /* 1-based DES index from bin          */
    u8   enabled;           /* 1 = active                          */
    u8   core_id;           /* RPU core ID (6, 7, 8, 9)           */
    u8   i2c_bus;           /* I2C bus (0 = PS, 1–3 = AXI)        */
    u8   pdb_gpio;          /* Power-down GPIO index               */
    u8   lane_rate;         /* 0=none, 1=3Gbps, 2=6Gbps           */
    u8   lane_count;        /* 0, 1, 2, or 4                      */
    u8   des_no;            /* DES number (0–5 = DS_ONE..DS_SIX)   */
    u8   link_a;            /* Global link index for Link A        */
    u8   link_b;            /* Global link index for Link B        */
    char alias[9];          /* Null-terminated name e.g. "DS_FIVE" */
    char chip[9];           /* Chip name e.g. "MAX9296"            */
} fmc_des_info_t;

/* -- Per-I2C-controller info (bus section, v2 - sourced from the XSA) -- */

typedef struct {
    u8  bus_id;             /* 0 = PS I2C, 1..N = AXI I2C instance   */
    u8  mode;               /* FMC_I2C_MODE_AXI / FMC_I2C_MODE_PS    */
    u16 intr_id;            /* GIC interrupt ID from the handoff     */
    u32 base_addr;          /* controller base address               */
    u32 high_addr;          /* controller high address               */
    u32 clock_khz;          /* controller input clock (kHz)          */
    u8  valid;              /* 1 = populated from the bin            */
} fmc_i2c_bus_info_t;

/* ── Little-endian read helpers ────────────────────────── */

static inline u16 fmc_rd16(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static inline u32 fmc_rd32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8)
         | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}
/* ── Overflow-safe section-bounds check ───────────── */
/*  Returns 1 if the byte span [off, off + count*size) lies fully within
 *  data_len, evaluated without unsigned wrap-around. Rejects malformed or
 *  hostile headers before any section pointer is dereferenced
 *  (MISRA C:2012 Dir 4.1 — minimise run-time failures; guards CWE-125). */
static inline int fmc_span_ok(u32 off, u32 count, u32 size, u32 data_len)
{
    if (off > data_len)
        return 0;
    if ((count != 0U) && (size != 0U) &&
        (count > ((data_len - off) / size)))
        return 0;
    return 1;
}
/* ── CRC-32 (IEEE 802.3) over a byte span ─────────────── */
/*  Bit-reflected CRC-32, polynomial 0xEDB88320, init/final XOR 0xFFFFFFFF —
 *  the same algorithm used by zlib/PNG/ZIP. Table-free bit-serial form keeps
 *  the header self-contained (no static table, MISRA C:2012 friendly). */
static inline u32 fmc_crc32(const u8 *data, u32 len)
{
    u32 crc = 0xFFFFFFFFU;
    u32 i;
    int bit;

    for (i = 0U; i < len; i++) {
        crc ^= (u32)data[i];
        for (bit = 0; bit < 8; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc = (crc >> 1);
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}
/* ── v3 CRC-32 trailer verification ───────────────────── */
/*
 * fmc_config_verify_crc — Validate the v3 integrity trailer.
 *
 * v3+ bins append a little-endian CRC-32 of all preceding bytes. v1/v2 bins
 * have no trailer, so this returns success for them (nothing to check).
 *
 * @param data      Raw binary data (exact file length in data_len)
 * @param data_len  Size of binary data in bytes
 *
 * @return  0 = OK (valid CRC, or a pre-v3 bin), negative on error:
 *          -1 = NULL pointer argument
 *          -2 = data too small for header (or missing trailer)
 *          -3 = bad magic ("FMCB" expected)
 *          -8 = CRC mismatch (corrupt / truncated binary)
 */
static inline int fmc_config_verify_crc(const u8 *data, u32 data_len)
{
    u16 ver;
    u32 payload_len, stored, computed;

    if (!data)
        return -1;
    if (data_len < FMC_HEADER_SIZE)
        return -2;

    {
        u32 magic = ((u32)data[0] << 24) | ((u32)data[1] << 16)
                  | ((u32)data[2] << 8)  |  (u32)data[3];
        if (magic != FMC_BIN_MAGIC)
            return -3;
    }

    ver = fmc_rd16(&data[4]);
    if (ver < FMC_BIN_VERSION_CRC)
        return 0;   /* pre-v3: no CRC trailer present */

    if (data_len < (FMC_HEADER_SIZE + FMC_CRC_TRAILER_SIZE))
        return -2;

    payload_len = data_len - FMC_CRC_TRAILER_SIZE;
    stored      = fmc_rd32(&data[payload_len]);
    computed    = fmc_crc32(data, payload_len);

    return (stored == computed) ? 0 : -8;
}
/* ── FMC selection id (header byte [26]) ──────────────── */
/*
 * fmc_config_get_fmc_id — Return the FMC board id carried by the .bin.
 *
 * The DES chip name in the first DES entry is authoritative (written in every
 * bin version); the explicit header byte [26] is only a fallback for a bin
 * whose chip field is blank. Mapping matches the FMC_ID_* constants above:
 *   0 = MAX9296, 1 = MAX96716, 2 = MAX96724, 3 = MAX96726.
 *
 * @param data      Raw binary data
 * @param data_len  Size of binary data in bytes
 *
 * @return  FMC id (0..3), or -1 on a NULL / too-short / bad-magic buffer.
 */
static inline int fmc_config_get_fmc_id(const u8 *data, u32 data_len)
{
    u16 des_cnt;
    u32 des_off;

    if (!data || data_len < FMC_HEADER_SIZE)
        return -1;

    {
        u32 magic = ((u32)data[0] << 24) | ((u32)data[1] << 16)
                  | ((u32)data[2] << 8)  |  (u32)data[3];
        if (magic != FMC_BIN_MAGIC)
            return -1;
    }

    des_cnt = fmc_rd16(&data[6]);
    des_off = fmc_rd32(&data[12]);
    if ((des_cnt > 0U) &&
        fmc_span_ok(des_off, des_cnt, FMC_DES_ENTRY_SIZE, data_len)) {
        const char *chip = (const char *)&data[des_off + 24U];

        if (strncmp(chip, "MAX96716", 8) == 0) return FMC_ID_MAX96716;
        if (strncmp(chip, "MAX96724", 8) == 0) return FMC_ID_MAX96724;
        if (strncmp(chip, "MAX96726", 8) == 0) return FMC_ID_MAX96726;
        if (strncmp(chip, "MAX9296",  7) == 0) return FMC_ID_MAX9296;
    }

    /* Chip field blank/unknown: trust the explicit id byte. */
    return (int)data[26];
}
/* ── Consolidated loader ───────────────────────────────── */

/**
 * fmc_config_load_bin — Parse FMC binary and populate sensor_lib structures.
 *
 * @param data             Raw binary data (from flash, SD, memory-mapped, etc.)
 * @param data_len         Size of binary data in bytes
 * @param des_out          desInterface array to populate          [DS_MAX]
 * @param des_info_out     fmc_des_info_t array (extended DES metadata)  [DS_MAX]
 * @param des_count_out    Output: number of deserializer entries
 * @param core_map_out     core_des_mapping_t array to populate    [DS_MAX]
 * @param core_count_out   Output: number of unique core mappings
 * @param ser_out          serializer_driver array to update (existing static
 *                         instances — only address/bus/enable fields are written;
 *                         function pointers and init_array are left untouched)
 * @param pipe_info_out    Per-pipe sensor info array              [FMC_MAX_PIPES]
 * @param sensor_count_out Output: number of sensor/pipe entries
 *
 * @return  0 on success, negative on error:
 *          -1 = NULL pointer argument
 *          -2 = data too small for header
 *          -3 = bad magic ("FMCB" expected)
 *          -4 = unsupported version
 *          -5 = DES count exceeds DS_MAX
 *          -6 = sensor count exceeds FMC_MAX_PIPES
 *          -7 = data truncated (smaller than declared entries)
 *          -8 = CRC mismatch (v3+ integrity trailer failed)
 */
static inline int fmc_config_load_bin(
    const u8 *data, u32 data_len,
    desInterface *des_out,          fmc_des_info_t *des_info_out,
    int *des_count_out,
    core_des_mapping_t *core_map_out, int *core_count_out,
    struct serializer_driver *ser_out,
    fmc_pipe_info_t *pipe_info_out, int *sensor_count_out)
{
    u32 des_off, sen_off, i2c_off;
    u16 des_cnt, sen_cnt, i2c_cnt;
    int i, j, core_idx;
    const u8 *p;
    u8 link_used[DS_MAX][2];  /* [des_no][0]=linkA active, [1]=linkB active */
    u8 parse_to_dno[DS_MAX];  /* parse index -> physical DES slot (des_no)  */

    /* ── Validate arguments ── */
    if (!data || !des_out || !des_info_out || !core_map_out || !ser_out ||
        !pipe_info_out || !des_count_out || !core_count_out || !sensor_count_out)
        return -1;

    if (data_len < FMC_HEADER_SIZE)
        return -2;

    /* ── Parse header ── */
    {
        u32 magic = ((u32)data[0] << 24) | ((u32)data[1] << 16)
                  | ((u32)data[2] << 8)  |  (u32)data[3];
        if (magic != FMC_BIN_MAGIC)
            return -3;
    }

    {
        u16 ver = fmc_rd16(&data[4]);
        if ((ver < FMC_BIN_VERSION_MIN) || (ver > FMC_BIN_VERSION))
            return -4;
    }

    /* Reject a corrupt/truncated binary before parsing (v3+ CRC trailer). */
    if (fmc_config_verify_crc(data, data_len) < 0)
        return -8;

    des_cnt = fmc_rd16(&data[6]);
    sen_cnt = fmc_rd16(&data[8]);
    i2c_cnt = fmc_rd16(&data[10]);
    des_off = fmc_rd32(&data[12]);
    sen_off = fmc_rd32(&data[16]);
    i2c_off = fmc_rd32(&data[20]);

    if (des_cnt > DS_MAX)
        return -5;
    if (sen_cnt > FMC_MAX_PIPES)
        return -6;

    /* Validate every section fits inside the buffer (overflow-safe) so no
     * subsequent read can run past the end of `data`, even if the header
     * offsets/counts are corrupt or crafted. */
    if (!fmc_span_ok(des_off, des_cnt, FMC_DES_ENTRY_SIZE, data_len) ||
        !fmc_span_ok(sen_off, sen_cnt, FMC_SEN_ENTRY_SIZE, data_len) ||
        !fmc_span_ok(i2c_off, i2c_cnt, FMC_I2C_ENTRY_SIZE, data_len))
        return -7;

    /* ──────────────────────────────────────────────────── */
    /*  1. Populate desInterface + fmc_des_info_t arrays    */
    /* ──────────────────────────────────────────────────── */
    (void)memset(des_out,      0, sizeof(desInterface) * DS_MAX);
    (void)memset(des_info_out, 0, sizeof(fmc_des_info_t) * DS_MAX);
    (void)memset(link_used,    0, sizeof(link_used));

    for (i = 0; i < (int)des_cnt; i++) {
        int dno;

        p = &data[des_off + (u32)i * FMC_DES_ENTRY_SIZE];

        /* Physical DES slot the firmware indexes by (DS_ONE..DS_SIX). The
         * runtime addresses des_arr[] and the per-DES I2C bus by this slot
         * (MAPPING_INPIPE_TO_DES_ID(pipe) = pipe/2), NOT by parse order, so a
         * board that populates only e.g. DS_ONE/DS_TWO/DS_SIX must land at
         * slots 0/1/5. Fall back to parse order if des_no is out of range. */
        dno = p[21];
        if (dno >= DS_MAX)
            dno = i;
        parse_to_dno[i] = (u8)dno;

        /* -- desInterface fields (placed at the physical slot) -- */
        des_out[dno].des_actual_addr = (u8)fmc_rd16(&p[4]);
        des_out[dno].des_alias_addr  = (u8)fmc_rd16(&p[6]);
        des_out[dno].Port_DES_index  = (u8)dno;      /* physical DES id */
        des_out[dno].des_state       = in_deinit;
        des_out[dno].link_type       = NO_LINK;      /* updated below */

        /* -- fmc_des_info_t extended fields (kept in parse order) -- */
        des_info_out[i].id         = p[0];
        des_info_out[i].enabled    = p[1];
        des_info_out[i].core_id    = p[2];
        des_info_out[i].i2c_bus    = p[3];
        des_info_out[i].pdb_gpio   = p[8];
        des_info_out[i].lane_rate  = p[9];           /* 0=none, 1=3Gbps, 2=6Gbps */
        des_info_out[i].link_a     = p[10];
        des_info_out[i].link_b     = p[11];
        (void)memcpy(des_info_out[i].alias, &p[12], 8);
        des_info_out[i].alias[8]   = '\0';
        des_info_out[i].lane_count = p[20];
        des_info_out[i].des_no     = p[21];
        (void)memcpy(des_info_out[i].chip, &p[24], 8);
        des_info_out[i].chip[8]    = '\0';
    }

    /* ──────────────────────────────────────────────────── */
    /*  2. Process sensor entries                           */
    /*     -> fill dslink in desInterface                   */
    /*     -> update serializer_driver fields               */
    /*     -> build pipe_info for caller                    */
    /* ──────────────────────────────────────────────────── */
    (void)memset(pipe_info_out, 0, sizeof(fmc_pipe_info_t) * FMC_MAX_PIPES);

    for (i = 0; i < (int)sen_cnt; i++) {
        u16 ser_base, ser_alias, sen_base, sen_alias;
        u8  pipe_no, lane, des_link, bus;
        u32 drv_id;

        p = &data[sen_off + (u32)i * FMC_SEN_ENTRY_SIZE];

        pipe_no   = p[2];
        lane      = p[3];                  /* LINK_A=1 or LINK_B=2 */
        sen_base  = fmc_rd16(&p[4]);
        sen_alias = fmc_rd16(&p[6]);
        ser_base  = fmc_rd16(&p[8]);
        ser_alias = fmc_rd16(&p[10]);
        drv_id    = fmc_rd32(&p[12]);
        bus       = p[16];
        des_link  = p[23];                 /* 0xFF = unassigned */

        /* -- Fill dslink addresses in the parent DES entry (physical slot).
         *    des_link is the tool's parse-order global link index; map it
         *    back to the DES's physical slot so the link lands where the
         *    firmware reads it (des_arr[desId].link_a/link_b). -- */
        if (des_link != 0xFF) {
            int parse_idx = des_link / 2;
            if (parse_idx < (int)des_cnt) {
                u8 dno = parse_to_dno[parse_idx];
                dslink *lk = (des_link % 2 == 0)
                           ? &des_out[dno].link_a
                           : &des_out[dno].link_b;

                lk->serializer_default_addr = (u8)ser_base;
                lk->serializer_alias_addr   = (u8)ser_alias;
                lk->sensor_default_addr     = (u8)sen_base;
                lk->sensor_alias_addr       = (u8)sen_alias;

                link_used[dno][des_link % 2] = 1;
            }
        }

        /* -- Update serializer_driver instance -- */
        if (pipe_no < FMC_MAX_PIPES) {
            ser_out[pipe_no].link_lane  = lane;
            ser_out[pipe_no].i2c_addr   = (int)ser_base;
            ser_out[pipe_no].alias_addr = (int)ser_alias;
            ser_out[pipe_no].bus_num    = (int)bus;
            ser_out[pipe_no].enable     = 1;
            ser_out[pipe_no].ser_state  = in_deinit;
        }

        /* -- Fill per-pipe info for the caller -- */
        pipe_info_out[i].pipe_no      = pipe_no;
        pipe_info_out[i].enabled      = p[1];
        pipe_info_out[i].des_link     = des_link;
        pipe_info_out[i].link_lane    = lane;
        pipe_info_out[i].driver_id    = drv_id;
        pipe_info_out[i].sensor_alias = (u8)sen_alias;
        pipe_info_out[i].bus_num      = bus;
        (void)memcpy(pipe_info_out[i].sensor_model, &p[24], 12);
        pipe_info_out[i].sensor_model[12] = '\0';
        (void)memcpy(pipe_info_out[i].serializer_model, &p[36], 12);
        pipe_info_out[i].serializer_model[12] = '\0';
    }

    /* -- Set desInterface.link_type per physical slot (link_used is indexed
     *    by des_no); slots absent from the bin stay NO_LINK. -- */
    for (i = 0; i < DS_MAX; i++) {
        u8 a = link_used[i][0], b = link_used[i][1];
        if (a && b)       des_out[i].link_type = FMC_LINK_DUAL;
        else if (a)       des_out[i].link_type = LINK_A;
        else if (b)       des_out[i].link_type = LINK_B;
        else              des_out[i].link_type = NO_LINK;
    }

    /* ──────────────────────────────────────────────────── */
    /*  3. Build core_des_mapping_t from DES entries        */
    /*     Groups DES indices by core_id                    */
    /* ──────────────────────────────────────────────────── */
    core_idx = 0;
    (void)memset(core_map_out, 0, sizeof(core_des_mapping_t) * DS_MAX);

    for (i = 0; i < (int)des_cnt; i++) {
        u8 cid, i2c_bus, found, dno;

        /* Reuse the metadata already parsed in step 1 instead of re-reading
         * the raw buffer (one less pass over the DES section). */
        if (!des_info_out[i].enabled) continue;   /* skip disabled DES */

        cid     = des_info_out[i].core_id;        /* RPU core id      */
        i2c_bus = des_info_out[i].i2c_bus;        /* I2C bus          */
        dno     = des_info_out[i].des_no;         /* physical DES slot */

        /* Find or create core_map entry */
        found = 0;
        for (j = 0; j < core_idx; j++) {
            if (core_map_out[j].core_id == cid) {
                u8 n = core_map_out[j].num_deserializers;
                if (n < 3) {
                    core_map_out[j].des_indices[n] = dno;
                    core_map_out[j].i2cBusIds[n]   = i2c_bus;
                    core_map_out[j].num_deserializers = n + 1;
                }
                found = 1;
                break;
            }
        }
        if (!found && core_idx < DS_MAX) {
            core_map_out[core_idx].core_id            = cid;
            core_map_out[core_idx].num_deserializers   = 1;
            core_map_out[core_idx].des_indices[0]      = dno;
            core_map_out[core_idx].i2cBusIds[0]        = i2c_bus;
            core_idx++;
        }
    }

    /* ── Output counts ── */
    *des_count_out    = (int)des_cnt;
    *sensor_count_out = (int)sen_cnt;
    *core_count_out   = core_idx;

    return 0;
}

/**
 * fmc_config_load_i2c_buses — Parse the v2 I2C bus/interrupt section.
 *
 * Populates a table of per-controller records (bus id, mode, GIC interrupt
 * ID, base/high address, input clock) that the HAL uses instead of the
 * compile-time macros. Safe to call on a v1 bin: it reports zero buses so
 * the caller falls back to its built-in defaults.
 *
 * @param data           Raw binary data
 * @param data_len       Size of binary data in bytes
 * @param bus_out        Output table                             [FMC_MAX_I2C_BUS]
 * @param bus_count_out  Output: number of I2C controller entries
 *
 * @return  0 on success (including "no bus section"), negative on error:
 *          -1 = NULL pointer argument
 *          -2 = data too small for header
 *          -3 = bad magic
 *          -4 = unsupported version
 *          -7 = bus section out of bounds
 *          -8 = CRC mismatch (v3+ integrity trailer failed)
 */
static inline int fmc_config_load_i2c_buses(
    const u8 *data, u32 data_len,
    fmc_i2c_bus_info_t *bus_out, int *bus_count_out)
{
    u32 bus_off;
    u16 bus_cnt, ver;
    int i;
    const u8 *p;

    if (!data || !bus_out || !bus_count_out)
        return -1;
    if (data_len < FMC_HEADER_SIZE)
        return -2;

    {
        u32 magic = ((u32)data[0] << 24) | ((u32)data[1] << 16)
                  | ((u32)data[2] << 8)  |  (u32)data[3];
        if (magic != FMC_BIN_MAGIC)
            return -3;
    }

    ver = fmc_rd16(&data[4]);
    if ((ver < FMC_BIN_VERSION_MIN) || (ver > FMC_BIN_VERSION))
        return -4;

    /* Reject a corrupt/truncated binary before parsing (v3+ CRC trailer). */
    if (fmc_config_verify_crc(data, data_len) < 0)
        return -8;

    *bus_count_out = 0;
    (void)memset(bus_out, 0, sizeof(fmc_i2c_bus_info_t) * FMC_MAX_I2C_BUS);

    /* v1 bins have no bus section (bytes [24..31] are zero) */
    bus_cnt = fmc_rd16(&data[24]);
    bus_off = fmc_rd32(&data[28]);
    if ((bus_cnt == 0U) || (bus_off == 0U))
        return 0;

    if (bus_cnt > FMC_MAX_I2C_BUS)
        bus_cnt = FMC_MAX_I2C_BUS;

    if (!fmc_span_ok(bus_off, bus_cnt, FMC_BUS_ENTRY_SIZE, data_len))
        return -7;

    for (i = 0; i < (int)bus_cnt; i++) {
        p = &data[bus_off + (u32)i * FMC_BUS_ENTRY_SIZE];
        bus_out[i].bus_id    = p[0];
        bus_out[i].mode      = p[1];
        bus_out[i].intr_id   = fmc_rd16(&p[2]);
        bus_out[i].base_addr = fmc_rd32(&p[4]);
        bus_out[i].high_addr = fmc_rd32(&p[8]);
        bus_out[i].clock_khz = fmc_rd32(&p[12]);
        bus_out[i].valid     = 1U;
    }

    *bus_count_out = (int)bus_cnt;
    return 0;
}

#endif /* FMC_CONFIG_BIN_H */
