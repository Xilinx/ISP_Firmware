/**************************************************************************
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
 ************************************************************************/

#include "isi_fmc.h"
#include "isi/isi.h"
#include "sensor_drv/sensor_drv.h"
#include <fmc/fmc_config.h>

#define FMC_ID_0    (0)
#define FMC_ID_1    (1)
#define FMC_ID_RAVE (2)		/* RAVE board -> MAX96724 quad deserializer */
#define MAX_FMC_COUNT (3)

#define INVALID_I2C_BUS_ID         (0xFF)
// Deserializer pipe status registers (MAX9296)
#define DES_REG_PIPE_X_STATUS      (0x108)
#define DES_REG_PIPE_Y_STATUS      (0x11a)
#define DES_REG_PIPE_Z_STATUS      (0x12c)
#define DES_REG_PIPE_U_STATUS      (0x13e)
// Serializer pipe status registers (MAX9295)
#define SER_REG_PIPE_X_STATUS      (0x102)
#define SER_REG_PIPE_Y_STATUS      (0x10a)
#define SER_REG_PIPE_Z_STATUS      (0x112)
#define SER_REG_PIPE_U_STATUS      (0x11a)

/* ---- File-wide externs (kept at the top of the file) ---- */
extern SensorDrvConfigList_t sensorCfgList[];
extern RESULT SensorDrvGetSensorNumber(uint16_t *pNumber);

/* Fault reporter word-wrap column count. */
#define FAULT_TEXT_COLS 50

/*
 * FMC FRU EEPROM identification (ANSI/VITA 57.1). The card's product-name
 * string identifies it: MAX96716 => "hp150AB", MAX96726 => "hp178AB",
 * MAX9296 carries no product string (concluded by elimination).
 */
#define FMC_FRU_EEPROM_ADDR   (0x50)
#define FMC_FRU_SCAN_LEN      (256)
#define FMC_FRU_KEY_MAX96716  "hp150AB"
#define FMC_FRU_KEY_MAX96726  "hp178AB"
#define FMC_ID_MAX96726       (2)
#define FMC_ID_MISMATCH       (-2)

/* Number of chip-ID registers a sensor entry can define. */
#define SENSOR_ID_REG_COUNT  3

/* Upper bound on the distinct sensor native addresses ("families") the scan
 * can re-point the serializer translation to. Sized to the max sensors the
 * FMC supports, in case every one sits at a different native address. */
#define SENSOR_ID_FAMILY_MAX SENSOR_MAX

/* Bordered box for the sensor identity/access error reports so they stand
 * out clearly in the console log. */
#define SENSOR_BOX_EDGE \
	"  ============================================================\r\n"
#define SENSOR_BOX_RULE \
	"  ------------------------------------------------------------\r\n"

struct accessIIC accessiic_handle[MAX_SENSOR_COUNT];
struct sensor_driver sensor_handle[MAX_SENSOR_COUNT];

CREATE_TRACER(MS_FW_ALWAYS, "MS_FW_ALWAYS: ", ALWAYS, 1);
CREATE_TRACER(MS_FW_INFO, "MS_FW_INFO: ", INFO, 1);
CREATE_TRACER(MS_FW_WARNING, "MS_FW_WARNING: ", WARNING, 1);
CREATE_TRACER(MS_FW_ERROR, "MS_FW_ERROR: ", ERROR, 1);

int current_active_fmc = FMC_ID_0;
/* Fallback only; the real value is set at runtime from the serializer DEV_ID. */
int current_active_serializer = SER_ID_MAX9295;

static void report_wrapped(const char *label, const char *text);

/**************************************************************************
 *          report_pipeline_fault
 *
 * @brief   Shared, plain-English fault reporter for the whole sensor
 *          library. Prints a clear, boxed message that first states what
 *          went wrong and then tells the operator exactly what to do to
 *          resolve it, instead of leaving only a terse code in the log.
 *          Callers should emit the precise technical detail (addresses,
 *          ids, error codes) in a normal TRACE first, then call this to
 *          give the operator the problem summary and the corrective action.
 *
 * @param   device   Device/sub-system reporting the fault (e.g. "Camera sensor")
 * @param   stage    Stage where the fault occurred (e.g. "sensor identity check")
 * @param   problem  One-line, plain-English description of the problem
 * @param   action   One-line, plain-English corrective action for the operator
 *
 ************************************************************************/
void report_pipeline_fault(const char *device, const char *stage,
	const char *problem, const char *action)
{
	TRACE(MS_FW_ERROR, "\r\n");
	TRACE(MS_FW_ERROR, "  ============================================================\r\n");
	TRACE(MS_FW_ERROR, "   IMAGING PIPELINE HALTED\r\n");
	TRACE(MS_FW_ERROR, "  ------------------------------------------------------------\r\n");
	TRACE(MS_FW_ERROR, "   Device  : %s\r\n", device);
	TRACE(MS_FW_ERROR, "   Stage   : %s\r\n", stage);
	report_wrapped("   Problem : ", problem);
	report_wrapped("   Action  : ", action);
	TRACE(MS_FW_ERROR, "  ============================================================\r\n");
}

/**************************************************************************
 *          report_wrapped
 *
 * @brief   Print one labelled fault field, word-wrapped so no text line
 *          exceeds FAULT_TEXT_COLS (50) characters. Continuation lines are
 *          aligned under the text column. A '\n' in the text forces a new
 *          line, so callers can pass multi-step ("point-wise") actions such
 *          as "- step one\n- step two".
 *
 * @param   label   Fixed-width field prefix, e.g. "   Action  : "
 * @param   text    Field text to wrap
 *
 ************************************************************************/
static void report_wrapped(const char *label, const char *text)
{
	char line[FAULT_TEXT_COLS + 1];
	const char *prefix = label;
	const char *cont = "             ";	/* 13 blanks - aligns under text */
	int n = 0;
	int i = 0;

	while (text[i] != '\0') {
		int start = i;
		int wlen;
		int k;

		while (text[i] != '\0' && text[i] != ' ' && text[i] != '\n')
			i++;
		wlen = i - start;

		if (n > 0 && (n + 1 + wlen) > FAULT_TEXT_COLS) {
			line[n] = '\0';
			TRACE(MS_FW_ERROR, "%s%s\r\n", prefix, line);
			prefix = cont;
			n = 0;
		}

		if (n > 0)
			line[n++] = ' ';

		for (k = 0; k < wlen && n < FAULT_TEXT_COLS; k++)
			line[n++] = text[start + k];

		if (text[i] == '\n') {
			line[n] = '\0';
			TRACE(MS_FW_ERROR, "%s%s\r\n", prefix, line);
			prefix = cont;
			n = 0;
			i++;
		} else if (text[i] == ' ') {
			i++;
		}
	}

	if (n > 0) {
		line[n] = '\0';
		TRACE(MS_FW_ERROR, "%s%s\r\n", prefix, line);
	}
}


/**************************************************************************
 *          get_active_des_array
 *
 * @brief   Get the deserializer array based on active FMC
 *
 * @return  Pointer to des_arr or max96716_des_arr
 *
 ************************************************************************/
desInterface *get_active_des_array(void)
{
	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (current_active_fmc == FMC_ID_0)
		return des_arr;

	if (current_active_fmc == FMC_ID_1)
		return max96716_des_arr;

	if (current_active_fmc == FMC_ID_RAVE)
		return ndes_arr;

	TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, current_active_fmc);
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return NULL;
}

/**************************************************************************
 *          get_fmc_instance
 *
 * @brief   Get FMC instance based on FMC ID
 *
 * @param   fmc_id    FMC identifier
 *
 * @return  Pointer to FMC instance or NULL
 *
 ************************************************************************/
IsiFmc_t *get_fmc_instance(int fmc_id)
{
	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	switch (fmc_id) {
	case FMC_ID_0:
		return &g_fmc_max9296;
	case FMC_ID_1:
		return &g_fmc_max96716;
	case FMC_ID_RAVE:
		return &g_fmc_rave;
	default:
		TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, fmc_id);
		TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
		return NULL;
	}
}

/**************************************************************************
 *          get_active_fmc
 *
 * @brief   Get currently active FMC instance
 *
 * @return  Pointer to active FMC or NULL
 *
 ************************************************************************/
IsiFmc_t *get_active_fmc(void)
{
	IsiFmc_t *fmc;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (current_active_fmc == -1) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected\r\n", __func__);
		return NULL;
	}
	fmc = get_fmc_instance(current_active_fmc);
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return fmc;
}

/**************************************************************************
 *          get_active_core_des_map
 *
 * @brief   Get active core deserializer map
 *
 * @return  Pointer to core_des_mapping_t or NULL
 *
 ************************************************************************/
core_des_mapping_t *get_active_core_des_map(void)
{
	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (current_active_fmc == FMC_ID_0)
		return max9296_core_des_map;

	if (current_active_fmc == FMC_ID_1)
		return max96716_core_des_map;

	if (current_active_fmc == FMC_ID_RAVE)
		return max96724_core_des_map;

	TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, current_active_fmc);
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return NULL;
}

/**************************************************************************
 *          get_active_serializer_array
 *
 * @brief   Get the serializer instance array for the selected serializer
 *          (MAX9295 or MAX96717), so a deserializer FMC is not tied to one
 *          hardcoded serializer.
 *
 * @return  Pointer to the selected serializer_driver array
 *
 ************************************************************************/
struct serializer_driver *get_active_serializer_array(void)
{
	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	switch (current_active_serializer) {
	case SER_ID_MAX9295:
		TRACE(MS_FW_ALWAYS,
			"get_active_serializer_array -> MAX9295 (id=%d)\r\n",
			current_active_serializer);
		return max9295_instance;
	case SER_ID_MAX96717:
		TRACE(MS_FW_ALWAYS,
			"get_active_serializer_array -> MAX96717 (id=%d)\r\n",
			current_active_serializer);
		return max96717_instance;
	default:
		TRACE(MS_FW_WARNING,
			"%s: serializer id %d not available, using MAX9295\r\n",
			__func__, current_active_serializer);
		return max9295_instance;
	}
}

/**************************************************************************
 *          apply_active_serializer
 *
 * @brief   Re-point the FMC's serializer_array at the selected serializer,
 *          replacing the compile-time hardcoded array so the MAX9295/MAX96717
 *          choice is decided at runtime before any serializer is programmed.
 *
 * @param   fmc    Active FMC whose serializer_array is bound
 *
 ************************************************************************/
static void apply_active_serializer(IsiFmc_t *fmc)
{
	struct serializer_driver *ser = get_active_serializer_array();
	int i;

	if (fmc == NULL || ser == NULL)
		return;

	TRACE(MS_FW_ALWAYS,
		"apply_active_serializer: binding serializer id=%d to FMC '%s'\r\n",
		current_active_serializer, fmc->FmcName);

	for (i = 0; i < MAX_SER_COUNT; i++)
		fmc->serializer_array[i] = &ser[i];
}

/**************************************************************************
 *          InitIIC
 *
 * @brief   Initialize PS I2C Instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Initialization successful
 * @retval  Other         Initialization failed
 *
 ************************************************************************/
RESULT InitIIC(void)
{
	RESULT Status = RET_SUCCESS;

	HalI2cConfig_t i2cConfig;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	i2cConfig.i2cBusId = 0;
	i2cConfig.HalI2cMode = HAL_PS_I2C_MODE;
	i2cConfig.hHalI2c = NULL;
	TRACE(MS_FW_INFO, "Inside %s and"
		"before PS INIT and Mode passed: %x\n",
		__func__, i2cConfig.HalI2cMode);

	Status = HalI2cInit(&i2cConfig);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: Failed to"
			"initialize I2C bus %d (error=%d)\n",
			__func__, i2cConfig.i2cBusId, Status);
		return Status;
	}
	TRACE(MS_FW_INFO, "%s: PS I2C bus %d"
		"initialized successfully\n",
		__func__, i2cConfig.i2cBusId);
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return Status;
}

/**************************************************************************
 *          fmc_apply_board_config
 *
 * @brief   Load the FMC .bin (0x1E4E8000, isp_sensor_bin DT node) and make
 *          the FMC / serializer it names active, so every DES / I2C-bus
 *          lookup follows the .bin topology whichever sensor runs first.
 *          Idempotent: after the first load it only re-asserts the selection.
 *
 * @param   retry_load  BOOL_TRUE retries a failed load on every call;
 *                      BOOL_FALSE tries the load only once.
 *
 ************************************************************************/
static void fmc_apply_board_config(bool_t retry_load)
{
	static bool_t load_tried = BOOL_FALSE;
	bool_t just_loaded = BOOL_FALSE;
	int bin_fmc;
	int sel;
	int bin_ser;

	if (!fmc_config_is_loaded() && (retry_load || !load_tried)) {
		load_tried = BOOL_TRUE;
		TRACE(MS_FW_ALWAYS, "%s: loading FMC config from .bin ...\r\n", __func__);
		(void)fmc_config_ensure_loaded();
		just_loaded = fmc_config_is_loaded() ? BOOL_TRUE : BOOL_FALSE;
		TRACE(MS_FW_ALWAYS, "%s: FMC config load done (loaded=%d)\r\n",
		      __func__, fmc_config_is_loaded());
	}

	if (!fmc_config_is_loaded())
		return;

	/* Bin FMC ids: 0=MAX9296 1=MAX96716 2=MAX96724 3=MAX96726 (shares RAVE). */
	bin_fmc = fmc_config_get_active_fmc_id();
	sel = (bin_fmc == 3) ? FMC_ID_RAVE : bin_fmc;

	if ((sel == FMC_ID_0) || (sel == FMC_ID_1) || (sel == FMC_ID_RAVE)) {
		if (sel != current_active_fmc) {
			(void)SelectFMCID(sel);
			TRACE(MS_FW_ALWAYS,
			      "%s: active FMC selected from .bin (id=%d)\r\n",
			      __func__, sel);
		}
	} else if (just_loaded) {
		TRACE(MS_FW_WARNING,
		      "%s: .bin FMC id %d invalid; keeping default id=%d\r\n",
		      __func__, bin_fmc, current_active_fmc);
	}

	bin_ser = fmc_config_get_serializer_id();
	if (((bin_ser == SER_ID_MAX9295) || (bin_ser == SER_ID_MAX96717)) &&
	    (bin_ser != current_active_serializer)) {
		(void)SelectSerializer(bin_ser);
		TRACE(MS_FW_ALWAYS,
		      "%s: serializer selected from .bin (id=%d)\r\n",
		      __func__, bin_ser);
	}
}

/**************************************************************************
 *          GetDesIdForPipe
 *
 * @brief   Map an input pipe (sensor) id to its deserializer id for the
 *          active FMC topology. MAX96724 (RAVE) is quad-link (4 pipes per
 *          chip); all other FMCs are dual-link (MAPPING_INPIPE_TO_DES_ID).
 *
 * @param   pipeId   Input pipe / sensor device id
 *
 * @return  Deserializer index for the active FMC
 *
 ************************************************************************/
u8 GetDesIdForPipe(u8 pipeId)
{
	fmc_apply_board_config(BOOL_FALSE);

	if (current_active_fmc == FMC_ID_RAVE)
		return pipeId / 4;	/* MAX96724 quad: 4 GMSL links per chip */

	return MAPPING_INPIPE_TO_DES_ID(pipeId);	/* dual-link: pipeId / 2 */
}

/**************************************************************************
 *          GetI2cBusIdForDes
 *
 * @brief   Lookup the I2C bus ID for a given deserializer by searching the
 *          core_des_map table. Centralizes I2C bus topology configuration
 *          in a single point of access.
 *
 * @param   desId   Deserializer index (DS_ONE, DS_TWO, DS_THREE, etc.)
 *
 * @return  I2C bus ID (0-3) on success, 0xFF on error
 *
 ************************************************************************/
u8 GetI2cBusIdForDes(u8 desId)
{
	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);
	fmc_apply_board_config(BOOL_FALSE);

	if (current_active_fmc == FMC_ID_0) {
		/* When a .bin has been applied, honour the per-DES I2C bus it
		 * carries (a board may spread deserializers across the PS and
		 * AXI I2C controllers). Otherwise fall back to the single
		 * shared PS_I2C bus. */
		if (fmc_config_is_loaded()) {
			u8 bus = fmc_config_get_des_bus(desId);

			if (bus != INVALID_I2C_BUS_ID)
				return bus;
		}
		return IIC_INSTANCE_ZERO;
	} else if (current_active_fmc == FMC_ID_1) {
		u8 core_idx, des_idx, busId;
		core_des_mapping_t *active_core_des_map;

		/* Prefer the per-DES I2C bus carried in the .bin when applied;
		 * otherwise fall back to the compiled max96716_core_des_map. */
		if (fmc_config_is_loaded()) {
			u8 bus = fmc_config_get_des_bus(desId);

			if (bus != INVALID_I2C_BUS_ID)
				return bus;
		}

		active_core_des_map = get_active_core_des_map();

		if (!active_core_des_map) {
			TRACE(MS_FW_ERROR, "%s: No Core Des Map selected for des_index %d\r\n",
			__func__, desId);
			return INVALID_I2C_BUS_ID;
		}

		TRACE(MS_FW_INFO, "MAP SIZE : %u\n", MAX96716_CORE_DES_MAP_SIZE);

		for (core_idx = 0; core_idx < MAX96716_CORE_DES_MAP_SIZE; core_idx++) {
			for (des_idx = 0;
			     des_idx < active_core_des_map[core_idx].num_deserializers;
			     des_idx++) {
				if (active_core_des_map[core_idx].des_indices[des_idx] == desId) {
					busId = active_core_des_map[core_idx].i2cBusIds[des_idx];
					TRACE(MS_FW_INFO, "%s: desId=%d -> i2cBusId=%d (core=%d)\n",
							__func__, desId, busId, active_core_des_map[core_idx].core_id);
					return busId;
				}
			}
		}

		TRACE(MS_FW_ERROR, "ERROR: %s - desId %d not found in core_des_map!\n",
			__func__, desId);
		return INVALID_I2C_BUS_ID;
	} else if (current_active_fmc == FMC_ID_RAVE) {
		u8 core_idx, des_idx, busId;

		core_des_mapping_t *active_core_des_map = get_active_core_des_map();

		if (!active_core_des_map) {
			TRACE(MS_FW_ERROR, "%s: No Core Des Map selected for des_index %d\r\n",
			__func__, desId);
			return INVALID_I2C_BUS_ID;
		}

		for (core_idx = 0; core_idx < MAX96724_CORE_DES_MAP_SIZE; core_idx++) {
			for (des_idx = 0;
			     des_idx < active_core_des_map[core_idx].num_deserializers;
			     des_idx++) {
				if (active_core_des_map[core_idx].des_indices[des_idx] == desId) {
					busId = active_core_des_map[core_idx].i2cBusIds[des_idx];
					return busId;
				}
			}
		}

		TRACE(MS_FW_ERROR, "ERROR: %s - desId %d not in max96724 core_des_map!\n",
			__func__, desId);
		return INVALID_I2C_BUS_ID;
	}
	TRACE(MS_FW_ERROR, "%s: Invalid FMC ID: %d\r\n", __func__, current_active_fmc);
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return INVALID_I2C_BUS_ID;
}

/**************************************************************************
 *          FMC identification via FRU EEPROM  (single FMC check)
 *
 * ANSI/VITA 57.1 FMC cards carry an IPMI FRU EEPROM (7-bit addr 0x50 on a
 * single-FMC carrier). The card's product-name string identifies it:
 *   - MAX96716 card stores "Xylon d.o.o." + "hp150AB"
 *   - MAX96726 card stores "Xylon d.o.o." + "hp178AB" (+ extended part no.)
 *   - MAX9296  card carries no product string => concluded by elimination.
 * The distinctive product codes ("hp150AB" / "hp178AB") are matched.
 ************************************************************************/

/**************************************************************************
 *          fmc_id_to_name
 *
 * @brief   Map an FMC id to a human-readable FMC name for tracing.
 *
 * @param   fmc_id    FMC identifier
 *
 * @return  Constant string naming the FMC (never NULL).
 *
 ************************************************************************/
static const char *fmc_id_to_name(int fmc_id)
{
	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	switch (fmc_id) {
	case FMC_ID_0:
		return "MAX9296";
	case FMC_ID_1:
		return "MAX96716";
	case FMC_ID_RAVE:
		return "RAVE_board (MAX96724)";
	default:
		TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
		return "UNKNOWN";
	}
}

/**************************************************************************
 *          fmc_fru_contains
 *
 * @brief   Return whether the byte buffer contains the given key string.
 *
 * @param   buf   Buffer to search
 * @param   len   Number of bytes in buf
 * @param   key   NUL-terminated key string to find
 *
 * @return  BOOL_TRUE if key is found in buf, else BOOL_FALSE.
 *
 ************************************************************************/
static bool_t fmc_fru_contains(const u8 *buf, u16 len, const char *key)
{
	u16 i, k, klen = 0;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	while (key[klen] != '\0')
		klen++;
	if (klen == 0 || klen > len)
		return BOOL_FALSE;

	for (i = 0; i + klen <= len; i++) {
		for (k = 0; k < klen; k++) {
			if (buf[i + k] != (u8)key[k])
				break;
		}
		if (k == klen)
			return BOOL_TRUE;
	}

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return BOOL_FALSE;
}

/**************************************************************************
 *          fmc_identify_by_eeprom
 *
 * @brief   The single FMC check: read the FRU EEPROM and conclude which
 *          FMC is connected from the product-name string it carries, then
 *          verify it matches the FMC the user selected. All FMC cards expose
 *          the FRU EEPROM at the SAME slave address (0x50), so a wrong FMC id
 *          cannot be caught by the address alone - the physically-detected
 *          FMC must be matched against the id the user passed.
 *          Requires PS I2C (bus 0) to be initialised first.
 *
 * @param   expected_fmc_id  FMC id the user/firmware selected.
 *
 * @return  FMC_ID_1 (MAX96716)      when "hp150AB" is found,
 *          FMC_ID_MAX96726          when "hp178AB" is found,
 *          FMC_ID_0 (MAX9296)       when neither string is found,
 *          FMC_ID_MISMATCH (-2)     when the detected FMC does not match
 *                                   expected_fmc_id (mismatch is traced),
 *          -1                       when the EEPROM cannot be read.
 *
 ************************************************************************/
int fmc_identify_by_eeprom(int expected_fmc_id)
{
	u8 fru[FMC_FRU_SCAN_LEN] = {0};
	RESULT Status;
	u16 off;
	int detected;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	/*
	 * Read the FRU EEPROM one byte at a time. Each register read returns a
	 * raw byte (a hex value); collected together these bytes are the ASCII
	 * characters of the strings stored in the FRU, which the search below
	 * matches against the product-name keys.
	 *
	 * A single bulk read cannot be used here: the HAL 'dataWidth' argument
	 * is a u8, so requesting FMC_FRU_SCAN_LEN (256) bytes wraps to 0 and
	 * reads nothing, leaving the buffer all-zero (the previous behaviour
	 * that made this always fall through to MAX9296).
	 */
	for (off = 0; off < FMC_FRU_SCAN_LEN; off++) {
		Status = HalReadI2CReg(IIC_INSTANCE_ZERO, FMC_FRU_EEPROM_ADDR,
				off, 0x1, &fru[off], 1);
		if (Status != RET_SUCCESS) {
			char problem[96];

			TRACE(MS_FW_ERROR,
				"%s: FMC NOT CONNECTED - you selected the '%s' FMC, but no "
				"FMC responds on the board (FRU EEPROM 0x%02x not accessible "
				"at offset %u, err=%d).\r\n",
				__func__, fmc_id_to_name(expected_fmc_id),
				FMC_FRU_EEPROM_ADDR, off, Status);

			strcpy(problem, "The selected ");
			strcat(problem, fmc_id_to_name(expected_fmc_id));
			strcat(problem, " FMC is not connected on the board.");

			report_pipeline_fault("FMC card", "FMC detection",
				problem,
				"Connect the selected FMC card to the board, then power-cycle to proceed.");
			return -1;
		}
	}

	if (fmc_fru_contains(fru, FMC_FRU_SCAN_LEN, FMC_FRU_KEY_MAX96716)) {
		TRACE(MS_FW_ALWAYS,
			"MAX96716 FMC connected (product '%s')\r\n",
			FMC_FRU_KEY_MAX96716);
		detected = FMC_ID_1;
	} else if (fmc_fru_contains(fru, FMC_FRU_SCAN_LEN, FMC_FRU_KEY_MAX96726)) {
		TRACE(MS_FW_ALWAYS,
			"MAX96726 FMC connected (product '%s')\r\n",
			FMC_FRU_KEY_MAX96726);
		detected = FMC_ID_MAX96726;
	} else {
		/* No known product string => MAX9296 (its FRU carries no product name). */
		TRACE(MS_FW_ALWAYS,
			"MAX9296 FMC connected\r\n");
		detected = FMC_ID_0;
	}

	/*
	 * All FMC cards expose the FRU EEPROM at the same slave address (0x50),
	 * so a wrong FMC id cannot be caught by the address alone. Compare the
	 * physically-detected FMC against the id the user selected and halt the
	 * caller on a mismatch.
	 */
	if (detected != expected_fmc_id) {
		TRACE(MS_FW_ERROR,
			"%s: FMC MISMATCH - user selected '%s' (id %d) but the FMC "
			"connected on the hardware is '%s' (id %d) per its FRU EEPROM. "
			"Halting: correct the selected FMC id or connect the matching "
			"FMC card.\r\n",
			__func__,
			fmc_id_to_name(expected_fmc_id), expected_fmc_id,
			fmc_id_to_name(detected), detected);
		return FMC_ID_MISMATCH;
	}

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return detected;
}
/**************************************************************************
 *          Sensor identification + summary  (common APIs)
 *
 * check_sensor_id() reads a sensor's chip id over I2C and resolves it to a
 * sensor name. It reuses the sensor table already defined in sensor_drv
 * (sensorCfgList) instead of duplicating it here.
 *
 * print_sensor_summary() is called by the FMC drivers once FMC setup is
 * done, to list which sensor is connected on each deserializer link.
 ************************************************************************/
/* NOTE: the 3x chip-ID read RETRY (glitch ride-out) was removed for now and
 * archived in check_sensor_id_retry.bak (same folder). check_sensor_id() does
 * a single read; restore from the .bak if transient I2C glitches resurface. */

/**************************************************************************
 *          read_chip_id
 *
 * @brief   Read one sensor entry's chip-ID registers over I2C and assemble
 *          its chip id exactly as that entry's sensorCfgList layout says.
 *          Reading each entry with its OWN registers is what lets two sensors
 *          that share an I2C address (e.g. ox03f10/ox08b40 at 0x6C,
 *          imx728/imx623 at 0x36) be told apart by the id they return.
 *
 * @param   i2cBusId   I2C bus to read on
 * @param   cfg        sensorCfgList entry describing the ID registers/layout
 * @param   addr7      7-bit I2C slave address to read from
 * @param   outId      Receives the assembled chip id on success
 *
 * @return  RET_SUCCESS when every non-zero ID register was read,
 *          the failing HAL status otherwise.
 *
 ************************************************************************/
static RESULT read_chip_id(u8 i2cBusId, const SensorDrvConfigList_t *cfg,
	u8 addr7, uint32_t *outId)
{
	uint8_t regVal[SENSOR_ID_REG_COUNT] = {0};
	uint8_t r;
	RESULT rd;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	for (r = 0; r < SENSOR_ID_REG_COUNT; r++) {
		if (cfg->regAddr[r] == 0)
			continue;

		rd = HalReadI2CReg(i2cBusId, addr7, cfg->regAddr[r],
			cfg->regWidth, &regVal[r], cfg->dataWidth);
		if (rd != RET_SUCCESS)
			return rd;
	}

	if (cfg->regAddr[0] && !cfg->regAddr[1] && !cfg->regAddr[2])
		*outId = (regVal[0] & 0xffff);
	else if (cfg->regAddr[0] && cfg->regAddr[1] && !cfg->regAddr[2])
		*outId = ((regVal[0] & 0xff) << 8) | (regVal[1] & 0xff);
	else if (cfg->regAddr[0] && cfg->regAddr[1] && cfg->regAddr[2])
		*outId = ((regVal[0] & 0xff) << 16) |
			 ((regVal[1] & 0xff) << 8) | (regVal[2] & 0xff);
	else
		*outId = 0;

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          check_sensor_id
 *
 * @brief   Identify the sensor physically connected on this link and validate
 *          it against the configured one.
 *
 *          Detection always happens at the per-link sensor alias (sensorAlias)
 *          - a UNIQUE address - so only THIS link's sensor is ever touched (no
 *          cross-link leakage on the shared I2C bus). The serializer's alias
 *          translation DST is re-pointed to each known sensor native address
 *          ("family") in turn, so a fitted sensor whose family differs from
 *          the selected one (e.g. an OV 3MP fitted where imx728 was selected)
 *          is still reached and named. The chip id read back names it:
 *
 *          - nothing answers on any family -> sensor not connected
 *          - answers, id not recognised    -> unsupported sensor
 *          - answers, id == expected chip-ID -> sensor identified (match)
 *          - answers, id != expected chip-ID -> WRONG sensor (fail)
 *
 *          The connected part is identified by chip-ID, then that chip-ID is
 *          compared against expected_id (the chip-ID of the sensor this
 *          pipeline was launched for). Because chip-IDs are globally unique,
 *          this distinguishes even SAME-address-family siblings (ox03f10 vs
 *          ox08b40 at 0x6c; imx728 vs imx623 at 0x36) - no name string needed.
 *
 * @param   i2cBusId       I2C bus the sensor is reached on
 * @param   ser7bit        7-bit serializer I2C address (for translation)
 * @param   sensorAlias    8-bit sensor I2C alias the sensor answers to
 * @param   sensorNative   8-bit native sensor address programmed as the
 *                         serializer translation DST; tried first so the
 *                         common case does no extra translation writes
 * @param   expected_id    Chip-ID of the SELECTED sensor (from sensorCfgList).
 *                         The detected chip-ID must equal this or the call
 *                         fails with RET_WRONG_CONFIG.
 * @param   outName        Buffer to receive the identified sensor name
 * @param   outNameLen     Size of outName
 *
 * @return  RET_SUCCESS       Connected sensor identified and matches expected
 * @retval  RET_NOTAVAILABLE  No sensor accessible, or chip id unrecognised
 * @retval  RET_WRONG_CONFIG  Connected sensor is not the selected one
 *
 ************************************************************************/
RESULT check_sensor_id(u8 i2cBusId, u8 ser7bit, u8 sensorAlias,
	u8 sensorNative, uint32_t expected_id, char *outName, uint32_t outNameLen)
{
	const SensorDrvConfigList_t *detected = NULL;
	uint32_t families[SENSOR_ID_FAMILY_MAX];
	uint16_t nf = 0;
	uint16_t fidx;
	uint16_t tableSize = 0;
	uint16_t idx;
	uint32_t id = 0;
	uint8_t s_addr = 0;
	bool_t anyAck = BOOL_FALSE;
	RESULT rd;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (outName != NULL && outNameLen > 0)
		outName[0] = '\0';

	SensorDrvGetSensorNumber(&tableSize);

	/*
	 * Detection is done at the per-link alias (s_addr), which the serializer
	 * translation maps to whatever native address (DST) we program. Because
	 * the alias is unique to this link, only this link's sensor can respond -
	 * so we can safely re-point the translation across families without ever
	 * catching another link's sensor on the shared bus.
	 */
	s_addr = sensorAlias >> 1;

	/*
	 * Build the list of distinct native addresses ("families"), the address
	 * passed to init_sensor first so the common case (a sensor answering at
	 * that address) is identified in the very first pass with no extra
	 * translation writes.
	 */
	families[nf++] = sensorNative;
	for (idx = 0; idx < tableSize; idx++) {
		uint32_t a = sensorCfgList[idx].slaveAddr;
		bool_t seen = BOOL_FALSE;
		uint16_t j;

		for (j = 0; j < nf; j++) {
			if (families[j] == a) {
				seen = BOOL_TRUE;
				break;
			}
		}
		if (!seen && nf < SENSOR_ID_FAMILY_MAX)
			families[nf++] = a;
	}

	/*
	 * For each family: re-point the serializer translation DST to that native
	 * address, then read the chip id of each candidate in that family at the
	 * alias. The first read that acknowledges proves a sensor is present
	 * (anyAck); the chip id that matches names it. Stop as soon as a sensor is
	 * identified - trying the configured family first means a correct sensor
	 * leaves the translation programmed for it, so no restore is needed (and a
	 * wrong/absent sensor halts the pipeline and the next init_sensor re-aliases
	 * this link from scratch anyway).
	 */
	for (fidx = 0; fidx < nf && detected == NULL; fidx++) {
		/*
		 * The passed address (fidx 0) is already programmed into the
		 * serializer translation DST (I2C_3) by init_sensor, so re-point the
		 * translation only for the OTHER families. The common case - a sensor
		 * answering at the passed address is identified in the first pass -
		 * does no extra translation writes at all.
		 */
		if (fidx > 0) {
			uint8_t native8 = (uint8_t)families[fidx];

			rd = HalWriteI2CReg(i2cBusId, ser7bit, I2C_3, 0x2,
					    &native8, 1);
			if (rd != RET_SUCCESS)
				continue;
			osSleep(1);	/* let the new translation take effect */
		}

		for (idx = 0; idx < tableSize; idx++) {
			const SensorDrvConfigList_t *e = &sensorCfgList[idx];
			uint32_t idmask =
				(e->regAddr[2] != 0) ? 0xFFFFFFu : 0xFFFFu;

			if (e->slaveAddr != families[fidx])
				continue;

			/* Unlock a gated chip-ID page (e.g. imx728) before reading. */
			if (e->prepareId != NULL)
				e->prepareId(i2cBusId, s_addr);

			/*
			 * Single chip-ID read (no retries for now - the 3-retry
			 * glitch ride-out was moved to check_sensor_id_retry.bak;
			 * fetch it from there if transient glitches resurface).
			 */
			rd = read_chip_id(i2cBusId, e, s_addr, &id);
			if (rd != RET_SUCCESS)
				continue;	/* nothing of this family at the alias */

			anyAck = BOOL_TRUE;	/* a device answered at the alias */

			if ((id & idmask) == (e->sensorId & idmask)) {
				detected = e;
				break;
			}
		}
	}

	/* Case 0 - nothing answered on any family: no sensor on this link. */
	if (!anyAck) {
		TRACE(MS_FW_ERROR, "\r\n");
		TRACE(MS_FW_ERROR, SENSOR_BOX_EDGE);
		TRACE(MS_FW_ERROR, "   SENSOR NOT CONNECTED\r\n");
		TRACE(MS_FW_ERROR, SENSOR_BOX_RULE);
		TRACE(MS_FW_ERROR,
			"   No sensor answered at I2C 0x%02x.\r\n",
			(uint8_t)s_addr);
		TRACE(MS_FW_ERROR, SENSOR_BOX_EDGE);
		report_pipeline_fault("Camera sensor", "sensor access check",
			"No camera sensor is connected, or it cannot be accessed "
			"over I2C on this link.",
			"Connect the camera sensor and check the cabling, then "
			"power-cycle the board.");
		return RET_NOTAVAILABLE;
	}

	/* Case 1 - a device answered but its chip id is not known. */
	if (detected == NULL) {
		TRACE(MS_FW_ERROR, "\r\n");
		TRACE(MS_FW_ERROR, SENSOR_BOX_EDGE);
		TRACE(MS_FW_ERROR, "   UNRECOGNISED SENSOR\r\n");
		TRACE(MS_FW_ERROR, SENSOR_BOX_RULE);
		TRACE(MS_FW_ERROR, "   Detected id  : 0x%x (unknown)\r\n", id);
		TRACE(MS_FW_ERROR, SENSOR_BOX_EDGE);
		report_pipeline_fault("Camera sensor", "sensor identity check",
			"A sensor is connected but its chip ID does not match any "
			"sensor this firmware supports.",
			"Fit a supported camera sensor, then power-cycle the "
			"board.");
		return RET_NOTAVAILABLE;
	}

	/* Case 2 - identified, but NOT the sensor this pipeline was launched for.
	 * The connected sensor's chip-ID differs from the selected one
	 * (expected_id). Chip-IDs are globally unique, so this catches even a
	 * SAME-address-family sibling (e.g. ox03f10 fitted where ox08b40 was
	 * selected, both at 0x6c) - fail HERE rather than let the selected
	 * sensor's driver proceed on the wrong part and fail cryptically later. */
	if (detected->sensorId != expected_id) {
		const char *selName = "unknown";

		/* Resolve the SELECTED sensor's name from its chip-ID so the
		 * operator sees a readable part name, not a raw ID. */
		for (idx = 0; idx < tableSize; idx++) {
			if (sensorCfgList[idx].sensorId == expected_id) {
				selName = sensorCfgList[idx].pSensorName;
				break;
			}
		}

		TRACE(MS_FW_ERROR, "\r\n");
		TRACE(MS_FW_ERROR, SENSOR_BOX_EDGE);
		TRACE(MS_FW_ERROR, "   WRONG SENSOR CONNECTED\r\n");
		TRACE(MS_FW_ERROR, SENSOR_BOX_RULE);
		TRACE(MS_FW_ERROR, "   Selected sensor  : %s\r\n", selName);
		TRACE(MS_FW_ERROR, "   Connected sensor : %s\r\n",
			detected->pSensorName);
		TRACE(MS_FW_ERROR, SENSOR_BOX_EDGE);
		report_pipeline_fault("Camera sensor", "sensor identity check",
			"The sensor connected on this link is not the one selected "
			"for this pipeline.",
			"Connect the selected sensor, or launch the pipeline for the "
			"sensor that is actually connected, then power-cycle the board.");
		return RET_WRONG_CONFIG;
	}

	/* Case 3 - identified and matches the selected sensor. */
	if (outName != NULL && outNameLen > 0) {
		strncpy(outName, detected->pSensorName, outNameLen - 1);
		outName[outNameLen - 1] = '\0';
	}

	TRACE(MS_FW_ALWAYS,
		"Connected sensor : %s (id 0x%x)\r\n",
		detected->pSensorName, id);
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          print_sensor_summary
 *
 * @brief   Print the connected-sensor summary table for one deserializer.
 *          Names come from successful init_sensor validation; this function
 *          performs no sensor I2C probing.
 *
 *          Each link shows one of three states: "active" (sensor validated,
 *          pipeline up), "pending" (deserializer remapped but this link's
 *          pipeline not launched yet - typically the peer link on the same
 *          des is still coming up, so it is not mislabelled "absent"), and
 *          "absent" (link_type excludes this link). The firmware TRACE
 *          (xil_printf) has no field-width support, so the only
 *          variable-width column - the sensor name - is space-padded inline;
 *          every other column is already fixed width.
 *
 * @param   des_array    Active deserializer array (base pointer)
 * @param   des_index    Index of the deserializer to summarise
 *
 ************************************************************************/
void print_sensor_summary(desInterface *des_array, int des_index)
{
	const char *labels[2] = {"Link-A", "Link-B"};
	core_des_mapping_t *map = get_active_core_des_map();
	desInterface *d;
	int link;
	int owner;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (des_array == NULL) {
		TRACE(MS_FW_ERROR, "%s: no active des array for summary\r\n",
			__func__);
		return;
	}

	d = &des_array[des_index];

	/*
	 * Label this deserializer with the core that OWNS it, so a later core's
	 * summary lists every earlier core's deserializers under their own core
	 * id, in any launch order.
	 *   - MAX96716: from the static core_des_map.
	 *   - MAX9296: no usable core map, but it is multi-core too - the owner is
	 *     the core that LAUNCHED a link on this des (recorded in the shared
	 *     topology at launch). Fall back to the current core until a link on
	 *     this des has come up.
	 */
	if (current_active_fmc == FMC_ID_0) {
		if (d->link_a.sensor_name[0] != '\0')
			owner = (int)d->link_a.owner_core;
		else if (d->link_b.sensor_name[0] != '\0')
			owner = (int)d->link_b.owner_core;
		else
			owner = (int)cpu_id;
	} else {
		/*
		 * Multi-I2C-bus FMC (has a core<->deserializer map): the owner is
		 * the core whose des_indices[] contains this deserializer.
		 */
		u8 c, dd;

		owner = -1;
		for (c = 0; map != NULL && c < MAX96716_CORE_DES_MAP_SIZE; c++) {
			for (dd = 0; dd < map[c].num_deserializers; dd++) {
				if (map[c].des_indices[dd] == (u8)des_index) {
					owner = (int)map[c].core_id;
					break;
				}
			}
		}
	}

	TRACE(MS_FW_ALWAYS, "\r\n");
	TRACE(MS_FW_ALWAYS,
		"  Connected Sensor Summary : Core-%x  DS_%d  \r\n",
		(owner >= 0) ? (u8)owner : cpu_id, des_index + 1);
	TRACE(MS_FW_ALWAYS, "  +--------------+--------+--------------+------+---------+\r\n");
	TRACE(MS_FW_ALWAYS, "  | Deserializer | Link   | Sensor       | Addr | Status  |\r\n");
	TRACE(MS_FW_ALWAYS, "  +--------------+--------+--------------+------+---------+\r\n");

	for (link = 0; link < 2; link++) {
		int dn = des_index + 1;
		bool_t present = (link == 0)
			? (d->link_type == LINK_A ||
			   d->link_type == LINK_REVERSE_SPLITTER)
			: (d->link_type == LINK_B ||
			   d->link_type == LINK_REVERSE_SPLITTER);
		/*
		 * Sensor name + alias come from the (shared) deserializer topology.
		 * link_a/link_b.sensor_name is filled by whichever CORE launched
		 * that link, so every core's summary shows the sensor on every link,
		 * not only the links this core launched.
		 */
		const char *nm = (link == 0) ? d->link_a.sensor_name
					     : d->link_b.sensor_name;
		u8 alias = (link == 0) ? d->link_a.sensor_alias_addr
				       : d->link_b.sensor_alias_addr;
		u8 killed_flag = (link == 0) ? d->link_a.killed
					     : d->link_b.killed;
		const char *src;
		const char *status;
		char des[13];		/* "DS_n" on the first row, blank under it */
		char name[13];		/* 12-wide field + NUL */
		int j = 0;
		int k = 0;

		/*
		 * Show the Des only on its first (Link-A) row and leave it blank on
		 * the Link-B row, so both links read as grouped under the same
		 * deserializer. The field is padded to 12 chars to align under the
		 * "Deserializer" header column.
		 */
		if (link == 0) {
			des[k++] = 'D'; des[k++] = 'S'; des[k++] = '_';
			if (dn >= 10) {
				des[k++] = (char)('0' + dn / 10);
				des[k++] = (char)('0' + dn % 10);
			} else {
				des[k++] = (char)('0' + dn);
			}
		}
		while (k < 12)
			des[k++] = ' ';
		des[12] = '\0';

		if (killed_flag) {
			/*
			 * Pipeline was killed / torn down. Show it as "killed"
			 * (keeping the last known sensor name if any) so a
			 * stopped pipe is not mistaken for a live one when
			 * another pipeline is subsequently launched.
			 */
			src = (nm[0] != '\0') ? nm : "-";
			status = "killed ";
		} else if (!present) {
			src = "-";
			status = "absent ";
		} else if (nm[0] != '\0') {
			src = nm;
			status = "active ";
		} else {
			src = "-";
			status = "pending";
		}

		/* Left-justify the sensor name into a fixed 12-char field. */
		while (src[j] != '\0' && j < 12) {
			name[j] = src[j];
			j++;
		}
		while (j < 12)
			name[j++] = ' ';
		name[12] = '\0';

		if (present)
			TRACE(MS_FW_ALWAYS,
				"  | %s | %s | %s | 0x%02x | %s |\r\n",
				des, labels[link], name, (u8)(alias >> 1), status);
		else
			TRACE(MS_FW_ALWAYS,
				"  | %s | %s | %s | ---- | %s |\r\n",
				des, labels[link], name, status);
	}

	TRACE(MS_FW_ALWAYS, "  +--------------+--------+--------------+------+---------+\r\n");
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
}

/**************************************************************************
 *          des_owned_by_current_core
 *
 * @brief   Decide whether a deserializer should appear in THIS core's
 *          summary. Ownership follows the static core_des_map struct, which
 *          is only used for the MAX96716 FMC (multi-I2C-bus / multi-core): a
 *          deserializer belongs to the core whose map entry lists it. Other
 *          FMCs have no per-core map, so nothing is filtered for them.
 *
 * @param   merged   Local copy of the shared deserializer topology
 * @param   di       Deserializer index to test
 *
 * @return  1 if core_des_map assigns this des to the current core, else 0
 *
 ************************************************************************/
static int des_owned_by_current_core(desInterface *merged, int di)
{
	core_des_mapping_t *map;
	u8 des_id = merged[di].Port_DES_index;
	u8 map_size;
	u8 c, d;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	/*
	 * Pick the core<->deserializer map size for the ACTIVE FMC instead of
	 * hard-coding one FMC's size. core_des_map is only used by multi-I2C-bus
	 * / multi-core FMCs (currently the MAX96716). A future FMC that adds its
	 * own map just adds an `else if` here with its own size macro; an FMC
	 * without a map is not filtered.
	 */
	if (current_active_fmc == FMC_ID_1)
		map_size = MAX96716_CORE_DES_MAP_SIZE;
	else
		return 1;	/* no per-core map for this FMC -> do not filter */

	map = get_active_core_des_map();
	if (map == NULL)
		return 1;	/* no map -> do not filter */

	for (c = 0; c < map_size; c++) {
		if (map[c].core_id != cpu_id)
			continue;
		for (d = 0; d < map[c].num_deserializers; d++)
			if (map[c].des_indices[d] == des_id)
				return 1;
	}

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return 0;
}

/**************************************************************************
 *          print_all_sensor_summary
 *
 * @brief   Print the connected-sensor summary for EVERY deserializer that has
 *          been remapped / set up this run - not just the one whose pipeline
 *          was launched last. It loops the active deserializer array and emits
 *          one table per remapped deserializer, so the operator sees the whole
 *          picture in a single place. Deserializers that were never remapped
 *          (still in deinit with no link) are skipped.
 *
 ************************************************************************/
void print_all_sensor_summary(void)
{
	desInterface *des_array = get_active_des_array();
	desInterface merged[DS_MAX];
	int di;

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (des_array == NULL) {
		TRACE(MS_FW_ERROR, "%s: no active des array for summary\r\n",
			__func__);
		return;
	}

	/*
	 * Pull the latest deserializer topology AND the per-link sensor names
	 * from shared memory into a local copy, so this core's summary reflects
	 * EVERY core's launched links (each launching core publishes its link's
	 * sensor name into the shared des topology). Printing from the copy
	 * leaves the live des array untouched.
	 */
	byte_memcpy(merged, SHARED_DES_ARRAY_SRUCT,
		DS_MAX * sizeof(desInterface));

	for (di = 0; di < DS_MAX; di++) {
		if (merged[di].des_state == in_deinit &&
		    merged[di].link_type == NO_LINK)
			continue;

		/*
		 * Per-core view: only summarise the deserializers this core
		 * launched/remapped, so core r6's summary shows only r6's Deses.
		 */
		if (!des_owned_by_current_core(merged, di))
			continue;

		print_sensor_summary(merged, di);
	}

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
}

/**************************************************************************
 *          check_ser_id
 *
 * @brief   Serializer presence and identity check (same idea as
 *          check_sensor_id, but for the serializer). Called by the FMC
 *          drivers on the first I2C access to a serializer: it reads the
 *          serializer's REG0 (device-address register, offset 0x00). If it
 *          cannot be read, the serializer/camera link is absent or
 *          defective; if it answers with an unexpected device address, the
 *          wrong device is on this link. Either way a clear error is traced,
 *          report_pipeline_fault() guides the operator, and a failure code is
 *          returned so the caller halts instead of proceeding with a bad
 *          device.
 *
 * @param   i2cBusId       I2C bus the serializer is reached on
 * @param   ser7bit        7-bit serializer I2C address
 * @param   link_tag       Link label for the terminal trace (e.g. "link-A")
 *
 * @return  RET_SUCCESS       REG0 read OK and serializer identity verified
 * @retval  RET_NOTAVAILABLE  REG0 not readable (absent / defective)
 * @retval  RET_WRONG_CONFIG  Unexpected device answered at this address
 *
 ************************************************************************/
RESULT check_ser_id(u8 i2cBusId, u8 ser7bit, const char *link_tag)
{
	RESULT Status;
	u8 rd[2] = {0};
	u8 expected = (u8)(ser7bit << 1);

	if (link_tag == NULL)
		link_tag = "link-?";

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	Status = HalReadI2CReg(i2cBusId, ser7bit, DEV_ADDR_REG, 0x2, rd, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR,
			"SERIALIZER ID CHECK FAILED (%s): serializer at 0x%02x did not "
			"respond on I2C bus %d (device-address register not readable).\r\n",
			link_tag, expected, i2cBusId);
		report_pipeline_fault("GMSL serializer", "serializer identity check",
			"The serializer/camera link did not respond - it is absent or defective.",
			"Check the camera cable and serializer board connections, "
			"then power-cycle the board.");
		return RET_NOTAVAILABLE;
	}

	if ((rd[0] & 0xFE) != (expected & 0xFE)) {
		TRACE(MS_FW_ERROR,
			"SERIALIZER MISMATCH (%s): expected serializer at device address "
			"0x%02x on I2C bus %d, but the device reports 0x%02x.\r\n",
			link_tag, expected, i2cBusId, rd[0]);
		report_pipeline_fault("GMSL serializer", "serializer identity check",
			"The device on this link is not the expected serializer.",
			"Verify the correct camera/serializer is fitted for this "
			"link, then power-cycle the board.");
		return RET_WRONG_CONFIG;
	}

	TRACE(MS_FW_ALWAYS,
		"Serializer 0x%02x (%s) present and identity verified (REG0 = 0x%02x)\r\n",
		expected, link_tag, rd[0]);
	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          init_des
 *
 * @brief   Initialize deserializer interface for the specified index
 *
 * @param   des_index     Index of the deserializer to initialize
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Initialization successful
 * @retval  RET_FAILURE   Initialization failed
 *
 ************************************************************************/
RESULT init_des(int des_index)
{
	RESULT Status = RET_SUCCESS;

	/* If the log STOPS after the loader's "about to READ .bin header" line,
	 * 0x1E4E8000 is not readable by the R5 (MPU region not mapped). */
	fmc_apply_board_config(BOOL_TRUE);
	if (!fmc_config_is_loaded()) {
		TRACE(MS_FW_ALWAYS,
		      "init_des: no .bin applied; using platform default FMC id=%d\r\n",
		      current_active_fmc);
	}

	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected for des_index %d\r\n",
			__func__, des_index);
		return XST_INVALID_PARAM;
	}

	/* Bind the selected serializer (MAX9295 / MAX96717) before any
	 * serializer is configured, replacing the hardcoded array. */
	TRACE(MS_FW_ALWAYS,
		"init_des(%d): applying selected serializer\r\n", des_index);
	apply_active_serializer(active_fmc);

	desInterface *active_des_array = get_active_des_array();

	if (!active_des_array) {
		TRACE(MS_FW_ERROR, "%s: No Des Array selected for des_index %d\r\n",
			__func__, des_index);
		return XST_INVALID_PARAM;
	}

	TRACE(MS_FW_INFO, "Initializing deserializer"
		"%d using FMC: %s with %s array\r\n",
			des_index, active_fmc->FmcName,
			(current_active_fmc == FMC_ID_0) ? "des_arr" : "max96716_des_arr");

	Status = InitIIC();
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: PS I2C Init Failed for des_index %d (err=%d)\r\n",
			__func__, des_index, Status);
		return Status;
	}

	/*
	 * Single FMC check: now that PS I2C is up, conclude which FMC is
	 * physically connected from its FRU EEPROM product string and verify it
	 * matches the FMC the firmware was told to use. All FMCs share the same
	 * EEPROM slave address, so this id-vs-hardware match is the only reliable
	 * guard - halt on ANY negative result: FMC_ID_MISMATCH (wrong FMC fitted)
	 * or -1 (no FMC connected / FRU EEPROM not accessible). Both are already
	 * traced + reported inside the check.
	 */
	if (fmc_identify_by_eeprom(current_active_fmc) < 0)
		return RET_FAILURE;

	Status = active_fmc->pIsiIsiFmcSetup(des_index);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: FMC Setup Failed for des_index %d (err=%d)\r\n",
			__func__, des_index, Status);
		return Status;
	}

	Status = active_fmc->pIsiDeserSetup(&active_des_array[des_index]);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR, "%s: Deserializer Setup"
			" Failed for des_index %d (err=%d)\r\n",
			__func__, des_index, Status);
		return Status;
	}

	TRACE(MS_FW_INFO, "%s (exit) des_index %d\r\n", __func__, des_index);
	return RET_SUCCESS;
}

/**************************************************************************
 *          start_sensor
 *
 * @brief   Start streaming on the specified sensor pipeline
 *
 * @param   in_pipe       Input pipeline index to start streaming
 *
 * @return  None
 *
 ************************************************************************/
void start_sensor(int in_pipe)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected"
			"for sensor start on pipeline %d\r\n",
			__func__, in_pipe);
		return;
	}

	TRACE(MS_FW_INFO, "enter %s on pipeline"
		"%d using FMC: %s\r\n",
		__func__, in_pipe, active_fmc->FmcName);

	if ((active_fmc->sensor_array[in_pipe])->sensor_state == in_deinit) {
		TRACE(MS_FW_INFO, "%s: Sensor %s on"
			"Pipeline-0x%x , cannot stream"
			"as it is not initialized\r\n",
			__func__,
			(active_fmc->sensor_array[in_pipe])->name,
			in_pipe);
	} else if ((active_fmc->sensor_array[in_pipe])->sensor_state == in_running) {
		TRACE(MS_FW_INFO, "%s: Sensor %s on Pipeline-0x%x ,Already streaming \r\n",
				__func__, (active_fmc->sensor_array[in_pipe])->name, in_pipe);
	} else {
		/*
		 * 1) We reach here after init time,
		 * 2) We reach here after previous stop
		 */
		int ret = (active_fmc->sensor_array[in_pipe])->
				stream_on(active_fmc->sensor_array[in_pipe]);

		if (ret != RET_SUCCESS) {
			TRACE(MS_FW_ERROR, "%s: stream_on failed for pipe %d (error=%d)\n",
					__func__, in_pipe, ret);
			return;
		}
		(active_fmc->sensor_array[in_pipe])->sensor_state = in_running;

		TRACE(MS_FW_INFO, "Streaming Started on Sensor %s on Pipeline-0x%x...\r\n",
				(active_fmc->sensor_array[in_pipe])->name, in_pipe);
	}

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
}

/**************************************************************************
 *          access_iic_read
 *
 * @brief   Read data from I2C slave device register
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   slave_addr    Slave device address
 * @param   addr          Register address to read from
 * @param   regWidth      Register address width in bytes
 * @param   pValue        Pointer to store read value
 * @param   dataWidth     Data width in bytes
 *
 * @return  Return the result of the function call.
 *
 ************************************************************************/
int access_iic_read(u8 i2cBusId, u8 slave_addr, uint16_t addr,
		uint8_t regWidth, uint8_t *pValue, uint8_t dataWidth)
{
	return HalReadI2CReg(i2cBusId, slave_addr, addr, regWidth, pValue, dataWidth);
}

/**************************************************************************
 *          access_iic_write
 *
 * @brief   Write data to I2C slave device register
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   slave_addr    Slave device address
 * @param   addr          Register address to write to
 * @param   regWidth      Register address width in bytes
 * @param   value         Value to write
 * @param   dataWidth     Data width in bytes
 *
 * @return  Return the result of the function call.
 *
 ************************************************************************/
int access_iic_write(u8 i2cBusId, u8 slave_addr, uint16_t addr,
		uint8_t regWidth, uint8_t *pValue, uint8_t dataWidth)
{
	return HalWriteI2CReg(i2cBusId, slave_addr, addr,
		regWidth, pValue, dataWidth);
}


/**************************************************************************
 *          init_iic_access
 *
 * @brief   Initialize I2C access interface for the specified pipeline
 *
 * @param   i2cBusId      I2C bus identifier
 * @param   in_pipe       Input pipeline index
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Initialization successful
 * @retval  RET_OUTOFMEM  Memory allocation failed
 * @retval  RET_FAILURE   No active FMC selected
 *
 ************************************************************************/
RESULT init_iic_access(u8 i2cBusId, int in_pipe)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s: No FMC selected"
			" for IIC access init on pipe %d\r\n",
			__func__, in_pipe);
		return RET_FAILURE;
	}

	TRACE(MS_FW_INFO, "Enter %s for pipe %d"
		"using FMC: %s\r\n",
		__func__, in_pipe, active_fmc->FmcName);

	// accessiic_handle[in_pipe] =
	// 	(struct accessIIC *)osMalloc(
	// 		sizeof(struct accessIIC));
	// if (!accessiic_handle[in_pipe]) {
	// 	TRACE(MS_FW_ERROR, "%s: osMalloc failed for accessIIC (pipe=%d)\n",
	// 		__func__, in_pipe);
	// 	return RET_OUTOFMEM;
	// }

	memset(&accessiic_handle[in_pipe], 0, sizeof(struct accessIIC));

	
	active_fmc->accessiic_array[in_pipe] = &accessiic_handle[in_pipe];
	active_fmc->accessiic_array[in_pipe]->i2cBusId = i2cBusId;
	active_fmc->accessiic_array[in_pipe]->readIIC  = access_iic_read;
	active_fmc->accessiic_array[in_pipe]->writeIIC = access_iic_write;

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
	return RET_SUCCESS;
}

/**************************************************************************
 *          init_sensor
 *
 * @brief   Initialize sensor driver for the specified
 *          pipeline and deserializer
 *
 * @param   in_pipe       Input pipeline index
 * @param   des_index     Deserializer index
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS    Initialization successful
 * @retval  RET_OUTOFMEM   Memory allocation failed
 * @retval  RET_FAILURE    No active FMC or Des Array selected
 *
 ************************************************************************/
RESULT init_sensor(int in_pipe, int des_index, int sensor_address,
	uint32_t expected_id)
{
	RESULT Status = RET_SUCCESS;
	IsiFmc_t *active_fmc = get_active_fmc();

	TRACE(MS_FW_INFO, "%s (enter) pipe %d\r\n", __func__, in_pipe);

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "%s :No FMC selected"
			"for sensor init on pipe %d\r\n",
			__func__, in_pipe);
		return RET_FAILURE;
	}

	u8 i2cbusid = GetI2cBusIdForDes(des_index);

	desInterface *active_des_array = get_active_des_array();

	if (!active_des_array) {
		TRACE(MS_FW_ERROR, "%s :No Des Array selected"
			"for sensor init on pipe %d\r\n",
			__func__, in_pipe);
		return RET_FAILURE;
	}

	TRACE(MS_FW_INFO, "enter %s for pipe %d using FMC: %s with %s array\r\n",
			__func__, in_pipe, active_fmc->FmcName,
			(current_active_fmc == FMC_ID_0) ? "des_arr" : "max96716_des_arr");

	// sensor_handle[in_pipe] = (struct sensor_driver *)osMalloc
	// 	(sizeof(struct sensor_driver));

	// if (sensor_handle[in_pipe] == NULL) {
	// 	TRACE(MS_FW_ERROR, "%s: osMalloc failed for sensor_driver (pipe=%d)\n",
	// 		__func__, in_pipe);
	// 	return RET_OUTOFMEM;
	// }
	memset(&sensor_handle[in_pipe], 0, sizeof(struct sensor_driver));

	active_fmc->sensor_array[in_pipe] = &sensor_handle[in_pipe];
	active_fmc->sensor_array[in_pipe]->pipe_no = in_pipe;


	if (active_des_array[des_index].link_type == NO_LINK) {
		TRACE(MS_FW_ERROR, "SENSOR NOT CONNECTED:\r\n");
		TRACE(MS_FW_ERROR, "  pipe : %d\r\n", in_pipe);
		TRACE(MS_FW_ERROR, "  no GMSL link detected on this port.\r\n");
		report_pipeline_fault("Camera sensor", "link detection",
			"No camera sensor is connected on this port "
			"(no GMSL link detected).",
			"Connect the camera sensor and check the cabling, then "
			"power-cycle the board.");
		Status = XST_DEVICE_NOT_FOUND;
		goto cleanup_sensor;

	} else if (active_des_array[des_index].link_type == LINK_A) {
		TRACE(MS_FW_ALWAYS, "%s: Programming Link-A Device \r\n",
			__func__);
		active_fmc->serializer_array[in_pipe]->alias_addr =
			(active_des_array[des_index].link_a.serializer_alias_addr);
		active_fmc->serializer_array[in_pipe]->bus_num =
		GetI2cBusIdForDes(des_index);
		if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes failed for des_index %d\n",
				__func__, des_index);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		Status = (active_fmc->serializer_array[in_pipe])->
			init_serializer(active_fmc->serializer_array[in_pipe]);
		if (Status != RET_SUCCESS){
			TRACE(MS_FW_ERROR,
				"%s: init_serializer"
				"failed pipe %d (err=%d)\n",
				__func__, in_pipe, Status);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
			(active_des_array[des_index].link_a.sensor_alias_addr);

	} else if (active_des_array[des_index].link_type == LINK_B) {
		TRACE(MS_FW_ALWAYS, "%s: Programming Link-B Device \r\n",
			__func__);
		active_fmc->serializer_array[in_pipe]->alias_addr =
			(active_des_array[des_index].link_b.serializer_alias_addr);
		active_fmc->serializer_array[in_pipe]->bus_num =
		GetI2cBusIdForDes(des_index);
		if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes"
				"failed for des_index %d\n",
				__func__, des_index);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		Status = (active_fmc->serializer_array[in_pipe])->
			init_serializer(active_fmc->serializer_array[in_pipe]);
		if (Status != RET_SUCCESS){
			TRACE(MS_FW_ERROR,
				"%s: init_serializer"
				"failed pipe %d (err=%d)\n",
				__func__, in_pipe, Status);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
			(active_des_array[des_index].link_b.sensor_alias_addr);

	} else if (active_des_array[des_index].link_type == LINK_REVERSE_SPLITTER) {
		if ((in_pipe % 2) == 0) {
			TRACE(MS_FW_ALWAYS, "%s: Programming Link-A Device \r\n", __func__);
			active_fmc->serializer_array[in_pipe]->alias_addr =
				(active_des_array[des_index].link_a.serializer_alias_addr);
			active_fmc->serializer_array[in_pipe]->bus_num =
				GetI2cBusIdForDes(des_index);
			if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
				TRACE(MS_FW_ERROR,
					"%s: GetI2cBusIdForDes failed for des_index %d\n",
					__func__, des_index);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			Status = (active_fmc->serializer_array[in_pipe])->
			init_serializer(active_fmc->serializer_array[in_pipe]);
			if (Status != RET_SUCCESS){
				TRACE(MS_FW_ERROR,
					"%s: init_serializer"
					"failed pipe %d (err=%d)\n",
					__func__, in_pipe, Status);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
				(active_des_array[des_index].link_a.sensor_alias_addr);
		} else {
			TRACE(MS_FW_ALWAYS, "%s: Programming Link-B Device \r\n", __func__);
			active_fmc->serializer_array[in_pipe]->alias_addr =
				(active_des_array[des_index].link_b.serializer_alias_addr);
			active_fmc->serializer_array[in_pipe]->bus_num =
				GetI2cBusIdForDes(des_index);
			if (active_fmc->serializer_array[in_pipe]->bus_num == INVALID_I2C_BUS_ID) {
				TRACE(MS_FW_ERROR, "%s: GetI2cBusIdForDes failed for des_index %d\n",
				__func__, des_index);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			Status = (active_fmc->serializer_array[in_pipe])->
				init_serializer(active_fmc->serializer_array[in_pipe]);
			if (Status != RET_SUCCESS){
				TRACE(MS_FW_ERROR,
					"%s: init_serializer"
					"failed pipe %d (err=%d)\n",
					__func__, in_pipe, Status);
				Status = RET_FAILURE;
				goto cleanup_sensor;
			}
			active_fmc->sensor_array[in_pipe]->sensor_alias_addr =
				(active_des_array[des_index].link_b.sensor_alias_addr);
		}
	}

	if (active_des_array[des_index].link_type != NO_LINK) {
		TRACE(MS_FW_INFO, "Sensor address passed: %x\n", sensor_address);
		u16 reg_addr;
		u8 ser_addr = active_fmc->serializer_array[in_pipe]->alias_addr >> 1;
		u8 i2cBusId = GetI2cBusIdForDes(des_index);

		if (i2cBusId == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes failed for des_index %d\n",
				__func__, des_index);
			Status = RET_FAILURE;
			goto cleanup_sensor;
		}
		u8 wr_data[2] = {0};
		u8 read_data[2] = {0};

		reg_addr = I2C_2;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_2 pipe=%d (err=%d)\n",
					__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "before I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_3 pipe=%d (err=%d)\n",
					__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "before I2C_3 :%x\r\n", read_data[0]);

		reg_addr = I2C_2;
		wr_data[0] = active_fmc->sensor_array[in_pipe]->sensor_alias_addr;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C write failed reg=I2C_2 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}

		reg_addr = I2C_3;
		wr_data[0] = sensor_address;
		Status = HalWriteI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, wr_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C write failed reg=I2C_3 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}

		reg_addr = I2C_2;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_2 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "After I2C_2 :%x\r\n", read_data[0]);

		reg_addr = I2C_3;
		Status = HalReadI2CReg(i2cBusId, ser_addr, reg_addr, 0x2, read_data, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=I2C_3 pipe=%d (err=%d)\n",
				__func__, __LINE__, in_pipe, Status);
			goto cleanup_sensor;
		}
		TRACE(MS_FW_INFO, "After I2C_3 :%x\r\n", read_data[0]);

		/*
		 * Sensor identity check, continued in this same (already
		 * NO_LINK-guarded) block: the serializer is initialised and the
		 * sensor alias has just been programmed, so the sensor is reachable
		 * on the SoC bus at that alias - the earliest point it can be probed.
		 * The connected sensor identifies ITSELF: probe the alias, read the
		 * chip-ID at the address just programmed, and name it from
		 * sensorCfgList. No configured/expected name is used.
		 */
		char detected_name[16] = {0};

		if (sensor_address == 0)
			goto sensor_check_done;

		Status = check_sensor_id(i2cbusid, ser_addr,
				 active_fmc->sensor_array[in_pipe]->sensor_alias_addr,
				 (u8)sensor_address, expected_id,
				 detected_name, sizeof(detected_name));
		if (Status != RET_SUCCESS)
			goto cleanup_sensor;

		strncpy(active_fmc->sensor_array[in_pipe]->name, detected_name,
			sizeof(active_fmc->sensor_array[in_pipe]->name) - 1);
		active_fmc->sensor_array[in_pipe]->name[
			sizeof(active_fmc->sensor_array[in_pipe]->name) - 1] = '\0';

		/*
		 * Publish this link's validated sensor name into the SHARED
		 * deserializer topology so EVERY core's summary can show it, not just
		 * the core that launched this pipeline. Write it into the link this
		 * pipe actually programmed (LINK_A/LINK_B use that link; a splitter
		 * uses even->Link-A, odd->Link-B), then copy just THIS deserializer's
		 * entry to shared memory - each core owns distinct deserializers, so
		 * no other core's entry is clobbered.
		 */
		{
			u8 lt = active_des_array[des_index].link_type;
			dslink *lk;

			if (lt == LINK_A)
				lk = &active_des_array[des_index].link_a;
			else if (lt == LINK_B)
				lk = &active_des_array[des_index].link_b;
			else
				lk = ((in_pipe % 2) == 0)
					? &active_des_array[des_index].link_a
					: &active_des_array[des_index].link_b;

			strncpy(lk->sensor_name, detected_name,
				sizeof(lk->sensor_name) - 1);
			lk->sensor_name[sizeof(lk->sensor_name) - 1] = '\0';
			lk->owner_core = cpu_id;
			/*
			 * This link is now launched/validated - clear any prior
			 * "killed" mark so a re-launched pipeline shows "active"
			 * again rather than staying "killed".
			 */
			lk->killed = 0;

			byte_memcpy((u8 *)SHARED_DES_ARRAY_SRUCT
					+ des_index * sizeof(desInterface),
				&active_des_array[des_index],
				sizeof(desInterface));
		}
	}

sensor_check_done:

	/*
	 * Print the connected-sensor summary once the sensor I2C alias has been
	 * remapped for this des/pipe. Doing it here (instead of from the FMC
	 * drivers) guarantees the alias is programmed, so the summary's I2C
	 * accesses succeed for every des remap. Summarise ALL deserializers
	 * remapped this run, not just this pipe's, so the operator gets the full
	 * picture.
	 */
	print_all_sensor_summary();

	TRACE(MS_FW_INFO, "%s (exit) pipe %d\r\n", __func__, in_pipe);
	return RET_SUCCESS;

cleanup_sensor:
	// osFree(sensor_handle[in_pipe]);
	// sensor_handle[in_pipe] = NULL;
	active_fmc->sensor_array[in_pipe] = NULL;
	// if (accessiic_handle[in_pipe]) {
	// 	osFree(accessiic_handle[in_pipe]);
	// 	accessiic_handle[in_pipe] = NULL;
	// 	active_fmc->accessiic_array[in_pipe] = NULL;
	// }
	return Status;
}

/**************************************************************************
 *          mark_pipe_killed
 *
 * @brief   Flag a pipeline as "killed" in the (shared) deserializer topology
 *          so the connected-sensor summary shows it as "killed". This is a
 *          plain per-link DISPLAY flag on the summary's own data - NOT a
 *          des/ser/sensor state machine. The flag persists in shared memory
 *          until the pipe is (re)launched, when init_sensor clears it. It only
 *          updates shared memory; it does NOT print the summary (that happens
 *          on the next pipeline launch).
 *
 * @param   in_pipe   Input pipeline index (even = Link-A, odd = Link-B)
 *
 * @return  None
 *
 ************************************************************************/
void mark_pipe_killed(int in_pipe)
{
	desInterface *active_des_array = get_active_des_array();
	int des_index = MAPPING_INPIPE_TO_DES_ID(in_pipe);
	dslink *lk;

	TRACE(MS_FW_INFO, "%s (enter) pipe %d\r\n", __func__, in_pipe);

	if (active_des_array == NULL) {
		TRACE(MS_FW_ERROR, "%s: no active des array\r\n", __func__);
		return;
	}

	if (des_index < 0 || des_index >= DS_MAX) {
		TRACE(MS_FW_ERROR, "%s: pipe %d maps to invalid des %d\r\n",
			__func__, in_pipe, des_index);
		return;
	}

	lk = ((in_pipe % 2) == 0)
		? &active_des_array[des_index].link_a
		: &active_des_array[des_index].link_b;

	lk->killed = 1;
	lk->owner_core = cpu_id;

	/*
	 * Publish just this deserializer's entry to the shared topology so every
	 * core's summary reflects the new status. Each core owns distinct
	 * deserializers, so no other core's entry is clobbered.
	 */
	byte_memcpy((u8 *)SHARED_DES_ARRAY_SRUCT
			+ des_index * sizeof(desInterface),
		&active_des_array[des_index],
		sizeof(desInterface));

	TRACE(MS_FW_INFO, "%s (exit) pipe %d\r\n", __func__, in_pipe);
}

/**************************************************************************
 *          verify_deser_identity
 *
 * @brief   Common "right deserializer?" check, shared by BOTH FMC drivers
 *          (MAX9296 and MAX96716) so the logic lives in one place. Each driver
 *          simply calls this with its own bus and (optional) address mask.
 *
 *          IDENTIFY THE RIGHT DES BY ITS NATIVE SLAVE ADDRESS (register 0x00):
 *          the device-address register must read back the Des native address
 *          that des_arr assigns to THIS desId, taken straight from the FMC
 *          header (MAX9296_DSx_DEFAULT_ADDRESS / MAX96716_DSx_DEFAULT_ADDRESS).
 *          MAX96716 Des alternate 0x50 / 0x54 and MAX9296 Des are grouped
 *          (0xD0/0x94/0xD8), so a mismatch between the address expected for
 *          THIS Des index and the address actually read means the WRONG Des is
 *          being remapped - a hard FAILURE. This read is also the accessibility
 *          probe: if the Des does not ACK at its default address, the
 *          deserializer for this sensorDevId is absent/unreachable. Uses only
 *          the hardware's own address register, so it does NOT depend on
 *          core_des_map. (The DEV_ID register 0x0D is deliberately NOT used -
 *          it does not reliably distinguish the fitted deserializer.)
 *
 *          On a Des slave-address mismatch the call FAILS (RET_WRONG_CONFIG)
 *          so the FMC driver aborts the remap. If the address register is
 *          simply not readable (Des absent), it only TRACEs and continues (a
 *          genuinely dead Des then fails downstream on its next I2C access).
 *          The summary is NOT touched here - a remapped-but-not-launched Des
 *          shows as "pending"; "killed" is reserved for a launched pipeline
 *          later stopped via stop_sensor().
 *
 * @param   i2cBusId        I2C bus the deserializer is reached on
 * @param   desIface        Deserializer interface (holds the default address)
 * @param   fmc_name        FMC name for the operator messages (e.g. "MAX9296")
 * @param   addr_mask       Device-address compare mask (0 to skip the check)
 *
 * @return  RESULT
 * @retval  RET_SUCCESS       Right Des confirmed, or the address register was
 *                            unreadable and TRACEd (bring-up continues)
 * @retval  RET_WRONG_CONFIG  Des slave-address mismatch - the wrong
 *                            deserializer is being remapped (caller aborts)
 *
 ************************************************************************/
RESULT verify_deser_identity(u8 i2cBusId, desInterface *desIface,
			     const char *fmc_name, u8 addr_mask)
{
	RESULT Status;
	u8 read_data[2] = {0};
	u8 des_7bit_addr;
	u8 des_num;

	TRACE(MS_FW_INFO, "%s (enter) %s\r\n", __func__, fmc_name);

	if (desIface == NULL) {
		TRACE(MS_FW_ERROR, "%s: NULL deserializer interface\r\n", __func__);
		return RET_FAILURE;
	}

	des_7bit_addr = (desIface->des_actual_addr >> 1);
	des_num = desIface->Port_DES_index + 1;

	/*
	 * IDENTIFY THE RIGHT DES BY ITS NATIVE SLAVE ADDRESS (register 0x00).
	 * The device-address register must read back the Des native address that
	 * des_arr assigns to THIS desId - taken straight from the FMC header
	 * (MAX9296_DSx_DEFAULT_ADDRESS / MAX96716_DSx_DEFAULT_ADDRESS). A
	 * different address means a different Des is wired/enabled at this slot
	 * than was selected, i.e. the WRONG Des is being remapped. This read is
	 * also the accessibility probe. Uses only the hardware's own address
	 * register (NOT core_des_map). addr_mask ignores the R/W LSB; 0 skips
	 * (parts with no readable address register only).
	 */
	Status = HalReadI2CReg(i2cBusId, des_7bit_addr, DEV_ADDR_REG, 0x2,
			read_data, 1);
	if (Status != RET_SUCCESS) {
		TRACE(MS_FW_ERROR,
			"%s DS_%d not accessible at 0x%02x on bus %d (err=%d) - the "
			"deserializer selected for this sensorDevId is absent/unreachable "
			"(likely a WRONG sensorDevId).\r\n",
			fmc_name, des_num, desIface->des_actual_addr, i2cBusId, Status);
		return RET_SUCCESS;
	}

	if (addr_mask != 0 &&
	    (read_data[0] & addr_mask) != (desIface->des_actual_addr & addr_mask)) {
		TRACE(MS_FW_ERROR,
			"%s DS_%d WRONG DESERIALIZER being remapped: this slot answers "
			"with native address 0x%02x but DS_%d is defined at 0x%02x - a "
			"different Des is selected than intended. Aborting remap.\r\n",
			fmc_name, des_num, read_data[0], des_num,
			desIface->des_actual_addr);
		return RET_WRONG_CONFIG;
	}

	TRACE(MS_FW_ALWAYS,
		"%s right deserializer DS_%d (native 0x%02x) selected and being "
		"remapped.\r\n",
		fmc_name, des_num, desIface->des_actual_addr);

	TRACE(MS_FW_INFO, "%s (exit) DS_%d\r\n", __func__, des_num);
	return RET_SUCCESS;
}


/**************************************************************************
 *          stop_sensor
 *
 * @brief   Stop / tear down the specified sensor pipeline and flag it "killed"
 *          in the (shared) summary topology. The summary is deliberately NOT
 *          printed here - the killed pipe just carries its "killed" flag until
 *          the NEXT pipeline launch, whose init_sensor prints the summary and
 *          shows this pipe as "killed" while the newly launched one is "active".
 *
 * @param   in_pipe       Input pipeline index to stop
 *
 * @return  None
 *
 ************************************************************************/
void stop_sensor(int in_pipe)
{
	TRACE(MS_FW_ALWAYS, "%s: stopping pipeline %d\r\n", __func__, in_pipe);
	mark_pipe_killed(in_pipe);
}

/**************************************************************************
 *          sensor_status
 *
 * @brief   Display status information for the specified sensor
 *
 * @param   index         Sensor index to display status for
 *
 * @return  None
 *
 ************************************************************************/
void sensor_status(int index)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	TRACE(MS_FW_INFO, "%s (enter)\r\n", __func__);

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "Error: No active FMC selected for %s\r\n", __func__);
		return;
	}

	if (((active_fmc->sensor_array[index])->sensor_state == in_stop) ||
		((active_fmc->sensor_array[index])->sensor_state == in_running)) {

	} else {
		/*Yet to Impliment*/
	}

	TRACE(MS_FW_INFO, "%s (exit)\r\n", __func__);
}

/**************************************************************************
 *          des_stats
 *
 * @brief   Display deserializer statistics for the specified index
 *
 * @param   des_index     Deserializer index to display statistics for
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 ************************************************************************/
RESULT des_stats(int des_index)
{
	u8 des_addr, i2cBusId;
	u8 pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	RESULT Status = RET_SUCCESS;
	desInterface *active_des_array = get_active_des_array();

	if (!active_des_array) {
		TRACE(MS_FW_ERROR, "%s: No Des Array selected for des_index %d\r\n",
			__func__, des_index);
		return RET_FAILURE;
	}

	if (active_des_array[des_index].des_state == in_running) {
		des_addr = (active_des_array[des_index].des_alias_addr) >> 1;
		i2cBusId = GetI2cBusIdForDes(active_des_array[des_index].Port_DES_index);
		if (i2cBusId == INVALID_I2C_BUS_ID) {
			TRACE(MS_FW_ERROR,
				"%s: GetI2cBusIdForDes failed\r for des_index %d\n",
				__func__, des_index);
			return RET_FAILURE;
		}

		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_X_STATUS, 2, &pipe_x_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_X_STATUS, des_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_Y_STATUS, 2, &pipe_y_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_Y_STATUS, des_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_Z_STATUS, 2, &pipe_z_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_Z_STATUS, des_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, des_addr,
				DES_REG_PIPE_U_STATUS, 2, &pipe_u_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x des=%d (err=%d)\n",
					__func__, __LINE__, DES_REG_PIPE_U_STATUS, des_index, Status);
			return Status;
		}
		TRACE(MS_FW_INFO,
			"Des[%x] Pipe Status :"
				" pipe_x =\r 0x%x, pipe_y = 0x%x, pipe_z = 0x%x, pipe_u = 0x%x\r\n",
				active_des_array[des_index].Port_DES_index + 1, pipe_x_s,
				pipe_y_s, pipe_z_s, pipe_u_s);
	}

	return Status;
}

/**************************************************************************
 *          ser_stats
 *
 * @brief   Display serializer statistics for the specified index
 *
 * @param   ser_index     Serializer index to display statistics for
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS   Operation completed successfully
 * @retval  RET_FAILURE   Operation failed
 *
 ************************************************************************/
RESULT ser_stats(int ser_index)
{
	IsiFmc_t *active_fmc = get_active_fmc();

	if (!active_fmc) {
		TRACE(MS_FW_ERROR, "Error: No active FMC selected for %s\r\n", __func__);
		return RET_FAILURE;
	}

	u8 ser_addr, i2cBusId;
	u8 pipe_x_s = 0, pipe_y_s = 0, pipe_z_s = 0, pipe_u_s = 0;
	RESULT Status = RET_SUCCESS;

	if (active_fmc->serializer_array[ser_index]->ser_state == in_running) {
		ser_addr = (active_fmc->serializer_array[ser_index]->alias_addr) >> 1;
		i2cBusId = active_fmc->serializer_array[ser_index]->bus_num;

		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_X_STATUS, 2, &pipe_x_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_X_STATUS, ser_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_Y_STATUS, 2, &pipe_y_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_Y_STATUS, ser_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_Z_STATUS, 2, &pipe_z_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_Z_STATUS, ser_index, Status);
			return Status;
		}
		Status = HalReadI2CReg(i2cBusId, ser_addr,
				SER_REG_PIPE_U_STATUS, 2, &pipe_u_s, 1);
		if (Status != RET_SUCCESS) {
			TRACE(MS_FW_ERROR,
				"%s:%d I2C read failed reg=0x%x ser=%d (err=%d)\n",
					__func__, __LINE__, SER_REG_PIPE_U_STATUS, ser_index, Status);
			return Status;
		}
		TRACE(MS_FW_INFO, "ser[%x]: Px:0x%x, Py: 0x%x, Pz: 0x%x, Pu: 0x%x\n",
				ser_index + 1, pipe_x_s, pipe_y_s, pipe_z_s, pipe_u_s);
	}

	return Status;
}

/**************************************************************************
 *          Fmc_Sensor_Statustask
 *
 * @brief   Display status for all deserializers and serializers in the system
 *
 * @return  None
 *
 ************************************************************************/
void Fmc_Sensor_Statustask(void)
{
	int des_index, ser_index;

	for (des_index = 0; des_index < DS_MAX; des_index++)
		des_stats(des_index);
	for (ser_index = 0; ser_index < IN_PIPE_LAST; ser_index++)
		ser_stats(ser_index);
}
