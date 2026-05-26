
#ifndef MEMORY_LAYOUT_H
#define MEMORY_LAYOUT_H

#include <stdint.h>

/* HAL reserved memory */
extern uint32_t HAL_RESERVED_MEM_PRIV_START;
extern uint32_t HAL_RESERVED_MEM_PRIV_SIZE;

/* VSI MMB memory */
extern uint32_t VSI_MMB_RESERVED_MEM_OPEN_START;
extern uint32_t VSI_MMB_RESERVED_MEM_OPEN_SIZE;

/* Trace memory */
extern uint32_t TRACE_LOG_START_ADDR;

/* Constants */
#define VSI_MMB_OPEN_REGION_SIZE   (0x2000U)
#define TRACE_SHARED_MEM_OFFSET    (0x1000U)

/* Helpers */
static inline uint32_t get_trace_entry_base(void)
{
    return TRACE_LOG_START_ADDR + TRACE_SHARED_MEM_OFFSET;
}

#endif
