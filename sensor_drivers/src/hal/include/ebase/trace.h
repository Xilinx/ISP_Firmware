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

#ifndef TRACE_H_
#define TRACE_H_

#include "linux_compat.h"
#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef ERROR
#undef ERROR
#endif

/*****************************************************************************/
/**
 *          TraceLevel_e
 *
 * @brief   Trace level enumeration for controlling trace output verbosity
 *
 * @note    These constants define the available trace levels. Multiple levels
 *          can be combined using bitwise OR operations. MAX_LEVEL represents
 *          all levels enabled.
 *
 *****************************************************************************/
enum {
	TRACE_OFF		= 0x00,
	INFO			= 0x01,
	WARNING			= 0x02,
	ERROR			= 0x04,
	DEBUG			= 0x08,
	ALWAYS			= 0x10,
	MAX_LEVEL		= (0x01U | 0x02U | 0x04U | 0x08U)
};

/*****************************************************************************/
/**
 *          Tracer
 *
 * @brief   Structure representing a tracer instance with output configuration
 *
 * @note    This structure contains all the information needed to manage
 *          a tracer including output file, prefix, level, and state.
 *
 *****************************************************************************/
typedef struct {
	FILE			*fp;
	char_t			*prefix;
	int16_t			level;
	int8_t			enabled;
	int8_t			linked;
	char_t			*name;
	struct tracer_s		*next;
} Tracer;

#ifndef NDEBUG
int getTraceLevel(void);
void setTraceLevel(int);
void enableTracer(Tracer *);
void disableTracer(Tracer *);
void setTracerFile(Tracer *, FILE *);
void flushTracer(const Tracer *);
void trace(uint8_t, Tracer *, const CHAR *, ...);
Tracer *getTracerList(void);

/*****************************************************************************/
/**
 *          TRACER_DATA
 *
 * @brief   Macro to define tracer data storage location based on configuration
 *
 * @note    When USE_SDRAM_FOR_TRACE is defined, tracer data is stored in
 *          SDRAM using DRAM_DATA attribute. Otherwise, data is stored in
 *          default memory sections.
 *
 *****************************************************************************/
#if !defined(USE_SDRAM_FOR_TRACE)
#define TRACER_DATA
#else
#define TRACER_DATA		DRAM_DATA
#endif

/*****************************************************************************/
/**
 *          CREATE_TRACER
 *
 * @brief   Macro to create and initialize a new tracer instance
 *
 * @param   name         Name identifier for the tracer
 * @param   arg_prefix   String prefix prepended to all tracer output
 * @param   arg_level    Initial trace level (INFO, WARNING, ERROR, DEBUG)
 * @param   arg_enabled  Initial enabled state (0=disabled, 1=enabled)
 *
 * @note    This macro creates a tracer with its own output destination and
 *          trace level. A tracer may be enabled/disabled independently.
 *          Output is generated only if both the tracer is enabled AND the
 *          global trace level permits the tracer's level.
 *
 * @warning THIS MACRO MUST BE USED IN GLOBAL SCOPE
 *
 *****************************************************************************/
#define CREATE_TRACER(name, arg_prefix, arg_level, arg_enabled)			\
			CHAR tracerName##name[] TRACER_DATA = #name;		\
			CHAR tracerPrefix##name[] TRACER_DATA = arg_prefix;	\
			Tracer instance__##name TRACER_DATA =			\
			{					\
				0,				\
				&tracerPrefix##name[0],		\
				arg_level,			\
				arg_enabled,			\
				0,				\
				&tracerName##name[0],		\
				NULL				\
			};					\
			Tracer *name = &instance__##name

/*****************************************************************************/
/**
 *          USE_TRACER
 *
 * @brief   Macro to declare an external tracer created in another compilation unit
 *
 * @param   name    Name of the tracer to import
 *
 * @note    Use this macro to access a tracer that was created with
 *          CREATE_TRACER in a different source file.
 *
 *****************************************************************************/
#define USE_TRACER(name)	(extern Tracer *name)

/*****************************************************************************/
/**
 *          TRACE
 *
 * @brief   Macro to send formatted output to a specified tracer
 *
 * @param   ...    Variable argument list: first parameter is tracer name,
 *                 followed by printf-style format string and arguments
 *
 * @note    Output is generated only if the tracer is enabled and the
 *          global trace level permits the tracer's configured level.
 *
 *****************************************************************************/
#define USE_XIL_PRINTF			(0)
#define USE_VPRINTF			(1)

#define TRACE(...)		        trace(USE_XIL_PRINTF, __VA_ARGS__)
#define TRACE_FLOAT(...)		trace(USE_VPRINTF, __VA_ARGS__)

/*****************************************************************************/
/**
 *          DL_TRACE
 *
 * @brief   Macro for conditional trace output based on debug level
 *
 * @param   level    Minimum DEBUG_LEVEL required for output
 * @param   ...      Variable argument list: tracer name followed by
 *                   printf-style format string and arguments
 *
 * @note    Output is generated only if the compiled DEBUG_LEVEL is equal
 *          to or greater than the specified level parameter.
 *          If DEBUG_LEVEL is not defined, this macro becomes a no-op.
 *
 *****************************************************************************/
#if defined(DEBUG_LEVEL)
#define DL_TRACE(level, ...)		do { if (level <= DEBUG_LEVEL) \
						trace(USE_XILPRINTF, __VA_ARGS__); \
					} while (0)
#define DL_TRACE_FLOAT(level, ...)	do { if (level <= DEBUG_LEVEL) \
						trace(USE_VPRINTF, __VA_ARGS__); \
					} while (0)
#else
#define DL_TRACE(level, ...)		((void)0)
#define DL_TRACE_FLOAT(level, ...)	((void)0)
#endif

/*****************************************************************************/
/**
 *          ENABLE_TRACER
 *
 * @brief   Macro to enable output for a specified tracer
 *
 * @param   T    Name of the tracer to enable
 *
 * @note    Enabling a tracer allows it to generate output when TRACE
 *          is called, subject to the global trace level settings.
 *
 *****************************************************************************/
#define ENABLE_TRACER(T)		(enableTracer(T))

/*****************************************************************************/
/**
 *          DISABLE_TRACER
 *
 * @brief   Macro to disable output for a specified tracer
 *
 * @param   T    Name of the tracer to disable
 *
 * @note    Disabling a tracer prevents it from generating any output
 *          when TRACE is called, regardless of global trace level.
 *
 *****************************************************************************/
#define DISABLE_TRACER(T)		(disableTracer(T))

/*****************************************************************************/
/**
 *          SET_TRACE_LEVEL
 *
 * @brief   Macro to set the global trace level threshold
 *
 * @param   L    Trace level (INFO, WARNING, ERROR, DEBUG, or OFF)
 *
 * @note    Only tracers with levels at or above this threshold will
 *          generate output. Lower priority messages will be filtered out.
 *
 *****************************************************************************/
#define SET_TRACE_LEVEL(L)		(setTraceLevel(L))

/*****************************************************************************/
/**
 *          SET_TRACER_FILE
 *
 * @brief   Macro to redirect tracer output to a specific file
 *
 * @param   T    Name of the tracer to redirect
 * @param   F    File pointer (FILE*) for output destination
 *
 * @note    Redirects all output from the specified tracer to the given
 *          file. Use stdout/stderr for console output, or a file opened
 *          with fopen() for file output.
 *
 *****************************************************************************/
#define SET_TRACER_FILE(T, F)		(setTracerFile(T, F))

/*****************************************************************************/
/**
 *          FLUSH_TRACER
 *
 * @brief   Macro to flush pending output for a specified tracer
 *
 * @param   T    Name of the tracer to flush
 *
 * @note    Forces any buffered output from the tracer to be written
 *          immediately to its output destination. Useful for ensuring
 *          critical messages are not lost if the program terminates.
 *
 *****************************************************************************/
#define FLUSH_TRACER(T)			(flushTracer(T))

/*****************************************************************************/
/**
 *          GET_TRACE_LEVEL
 *
 * @brief   Macro to retrieve the current global trace level
 *
 * @return  Current trace level setting (INFO, WARNING, ERROR, DEBUG, or OFF)
 *
 * @note    Returns the threshold level that determines which tracer
 *          output will be displayed. Can be used for conditional logic.
 *
 *****************************************************************************/
#define GET_TRACE_LEVEL()		(getTraceLevel())

/*****************************************************************************/
/**
 *          GET_TRACER_LIST
 *
 * @brief   Macro to retrieve a pointer to the global tracer list
 *
 * @return  Pointer to the linked list of all registered tracers
 *
 * @note    Returns a pointer to the internal tracer list structure.
 *          Can be used to iterate through all tracers for management
 *          or debugging purposes.
 *
 *****************************************************************************/
#define GET_TRACER_LIST()		(getTracerList())

/*****************************************************************************/
/**
 *          IF_TRACE_ON
 *
 * @brief   Conditional compilation macro for trace-enabled code
 *
 * @param   x    Code block to include when tracing is enabled
 *
 * @note    When TRACE_ON is defined, the code in parameter x is compiled.
 *          When tracing is disabled, the code is completely removed.
 *          Use this to avoid overhead of trace-related computations
 *          in release builds.
 *
 *****************************************************************************/
#define IF_TRACE_ON(x)			x
#else
    /*****************************************************************************
     *
     *          CREATE_TRACER
     *
     * @brief   Dummy tracer creation macro when tracing is disabled.
     *
     *****************************************************************************/
#define CREATE_TRACER(name, arg_prefix, arg_level, arg_enabled)		extern int32_t name

/*****************************************************************************
 *
 *          USE_TRACER
 *
 * @brief   Dummy tracer usage macro when tracing is disabled.
 *
 *****************************************************************************/
#define USE_TRACER(name)		(extern int32_t use##name)

/*****************************************************************************
 *
 *          TRACE
 *
 * @brief   Dummy trace macro when tracing is disabled.
 *
 *****************************************************************************/
#define TRACE(...)			((void)0)

/*****************************************************************************
 *
 *          DL_TRACE
 *
 * @brief   Dummy debug level trace macro when tracing is disabled.
 *
 *****************************************************************************/
#define DL_TRACE(level, ...)		((void)0)

/*****************************************************************************
 *
 *          ENABLE_TRACER
 *
 * @brief   Dummy tracer enable macro when tracing is disabled.
 *
 *****************************************************************************/
#define ENABLE_TRACER(T)		((void)0)

/*****************************************************************************
 *
 *          DISABLE_TRACER
 *
 * @brief   Dummy tracer disable macro when tracing is disabled.
 *
 *****************************************************************************/
#define DISABLE_TRACER(T)		((void)0)

/*****************************************************************************
 *
 *          SET_TRACE_LEVEL
 *
 * @brief   Dummy trace level set macro when tracing is disabled.
 *
 *****************************************************************************/
#define SET_TRACE_LEVEL(L)		((void)0)

/*****************************************************************************
 *
 *          SET_TRACER_FILE
 *
 * @brief   Dummy tracer file set macro when tracing is disabled.
 *
 *****************************************************************************/
#define SET_TRACER_FILE(T, F)		((void)0)

/*****************************************************************************
 *
 *          FLUSH_TRACER
 *
 * @brief   Dummy tracer flush macro when tracing is disabled.
 *
 *****************************************************************************/
#define FLUSH_TRACER(T)			((void)0)

/*****************************************************************************
 *
 *          GET_TRACE_LEVEL
 *
 * @brief   Dummy trace level get macro when tracing is disabled.
 *
 *****************************************************************************/
#define GET_TRACE_LEVEL()		((void)0)

/*****************************************************************************
 *
 *          GET_TRACER_LIST
 *
 * @brief   Dummy tracer list get macro when tracing is disabled.
 *
 *****************************************************************************/
#define GET_TRACER_LIST()		((void)0)

/*****************************************************************************
 *
 *          IF_TRACE_ON
 *
 * @brief   Dummy conditional compilation macro when tracing is disabled.
 *
 *****************************************************************************/
#define IF_TRACE_ON(x)
#endif

#ifdef __cplusplus
}
#endif

#endif
