/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 VeriSilicon Holdings Co., Ltd. ("VeriSilicon")
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

enum {
	TRACE_OFF		= 0x00,
	INFO			= 0x01,
	WARNING			= 0x02,
	ERROR			= 0x04,
	DEBUG			= 0x08,
	MAX_LEVEL		= (0x01U | 0x02U | 0x04U | 0x08U)
};

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
void trace(Tracer *, const CHAR *, ...);
Tracer *getTracerList(void);

#if !defined(USE_SDRAM_FOR_TRACE)
#define TRACER_DATA
#else
#define TRACER_DATA		DRAM_DATA
#endif

/*****************************************************************************
 *
 *          This macro creates a Tracer. Every Tracer has its own output like
 *          stdout or a file and a trace level associated. A tracer may be
 *          enabled and disabled. If the global trace level is lower than the
 *          Tracers level, than output send to an enabled tracer is output to
 *          its file or stdout. If eg the global trace level is INFO and the
 *          Tracer has the level WARNING, its output is active. If the global
 *          trace level were ERROR instead, output of the tracer were
 *          suppressed.
 *
 *          \warning THIS MACRO MUST BE USED IN GLOBAL SCOPE.
 *
 *
 *  @param    name          Name of the tracer.
 *  @param    arg_prefix    All output of this tracer is preceeded by
 *                          <arg_prefix>.
 *  @param    arg_level     Initial trace level.
 *  @param    arg_enabled   Decide wether the tracer starts in enabled state.
 *
 *  @return   No return value.
 *
 *****************************************************************************/
#define CREATE_TRACER(name, arg_prefix, arg_level, arg_enabled)		\
	CHAR tracerName##name[] TRACER_DATA = #name; \
	CHAR tracerPrefix##name[] TRACER_DATA = arg_prefix; \
	Tracer instance__##name TRACER_DATA =	\
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

    /*****************************************************************************
     *
     *              If tracer was created in another compile unit this macro
     *              makes the named tracer available in the current unit.
     *
     *  @param      ...     First parameter is name of tracer.Use variable argument
     *                      list like printf.
     *  @return     No return value.
     *
     *****************************************************************************/
#define USE_TRACER(name)	(extern Tracer *name)

    /*****************************************************************************
     *
     *              Send output to a tracer.
     *
     *  @param      ...     First parameter is name of tracer.Use variable argument
     *                      list like printf.
     *  @return             No return value.
     *
     *****************************************************************************/
#define TRACE(...)		trace(__VA_ARGS__)

    /*****************************************************************************
     *
     *              Send output to a tracer, If DEBUG_LEVEL if high enough
     *
     *  @param      ...     First parameter is the required DEBUG_LEVEL to get the
     *                      output, second parameter is name of tracer. Use variable argument
     *                      list like printf.
     *  @return             No return value.
     *
     *****************************************************************************/
#if defined(DEBUG_LEVEL)
#define DL_TRACE(level, ...)		(if (DEBUG_LEVEL >= level) { trace(__VA_ARGS__); })
#else
#define DL_TRACE(level, ...)		((void)0)
#endif

    /*****************************************************************************
     *
     *              Enable a tracer.
     *
     *  @param      T   name of tracer.
     *  @return     No return value.
     *
     *****************************************************************************/
#define ENABLE_TRACER(T)		(enableTracer(T))

    /*****************************************************************************
     *
     *              Disable a tracer.
     *
     *  @param      T   name of tracer.
     *  @return     No return value.
     *
     *****************************************************************************/
#define DISABLE_TRACER(T)		(disableTracer(T))

    /*****************************************************************************
     *
     *              Set the global trace level.
     *
     *  @param      L   Trace level.
     *  @return     No return value.
     *
     *****************************************************************************/
#define SET_TRACE_LEVEL(L)		(setTraceLevel(L))

    /*****************************************************************************
     *
     *              Redirect a tracer to a file.
     *
     *  @param      T   name of tracer.
     *  @param      F   A valid FILE*.
     *
     *  @return     No return value.
     *
     *****************************************************************************/
#define SET_TRACER_FILE(T, F)		(setTracerFile(T, F))

    /*****************************************************************************
     *
     *              Flush a tracer.
     *
     *  @param      T   name of tracer.
     *
     *  @return     No return value.
     *
     *****************************************************************************/
#define FLUSH_TRACER(T)			(flushTracer(T))

#define GET_TRACE_LEVEL()		(getTraceLevel())
#define GET_TRACER_LIST()		(getTracerList())
#define IF_TRACE_ON(x)			x
#else
#define CREATE_TRACER(name, arg_prefix, arg_level, arg_enabled)		extern int32_t name
#define USE_TRACER(name)		(extern int32_t use##name)
#define TRACE(...)			((void)0)
#define DL_TRACE(level, ...)		((void)0)
#define ENABLE_TRACER(T)		((void)0)
#define DISABLE_TRACER(T)		((void)0)
#define SET_TRACE_LEVEL(L)		((void)0)
#define SET_TRACER_FILE(T, F)		((void)0)
#define FLUSH_TRACER(T)			((void)0)
#define GET_TRACE_LEVEL()		((void)0)
#define GET_TRACER_LIST()		((void)0)
#define IF_TRACE_ON(x)
#endif

#ifdef __cplusplus
}
#endif

#endif
