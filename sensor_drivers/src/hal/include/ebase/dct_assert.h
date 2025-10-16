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

#ifndef ASSERT_H_
#define ASSERT_H_

#define RET_FAILURE		(1)

/*****************************************************************************/
/**
 *          ASSERT_HANDLER
 *
 * @brief   Function pointer type for custom assertion failure handlers
 *
 * @note    The assert handler is a function that is called when an assertion
 *          fails. It must not return (marked with noreturn attribute).
 *          If no custom handler is registered, the default exit_() function
 *          is called.
 *
 *****************************************************************************/
typedef void (*ASSERT_HANDLER)(void) __attribute__((noreturn));

/*****************************************************************************/
/**
 *          assert_handler
 *
 * @brief   Global assertion handler function pointer
 *
 * @note    This variable holds a pointer to the current assertion handler
 *          function. By default, it is NULL, which means the default
 *          exit_() function will be used. Users can register a custom
 *          handler by assigning to this variable.
 *
 *****************************************************************************/
extern ASSERT_HANDLER   assert_handler;

#if defined(ENABLE_ASSERT) || !defined(NDEBUG)
/*****************************************************************************/
/**
 *          exit_
 *
 * @brief   Default assertion failure handler that dumps debug information
 *          and terminates the program
 *
 * @param   file    Filename where the assertion occurred
 * @param   line    Line number where the assertion occurred
 *
 * @note    This function is called when an assertion fails and no custom
 *          handler is registered. It outputs diagnostic information to
 *          stderr and then exits the program.
 *
 * @warning This function never returns (marked with noreturn attribute)
 *
 *****************************************************************************/
#ifdef __cplusplus
extern "C"
#endif
void exit_(const char *file, int line) __attribute__((noreturn));

/*****************************************************************************/
/**
 *          DCT_ASSERT
 *
 * @brief   Macro for runtime assertion checking with debug information
 *
 * @param   exp    Expression that is expected to be true
 *
 * @note    When ENABLE_ASSERT is defined or NDEBUG is not defined, this
 *          macro evaluates the expression and calls exit_() with file
 *          and line information if the expression is false. In release
 *          builds (NDEBUG defined and ENABLE_ASSERT not defined), this
 *          macro expands to a no-op.
 *
 * @warning If the assertion fails, the program will terminate
 *
 * @example
 *          DCT_ASSERT(pointer != NULL);
 *          DCT_ASSERT(value > 0 && value < MAX_VALUE);
 *
 *****************************************************************************/
#define DCT_ASSERT(exp)			do { if (!(exp)) { static CHAR filename[] = __FILE__;	\
					exit_(&filename[0], __LINE__);				\
					} } while (0)

#else
/*****************************************************************************/
/**
 *          DCT_ASSERT (Release Version)
 *
 * @brief   No-operation assertion macro for release builds
 *
 * @param   exp    Expression (evaluated but ignored in release builds)
 *
 * @note    In release builds (when NDEBUG is defined and ENABLE_ASSERT
 *          is not defined), this macro expands to a do-nothing loop that
 *          the compiler will optimize away. The expression is still
 *          evaluated to avoid unused variable warnings.
 *
 *****************************************************************************/
#define DCT_ASSERT(exp)			\
	do {				\
		if ((exp)) {		\
		} else {		\
		}			\
	} while (0)
#endif

#endif
