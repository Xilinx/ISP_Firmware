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
#ifndef __MISC_H__
#define __MISC_H__

#ifndef __FLT_EPSILON__
#define __FLT_EPSILON__		(0.000000119209289550781250000000)
#endif

#ifndef FLT_EPSILON
#define FLT_EPSILON		__FLT_EPSILON__
#endif

#ifndef FLT_MAX
#define FLT_MAX			((float)3.40282346638528860e+38)
#endif

#ifndef MIN
#define MIN(a, b)		(((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b)		(((a) > (b)) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(a)			(((a) < 0) ? -(a) : (a))
#endif

#ifndef SIGN
#define SIGN(a)			(((a) < 0) ? -1 : ((a) > 0) ? 1 : 0)
#endif

#define ARRAY_SIZE(arr)		(sizeof(arr) / sizeof((arr)[0]))

#endif
