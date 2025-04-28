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
#ifndef __RETURN_CODES_H__
#define __RETURN_CODES_H__

typedef int RESULT;

#define RET_SUCCESS			(0)
#define RET_FAILURE			(1)
#define RET_NOTSUPP			(2)
#define RET_BUSY			(3)
#define RET_CANCELED			(4)
#define RET_OUTOFMEM			(5)
#define RET_OUTOFRANGE			(6)
#define RET_IDLE			(7)
#define RET_WRONG_HANDLE		(8)
#define RET_NULL_POINTER		(9)
#define RET_NOTAVAILABLE		(10)
#define RET_DIVISION_BY_ZERO		(11)
#define RET_WRONG_STATE			(12)
#define RET_INVALID_PARM		(13)
#define RET_PENDING			(14)
#define RET_WRONG_CONFIG		(15)
#define RET_TIME_OUT			(16)
#define RET_UNSUPPORT_ID		(0xFFFF)

#define UPDATE_RESULT(cur_res, new_res)				\
		{ RESULT __lres__ = (new_res);			\
		do {						\
			if (curr_res == RET_SUCCESS)		\
				cur_res = __lres__;		\
		} while (0) }

#define RETURN_RESULT_IF_DIFFERENT(cur_res, exp_res)		\
		{ do {						\
			if (exp_res != cur_res) {		\
				return cur_res;			\
			}					\
		} while (0) }

#endif
