/****************************************************************************
 * *
 * * The MIT License (MIT)
 * *
 * * Copyright (c) 2025 Advanced Micro Devices, Inc. All right reserved.
 * *
 * * Permission is hereby granted, free of charge, to any person obtaining a
 * * copy of this software and associated documentation files (the "Software"),
 * * to deal in the Software without restriction, including without limitation
 * * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * * and/or sell copies of the Software, and to permit persons to whom the
 * * Software is furnished to do so, subject to the following conditions:
 * *
 * * The above copyright notice and this permission notice shall be included in
 * * all copies or substantial portions of the Software.
 * *
 * * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * * DEALINGS IN THE SOFTWARE.
 * *
 * ****************************************************************************/
int main_lib(void);

#define RPU6_FW_START_ADDR 					(0x0C000000U)
#define RPU6_FW_SIZE 						(0x5125fff)
			
#define RPU6_LOAD_CALIB_START_ADDR           (0x11126000U)
#define RPU6_LOAD_CALIB_SIZE     	         (0x28000U)

#define RPU6_PRIV_MEM_START_ADDR              (0x1114E000U)
#define RPU6_PRIV_MEM_SIZE     	             (0x1001000U)


#define RPU6_MBOX_START_ADDR		    	(0x1829E000)
#define RPU6_MBOX_SIZE                      (0x400000U)


/*User shall not modify the below MACROS*/
#define RPU6_SHM_SIZE                      (0X40000U)  
#define RPU6_MBOX_RPUSHM_SIZE        	  (RPU6_MBOX_SIZE + RPU6_SHM_SIZE - 1 )
#define RPU6_LOAD_CALIB_PRIV_MEM_SIZE     (RPU6_LOAD_CALIB_SIZE +RPU6_PRIV_MEM_SIZE -1 )

