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

#ifndef ISP_FW_MAIN_H
#define ISP_FW_MAIN_H

#if defined(RPU6_FW)

#define RPU6_FW_START_ADDR 					(0x0C000000U)
#define RPU6_FW_SIZE 						(0x37FFFFFU)
		
#define RPU6_LOAD_CALIB_START_ADDR           (0xF800000U)
#define RPU6_LOAD_CALIB_SIZE     	         (0x28000U)

#define RPU6_PRIV_MEM_START_ADDR             (0xF828000U)
#define RPU6_PRIV_MEM_SIZE     	             (0x1001000U)
#define RPU6_LOAD_CALIB_PRIV_MEM_SIZE        (RPU6_LOAD_CALIB_SIZE +RPU6_PRIV_MEM_SIZE -1)

#define RPU_FW_START_ADDR                    RPU6_FW_START_ADDR
#define RPU_FW_SIZE                          RPU6_FW_SIZE
#define RPU_LOAD_CALIB_START_ADDR            RPU6_LOAD_CALIB_START_ADDR
#define RPU_LOAD_CALIB_SIZE                  RPU6_LOAD_CALIB_SIZE
#define RPU_PRIV_MEM_START_ADDR              RPU6_PRIV_MEM_START_ADDR
#define RPU_PRIV_MEM_SIZE                    RPU6_PRIV_MEM_SIZE
#define RPU_LOAD_CALIB_PRIV_MEM_SIZE         RPU6_LOAD_CALIB_PRIV_MEM_SIZE

/*************************RPU7 *****************************************************/
#elif defined(RPU7_FW)

#define RPU7_FW_START_ADDR 					(0x1082A000U)
#define RPU7_FW_SIZE 						(0x37FFFFFU)
#define RPU7_LOAD_CALIB_START_ADDR          (0x1402A000U)
#define RPU7_LOAD_CALIB_SIZE     	         (0x28000U)
#define RPU7_PRIV_MEM_START_ADDR              (0x14052000U)
#define RPU7_PRIV_MEM_SIZE     	             (0x1001000U)
#define RPU7_LOAD_CALIB_PRIV_MEM_SIZE       (RPU7_LOAD_CALIB_SIZE +RPU7_PRIV_MEM_SIZE -1)


#define RPU_FW_START_ADDR                    RPU7_FW_START_ADDR
#define RPU_FW_SIZE                          RPU7_FW_SIZE
#define RPU_LOAD_CALIB_START_ADDR            RPU7_LOAD_CALIB_START_ADDR
#define RPU_LOAD_CALIB_SIZE                  RPU7_LOAD_CALIB_SIZE
#define RPU_PRIV_MEM_START_ADDR              RPU7_PRIV_MEM_START_ADDR
#define RPU_PRIV_MEM_SIZE                    RPU7_PRIV_MEM_SIZE
#define RPU_LOAD_CALIB_PRIV_MEM_SIZE         RPU7_LOAD_CALIB_PRIV_MEM_SIZE


/*************************RPU8 *****************************************************/
#elif defined(RPU8_FW)

#define RPU8_FW_START_ADDR 					(0x15054000U)
#define RPU8_FW_SIZE 						(0x37FFFFFU)
#define RPU8_LOAD_CALIB_START_ADDR          (0x18854000U)
#define RPU8_LOAD_CALIB_SIZE     	         (0x28000U)
#define RPU8_PRIV_MEM_START_ADDR              (0x1887C000U)
#define RPU8_PRIV_MEM_SIZE     	             (0x1001000U)
#define RPU8_LOAD_CALIB_PRIV_MEM_SIZE       (RPU8_LOAD_CALIB_SIZE +RPU8_PRIV_MEM_SIZE -1)


#define RPU_FW_START_ADDR                    RPU8_FW_START_ADDR
#define RPU_FW_SIZE                          RPU8_FW_SIZE
#define RPU_LOAD_CALIB_START_ADDR            RPU8_LOAD_CALIB_START_ADDR
#define RPU_LOAD_CALIB_SIZE                  RPU8_LOAD_CALIB_SIZE
#define RPU_PRIV_MEM_START_ADDR              RPU8_PRIV_MEM_START_ADDR
#define RPU_PRIV_MEM_SIZE                    RPU8_PRIV_MEM_SIZE
#define RPU_LOAD_CALIB_PRIV_MEM_SIZE         RPU8_LOAD_CALIB_PRIV_MEM_SIZE
 
/*************************RPU9 *****************************************************/
#elif defined(RPU9_FW)

#define RPU9_FW_START_ADDR 					(0x1987E000U)
#define RPU9_FW_SIZE 						(0x37FFFFFU)
#define RPU9_LOAD_CALIB_START_ADDR          (0x1D07E000U)
#define RPU9_LOAD_CALIB_SIZE     	         (0x28000U)
#define RPU9_PRIV_MEM_START_ADDR              (0x1D0A6000U)
#define RPU9_PRIV_MEM_SIZE     	             (0x1001000U)
#define RPU9_LOAD_CALIB_PRIV_MEM_SIZE       (RPU9_LOAD_CALIB_SIZE +RPU9_PRIV_MEM_SIZE -1)


#define RPU_FW_START_ADDR                    RPU9_FW_START_ADDR
#define RPU_FW_SIZE                          RPU9_FW_SIZE
#define RPU_LOAD_CALIB_START_ADDR            RPU9_LOAD_CALIB_START_ADDR
#define RPU_LOAD_CALIB_SIZE                  RPU9_LOAD_CALIB_SIZE
#define RPU_PRIV_MEM_START_ADDR              RPU9_PRIV_MEM_START_ADDR
#define RPU_PRIV_MEM_SIZE                    RPU9_PRIV_MEM_SIZE
#define RPU_LOAD_CALIB_PRIV_MEM_SIZE         RPU9_LOAD_CALIB_PRIV_MEM_SIZE

#else
    #error "Processor  Not Supported"
#endif

#define RPU_MBOX_START_ADDR		    	(0x1E0A8000U)
#define RPU_MBOX_SIZE                      (0x400000U)

// /*User shall not modify the below MACROS*/
#define RPU_SHM_START_ADDR		        	(0x1E4A8000U)
#define RPU_SHM_SIZE                      (0X40000U)  
#define RPU_SHM_END_ADDR                  (RPU_SHM_START_ADDR+RPU_SHM_SIZE)
#define RPU_MBOX_RPUSHM_SIZE        	  (RPU_MBOX_SIZE + RPU_SHM_SIZE - 1 )

void print_memory_layout_info();
#endif // ISP_FW_MAIN_H
