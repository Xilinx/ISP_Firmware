set(DDR axi_noc2_DDR_LOW_0)
set(axi_noc2_DDR_LOW_0 "0x100000;0x7ff00000")
set(ocm_ram_0 "0xbbf00000;0x80000")
set(ocm_ram_1 "0xbbf80000;0x80000")
set(ocm_ram_2 "0xbbe00000;0x80000")
set(ocm_ram_3 "0xbbe80000;0x80000")
set(TOTAL_MEM_CONTROLLERS "axi_noc2_DDR_LOW_0;ocm_ram_0;ocm_ram_1;ocm_ram_2;ocm_ram_3")
set(MEMORY_SECTION "MEMORY
{
	pmc_ram : ORIGIN = 0xF2000000, LENGTH = 0x20000
	r52_tcm_alias : ORIGIN = 0x0, LENGTH = 0x20000
	axi_noc2_DDR_LOW_0 : ORIGIN = 0x100000, LENGTH = 0x7ff00000
	ocm_ram_0 : ORIGIN = 0xbbf00000, LENGTH = 0x80000
	ocm_ram_1 : ORIGIN = 0xbbf80000, LENGTH = 0x80000
	ocm_ram_2 : ORIGIN = 0xbbe00000, LENGTH = 0x80000
	ocm_ram_3 : ORIGIN = 0xbbe80000, LENGTH = 0x80000
}")
set(STACK_SIZE 0x2000)
set(HEAP_SIZE 0x2000)
