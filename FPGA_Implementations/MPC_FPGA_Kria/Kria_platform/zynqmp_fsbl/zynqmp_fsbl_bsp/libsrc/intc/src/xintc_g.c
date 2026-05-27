#include "xintc.h"

XIntc_Config XIntc_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,axi-intc-4.1", /* compatible */
		0x80000000, /* reg */
		0x1, /* xlnx,kind-of-intr */
		0x0, /* xlnx,is-fast */
		0x10, /* xlnx,ivar-rst-val */
		0x20, /* xlnx,num-intr-inputs */
		0x20, /* xlnx,addr-width */
		0x0, /* options */
		0x0, /* xlnx,intctype */
		{{0U}}, /* Handler-table */
		0x0, /* xlnx,num-sw-intr */
		0x4059, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	 {
		 NULL
	}
};