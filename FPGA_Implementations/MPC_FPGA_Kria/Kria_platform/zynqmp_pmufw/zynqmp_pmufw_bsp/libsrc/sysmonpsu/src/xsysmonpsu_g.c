#include "xsysmonpsu.h"

XSysMonPsu_Config XSysMonPsu_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"xlnx,zynqmp-ams", /* compatible */
		0xffa50000, /* reg */
		0x31 /* xlnx,clock-freq */
	},
	 {
		 NULL
	}
};