#include "xttcps.h"

XTtcPs_Config XTtcPs_ConfigTable[] __attribute__ ((section (".drvcfg_sec"))) = {

	{
		"cdns,ttc", /* compatible */
		0xff110000, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x4024}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff110004, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x4025}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff110008, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x4026}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff120000, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x4027}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff120004, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x4028}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff120008, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x4029}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff130000, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x402a}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff130004, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x402b}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff130008, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x402c}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff140000, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x402d}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff140004, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x402e}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	{
		"cdns,ttc", /* compatible */
		0xff140008, /* reg */
		0x5f5e100, /* xlnx,clock-freq */
		{0x402f}, /* interrupts */
		0xf9010000 /* interrupt-parent */
	},
	 {
		 NULL
	}
};