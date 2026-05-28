/******************************************************************************
* Copyright (c) 2023 - 2025 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
#ifndef BSPCONFIG_H
#define BSPCONFIG_H

#include "xmem_config.h"
#define XPAR_XILTIMER_ENABLED
#include "xparameters_ps.h"


/* FIXME: Remove dependency with YOCTO_FLOW variable */
#if (defined (__aarch64__) || defined (ARMA53_32)) && (defined (YOCTO_FLOW) || !defined (FREERTOS_BSP))
#define EL3   1
#define EL1_NONSECURE  0
#define HYP_GUEST  0
#endif

/* #undef versal */
/* #undef VERSAL */
/* #undef VERSAL_NET */
/* #undef VERSAL_AIEPG2 */
/* #undef VERSAL_2VE_2VM */
/* #undef SPARTANUP */
/* #undef PSU_PMU */
#define PLATFORM_ZYNQMP  
/* #undef XCLOCKING */
/* #undef PLATFORM_ZYNQ */
/* #undef VERSAL_PLM */
/* #undef VERSALNET_PLM */
/* #undef PLATFORM_MB */
/* #undef ASUFW */
#define XPAR_CPU_ID 0
/* #undef XIL_INTERRUPT */
/* #undef XPM_SUPPORT */
/* #undef ENABLE_4K_PAGES_OCM_TCM */
/* #undef XPAR_STDIN_IS_UARTLITE */
/* #undef XPAR_STDIN_IS_UARTNS550 */
/* #undef XPAR_STDIN_IS_UARTPS */
/* #undef XPAR_STDIN_IS_UARTPSV */
#define XPAR_STDIN_IS_CORESIGHTPS_DCC  
/* #undef XPAR_STDIN_IS_IOMODULE */
#define STDIN_BASEADDRESS 0xfe800000
#define STDOUT_BASEADDRESS 0xfe800000



#endif /* BSPCONFIG_H */
