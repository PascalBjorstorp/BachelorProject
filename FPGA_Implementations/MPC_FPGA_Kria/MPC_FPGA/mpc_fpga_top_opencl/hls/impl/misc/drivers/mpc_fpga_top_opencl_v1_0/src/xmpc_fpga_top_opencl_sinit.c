// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
// Tool Version Limit: 2025.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#ifdef SDT
#include "xparameters.h"
#endif
#include "xmpc_fpga_top_opencl.h"

extern XMpc_fpga_top_opencl_Config XMpc_fpga_top_opencl_ConfigTable[];

#ifdef SDT
XMpc_fpga_top_opencl_Config *XMpc_fpga_top_opencl_LookupConfig(UINTPTR BaseAddress) {
	XMpc_fpga_top_opencl_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XMpc_fpga_top_opencl_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XMpc_fpga_top_opencl_ConfigTable[Index].Control_BaseAddress == BaseAddress) {
			ConfigPtr = &XMpc_fpga_top_opencl_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XMpc_fpga_top_opencl_Initialize(XMpc_fpga_top_opencl *InstancePtr, UINTPTR BaseAddress) {
	XMpc_fpga_top_opencl_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XMpc_fpga_top_opencl_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XMpc_fpga_top_opencl_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XMpc_fpga_top_opencl_Config *XMpc_fpga_top_opencl_LookupConfig(u16 DeviceId) {
	XMpc_fpga_top_opencl_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XMPC_FPGA_TOP_OPENCL_NUM_INSTANCES; Index++) {
		if (XMpc_fpga_top_opencl_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XMpc_fpga_top_opencl_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XMpc_fpga_top_opencl_Initialize(XMpc_fpga_top_opencl *InstancePtr, u16 DeviceId) {
	XMpc_fpga_top_opencl_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XMpc_fpga_top_opencl_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XMpc_fpga_top_opencl_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif

