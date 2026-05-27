// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
// Tool Version Limit: 2025.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xmpc_fpga_top_opencl.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XMpc_fpga_top_opencl_CfgInitialize(XMpc_fpga_top_opencl *InstancePtr, XMpc_fpga_top_opencl_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XMpc_fpga_top_opencl_Start(XMpc_fpga_top_opencl *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL) & 0x80;
    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XMpc_fpga_top_opencl_IsDone(XMpc_fpga_top_opencl *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XMpc_fpga_top_opencl_IsIdle(XMpc_fpga_top_opencl *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XMpc_fpga_top_opencl_IsReady(XMpc_fpga_top_opencl *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XMpc_fpga_top_opencl_Continue(XMpc_fpga_top_opencl *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL) & 0x80;
    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL, Data | 0x10);
}

void XMpc_fpga_top_opencl_EnableAutoRestart(XMpc_fpga_top_opencl *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XMpc_fpga_top_opencl_DisableAutoRestart(XMpc_fpga_top_opencl *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_AP_CTRL, 0);
}

void XMpc_fpga_top_opencl_Set_input_words512(XMpc_fpga_top_opencl *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_INPUT_WORDS512_DATA, (u32)(Data));
    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_INPUT_WORDS512_DATA + 4, (u32)(Data >> 32));
}

u64 XMpc_fpga_top_opencl_Get_input_words512(XMpc_fpga_top_opencl *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_INPUT_WORDS512_DATA);
    Data += (u64)XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_INPUT_WORDS512_DATA + 4) << 32;
    return Data;
}

void XMpc_fpga_top_opencl_Set_output_words128(XMpc_fpga_top_opencl *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_OUTPUT_WORDS128_DATA, (u32)(Data));
    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_OUTPUT_WORDS128_DATA + 4, (u32)(Data >> 32));
}

u64 XMpc_fpga_top_opencl_Get_output_words128(XMpc_fpga_top_opencl *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_OUTPUT_WORDS128_DATA);
    Data += (u64)XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_OUTPUT_WORDS128_DATA + 4) << 32;
    return Data;
}

void XMpc_fpga_top_opencl_InterruptGlobalEnable(XMpc_fpga_top_opencl *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_GIE, 1);
}

void XMpc_fpga_top_opencl_InterruptGlobalDisable(XMpc_fpga_top_opencl *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_GIE, 0);
}

void XMpc_fpga_top_opencl_InterruptEnable(XMpc_fpga_top_opencl *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_IER);
    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_IER, Register | Mask);
}

void XMpc_fpga_top_opencl_InterruptDisable(XMpc_fpga_top_opencl *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_IER);
    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_IER, Register & (~Mask));
}

void XMpc_fpga_top_opencl_InterruptClear(XMpc_fpga_top_opencl *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XMpc_fpga_top_opencl_WriteReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_ISR, Mask);
}

u32 XMpc_fpga_top_opencl_InterruptGetEnabled(XMpc_fpga_top_opencl *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_IER);
}

u32 XMpc_fpga_top_opencl_InterruptGetStatus(XMpc_fpga_top_opencl *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XMpc_fpga_top_opencl_ReadReg(InstancePtr->Control_BaseAddress, XMPC_FPGA_TOP_OPENCL_CONTROL_ADDR_ISR);
}

