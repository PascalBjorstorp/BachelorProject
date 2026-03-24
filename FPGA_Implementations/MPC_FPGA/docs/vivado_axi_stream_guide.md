# Vivado Block Design Guide: AXI-Stream MPC Implementation

This guide explains how to modify your Vivado block design to use AXI-Stream DMA for the MPC FPGA IP, replacing the slower DDR-based approach.

## Overview

**Before (DDR Path):**
```
ARM → DDR Buffer → FPGA (m_axi reads from DDR)
```

**After (AXI-Stream DMA Path):**
```
ARM → DMA Buffer → AXI DMA → AXI-Stream → FPGA
```

**Expected Improvement:**
- Transfer time: ~1850 ns → ~210 ns (**8.8× faster**)
- Eliminates DDR read latency from FPGA
- Reduces bus contention on memory interface

---

## Step 1: Export and Synthesize New HLS IP

### 1.1 Update HLS Source
The `mpc_fpga_top.cpp` source is compiled as C++ for HLS and exposes the AXI-stream top:

```c
void mpc_fpga_top(
    hls::stream<axis_word_t>& input_stream,  // AXI-Stream input
    int *out_steering_fp,                     // AXI-Lite outputs
    int *out_accel_fp,
    int *out_status,
    int *out_iterations)
```

### 1.2 HLS Pragmas (Already Added)
```c
#pragma HLS INTERFACE axis port=input_stream
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_accel_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_status bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_iterations bundle=ctrl
```

### 1.3 Synthesize in Vitis HLS
1. Open Vitis HLS project
2. Run C Synthesis
3. Export RTL → IP Catalog (.zip)
4. Note the exported IP location

---

## Step 2: Add AXI DMA IP to Block Design

### 2.1 Open Vivado Project
1. Open your existing `mpc_design` project
2. Open Block Design (`mpc_design_wrapper.bd` or similar)

### 2.2 Add AXI DMA IP
1. Click "Add IP" (+ icon) in diagram
2. Search for "AXI Direct Memory Access"
3. Double-click to add to design

### 2.3 Configure AXI DMA
Double-click the DMA IP to open configuration:

| Setting | Value | Notes |
|---------|-------|-------|
| **Enable Scatter Gather** | ❌ Unchecked | Simple mode, no descriptor chain |
| **Width of Buffer Length Register** | 14 | Supports up to 16KB transfers |
| **Enable Read Channel (MM2S)** | ✅ Checked | Memory to Stream (we need this) |
| **Enable Write Channel (S2MM)** | ❌ Unchecked | Stream to Memory (not needed) |
| **MM2S Data Width** | 128 | Match stream width |
| **Max Burst Size** | 16 | Balance latency vs efficiency |
| **Allow Unaligned Transfers** | ❌ Unchecked | Our data is aligned |

Click OK to apply.

---

## Step 3: Update MPC IP in Block Design

### 3.1 Refresh MPC IP
1. In IP Catalog, right-click → "Add Repository"
2. Navigate to your exported HLS IP location
3. Right-click MPC IP in design → "Upgrade IP"

### 3.2 Verify New Ports
After upgrade, the MPC IP should show:
- `input_stream` (AXI-Stream Slave, 128-bit)
- `s_axi_ctrl` (AXI-Lite Slave for control/outputs)

**Note:** The old `m_axi_*` ports for DDR access should be removed.

---

## Step 4: Connect Block Design

### 4.1 AXI DMA Connections

| DMA Port | Connect To | Notes |
|----------|------------|-------|
| `S_AXI_LITE` | `axi_interconnect_0/M0x_AXI` | Control interface |
| `M_AXI_MM2S` | `axi_interconnect_hp/S0x_AXI` | Memory read port |
| `M_AXIS_MM2S` | `mpc_fpga_top/input_stream` | **Stream to MPC** |

### 4.2 MPC IP Connections

| MPC Port | Connect To | Notes |
|----------|------------|-------|
| `input_stream` | `axi_dma/M_AXIS_MM2S` | **Input from DMA** |
| `s_axi_ctrl` | `axi_interconnect_0/M0x_AXI` | Control + outputs |

### 4.3 Clock & Reset
- Connect all `aclk` ports to same clock (e.g., 100 MHz from PS)
- Connect all `aresetn` ports to same reset signal

### 4.4 Address Editor
Open Address Editor tab and verify/assign addresses:

| IP | Interface | Base Address | Size |
|----|-----------|--------------|------|
| `mpc_fpga_top` | `s_axi_ctrl` | `0xA0000000` | 64K |
| `axi_dma` | `S_AXI_LITE` | `0xA0010000` | 64K |

**DMA Memory Range:**
- Ensure HP port can access `0x70000000 - 0x70000FFF` (DMA buffer)

---

## Step 5: Block Design Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          ZYNQ PS (ARM)                                  │
│  ┌──────────────┐                                                       │
│  │ M_AXI_GP0    │ ────────────────────────────────────┐                 │
│  │ (AXI-Lite)   │                                     │                 │
│  └──────────────┘                                     │                 │
│  ┌──────────────┐                                     │                 │
│  │ S_AXI_HP0    │ ◄───────────────────────┐           │                 │
│  │ (Memory)     │                         │           │                 │
│  └──────────────┘                         │           │                 │
└─────────────────────────────────────────────────────────────────────────┘
                                            │           │
                                            │           ▼
                                            │   ┌───────────────────┐
                                            │   │ AXI Interconnect  │
                                            │   │    (AXI-Lite)     │
                                            │   └───────────────────┘
                                            │       │           │
                                            │       │           │
                                            │       ▼           ▼
┌─────────────────────────────────────┐     │   ┌───────┐   ┌──────────┐
│           AXI DMA                   │     │   │ DMA   │   │  MPC IP  │
│ ┌─────────────┐  ┌───────────────┐  │     │   │ ctrl  │   │  ctrl    │
│ │ S_AXI_LITE  │◄─┤               │  │     │   │0xA001 │   │ 0xA000   │
│ │  (control)  │  │               │  │     │   └───────┘   └──────────┘
│ └─────────────┘  │               │  │     │
│ ┌─────────────┐  │    MM2S       │  │     │
│ │ M_AXI_MM2S  │──┤   (read)      │──┼─────┘
│ │  (memory)   │  │               │  │
│ └─────────────┘  │               │  │
│ ┌─────────────┐  │               │  │       ┌──────────────────────────┐
│ │ M_AXIS_MM2S │──┼───────────────┼──┼──────►│       MPC FPGA IP        │
│ │  (stream)   │  │               │  │       │                          │
│ └─────────────┘  └───────────────┘  │       │  ┌────────────────────┐  │
└─────────────────────────────────────┘       │  │  input_stream      │◄─┤
                                              │  │  (AXI-Stream 128b) │  │
                                              │  └────────────────────┘  │
                                              │                          │
                                              │  ┌────────────────────┐  │
                                              │  │  s_axi_ctrl        │◄─┼─ Outputs
                                              │  │  (out_steering_fp) │  │
                                              │  │  (out_accel_fp)    │  │
                                              │  │  (out_status)      │  │
                                              │  │  (out_iterations)  │  │
                                              │  └────────────────────┘  │
                                              └──────────────────────────┘
```

---

## Step 6: Validate and Generate

### 6.1 Validate Design
1. Click "Validate Design" (checkmark icon)
2. Fix any errors:
   - Missing connections
   - Address conflicts
   - Clock domain issues

### 6.2 Generate Bitstream
1. Generate Block Design wrapper (if needed)
2. Run Synthesis
3. Run Implementation
4. Generate Bitstream

### 6.3 Export XSA
1. File → Export → Export Hardware
2. Include bitstream: ✅ Yes
3. Save as `mpc_design_wrapper.xsa`

---

## Step 7: Update Device Tree Overlay

The existing `mpc_ref_buffers.dtbo` should still work since we're reusing the same reserved memory region at `0x70000000`. However, you may want to reduce the reserved size since we only need 336 bytes (vs 1MB before).

**Optional: Minimal DT Overlay**
```dts
/dts-v1/;
/plugin/;

/ {
    fragment@0 {
        target-path = "/reserved-memory";
        __overlay__ {
            #address-cells = <2>;
            #size-cells = <2>;
            
            mpc_dma_buffer: mpc_dma_buffer@70000000 {
                compatible = "shared-dma-pool";
                reg = <0x0 0x70000000 0x0 0x1000>;  /* 4KB is plenty */
                no-map;
            };
        };
    };
};
```

---

## Step 8: Deploy and Test

### 8.1 Copy Files to Ultra96
```bash
scp mpc_design_wrapper.xsa xilinx@10.23.0.148:/home/xilinx/MPC_FPGA/
```

### 8.2 Run Flash Script
```bash
ssh xilinx@10.23.0.148
cd /home/xilinx
./ultra96_flash_and_run_receiver.sh
```

### 8.3 Verify Operation
Expected log output:
```
MPC-FPGA: Init OK - MPC@0xA0000000 (AP_CTRL=0x04), DMA@0xA0010000 (STATUS=0x02), Buffer@0x70000000
MPC Receiver [FPGA AXI-Stream] ready.  /mpc_state → /drive
```

---

## Troubleshooting

### DMA Init Fails
- Check address mapping in Vivado Address Editor
- Verify DMA IP is connected to HP port
- Check `/sys/class/fpga_manager/fpga0/state` = "operating"

### DMA Transfer Timeout
- Verify `M_AXIS_MM2S` connected to MPC `input_stream`
- Check stream widths match (both 128-bit)
- Enable DMA debug: check `DMA_MM2S_STATUS` register for errors

### MPC Compute Timeout
- DMA transfer completes but MPC doesn't finish
- Check HLS synthesis report for II violations
- Verify horizon data format matches HLS expectations

### FPGA Crashes (Red LED)
- Usually indicates AXI transaction error
- Check reserved memory is properly set up
- Verify DMA source address is within valid range

---

## Register Map Summary

### MPC IP (0xA0000000)
| Offset | Name | Access | Description |
|--------|------|--------|-------------|
| 0x00 | AP_CTRL | R/W | Control (bit0=START, bit1=DONE, bit2=IDLE) |
| 0x10 | out_steering_fp | R | Output steering (Q16.16) |
| 0x20 | out_accel_fp | R | Output acceleration (Q16.16) |
| 0x30 | out_status | R | Status code |
| 0x40 | out_iterations | R | ADMM iterations used |

### AXI DMA (0xA0010000)
| Offset | Name | Access | Description |
|--------|------|--------|-------------|
| 0x00 | MM2S_DMACR | R/W | MM2S Control (bit0=RUN, bit2=RESET) |
| 0x04 | MM2S_DMASR | R | MM2S Status (bit1=IDLE) |
| 0x18 | MM2S_SA_LO | W | Source Address (low 32 bits) |
| 0x1C | MM2S_SA_HI | W | Source Address (high 32 bits) |
| 0x28 | MM2S_LENGTH | W | Transfer length in bytes |

---

## Performance Summary

| Metric | DDR Path | AXI-Stream | Improvement |
|--------|----------|------------|-------------|
| Transfer time | ~1850 ns | ~210 ns | 8.8× |
| Total cycle | ~802 µs | ~800 µs | 0.25% |
| Bus utilization | High | Low | Less contention |
| Code complexity | Medium | Low | Simpler driver |

The AXI-Stream approach provides marginal total performance improvement (~2 µs/cycle) but significantly reduces bus contention and eliminates the DDR-related crash issues.
