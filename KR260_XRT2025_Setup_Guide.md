# KR260 XRT 2025.2 Setup Guide
## Complete Walkthrough — From Source Build to Running xclbin

---

## Overview

This document describes everything required to run a Vitis 2025.2 HLS-generated `.xclbin` on a Kria KR260 board running Ubuntu 22.04, using a self-built XRT 2025.2 stack.

### What the Final Working Stack Looks Like

| Component | Version | Source |
|---|---|---|
| Ubuntu | 22.04.5 LTS | Kria BSP |
| Linux Kernel | 5.15.0-1069-xilinx-zynqmp | Kria BSP |
| XRT Userspace | 2.20.0 (2025.2) | Built from source |
| zocl.ko | 2.20.0 (2025.2) | Built from source |
| dfx-mgr | 2023.1 | Kria BSP apt |
| xmutil | 1.2.1 | Kria BSP apt |

---

## Part 1 — Building XRT from Source

### 1.1 Clone XRT

```bash
git clone https://github.com/Xilinx/XRT.git
cd XRT
git checkout 2025.2  # or the appropriate branch/tag
```

### 1.2 Install Build Dependencies

```bash
cd XRT
sudo ./src/runtime_src/tools/scripts/xrtdeps.sh
```

### 1.3 Fix the AIE Compile Error (KR260 Has No AI Engine)

Before building, patch the file that references AIE functions not present in the KR260 shim:

```bash
# Open the file
nano src/runtime_src/core/edge/user/hwctx_object.cpp
```

Find the two functions and replace their bodies:

```cpp
uint64_t
hwctx_object::
get_aie_freq() const
{
  return 0; // AIE not available on KR260
}

void
hwctx_object::
set_aie_freq(uint64_t freq_hz) const
{
  return; // AIE not available on KR260
}
```

### 1.4 Build XRT for Edge (Zynq/KR260)

This is the critical step. The board is aarch64 so build natively on the board itself.
You MUST use `-edge` and `-noert` flags — without them, XRT builds for PCIe/Alveo instead
and the resulting libraries cannot talk to zocl.

```bash
cd XRT/build
./build.sh -edge -noert 2>&1 | tee /tmp/xrt_edge_build.log
```

This takes approximately 40-60 minutes on the KR260.

### 1.5 Install the XRT .deb Package

```bash
# Install only the main XRT package — the others are cloud/flash specific
sudo dpkg -i ~/XRT/build/Release/xrt_*-xrt.deb

# Expected output shows:
# | XRT USERSPACE  | Success |   <- This is what matters
# | XOCL Driver    | Failed  |   <- This is NORMAL, KR260 uses zocl not xocl
```

### 1.6 Replace the PCIe libxrt_core.so with the Edge Version

The .deb installs the PCIe version of the core library. You must replace it with the
edge version from the Debug build output:

```bash
# Back up the PCIe version
sudo cp /opt/xilinx/xrt/lib/libxrt_core.so /opt/xilinx/xrt/lib/libxrt_core.so.pcie_backup
sudo cp /opt/xilinx/xrt/lib/libxrt_core.so.2 /opt/xilinx/xrt/lib/libxrt_core.so.2.pcie_backup

# Install the edge version
sudo cp ~/XRT/build/Debug/runtime_src/core/edge/user/libxrt_core.so /opt/xilinx/xrt/lib/libxrt_core.so
sudo cp ~/XRT/build/Debug/runtime_src/core/edge/user/libxrt_core.so /opt/xilinx/xrt/lib/libxrt_core.so.2
sudo cp ~/XRT/build/Debug/runtime_src/core/edge/user/libxrt_core.so /opt/xilinx/xrt/lib/libxrt_core.so.2.20.0

# Update linker cache
sudo ldconfig
```

### 1.7 Source the XRT Environment

```bash
source /opt/xilinx/xrt/setup.sh

# Add to .bashrc to make it permanent
echo "source /opt/xilinx/xrt/setup.sh" >> ~/.bashrc
```

---

## Part 2 — Building and Loading zocl

zocl is the kernel driver that sits between XRT userspace and the FPGA fabric on Zynq devices.
It is NOT the same as xocl (which is for PCIe/Alveo cards).

### 2.1 Build zocl.ko

zocl is built as part of the XRT source tree:

```bash
cd ~/XRT/src/runtime_src/core/edge/drm/zocl
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
```

The output is `zocl.ko` in the same directory.

### 2.2 Load zocl

```bash
sudo insmod ~/XRT/src/runtime_src/core/edge/drm/zocl/zocl.ko
```

Verify it loaded (tainting warnings are normal for out-of-tree modules):

```bash
lsmod | grep zocl
# Expected: zocl   249856  0
```

### 2.3 Make zocl Load Automatically on Boot

```bash
echo "$(readlink -f ~/XRT/src/runtime_src/core/edge/drm/zocl/zocl.ko)" | \
  sudo tee /etc/modules-load.d/zocl.conf
```

---

## Part 3 — Device Tree Overlay

The device tree overlay tells Linux about the hardware in the PL fabric and provides
the `zyxclmm_drm` node that zocl binds to.

### 3.1 Why a Custom dtbo Is Needed

Vitis generates a `.dtbo` but it is compiled WITHOUT the `/plugin/;` directive and without
the `-@` flag, so phandle references are unresolved (shown as `0xffffffff`). The overlay
cannot be applied as-is.

### 3.2 The Correct dtbo Structure

Create `/tmp/kr260_mpc_app.dts`:

```
/dts-v1/;
/plugin/;

/ {
    fragment@0 {
        target = <&fpga_full>;
        __overlay__ {
            #address-cells = <2>;
            #size-cells = <2>;
            firmware-name = "kr260_mpc_app.bit.bin";
            resets = <&zynqmp_reset 116>, <&zynqmp_reset 117>,
                     <&zynqmp_reset 118>, <&zynqmp_reset 119>;
        };
    };
    fragment@1 {
        target = <&amba>;
        __overlay__ {
            zyxclmm_drm {
                compatible = "xlnx,zocl";
                status = "okay";
            };
        };
    };
};
```

Key points:
- `target = <&fpga_full>` — programs the PL with the bitstream
- `target = <&amba>` — adds the zocl node to the live AXI bus
- `zyxclmm_drm` with `compatible = "xlnx,zocl"` is what zocl binds to
- NO interrupt controller node needed for basic operation
- The bitstream programming is handled by the FPGA manager via `firmware-name`

### 3.3 Compile and Install the dtbo

```bash
# Compile with plugin/symbol support
dtc -@ -I dts -O dtb -o /tmp/kr260_mpc_app.dtbo /tmp/kr260_mpc_app.dts

# Install alongside the bitstream
sudo mkdir -p /lib/firmware/xilinx/kr260_mpc_app
sudo cp /tmp/kr260_mpc_app.dtbo /lib/firmware/xilinx/kr260_mpc_app/kr260_mpc_app.dtbo
```

### 3.4 Required Files in the App Folder

```
/lib/firmware/xilinx/kr260_mpc_app/
├── kr260_mpc_app.bit.bin     # Bitstream in binary format (from Vitis)
├── kr260_mpc_app.dtbo        # Compiled device tree overlay (rebuilt as above)
├── shell.json                # App metadata for dfx-mgr
└── mpc_fpga_top_opencl.xclbin  # XRT runtime descriptor (from Vitis)
```

The `shell.json` should contain:
```json
{
  "shell_type" : "XRT_FLAT",
  "num_slots" : "1"
}
```

---

## Part 4 — Loading the App

### 4.1 Start dfx-mgrd

dfx-mgrd is the daemon that manages FPGA overlays. It must be running before xmutil works:

```bash
sudo dfx-mgrd &
sleep 2
```

### 4.2 Load the App

```bash
sudo xmutil unloadapp            # Unload whatever is currently loaded
sudo xmutil loadapp MPC_FPGA
```

Successful output looks like:
```
DFX-MGRD> sys_load_accel():213 Successfully loaded base design.
kr260_mpc_app: loaded to slot 0
```

This does three things:
1. Writes the bitstream to the PL via FPGA manager
2. Applies the device tree overlay (creating the zyxclmm_drm node)
3. Writes the xclbin path to /etc/vart.conf

### 4.3 Verify renderD128 Appeared

```bash
ls /dev/dri/
# Must show: by-path  card0  card1  renderD128
```

`renderD128` is the device node that XRT uses to communicate with zocl.

### 4.4 Load the xclbin via XRT

```bash
xrt-smi examine  # Should show the device

xrt-smi program --device 0000:00:00.0 \
  --user /lib/firmware/xilinx/kr260_mpc_app/mpc_fpga_top_opencl.xclbin

# INFO: xrt-smi program succeeded on 0000:00:00.0
```

---

## Part 5 — Full Verification Checklist

Run these checks in order to confirm everything is working:

### ✅ Check 1 — zocl Is Loaded

```bash
lsmod | grep zocl
# Expected: zocl  249856  0
```

### ✅ Check 2 — App Is Loaded

```bash
sudo xmutil listapps
# Expected: kr260_mpc_app shows Active_slot = 0
```

### ✅ Check 3 — renderD128 Exists

```bash
ls /dev/dri/
# Expected: by-path  card0  card1  renderD128
```

### ✅ Check 4 — renderD128 Symlink Exists

```bash
ls /dev/dri/by-path/ | grep render
# Expected: platform-axi:zyxclmm_drm-render -> ../renderD128
```

### ✅ Check 5 — zocl Bound to Device

```bash
cat /sys/class/drm/renderD128/device/uevent
# Expected: DRIVER=zocl-drm
#           OF_COMPATIBLE_0=xlnx,zocl
```

### ✅ Check 6 — XRT Sees the Device

```bash
source /opt/xilinx/xrt/setup.sh
xrt-smi examine
# Expected: zocl hash matches XRT hash
#           Device [0000:00:00.0] edge  Yes
```

### ✅ Check 7 — xclbin Is Loaded with Correct Kernel

```bash
xrt-smi examine --device 0000:00:00.0 --report all 2>&1 | grep -A5 "Compute Units"
# Expected: mpc_fpga_top_opencl:mpc_fpga_top_opencl_1  0xa0000000  (IDLE)
```

### ✅ Check 8 — XRT Hash Matches zocl Hash

```bash
xrt-smi examine | grep -A2 "zocl"
# The hash after the comma must match the XRT build hash
# Example: zocl : unknown, 4d047a9e87de64f483fda15776381f3e92075647
```

### ✅ Check 9 — dmesg Shows Clean zocl Init

```bash
sudo dmesg | grep -i zocl
# Expected:
# [drm] Probing for xlnx,zocl
# [drm] Initialized zocl 2.20.0 <date> for axi:zyxclmm_drm on minor 1
```

### ✅ Check 10 — vart.conf Points to Your xclbin

```bash
cat /etc/vart.conf
# Expected: firmware: /lib/firmware/xilinx/kr260_mpc_app/mpc_fpga_top_opencl.xclbin
```

---

## Part 6 — When You Update Your HLS Design

When you rebuild your Vitis project and get new output files, here is the exact update procedure:

### Step 1 — Copy New Files from Vitis

From your Vitis build output, copy to the board:

```bash
# Files you need from Vitis 2025.2 output:
scp MPC_FPGA.bit.bin  ubuntu@10.23.0.2:/lib/firmware/xilinx/MPC_FPGA/MPC_FPGA.bit.bin
scp mpc_fpga_top_opencl.xclbin   ubuntu@10.23.0.2:/lib/firmware/xilinx/MPC_FPGA/mpc_fpga_top_opencl.xclbin
# Note: do NOT copy the Vitis-generated .dtbo — use your custom one instead
```

### Step 2 — Unload the Current App

```bash
sudo xmutil unloadapp
sudo rmmod zocl
```

### Step 3 — Reload

```bash
sudo insmod ~/XRT/src/runtime_src/core/edge/drm/zocl/zocl.ko
sudo xmutil loadapp MPC_FPGA
```

### Step 4 — Load the New xclbin

```bash
xrt-smi program --device 0000:00:00.0 \
  --user /lib/firmware/xilinx/MPC_FPGA/mpc_fpga_top_opencl.xclbin
```

### Step 5 — Verify

```bash
xrt-smi examine --device 0000:00:00.0 --report all | grep -A5 "Compute Units"
```

---

## Part 7 — Complete Startup Script

Save this as `~/start_fpga.sh` for easy reloading after reboots:

```bash
#!/bin/bash
set -e

echo "=== Starting KR260 FPGA Stack ==="

# Source XRT
source /opt/xilinx/xrt/setup.sh

# Load zocl if not loaded
if ! lsmod | grep -q zocl; then
    echo "[1/5] Loading zocl..."
    sudo insmod ~/XRT/src/runtime_src/core/edge/drm/zocl/zocl.ko
else
    echo "[1/5] zocl already loaded"
fi

# Start dfx-mgrd if not running
if ! pgrep -x dfx-mgrd > /dev/null; then
    echo "[2/5] Starting dfx-mgrd..."
    sudo dfx-mgrd &
    sleep 3
else
    echo "[2/5] dfx-mgrd already running"
fi

# Unload any existing app
echo "[3/5] Unloading existing app..."
sudo xmutil unloadapp 2>/dev/null || true

# Load your app
echo "[4/5] Loading kr260_mpc_app..."
sudo xmutil loadapp kr260_mpc_app

# Load xclbin
echo "[5/5] Programming xclbin..."
xrt-smi program --device 0000:00:00.0 \
  --user /lib/firmware/xilinx/kr260_mpc_app/mpc_fpga_top_opencl.xclbin

echo ""
echo "=== Done! Verifying... ==="
xrt-smi examine --device 0000:00:00.0 --report all | grep -A5 "Compute Units"
echo ""
echo "FPGA is ready."
```

```bash
chmod +x ~/start_fpga.sh
```

---

## Part 8 — Known Issues and Their Fixes

### Issue: `xrt-smi: command not found`
**Fix:** `source /opt/xilinx/xrt/setup.sh`

### Issue: `xmutil loadapp` returns `load Error: -1`
**Fix:** dfx-mgrd is not running. Run `sudo dfx-mgrd &` then retry.

### Issue: `0 devices found` in xrt-smi examine
**Fix:** Either the app is not loaded (`sudo xmutil loadapp kr260_mpc_app`) or the
edge `libxrt_core.so` is not installed (repeat step 1.6).

### Issue: `ERROR: Unexpected error creating shim library name`
**Fix:** The PCIe version of `libxrt_core.so` is installed instead of the edge version.
Repeat step 1.6 to replace it with the Debug build output.

### Issue: `create_overlay: Failed to create overlay (err=-22)`
**Fix:** The dtbo references a symbol not in the base device tree. Use the custom dtbo
from Part 3 which only references `fpga_full` and `amba` — both present on KR260.

### Issue: `create_overlay: Failed to create overlay (err=-12)`
**Fix:** Usually means the old dtbo is not a `/plugin/` overlay. Recompile with
`dtc -@` as shown in Part 3.

### Issue: zocl loads but `renderD128` never appears
**Fix:** Load order matters. zocl must be loaded BEFORE `xmutil loadapp` so it can
bind to the `zyxclmm_drm` node when the overlay is applied.

### Issue: Two dfx-mgrd daemons running, causing `no empty slot` errors
**Fix:** `sudo pkill dfx-mgrd && sudo dfx-mgrd &`

### Issue: XRT hash and zocl hash don't match
**Fix:** Rebuild zocl.ko from the same XRT source tree as the userspace.
They must come from the same git commit.

---

## Part 9 — Why Certain Things Are the Way They Are

### Why NOT use the Vitis-generated dtbo?
Vitis generates the dtbo without the `/plugin/;` directive and without compiling with
`dtc -@`. This means all external phandle references (`&zynqmp_clk`, `&gic`, etc.)
are stored as `0xffffffff` (unresolved). The kernel overlay resolver cannot fix these
at runtime, causing `err=-22`. The custom dtbo sidesteps this by only referencing
`fpga_full` and `amba` which are guaranteed present in the KR260 base DTB.

### Why does the xclbin need to be loaded twice?
`xmutil loadapp` writes the bitstream to PL and sets up the device tree.
`xrt-smi program` loads the xclbin into XRT's runtime context so it knows about
your kernels, memory topology, and connectivity. Both steps are required.

### Why the edge libxrt_core.so instead of the installed one?
The `-xrt.deb` package installs the PCIe/Alveo version of `libxrt_core.so` even on
aarch64. This is because the same package is used for cross-compilation scenarios.
The edge version (`core/edge/user/libxrt_core.so`) contains the zocl device
enumeration code, DRM ioctl interface, and Zynq-specific shim — none of which exist
in the PCIe version.

### Why build natively on the board instead of cross-compiling?
Cross-compilation with `build_edge.sh` requires a full PetaLinux installation which
is a large x86 host tool. Building natively on the KR260 is slower but requires no
additional tools beyond the standard build dependencies.

---

## Part 10 — Complete Library Replacement List

When replacing XRT PCIe libraries with edge versions, ALL of these must be replaced.
Replacing only `libxrt_core.so` is not enough:

```bash
# Replace all edge libraries at once
for lib in \
  "runtime_src/core/common/libxrt_coreutil.so" \
  "runtime_src/core/edge/user/libxrt_core.so"; do
    base=$(basename $lib)
    # Remove old versioned file
    sudo rm -f /opt/xilinx/xrt/lib/${base}.2.20.0
    # Copy edge version
    sudo cp ~/XRT/build/Debug/$lib /opt/xilinx/xrt/lib/${base}.2.20.0
    # Recreate symlinks
    sudo ln -sf /opt/xilinx/xrt/lib/${base}.2.20.0 /opt/xilinx/xrt/lib/${base}.2
    sudo ln -sf /opt/xilinx/xrt/lib/${base}.2.20.0 /opt/xilinx/xrt/lib/${base}
done
sudo ldconfig
```

Verify edge libraries are installed (zyxclmm strings must appear):
```bash
strings /opt/xilinx/xrt/lib/libxrt_core.so | grep zyxclmm
strings /opt/xilinx/xrt/lib/libxrt_coreutil.so | grep -i "emulation\|shim"
```

---

## Part 11 — The `XCL_EMULATION_MODE` Gotcha

### Symptom
```
[xrt-smi] ERROR: Unexpected error creating shim library name
```

### Cause
`XCL_EMULATION_MODE` is set to an empty string (not unset, but empty).
The `is_emulation()` function in XRT treats an empty string as truthy,
causing `shim_name()` to fall through all if-branches and throw.

### How to Check
```bash
env | grep -i emulation
# If you see: XCL_EMULATION_MODE=
# That empty value is the bug
```

### Fix
```bash
unset XCL_EMULATION_MODE

# Make permanent
echo "unset XCL_EMULATION_MODE" >> ~/.bashrc
```

### Add to start_fpga.sh
Always add this at the top of any script that calls xrt-smi:
```bash
unset XCL_EMULATION_MODE
source /opt/xilinx/xrt/setup.sh
```

---

## Part 12 — Standalone zocl Build

When you need to rebuild zocl without doing a full XRT build (e.g. after a kernel update):

### Patch the Source Files First

The standalone build is missing cmake-injected defines. Patch them manually:

```bash
cd ~/XRT/src/runtime_src/core/edge/drm/zocl

# Fix XRT_HASH — must use word boundary to avoid matching XRT_HASH_DATE
sed -i 's/\bXRT_HASH_DATE\b/"2026-05-04"/' common/zocl_sysfs.c common/zocl_drv.c
sed -i 's/\bXRT_HASH\b/"4d047a9e87de64f483fda15776381f3e92075647"/' common/zocl_sysfs.c common/zocl_drv.c
sed -i 's/\bXRT_BRANCH\b/"2025.2"/' common/zocl_sysfs.c common/zocl_drv.c
sed -i 's/\bXRT_MODIFIED_FILES\b/""/' common/zocl_sysfs.c common/zocl_drv.c
sed -i 's/\bXRT_DATE\b/"2026-05-04"/' common/zocl_sysfs.c common/zocl_drv.c
```

These are all informational strings only — they have no effect on functionality.

### Build

```bash
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
```

### Reload

```bash
sudo rmmod zocl 2>/dev/null || true
sudo insmod ~/XRT/src/runtime_src/core/edge/drm/zocl/zocl.ko
sudo dmesg | grep -i zocl | tail -3
```

---

## Part 13 — Updated Complete Startup Script

```bash
#!/bin/bash
set -e

echo "=== Starting KR260 FPGA Stack ==="

# Critical: unset emulation mode or xrt-smi will fail with shim error
unset XCL_EMULATION_MODE

# Source XRT
source /opt/xilinx/xrt/setup.sh

# Ensure edge libraries are installed
if ! strings /opt/xilinx/xrt/lib/libxrt_core.so 2>/dev/null | grep -q zyxclmm; then
    echo "WARNING: Installing edge XRT libraries..."
    for lib in \
      "runtime_src/core/common/libxrt_coreutil.so" \
      "runtime_src/core/edge/user/libxrt_core.so"; do
        base=$(basename $lib)
        sudo rm -f /opt/xilinx/xrt/lib/${base}.2.20.0
        sudo cp ~/XRT/build/Debug/$lib /opt/xilinx/xrt/lib/${base}.2.20.0
        sudo ln -sf /opt/xilinx/xrt/lib/${base}.2.20.0 /opt/xilinx/xrt/lib/${base}.2
        sudo ln -sf /opt/xilinx/xrt/lib/${base}.2.20.0 /opt/xilinx/xrt/lib/${base}
    done
    sudo ldconfig
fi

# Load zocl if not loaded
if ! lsmod | grep -q zocl; then
    echo "[1/5] Loading zocl..."
    sudo insmod ~/XRT/src/runtime_src/core/edge/drm/zocl/zocl.ko
else
    echo "[1/5] zocl already loaded"
fi

# Start dfx-mgrd if not running
if ! pgrep -x dfx-mgrd > /dev/null; then
    echo "[2/5] Starting dfx-mgrd..."
    sudo dfx-mgrd &
    sleep 3
else
    echo "[2/5] dfx-mgrd already running"
fi

# Unload any existing app
echo "[3/5] Unloading existing app..."
sudo xmutil unloadapp 2>/dev/null || true

# Load your app
echo "[4/5] Loading kr260_mpc_app..."
sudo xmutil loadapp kr260_mpc_app

# Load xclbin
echo "[5/5] Programming xclbin..."
xrt-smi program --device 0000:00:00.0 \
  --user /lib/firmware/xilinx/kr260_mpc_app/mpc_fpga_top_opencl.xclbin

echo ""
echo "=== Done! Verifying... ==="
xrt-smi examine --device 0000:00:00.0 --report all | grep -A5 "Compute Units"
echo ""
echo "FPGA is ready."
```

---

*Generated after successful XRT 2025.2 bring-up on KR260 revB — May 2026*
