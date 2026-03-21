# Ultra96 Ubuntu 22: MPC DDR Buffer Setup (Bulk Horizon Interface)

This guide configures reserved DDR buffers for the MPC FPGA bulk-memory interface and adds a complete deployment flow (build, copy, program, validate, run) for Ultra96.

Important:
- You cannot flash an `.xsa` file directly on Ultra96.
- `.xsa` is a hardware handoff/archive for tools.
- Runtime programming on Ultra96 uses a bitstream artifact (`.bit`/`.bin`) and optional device-tree overlay (`.dtbo`).

## 1. What this config does

The updated MPC IP reads horizon data from DDR via AXI masters (`m_axi_gmem0..3`).
The receiver writes buffer physical addresses into control registers before each compute.

Buffers used:
- `ref_vx`
- `ref_kappa`
- `ref_left`
- `ref_right`

This flow uses per-cycle bulk reference transfer through AXI memory interfaces.

## 1.1 Quick safety rule

Do not start `mpc_receiver` until FPGA is programmed and address map is verified.

If `mpc_receiver` accesses an invalid AXI-Lite base address (for example `0xA0000000` when IP is not present), the board can stall.

## 2. DTS and DTB quick explanation

- DTS: human-readable device tree source (`.dts` / `.dtsi`).
- DTB: compiled binary device tree blob used by the kernel at boot.

You edit DTS, then compile it into DTB, then boot with that DTB.

For PYNQ-style images, runtime overlays (`.dtbo`) via configfs are often easier than replacing boot DTB.

## 3. Recommended physical layout

Use one reserved 1 MB block, split into four 256 KB buffers:

- `ref_vx_phys_addr` = `0x70000000`
- `ref_kappa_phys_addr` = `0x70040000`
- `ref_left_phys_addr` = `0x70080000`
- `ref_right_phys_addr` = `0x700C0000`

Total reserved range:
- `0x70000000` to `0x700FFFFF`

## 4. How to open the DTS on Ubuntu 22

If you already have board DTS sources in your build tree, edit those directly.

If not, you can decompile your current DTB to a temporary DTS on the Ultra96:

```bash
sudo apt-get update
sudo apt-get install -y device-tree-compiler

# Find active DTB path from extlinux
grep -R "^\s*FDT" /boot /boot/extlinux 2>/dev/null

# Example decompile (adjust DTB path)
dtc -I dtb -O dts -o current.dts /boot/dtb/<your_board>.dtb
```

Then edit `current.dts` with your editor and add the reserved-memory node.

## 5. Add reserved-memory in device tree

Add this node in your board DTS/DTSI (or overlay), under `/`:

```dts
/ {
  reserved-memory {
    #address-cells = <2>;
    #size-cells = <2>;
    ranges;

    mpc_ref_buffers: mpc_ref_buffers@70000000 {
      no-map;
      reg = <0x0 0x70000000 0x0 0x00100000>;
    };
  };
};
```

Notes:
- `no-map` prevents normal Linux memory allocation in this region.
- Keep this region inside DDR that is reachable by `S_AXI_HPC0_FPD`.

## 6. Compile DTS to DTB and install on Ubuntu 22

Example flow if using extlinux:

```bash
# Compile
dtc -I dts -O dtb -o mpc-reserved.dtb current.dts

# Backup current DTB
sudo cp /boot/dtb/<your_board>.dtb /boot/dtb/<your_board>.dtb.bak

# Install new DTB
sudo cp mpc-reserved.dtb /boot/dtb/<your_board>.dtb

# Verify extlinux points to this DTB
grep -R "^\s*FDT" /boot/extlinux/extlinux.conf 2>/dev/null
```

Reboot after copying.

Alternative (runtime overlay, recommended on PYNQ images):

```bash
# Compile overlay to dtbo (syntax may vary depending on your overlay source)
dtc -@ -I dts -O dtb -o mpc_ref_buffers.dtbo mpc_ref_buffers_overlay.dts

# Copy and apply through configfs using your existing overlay script/service
# (example path from this repo workflow)
sudo mkdir -p /home/xilinx/pynq/overlays/mpc_ref_buffers
sudo cp mpc_ref_buffers.dtbo /home/xilinx/pynq/overlays/mpc_ref_buffers/
```

## 7. Verify reservation after boot

Run on Ultra96:

```bash
cat /proc/iomem | grep -i reserved
```

You should see `0x70000000-0x700fffff` as reserved.

Also verify your control base still matches software (`0xA0000000` by default).

On images without `/proc/device-tree`, use:

```bash
ls /sys/firmware/devicetree/base/reserved-memory
```

## 8. Receiver parameter values

Set these in:
`f1tenth_communication/state_receiver/config/mpc_params.yaml`

```yaml
ref_vx_phys_addr: 0x70000000
ref_kappa_phys_addr: 0x70040000
ref_left_phys_addr: 0x70080000
ref_right_phys_addr: 0x700C0000
ref_buffer_capacity: 64
```

## 9. Why capacity 64 is safe

Each element is one `int32` (4 bytes).

Required bytes per buffer:

`bytes = 4 * ref_buffer_capacity`

For capacity 64:
- 256 bytes needed per buffer
- 256 KB allocated per buffer (ample headroom)

## 10. Bring-up checklist

1. Build/export hardware in Vivado/Vitis and generate deployable bitstream.
2. Ensure AXI-Lite base address in hardware matches software parameter (default `0xA0000000`).
3. Boot Ultra96 with reserved-memory config active (DTB or overlay).
4. Program FPGA on Ultra96 using the deployable bitstream.
5. Validate MMIO reads return quickly before launching receiver.
6. Launch receiver.
7. Confirm logs do not show:
   - "MPC FPGA reference buffer mapping failed"
   - FPGA timeout warnings
8. Verify drive output updates as expected.

## 10.1 End-to-end flow including XSA

### A) Build side (host machine with Vivado)

1. Generate/export hardware from Vivado (`.xsa`).
2. Build the deployable FPGA image from that design (`.bit` or bootable `.bin`).
3. Export/register map data (`.hwh` or generated headers) and confirm:
  - MPC AXI-Lite base address (software expects `0xA0000000` unless changed).
  - Address range (for example `0xA0000000-0xA000FFFF`).

Note:
- `.xsa` is not what Ultra96 runtime loader programs directly.
- Use `.xsa` in tools to produce runtime artifacts.

### B) Copy artifacts to Ultra96

Copy required runtime files (example):

```bash
scp <your_mpc>.bit xilinx@<ultra96-ip>:/home/xilinx/
scp mpc_ref_buffers.dtbo xilinx@<ultra96-ip>:/home/xilinx/
```

If your flow outputs `.bin` instead of `.bit`, copy `.bin` accordingly.

### C) Program FPGA on Ultra96

Use your board/runtime loader flow (for example PYNQ/fpgautil path used in your project).

After programming, validate quickly that the control region responds:

```bash
sudo devmem 0xA0000000 32
sudo devmem 0xA0000004 32
```

These reads must return promptly.

### D) Start order (safe)

1. Apply reserved-memory overlay/DT.
2. Program FPGA.
3. Validate MMIO reads.
4. Start receiver.

Do not reverse this order.

## 10.2 Service vs manual run policy

If you require "program must stop when SSH disconnects":

- Keep services disabled:

```bash
sudo systemctl disable --now mpc-receiver.service mpc-overlay.service
```

- Run manually in foreground:

```bash
sudo /bin/bash -lc 'source /home/xilinx/ros2_humble/install/setup.bash; source /home/xilinx/ros2_ws/install/setup.bash; ros2 launch state_receiver mpc_launch.py'
```

Foreground launch ends when session ends.

## 11. Troubleshooting

If mapping fails:
- Check `/dev/mem` permissions.
- Confirm addresses are page-aligned.
- Confirm DTB with reservation is actually loaded.
- Ensure reserved region does not overlap kernel/CMA allocations.

If you cannot find current DTB location:
- `find /boot -name "*.dtb"`
- check `extlinux.conf` `FDT` line and use that exact path.

If compute works but output is invalid:
- Confirm `s_axi_ctrl` base address matches Vivado Address Editor.
- Confirm `ref_buffer_capacity` >= `horizon_length`.
- Confirm `horizon_length` and reference arrays are populated each message.

If board stalls after starting receiver:
- Stop/disable receiver immediately.
- Re-check that FPGA is programmed with the expected design.
- Re-check AXI-Lite base address match between hardware and `mpc_params.yaml`.
- Validate `devmem` reads at base address before launching ROS node.

If SD/boot becomes unstable:
- Repair filesystems offline (`e2fsck` for ext4, `fsck.vfat` for boot partition).
- Ensure SD adapter is not write-protected before repair.
