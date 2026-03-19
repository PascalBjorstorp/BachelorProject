# Ultra96 Ubuntu 22: MPC DDR Buffer Setup (Bulk Horizon Interface)

This guide configures reserved DDR buffers for the MPC FPGA bulk-memory interface and explains DTS/DTB editing on Ubuntu 22.

## 1. What this config does

The updated MPC IP reads horizon data from DDR via AXI masters (`m_axi_gmem0..3`).
The receiver writes buffer physical addresses into control registers before each compute.

Buffers used:
- `ref_vx`
- `ref_kappa`
- `ref_left`
- `ref_right`

This flow uses per-cycle bulk reference transfer through AXI memory interfaces.

## 2. DTS and DTB quick explanation

- DTS: human-readable device tree source (`.dts` / `.dtsi`).
- DTB: compiled binary device tree blob used by the kernel at boot.

You edit DTS, then compile it into DTB, then boot with that DTB.

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

## 7. Verify reservation after boot

Run on Ultra96:

```bash
cat /proc/iomem | grep -i reserved
```

You should see `0x70000000-0x700fffff` as reserved.

Also verify your control base still matches software (`0xA0000000` by default).

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

1. Program new bitstream with upgraded MPC IP wiring.
2. Boot with DTB containing reserved memory node.
3. Launch receiver.
4. Confirm logs do not show:
   - "MPC FPGA reference buffer mapping failed"
   - FPGA timeout warnings
5. Verify drive output updates as expected.

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
