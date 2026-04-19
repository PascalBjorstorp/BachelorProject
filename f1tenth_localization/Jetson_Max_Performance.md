# Jetson Max Performance Setup

This guide explains how to configure your NVIDIA Jetson (Xavier, Orin, Nano, etc.) to always run CPU and GPU at maximum frequency, disabling dynamic frequency scaling for consistent, peak performance.

## 1. Set Power Mode to Maximum

Use `nvpmodel` to select the highest power/performance mode:

```bash
sudo nvpmodel -m 0
```
- `-m 0` selects the max performance mode (all cores, max clocks, max power budget).
- You can check available modes with `sudo nvpmodel -q`.

## 2. Lock CPU and GPU Clocks

Use the NVIDIA utility to force all clocks to their maximum:

```bash
sudo jetson_clocks
```
- This sets all CPU, GPU, and memory clocks to their highest supported values and disables scaling.
- To revert, reboot or run `sudo jetson_clocks --restore`.

## 3. Optional: Manual CPU Clock Control

For fine-grained CPU control, you can manually set clocks via sysfs (advanced):

```bash
# List available frequencies
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_available_frequencies
# Set a specific frequency (replace X with value)
echo X | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq
```

Note: direct GPU frequency control via sysfs is limited/inconsistent across Jetson/L4T versions. For GPU clocks, `jetson_clocks` is the recommended approach.

## 4. Persistence

These settings are not persistent across reboots. Prefer a `systemd` service for automation (use `/etc/rc.local` only if your image enables it).

## 5. Downsides

- **Power Usage:** Jetson will draw more power at idle and under load.
- **Heat:** Higher sustained temperatures; ensure adequate cooling.
- **Wear:** Slightly more long-term wear, but Jetsons are designed for this.

## References
- NVIDIA Docs: https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3271/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/jetson_clocks.html
- JetsonHacks: https://www.jetsonhacks.com/2020/06/24/jetson-clocks/
- NVIDIA Forums: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/70
