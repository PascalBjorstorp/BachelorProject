# CPU & GPU Usage Measurement: System and Code

## 1. How Linux Measures CPU Usage

- The Linux kernel tracks, for each CPU/core, the cumulative time spent in different states (user, system, idle, iowait, etc.) since boot.
- These counters are exposed in `/proc/stat` as lines like:
  ```
  cpu  12345 678 910 ...
  cpu0 1234 56 78 ...
  ```
  - Fields: user, nice, system, idle, iowait, irq, softirq, steal
- Units: "jiffies" (kernel ticks, configurable via `CONFIG_HZ`; common values are 100, 250, or 1000, giving 10ms, 4ms, or 1ms per tick)
- Tools sample these values at intervals to compute usage percent per state.
- You can distinguish user (application), system (kernel/OS), and idle time.
- See: https://www.kernel.org/doc/html/latest/filesystems/proc.html#stat

## 2. How Linux Measures GPU Usage (Jetson)

- Jetson exposes GPU load as a percent in sysfs (e.g., `/sys/devices/gpu.0/load`).
- On x86, `nvidia-smi` reports utilization.
- These are updated by the kernel/NVIDIA drivers, often in the tens-of-milliseconds range (commonly around 20-50ms on Jetson, workload/driver dependent).
- See: https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3271/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/gpu.html

## 3. How Your Code Measures Usage

- **CPU:** Reads `/proc/stat` at a configurable rate (default 400Hz/2.5ms). Computes percent busy per core and aggregate, using deltas between samples.
- **Per-process CPU:** Reads `/proc/[pid]/stat` for each process, normalizes by wall time and core count.
- **GPU:** Reads Jetson sysfs or `nvidia-smi` at a configurable rate (default 200Hz/5ms).
- **Rolling averages:** 1s (long) and 5ms (short) windows are used for smoothing.
- **Precision:** Kernel counters are precise at jiffy level, but percent usage is only as accurate as the sampling interval and window.

## 4. Why Very Fast Sampling Can Look Noisy or "Steppy"

Great question: measuring CPU usage at 400Hz (every 2.5ms) can be faster than the kernel tick period (often 10ms or 4ms). If no jiffy changed between two samples, the delta is zero, so usage appears as repeated zeros with occasional jumps.

The same effect appears for GPU load on Jetson: if the driver updates the sysfs value every 10-100ms, sampling every 5ms mostly repeats the same value until the next driver update.

Practical takeaway:
- Sampling faster than the source update interval does not increase true precision.
- It usually increases noise/staleness and log volume.
- For smoother, more meaningful graphs, use lower rates (for example 10-20Hz) and keep rolling averages.

## 5. How To Measure Real Update Rates On Jetson

### CPU counter update interval (`/proc/stat`)

Check configured kernel tick rate (if config is available):

```bash
grep '^CONFIG_HZ=' /boot/config-$(uname -r)
```

Then measure observed counter-change intervals directly:

```bash
#!/usr/bin/env bash
set -euo pipefail

prev_line="$(head -n1 /proc/stat)"
prev_ts="$(date +%s%N)"

for ((i=0; i<5000; i++)); do
  line="$(head -n1 /proc/stat)"
  ts="$(date +%s%N)"
  if [[ "$line" != "$prev_line" ]]; then
    echo $((ts - prev_ts))
    prev_line="$line"
    prev_ts="$ts"
  fi
done | awk '
  {sum+=$1; n++; if(min==0 || $1<min) min=$1; if($1>max) max=$1}
  END {
    if (n==0) {
      print "No CPU counter changes observed.";
    } else {
      printf "CPU update interval: min=%.3fms avg=%.3fms max=%.3fms (n=%d)\n", min/1e6, (sum/n)/1e6, max/1e6, n;
    }
  }
'
```

### GPU load update interval (Jetson sysfs)

```bash
#!/usr/bin/env bash
set -euo pipefail

GPU_LOAD_PATH="/sys/devices/gpu.0/load"
if [[ ! -f "$GPU_LOAD_PATH" ]]; then
  echo "GPU load file not found at $GPU_LOAD_PATH"
  exit 1
fi

prev_val="$(cat "$GPU_LOAD_PATH")"
prev_ts="$(date +%s%N)"

for ((i=0; i<5000; i++)); do
  val="$(cat "$GPU_LOAD_PATH")"
  ts="$(date +%s%N)"
  if [[ "$val" != "$prev_val" ]]; then
    echo $((ts - prev_ts))
    prev_val="$val"
    prev_ts="$ts"
  fi
  sleep 0.001
done | awk '
  {sum+=$1; n++; if(min==0 || $1<min) min=$1; if($1>max) max=$1}
  END {
    if (n==0) {
      print "No GPU value changes observed (try adding GPU workload).";
    } else {
      printf "GPU update interval: min=%.3fms avg=%.3fms max=%.3fms (n=%d)\n", min/1e6, (sum/n)/1e6, max/1e6, n;
    }
  }
'
```

Tip: run a GPU workload while measuring, otherwise GPU load may stay mostly constant at idle.

## 6. References
- Linux /proc/stat: https://www.kernel.org/doc/html/latest/filesystems/proc.html#stat
- Jetson GPU load: https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3271/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/gpu.html
- NVIDIA nvidia-smi: https://docs.nvidia.com/deploy/nvidia-smi/index.html
- JetsonHacks: https://www.jetsonhacks.com/2020/06/24/jetson-clocks/
- Source code: f1tenth_localization/src/core/performance_monitor.cpp
