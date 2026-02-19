#!/usr/bin/env python3
"""
VESC PID Speed Controller Step Response Analyzer
=================================================

Analyzes VESC Tool CSV logs to evaluate PID speed controller tuning.

WORKFLOW:
  1. Connect to your VESC via USB with VESC Tool
  2. In VESC Tool: Realtime Data → Check "Activate Sampling" (top bar)
  3. In VESC Tool: Data Analysis → Start logging to CSV
     OR: Realtime Data → top-right hamburger menu → "Start CSV Logging"
  4. In the Terminal tab, type the step response commands:
        rpm 0
        (wait 2 seconds)
        rpm 5000
        (wait 3-5 seconds until speed is stable)
        rpm 0
        (wait 3 seconds)
  5. Stop CSV logging
  6. Run this script:
        python3 analyze_pid_response.py <path_to_vesc_tool_csv>

The script will compute:
  - Rise time (10% → 90% of target)
  - Overshoot (%)
  - Settling time (within ±2% of target)
  - Steady-state error
  - PID tuning recommendations

VESC Tool CSV format:
  The CSV has columns including: ms_today, rpm, rpm_filtered (if available),
  duty_cycle, current_motor, etc. The exact columns depend on the VESC Tool version.

Notes:
  - The VESC speed PID operates on ERPM, not m/s
  - The 'rpm' column in VESC Tool output is actually ERPM
  - Typical F1TENTH target: 4000-6000 ERPM ≈ 1-1.5 m/s
"""

import argparse
import csv
import os
import sys
import numpy as np
from typing import Optional, Tuple


def load_vesc_csv(filepath: str) -> dict:
    """
    Load VESC Tool CSV export.
    
    VESC Tool writes CSVs with semicolons as delimiters (European locale)
    or commas. We try both.
    """
    with open(filepath, 'r') as f:
        first_line = f.readline()
    
    # Detect delimiter
    if ';' in first_line:
        delimiter = ';'
    else:
        delimiter = ','
    
    data = {}
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f, delimiter=delimiter)
        rows = list(reader)
    
    if not rows:
        print("ERROR: CSV file is empty")
        sys.exit(1)
    
    print(f"Loaded {len(rows)} samples from {os.path.basename(filepath)}")
    print(f"Columns: {list(rows[0].keys())}")
    
    for key in rows[0].keys():
        try:
            # Handle European decimal format (comma → dot)
            vals = []
            for row in rows:
                val_str = row[key].strip().replace(',', '.')
                vals.append(float(val_str))
            data[key.strip()] = np.array(vals)
        except (ValueError, TypeError):
            data[key.strip()] = np.array([row[key] for row in rows])
    
    return data


def find_rpm_column(data: dict) -> str:
    """Find the ERPM/RPM column in the data."""
    candidates = ['rpm', 'erpm', 'RPM', 'ERPM', 'rpm_filtered', 'speed_erpm']
    for c in candidates:
        if c in data:
            return c
    
    # Fuzzy search
    for key in data.keys():
        if 'rpm' in key.lower() or 'erpm' in key.lower():
            return key
    
    print(f"ERROR: Could not find RPM column. Available: {list(data.keys())}")
    sys.exit(1)


def find_time_column(data: dict) -> Tuple[str, np.ndarray]:
    """Find and normalize the time column (return in seconds)."""
    # VESC Tool typically uses ms_today (milliseconds since midnight)
    candidates = ['ms_today', 'time_ms', 'time', 'Time', 'timestamp', 'ms', 'time_s']
    
    for c in candidates:
        if c in data:
            t = data[c]
            # Normalize to start at 0
            t = t - t[0]
            # Convert to seconds if in milliseconds
            if t[-1] > 1000:  # probably milliseconds
                t = t / 1000.0
            return c, t
    
    # If no time column, assume constant sample rate
    print("WARNING: No time column found, assuming 100Hz sample rate")
    n = len(next(iter(data.values())))
    return 'synthetic', np.arange(n) / 100.0


def detect_step(rpm: np.ndarray, time: np.ndarray) -> Tuple[int, int, int, float]:
    """
    Detect the step-up event in the RPM data.
    
    Returns:
        step_start_idx: Index where step command was given
        step_end_idx: Index where step-down command was given
        target_rpm: The target RPM value
    """
    # Find the steady-state RPM during the step (the plateau)
    # Use the median of the upper half of RPM values
    rpm_sorted = np.sort(rpm)
    high_rpm = rpm_sorted[len(rpm_sorted) * 3 // 4:]  # top 25%
    target_rpm = np.median(high_rpm)
    
    if target_rpm < 100:
        print("ERROR: Could not detect a step response. Max RPM too low.")
        print(f"  RPM range: {rpm.min():.0f} to {rpm.max():.0f}")
        sys.exit(1)
    
    # 10% threshold to detect step start
    threshold_10 = 0.10 * target_rpm
    threshold_90 = 0.90 * target_rpm
    
    # Find step-up: first sample where RPM crosses 10% of target
    above_threshold = np.where(rpm > threshold_10)[0]
    if len(above_threshold) == 0:
        print("ERROR: RPM never exceeds 10% of detected target")
        sys.exit(1)
    
    step_start_idx = above_threshold[0]
    
    # Back up a few samples to get the actual command time
    # (the RPM starts rising slightly after the command)
    step_start_idx = max(0, step_start_idx - 5)
    
    # Find step-down: after reaching 90% of target, find where it drops back below 50%
    above_90 = np.where(rpm > threshold_90)[0]
    if len(above_90) == 0:
        print("WARNING: RPM never reached 90% of target. PID may be very underdamped.")
        step_end_idx = len(rpm) - 1
    else:
        # Look for the drop after the plateau
        plateau_start = above_90[0]
        post_plateau = rpm[plateau_start:]
        below_50_post = np.where(post_plateau < 0.50 * target_rpm)[0]
        if len(below_50_post) > 0:
            step_end_idx = plateau_start + below_50_post[0]
        else:
            step_end_idx = len(rpm) - 1
    
    return step_start_idx, step_end_idx, target_rpm


def analyze_step_response(rpm: np.ndarray, time: np.ndarray,
                          step_start: int, step_end: int,
                          target_rpm: float):
    """
    Compute PID performance metrics from a step response.
    """
    # Extract the step-up portion
    t = time[step_start:step_end] - time[step_start]
    r = rpm[step_start:step_end]
    
    # Target: use the mean of the last 20% of the step (steady state)
    n = len(r)
    steady_samples = r[int(n * 0.8):]
    steady_state = np.mean(steady_samples)
    steady_state_std = np.std(steady_samples)
    
    print(f"\n{'='*60}")
    print(f"  STEP RESPONSE ANALYSIS")
    print(f"{'='*60}")
    print(f"  Detected target RPM:      {target_rpm:.0f}")
    print(f"  Steady-state RPM:         {steady_state:.0f} ± {steady_state_std:.0f}")
    print(f"  Steady-state error:       {abs(target_rpm - steady_state):.0f} RPM "
          f"({abs(target_rpm - steady_state)/target_rpm*100:.1f}%)")
    print(f"  Step duration:            {t[-1]:.2f}s")
    print(f"  Samples:                  {n}")
    
    # Rise time (10% → 90% of steady state)
    threshold_10 = 0.10 * steady_state
    threshold_90 = 0.90 * steady_state
    
    t_10 = None
    t_90 = None
    for i in range(len(r)):
        if t_10 is None and r[i] >= threshold_10:
            t_10 = t[i]
        if t_90 is None and r[i] >= threshold_90:
            t_90 = t[i]
    
    rise_time = None
    if t_10 is not None and t_90 is not None:
        rise_time = t_90 - t_10
        print(f"\n  Rise time (10%→90%):      {rise_time*1000:.1f} ms")
    else:
        print(f"\n  Rise time:                Could not compute (RPM didn't reach 90%)")
    
    # Overshoot
    peak_rpm = np.max(r)
    overshoot_pct = (peak_rpm - steady_state) / steady_state * 100
    peak_time_idx = np.argmax(r)
    peak_time = t[peak_time_idx]
    
    print(f"  Peak RPM:                 {peak_rpm:.0f}")
    print(f"  Overshoot:                {overshoot_pct:.1f}%")
    print(f"  Time to peak:             {peak_time*1000:.1f} ms")
    
    # Settling time (within ±2% of steady state)
    settling_band = 0.02 * steady_state
    settled = np.abs(r - steady_state) <= settling_band
    
    settling_time = None
    # Find the last time the signal leaves the settling band
    for i in range(len(settled) - 1, -1, -1):
        if not settled[i]:
            if i < len(t) - 1:
                settling_time = t[i + 1]
            break
    
    if settling_time is not None:
        print(f"  Settling time (±2%):      {settling_time*1000:.1f} ms")
    else:
        print(f"  Settling time:            Already within ±2% (or never settles)")
    
    # Oscillation count (with dead band to ignore noise)
    noise_band = 3 * steady_state_std
    oscillation_count = 0
    if t_90 is not None:
        idx_90 = np.searchsorted(t, t_90)
        post_rise = r[idx_90:] - steady_state
        state = 0
        for val in post_rise:
            if val > noise_band:
                if state == -1:
                    oscillation_count += 1
                state = 1
            elif val < -noise_band:
                if state == 1:
                    oscillation_count += 1
                state = -1
        print(f"  Oscillations after rise:  {oscillation_count}"
              f"  (noise band: ±{noise_band:.0f} RPM)")
    
    # PID tuning recommendations
    print(f"\n{'─'*60}")
    print(f"  TUNING RECOMMENDATIONS")
    print(f"{'─'*60}")
    
    issues = []
    
    if overshoot_pct > 25:
        issues.append(("HIGH OVERSHOOT", 
                       f"  {overshoot_pct:.0f}% overshoot → Reduce Kp or increase Kd"))
    elif overshoot_pct > 10:
        issues.append(("MODERATE OVERSHOOT",
                       f"  {overshoot_pct:.0f}% → Consider slightly reducing Kp or increasing Kd"))
    
    if rise_time is not None:
        if rise_time > 0.5:
            issues.append(("SLOW RESPONSE",
                          f"  Rise time {rise_time*1000:.0f}ms → Increase Kp"))
        elif rise_time < 0.05:
            issues.append(("VERY FAST (aggressive)",
                          f"  Rise time {rise_time*1000:.0f}ms → May cause oscillation, reduce Kp"))
    
    if settling_time is not None and settling_time > 1.0:
        issues.append(("SLOW SETTLING",
                       f"  Settling time {settling_time*1000:.0f}ms → Increase Kd (damping)"))
    
    ss_error_pct = abs(target_rpm - steady_state) / target_rpm * 100
    if ss_error_pct > 5:
        issues.append(("STEADY-STATE ERROR",
                       f"  {ss_error_pct:.1f}% error → Increase Ki"))
    
    if oscillation_count > 3:
        issues.append(("OSCILLATORY",
                       f"  {oscillation_count} oscillations → Reduce Ki, increase Kd"))
    
    if not issues:
        print("  ✓ PID tuning looks GOOD")
    else:
        for label, suggestion in issues:
            print(f"  ⚠ {label}")
            print(f"  {suggestion}")
    
    # Summary table
    print(f"\n{'─'*60}")
    print(f"  {'Metric':<25} {'Your Value':<15} {'Good Range':<15} {'Status':<10}")
    print(f"  {'─'*25} {'─'*15} {'─'*15} {'─'*10}")
    
    rt_str = f"{rise_time*1000:.0f} ms" if rise_time else "N/A"
    st_str = f"{settling_time*1000:.0f} ms" if settling_time else "N/A"
    
    def status_icon(ok): return "✓" if ok else "⚠"
    
    rt_ok = rise_time is not None and rise_time <= 0.2
    os_ok = overshoot_pct < 15
    st_ok = settling_time is None or settling_time <= 0.5
    se_ok = ss_error_pct < 3
    oc_ok = oscillation_count <= 2
    
    print(f"  {'Rise time':<25} {rt_str:<15} {'50-200 ms':<15} {status_icon(rt_ok):<10}")
    print(f"  {'Overshoot':<25} {f'{overshoot_pct:.1f}%':<15} {'< 15%':<15} {status_icon(os_ok):<10}")
    print(f"  {'Settling time':<25} {st_str:<15} {'< 500 ms':<15} {status_icon(st_ok):<10}")
    print(f"  {'Steady-state error':<25} {f'{ss_error_pct:.1f}%':<15} {'< 3%':<15} {status_icon(se_ok):<10}")
    print(f"  {'Oscillations':<25} {f'{oscillation_count}':<15} {'0-2':<15} {status_icon(oc_ok):<10}")
    
    # Only show specific actions if there are issues
    if issues:
        print(f"\n{'─'*60}")
        print(f"  SPECIFIC ACTIONS (in VESC Tool → Motor Settings → PID → Speed)")
        print(f"{'─'*60}")
        for label, suggestion in issues:
            print(f"  → {suggestion.strip()}")
    
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze VESC PID step response from VESC Tool CSV log',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLE WORKFLOW:
  1. Open VESC Tool, connect to VESC
  2. Go to Realtime Data → activate sampling
  3. Start CSV logging (Data Analysis or top-right menu)
  4. In Terminal tab, type:
       rpm 0
       (wait 2s)
       rpm 5000
       (wait 3-5s)
       rpm 0
       (wait 3s)
  5. Stop CSV logging
  6. Run: python3 analyze_pid_response.py /path/to/log.csv
  
ALTERNATIVE - from VESC Tool Terminal (no CSV needed):
  If you just want quick numbers, type in VESC Terminal:
    rpm 5000
  Then watch the Realtime Data plot. Note the overshoot visually.
  Type: rpm 0
""")
    
    parser.add_argument('csv_file', type=str,
                        help='Path to VESC Tool CSV log file')
    parser.add_argument('--target-rpm', type=float, default=None,
                        help='Override target RPM (auto-detected by default)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.csv_file):
        print(f"ERROR: File not found: {args.csv_file}")
        sys.exit(1)
    
    # Load data
    data = load_vesc_csv(args.csv_file)
    
    # Find RPM column
    rpm_col = find_rpm_column(data)
    rpm = data[rpm_col]
    print(f"Using RPM column: '{rpm_col}' (range: {rpm.min():.0f} to {rpm.max():.0f})")
    
    # Find time column
    time_col, time = find_time_column(data)
    print(f"Using time column: '{time_col}' (duration: {time[-1]:.2f}s)")
    
    # Detect step
    target = args.target_rpm
    step_start, step_end, detected_target = detect_step(rpm, time)
    
    if target is None:
        target = detected_target
    
    print(f"\nStep detected: t={time[step_start]:.2f}s to t={time[step_end]:.2f}s")
    print(f"Target RPM: {target:.0f}")
    
    # Analyze
    analyze_step_response(rpm, time, step_start, step_end, target)


if __name__ == '__main__':
    main()
