#!/usr/bin/env python3
"""
VESC PID Step Response Tester & Analyzer
========================================

Two modes of operation:

**Mode 1 — Direct serial test (default):**
    Connects directly to the VESC over USB serial, runs a step response test,
    logs RPM data at high rate, and analyzes PID performance.

    REQUIREMENTS:
        pip install pyserial numpy

    USAGE:
        1. Make sure VESC Tool is NOT connected (only one app can use the port)
        2. Put the car on a stand so the wheels spin freely!
        3. Run:
           python3 vesc_pid_test.py [--port /dev/ttyACM0] [--target-rpm 5000]

        On Linux the port is usually /dev/ttyACM0.
        On Windows it's COMx (check Device Manager).
        On Mac it's /dev/tty.usbmodemXXXX.

**Mode 2 — Analyze VESC Tool CSV log (--analyze):**
    Analyzes a CSV file exported from VESC Tool's Realtime Data logging.

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
        6. Run:
           python3 vesc_pid_test.py --analyze /path/to/vesc_tool_log.csv

SAFETY:
    - Put the car on a STAND (wheels off the ground)!
    - The script sends RPM commands directly to the motor
    - Press Ctrl+C at any time to abort (sends RPM 0)
    - Max test duration is 10 seconds
"""

import argparse
import csv
import struct
import sys
import time
import os
from typing import Optional, Tuple

import numpy as np

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# ============================================================================
# VESC Protocol Constants
# ============================================================================
# Packet framing
VESC_START_SHORT = 0x02  # payload < 256 bytes
VESC_START_LONG = 0x03   # payload >= 256 bytes
VESC_END = 0x03

# Command IDs (from datatypes.hpp)
COMM_GET_VALUES = 4
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_ALIVE = 30


# ============================================================================
# CRC16/XMODEM (used by VESC protocol)
# ============================================================================
def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM used by VESC protocol."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ============================================================================
# VESC Packet Builder / Parser
# ============================================================================
def build_packet(payload: bytes) -> bytes:
    """Build a VESC protocol packet from a payload."""
    length = len(payload)
    crc = crc16_xmodem(payload)
    
    if length < 256:
        packet = bytes([VESC_START_SHORT, length])
    else:
        packet = bytes([VESC_START_LONG, (length >> 8) & 0xFF, length & 0xFF])
    
    packet += payload
    packet += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    packet += bytes([VESC_END])
    
    return packet


def build_set_rpm(rpm: int) -> bytes:
    """Build a COMM_SET_RPM packet."""
    payload = struct.pack('>Bi', COMM_SET_RPM, int(rpm))
    return build_packet(payload)


def build_set_brake(current: float) -> bytes:
    """Build a COMM_SET_CURRENT_BRAKE packet. Current in amps."""
    payload = struct.pack('>Bi', COMM_SET_CURRENT_BRAKE, int(current * 1000))
    return build_packet(payload)


def build_get_values() -> bytes:
    """Build a COMM_GET_VALUES request packet."""
    payload = bytes([COMM_GET_VALUES])
    return build_packet(payload)


def build_alive() -> bytes:
    """Build a COMM_ALIVE packet (keeps VESC from timing out)."""
    payload = bytes([COMM_ALIVE])
    return build_packet(payload)


def read_packet(ser: serial.Serial, timeout: float = 0.1) -> bytes:
    """
    Read a complete VESC packet from serial.
    Returns the payload (without framing/CRC), or empty bytes on timeout.
    """
    start_time = time.time()
    
    # Wait for start byte
    while True:
        if time.time() - start_time > timeout:
            return b''
        
        b = ser.read(1)
        if not b:
            continue
        
        if b[0] == VESC_START_SHORT:
            # Read 1-byte length
            length_bytes = ser.read(1)
            if not length_bytes:
                return b''
            length = length_bytes[0]
            break
        elif b[0] == VESC_START_LONG:
            # Read 2-byte length
            length_bytes = ser.read(2)
            if len(length_bytes) < 2:
                return b''
            length = (length_bytes[0] << 8) | length_bytes[1]
            break
    
    # Read payload + CRC (2 bytes) + end byte (1 byte)
    remaining = ser.read(length + 3)
    if len(remaining) < length + 3:
        return b''
    
    payload = remaining[:length]
    crc_received = (remaining[length] << 8) | remaining[length + 1]
    end_byte = remaining[length + 2]
    
    # Verify CRC
    crc_calc = crc16_xmodem(payload)
    if crc_calc != crc_received:
        return b''
    
    if end_byte != VESC_END:
        return b''
    
    return payload


def parse_get_values(payload: bytes) -> dict:
    """
    Parse COMM_GET_VALUES response.
    
    Byte offsets from vesc_packet.cpp in your VESC driver:
    [0]: command ID (4)
    [1-2]: temp_fet * 10
    [3-4]: temp_motor * 10
    [5-8]: avg_motor_current * 100
    [9-12]: avg_input_current * 100
    [13-16]: avg_id * 100
    [17-20]: avg_iq * 100
    [21-22]: duty_cycle * 1000
    [23-26]: rpm (ERPM, integer)
    [27-28]: v_in * 10
    """
    if len(payload) < 29:
        return {}
    
    if payload[0] != COMM_GET_VALUES:
        return {}
    
    values = {}
    values['temp_fet'] = struct.unpack('>h', payload[1:3])[0] / 10.0
    values['temp_motor'] = struct.unpack('>h', payload[3:5])[0] / 10.0
    values['motor_current'] = struct.unpack('>i', payload[5:9])[0] / 100.0
    values['input_current'] = struct.unpack('>i', payload[9:13])[0] / 100.0
    values['duty_cycle'] = struct.unpack('>h', payload[21:23])[0] / 1000.0
    values['rpm'] = struct.unpack('>i', payload[23:27])[0]
    values['v_in'] = struct.unpack('>h', payload[27:29])[0] / 10.0
    
    return values


# ============================================================================
# Step Response Test
# ============================================================================
def run_step_test(ser: serial.Serial, target_rpm: int,
                  hold_time: float = 3.0, settle_time: float = 2.0,
                  sample_rate: float = 200.0):
    """
    Execute a step response test.
    
    1. Hold at RPM 0 for settle_time seconds (baseline)
    2. Step to target_rpm for hold_time seconds (step up)
    3. Step back to RPM 0 for settle_time seconds (step down)
    
    Records RPM, current, duty cycle at ~sample_rate Hz.
    """
    dt = 1.0 / sample_rate
    
    # Data storage
    timestamps = []
    rpm_data = []
    current_data = []
    duty_data = []
    voltage_data = []
    phase = []  # 'baseline', 'step_up', 'step_down'
    
    total_time = settle_time + hold_time + settle_time
    
    print(f"\n{'='*60}")
    print(f"  VESC PID STEP RESPONSE TEST")
    print(f"{'='*60}")
    print(f"  Target RPM:     {target_rpm}")
    print(f"  Hold time:      {hold_time}s")
    print(f"  Sample rate:    ~{sample_rate:.0f} Hz")
    print(f"  Total duration: {total_time:.1f}s")
    print(f"{'='*60}")
    print()
    
    print("  ⚠ Make sure the car is on a STAND (wheels off ground)!")
    print("  ⚠ Press Ctrl+C at any time to abort safely")
    print()
    
    # Countdown
    for i in range(3, 0, -1):
        print(f"  Starting in {i}...")
        time.sleep(1.0)
    
    print("  GO!")
    
    t_start = time.monotonic()
    
    try:
        # Phase 1: Baseline (RPM 0)
        print(f"  Phase 1: Baseline (RPM 0) for {settle_time}s")
        while True:
            t_now = time.monotonic() - t_start
            if t_now >= settle_time:
                break
            
            ser.write(build_set_rpm(0))
            ser.write(build_get_values())
            
            payload = read_packet(ser, timeout=dt)
            if payload:
                vals = parse_get_values(payload)
                if vals:
                    timestamps.append(t_now)
                    rpm_data.append(vals['rpm'])
                    current_data.append(vals['motor_current'])
                    duty_data.append(vals['duty_cycle'])
                    voltage_data.append(vals['v_in'])
                    phase.append('baseline')
            
            # Maintain timing
            elapsed = time.monotonic() - t_start
            sleep_until = (len(timestamps)) * dt
            if sleep_until > elapsed:
                time.sleep(sleep_until - elapsed)
        
        # Phase 2: Step up
        t_step = time.monotonic() - t_start
        print(f"  Phase 2: Step to RPM {target_rpm} for {hold_time}s")
        while True:
            t_now = time.monotonic() - t_start
            if t_now >= settle_time + hold_time:
                break
            
            ser.write(build_set_rpm(target_rpm))
            ser.write(build_get_values())
            
            payload = read_packet(ser, timeout=dt)
            if payload:
                vals = parse_get_values(payload)
                if vals:
                    timestamps.append(t_now)
                    rpm_data.append(vals['rpm'])
                    current_data.append(vals['motor_current'])
                    duty_data.append(vals['duty_cycle'])
                    voltage_data.append(vals['v_in'])
                    phase.append('step_up')
            
            elapsed = time.monotonic() - t_start
            sleep_until = (len(timestamps)) * dt
            if sleep_until > elapsed:
                time.sleep(sleep_until - elapsed)
        
        # Phase 3: Step down (use brake command to avoid reversing)
        print(f"  Phase 3: Braking to stop for {settle_time}s")
        brake_current = 5.0  # Amps — gentle brake, no reverse
        while True:
            t_now = time.monotonic() - t_start
            if t_now >= total_time:
                break
            
            # Use brake command until stopped, then RPM 0 to hold
            if len(rpm_data) > 0 and abs(rpm_data[-1]) < 200:
                ser.write(build_set_rpm(0))  # hold at zero once stopped
            else:
                ser.write(build_set_brake(brake_current))
            ser.write(build_get_values())
            
            payload = read_packet(ser, timeout=dt)
            if payload:
                vals = parse_get_values(payload)
                if vals:
                    timestamps.append(t_now)
                    rpm_data.append(vals['rpm'])
                    current_data.append(vals['motor_current'])
                    duty_data.append(vals['duty_cycle'])
                    voltage_data.append(vals['v_in'])
                    phase.append('step_down')
            
            elapsed = time.monotonic() - t_start
            sleep_until = (len(timestamps)) * dt
            if sleep_until > elapsed:
                time.sleep(sleep_until - elapsed)
    
    finally:
        # Always stop motor — use brake first, then RPM 0
        for _ in range(3):
            ser.write(build_set_brake(5.0))
            time.sleep(0.02)
        time.sleep(0.5)
        for _ in range(5):
            ser.write(build_set_rpm(0))
            time.sleep(0.01)
        print("  Motor stopped.")
    
    print(f"\n  Collected {len(timestamps)} samples in {timestamps[-1]:.2f}s")
    print(f"  Effective sample rate: {len(timestamps)/timestamps[-1]:.1f} Hz")
    
    return {
        'time': np.array(timestamps),
        'rpm': np.array(rpm_data),
        'current': np.array(current_data),
        'duty': np.array(duty_data),
        'voltage': np.array(voltage_data),
        'phase': phase,
        'target_rpm': target_rpm,
        'step_time': t_step,
    }


# ============================================================================
# Multi-Step Test
# ============================================================================
def run_multi_step_test(ser: serial.Serial, rpm_steps: list,
                        hold_time: float = 3.0, settle_time: float = 2.0,
                        sample_rate: float = 200.0):
    """
    Execute a multi-step test through a sequence of RPM setpoints.

    Example: rpm_steps=[5000, 9000, 5000] does
      baseline(0) → 5000 → 9000 → 5000 → brake → 0
    """
    dt = 1.0 / sample_rate

    # Data storage
    timestamps = []
    rpm_data = []
    current_data = []
    duty_data = []
    voltage_data = []
    phase = []

    n_steps = len(rpm_steps)
    total_time = settle_time + n_steps * hold_time + settle_time

    print(f"\n{'='*60}")
    print(f"  VESC PID MULTI-STEP RESPONSE TEST")
    print(f"{'='*60}")
    print(f"  Steps:          0 → {' → '.join(str(s) for s in rpm_steps)} → brake")
    print(f"  Hold time:      {hold_time}s per step")
    print(f"  Sample rate:    ~{sample_rate:.0f} Hz")
    print(f"  Total duration: ~{total_time:.1f}s")
    print(f"{'='*60}")
    print()
    print("  ⚠ Make sure the car is on a STAND (wheels off ground)!")
    print("  ⚠ Press Ctrl+C at any time to abort safely")
    print()

    # Countdown
    for i in range(3, 0, -1):
        print(f"  Starting in {i}...")
        time.sleep(1.0)
    print("  GO!")

    t_start = time.monotonic()
    step_times = []

    def sample_loop(phase_label, duration, rpm_target=0, use_brake=False):
        """Inner loop: command motor, record data for `duration` seconds."""
        t_phase_start = time.monotonic() - t_start
        while True:
            t_now = time.monotonic() - t_start
            if t_now >= t_phase_start + duration:
                break

            if use_brake:
                # Brake mode: apply brake current, switch to RPM 0 when stopped
                if len(rpm_data) > 0 and abs(rpm_data[-1]) < 200:
                    ser.write(build_set_rpm(0))
                else:
                    ser.write(build_set_brake(5.0))
            else:
                ser.write(build_set_rpm(rpm_target))

            ser.write(build_get_values())

            payload = read_packet(ser, timeout=dt)
            if payload:
                vals = parse_get_values(payload)
                if vals:
                    timestamps.append(t_now)
                    rpm_data.append(vals['rpm'])
                    current_data.append(vals['motor_current'])
                    duty_data.append(vals['duty_cycle'])
                    voltage_data.append(vals['v_in'])
                    phase.append(phase_label)

            # Maintain timing
            elapsed = time.monotonic() - t_start
            sleep_until = len(timestamps) * dt
            if sleep_until > elapsed:
                time.sleep(sleep_until - elapsed)

    try:
        # Phase: Baseline
        print(f"  Phase 0: Baseline (RPM 0) for {settle_time}s")
        sample_loop('baseline', settle_time, rpm_target=0)

        # Phases: Each step
        for i, target in enumerate(rpm_steps):
            step_times.append(time.monotonic() - t_start)
            print(f"  Phase {i+1}: Step to RPM {target} for {hold_time}s")
            sample_loop(f'step_{i}', hold_time, rpm_target=target)

        # Phase: Brake to stop
        step_times.append(time.monotonic() - t_start)
        print(f"  Phase {n_steps+1}: Braking to stop for {settle_time}s")
        sample_loop('brake', settle_time, use_brake=True)

    finally:
        # Always stop motor — use brake first, then RPM 0
        for _ in range(3):
            ser.write(build_set_brake(5.0))
            time.sleep(0.02)
        time.sleep(0.5)
        for _ in range(5):
            ser.write(build_set_rpm(0))
            time.sleep(0.01)
        print("  Motor stopped.")

    print(f"\n  Collected {len(timestamps)} samples in {timestamps[-1]:.2f}s")
    print(f"  Effective sample rate: {len(timestamps)/timestamps[-1]:.1f} Hz")

    return {
        'time': np.array(timestamps),
        'rpm': np.array(rpm_data),
        'current': np.array(current_data),
        'duty': np.array(duty_data),
        'voltage': np.array(voltage_data),
        'phase': phase,
        'target_rpm': rpm_steps[-1],
        'step_time': step_times[0] if step_times else 0,
        'rpm_steps': rpm_steps,
        'step_times': step_times,
    }


# ============================================================================
# Analysis
# ============================================================================
def analyze_results(data: dict):
    """Analyze the step response data."""
    t = data['time']
    rpm = data['rpm']
    target = data['target_rpm']
    step_time = data['step_time']
    
    # Extract step-up phase
    step_mask = np.array([p == 'step_up' for p in data['phase']])
    t_step = t[step_mask] - step_time
    rpm_step = rpm[step_mask]
    
    if len(rpm_step) < 10:
        print("ERROR: Not enough step-up samples. Check serial connection.")
        return
    
    # Steady state: last 20% of step phase
    n = len(rpm_step)
    steady_samples = rpm_step[int(n * 0.8):]
    steady_state = np.mean(steady_samples)
    steady_std = np.std(steady_samples)
    
    print(f"\n{'='*60}")
    print(f"  STEP RESPONSE ANALYSIS")
    print(f"{'='*60}")
    print(f"  Target RPM:               {target}")
    print(f"  Steady-state RPM:         {steady_state:.0f} ± {steady_std:.0f}")
    print(f"  Steady-state error:       {abs(target - steady_state):.0f} RPM"
          f" ({abs(target - steady_state)/target*100:.1f}%)")
    print(f"  Samples in step phase:    {n}")
    
    # Rise time (10% → 90%)
    threshold_10 = 0.10 * steady_state
    threshold_90 = 0.90 * steady_state
    
    t_10 = None
    t_90 = None
    for i in range(len(rpm_step)):
        if t_10 is None and rpm_step[i] >= threshold_10:
            t_10 = t_step[i]
        if t_90 is None and rpm_step[i] >= threshold_90:
            t_90 = t_step[i]
    
    if t_10 is not None and t_90 is not None:
        rise_time = t_90 - t_10
        print(f"\n  Rise time (10%→90%):      {rise_time*1000:.1f} ms")
    else:
        rise_time = None
        print(f"\n  Rise time:                Could not compute")
    
    # Overshoot
    peak_rpm = np.max(rpm_step)
    peak_idx = np.argmax(rpm_step)
    peak_time = t_step[peak_idx]
    overshoot = (peak_rpm - steady_state) / steady_state * 100
    
    print(f"  Peak RPM:                 {peak_rpm:.0f}")
    print(f"  Overshoot:                {overshoot:.1f}%")
    print(f"  Time to peak:             {peak_time*1000:.1f} ms")
    
    # Settling time (±2% band)
    settling_band = 0.02 * steady_state
    settled = np.abs(rpm_step - steady_state) <= settling_band
    settling_time = None
    for i in range(len(settled) - 1, -1, -1):
        if not settled[i]:
            if i < len(t_step) - 1:
                settling_time = t_step[i + 1]
            break
    
    if settling_time is not None:
        print(f"  Settling time (±2%):      {settling_time*1000:.1f} ms")
    else:
        print(f"  Settling time:            Within ±2% immediately (or never)")
    
    # Oscillation count (with dead band to ignore noise)
    # Use ±3*std as the noise threshold — only count crossings that
    # exceed this dead band as real oscillations
    noise_band = 3 * steady_std  # ~99.7% of noise is within ±3σ
    oscillation_count = 0
    if t_90 is not None:
        idx_90 = np.searchsorted(t_step, t_90)
        if idx_90 < len(rpm_step):
            post_rise = rpm_step[idx_90:] - steady_state
            # Only count transitions that cross OUTSIDE the noise band
            above_band = post_rise > noise_band
            below_band = post_rise < -noise_band
            # An oscillation = goes above band then below (or vice versa)
            state = 0  # 0=in band, 1=above, -1=below
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
    
    print(f"  Steady-state noise (σ):   ±{steady_std:.0f} RPM")
    
    # Current during step
    current_step = data['current'][step_mask]
    print(f"\n  Peak motor current:       {np.max(np.abs(current_step)):.1f} A")
    print(f"  Steady-state current:     {np.mean(current_step[int(n*0.8):]):.1f} A")
    
    # Battery voltage
    voltage = data['voltage'][step_mask]
    print(f"  Battery voltage:          {np.mean(voltage):.1f} V  "
          f"(min: {np.min(voltage):.1f}V during test)")
    
    # PID Recommendations
    print(f"\n{'─'*60}")
    print(f"  TUNING RECOMMENDATIONS")
    print(f"{'─'*60}")
    
    issues = []
    
    if overshoot > 25:
        issues.append(("HIGH OVERSHOOT",
                       f"{overshoot:.0f}% → Reduce Kp by ~30% or increase Kd by ~50%"))
    elif overshoot > 10:
        issues.append(("MODERATE OVERSHOOT",
                       f"{overshoot:.0f}% → Slightly reduce Kp or increase Kd"))
    
    if rise_time is not None:
        if rise_time > 0.5:
            issues.append(("SLOW RESPONSE",
                          f"Rise time {rise_time*1000:.0f}ms → Increase Kp by ~30%"))
        elif rise_time < 0.05:
            issues.append(("VERY AGGRESSIVE",
                          f"Rise time {rise_time*1000:.0f}ms → May cause oscillation"))
    
    if settling_time is not None and settling_time > 1.0:
        issues.append(("SLOW SETTLING",
                       f"Settling time {settling_time*1000:.0f}ms → Increase Kd"))
    
    ss_error = abs(target - steady_state) / target * 100
    if ss_error > 5:
        issues.append(("STEADY-STATE ERROR",
                       f"{ss_error:.1f}% → Increase Ki by ~30%"))
    
    if oscillation_count > 3:
        issues.append(("OSCILLATORY",
                       f"{oscillation_count} oscillations → Reduce Ki by 50%, increase Kd"))
    
    if not issues:
        print("  ✓ PID tuning looks GOOD for F1TENTH use")
    else:
        for label, suggestion in issues:
            print(f"  ⚠ {label}: {suggestion}")
    
    # Reference table
    print(f"\n{'─'*60}")
    print(f"  {'Metric':<25} {'Your Value':<15} {'Good Range':<15} {'Status':<10}")
    print(f"  {'─'*25} {'─'*15} {'─'*15} {'─'*10}")
    
    rt_str = f"{rise_time*1000:.0f} ms" if rise_time else "N/A"
    st_str = f"{settling_time*1000:.0f} ms" if settling_time else "N/A"
    
    def status_icon(ok): return "✓" if ok else "⚠"
    
    rt_ok = rise_time is not None and rise_time <= 0.2
    os_ok = overshoot < 15
    st_ok = settling_time is None or settling_time <= 0.5
    se_ok = ss_error < 3
    oc_ok = oscillation_count <= 2
    
    print(f"  {'Rise time':<25} {rt_str:<15} {'50-200 ms':<15} {status_icon(rt_ok):<10}")
    print(f"  {'Overshoot':<25} {f'{overshoot:.1f}%':<15} {'< 15%':<15} {status_icon(os_ok):<10}")
    print(f"  {'Settling time':<25} {st_str:<15} {'< 500 ms':<15} {status_icon(st_ok):<10}")
    print(f"  {'Steady-state error':<25} {f'{ss_error:.1f}%':<15} {'< 3%':<15} {status_icon(se_ok):<10}")
    print(f"  {'Oscillations':<25} {f'{oscillation_count}':<15} {'0-2':<15} {status_icon(oc_ok):<10}")
    print(f"  {'Noise (σ)':<25} {f'±{steady_std:.0f} RPM':<15} {'< 50 RPM':<15} {status_icon(steady_std < 50):<10}")
    
    # Only show specific, actionable recommendations if there are issues
    if issues:
        print(f"\n{'─'*60}")
        print(f"  SPECIFIC ACTIONS (in VESC Tool → Motor Settings → PID → Speed)")
        print(f"{'─'*60}")
        for label, suggestion in issues:
            print(f"  → {suggestion}")
    
    print(f"{'='*60}")
    
    return {
        'rise_time': rise_time,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'steady_state_error': ss_error,
        'oscillations': oscillation_count,
    }


def analyze_multi_step_results(data: dict):
    """Analyze multi-step test results, showing metrics for each transition."""
    t = data['time']
    rpm = data['rpm']
    rpm_steps = data['rpm_steps']
    phases = data['phase']

    print(f"\n{'='*60}")
    print(f"  MULTI-STEP RESPONSE ANALYSIS")
    print(f"{'='*60}")

    # Full RPM sequence: baseline(0) → step_0 → step_1 → ... → brake(0)
    all_rpms = [0] + list(rpm_steps) + [0]

    for i in range(len(rpm_steps)):
        from_rpm = all_rpms[i]
        to_rpm = all_rpms[i + 1]
        phase_label = f'step_{i}'

        # Extract data for this phase
        mask = np.array([p == phase_label for p in phases])
        if not np.any(mask):
            print(f"\n  Transition {from_rpm} → {to_rpm}: NO DATA")
            continue

        t_phase = t[mask]
        rpm_phase = rpm[mask]
        t_0 = t_phase[0]
        t_phase = t_phase - t_0  # relative time

        # Steady state = last 20% of this phase
        n = len(rpm_phase)
        steady_samples = rpm_phase[int(n * 0.8):]
        steady_state = np.mean(steady_samples)
        steady_std = np.std(steady_samples)

        print(f"\n{'─'*60}")
        print(f"  Transition: {from_rpm} → {to_rpm} ERPM")
        print(f"{'─'*60}")
        print(f"  Steady-state RPM:    {steady_state:.0f} ± {steady_std:.0f}")

        ss_err = abs(to_rpm - steady_state)
        ss_pct = ss_err / max(abs(to_rpm), 1) * 100
        print(f"  Steady-state error:  {ss_err:.0f} RPM ({ss_pct:.1f}%)")

        step_magnitude = abs(to_rpm - from_rpm)

        if to_rpm > from_rpm:
            # Step up
            threshold_10 = from_rpm + 0.10 * (steady_state - from_rpm)
            threshold_90 = from_rpm + 0.90 * (steady_state - from_rpm)
            t_10 = t_90 = None
            for j in range(len(rpm_phase)):
                if t_10 is None and rpm_phase[j] >= threshold_10:
                    t_10 = t_phase[j]
                if t_90 is None and rpm_phase[j] >= threshold_90:
                    t_90 = t_phase[j]
            if t_10 is not None and t_90 is not None:
                print(f"  Rise time (10%→90%): {(t_90 - t_10)*1000:.1f} ms")

            # Overshoot
            peak = np.max(rpm_phase)
            overshoot = (peak - steady_state) / max(step_magnitude, 1) * 100
            print(f"  Peak RPM:            {peak:.0f}")
            print(f"  Overshoot:           {overshoot:.1f}%")

        elif to_rpm < from_rpm:
            # Step down
            threshold_90_down = from_rpm - 0.10 * (from_rpm - steady_state)
            threshold_10_down = from_rpm - 0.90 * (from_rpm - steady_state)
            t_10 = t_90 = None
            for j in range(len(rpm_phase)):
                if t_10 is None and rpm_phase[j] <= threshold_90_down:
                    t_10 = t_phase[j]
                if t_90 is None and rpm_phase[j] <= threshold_10_down:
                    t_90 = t_phase[j]
            if t_10 is not None and t_90 is not None:
                print(f"  Fall time (10%→90%): {(t_90 - t_10)*1000:.1f} ms")

            # Undershoot
            trough = np.min(rpm_phase)
            undershoot = (steady_state - trough) / max(step_magnitude, 1) * 100
            print(f"  Trough RPM:          {trough:.0f}")
            print(f"  Undershoot:          {undershoot:.1f}%")

        # Settling time (±2% of step magnitude, min 100 RPM band)
        settling_band = max(0.02 * step_magnitude, 100)
        settled = np.abs(rpm_phase - steady_state) <= settling_band
        settling_time = None
        for j in range(len(settled) - 1, -1, -1):
            if not settled[j]:
                if j < len(t_phase) - 1:
                    settling_time = t_phase[j + 1]
                break
        if settling_time is not None:
            print(f"  Settling time (±2%): {settling_time*1000:.1f} ms")

        print(f"  Noise (σ):           ±{steady_std:.0f} RPM")

    # Brake phase analysis
    brake_mask = np.array([p == 'brake' for p in phases])
    if np.any(brake_mask):
        t_brake = t[brake_mask]
        rpm_brake = rpm[brake_mask]

        print(f"\n{'─'*60}")
        print(f"  Brake Phase: {rpm_steps[-1]} → 0 ERPM")
        print(f"{'─'*60}")

        # Time to stop (rpm < 100)
        t_0 = t_brake[0]
        stopped = False
        for j in range(len(rpm_brake)):
            if abs(rpm_brake[j]) < 100:
                print(f"  Time to stop:        {(t_brake[j] - t_0)*1000:.0f} ms")
                stopped = True
                break
        if not stopped:
            print(f"  Time to stop:        Did not reach 0 within settle time")

        # Check for reverse
        min_rpm = np.min(rpm_brake)
        if min_rpm < -200:
            print(f"  ⚠ REVERSE DETECTED:  {min_rpm:.0f} RPM (braking overshoot!)")
        else:
            print(f"  ✓ No reverse driving detected")

        print(f"  Final RPM:           {rpm_brake[-1]:.0f}")

    # Summary
    print(f"\n{'─'*60}")
    print(f"  Peak motor current:  {np.max(np.abs(data['current'])):.1f} A")
    print(f"  Battery voltage:     {np.mean(data['voltage']):.1f} V"
          f"  (min: {np.min(data['voltage']):.1f}V)")
    print(f"{'='*60}")


def save_csv(data: dict, filepath: str):
    """Save recorded data to CSV."""
    import csv
    
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'rpm', 'motor_current_A', 'duty_cycle', 'voltage_V', 'phase'])
        for i in range(len(data['time'])):
            writer.writerow([
                f"{data['time'][i]:.4f}",
                f"{data['rpm'][i]:.0f}",
                f"{data['current'][i]:.2f}",
                f"{data['duty'][i]:.4f}",
                f"{data['voltage'][i]:.1f}",
                data['phase'][i],
            ])
    
    print(f"\n  Data saved to: {filepath}")


# ============================================================================
# VESC Tool CSV Analysis (Mode 2: --analyze)
# ============================================================================

def load_vesc_tool_csv(filepath: str) -> dict:
    """
    Load VESC Tool CSV export.

    VESC Tool writes CSVs with semicolons (European locale) or commas.
    """
    with open(filepath, 'r') as f:
        first_line = f.readline()

    delimiter = ';' if ';' in first_line else ','

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
            vals = []
            for row in rows:
                val_str = row[key].strip().replace(',', '.')
                vals.append(float(val_str))
            data[key.strip()] = np.array(vals)
        except (ValueError, TypeError):
            data[key.strip()] = np.array([row[key] for row in rows])

    return data


def _find_rpm_column(data: dict) -> str:
    """Find the ERPM/RPM column in VESC Tool data."""
    candidates = ['rpm', 'erpm', 'RPM', 'ERPM', 'rpm_filtered', 'speed_erpm']
    for c in candidates:
        if c in data:
            return c
    for key in data.keys():
        if 'rpm' in key.lower() or 'erpm' in key.lower():
            return key
    print(f"ERROR: Could not find RPM column. Available: {list(data.keys())}")
    sys.exit(1)


def _find_time_column(data: dict) -> Tuple[str, np.ndarray]:
    """Find and normalise the time column (return in seconds)."""
    candidates = ['ms_today', 'time_ms', 'time', 'Time', 'timestamp', 'ms', 'time_s']
    for c in candidates:
        if c in data:
            t = data[c] - data[c][0]
            if t[-1] > 1000:
                t = t / 1000.0
            return c, t
    print("WARNING: No time column found, assuming 100 Hz sample rate")
    n = len(next(iter(data.values())))
    return 'synthetic', np.arange(n) / 100.0


def _detect_step_in_csv(rpm: np.ndarray, time: np.ndarray):
    """Detect the step-up event in RPM data from a VESC Tool CSV."""
    rpm_sorted = np.sort(rpm)
    high_rpm = rpm_sorted[len(rpm_sorted) * 3 // 4:]
    target_rpm = np.median(high_rpm)

    if target_rpm < 100:
        print("ERROR: Could not detect a step response. Max RPM too low.")
        print(f"  RPM range: {rpm.min():.0f} to {rpm.max():.0f}")
        sys.exit(1)

    threshold_10 = 0.10 * target_rpm
    threshold_90 = 0.90 * target_rpm

    above_threshold = np.where(rpm > threshold_10)[0]
    if len(above_threshold) == 0:
        print("ERROR: RPM never exceeds 10% of detected target")
        sys.exit(1)
    step_start_idx = max(0, above_threshold[0] - 5)

    above_90 = np.where(rpm > threshold_90)[0]
    if len(above_90) == 0:
        step_end_idx = len(rpm) - 1
    else:
        plateau_start = above_90[0]
        post_plateau = rpm[plateau_start:]
        below_50_post = np.where(post_plateau < 0.50 * target_rpm)[0]
        step_end_idx = plateau_start + below_50_post[0] if len(below_50_post) > 0 else len(rpm) - 1

    return step_start_idx, step_end_idx, target_rpm


def _analyze_csv_step_response(rpm, time, step_start, step_end, target_rpm):
    """Analyse a step response from VESC Tool CSV data (Mode 2)."""
    t = time[step_start:step_end] - time[step_start]
    r = rpm[step_start:step_end]
    n = len(r)
    steady_samples = r[int(n * 0.8):]
    steady_state = np.mean(steady_samples)
    steady_state_std = np.std(steady_samples)

    print(f"\n{'='*60}")
    print(f"  STEP RESPONSE ANALYSIS  (VESC Tool CSV)")
    print(f"{'='*60}")
    print(f"  Detected target RPM:      {target_rpm:.0f}")
    print(f"  Steady-state RPM:         {steady_state:.0f} ± {steady_state_std:.0f}")
    print(f"  Steady-state error:       {abs(target_rpm - steady_state):.0f} RPM "
          f"({abs(target_rpm - steady_state)/target_rpm*100:.1f}%)")
    print(f"  Step duration:            {t[-1]:.2f}s")
    print(f"  Samples:                  {n}")

    # Rise time
    threshold_10 = 0.10 * steady_state
    threshold_90 = 0.90 * steady_state
    t_10 = t_90 = None
    for i in range(len(r)):
        if t_10 is None and r[i] >= threshold_10:
            t_10 = t[i]
        if t_90 is None and r[i] >= threshold_90:
            t_90 = t[i]
    rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None
    if rise_time is not None:
        print(f"\n  Rise time (10%→90%):      {rise_time*1000:.1f} ms")
    else:
        print(f"\n  Rise time:                Could not compute")

    # Overshoot
    peak_rpm = np.max(r)
    overshoot_pct = (peak_rpm - steady_state) / steady_state * 100
    peak_time = t[np.argmax(r)]
    print(f"  Peak RPM:                 {peak_rpm:.0f}")
    print(f"  Overshoot:                {overshoot_pct:.1f}%")
    print(f"  Time to peak:             {peak_time*1000:.1f} ms")

    # Settling time
    settling_band = 0.02 * steady_state
    settled = np.abs(r - steady_state) <= settling_band
    settling_time = None
    for i in range(len(settled) - 1, -1, -1):
        if not settled[i]:
            if i < len(t) - 1:
                settling_time = t[i + 1]
            break
    if settling_time is not None:
        print(f"  Settling time (±2%):      {settling_time*1000:.1f} ms")
    else:
        print(f"  Settling time:            Already within ±2%")

    # Oscillation count
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

    # Recommendations
    ss_error_pct = abs(target_rpm - steady_state) / target_rpm * 100
    issues = []
    if overshoot_pct > 25:
        issues.append(("HIGH OVERSHOOT",
                       f"{overshoot_pct:.0f}% → Reduce Kp or increase Kd"))
    elif overshoot_pct > 10:
        issues.append(("MODERATE OVERSHOOT",
                       f"{overshoot_pct:.0f}% → Consider slightly reducing Kp"))
    if rise_time is not None and rise_time > 0.5:
        issues.append(("SLOW RESPONSE",
                       f"Rise time {rise_time*1000:.0f}ms → Increase Kp"))
    if settling_time is not None and settling_time > 1.0:
        issues.append(("SLOW SETTLING",
                       f"Settling time {settling_time*1000:.0f}ms → Increase Kd"))
    if ss_error_pct > 5:
        issues.append(("STEADY-STATE ERROR",
                       f"{ss_error_pct:.1f}% → Increase Ki"))
    if oscillation_count > 3:
        issues.append(("OSCILLATORY",
                       f"{oscillation_count} oscillations → Reduce Ki, increase Kd"))

    print(f"\n{'─'*60}")
    print(f"  TUNING RECOMMENDATIONS")
    print(f"{'─'*60}")
    if not issues:
        print("  ✓ PID tuning looks GOOD")
    else:
        for label, suggestion in issues:
            print(f"  ⚠ {label}: {suggestion}")

    # Summary table
    rt_str = f"{rise_time*1000:.0f} ms" if rise_time else "N/A"
    st_str = f"{settling_time*1000:.0f} ms" if settling_time else "N/A"
    def status_icon(ok): return "✓" if ok else "⚠"
    rt_ok = rise_time is not None and rise_time <= 0.2
    os_ok = overshoot_pct < 15
    st_ok = settling_time is None or settling_time <= 0.5
    se_ok = ss_error_pct < 3
    oc_ok = oscillation_count <= 2

    print(f"\n  {'Metric':<25} {'Your Value':<15} {'Good Range':<15} {'Status':<10}")
    print(f"  {'─'*25} {'─'*15} {'─'*15} {'─'*10}")
    print(f"  {'Rise time':<25} {rt_str:<15} {'50-200 ms':<15} {status_icon(rt_ok):<10}")
    print(f"  {'Overshoot':<25} {f'{overshoot_pct:.1f}%':<15} {'< 15%':<15} {status_icon(os_ok):<10}")
    print(f"  {'Settling time':<25} {st_str:<15} {'< 500 ms':<15} {status_icon(st_ok):<10}")
    print(f"  {'Steady-state error':<25} {f'{ss_error_pct:.1f}%':<15} {'< 3%':<15} {status_icon(se_ok):<10}")
    print(f"  {'Oscillations':<25} {f'{oscillation_count}':<15} {'0-2':<15} {status_icon(oc_ok):<10}")
    print(f"{'='*60}")


def run_csv_analysis(csv_path: str, target_rpm_override: float = None):
    """Entry point for Mode 2: analyse a VESC Tool CSV log."""
    if not os.path.exists(csv_path):
        print(f"ERROR: File not found: {csv_path}")
        sys.exit(1)

    data = load_vesc_tool_csv(csv_path)
    rpm_col = _find_rpm_column(data)
    rpm = data[rpm_col]
    print(f"Using RPM column: '{rpm_col}' (range: {rpm.min():.0f} to {rpm.max():.0f})")

    time_col, t = _find_time_column(data)
    print(f"Using time column: '{time_col}' (duration: {t[-1]:.2f}s)")

    step_start, step_end, detected_target = _detect_step_in_csv(rpm, t)
    target = target_rpm_override if target_rpm_override is not None else detected_target
    print(f"\nStep detected: t={t[step_start]:.2f}s to t={t[step_end]:.2f}s")
    print(f"Target RPM: {target:.0f}")

    _analyze_csv_step_response(rpm, t, step_start, step_end, target)


# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='VESC PID Step Response Tester & Analyzer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:
    # Simple step: 0 → 5000 → 0
    python3 vesc_pid_test.py --target-rpm 5000

    # Multi-step: 0 → 5000 → 9000 → 5000 → 0
    python3 vesc_pid_test.py --steps 5000,9000,5000

    # Analyze a VESC Tool CSV log (no serial connection needed)
    python3 vesc_pid_test.py --analyze /path/to/vesc_tool_log.csv

Make sure:
    - VESC Tool is NOT connected (close it first)
    - Car is on a STAND (wheels off the ground)
    - Battery is charged
""")
    
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                        help='Serial port for VESC (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--target-rpm', type=int, default=5000,
                        help='Target ERPM for single step test (default: 5000 ≈ 1.1 m/s)')
    parser.add_argument('--steps', type=str, default=None,
                        help='Comma-separated RPM values for multi-step test (e.g. 5000,9000,5000)')
    parser.add_argument('--hold-time', type=float, default=3.0,
                        help='How long to hold at each RPM (default: 3.0s)')
    parser.add_argument('--settle-time', type=float, default=2.0,
                        help='Baseline/settle time before and after test (default: 2.0s)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output CSV file (default: pid_step_YYYYMMDD_HHMMSS.csv)')
    parser.add_argument('--analyze', type=str, default=None,
                        metavar='CSV_FILE',
                        help='Analyze a VESC Tool CSV log instead of running a serial test')
    
    args = parser.parse_args()
    
    # ---- Mode 2: Analyze a VESC Tool CSV log ----
    if args.analyze is not None:
        target_override = args.target_rpm if '--target-rpm' in sys.argv else None
        run_csv_analysis(args.analyze, target_override)
        return
    
    # ---- Mode 1: Direct serial test ----
    # Default output filename
    if args.output is None:
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        args.output = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'data',
            f'pid_step_{timestamp}.csv')
        os.makedirs(os.path.dirname(args.output), exist_ok=True)
    
    print(f"Connecting to VESC at {args.port} ({args.baud} baud)...")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {args.port}: {e}")
        print("  - Is VESC Tool still connected? Close it first.")
        print("  - Is the USB cable connected?")
        print("  - Check port name (try: ls /dev/ttyACM*)")
        sys.exit(1)
    
    time.sleep(0.5)  # Wait for serial to stabilize
    ser.reset_input_buffer()
    
    # Quick connectivity check: request values
    ser.write(build_get_values())
    payload = read_packet(ser, timeout=1.0)
    if not payload:
        print("ERROR: No response from VESC. Check connection and power.")
        ser.close()
        sys.exit(1)
    
    vals = parse_get_values(payload)
    if not vals:
        print("ERROR: Invalid response from VESC.")
        ser.close()
        sys.exit(1)
    
    print(f"  Connected! Battery: {vals['v_in']:.1f}V, Current RPM: {vals['rpm']:.0f}")
    
    if vals['v_in'] < 10.0:
        print("  ⚠ WARNING: Battery voltage is low!")
    
    try:
        if args.steps:
            # Multi-step test
            steps = [int(s) for s in args.steps.split(',')]
            data = run_multi_step_test(
                ser,
                rpm_steps=steps,
                hold_time=args.hold_time,
                settle_time=args.settle_time,
            )
        else:
            # Single step test
            data = run_step_test(
                ser,
                target_rpm=args.target_rpm,
                hold_time=args.hold_time,
                settle_time=args.settle_time,
            )
        
        # Analyze
        if 'rpm_steps' in data:
            analyze_multi_step_results(data)
        else:
            analyze_results(data)
        
        # Save
        save_csv(data, args.output)
        
    except KeyboardInterrupt:
        print("\n\n  Ctrl+C detected — braking motor!")
        for _ in range(5):
            ser.write(build_set_brake(10.0))
            time.sleep(0.02)
        time.sleep(0.5)
        for _ in range(3):
            ser.write(build_set_rpm(0))
            time.sleep(0.01)
        print("  Motor stopped safely.")
    
    finally:
        # Always stop motor and close
        try:
            for _ in range(3):
                ser.write(build_set_brake(5.0))
                time.sleep(0.02)
            time.sleep(0.3)
            for _ in range(3):
                ser.write(build_set_rpm(0))
                time.sleep(0.01)
            ser.close()
        except Exception:
            pass
    
    print("\nDone. Adjust PID in VESC Tool and re-run to compare.")


if __name__ == '__main__':
    main()
