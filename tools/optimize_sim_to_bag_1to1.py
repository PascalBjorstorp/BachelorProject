#!/usr/bin/env python3
"""
1:1 Sim-to-Real CPU plant calibration — Phase 2 (Full 100s, parallel DE).

Uses differential_evolution with up to 12 workers to globally optimise 6
plant parameters against the consensus peak |e_y|, mean |e_y|, and mean vx
profile from all 3 real-car bags (full 100-second closed-loop runs).

Usage:
    python3 tools/optimize_sim_to_bag_1to1.py
"""

import os
import sys
import subprocess
import tempfile
import pandas as pd
import numpy as np
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Paths ──────────────────────────────────────────────────────────────────
REPO_ROOT  = "/home/akselmo/Documents/GitHub/BachelorProject"
CPU_BINARY = os.path.join(REPO_ROOT, "MPC/test_sim_drive")
CPU_CWD    = os.path.join(REPO_ROOT, "MPC")
RACELINE   = os.path.join(REPO_ROOT, "MPC/trajectories/my_track_raceline.csv")
OUT_DIR    = os.path.join(REPO_ROOT, "tools/output")
ENV_SCRIPT = os.path.join(OUT_DIR, "optimized_real_car_env.sh")

BAG_PATHS = [
    os.path.join(REPO_ROOT, "tools/output/mpc_deep_diag_20260519_125104/state_replay.csv"),
    os.path.join(REPO_ROOT, "tools/output/hw_vs_sim_ros2/state_replay.csv"),
    os.path.join(REPO_ROOT, "tools/output/hw_vs_sim_udp/state_replay.csv"),
]

QP_SCALE   = 262144.0
N_WORKERS  = 12   # parallel DE evaluations

# ── Load raceline ──────────────────────────────────────────────────────────
race = pd.read_csv(RACELINE, comment='#',
    names=['s_m','x_m','y_m','psi_rad','kappa_radpm','vx_mps','ax_mps2','d_left_m','d_right_m'])
RX, RY  = race['x_m'].to_numpy(), race['y_m'].to_numpy()
N_WPS   = len(race)

# Mask: skip tight-wall sections (geometry-limited, not tire-slip)
OPT_MASK = (race['d_left_m'].to_numpy() >= 0.30) & (race['d_right_m'].to_numpy() >= 0.30)
print(f"Raceline: {N_WPS} waypoints, {OPT_MASK.sum()} in optimisation mask (≥0.30m walls)")

def closest_wp_vec(xs, ys):
    """Vectorised nearest-waypoint lookup."""
    wps = np.empty(len(xs), dtype=int)
    for i, (x, y) in enumerate(zip(xs, ys)):
        d = (RX - x)**2 + (RY - y)**2
        wps[i] = np.argmin(d)
    return wps

# ── Build consensus target from all bags ───────────────────────────────────
def load_bag_target():
    print("Loading bag targets from all 3 bags (full 100s)...")
    peak_ey = np.zeros(N_WPS)
    sum_ey  = np.zeros(N_WPS)
    sum_vx  = np.zeros(N_WPS)
    cnt     = np.zeros(N_WPS)

    for path in BAG_PATHS:
        df  = pd.read_csv(path)
        x   = df['x_fp'].to_numpy()  / QP_SCALE
        y   = df['y_fp'].to_numpy()  / QP_SCALE
        vx  = df['velocity_fp'].to_numpy() / QP_SCALE
        ey  = np.abs(df['e_y_fp'].to_numpy() / QP_SCALE)

        mask = vx > 0.5
        x, y, vx, ey = x[mask], y[mask], vx[mask], ey[mask]
        wps = closest_wp_vec(x, y)

        for i, wp in enumerate(wps):
            if ey[i] > peak_ey[wp]:
                peak_ey[wp] = ey[i]
            sum_ey[wp] += ey[i]
            sum_vx[wp] += vx[i]
            cnt[wp]    += 1

    mean_ey = np.where(cnt > 0, sum_ey / np.maximum(cnt, 1), np.nan)
    mean_vx = np.where(cnt > 0, sum_vx / np.maximum(cnt, 1), np.nan)

    for arr in (peak_ey, mean_ey, mean_vx):
        s = pd.Series(arr)
        s = s.replace(0, np.nan).interpolate('linear').ffill().bfill()
        arr[:] = s.to_numpy()

    return peak_ey, mean_ey, mean_vx

TARGET_PEAK_EY, TARGET_MEAN_EY, TARGET_VX = load_bag_target()
print(f"  peak|e_y|: max={TARGET_PEAK_EY.max():.4f} m, mean={TARGET_PEAK_EY.mean():.4f} m")
print(f"  mean|e_y|: mean={TARGET_MEAN_EY.mean():.4f} m")
print(f"  mean_vx:   mean={TARGET_VX.mean():.3f} m/s")

# ── Run CPU simulator (thread-safe via unique trace path) ──────────────────
def run_simulation(params, duration=100.0):
    mu, Iz, C_Sf, C_Sr, accel_gain, steer_gain = params

    # Each parallel call gets its own unique trace file
    trace = tempfile.mktemp(suffix='.csv', prefix='sim_trace_', dir='/tmp')

    env = os.environ.copy()
    env.update({
        "SIM_DURATION":             f"{duration:.1f}",
        "SIM_TRACE_LOG":            trace,
        "SIM_MU":                   f"{mu:.6f}",
        "SIM_MU_FRONT":             f"{mu:.6f}",
        "SIM_MU_REAR":              f"{mu:.6f}",
        "SIM_IZ":                   f"{Iz:.6f}",
        "SIM_C_SF":                 f"{C_Sf:.6f}",
        "SIM_C_SR":                 f"{C_Sr:.6f}",
        "SIM_C_SF_HIGH_SLIP":       f"{C_Sf:.6f}",
        "SIM_C_SR_HIGH_SLIP":       f"{C_Sr:.6f}",
        "ACCEL_GAIN_POS":           f"{accel_gain:.6f}",
        "SIM_STEER_GAIN":           f"{steer_gain:.6f}",
        "SIM_STEER_GAIN_HIGH_SLIP": f"{steer_gain:.6f}",
        "BODY_SAFETY_MARGIN":       "0.0",
    })

    try:
        subprocess.run([CPU_BINARY], env=env, capture_output=True,
                       cwd=CPU_CWD, timeout=120)
    except subprocess.TimeoutExpired:
        if os.path.exists(trace): os.remove(trace)
        return None

    if not os.path.exists(trace):
        return None

    try:
        sim = pd.read_csv(trace)
        if len(sim) < 200:
            os.remove(trace)
            return None
        sim['abs_ey'] = sim['e_y'].abs()
        collided  = int(sim['wall_hit'].abs().max() > 0)
        peak_ey   = sim.groupby('closest_wp')['abs_ey'].max().reindex(range(N_WPS))
        mean_ey   = sim.groupby('closest_wp')['abs_ey'].mean().reindex(range(N_WPS))
        mean_vx   = sim.groupby('closest_wp')['vx'].mean().reindex(range(N_WPS))
        avg_laptime = None
        laps_done = int(sim['completed_laps'].max()) if 'completed_laps' in sim.columns else 0
        for arr in (peak_ey, mean_ey, mean_vx):
            arr.interpolate(inplace=True); arr.ffill(inplace=True); arr.bfill(inplace=True)
        os.remove(trace)
        return {
            'peak_ey':  peak_ey.to_numpy(),
            'mean_ey':  mean_ey.to_numpy(),
            'mean_vx':  mean_vx.to_numpy(),
            'collided': collided,
            'laps':     laps_done,
        }
    except Exception as e:
        if os.path.exists(trace): os.remove(trace)
        return None

# ── Loss function (called by DE, must be picklable) ────────────────────────
call_count = [0]

def loss_function(params):
    mu, Iz, C_Sf, C_Sr, accel_gain, steer_gain = params
    call_count[0] += 1

    sim = run_simulation(params, duration=100.0)
    if sim is None:
        return 9999.0

    if sim['collided']:
        # Strong penalty, but still shaped so DE can steer away
        ey_loss = np.mean(np.abs(sim['peak_ey'][OPT_MASK] - TARGET_PEAK_EY[OPT_MASK]))
        return 5000.0 + ey_loss

    m = OPT_MASK
    # Primary: match peak |e_y| shape (most important — real-world sliding behaviour)
    peak_loss = np.mean(np.abs(sim['peak_ey'][m] - TARGET_PEAK_EY[m]))
    # Secondary: match mean |e_y| (steady-state tracking)
    mean_loss = np.mean(np.abs(sim['mean_ey'][m] - TARGET_MEAN_EY[m]))
    # Tertiary: match speed profile
    vx_loss   = np.mean(np.abs(sim['mean_vx'][m] - TARGET_VX[m]))

    loss = peak_loss + 0.5 * mean_loss + 0.2 * vx_loss
    n = call_count[0]
    print(f"  [{n:4d}] loss={loss:.5f}  peak={peak_loss:.5f}  mean={mean_loss:.5f}  vx={vx_loss:.5f}"
          f"  mu={mu:.4f} Iz={Iz:.5f} C_Sf={C_Sf:.4f} C_Sr={C_Sr:.4f}"
          f"  a_gain={accel_gain:.4f} s_gain={steer_gain:.4f}  laps={sim['laps']}")
    sys.stdout.flush()
    return loss

# ── Main ───────────────────────────────────────────────────────────────────
def main():
    print("\n" + "="*60)
    print(" 1:1 SIM-TO-REAL  — Differential Evolution (CPU sim only)")
    print(" Full 100s runs | 12 parallel workers | All 3 bags")
    print("="*60 + "\n")

    # Evaluate warm-start first
    x0 = [1.032718, 0.029192, 3.306717, 2.171281, 0.750000, 0.670000]
    L0 = loss_function(x0)
    print(f"\nWarm-start loss: {L0:.5f}\n")

    # Parameter bounds:  [mu,   Iz,    C_Sf,  C_Sr,  accel, steer]
    bounds = [
        (0.70, 1.30),   # SIM_MU
        (0.018, 0.055), # SIM_IZ
        (2.0,  6.0),    # SIM_C_SF
        (1.2,  4.0),    # SIM_C_SR
        (0.55, 1.10),   # ACCEL_GAIN_POS
        (0.50, 1.05),   # SIM_STEER_GAIN
    ]

    # Seed the initial population from x0 + perturbations
    rng = np.random.default_rng(42)
    init_pop = np.tile(x0, (15, 1))
    scales = np.array([0.15, 0.008, 0.5, 0.3, 0.08, 0.08])
    for i in range(1, 15):
        init_pop[i] += rng.uniform(-scales, scales)
        for j, (lo, hi) in enumerate(bounds):
            init_pop[i, j] = np.clip(init_pop[i, j], lo, hi)

    print(f"Starting Differential Evolution (popsize=15, maxiter=40, workers={N_WORKERS})...")
    result = differential_evolution(
        loss_function,
        bounds,
        init=init_pop,
        strategy='best1bin',
        maxiter=40,
        popsize=15,
        tol=5e-4,
        mutation=(0.5, 1.0),
        recombination=0.7,
        seed=42,
        workers=N_WORKERS,
        updating='deferred',   # required for workers > 1
        disp=True,
    )

    mu, Iz, C_Sf, C_Sr, accel_gain, steer_gain = result.x

    print("\n" + "="*60)
    print(" FINAL CALIBRATED PLANT PARAMETERS")
    print("="*60)
    print(f"  SIM_MU         = {mu:.6f}")
    print(f"  SIM_IZ         = {Iz:.6f}")
    print(f"  SIM_C_SF       = {C_Sf:.6f}")
    print(f"  SIM_C_SR       = {C_Sr:.6f}")
    print(f"  ACCEL_GAIN_POS = {accel_gain:.6f}")
    print(f"  SIM_STEER_GAIN = {steer_gain:.6f}")
    print(f"  Final loss     = {result.fun:.6f}")
    print(f"  Converged: {result.success}  ({result.message})")

    # Save env script
    with open(ENV_SCRIPT, "w") as f:
        f.write("#!/usr/bin/env bash\n")
        f.write("# Calibrated 1:1 plant parameters — all 3 bags, full 100s DE optimisation\n")
        for k, v in [("SIM_MU", mu), ("SIM_MU_FRONT", mu), ("SIM_MU_REAR", mu),
                     ("SIM_IZ", Iz), ("SIM_C_SF", C_Sf), ("SIM_C_SR", C_Sr),
                     ("SIM_C_SF_HIGH_SLIP", C_Sf), ("SIM_C_SR_HIGH_SLIP", C_Sr),
                     ("ACCEL_GAIN_POS", accel_gain),
                     ("SIM_STEER_GAIN", steer_gain),
                     ("SIM_STEER_GAIN_HIGH_SLIP", steer_gain)]:
            f.write(f"export {k}={v:.6f}\n")
        f.write("export BODY_SAFETY_MARGIN=0.0\n")
    print(f"\nSaved env script → {ENV_SCRIPT}")

    # Final comparison plot
    print("\nGenerating final comparison plot...")
    sim_final = run_simulation(result.x, duration=100.0)
    sim_warm  = run_simulation(x0,       duration=100.0)
    wps = np.arange(N_WPS)

    fig, axes = plt.subplots(3, 1, figsize=(14, 13))
    for ax, sim_key, tgt, ylabel, title in [
        (axes[0], 'peak_ey', TARGET_PEAK_EY, 'Peak |e_y| per wp (m)', 'Peak Lateral Error'),
        (axes[1], 'mean_ey', TARGET_MEAN_EY, 'Mean |e_y| per wp (m)', 'Mean Lateral Error'),
        (axes[2], 'mean_vx', TARGET_VX,      'Mean vx per wp (m/s)', 'Speed Profile'),
    ]:
        ax.plot(wps, tgt,                    'k-',  lw=2.5, label='Real bags (all 3, consensus)')
        if sim_warm:
            ax.plot(wps, sim_warm[sim_key],  'r--', lw=1.5, label='Previous params')
        if sim_final:
            ax.plot(wps, sim_final[sim_key], 'g-',  lw=2.5, label='DE-optimised params')
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(title, fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.4)

    axes[2].set_xlabel('Track Waypoint Index', fontsize=11)
    plt.suptitle('1:1 Sim-to-Real Calibration — CPU Simulator (Differential Evolution)',
                 fontsize=13, y=1.01)
    plt.tight_layout()
    plot = os.path.join(OUT_DIR, "sim_to_real_1to1_alignment.png")
    plt.savefig(plot, dpi=150, bbox_inches='tight')
    print(f"Saved plot → {plot}")
    plt.close()

if __name__ == '__main__':
    sys.exit(main())
