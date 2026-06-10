#!/usr/bin/env python3
"""
reconstruct_raceline_from_bag.py
================================
Reconstructs the MPC-internal raceline from a state_replay.csv bag by
collecting all 20 horizon reference points per timestep, deduplicating,
sorting into a closed-loop ordered path, and writing a raceline CSV
compatible with test_sim_drive.

Also extracts the initial vehicle state so the sim can be spawned at the
exact same pose as the real car.

Usage:
    python3 tools/reconstruct_raceline_from_bag.py [bag_path] [out_dir]
"""

import os
import sys
import pandas as pd
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

QP_SCALE = 262144.0

# ── defaults ────────────────────────────────────────────────────────────────
REPO_ROOT = "/home/akselmo/Documents/GitHub/BachelorProject"
DEFAULT_BAG = os.path.join(REPO_ROOT, "tools/output/mpc_deep_diag_20260519_125104/state_replay.csv")
DEFAULT_OUT = os.path.join(REPO_ROOT, "tools/output")
STATIC_CSV  = os.path.join(REPO_ROOT, "MPC/trajectories/my_track_raceline.csv")

def reconstruct_raceline(bag_path, out_dir, label="bag"):
    print(f"\n{'='*60}")
    print(f" Reconstructing raceline from: {os.path.basename(bag_path)}")
    print(f"{'='*60}")

    df = pd.read_csv(bag_path)
    vx_raw = df['velocity_fp'].values / QP_SCALE

    # Only keep moving frames
    moving = vx_raw > 0.3
    df = df[moving].reset_index(drop=True)
    print(f"  Moving rows: {len(df)}  (duration ≈ {(df['stamp_ns'].values[-1]-df['stamp_ns'].values[0])/1e9:.1f}s)")

    # ── Check ref data is non-zero ──────────────────────────────────────────
    rx0 = df['ref_x_fp_0'].values / QP_SCALE
    if np.all(rx0 == 0):
        print("  ⚠ ref_x_fp_0 is all zeros — this bag has no reference data. Using static raceline.")
        return None, None

    # ── Collect all 20 horizon ref points ──────────────────────────────────
    pts_x, pts_y, pts_psi, pts_kappa, pts_vx = [], [], [], [], []
    pts_left, pts_right = [], []

    n_horizon = sum(1 for c in df.columns if c.startswith('ref_x_fp_'))
    print(f"  Horizon length in bag: {n_horizon}")

    for i in range(n_horizon):
        x = df[f'ref_x_fp_{i}'].values   / QP_SCALE
        y = df[f'ref_y_fp_{i}'].values   / QP_SCALE
        psi   = df[f'ref_psi_fp_{i}'].values  / QP_SCALE if f'ref_psi_fp_{i}' in df.columns else np.zeros(len(df))
        kappa = df[f'ref_kappa_fp_{i}'].values / QP_SCALE if f'ref_kappa_fp_{i}' in df.columns else np.zeros(len(df))
        vx    = df[f'ref_vx_fp_{i}'].values  / QP_SCALE  if f'ref_vx_fp_{i}' in df.columns else np.full(len(df), 4.0)
        left  = df[f'ref_left_bound_fp_{i}'].values  / QP_SCALE if f'ref_left_bound_fp_{i}' in df.columns else np.full(len(df), 1.0)
        right = df[f'ref_right_bound_fp_{i}'].values / QP_SCALE if f'ref_right_bound_fp_{i}' in df.columns else np.full(len(df), 1.0)

        pts_x.append(x); pts_y.append(y); pts_psi.append(psi)
        pts_kappa.append(kappa); pts_vx.append(vx)
        pts_left.append(left); pts_right.append(right)

    pts_x     = np.concatenate(pts_x)
    pts_y     = np.concatenate(pts_y)
    pts_psi   = np.concatenate(pts_psi)
    pts_kappa = np.concatenate(pts_kappa)
    pts_vx    = np.concatenate(pts_vx)
    pts_left  = np.concatenate(pts_left)
    pts_right = np.concatenate(pts_right)

    print(f"  Raw points: {len(pts_x)}")

    # ── Spatial deduplication: grid-snap to ~5cm resolution ────────────────
    GRID = 0.05  # metres
    grid_keys = (np.round(pts_x / GRID).astype(int),
                 np.round(pts_y / GRID).astype(int))
    unique_idx = {}
    for i, (gx, gy) in enumerate(zip(*grid_keys)):
        k = (gx, gy)
        if k not in unique_idx:
            unique_idx[k] = i
    sel = sorted(unique_idx.values())
    ux  = pts_x[sel]; uy  = pts_y[sel]
    upsi   = pts_psi[sel];   ukappa = pts_kappa[sel]
    uvx    = pts_vx[sel];    uleft  = pts_left[sel]; uright = pts_right[sel]
    print(f"  After dedup (5cm grid): {len(ux)} unique points")

    # ── Sort into a coherent closed path via greedy nearest-neighbour ───────
    static = pd.read_csv(STATIC_CSV, comment='#',
        names=['s_m','x_m','y_m','psi_rad','kappa_radpm','vx_mps','ax_mps2','d_left_m','d_right_m'])
    tree = cKDTree(np.stack([ux, uy], axis=1))

    static_xy = np.stack([static['x_m'].values, static['y_m'].values], axis=1)
    dists, inds = tree.query(static_xy, k=1)
    coverage = (dists < 0.15).mean()
    print(f"  Coverage vs static raceline (≤15cm): {coverage*100:.1f}%")

    if coverage > 0.5:
        print("  Using static template to order path...")
        ordered_idx = inds
    else:
        print("  ⚠ Low coverage! Using template-free greedy TSP solver to order closed-loop track...")
        # Greedy template-free path reconstruction
        N = len(ux)
        visited = np.zeros(N, dtype=bool)
        ordered_idx = []
        
        # Start at the first snapped point
        curr = 0
        ordered_idx.append(curr)
        visited[curr] = True
        
        for _ in range(N - 1):
            dx = ux - ux[curr]
            dy = uy - uy[curr]
            d = dx**2 + dy**2
            d[visited] = np.inf
            curr = int(np.argmin(d))
            ordered_idx.append(curr)
            visited[curr] = True
            
        ordered_idx = np.array(ordered_idx)

    ox  = ux[ordered_idx];   oy  = uy[ordered_idx]
    opsi   = upsi[ordered_idx];   okappa = ukappa[ordered_idx]
    ovx    = uvx[ordered_idx];    oleft  = uleft[ordered_idx]; oright = uright[ordered_idx]

    # Deduplicate consecutive identical points
    coords = np.stack([ox, oy], axis=1)
    keep = np.concatenate([[True], np.any(np.diff(coords, axis=0) != 0, axis=1)])
    ox, oy, opsi, okappa, ovx, oleft, oright = (
        ox[keep], oy[keep], opsi[keep], okappa[keep], ovx[keep], oleft[keep], oright[keep])
    print(f"  Ordered unique points: {len(ox)}")

    # ── Recompute arc-length ─────────────────────────────────────────────────
    ds = np.sqrt(np.diff(ox)**2 + np.diff(oy)**2)
    os_m = np.concatenate([[0.0], np.cumsum(ds)])
    track_len = os_m[-1]
    print(f"  Reconstructed track length: {track_len:.2f} m  (static: {static['s_m'].values[-1]:.2f} m)")

    # ── Smooth vx (median-filter to remove outliers) ─────────────────────────
    from scipy.ndimage import median_filter
    ovx_smooth = median_filter(ovx, size=11, mode='wrap')

    # ── Recompute kappa from psi/s (more reliable than raw kappa) ───────────
    dpsi = np.diff(np.unwrap(opsi))
    ds_safe = np.where(ds > 1e-6, ds, 1e-6)
    okappa_recomputed = np.concatenate([dpsi / ds_safe, [dpsi[-1] / ds_safe[-1]]])
    # Blend: trust kappa from bag for strong curves, geometry for straights
    okappa_final = okappa_recomputed

    # ax column = 0 (we don't have it from the bag)
    ax_arr = np.zeros(len(ox))

    # ── Write raceline CSV ────────────────────────────────────────────────────
    out_csv = os.path.join(out_dir, f"reconstructed_raceline_{label}.csv")
    with open(out_csv, 'w') as f:
        f.write("# Reconstructed from bag ref horizon points\n")
        f.write("# s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left_m,d_right_m\n")
        for i in range(len(ox)):
            f.write(f"{os_m[i]:.6f},{ox[i]:.6f},{oy[i]:.6f},{opsi[i]:.6f},"
                    f"{okappa_final[i]:.6f},{ovx_smooth[i]:.6f},{ax_arr[i]:.6f},"
                    f"{oleft[i]:.6f},{oright[i]:.6f}\n")
    print(f"  Saved: {out_csv}")

    # ── Extract initial state from bag (first moving frame) ─────────────────
    row0 = df.iloc[0]
    init = {
        'x':        row0['x_fp']        / QP_SCALE,
        'y':        row0['y_fp']        / QP_SCALE,
        'theta':    row0['theta_fp']    / QP_SCALE,
        'vx':       row0['velocity_fp'] / QP_SCALE,
        'vy':       row0['vy_fp']       / QP_SCALE,
        'omega':    row0['omega_fp']    / QP_SCALE,
        'e_y':      row0['e_y_fp']      / QP_SCALE,
        'e_psi':    row0['e_psi_fp']    / QP_SCALE,
        'stamp_ns': row0['stamp_ns'],
    }
    print(f"\n  Initial state:")
    for k, v in init.items():
        print(f"    {k:10s} = {v:.6f}")

    # ── Comparison plot ───────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor('#0d1117')

    # XY path comparison
    ax = axes[0, 0]; ax.set_facecolor('#161b22')
    ax.plot(static['x_m'], static['y_m'], 'w-', lw=1.5, alpha=0.5, label='Static raceline CSV')
    ax.scatter(ux, uy, s=1, c='#58a6ff', alpha=0.3, label=f'Bag ref pts ({len(ux)})')
    ax.plot(ox, oy, color='#ffa657', lw=2, label=f'Reconstructed ({len(ox)} pts)')
    # mark start
    ax.plot(ox[0], oy[0], 'go', ms=8, zorder=10, label='Reconstructed start')
    ax.set_title('XY Path Comparison', color='#e6edf3', fontweight='bold')
    ax.set_xlabel('x (m)', color='#c9d1d9'); ax.set_ylabel('y (m)', color='#c9d1d9')
    ax.tick_params(colors='#c9d1d9'); ax.grid(True, color='#30363d', alpha=0.6)
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', edgecolor='#30363d')
    ax.set_aspect('equal')
    ax.spines[:].set_color('#30363d')

    # vx profile
    ax = axes[0, 1]; ax.set_facecolor('#161b22')
    ax.plot(static['s_m'], static['vx_mps'], 'w-', lw=1.5, alpha=0.5, label='Static raceline vx')
    ax.plot(os_m, ovx_smooth, color='#ffa657', lw=2, label='Reconstructed vx (smoothed)')
    ax.plot(os_m, ovx, color='#3fb950', lw=1, alpha=0.5, label='Reconstructed vx (raw)')
    ax.set_title('Speed Profile', color='#e6edf3', fontweight='bold')
    ax.set_xlabel('Arc-length (m)', color='#c9d1d9'); ax.set_ylabel('vx (m/s)', color='#c9d1d9')
    ax.tick_params(colors='#c9d1d9'); ax.grid(True, color='#30363d', alpha=0.6)
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', edgecolor='#30363d')
    ax.spines[:].set_color('#30363d')

    # kappa profile
    ax = axes[1, 0]; ax.set_facecolor('#161b22')
    ax.plot(static['s_m'], static['kappa_radpm'], 'w-', lw=1.5, alpha=0.5, label='Static raceline κ')
    ax.plot(os_m, okappa_final, color='#ffa657', lw=1.5, label='Reconstructed κ')
    ax.set_title('Curvature Profile', color='#e6edf3', fontweight='bold')
    ax.set_xlabel('Arc-length (m)', color='#c9d1d9'); ax.set_ylabel('κ (rad/m)', color='#c9d1d9')
    ax.tick_params(colors='#c9d1d9'); ax.grid(True, color='#30363d', alpha=0.6)
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', edgecolor='#30363d')
    ax.spines[:].set_color('#30363d')

    # e_y time series from bag
    t_bag = (df['stamp_ns'].values - df['stamp_ns'].values[0]) / 1e9
    ey_bag = df['e_y_fp'].values / QP_SCALE
    ax = axes[1, 1]; ax.set_facecolor('#161b22')
    ax.plot(t_bag, ey_bag, color='#58a6ff', lw=1, alpha=0.8, label='Real car e_y')
    ax.axhline(0, color='#8b949e', lw=0.8, ls=':')
    ax.set_title('Real Car e_y Over Time', color='#e6edf3', fontweight='bold')
    ax.set_xlabel('Time (s)', color='#c9d1d9'); ax.set_ylabel('e_y (m)', color='#c9d1d9')
    ax.tick_params(colors='#c9d1d9'); ax.grid(True, color='#30363d', alpha=0.6)
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='#c9d1d9', edgecolor='#30363d')
    ax.spines[:].set_color('#30363d')

    fig.suptitle(f'Raceline Reconstruction from Bag: {label}',
                 color='#e6edf3', fontsize=13, fontweight='bold')
    plt.tight_layout()
    fig.patch.set_facecolor('#0d1117')
    plot_path = os.path.join(out_dir, f"reconstructed_raceline_{label}.png")
    fig.savefig(plot_path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Plot: {plot_path}")

    return out_csv, init


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG
    out_dir  = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT

    os.makedirs(out_dir, exist_ok=True)
    label = os.path.basename(os.path.dirname(bag_path))

    csv_path, init = reconstruct_raceline(bag_path, out_dir, label)
    if csv_path is None:
        print("No ref data — cannot reconstruct.")
        sys.exit(1)

    # Write init state script for sim
    init_sh = os.path.join(out_dir, f"bag_init_state_{label}.sh")
    with open(init_sh, 'w') as f:
        f.write("#!/usr/bin/env bash\n")
        f.write(f"# Initial vehicle state extracted from bag: {label}\n")
        # The sim supports START_OFFSET_X/Y/LAT + START_HEADING_OFFSET + START_SPEED
        # We set initial position via raceline matching: sim starts at raceline[0]
        # so we encode the offset from raceline[0]
        f.write(f"export START_SPEED={init['vx']:.6f}\n")
        f.write(f"export START_LAT_SPEED={init['vy']:.6f}\n")
        f.write(f"export START_YAW_RATE={init['omega']:.6f}\n")
        f.write(f"export START_STEER=0.0\n")
        # The bag e_y at t=0: use as lateral offset
        f.write(f"# Bag initial e_y={init['e_y']:.4f} e_psi={init['e_psi']:.4f}\n")
        f.write(f"export START_OFFSET_LAT={init['e_y']:.6f}\n")
        f.write(f"export START_HEADING_OFFSET={init['e_psi']:.6f}\n")
        f.write(f"export RACELINE_PATH={csv_path}\n")
    print(f"\n  Init state script: {init_sh}")
    print(f"\n  ✅ To run sim against this raceline:")
    print(f"     source {init_sh}")
    print(f"     source tools/output/optimized_real_car_env.sh")
    print(f"     SIM_TRACE_LOG=/tmp/matched_sim.csv SIM_DURATION=140 \\")
    print(f"     ./MPC/test_sim_drive")


if __name__ == '__main__':
    main()
