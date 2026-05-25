#!/usr/bin/env python3
"""
Global Vehicle Parameter Estimator using Differential Evolution
For F1TENTH MPC Simulator

This script executes a high-fidelity global search to estimate vehicle dynamics
parameters (mass, yaw inertia, front/rear frictions, and front/rear Pacejka shape factors)
from real-car bag replays. It uses parallel multi-core processing to evaluate
thousands of RK4 trajectory rollouts.
"""

import os
import sys
import time
import argparse
import pandas as pd
import numpy as np
from scipy.optimize import differential_evolution
import matplotlib.pyplot as plt

# Nominal physical constants
LF = 0.166
LR = 0.160
LWB = LF + LR
MASS = 3.314
IZ = 0.035
H_CG = 0.0703
GRAVITY = 9.81
DEFAULT_QP_SCALE = 262144.0

class GlobalEstimatorData:
    """Static container for preprocessed data segments to ensure picklability for parallel workers."""
    segments = []

def load_data(csv_paths, qp_scale):
    GlobalEstimatorData.segments = []
    print(f"Loading and preprocessing {len(csv_paths)} replay CSV files...")
    for path in csv_paths:
        if not os.path.exists(path):
            print(f"Warning: File not found: {path}", file=sys.stderr)
            continue
            
        df = pd.read_csv(path)
        
        # Scale fixed-point states
        t = (df['stamp_ns'] - df['stamp_ns'].iloc[0]).to_numpy() / 1e9
        x = df['x_fp'].to_numpy() / qp_scale
        y = df['y_fp'].to_numpy() / qp_scale
        psi = df['theta_fp'].to_numpy() / qp_scale
        vx = df['velocity_fp'].to_numpy() / qp_scale
        vy = df['vy_fp'].to_numpy() / qp_scale
        omega = df['omega_fp'].to_numpy() / qp_scale
        delta = df['steering_angle_fp'].to_numpy() / qp_scale
        
        dt = np.diff(t)
        dt[dt <= 0] = 0.005
        
        dvx = np.diff(vx) / dt
        dvy = np.diff(vy) / dt
        domega = np.diff(omega) / dt
        dpsi = np.diff(psi) / dt
        
        seg = {
            't': t[:-1],
            'dt': dt,
            'x': x[:-1],
            'y': y[:-1],
            'psi': psi[:-1],
            'vx': vx[:-1],
            'vy': vy[:-1],
            'omega': omega[:-1],
            'delta': delta[:-1],
            'dvx': dvx,
            'dvy': dvy,
            'domega': domega,
            'dpsi': dpsi
        }
        
        # Filter for high-excitement dynamic points (standstill excluded)
        valid_mask = (seg['vx'] > 0.5) & (np.abs(seg['dvy']) < 15.0) & (np.abs(seg['domega']) < 30.0)
        
        for k in seg:
            seg[k] = seg[k][valid_mask]
            
        if len(seg['t']) > 100:
            GlobalEstimatorData.segments.append(seg)
            print(f"  Loaded {len(seg['t'])} dynamic samples from: {os.path.basename(path)}")
        else:
            print(f"  Warning: Skipping {os.path.basename(path)} (too few dynamic samples).")

    if not GlobalEstimatorData.segments:
        raise ValueError("No valid dynamic data segments loaded!")


def compute_pacejka_forces(vx, vy, omega, delta, ax, C_af, C_ar, mu_f, mu_r, cf, cr, mass):
    # Slip angles
    vx_safe = np.maximum(vx, 0.5)
    alpha_f = delta - np.arctan2(vy + LF * omega, vx_safe)
    alpha_r = -np.arctan2(vy - LR * omega, vx_safe)
    
    # Longitudinal force
    Fx = mass * ax
    
    # Dynamic normal load transfer
    Fzf = (mass * GRAVITY * LR - Fx * H_CG) / LWB
    Fzr = (mass * GRAVITY * LF + Fx * H_CG) / LWB
    
    # Pacejka Formula
    B_f = C_af / cf
    B_r = C_ar / cr
    D_f = mu_f * Fzf
    D_r = mu_r * Fzr
    
    Fyf = D_f * np.sin(cf * np.arctan(B_f * alpha_f))
    Fyr = D_r * np.sin(cr * np.arctan(B_r * alpha_r))
    
    return Fyf, Fyr


def rk4_rollout(seg, start_idx, horizon, C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass):
    vx_meas = seg['vx'][start_idx : start_idx + horizon]
    vy_meas = seg['vy'][start_idx : start_idx + horizon]
    omega_meas = seg['omega'][start_idx : start_idx + horizon]
    delta = seg['delta'][start_idx : start_idx + horizon]
    dvx = seg['dvx'][start_idx : start_idx + horizon]
    dt = seg['dt'][start_idx : start_idx + horizon]
    
    h = len(vx_meas)
    if h < 2:
        return None
        
    vy_sim = np.zeros(h)
    omega_sim = np.zeros(h)
    
    vy_sim[0] = vy_meas[0]
    omega_sim[0] = omega_meas[0]
    
    for i in range(h - 1):
        t_dt = dt[i]
        
        def get_derivatives(v_x, v_y, w, d, ax):
            Fyf, Fyr = compute_pacejka_forces(v_x, v_y, w, d, ax, C_af, C_ar, mu_f, mu_r, cf, cr, mass)
            dvy = (Fyf * np.cos(d) + Fyr) / mass - v_x * w
            domega = (LF * Fyf * np.cos(d) - LR * Fyr) / Iz
            return dvy, domega
            
        k1_vy, k1_w = get_derivatives(vx_meas[i], vy_sim[i], omega_sim[i], delta[i], dvx[i])
        
        vy_half = vy_sim[i] + 0.5 * t_dt * k1_vy
        w_half = omega_sim[i] + 0.5 * t_dt * k1_w
        vx_half = 0.5 * (vx_meas[i] + vx_meas[i+1])
        delta_half = 0.5 * (delta[i] + delta[i+1])
        dvx_half = 0.5 * (dvx[i] + dvx[i+1])
        
        k2_vy, k2_w = get_derivatives(vx_half, vy_half, w_half, delta_half, dvx_half)
        
        vy_half2 = vy_sim[i] + 0.5 * t_dt * k2_vy
        w_half2 = omega_sim[i] + 0.5 * t_dt * k2_w
        
        k3_vy, k3_w = get_derivatives(vx_half, vy_half2, w_half2, delta_half, dvx_half)
        
        vy_end = vy_sim[i] + t_dt * k3_vy
        w_end = omega_sim[i] + t_dt * k3_w
        
        k4_vy, k4_w = get_derivatives(vx_meas[i+1], vy_end, w_end, delta[i+1], dvx[i+1])
        
        vy_sim[i+1] = vy_sim[i] + (t_dt / 6.0) * (k1_vy + 2.0*k2_vy + 2.0*k3_vy + k4_vy)
        omega_sim[i+1] = omega_sim[i] + (t_dt / 6.0) * (k1_w + 2.0*k2_w + 2.0*k3_w + k4_w)
        
    return vy_meas, omega_meas, vy_sim, omega_sim


def global_rollout_loss(opt_params):
    """
    Evaluates trajectory rollout error globally across all segments.
    opt_params: [C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass]
    """
    C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass = opt_params
    
    total_loss = 0.0
    total_evals = 0
    
    rollout_horizon = 50  # 250ms horizon is standard and highly dynamic
    stride = 100          # High-density evaluation
    
    for seg in GlobalEstimatorData.segments:
        n_pts = len(seg['vx'])
        for start_idx in range(0, n_pts - rollout_horizon, stride):
            res = rk4_rollout(seg, start_idx, rollout_horizon, C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass)
            if res is None:
                continue
                
            vy_meas, omega_meas, vy_sim, omega_sim = res
            
            vy_err = np.mean((vy_meas - vy_sim)**2)
            omega_err = np.mean((omega_meas - omega_sim)**2)
            
            # Loss = weighted tracking error
            total_loss += (vy_err + 0.1 * omega_err)
            total_evals += 1
            
    if total_evals == 0:
        return 9999.0
        
    return total_loss / total_evals


def run_global_optimization(max_iter=45, pop_size=15):
    print("\n--- Starting Global Parameter Estimation (Differential Evolution) ---")
    print(f"Parallel Worker Threads: Active (Multi-Core)")
    print(f"Horizon: 50 steps (250ms), Stride: 100 (High-Precision)")
    
    # Bounds for the 8 variables
    # [C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass]
    bounds = [
        (0.5, 12.0),     # C_af
        (0.5, 12.0),     # C_ar
        (0.01, 0.12),    # Iz
        (0.3, 1.2),      # mu_f
        (0.3, 1.2),      # mu_r
        (1.0, 3.0),      # cf (Pacejka shape factor front)
        (1.0, 3.0),      # cr (Pacejka shape factor rear)
        (2.8, 3.8)       # mass
    ]
    
    start_time = time.time()
    
    # Run Scipy's Global Differential Evolution Optimizer
    res = differential_evolution(
        global_rollout_loss,
        bounds=bounds,
        maxiter=max_iter,
        popsize=pop_size,
        tol=0.005,
        polish=True,          # Local refinement run at the end for exact decimal precision!
        workers=12,          # Run on at most 12 CPU cores in parallel!
        disp=True            # Display convergence updates
    )
    
    duration = time.time() - start_time
    print(f"\nOptimization Finished in {duration/60.0:.2f} minutes.")
    
    if res.success:
        C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass = res.x
        print("Global Optimization SUCCESSFUL!")
        
        # Save report
        report_path = 'tools/output/global_param_estimation_report.md'
        os.makedirs(os.path.dirname(report_path), exist_ok=True)
        
        avg_mu = 0.5 * (mu_f + mu_r)
        avg_c = 0.5 * (cf + cr)
        
        with open(report_path, 'w') as f:
            f.write("# High-Fidelity Global Parameter Calibration Report\n\n")
            f.write(f"Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write("## 1. Globally Optimized Parameters\n\n")
            f.write("| Parameter | Estimated Value | Nominal (Default) |\n")
            f.write("| :--- | :---: | :---: |\n")
            f.write(f"| **Vehicle Mass ($m$)** | **{mass:.4f} kg** | {MASS:.3f} kg |\n")
            f.write(f"| **Yaw Inertia ($I_z$)** | **{Iz:.6f} kg*m^2** | {IZ:.4f} kg*m^2 |\n")
            f.write(f"| **Front Friction ($\mu_f$)** | **{mu_f:.4f}** | 0.720 |\n")
            f.write(f"| **Rear Friction ($\mu_r$)** | **{mu_r:.4f}** | 0.720 |\n")
            f.write(f"| **Front Normalized Stiffness ($C_{{af}}$)** | **{C_af:.4f}** | 4.470 |\n")
            f.write(f"| **Rear Normalized Stiffness ($C_{{ar}}$)** | **{C_ar:.4f}** | 2.730 |\n")
            f.write(f"| **Front Pacejka Shape ($C_f$)** | **{cf:.4f}** | 1.9 |\n")
            f.write(f"| **Rear Pacejka Shape ($C_r$)** | **{cr:.4f}** | 1.9 |\n")
            
            f.write("\n\n## 2. Dynamic Integration Guide & Macro Definitions\n\n")
            f.write("### For `MPC/include/mpc_types.h` (MPC Solver):\n")
            f.write("```c\n")
            f.write(f"#define VP_MASS_KG                      {mass:.6f}f\n")
            f.write(f"#define VP_YAW_INERTIA_KGM2             {Iz:.6f}f\n")
            f.write(f"#define VP_FRICTION_COEFF               {avg_mu:.6f}f\n")
            f.write(f"#define VP_C_SHAPE                      {avg_c:.6f}f\n")
            f.write(f"#define VP_INV_C_SHAPE                  {1.0/avg_c:.6f}f\n")
            f.write(f"#define VP_FRONT_CORNERING_STIFFNESS    {C_af:.6f}f\n")
            f.write(f"#define VP_REAR_CORNERING_STIFFNESS     {C_ar:.6f}f\n")
            f.write("```\n\n")
            
            f.write("### For `MPC/test/test_sim_drive.c` (Simulator Defaults):\n")
            f.write("```c\n")
            f.write(f"const double sim_mu = {avg_mu:.6f};\n")
            f.write(f"const double sim_mu_front = {mu_f:.6f};\n")
            f.write(f"const double sim_mu_rear = {mu_r:.6f};\n")
            f.write(f"const double sim_mass = {mass:.6f};\n")
            f.write(f"const double sim_Iz = {Iz:.6f};\n")
            f.write(f"const double sim_C_Sf = {C_af:.6f};\n")
            f.write(f"const double sim_C_Sr = {C_ar:.6f};\n")
            f.write(f"const double sim_C_Sf_high_slip = {C_af:.6f};\n")
            f.write(f"const double sim_C_Sr_high_slip = {C_ar:.6f};\n")
            f.write(f"const double sim_pacejka_c_front = {cf:.6f};\n")
            f.write(f"const double sim_pacejka_c_rear = {cr:.6f};\n")
            f.write("```\n")
            
        print(f"Saved global report to: {report_path}")
        
        # Save dynamic comparison plots
        plot_comparison(res.x)
        
        return res.x
    else:
        print("Global Optimization FAILED!", res.message)
        return None


def plot_comparison(refined_params):
    C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass = refined_params
    seg = GlobalEstimatorData.segments[0]
    n_pts = len(seg['vx'])
    
    # Locate highest excitement window
    best_start = 0
    max_var = 0
    horizon = 100
    for i in range(0, n_pts - horizon, 50):
        var = np.var(seg['omega'][i:i+horizon])
        if var > max_var:
            max_var = var
            best_start = i
            
    res_nom = rk4_rollout(seg, best_start, horizon, 4.47, 2.73, 0.035, 0.72, 0.72, 1.9, 1.9, 3.314)
    res_opt = rk4_rollout(seg, best_start, horizon, C_af, C_ar, Iz, mu_f, mu_r, cf, cr, mass)
    
    if res_nom is None or res_opt is None:
        return
        
    t = np.arange(len(res_nom[0])) * 0.005
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    
    axes[0].plot(t, res_nom[0], 'k-', label='Actual Hardware Log', linewidth=2)
    axes[0].plot(t, res_nom[2], 'r--', label='Nominal Simulation', linewidth=1.5)
    axes[0].plot(t, res_opt[2], 'g-.', label='Globally Aligned Sim', linewidth=2)
    axes[0].set_ylabel('Lateral Velocity vy (m/s)', fontsize=12)
    axes[0].set_title('High-Precision Global Trajectory Alignment (Pacejka Tires & RK4)', fontsize=14)
    axes[0].grid(True)
    axes[0].legend(fontsize=10)
    
    axes[1].plot(t, res_nom[1], 'k-', label='Actual Hardware Log', linewidth=2)
    axes[1].plot(t, res_nom[3], 'r--', label='Nominal Simulation', linewidth=1.5)
    axes[1].plot(t, res_opt[3], 'g-.', label='Globally Aligned Sim', linewidth=2)
    axes[1].set_ylabel('Yaw Rate omega (rad/s)', fontsize=12)
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].grid(True)
    axes[1].legend(fontsize=10)
    
    plt.tight_layout()
    plot_path = 'tools/output/global_param_alignment.png'
    plt.savefig(plot_path, dpi=150)
    print(f"Saved trajectory alignment plot to: {plot_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Estimate physical vehicle parameters globally using Differential Evolution.')
    parser.add_argument('--csv', nargs='+', required=True, help='Paths to state_replay.csv logs')
    parser.add_argument('--qp-scale', type=float, default=DEFAULT_QP_SCALE, help='QP fixed-point scaling factor')
    parser.add_argument('--max-iter', type=int, default=50, help='Max Differential Evolution iterations')
    parser.add_argument('--pop-size', type=int, default=15, help='Differential Evolution population size')
    args = parser.parse_args()
    
    load_data(args.csv, args.qp_scale)
    run_global_optimization(max_iter=args.max_iter, pop_size=args.pop_size)
    return 0

if __name__ == '__main__':
    sys.exit(main())
