#!/usr/bin/env python3
import os
import sys
import pandas as pd
import numpy as np

# Physical bounds and nominal constants
LF = 0.166
LR = 0.160
LWB = LF + LR
MASS = 3.314
GRAVITY = 9.81
QP_SCALE = 262144.0

def load_data(csv_paths):
    segments = []
    for path in csv_paths:
        if not os.path.exists(path):
            continue
        df = pd.read_csv(path)
        t = (df['stamp_ns'] - df['stamp_ns'].iloc[0]).to_numpy() / 1e9
        vx = df['velocity_fp'].to_numpy() / QP_SCALE
        vy = df['vy_fp'].to_numpy() / QP_SCALE
        omega = df['omega_fp'].to_numpy() / QP_SCALE
        delta = df['steering_angle_fp'].to_numpy() / QP_SCALE
        
        dt = np.diff(t)
        dt[dt <= 0] = 0.005
        
        dvx = np.diff(vx) / dt
        
        seg = {
            'dt': dt,
            'vx': vx[:-1],
            'vy': vy[:-1],
            'omega': omega[:-1],
            'delta': delta[:-1],
            'dvx': dvx
        }
        
        valid_mask = (seg['vx'] > 0.5)
        for k in seg:
            seg[k] = seg[k][valid_mask]
            
        if len(seg['vx']) > 100:
            segments.append(seg)
    return segments

def compute_pacejka_forces(vx, vy, omega, delta, ax, C_af, C_ar, mu_f, mu_r, pacejka_c):
    vx_safe = np.maximum(vx, 0.5)
    alpha_f = delta - np.arctan2(vy + LF * omega, vx_safe)
    alpha_r = -np.arctan2(vy - LR * omega, vx_safe)
    
    Fx = MASS * ax
    Fzf = (MASS * GRAVITY * LR - Fx * 0.0703) / LWB
    Fzr = (MASS * GRAVITY * LF + Fx * 0.0703) / LWB
    
    B_f = C_af / pacejka_c
    B_r = C_ar / pacejka_c
    D_f = mu_f * Fzf
    D_r = mu_r * Fzr
    
    Fyf = D_f * np.sin(pacejka_c * np.arctan(B_f * alpha_f))
    Fyr = D_r * np.sin(pacejka_c * np.arctan(B_r * alpha_r))
    
    return Fyf, Fyr

def rk4_rollout(seg, start_idx, horizon, C_af, C_ar, Iz, mu_f, mu_r, pacejka_c):
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
            Fyf, Fyr = compute_pacejka_forces(v_x, v_y, w, d, ax, C_af, C_ar, mu_f, mu_r, pacejka_c)
            dvy = (Fyf * np.cos(d) + Fyr) / MASS - v_x * w
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

def main():
    csvs = [
        'tools/output/mpc_deep_diag_20260519_125104/state_replay.csv',
        'tools/output/mpc_deep_diag_20260519_164731/state_replay.csv',
        'tools/output/mpc_deep_diag_20260520_164020/state_replay.csv',
        'tools/output/deep_diag_run1/state_replay.csv'
    ]
    
    segments = load_data(csvs)
    
    # Nominals
    nom_C_af, nom_C_ar, nom_Iz, nom_mu_f, nom_mu_r, nom_pacejka_c = 4.47, 2.73, 0.035, 0.72, 0.72, 1.9
    
    # Stage 1 Optimized
    opt_C_af, opt_C_ar, opt_Iz, opt_mu_f, opt_mu_r, opt_pacejka_c = 4.3859, 5.3470, 0.080000, 0.4796, 0.4481, 3.0000
    
    nom_err_vy, nom_err_w = [], []
    opt_err_vy, opt_err_w = [], []
    
    horizon = 50
    stride = 100
    
    for seg in segments:
        n_pts = len(seg['vx'])
        for start_idx in range(0, n_pts - horizon, stride):
            res_nom = rk4_rollout(seg, start_idx, horizon, nom_C_af, nom_C_ar, nom_Iz, nom_mu_f, nom_mu_r, nom_pacejka_c)
            res_opt = rk4_rollout(seg, start_idx, horizon, opt_C_af, opt_C_ar, opt_Iz, opt_mu_f, opt_mu_r, opt_pacejka_c)
            
            if res_nom is None or res_opt is None:
                continue
                
            nom_err_vy.append(np.mean((res_nom[0] - res_nom[2])**2))
            nom_err_w.append(np.mean((res_nom[1] - res_nom[3])**2))
            
            opt_err_vy.append(np.mean((res_opt[0] - res_opt[2])**2))
            opt_err_w.append(np.mean((res_opt[1] - res_opt[3])**2))
            
    mse_nom_vy = np.mean(nom_err_vy)
    mse_nom_w = np.mean(nom_err_w)
    mse_opt_vy = np.mean(opt_err_vy)
    mse_opt_w = np.mean(opt_err_w)
    
    imp_vy = (1.0 - mse_opt_vy / mse_nom_vy) * 100
    imp_w = (1.0 - mse_opt_w / mse_nom_w) * 100
    
    print("\n==================================================")
    print(" STAGE 1 PACEJKA PLANT PARITY METRICS (4 BAGS)")
    print("==================================================")
    print(f"Lateral Velocity (vy) Rollout MSE:")
    print(f"  Nominal Plant: {mse_nom_vy:.6f} m^2/s^2")
    print(f"  Stage 1 Plant: {mse_opt_vy:.6f} m^2/s^2  ({imp_vy:+.1f}% Mismatch Reduction!)")
    print(f"Yaw Rate (omega) Rollout MSE:")
    print(f"  Nominal Plant: {mse_nom_w:.6f} rad^2/s^2")
    print(f"  Stage 1 Plant: {mse_opt_w:.6f} rad^2/s^2  ({imp_w:+.1f}% Mismatch Reduction!)")
    print("==================================================\n")

if __name__ == '__main__':
    main()
