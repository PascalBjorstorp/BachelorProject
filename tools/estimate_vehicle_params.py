#!/usr/bin/env python3
"""
Vehicle Parameter Estimator for F1TENTH MPC Simulator

This script reads state logs (state_replay.csv) from real-car bag replays,
reconstructs the dynamic states and inputs, and performs optimization to
estimate physical parameters (stiffnesses, mass, yaw inertia, friction, etc.)
so that the simulator closely mimics actual car behavior.
"""

import os
import sys
import argparse
import pandas as pd
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# Default scaling factor for Q14.18 fixed point
DEFAULT_QP_SCALE = 262144.0

# Vehicle geometry (nominal values from mpc_fpga_constants.h)
LF = 0.166  # CG to front axle (m)
LR = 0.160  # CG to rear axle (m)
LWB = LF + LR
MASS = 3.314  # Nominal mass (kg)
IZ = 0.035  # Nominal yaw inertia (kg*m^2)
H_CG = 0.0703  # CG height (m)
GRAVITY = 9.81
PACEJKA_C = 1.9  # Nominal shape factor

class VehicleParameterEstimator:
    def __init__(self, csv_paths, qp_scale=DEFAULT_QP_SCALE, mass=MASS, lf=LF, lr=LR, h_cg=H_CG):
        self.csv_paths = csv_paths if isinstance(csv_paths, list) else [csv_paths]
        self.qp_scale = qp_scale
        self.mass = mass
        self.lf = lf
        self.lr = lr
        self.h_cg = h_cg
        self.lwb = lf + lr
        self.g = GRAVITY
        
        self.data_segments = []
        self.load_and_preprocess_data()
        
    def load_and_preprocess_data(self):
        print(f"Loading and preprocessing {len(self.csv_paths)} replay CSV files...")
        for path in self.csv_paths:
            if not os.path.exists(path):
                print(f"Warning: File not found: {path}", file=sys.stderr)
                continue
                
            df = pd.read_csv(path)
            
            # Extract and scale dynamic states
            t = (df['stamp_ns'] - df['stamp_ns'].iloc[0]).to_numpy() / 1e9  # seconds
            x = df['x_fp'].to_numpy() / self.qp_scale
            y = df['y_fp'].to_numpy() / self.qp_scale
            psi = df['theta_fp'].to_numpy() / self.qp_scale
            vx = df['velocity_fp'].to_numpy() / self.qp_scale
            vy = df['vy_fp'].to_numpy() / self.qp_scale
            omega = df['omega_fp'].to_numpy() / self.qp_scale
            delta = df['steering_angle_fp'].to_numpy() / self.qp_scale
            
            # Compute time steps
            dt = np.diff(t)
            # Handle zero or negative dt issues if any
            dt[dt <= 0] = 0.005
            
            # Compute numerical derivatives using central difference or forward difference
            dvx = np.diff(vx) / dt
            dvy = np.diff(vy) / dt
            domega = np.diff(omega) / dt
            dpsi = np.diff(psi) / dt
            
            # Align lengths (truncate last element)
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
            
            # Filter data to active dynamics range:
            # - vx must be above standstill threshold (0.5 m/s) to avoid singular slip angles
            # - exclude points with extremely high noise or outlier jumps
            valid_mask = (seg['vx'] > 0.5) & (np.abs(seg['dvy']) < 15.0) & (np.abs(seg['domega']) < 30.0)
            
            # Keep only segments with valid masks
            for k in seg:
                seg[k] = seg[k][valid_mask]
                
            if len(seg['t']) > 100:
                self.data_segments.append(seg)
                print(f"  Loaded Segment from {os.path.basename(path)}: {len(seg['t'])} valid dynamic points.")
            else:
                print(f"  Warning: Segment from {os.path.basename(path)} has too few dynamic points ({len(seg['t'])}). Skipping.")
                
        if not self.data_segments:
            raise ValueError("No valid data segments loaded! Check the input CSVs.")
            
    def compute_linear_forces(self, vx, vy, omega, delta, ax, C_af, C_ar, mu):
        # Calculate slip angles
        vx_safe = np.maximum(vx, 0.5)
        alpha_f = delta - np.arctan2(vy + self.lf * omega, vx_safe)
        alpha_r = -np.arctan2(vy - self.lr * omega, vx_safe)
        
        # Longitudinal force (approx from measured acceleration)
        Fx = self.mass * ax
        
        # Quasi-static normal load transfer
        Fzf = (self.mass * self.g * self.lr - Fx * self.h_cg) / self.lwb
        Fzr = (self.mass * self.g * self.lf + Fx * self.h_cg) / self.lwb
        
        # Lateral forces (linear tire with normalized stiffness)
        # Note: C_af, C_ar here are the normalized cornering stiffnesses (dimensionless).
        Fyf = mu * C_af * alpha_f * Fzf
        Fyr = mu * C_ar * alpha_r * Fzr
        
        return Fyf, Fyr
        
    def compute_pacejka_forces(self, vx, vy, omega, delta, ax, C_af, C_ar, mu_f, mu_r, pacejka_c=PACEJKA_C):
        vx_safe = np.maximum(vx, 0.5)
        alpha_f = delta - np.arctan2(vy + self.lf * omega, vx_safe)
        alpha_r = -np.arctan2(vy - self.lr * omega, vx_safe)
        
        Fx = self.mass * ax
        Fzf = (self.mass * self.g * self.lr - Fx * self.h_cg) / self.lwb
        Fzr = (self.mass * self.g * self.lf + Fx * self.h_cg) / self.lwb
        
        B_f = C_af / pacejka_c
        B_r = C_ar / pacejka_c
        D_f = mu_f * Fzf
        D_r = mu_r * Fzr
        
        Fyf = D_f * np.sin(pacejka_c * np.arctan(B_f * alpha_f))
        Fyr = D_r * np.sin(pacejka_c * np.arctan(B_r * alpha_r))
        
        return Fyf, Fyr

    def derivative_loss_linear(self, params):
        # params: [C_af, C_ar, Iz, mu]
        C_af, C_ar, Iz, mu = params
        
        total_loss = 0.0
        total_pts = 0
        
        for seg in self.data_segments:
            vx = seg['vx']
            vy = seg['vy']
            omega = seg['omega']
            delta = seg['delta']
            dvx = seg['dvx']
            dvy_meas = seg['dvy']
            domega_meas = seg['domega']
            
            Fyf, Fyr = self.compute_linear_forces(vx, vy, omega, delta, dvx, C_af, C_ar, mu)
            
            # Predict derivatives
            dvy_pred = (Fyf * np.cos(delta) + Fyr) / self.mass - vx * omega
            domega_pred = (self.lf * Fyf * np.cos(delta) - self.lr * Fyr) / Iz
            
            # Loss is MSE of derivatives
            vy_err = np.mean((dvy_meas - dvy_pred)**2)
            omega_err = np.mean((domega_meas - domega_pred)**2)
            
            # Weighting: 1.0 for lateral velocity dot, 0.1 for yaw rate dot (scaling differences)
            total_loss += (vy_err + 0.1 * omega_err) * len(vx)
            total_pts += len(vx)
            
        return total_loss / total_pts

    def derivative_loss_pacejka(self, params):
        # params: [C_af, C_ar, Iz, mu_f, mu_r, pacejka_c]
        C_af, C_ar, Iz, mu_f, mu_r, pacejka_c = params
        
        total_loss = 0.0
        total_pts = 0
        
        for seg in self.data_segments:
            vx = seg['vx']
            vy = seg['vy']
            omega = seg['omega']
            delta = seg['delta']
            dvx = seg['dvx']
            dvy_meas = seg['dvy']
            domega_meas = seg['domega']
            
            Fyf, Fyr = self.compute_pacejka_forces(vx, vy, omega, delta, dvx, C_af, C_ar, mu_f, mu_r, pacejka_c)
            
            # Predict derivatives
            dvy_pred = (Fyf * np.cos(delta) + Fyr) / self.mass - vx * omega
            domega_pred = (self.lf * Fyf * np.cos(delta) - self.lr * Fyr) / Iz
            
            vy_err = np.mean((dvy_meas - dvy_pred)**2)
            omega_err = np.mean((domega_meas - domega_pred)**2)
            
            total_loss += (vy_err + 0.1 * omega_err) * len(vx)
            total_pts += len(vx)
            
        return total_loss / total_pts

    def run_stage_1_fit(self, model_type='linear'):
        print(f"\n--- Stage 1: Fitting State Derivatives (Model: {model_type.upper()}) ---")
        if model_type == 'linear':
            # Initial guess: [C_af, C_ar, Iz, mu]
            # Nominal normalized stiffnesses: C_af=4.47, C_ar=2.73, Iz=0.035, mu=0.72
            x0 = [4.47, 2.73, 0.035, 0.72]
            bounds = [(0.5, 15.0), (0.5, 15.0), (0.01, 0.08), (0.3, 1.2)]
            
            res = minimize(self.derivative_loss_linear, x0, bounds=bounds, method='L-BFGS-B')
            if res.success:
                C_af, C_ar, Iz, mu = res.x
                print("Stage 1 Fit Successful:")
                print(f"  Front normalized stiffness C_af: {C_af:.4f} (nominal: 4.4700)")
                print(f"  Rear normalized stiffness C_ar : {C_ar:.4f} (nominal: 2.7300)")
                print(f"  Yaw inertia Iz                 : {Iz:.6f} kg*m^2 (nominal: 0.0350)")
                print(f"  Friction coefficient mu        : {mu:.4f} (nominal: 0.7200)")
                return {'C_af': C_af, 'C_ar': C_ar, 'Iz': Iz, 'mu': mu, 'model': 'linear'}
            else:
                print("Stage 1 Fit Failed!", res.message)
                return None
        else:
            # Initial guess: [C_af, C_ar, Iz, mu_f, mu_r, pacejka_c]
            x0 = [4.47, 2.73, 0.035, 0.72, 0.72, 1.9]
            bounds = [(0.5, 15.0), (0.5, 15.0), (0.01, 0.08), (0.3, 1.2), (0.3, 1.2), (1.0, 3.0)]
            
            res = minimize(self.derivative_loss_pacejka, x0, bounds=bounds, method='L-BFGS-B')
            if res.success:
                C_af, C_ar, Iz, mu_f, mu_r, pacejka_c = res.x
                print("Stage 1 Fit Successful:")
                print(f"  Front normalized stiffness C_af: {C_af:.4f} (nominal: 4.4700)")
                print(f"  Rear normalized stiffness C_ar : {C_ar:.4f} (nominal: 2.7300)")
                print(f"  Yaw inertia Iz                 : {Iz:.6f} kg*m^2 (nominal: 0.0350)")
                print(f"  Front friction mu_f            : {mu_f:.4f} (nominal: 0.7200)")
                print(f"  Rear friction mu_r             : {mu_r:.4f} (nominal: 0.7200)")
                print(f"  Pacejka shape factor C         : {pacejka_c:.4f} (nominal: 1.9000)")
                return {'C_af': C_af, 'C_ar': C_ar, 'Iz': Iz, 'mu_f': mu_f, 'mu_r': mu_r, 'pacejka_c': pacejka_c, 'model': 'pacejka'}
            else:
                print("Stage 1 Fit Failed!", res.message)
                return None

    def rollout_simulation(self, seg, start_idx, horizon, params, model_type='linear'):
        """Simulate RK4 rollout over a short horizon starting from actual state."""
        # Extract inputs and true trajectory
        vx_meas = seg['vx'][start_idx : start_idx + horizon]
        vy_meas = seg['vy'][start_idx : start_idx + horizon]
        omega_meas = seg['omega'][start_idx : start_idx + horizon]
        delta = seg['delta'][start_idx : start_idx + horizon]
        dvx = seg['dvx'][start_idx : start_idx + horizon]
        dt = seg['dt'][start_idx : start_idx + horizon]
        
        h = len(vx_meas)
        if h < 2:
            return None
            
        # Simulated states: [vy, omega] (vx is driven directly by measured vx / dvx to stay aligned)
        vy_sim = np.zeros(h)
        omega_sim = np.zeros(h)
        
        # Initialize
        vy_sim[0] = vy_meas[0]
        omega_sim[0] = omega_meas[0]
        
        # RK4 integration
        for i in range(h - 1):
            t_dt = dt[i]
            
            # State vector
            def get_derivatives(v_x, v_y, w, d, ax):
                if model_type == 'linear':
                    C_af, C_ar, Iz, mu = params['C_af'], params['C_ar'], params['Iz'], params['mu']
                    Fyf, Fyr = self.compute_linear_forces(v_x, v_y, w, d, ax, C_af, C_ar, mu)
                else:
                    C_af, C_ar, Iz, mu_f, mu_r, pacejka_c = (
                        params['C_af'], params['C_ar'], params['Iz'], 
                        params['mu_f'], params['mu_r'], params['pacejka_c']
                    )
                    Fyf, Fyr = self.compute_pacejka_forces(v_x, v_y, w, d, ax, C_af, C_ar, mu_f, mu_r, pacejka_c)
                    
                dvy = (Fyf * np.cos(d) + Fyr) / self.mass - v_x * w
                domega = (self.lf * Fyf * np.cos(d) - self.lr * Fyr) / Iz
                return dvy, domega
                
            # RK4 Steps
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
            
        return {
            'vy_meas': vy_meas,
            'omega_meas': omega_meas,
            'vy_sim': vy_sim,
            'omega_sim': omega_sim
        }

    def rollout_loss(self, opt_params, initial_params, model_type='linear', rollout_horizon=40, step_size=200):
        # Unpack parameters
        params = {}
        if model_type == 'linear':
            params['C_af'], params['C_ar'], params['Iz'], params['mu'] = opt_params
        else:
            params['C_af'], params['C_ar'], params['Iz'], params['mu_f'], params['mu_r'], params['pacejka_c'] = opt_params
            
        total_loss = 0.0
        total_evals = 0
        
        for seg in self.data_segments:
            n_pts = len(seg['vx'])
            # Roll out from multiple starting points along the segment (downsampled with large step_size)
            for start_idx in range(0, n_pts - rollout_horizon, step_size):
                res = self.rollout_simulation(seg, start_idx, rollout_horizon, params, model_type)
                if res is None:
                    continue
                    
                vy_err = np.mean((res['vy_meas'] - res['vy_sim'])**2)
                omega_err = np.mean((res['omega_meas'] - res['omega_sim'])**2)
                
                total_loss += (vy_err + 0.1 * omega_err)
                total_evals += 1
                
        if total_evals == 0:
            return 999.0
        return total_loss / total_evals

    def run_stage_2_fit(self, stage1_params, rollout_horizon=40):
        print(f"\n--- Stage 2: Fitting Trajectory Rollouts (Horizon: {rollout_horizon} steps) ---")
        model_type = stage1_params['model']
        
        # Stride of 250 keeps evaluation fast but rich
        stride = 250
        
        if model_type == 'linear':
            x0 = [stage1_params['C_af'], stage1_params['C_ar'], stage1_params['Iz'], stage1_params['mu']]
            bounds = [(0.5, 15.0), (0.5, 15.0), (0.01, 0.08), (0.3, 1.2)]
            
            res = minimize(
                self.rollout_loss, x0, args=(stage1_params, 'linear', rollout_horizon, stride),
                bounds=bounds, method='L-BFGS-B', options={'maxiter': 50}
            )
            if res.success:
                C_af, C_ar, Iz, mu = res.x
                print("Stage 2 Rollout Fit Successful:")
                print(f"  Refined C_af: {C_af:.4f} (Stage 1: {stage1_params['C_af']:.4f})")
                print(f"  Refined C_ar: {C_ar:.4f} (Stage 1: {stage1_params['C_ar']:.4f})")
                print(f"  Refined Iz  : {Iz:.6f} kg*m^2 (Stage 1: {stage1_params['Iz']:.6f})")
                print(f"  Refined mu  : {mu:.4f} (Stage 1: {stage1_params['mu']:.4f})")
                return {'C_af': C_af, 'C_ar': C_ar, 'Iz': Iz, 'mu': mu, 'model': 'linear'}
            else:
                print("Stage 2 Fit Failed! Falling back to Stage 1 parameters.", res.message)
                return stage1_params
        else:
            x0 = [
                stage1_params['C_af'], stage1_params['C_ar'], stage1_params['Iz'], 
                stage1_params['mu_f'], stage1_params['mu_r'], stage1_params['pacejka_c']
            ]
            bounds = [(0.5, 15.0), (0.5, 15.0), (0.01, 0.08), (0.3, 1.2), (0.3, 1.2), (1.0, 3.0)]
            
            res = minimize(
                self.rollout_loss, x0, args=(stage1_params, 'pacejka', rollout_horizon, stride),
                bounds=bounds, method='L-BFGS-B', options={'maxiter': 50}
            )
            if res.success:
                C_af, C_ar, Iz, mu_f, mu_r, pacejka_c = res.x
                print("Stage 2 Rollout Fit Successful:")
                print(f"  Refined C_af     : {C_af:.4f} (Stage 1: {stage1_params['C_af']:.4f})")
                print(f"  Refined C_ar     : {C_ar:.4f} (Stage 1: {stage1_params['C_ar']:.4f})")
                print(f"  Refined Iz       : {Iz:.6f} kg*m^2 (Stage 1: {stage1_params['Iz']:.6f})")
                print(f"  Refined mu_f     : {mu_f:.4f} (Stage 1: {stage1_params['mu_f']:.4f})")
                print(f"  Refined mu_r     : {mu_r:.4f} (Stage 1: {stage1_params['mu_r']:.4f})")
                print(f"  Refined pacejka_c: {pacejka_c:.4f} (Stage 1: {stage1_params['pacejka_c']:.4f})")
                return {
                    'C_af': C_af, 'C_ar': C_ar, 'Iz': Iz, 
                    'mu_f': mu_f, 'mu_r': mu_r, 'pacejka_c': pacejka_c, 'model': 'pacejka'
                }
            else:
                print("Stage 2 Fit Failed! Falling back to Stage 1 parameters.", res.message)
                return stage1_params

    def validate_and_compare(self, refined_params):
        print("\n--- Validation & Performance Comparison ---")
        # Nominal simulator parameters (normalized C_af=4.47, C_ar=2.73)
        nominal_linear = {'C_af': 4.47, 'C_ar': 2.73, 'Iz': 0.035, 'mu': 0.72}
        nominal_pacejka = {
            'C_af': 4.47, 'C_ar': 2.73, 'Iz': 0.035, 
            'mu_f': 0.72, 'mu_r': 0.72, 'pacejka_c': 1.9
        }
        
        model_type = refined_params['model']
        nom_params = nominal_linear if model_type == 'linear' else nominal_pacejka
        
        nom_errors_vy = []
        nom_errors_w = []
        opt_errors_vy = []
        opt_errors_w = []
        
        horizon = 50
        step_size = 100
        
        for seg in self.data_segments:
            n_pts = len(seg['vx'])
            for start_idx in range(0, n_pts - horizon, step_size):
                # Rollout with nominal
                res_nom = self.rollout_simulation(seg, start_idx, horizon, nom_params, model_type)
                # Rollout with optimized
                res_opt = self.rollout_simulation(seg, start_idx, horizon, refined_params, model_type)
                
                if res_nom is None or res_opt is None:
                    continue
                    
                nom_errors_vy.append(np.mean((res_nom['vy_meas'] - res_nom['vy_sim'])**2))
                nom_errors_w.append(np.mean((res_nom['omega_meas'] - res_nom['omega_sim'])**2))
                
                opt_errors_vy.append(np.mean((res_opt['vy_meas'] - res_opt['vy_sim'])**2))
                opt_errors_w.append(np.mean((res_opt['omega_meas'] - res_opt['omega_sim'])**2))
                
        mse_nom_vy = np.mean(nom_errors_vy)
        mse_nom_w = np.mean(nom_errors_w)
        mse_opt_vy = np.mean(opt_errors_vy)
        mse_opt_w = np.mean(opt_errors_w)
        
        improvement_vy = (1.0 - mse_opt_vy / mse_nom_vy) * 100
        improvement_w = (1.0 - mse_opt_w / mse_nom_w) * 100
        
        print("Mean Trajectory Rollout Error (50 steps / 0.25s horizon):")
        print(f"  Lateral Velocity (vy) MSE:")
        print(f"    Nominal  : {mse_nom_vy:.6f} m^2/s^2")
        print(f"    Optimized: {mse_opt_vy:.6f} m^2/s^2  ({improvement_vy:+.1f}% improvement!)")
        print(f"  Yaw Rate (omega) MSE:")
        print(f"    Nominal  : {mse_nom_w:.6f} rad^2/s^2")
        print(f"    Optimized: {mse_opt_w:.6f} rad^2/s^2  ({improvement_w:+.1f}% improvement!)")
        
        # Save a comparison plot
        self.plot_representative_rollout(refined_params, nom_params, model_type)

    def plot_representative_rollout(self, opt_params, nom_params, model_type):
        # Pick the longest segment and a point of high excitement
        seg = self.data_segments[0]
        n_pts = len(seg['vx'])
        
        # Find a subsegment with significant yaw rate variance
        best_start = 0
        max_var = 0
        horizon = 100
        for i in range(0, n_pts - horizon, 50):
            var = np.var(seg['omega'][i:i+horizon])
            if var > max_var:
                max_var = var
                best_start = i
                
        res_nom = self.rollout_simulation(seg, best_start, horizon, nom_params, model_type)
        res_opt = self.rollout_simulation(seg, best_start, horizon, opt_params, model_type)
        
        if res_nom is None or res_opt is None:
            return
            
        t = np.arange(len(res_nom['vy_meas'])) * 0.005  # 5ms sim steps
        
        fig, axes = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot lateral velocity
        axes[0].plot(t, res_nom['vy_meas'], 'k-', label='Actual Real Car', linewidth=2)
        axes[0].plot(t, res_nom['vy_sim'], 'r--', label='Nominal Simulation', linewidth=1.5)
        axes[0].plot(t, res_opt['vy_sim'], 'g-.', label='Optimized Simulation', linewidth=2)
        axes[0].set_ylabel('Lateral Velocity vy (m/s)', fontsize=12)
        axes[0].set_title('Vehicle Dynamics Parameter Alignment: Real vs Simulated Trajectory Rollout', fontsize=14)
        axes[0].grid(True)
        axes[0].legend(fontsize=10)
        
        # Plot yaw rate
        axes[1].plot(t, res_nom['omega_meas'], 'k-', label='Actual Real Car', linewidth=2)
        axes[1].plot(t, res_nom['omega_sim'], 'r--', label='Nominal Simulation', linewidth=1.5)
        axes[1].plot(t, res_opt['omega_sim'], 'g-.', label='Optimized Simulation', linewidth=2)
        axes[1].set_ylabel('Yaw Rate omega (rad/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True)
        axes[1].legend(fontsize=10)
        
        plt.tight_layout()
        plot_path = 'tools/output/vehicle_param_alignment.png'
        os.makedirs(os.path.dirname(plot_path), exist_ok=True)
        plt.savefig(plot_path, dpi=150)
        print(f"\nSaved representative trajectory rollout plot to: {plot_path}")
        plt.close()

def main():
    parser = argparse.ArgumentParser(description='Estimate dynamic vehicle parameters from real car state replays.')
    parser.add_argument('--csv', nargs='+', required=True, help='Paths to state_replay.csv logs')
    parser.add_argument('--qp-scale', type=float, default=DEFAULT_QP_SCALE, help='QP fixed-point scaling factor (default: 262144.0)')
    parser.add_argument('--mass', type=float, default=MASS, help='Nominal car mass in kg (default: 3.314)')
    parser.add_argument('--model', choices=['linear', 'pacejka'], default='linear', help='Tire model to use for estimation (default: linear)')
    parser.add_argument('--horizon', type=int, default=40, help='Rollout horizon in steps for Stage 2 fitting (default: 40)')
    args = parser.parse_args()

    estimator = VehicleParameterEstimator(
        csv_paths=args.csv,
        qp_scale=args.qp_scale,
        mass=args.mass
    )
    
    stage1 = estimator.run_stage_1_fit(model_type=args.model)
    if stage1 is None:
        print("Parameter estimation failed in Stage 1.")
        return 1
        
    refined = estimator.run_stage_2_fit(stage1, rollout_horizon=args.horizon)
    estimator.validate_and_compare(refined)
    
    # Print the recommended macros for mpc_types.h and test_sim_drive.c
    print("\n" + "="*60)
    print(" RECOMMENDED VEHICLE PARAMETERS FOR SYSTEM INTEGRATION")
    print("="*60)
    print("\n1. For MPC Header (MPC/include/mpc_types.h):")
    print("   Note: The MPC equations in vehicle_model.c use normalized stiffnesses directly.")
    print("   To achieve parity, define the normalized values in your constants/types header:")
    print(f"   #define VP_FRONT_CORNERING_STIFFNESS    {refined['C_af']:.6f}f")
    print(f"   #define VP_REAR_CORNERING_STIFFNESS     {refined['C_ar']:.6f}f")
    print(f"   #define VP_YAW_INERTIA_KGM2             {refined['Iz']:.6f}f")
    if args.model == 'linear':
        print(f"   #define VP_FRICTION_COEFF               {refined['mu']:.6f}f")
    else:
        avg_mu = 0.5 * (refined['mu_f'] + refined['mu_r'])
        print(f"   #define VP_FRICTION_COEFF               {avg_mu:.6f}f")
        print(f"   #define VP_C_SHAPE                      {refined['pacejka_c']:.6f}f")
        print(f"   #define VP_INV_C_SHAPE                  {1.0/refined['pacejka_c']:.6f}f")

    print("\n2. For Simulator (MPC/test/test_sim_drive.c / Environment variables):")
    print(f"   export SIM_C_SF={refined['C_af']:.6f}")
    print(f"   export SIM_C_SR={refined['C_ar']:.6f}")
    print(f"   export SIM_IZ={refined['Iz']:.6f}")
    if args.model == 'linear':
        print(f"   export SIM_MU={refined['mu']:.6f}")
        print(f"   export SIM_MU_FRONT={refined['mu']:.6f}")
        print(f"   export SIM_MU_REAR={refined['mu']:.6f}")
    else:
        print(f"   export SIM_MU_FRONT={refined['mu_f']:.6f}")
        print(f"   export SIM_MU_REAR={refined['mu_r']:.6f}")
        print(f"   export SIM_PACEJKA_C_FRONT={refined['pacejka_c']:.6f}")
        print(f"   export SIM_PACEJKA_C_REAR={refined['pacejka_c']:.6f}")
    print("="*60 + "\n")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
