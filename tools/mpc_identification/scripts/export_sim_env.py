#!/usr/bin/env python3
"""Export fitted identification parameters as environment variables for test_sim_drive.c."""
from __future__ import annotations
import argparse
from pathlib import Path
from plant_model import PlantParameters

MAPPING = {
    "steer_delay_s": "SIM_STEER_DELAY_S",  # informational: current C test uses discrete steps
    "accel_delay_s": "SIM_ACCEL_DELAY_S",  # informational: current C test uses discrete steps
    "steer_angle_max": "SIM_STEER_ANGLE_MAX", "steer_rate_max": "SIM_STEER_RATE_MAX",
    "steer_tau_s": "SIM_STEER_TAU", "steer_gain": "SIM_STEER_GAIN",
    "steer_gain_high_slip": "SIM_STEER_GAIN_HIGH_SLIP",
    "accel_tau_pos_s": "ACCEL_TAU_POS", "accel_tau_neg_s": "ACCEL_TAU_NEG",
    "accel_gain_pos": "ACCEL_GAIN_POS", "accel_gain_neg": "ACCEL_GAIN_NEG",
    "drag_c0": "DRAG_C0", "drag_c1": "DRAG_C1", "drag_c2": "DRAG_C2",
    "roll_resistance_n": "SIM_ROLL_RES_N", "mass_kg": "SIM_MASS", "iz_kgm2": "SIM_IZ",
    "lf_m": "SIM_LF", "lr_m": "SIM_LR", "h_cg_m": "SIM_H_CG",
    "mu_front": "SIM_MU_FRONT", "mu_rear": "SIM_MU_REAR", "csf": "SIM_C_SF", "csr": "SIM_C_SR",
    "csf_high_slip": "SIM_C_SF_HIGH_SLIP", "csr_high_slip": "SIM_C_SR_HIGH_SLIP",
    "pacejka_c_front": "SIM_PACEJKA_C_FRONT", "pacejka_c_rear": "SIM_PACEJKA_C_REAR",
    "slip_blend_start_front": "SIM_SLIP_BLEND_START_FRONT", "slip_blend_end_front": "SIM_SLIP_BLEND_END_FRONT",
    "slip_blend_start_rear": "SIM_SLIP_BLEND_START_REAR", "slip_blend_end_rear": "SIM_SLIP_BLEND_END_REAR",
    "rel_len_front_m": "SIM_REL_LEN_FRONT", "rel_len_rear_m": "SIM_REL_LEN_REAR",
    "combined_slip_gain": "SIM_COMBINED_SLIP_GAIN", "front_peak_drop": "SIM_FRONT_PEAK_DROP",
    "front_peak_drop_start": "SIM_FRONT_PEAK_DROP_START", "front_peak_drop_end": "SIM_FRONT_PEAK_DROP_END",
    "front_peak_drop_pow": "SIM_FRONT_PEAK_DROP_POW", "front_combined_gain": "SIM_FRONT_COMBINED_GAIN",
    "front_peak_floor": "SIM_FRONT_PEAK_FLOOR", "v_switch_mps": "SIM_V_SWITCH",
    "v_min_mps": "SIM_V_MIN", "v_max_mps": "SIM_V_MAX",
}

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('parameters',type=Path); parser.add_argument('--output',type=Path,required=True); parser.add_argument('--sim-dt',type=float,default=0.002)
    args=parser.parse_args(); p=PlantParameters.from_json(args.parameters); args.output.parent.mkdir(parents=True,exist_ok=True)
    lines=['#!/usr/bin/env bash', '# Generated from fitted parameters. Source this before ./test_sim_drive.', 'set -euo pipefail']
    for field,env in MAPPING.items():
        value=getattr(p,field)
        if field=='steer_delay_s': lines.append(f'export SIM_STEER_DELAY_STEPS=$(python3 -c "print(round({value:.12g}/{args.sim_dt:.12g}))")')
        elif field=='accel_delay_s': lines.append(f'export SIM_ACCEL_DELAY_STEPS=$(python3 -c "print(round({value:.12g}/{args.sim_dt:.12g}))")')
        else: lines.append(f'export {env}={value:.12g}')
    args.output.write_text('\n'.join(lines)+'\n'); print(f'Wrote {args.output}')
if __name__=='__main__': main()
