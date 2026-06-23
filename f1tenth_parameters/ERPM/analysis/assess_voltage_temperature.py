#!/usr/bin/env python3
"""Report voltage/thermal context and speed-map residual sensitivity."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np
import pandas as pd
from common import analysis_dir, dump_yaml


def main() -> int:
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('session',type=Path); a=p.parse_args(); session=a.session.resolve(); out=analysis_dir(session)
    train=pd.read_parquet(out/'erpm_map_training_trials.parquet'); hold=pd.read_parquet(out/'erpm_map_holdout_trials.parquet')
    all_=pd.concat([train.assign(split='training'),hold.assign(split='holdout')],ignore_index=True)
    finite=all_.replace([np.inf,-np.inf],np.nan).dropna(subset=['battery_voltage_v','residual_mps'])
    voltage_span=float(finite.battery_voltage_v.max()-finite.battery_voltage_v.min()) if len(finite) else math.nan
    if len(finite)>=4 and finite.battery_voltage_v.std()>1e-6:
        slope,intercept=np.polyfit(finite.battery_voltage_v.to_numpy(float),finite.residual_mps.to_numpy(float),1)
        corr=float(np.corrcoef(finite.battery_voltage_v,finite.residual_mps)[0,1])
    else: slope=intercept=corr=math.nan
    min_span=float((__import__('yaml').safe_load((session/'calibration_config_snapshot.yaml').read_text(encoding='utf-8')) or {})['analysis']['voltage_stratification_min_span_v'])
    report={'samples':int(len(all_)),'voltage_span_v':voltage_span,'minimum_voltage_span_for_sensitivity_assessment_v':min_span,'voltage_span_sufficient_for_sensitivity_assessment':bool(math.isfinite(voltage_span) and voltage_span>=min_span),'speed_map_residual_vs_voltage_slope_mps_per_v':float(slope),'speed_map_residual_voltage_correlation':corr,'motor_temp_range_c':[float(np.nanmin(all_.motor_temp_c)),float(np.nanmax(all_.motor_temp_c))] if len(all_) else None,'fet_temp_range_c':[float(np.nanmin(all_.fet_temp_c)),float(np.nanmax(all_.fet_temp_c))] if len(all_) else None,'interpretation':'Voltage sensitivity is reported, not automatically folded into the primary speed map. A material residual-versus-voltage slope indicates a possible battery/thermal/drivetrain covariate requiring a deliberately extended model. Insufficient voltage span is reported explicitly rather than treated as evidence of no sensitivity.'}
    dump_yaml(out/'voltage_temperature_report.yaml',report); print(json.dumps(report,indent=2)); return 0
if __name__=='__main__': raise SystemExit(main())
