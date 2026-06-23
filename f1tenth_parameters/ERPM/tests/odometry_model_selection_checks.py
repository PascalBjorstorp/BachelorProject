#!/usr/bin/env python3
"""Numerical regression checks for the offline adaptive odometry selector."""
from __future__ import annotations
import importlib.util
import sys
from pathlib import Path
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'analysis'))
spec=importlib.util.spec_from_file_location('selector', ROOT/'analysis'/'fit_odom_model_selection.py')
m=importlib.util.module_from_spec(spec); assert spec.loader; sys.modules[spec.name]=m; spec.loader.exec_module(m)

# Zero-intercept contract for all static map forms.
x=np.array([-1000.,-400.,0.,400.,1000.])
p={'linear':0.001,'quadratic':1e-7}
y=m._eval_poly(x,p)
assert y[2] == 0.0
lut={'x_knots':[0.,400.,1000.],'y_knots':[0.,.4,1.1]}
assert m._eval_lut(np.array([0.]),lut)[0] == 0.0

# Synthetic high-drive wheel overspeed: static map overestimates ground speed;
# adaptive residual should recover the true speed on an unseen trajectory.
rng=np.random.default_rng(7)
n=600
t=np.arange(n)*0.02
a=np.where((t>2)&(t<5), 3.0, np.where((t>7)&(t<10),-2.8,0.1))
v=np.cumsum(a)*0.02+0.8
v=np.clip(v,.1,2.8)
erpm=v/0.001 + 450*a + rng.normal(0,5,n) # wheel overspeed/underspeed
current=8*a + rng.normal(0,.4,n)
frame=pd.DataFrame({'trial_id':['A']*n,'bag_ns':(t*1e9).astype('int64'),'vx_truth':v,'erpm':erpm,'imu_ax':a,'motor_current':current})
frame['a_filt']=m._causal_lowpass_by_trial(frame,.05)
static=.001*frame.erpm.to_numpy()
coef=m._ridge(m._design_residual_features(static,frame.a_filt.to_numpy(),frame.motor_current.to_numpy()),frame.vx_truth.to_numpy()-static,.02)
adapt=m._apply_adaptive(static,frame.a_filt.to_numpy(),frame.motor_current.to_numpy(),coef,1.5)
assert m._rmse(v,adapt) < m._rmse(v,static)*0.65
# Fusion remains finite and causal state starts at observation.
fused=m._fused(frame,adapt,frame.a_filt.to_numpy(),w_coast=.85,w_high=.12,accel_transition=1.4,min_weight=.04)
assert np.isfinite(fused).all() and abs(fused[0]-adapt[0])<1e-12
print('odometry model selection numerical checks passed')
