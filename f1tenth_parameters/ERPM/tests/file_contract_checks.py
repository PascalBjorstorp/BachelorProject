#!/usr/bin/env python3
"""Check documentation/config stage counts and no stale coverage promises."""
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
cfg=yaml.safe_load((ROOT/'config'/'erpm_calibration.yaml').read_text())
readme=(ROOT/'README.md').read_text()
assert '| 12 |' in readme
assert int(cfg['raw_erpm_map_training']['repetitions'])==5
assert int(cfg['raw_erpm_map_holdout']['repetitions'])==3
assert 'max_attempts_per_condition' not in cfg.get('session',{})
print('ERPM file-contract checks passed')
