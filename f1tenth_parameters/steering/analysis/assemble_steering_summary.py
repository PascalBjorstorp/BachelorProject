#!/usr/bin/env python3
"""Assemble principal steering-calibration outputs into one review file."""
from __future__ import annotations
import argparse
import json
from pathlib import Path


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args=parser.parse_args()
    session=args.session.resolve()
    out=session/'analysis'
    summary={
        'centre': load_json(out/'centre_trim_offline.json'),
        'static_map': load_json(out/'candidate_static_steering_map.json'),
        'effective_response': load_json(out/'command_to_effective_steering_response_summary.json'),
        'simulation_seed_report': load_json(out/'steering_simulation_seed_report.json'),
        'icp_observability': load_json(out/'icp_observability_report.json'),
        'review_required': True,
        'automatic_installation_performed': False,
    }
    (out/'steering_calibration_review_summary.json').write_text(json.dumps(summary, indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
