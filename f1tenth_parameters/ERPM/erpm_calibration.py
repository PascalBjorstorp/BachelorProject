#!/usr/bin/env python3
"""Run the complete ERPM, odometry and longitudinal-dynamics calibration."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from erpm_calibration.cli import main
if __name__=='__main__': raise SystemExit(main())
