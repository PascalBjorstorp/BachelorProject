from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
# The campaign is split into two scripts; default the module entry to Tier 1.
from erpm_calibration.cli import main_config_calibration
raise SystemExit(main_config_calibration())
