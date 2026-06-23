from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from erpm_calibration.cli import main
raise SystemExit(main())
