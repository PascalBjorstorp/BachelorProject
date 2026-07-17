"""Access the robust LiDAR time-window product used by calibration fits."""
from __future__ import annotations

from pathlib import Path

import pandas as pd


def read_motion(stage_derived: Path) -> pd.DataFrame:
    """Prefer multi-registration windows over correlated raw scan-pair rows.

    Older sessions do not contain the window file, so the raw product remains a
    read-only compatibility fallback.  New campaigns always write both files.
    """
    windowed = stage_derived / "lidar_window_motion.parquet"
    source = windowed if windowed.exists() else stage_derived / "lidar_velocity.parquet"
    return pd.read_parquet(source)

