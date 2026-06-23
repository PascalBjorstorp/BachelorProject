"""Helpers for excluding operator-rejected trial attempts from offline fitting."""
from __future__ import annotations

from typing import Iterable

import pandas as pd


def accepted_trial_ids(events: pd.DataFrame) -> set[str]:
    if "event" not in events.columns or "trial_id" not in events.columns:
        return set()
    decisions = events[events["event"] == "trial_decision"].copy()
    if not len(decisions):
        return set()
    decisions = decisions.sort_values("bag_ns")
    final = decisions.groupby("trial_id", as_index=False).tail(1)
    return {str(value) for value in final.loc[final.get("decision") == "accepted", "trial_id"].dropna()}


def accepted_event_rows(events: pd.DataFrame, names: str | Iterable[str]) -> pd.DataFrame:
    wanted = {names} if isinstance(names, str) else set(names)
    if "event" not in events.columns:
        return events.iloc[0:0].copy()
    subset = events[events["event"].isin(wanted)].copy()
    accepted = accepted_trial_ids(events)
    if not accepted or "trial_id" not in subset.columns:
        return subset.iloc[0:0].copy()
    return subset[subset["trial_id"].astype(str).isin(accepted)].copy()
