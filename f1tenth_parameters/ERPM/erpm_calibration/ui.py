"""Terminal UI for an operator-led ERPM calibration session."""
from __future__ import annotations

import shutil
import sys
from typing import Any, Iterable


WIDTH = 72


def banner(title: str, subtitle: str | None = None) -> None:
    print("\n" + "=" * WIDTH)
    print(title)
    if subtitle:
        print(subtitle)
    print("=" * WIDTH)


def note(text: str) -> None:
    print(text)


def checklist(items: Iterable[str]) -> None:
    for item in items:
        print(f"  [ ] {item}")


def require_ready(prompt: str = "Type READY to continue, or ABORT to stop") -> None:
    while True:
        answer = input(f"\n{prompt}: ").strip().upper()
        if answer == "READY":
            return
        if answer in {"ABORT", "QUIT", "Q"}:
            raise KeyboardInterrupt("operator aborted the session")
        print("Please type READY or ABORT.")


def pause_for_reposition(message: str) -> None:
    print("\n" + message)
    require_ready("Type READY when the vehicle is positioned, or ABORT to stop")


def review_trial(
    *,
    label: str,
    automatic_ok: bool,
    automatic_summary: dict[str, Any] | None = None,
) -> str:
    """Obtain the operator's disposition of a just-recorded trial.

    A rejected/redo trial remains in the MCAP bag, but it is marked by a
    structured event and offline analysis will exclude it. If automatic startup
    stability or runtime checks fail, the attempt is a mandatory redo.
    """
    print("\n" + "-" * WIDTH)
    print(f"TRIAL REVIEW — {label}")
    if automatic_summary:
        for key, value in automatic_summary.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.4f}")
            else:
                print(f"  {key}: {value}")
    if automatic_ok:
        print("\nDid the vehicle complete the intended manoeuvre safely and cleanly?")
        print("  ACCEPT = keep this trial for offline fitting")
        print("  REDO   = exclude this attempt and repeat the same condition")
        print("  SKIP   = exclude this condition and continue")
        allowed = {"ACCEPT": "accepted", "A": "accepted", "REDO": "redo", "R": "redo", "SKIP": "skipped", "S": "skipped"}
    else:
        print("\nAutomatic checks did not pass. This attempt must be redone.")
        print("  REDO = repeat this condition after repositioning")
        allowed = {"REDO": "redo", "R": "redo"}
    prompt = "Decision [ACCEPT/REDO/SKIP/ABORT]" if automatic_ok else "Decision [REDO/ABORT]"
    while True:
        answer = input(f"{prompt}: ").strip().upper()
        if answer in {"ABORT", "QUIT", "Q"}:
            raise KeyboardInterrupt("operator aborted during trial review")
        if answer in allowed:
            return allowed[answer]
        print("Enter one of the displayed decisions.")


def warn(message: str) -> None:
    print(f"\nWARNING: {message}\n", file=sys.stderr)


def info_block(lines: Iterable[str]) -> None:
    for line in lines:
        print(line)


def disk_line(path: str) -> str:
    usage = shutil.disk_usage(path)
    return f"Free disk space: {usage.free / (1024 ** 3):.1f} GiB"
