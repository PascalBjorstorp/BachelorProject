#!/usr/bin/env python3
"""Run one gated vehicle-calibration stage at a time.

Examples::

    python3 run_suite.py new
    python3 run_suite.py run --session runs/<id> --stage steering_centre
    python3 run_suite.py run --session runs/<id> --next
    python3 run_suite.py redo --session runs/<id> --from steering_centre_validation
    python3 run_suite.py report --session runs/<id>
    python3 run_suite.py recover --workspace ~/BachelorProject
    python3 run_suite.py gui

``run --next`` is still one stage: it captures, exports, analyses, gates and
rebuilds only the next stage, then returns.  There is intentionally no public
``--all`` shortcut: physical review of each B result and C validation is part
of the campaign safety/identifiability contract.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from calibration_suite.runner import SuiteRunner


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["new", "run", "redo", "report", "recover", "gui"])
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "suite.yaml")
    parser.add_argument("--session", type=Path)
    parser.add_argument("--stage")
    parser.add_argument("--from", dest="redo_from",
                        help="For redo: validation/fitted stage to restart from with a fresh A/B branch.")
    parser.add_argument("--next", action="store_true", dest="run_next")
    parser.add_argument("--workspace", type=Path)
    parser.add_argument("--promote", action="store_true",
                        help="For report: apply only the final gated patch to source vesc.yaml and rebuild.")
    parser.add_argument("--host", default="127.0.0.1",
                        help="For gui: localhost bind address (remote binding is intentionally rejected).")
    parser.add_argument("--port", type=int, default=8765,
                        help="For gui: local HTTP port; use 0 to select a free port.")
    parser.add_argument("--no-browser", action="store_true",
                        help="For gui: print the local URL without opening a browser automatically.")
    args = parser.parse_args()

    if args.command == "gui":
        from calibration_suite.gui import serve_gui
        workspace = (args.workspace or ROOT.parents[1]).expanduser().resolve()
        serve_gui(
            args.config.resolve(), workspace, args.session,
            host=args.host, port=args.port, open_browser=not args.no_browser,
        )
        return 0
    if args.command == "recover":
        SuiteRunner.recover(args.workspace)
        return 0
    if args.command in {"run", "redo", "report"} and args.session is None:
        parser.error(f"{args.command} requires --session")
    runner = SuiteRunner(args.config.resolve(), args.session, args.workspace)
    if args.command == "new":
        print(runner.session)
        return 0
    if args.command == "report":
        runner.build_report(promote=args.promote)
        return 0
    if args.command == "redo":
        if not args.redo_from:
            parser.error("redo requires --from <stage>")
        if args.stage or args.run_next:
            parser.error("redo accepts only --from and --session")
        runner.redo_from(args.redo_from)
        return 0
    if args.stage and args.run_next:
        parser.error("use either --stage or --next")
    if not (args.stage or args.run_next):
        parser.error("run requires --stage or --next")
    if args.stage:
        runner.run_one(args.stage)
    else:
        runner.run_next()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
