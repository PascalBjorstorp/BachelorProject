#!/usr/bin/env python3
import argparse
import csv
import math
from typing import Dict, List

FIELDS_FLOAT = [
    "primal_residual", "dual_residual",
    "state_primal_residual", "state_dual_residual",
    "ctrl_primal_residual", "ctrl_dual_residual",
    "rho", "rho_u",
    "u0_steer", "u0_accel",
    "z0_steer", "z0_accel",
    "y0_steer", "y0_accel",
]
FIELDS_INT = ["iter", "scale_rho", "scale_rho_u"]


def load(path: str) -> List[Dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def key(r: Dict[str, str]):
    return int(r["iter"])


def fmt_row(r: Dict[str, str]) -> str:
    return (
        f"iter={r['iter']} p={r['primal_residual']} d={r['dual_residual']} "
        f"sp={r['state_primal_residual']} sd={r['state_dual_residual']} "
        f"cp={r['ctrl_primal_residual']} cd={r['ctrl_dual_residual']} "
        f"rho={r['rho']} rho_u={r['rho_u']} "
        f"u0=[{r['u0_steer']},{r['u0_accel']}] z0=[{r['z0_steer']},{r['z0_accel']}] "
        f"y0=[{r['y0_steer']},{r['y0_accel']}] scale=[{r['scale_rho']},{r['scale_rho_u']}]"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cpu", required=True)
    ap.add_argument("--fpga", required=True)
    ap.add_argument("--rtol", type=float, default=1e-3)
    ap.add_argument("--atol", type=float, default=1e-6)
    args = ap.parse_args()

    cpu = {key(r): r for r in load(args.cpu)}
    fpga = {key(r): r for r in load(args.fpga)}

    common = sorted(set(cpu.keys()) & set(fpga.keys()))
    if not common:
        print("No common iterations between traces")
        return

    print(f"cpu iters={len(cpu)} fpga iters={len(fpga)} common={len(common)}")

    for it in common:
        rc = cpu[it]
        rf = fpga[it]

        for f in FIELDS_INT:
            vc = int(rc[f])
            vf = int(rf[f])
            if vc != vf:
                print(f"First mismatch iter={it}, field={f}, cpu={vc}, fpga={vf}")
                print("cpu:", fmt_row(rc))
                print("fpga:", fmt_row(rf))
                return

        for f in FIELDS_FLOAT:
            vc = float(rc[f])
            vf = float(rf[f])
            if not math.isclose(vc, vf, rel_tol=args.rtol, abs_tol=args.atol):
                print(f"First mismatch iter={it}, field={f}, cpu={vc:.9g}, fpga={vf:.9g}")
                print("cpu:", fmt_row(rc))
                print("fpga:", fmt_row(rf))
                return

    print("No mismatches within tolerances")


if __name__ == "__main__":
    main()
