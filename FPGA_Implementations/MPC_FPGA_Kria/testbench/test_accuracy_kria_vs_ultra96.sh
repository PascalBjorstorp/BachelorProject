#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KRIA_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ULTRA_DIR="${KRIA_DIR}/../MPC_FPGA_Ultra96"
BUILD_DIR="${KRIA_DIR}/build/accuracy_compare"
mkdir -p "${BUILD_DIR}"

if [[ ! -d "${ULTRA_DIR}" ]]; then
  echo "FAIL: Ultra96 project not found at ${ULTRA_DIR}" >&2
  exit 1
fi

if [[ -z "${XILINX_HLS:-}" ]]; then
    if [[ -f "/tools/Xilinx/2025.1/Vitis/settings64.sh" ]]; then
        # shellcheck disable=SC1091
        source /tools/Xilinx/2025.1/Vitis/settings64.sh
    fi
fi

if [[ -z "${XILINX_HLS:-}" ]]; then
    echo "FAIL: XILINX_HLS is not set; source Vitis settings first." >&2
    exit 1
fi

SEED="${SIM_SEED:-1}"
START_WP="${START_WP:-0}"
DIFF_TRACE="${MPC_DIFF_TRACE:-1}"

EXTRA_DEFS_ARR=()
if [[ -n "${EXTRA_DEFS:-}" ]]; then
    # shellcheck disable=SC2206
    EXTRA_DEFS_ARR=(${EXTRA_DEFS})
fi

KRIA_BIN="${BUILD_DIR}/kria_sim"
ULTRA_BIN="${BUILD_DIR}/ultra_sim"
KRIA_LOG="${BUILD_DIR}/kria.log"
ULTRA_LOG="${BUILD_DIR}/ultra.log"

# Build Kria (all C++ sources + C testbench compiled as C++)
(
  cd "${KRIA_DIR}"
  g++ -O2 -std=c++17 -D_GNU_SOURCE -DMPC_HLS_TARGET \
        "${EXTRA_DEFS_ARR[@]}" \
    -Iinclude \
        -I"${XILINX_HLS}/include" \
    -x c++ \
    src/fp_math_hls.cpp \
    src/vehicle_model_hls.cpp \
    src/riccati_solver_hls.cpp \
    src/mpc_riccati_hls.cpp \
    src/mpc_internal_diag.cpp \
    src/mpc_fpga_top.cpp \
    testbench/test_fpga_sim_drive.c \
    -lm -o "${KRIA_BIN}"
)

# Build Ultra96 reference (C core + C++ top)
ULTRA_OBJ_DIR="${BUILD_DIR}/ultra_obj"
mkdir -p "${ULTRA_OBJ_DIR}"
(
  cd "${ULTRA_DIR}"
    g++ -O2 -std=c++17 -D_GNU_SOURCE -DMPC_HLS_TARGET -Iinclude -I"${XILINX_HLS}/include" -Wno-unknown-pragmas -x c++ -c src/fp_math_hls.c -o "${ULTRA_OBJ_DIR}/fp_math_hls.o"
    g++ -O2 -std=c++17 -D_GNU_SOURCE -DMPC_HLS_TARGET -Iinclude -I"${XILINX_HLS}/include" -Wno-unknown-pragmas -x c++ -c src/vehicle_model_hls.c -o "${ULTRA_OBJ_DIR}/vehicle_model_hls.o"
    g++ -O2 -std=c++17 -D_GNU_SOURCE -DMPC_HLS_TARGET -Iinclude -I"${XILINX_HLS}/include" -Wno-unknown-pragmas -x c++ -c src/riccati_solver_hls.c -o "${ULTRA_OBJ_DIR}/riccati_solver_hls.o"
    g++ -O2 -std=c++17 -D_GNU_SOURCE -DMPC_HLS_TARGET -Iinclude -I"${XILINX_HLS}/include" -Wno-unknown-pragmas -x c++ -c src/mpc_riccati_hls.c -o "${ULTRA_OBJ_DIR}/mpc_riccati_hls.o"
    g++ -O2 -std=c++17 -D_GNU_SOURCE -DMPC_HLS_TARGET -Iinclude -I"${XILINX_HLS}/include" -Wno-unknown-pragmas -x c++ -c testbench/test_fpga_sim_drive.c -o "${ULTRA_OBJ_DIR}/test_fpga_sim_drive.o"
    g++ -O2 -std=c++17 -D_GNU_SOURCE -DMPC_HLS_TARGET -Iinclude -I"${XILINX_HLS}/include" -Wno-unknown-pragmas -x c++ -c src/mpc_fpga_top.cpp -o "${ULTRA_OBJ_DIR}/mpc_fpga_top.o"
  g++ "${ULTRA_OBJ_DIR}"/*.o -lm -o "${ULTRA_BIN}"
)

set +e
SIM_SEED="${SEED}" START_WP="${START_WP}" MPC_DIFF_TRACE="${DIFF_TRACE}" "${KRIA_BIN}" >"${KRIA_LOG}" 2>&1
kria_rc=$?
SIM_SEED="${SEED}" START_WP="${START_WP}" MPC_DIFF_TRACE="${DIFF_TRACE}" "${ULTRA_BIN}" >"${ULTRA_LOG}" 2>&1
ultra_rc=$?
set -e

python3 - "${KRIA_LOG}" "${ULTRA_LOG}" "${kria_rc}" "${ultra_rc}" <<'PY'
import re
import sys
from pathlib import Path

kria_log = Path(sys.argv[1]).read_text(errors="ignore")
ultra_log = Path(sys.argv[2]).read_text(errors="ignore")
kria_rc = int(sys.argv[3])
ultra_rc = int(sys.argv[4])

patterns = {
    "max_lat": r"Max lateral error:\s*([0-9.]+)\s*m",
    "avg_lat": r"Avg lateral error:\s*([0-9.]+)\s*m",
    "max_hdg": r"Max heading error:\s*([0-9.]+)\s*rad",
    "avg_hdg": r"Avg heading error:\s*([0-9.]+)\s*rad",
    "max_vel": r"Max velocity error:\s*([0-9.]+)\s*m/s",
    "avg_vel": r"Avg velocity error:\s*([0-9.]+)\s*m/s",
    "avg_iter": r"Avg iterations/call:\s*([0-9.]+)",
    "wall_hits": r"Wall collisions:\s*([0-9]+)",
}

limits = {
    "max_lat": 0.20,
    "avg_lat": 0.06,
    "max_hdg": 0.20,
    "avg_hdg": 0.08,
    "max_vel": 1.0,
    "avg_vel": 0.50,
    "avg_iter": 6.0,
}

def parse(text: str):
    out = {}
    for key, pat in patterns.items():
        m = re.search(pat, text)
        if not m:
            raise SystemExit(f"FAIL: could not parse {key}")
        out[key] = int(m.group(1)) if key == "wall_hits" else float(m.group(1))
    return out

def parse_crash(text: str):
    m = re.search(
        r"WALL CRASH(?: \(MAP\))?: e_y =\s*([+-]?[0-9.]+) m(?: \(bound: ([0-9.]+)\))? at step ([0-9]+) \(t=([0-9.]+)s, wp=([0-9]+), v=([0-9.]+)\)",
        text,
    )
    if not m:
        return None
    return {
        "e_y": float(m.group(1)),
        "bound": float(m.group(2)) if m.group(2) is not None else None,
        "step": int(m.group(3)),
        "time_s": float(m.group(4)),
        "wp": int(m.group(5)),
        "v": float(m.group(6)),
    }

def parse_trace_mpc(text: str):
    rows = {}
    for line in text.splitlines():
        if not line.startswith("TRACE_MPC,"):
            continue
        parts = line.split(",")
        if len(parts) < 12:
            continue
        step = int(parts[1])
        if len(parts) >= 14:
            rows[step] = {
                "wp": int(parts[2]),
                "mpc_e_y": float(parts[3]),
                "mpc_e_psi": float(parts[4]),
                "ref_left0": float(parts[5]),
                "ref_right0": float(parts[6]),
                "ref_left0_raw": int(parts[7]),
                "ref_right0_raw": int(parts[8]),
                "cmd_steer": float(parts[9]),
                "cmd_accel": float(parts[10]),
                "st_delta": float(parts[11]),
                "status": int(parts[12]),
                "iters": int(parts[13]),
            }
        else:
            rows[step] = {
                "wp": int(parts[2]),
                "mpc_e_y": float(parts[3]),
                "mpc_e_psi": float(parts[4]),
                "ref_left0": float(parts[5]),
                "ref_right0": float(parts[6]),
                "ref_left0_raw": None,
                "ref_right0_raw": None,
                "cmd_steer": float(parts[7]),
                "cmd_accel": float(parts[8]),
                "st_delta": float(parts[9]),
                "status": int(parts[10]),
                "iters": int(parts[11]),
            }
    return rows

kria = parse(kria_log)
ultra = parse(ultra_log)
kria_crash = parse_crash(kria_log)
ultra_crash = parse_crash(ultra_log)
kria_trace = parse_trace_mpc(kria_log)
ultra_trace = parse_trace_mpc(ultra_log)

fails = []
if kria_rc not in (0, 1):
    fails.append(f"unexpected Kria exit code: {kria_rc}")
if ultra_rc not in (0, 1):
    fails.append(f"unexpected Ultra96 exit code: {ultra_rc}")
if kria["wall_hits"] != 0:
    fails.append(f"Kria wall hits: {kria['wall_hits']}")
if ultra["wall_hits"] != 0:
    fails.append(f"Ultra96 wall hits: {ultra['wall_hits']}")

for k, lim in limits.items():
    drift = abs(kria[k] - ultra[k])
    if drift > lim:
        fails.append(f"{k} drift {drift:.6f} > {lim:.6f}")

print("=== Kria vs Ultra96 Accuracy ===")
for k in ("max_lat", "avg_lat", "max_hdg", "avg_hdg", "max_vel", "avg_vel", "avg_iter"):
    drift = abs(kria[k] - ultra[k])
    print(f"{k:8s} kria={kria[k]:.6f} ultra={ultra[k]:.6f} drift={drift:.6f}")
print(f"wall_hits kria={kria['wall_hits']} ultra={ultra['wall_hits']}")

if kria_crash:
    print(
        "kria_crash"
        f" step={kria_crash['step']} wp={kria_crash['wp']}"
        f" e_y={kria_crash['e_y']:.6f}"
        f" bound={(kria_crash['bound'] if kria_crash['bound'] is not None else float('nan')):.6f}"
        f" v={kria_crash['v']:.3f}"
    )
if ultra_crash:
    print(
        "ultra_crash"
        f" step={ultra_crash['step']} wp={ultra_crash['wp']}"
        f" e_y={ultra_crash['e_y']:.6f}"
        f" bound={(ultra_crash['bound'] if ultra_crash['bound'] is not None else float('nan')):.6f}"
        f" v={ultra_crash['v']:.3f}"
    )

if kria_trace and ultra_trace:
    common_steps = sorted(set(kria_trace) & set(ultra_trace))
    first_div = None
    for step in common_steps:
        krow = kria_trace[step]
        urow = ultra_trace[step]
        ey_diff = abs(krow["mpc_e_y"] - urow["mpc_e_y"])
        steer_diff = abs(krow["cmd_steer"] - urow["cmd_steer"])
        accel_diff = abs(krow["cmd_accel"] - urow["cmd_accel"])
        if ey_diff > 0.01 or steer_diff > 0.01 or accel_diff > 0.10 or krow["wp"] != urow["wp"]:
            first_div = (step, ey_diff, steer_diff, accel_diff, krow, urow)
            break
    if first_div:
        step, ey_diff, steer_diff, accel_diff, krow, urow = first_div
        print(
            "first_divergence"
            f" step={step}"
            f" ey_diff={ey_diff:.6f}"
            f" steer_diff={steer_diff:.6f}"
            f" accel_diff={accel_diff:.6f}"
        )
        print(
            "  kria"
            f" wp={krow['wp']} mpc_e_y={krow['mpc_e_y']:.6f}"
            f" steer={krow['cmd_steer']:.6f} accel={krow['cmd_accel']:.6f}"
            f" status={krow['status']} iters={krow['iters']}"
            f" refL0={krow['ref_left0']:.6f} refR0={krow['ref_right0']:.6f}"
            f" refL0_raw={krow['ref_left0_raw']} refR0_raw={krow['ref_right0_raw']}"
        )
        print(
            "  ultra"
            f" wp={urow['wp']} mpc_e_y={urow['mpc_e_y']:.6f}"
            f" steer={urow['cmd_steer']:.6f} accel={urow['cmd_accel']:.6f}"
            f" status={urow['status']} iters={urow['iters']}"
            f" refL0={urow['ref_left0']:.6f} refR0={urow['ref_right0']:.6f}"
            f" refL0_raw={urow['ref_left0_raw']} refR0_raw={urow['ref_right0_raw']}"
        )
    else:
        print("first_divergence none (within configured thresholds)")

if fails:
    print("FAIL accuracy check")
    for msg in fails:
        print(f" - {msg}")
    raise SystemExit(1)

print("PASS accuracy check")
PY
