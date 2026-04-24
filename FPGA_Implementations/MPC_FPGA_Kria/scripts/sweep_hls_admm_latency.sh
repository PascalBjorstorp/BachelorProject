#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TCL_PATH="scripts/run_hls_stage2_export_impl_aggressive.tcl"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${1:-$ROOT_DIR/sweep_runs/admm_latency_sweep_${STAMP}}"
mkdir -p "$OUT_DIR"

CSV_PATH="$OUT_DIR/results.csv"
echo "run_id,status,gma,pshift,kx,pnew,ctrl,step_ii,adaptive_rho,top_est_ns,top_min_cycles,top_max_cycles,admm_min_cycles,admm_max_cycles,admm_iter_min_cycles,admm_iter_max_cycles,mpc_min_cycles,mpc_max_cycles,warnings,log_path" > "$CSV_PATH"

read_top_metrics() {
    local rpt="$1"
    awk -F'|' '
        /\|ap_clk/ && !seen_clk {
            gsub(/ ns/, "", $4); gsub(/[[:space:]]/, "", $4);
            clk=$4; seen_clk=1;
        }
        /^[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9.]+[[:space:]]*us/ && !seen_lat {
            gsub(/[[:space:]]/, "", $2); gsub(/[[:space:]]/, "", $3);
            lmin=$2; lmax=$3; seen_lat=1;
        }
        END {
            if (!seen_clk) clk="NA";
            if (!seen_lat) { lmin="NA"; lmax="NA"; }
            printf "%s,%s,%s\n", clk, lmin, lmax;
        }
    ' "$rpt"
}

read_admm_metrics() {
    local rpt="$1"
    awk -F'|' '
        /^[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9.]+[[:space:]]*us/ && !seen_lat {
            gsub(/[[:space:]]/, "", $2); gsub(/[[:space:]]/, "", $3);
            lmin=$2; lmax=$3; seen_lat=1;
        }
        /VITIS_LOOP_615_19/ && !seen_iter {
            gsub(/[[:space:]]/, "", $3); gsub(/[[:space:]]/, "", $4);
            imin=$3; imax=$4; seen_iter=1;
        }
        END {
            if (!seen_lat) { lmin="NA"; lmax="NA"; }
            if (!seen_iter) { imin="NA"; imax="NA"; }
            printf "%s,%s,%s,%s\n", lmin, lmax, imin, imax;
        }
    ' "$rpt"
}

read_mpc_metrics() {
    local rpt="$1"
    awk -F'|' '
        /^[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9.]+[[:space:]]*us/ && !seen_lat {
            gsub(/[[:space:]]/, "", $2); gsub(/[[:space:]]/, "", $3);
            lmin=$2; lmax=$3; seen_lat=1;
        }
        END {
            if (!seen_lat) { lmin="NA"; lmax="NA"; }
            printf "%s,%s\n", lmin, lmax;
        }
    ' "$rpt"
}

# Focused sweep: enough spread to move ADMM cycle count without exploding runtime.
gma_vals=(2 4)
pshift_vals=(2 4)
kx_vals=(2 4)
pnew_vals=(2 4)
ctrl_vals=(2)
step_ii_vals=(6)
adaptive_rho_vals=(1)

run_id=0
for gma in "${gma_vals[@]}"; do
    for pshift in "${pshift_vals[@]}"; do
        for kx in "${kx_vals[@]}"; do
            for pnew in "${pnew_vals[@]}"; do
                for ctrl in "${ctrl_vals[@]}"; do
                    for step_ii in "${step_ii_vals[@]}"; do
                        for adaptive_rho in "${adaptive_rho_vals[@]}"; do
                            run_id=$((run_id + 1))
                            run_tag=$(printf "run_%03d_g%d_p%d_k%d_pn%d_c%d_s%d_ar%d" \
                                "$run_id" "$gma" "$pshift" "$kx" "$pnew" "$ctrl" "$step_ii" "$adaptive_rho")
                            run_dir="$OUT_DIR/$run_tag"
                            mkdir -p "$run_dir"
                            run_log="$run_dir/synth.log"

                            status="ok"
                            if ! (
                                cd "$ROOT_DIR"
                                HLS_MODE=synth \
                                HLS_UNROLL_GMA_FACTOR="$gma" \
                                HLS_UNROLL_PSHIFT_FACTOR="$pshift" \
                                HLS_UNROLL_KX_FACTOR="$kx" \
                                HLS_AFFINE_PNEW_UNROLL="$pnew" \
                                HLS_AFFINE_CTRL_UNROLL="$ctrl" \
                                HLS_STEP_ASSEMBLY_II="$step_ii" \
                                HLS_ADAPTIVE_RHO="$adaptive_rho" \
                                vitis-run --mode hls --tcl "$TCL_PATH"
                            ) >"$run_log" 2>&1; then
                                status="fail"
                            fi

                            report_dir="$ROOT_DIR/mpc_fpga_hls_stage1/kria_kv260/syn/report"
                            top_rpt="$report_dir/mpc_fpga_top_opencl_csynth.rpt"
                            admm_rpt="$report_dir/riccati_admm_solve_hls_1_csynth.rpt"
                            mpc_rpt="$report_dir/mpc_compute_hls_csynth.rpt"

                            top_est_ns="NA"
                            top_min_cycles="NA"
                            top_max_cycles="NA"
                            admm_min_cycles="NA"
                            admm_max_cycles="NA"
                            admm_iter_min_cycles="NA"
                            admm_iter_max_cycles="NA"
                            mpc_min_cycles="NA"
                            mpc_max_cycles="NA"

                            if [[ -f "$top_rpt" ]]; then
                                IFS=',' read -r top_est_ns top_min_cycles top_max_cycles < <(read_top_metrics "$top_rpt")
                                cp "$top_rpt" "$run_dir/"
                            fi
                            if [[ -f "$admm_rpt" ]]; then
                                IFS=',' read -r admm_min_cycles admm_max_cycles admm_iter_min_cycles admm_iter_max_cycles < <(read_admm_metrics "$admm_rpt")
                                cp "$admm_rpt" "$run_dir/"
                            fi
                            if [[ -f "$mpc_rpt" ]]; then
                                IFS=',' read -r mpc_min_cycles mpc_max_cycles < <(read_mpc_metrics "$mpc_rpt")
                                cp "$mpc_rpt" "$run_dir/"
                            fi

                            warnings="$(grep -c '^WARNING:' "$run_log" || true)"
                            echo "$run_id,$status,$gma,$pshift,$kx,$pnew,$ctrl,$step_ii,$adaptive_rho,$top_est_ns,$top_min_cycles,$top_max_cycles,$admm_min_cycles,$admm_max_cycles,$admm_iter_min_cycles,$admm_iter_max_cycles,$mpc_min_cycles,$mpc_max_cycles,$warnings,$run_log" >> "$CSV_PATH"
                            echo "[$run_id] $status -> $run_tag"
                        done
                    done
                done
            done
        done
    done
done

echo "Sweep complete: $CSV_PATH"
