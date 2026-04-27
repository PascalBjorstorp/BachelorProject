#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${script_dir}/.." && pwd)"
cd "${project_dir}"

vitis_run="${VITIS_RUN:-/tools/Xilinx/2025.1/Vitis/bin/vitis-run}"
tcl_script="${HLS_TCL_SCRIPT:-${script_dir}/run_hls_stage2_export_impl_aggressive.tcl}"
solution="${HLS_SOLUTION_NAME:-kria_kv260}"
project_prefix="${SWEEP_PROJECT_PREFIX:-mpc_fpga_hls_sweep}"
out_dir="${SWEEP_OUT_DIR:-hls_sweep_results}"

kx_values="${SWEEP_KX_VALUES:-1 2}"
ctrl_values="${SWEEP_CTRL_VALUES:-2}"
riccati_mul_values="${SWEEP_RICCATI_MUL_LIMIT_VALUES:-6}"
adaptive_values="${SWEEP_ADAPTIVE_RHO_VALUES:-1}"

mkdir -p "${out_dir}"
csv="${out_dir}/affine_sweep_$(date +%Y%m%d_%H%M%S).csv"

trim_awk='function trim(s){gsub(/^[ \t]+|[ \t]+$/, "", s); return s}'

extract_latency() {
    local report="$1"
    awk -F'|' "${trim_awk}"'
        /^[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|/ {
            print trim($2) "," trim($3);
            exit;
        }
    ' "${report}"
}

extract_resources() {
    local report="$1"
    awk -F'|' "${trim_awk}"'
        /^\|Total[[:space:]]*\|/ {
            print trim($2) "," trim($3) "," trim($4) "," trim($5);
            exit;
        }
    ' "${report}"
}

extract_estimated_fmax() {
    local report="$1"
    local est_ns
    est_ns="$(awk -F'|' '
        /\|ap_clk[[:space:]]*\|/ {
            v=$3;
            gsub(/ ns/, "", v);
            gsub(/[ \t]/, "", v);
            print v;
            exit;
        }
    ' "${report}")"
    awk -v ns="${est_ns:-0}" 'BEGIN { if (ns > 0) printf "%.2f", 1000.0 / ns; }'
}

extract_admm_loop() {
    local report="$1"
    awk -F'|' "${trim_awk}"'
        /- VITIS_LOOP_/ {
            print trim($3) "," trim($4) "," trim($5);
            exit;
        }
    ' "${report}"
}

echo "project,kx_unroll,ctrl_unroll,riccati_mul_limit,adaptive_rho,top_min_cycles,top_max_cycles,top_bram,top_dsp,top_ff,top_lut,top_est_fmax_mhz,admm_min_cycles,admm_max_cycles,admm_loop_min_cycles,admm_loop_max_cycles,admm_iter_cycles,affine_cycles,affine_dsp,factor_cycles,factor_dsp,log" > "${csv}"

for kx in ${kx_values}; do
    for ctrl in ${ctrl_values}; do
        for mul_limit in ${riccati_mul_values}; do
            for adaptive in ${adaptive_values}; do
                    run_name="kx${kx}_ctrl${ctrl}_mul${mul_limit}_adapt${adaptive}"
                    hls_project="${project_prefix}/${run_name}"
                    log="${out_dir}/${run_name}.log"

                    echo "Running ${run_name}"
                    HLS_MODE=synth \
                    HLS_PROJECT_DIR="${hls_project}" \
                    HLS_SOLUTION_NAME="${solution}" \
                    HLS_UNROLL_KX_FACTOR="${kx}" \
                    HLS_AFFINE_CTRL_UNROLL="${ctrl}" \
                    HLS_RICCATI_MUL_LIMIT="${mul_limit}" \
                    HLS_ADAPTIVE_RHO="${adaptive}" \
                    "${vitis_run}" --mode hls --tcl "${tcl_script}" 2>&1 | tee "${log}"

                    report_dir="${hls_project}/${solution}/syn/report"
                    top_report="${report_dir}/mpc_fpga_top_opencl_csynth.rpt"
                    admm_report="${report_dir}/riccati_admm_solve_hls_csynth.rpt"
                    affine_report="${report_dir}/riccati_affine_forward_hls_csynth.rpt"
                    factor_report="${report_dir}/riccati_factor_hls_csynth.rpt"

                    top_latency="$(extract_latency "${top_report}")"
                    top_resources="$(extract_resources "${top_report}")"
                    top_fmax="$(extract_estimated_fmax "${top_report}")"
                    admm_latency="$(extract_latency "${admm_report}")"
                    admm_loop="$(extract_admm_loop "${admm_report}")"
                    affine_latency="$(extract_latency "${affine_report}" | cut -d, -f2)"
                    affine_dsp="$(extract_resources "${affine_report}" | cut -d, -f2)"
                    factor_latency="$(extract_latency "${factor_report}" | cut -d, -f2)"
                    factor_dsp="$(extract_resources "${factor_report}" | cut -d, -f2)"

                    echo "${hls_project},${kx},${ctrl},${mul_limit},${adaptive},${top_latency},${top_resources},${top_fmax},${admm_latency},${admm_loop},${affine_latency},${affine_dsp},${factor_latency},${factor_dsp},${log}" >> "${csv}"
            done
        done
    done
done

echo "Wrote ${csv}"
