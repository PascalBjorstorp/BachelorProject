#!/usr/bin/env python3
import math
from pathlib import Path

N = 1024
ATAN_DOMAIN = 8
TRIG_DOMAIN = math.pi
TRIG_SCALE = N / TRIG_DOMAIN
RECIP_N = 256

ROOT = Path(__file__).resolve().parents[1]
INCLUDE_DIR = ROOT / "include"


def fmt(value: float) -> str:
    return format(value, ".20f")


def trig_tables():
    sin_vals = [math.sin(TRIG_DOMAIN * i / N) for i in range(N + 1)]
    cos_vals = [math.cos(TRIG_DOMAIN * i / N) for i in range(N + 1)]

    sin_vals[0] = 0.0
    sin_vals[-1] = 0.0
    cos_vals[0] = 1.0
    cos_vals[-1] = -1.0
    return sin_vals, cos_vals


def emit_qp_header() -> None:
    sin_vals, cos_vals = trig_tables()
    atan_vals = [math.atan(ATAN_DOMAIN * i / N) for i in range(N + 1)]

    fp_frac_bits = 14  # QP now Q12.14
    one_raw = 1 << fp_frac_bits
    recip_vals = [int((one_raw * (2 * RECIP_N)) // (RECIP_N + i))
                  for i in range(RECIP_N + 1)]

    lines = [
        "#ifndef FP_TRIG_LUT_1024_H",
        "#define FP_TRIG_LUT_1024_H",
        "",
        '#include "fp_math_hls.h"',
        "",
        "/* QP trig LUT scale: 1024 / pi. Tables span [0, pi] inclusive.",
        " * Angles are normalized to [-pi, pi], folded with symmetry into",
        " * [0, pi], and sine sign is restored in fp_math_hls.cpp. */",
        f"#define FP_TRIG_LUT_SCALE FP_QP_CONST({fmt(TRIG_SCALE)})",
        "#define FP_TRIG_LUT_SIZE 1024",
        "#define FP_TRIG_LUT_MASK (FP_TRIG_LUT_SIZE - 1)",
        "",
        "static const fp_QP_t sin_lut[1025] = {",
    ]
    lines.extend(f"    FP_QP_CONST({fmt(v)})," for v in sin_vals)
    lines.extend([
        "};",
        "",
        "static const fp_QP_t cos_lut[1025] = {",
    ])
    lines.extend(f"    FP_QP_CONST({fmt(v)})," for v in cos_vals)
    lines.extend([
        "};",
        "",
        "static const fp_QP_t atan_lut[1025] = {",
    ])
    lines.extend(f"    FP_QP_CONST({fmt(v)})," for v in atan_vals)
    lines.extend([
        "};",
        "",
        f"/* QP reciprocal LUT for x_norm=(L+i)/(2L), L={RECIP_N}. */",
        f"static const int32_t recip_lut[{RECIP_N + 1}] = {{",
    ])
    lines.extend(f"    {v}," for v in recip_vals)
    lines.extend([
        "};",
        "",
        "#endif",
    ])

    (INCLUDE_DIR / "fp_trig_lut_1024.h").write_text("\n".join(lines) + "\n")


def emit_fn_header() -> None:
    sin_vals, cos_vals = trig_tables()
    atan_vals = [math.atan(ATAN_DOMAIN * i / N) for i in range(N + 1)]

    fp_frac_bits = 14  # FN unified to F=14
    one_raw = 1 << fp_frac_bits
    recip_vals = [int((one_raw * (2 * RECIP_N)) // (RECIP_N + i))
                  for i in range(RECIP_N + 1)]
    trig_scale_raw = round(TRIG_SCALE * (1 << fp_frac_bits))

    lines = [
        "#ifndef FP_TRIG_LUT_FN_1024_H",
        "#define FP_TRIG_LUT_FN_1024_H",
        "",
        '#include "fp_math_hls.h"',
        "",
        "/* FN trig LUT scale in raw Q17 form. 1024/pi cannot be stored in",
        " * fp_FN_t because Q26.17 with 9 integer bits saturates below 256.",
        " * Angles are normalized to [-pi, pi], folded with symmetry into",
        " * [0, pi], and multiplied by this raw scale in fp_math_hls.cpp. */",
        f"#define FP_FN_TRIG_LUT_SCALE_RAW ((int32_t){trig_scale_raw})",
        "#define FP_FN_TRIG_LUT_SIZE 1024",
        "#define FP_FN_TRIG_LUT_MASK (FP_FN_TRIG_LUT_SIZE - 1)",
        "",
        "static const fp_FN_t sin_lut_fn[1025] = {",
    ]
    lines.extend(f"    FP_FN_CONST({fmt(v)})," for v in sin_vals)
    lines.extend([
        "};",
        "",
        "static const fp_FN_t cos_lut_fn[1025] = {",
    ])
    lines.extend(f"    FP_FN_CONST({fmt(v)})," for v in cos_vals)
    lines.extend([
        "};",
        "",
        "static const fp_FN_t atan_lut_fn[1025] = {",
    ])
    lines.extend(f"    FP_FN_CONST({fmt(v)})," for v in atan_vals)
    lines.extend([
        "};",
        "",
        "/* Correct NR seed LUT for fp_FN_t reciprocal (Q26.17 raw).",
        f" * Entry i = floor(1/((L+i)/(2L)) * 2^17), L={RECIP_N}.",
        " * x_norm is normalized into [0.5, 1.0) before this lookup. */",
        f"static const int32_t recip_lut_fn[{RECIP_N + 1}] = {{",
    ])
    lines.extend(f"    {v}," for v in recip_vals)
    lines.extend([
        "};",
        "",
        "#endif",
    ])

    (INCLUDE_DIR / "fp_trig_lut_fn_1024.h").write_text("\n".join(lines) + "\n")


def main() -> None:
    emit_qp_header()
    emit_fn_header()
    print(f"Wrote {INCLUDE_DIR / 'fp_trig_lut_1024.h'}")
    print(f"Wrote {INCLUDE_DIR / 'fp_trig_lut_fn_1024.h'}")


if __name__ == "__main__":
    main()
