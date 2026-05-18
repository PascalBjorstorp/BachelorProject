#!/usr/bin/env python3
from decimal import Decimal, getcontext
import math
getcontext().prec = 50

FPREFIX = '/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/include'

def fmt(x):
    # format with 20 decimal places
    return format(x, '.20f')

# trig LUTs: 1024 segments over 2*pi -> 1025 entries
N = 1024
sin_vals = [math.sin(2 * math.pi * i / N) for i in range(N + 1)]
cos_vals = [math.cos(2 * math.pi * i / N) for i in range(N + 1)]

# atan LUT: map [0, ATAN_DOMAIN] into 1024 segments of atan (reciprocal-free).
# Must match FP_ATAN_LUT_DOMAIN in fp_math_hls.h. NOTE: this script only
# rewrites the QP header (fp_trig_lut_1024.h). The FN header's atan_lut_fn
# block must be regenerated with the SAME ATAN_DOMAIN (atan(ATAN_DOMAIN*i/N)).
ATAN_DOMAIN = 8
atan_vals = [math.atan(ATAN_DOMAIN * i / N) for i in range(N + 1)]

# recip LUT for x_norm in [0.5,1), 10-bit index, 1025 entries (+1 lerp guard).
# recip_lut[i] = 2^F / x_norm at x_norm=(1024+i)/2048. Accurate values used
# DIRECTLY with linear interpolation (no Newton-Raphson) in fp_recip.
# NOTE: only the QP header is written here; the FN header's recip_lut_fn[1025]
# must be regenerated the same way with F=FP_FN_FRAC_BITS (17).
FP_FRAC_BITS = 18
one_raw = 1 << FP_FRAC_BITS
recip = []
for i in range(1025):
    val = (one_raw * (1 << 11)) // (1024 + i)
    recip.append(int(val))

header = []
header.append('#ifndef FP_TRIG_LUT_1024_H')
header.append('#define FP_TRIG_LUT_1024_H')
header.append('\n#include "fp_math_hls.h"')
header.append('\nstatic const fp_QP_t sin_lut[1025] = {')
for v in sin_vals:
    header.append('    FP_QP_CONST(%s),' % fmt(v))
header.append('};\n')
header.append('static const fp_QP_t cos_lut[1025] = {')
for v in cos_vals:
    header.append('    FP_QP_CONST(%s),' % fmt(v))
header.append('};\n')
header.append('static const fp_QP_t atan_lut[1025] = {')
for v in atan_vals:
    header.append('    FP_QP_CONST(%s),' % fmt(v))
header.append('};\n')
header.append('static const int32_t recip_lut[1025] = {')
for v in recip:
    header.append('    %d,' % v)
header.append('};\n')
header.append('#endif')

open(FPREFIX + '/fp_trig_lut_1024.h','w').write('\n'.join(header))
print('Wrote', FPREFIX + '/fp_trig_lut_1024.h')
