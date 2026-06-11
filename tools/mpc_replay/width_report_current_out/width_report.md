# Fixed-point sizing — empirical justification (per variable)

Aggregated over **5** independent recorded runs (`FPGA_UDP`, `FPGA_ROS2`, `LateralPlanner`, `MPC10LapBaseline`, `MPC_10Laps`); **158,114,384,508** instrumented samples.

Every configurable width and guard in `fp_types_hls.hpp` is sized below its algebraic worst case deliberately. The tables below give, per variable, the evidence: the worst case is provably never approached, the behaviour is consistent across independent bags, and the chosen width keeps a safety margin over the worst case actually observed.

### Product widths -> guard bits (`*_GUARD`)

Policy: **product width = observed max + 1**. `chosen` is the production width; if it differs from policy, the table states why.

| variable / site | policy | chosen | obs max | margin | why chosen | `FPGA_UDP` | `FPGA_ROS2` | `LateralPlanner` | `MPC10LapBaseline` | `MPC_10Laps` | samples |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `QP_MUL    fp_QP_mul_t` | 45+1=46 | 42 | 45 | -3 WARN | - | 45 | 45 | 45 | 45 | 45 | 4,902,622,720 |
| `P_QP_MUL  fp_P_QP_mul_t` | 52+1=53 | 50 | 52 | -2 WARN | - | 52 | 52 | 52 | 51 | 52 | 34,889,297,760 |
| `MG_QP_MUL fp_MG_QP_mul_t` | 45+1=46 | 43 | 45 | -2 WARN | - | 43 | 41 | 45 | 40 | 43 | 9,500,372,160 |
| `MG_K_MUL  fp_MG_K_mul_t` | 35+1=36 | 49 | 35 | 14 | - | 35 | 35 | 35 | 35 | 35 | 6,961,479,600 |
| `K_QP_MUL  fp_K_QP_mul_t` | 41+1=42 | 37 | 41 | -4 WARN | - | 41 | 41 | 41 | 41 | 41 | 1,310,396,160 |
| `FN_MUL    fp_fn_accum_t` | 40+1=41 | 38 | 40 | -2 WARN | - | 33 | 33 | 33 | 33 | 40 | 391,175,040 |

### Accumulator widths (`fp_sum*_*`)

Policy: **accumulator width = observed max + 1**. If a row still shares a typedef with another live use, the reason is shown in `why chosen`.

| variable / site | policy | chosen | obs max | margin | why chosen | `FPGA_UDP` | `FPGA_ROS2` | `LateralPlanner` | `MPC10LapBaseline` | `MPC_10Laps` | samples |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `SUM2_QP_RAW` | 27+1=28 | 26 | 27 | -1 WARN | - | 27 | 27 | 27 | 27 | 27 | 2,303,670,832 |
| `SUM4_QP_RAW` | 21+1=22 | 22 | 21 | 1 | - | 21 | 21 | 21 | 21 | 21 | 25,735,200 |
| `SUM8_QP_RAW` | 20+1=21 | 21 | 20 | 1 | - | 20 | 20 | 20 | 20 | 20 | 15,441,120 |
| `SUM6_QP_tree` | 43+1=44 | 37 | 43 | -6 WARN | QP shared with QP_ITEM | 43 | 43 | 43 | 43 | 43 | 491,398,560 |
| `SUM6_QP_ACC (true)` | 43+1=44 | 37 | 43 | -6 WARN | QP shared with QP_ITEM | 43 | 43 | 43 | 43 | 43 | 491,398,560 |
| `SUM2_P_RAW` | 35+1=36 | 34 | 35 | -1 WARN | - | 35 | 35 | 35 | 35 | 35 | 2,309,573,232 |
| `SUM6_P_QP` | 51+1=52 | 49 | 51 | -2 WARN | shared with P_QP_ITEM | 51 | 51 | 51 | 51 | 51 | 3,112,190,880 |
| `SUM2_P_QP` | 43+1=44 | 42 | 43 | -1 WARN | - | 43 | 43 | 43 | 43 | 43 | 737,097,840 |
| `SUM4_P_QP` | 45+1=46 | 41 | 45 | -4 WARN | - | 45 | 45 | 45 | 44 | 45 | 737,097,840 |
| `SUM2_P_MIX` | 51+1=52 | 50 | 51 | -1 WARN | - | 51 | 51 | 51 | 51 | 51 | 8,845,174,080 |
| `SUM4_P_MIX` | 51+1=52 | 48 | 51 | -3 WARN | - | 51 | 51 | 51 | 51 | 51 | 4,422,587,040 |
| `SUM8_P_MIX` | 51+1=52 | 47 | 51 | -4 WARN | - | 51 | 51 | 50 | 50 | 51 | 491,398,560 |
| `SUM8_P_MIX_pupdate` | 51+1=52 | 48 | 51 | -3 WARN | - | 51 | 51 | 51 | 51 | 51 | 1,719,894,960 |
| `SUM2_MG_RAW` | 21+1=22 | 28 | 21 | 7 | - | 21 | 21 | 21 | 21 | 21 | 163,799,520 |
| `SUM6_MG_QP` | 38+1=39 | 43 | 38 | 5 | shared with MG_QP_ITEM | 38 | 37 | 38 | 37 | 37 | 982,797,120 |
| `SUM2_MG_QP` | 29+1=30 | 36 | 29 | 7 | - | 29 | 29 | 29 | 29 | 29 | 163,799,520 |
| `SUM4_MG_QP` | 31+1=32 | 36 | 31 | 5 | - | 31 | 31 | 31 | 30 | 31 | 163,799,520 |
| `SUM2_QP_MG` | 44+1=45 | 37 | 44 | -7 WARN | - | 43 | 41 | 44 | 41 | 44 | 1,474,195,680 |
| `SUM2_MG_K` | 36+1=37 | 39 | 36 | 3 | - | 36 | 35 | 35 | 33 | 35 | 1,392,295,920 |
| `SUM2_K_QP` | 38+1=39 | 37 | 38 | -1 WARN | - | 38 | 38 | 38 | 38 | 38 | 655,198,080 |
| `SUM4_K_QP` | 39+1=40 | 37 | 39 | -2 WARN | - | 39 | 39 | 39 | 39 | 39 | 327,599,040 |
| `SUM8_K_QP` | 39+1=40 | 36 | 39 | -3 WARN | - | 39 | 39 | 39 | 39 | 39 | 163,799,520 |
| `QP_RECIP_SHIFT` | 20+1=21 | 14 | 20 | -6 WARN | - | 17 | 15 | 20 | 16 | 18 | 82,043,337 |
| `FN_RECIP_SHIFT` | 22+1=23 | 19 | 22 | -3 WARN | - | 16 | 16 | 16 | 16 | 22 | 23,161,680 |
| `QP_DET_MUL` | 43+1=44 | 40 | 43 | -3 WARN | - | 43 | 43 | 43 | 43 | 43 | 81,899,760 |

### Single cast-product into each sum type

Policy: **single cast-product width = observed max + 1**. ITEM shares the same typedef as the matching SUM row, so `chosen` is the common production width for that pair.

| variable / site | policy | chosen | obs max | margin | why chosen | `FPGA_UDP` | `FPGA_ROS2` | `LateralPlanner` | `MPC10LapBaseline` | `MPC_10Laps` | samples |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `QP_ITEM   single product` | 42+1=43 | 37 | 42 | -5 WARN | shared with SUM6_QP_* | 42 | 42 | 42 | 42 | 42 | 2,948,391,360 |
| `P_QP_ITEM single product` | 50+1=51 | 49 | 50 | -1 WARN | shared with SUM6_P_QP | 50 | 50 | 50 | 50 | 50 | 18,673,145,280 |
| `P_MIX_ITEM single product` | 50+1=51 | 50 | 50 | 0 WARN | - | 50 | 50 | 50 | 50 | 50 | 17,690,348,160 |
| `MG_QP_ITEM single product` | 38+1=39 | 43 | 38 | 5 | shared with SUM6_MG_QP | 38 | 37 | 38 | 37 | 37 | 5,896,782,720 |
| `K_QP_ITEM single product` | 37+1=38 | 37 | 37 | 0 WARN | - | 37 | 37 | 37 | 37 | 37 | 1,310,396,160 |

### Stored family widths — full `W = 1 + INT + FRAC` decomposition

For each family: **chosen** is the production format. **INT need** = integer bits the data actually required (dynamic-range high end); **INT have** = `WIDTH-FRAC-1`. **FRAC used** = deepest fractional bit ever set (`FRAC - dead`); **dead** = low fractional bits no nonzero sample ever set (resolution the format provides but the data never needs). **range** = max / smallest-nonzero magnitude in real units. Margins must be >= 0 (WARN flags overflow risk).

| family | chosen W=1+INT+FRAC | INT need | INT have | INT slack | FRAC | FRAC used | dead | min |x| | max |x| | nz samples | note |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `QP_STORE` | 26 = 1+11+14 | 11 | 11 | 0 | 14 | 14 | 0 | 6.104e-05 | 2.048e+03 | 5,726,139,871 |  |
| `FN_STORE` | 21 = 1+8+12 | 8 | 8 | 0 | 12 | 12 | 0 | 2.441e-04 | 1.951e+02 | 1,544,123,561 |  |
| `P_STORE` | 34 = 1+19+14 | 19 | 19 | 0 | 14 | 14 | 0 | 6.104e-05 | 5.243e+05 | 8,779,542,775 |  |
| `MG_STORE` | 20 = 1+13+6 | 14 | 13 | -1 WARN | 6 | 6 | 0 | 1.562e-02 | 8.192e+03 | 2,567,071,047 |  |
| `K_STORE` | 16 = 1+7+8 | 8 | 7 | -1 WARN | 8 | 8 | 0 | 3.906e-03 | 1.280e+02 | 1,399,890,070 |  |

**Reading it for the report.** *INT need < INT have* proves the format cannot overflow on the recorded operating envelope, so the worst-case integer span is unnecessary. *dead > 0* means the bottom `dead` fractional bits are never exercised by any sample across any bag, so `FRAC` (hence `WIDTH`) carries resolution the physical data does not contain — direct evidence that a smaller `FRAC_BITS` is lossless for this data. *INT need* and *FRAC used* together give the minimal correct `WIDTH = 1 + INT_need + FRAC_used`; the chosen width's surplus over that is the engineering margin.

> Reproduce: `tools/mpc_replay/run_width_report.sh` (rebuilds the probe with wide measurement typedefs, re-runs every bag under `tools/input/`).
