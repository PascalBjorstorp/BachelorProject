# Fixed-point sizing — empirical justification (per variable)

Aggregated over **4** independent recorded runs (`input_root`, `FPGA_ROS2`, `FPGA_UDP`, `MPC_10Laps`); **14,550,778,239** instrumented samples.

Every configurable width and guard in `fp_types_hls.hpp` is sized below its algebraic worst case deliberately. The tables below give, per variable, the evidence: the worst case is provably never approached, the behaviour is consistent across independent bags, and the chosen width keeps a safety margin over the worst case actually observed.

### Product widths -> guard bits (`*_GUARD`)

Policy: **product width = observed max + 1**. `chosen` is the production width; if it differs from policy, the table states why.

| variable / site | policy | chosen | obs max | margin | why chosen | `input_root` | `FPGA_ROS2` | `FPGA_UDP` | `MPC_10Laps` | samples |
|---|---|---|---|---|---|---|---|---|---|---|
| `QP_MUL    fp_QP_mul_t` | 51+1=52 | 52 | 51 | 1 | - | 51 | 51 | 51 | 51 | 486,312,028 |
| `P_QP_MUL  fp_P_QP_mul_t` | 59+1=60 | 60 | 59 | 1 | - | 59 | 58 | 58 | 59 | 2,818,407,480 |
| `MG_QP_MUL fp_MG_QP_mul_t` | 52+1=53 | 53 | 52 | 1 | - | 52 | 51 | 51 | 51 | 767,453,680 |
| `MG_K_MUL  fp_MG_K_mul_t` | 58+1=59 | 59 | 58 | 1 | - | 58 | 58 | 58 | 58 | 562,358,300 |
| `K_QP_MUL  fp_K_QP_mul_t` | 45+1=46 | 46 | 45 | 1 | - | 45 | 44 | 44 | 44 | 105,855,680 |
| `FN_MUL    fp_fn_accum_t` | 44+1=45 | 45 | 44 | 1 | - | 44 | 44 | 44 | 44 | 317,747,040 |

### Accumulator widths (`fp_sum*_*`)

Policy: **accumulator width = observed max + 1**. If a row still shares a typedef with another live use, the reason is shown in `why chosen`.

| variable / site | policy | chosen | obs max | margin | why chosen | `input_root` | `FPGA_ROS2` | `FPGA_UDP` | `MPC_10Laps` | samples |
|---|---|---|---|---|---|---|---|---|---|---|
| `SUM2_QP_RAW` | 31+1=32 | 32 | 31 | 1 | - | 31 | 31 | 30 | 30 | 222,669,690 |
| `SUM4_QP_RAW` | 25+1=26 | 26 | 25 | 1 | - | 25 | 25 | 25 | 25 | 20,368,400 |
| `SUM8_QP_RAW` | 24+1=25 | 25 | 24 | 1 | - | 24 | 23 | 23 | 24 | 12,221,040 |
| `SUM6_QP_tree` | 44+1=45 | 45 | 44 | 1 | - | 44 | 43 | 43 | 44 | 39,695,880 |
| `SUM6_QP_ACC (true)` | 43+1=44 | 45 | 43 | 2 | QP shared with QP_ITEM | 43 | 43 | 43 | 43 | 39,695,880 |
| `SUM2_P_RAW` | 39+1=40 | 40 | 39 | 1 | - | 39 | 39 | 39 | 39 | 186,570,636 |
| `SUM6_P_QP` | 58+1=59 | 59 | 58 | 1 | - | 58 | 57 | 57 | 58 | 251,407,240 |
| `SUM2_P_QP` | 51+1=52 | 52 | 51 | 1 | - | 51 | 50 | 50 | 51 | 59,543,820 |
| `SUM4_P_QP` | 50+1=51 | 51 | 50 | 1 | - | 50 | 50 | 50 | 50 | 59,543,820 |
| `SUM2_P_MIX` | 59+1=60 | 60 | 59 | 1 | - | 59 | 58 | 58 | 59 | 714,525,840 |
| `SUM4_P_MIX` | 57+1=58 | 58 | 57 | 1 | - | 57 | 56 | 57 | 57 | 357,262,920 |
| `SUM8_P_MIX` | 56+1=57 | 57 | 56 | 1 | - | 55 | 55 | 55 | 56 | 39,695,880 |
| `SUM8_P_MIX_pupdate` | 57+1=58 | 58 | 57 | 1 | - | 57 | 57 | 57 | 57 | 138,935,580 |
| `SUM2_MG_RAW` | 33+1=34 | 34 | 33 | 1 | - | 32 | 32 | 32 | 33 | 13,231,960 |
| `SUM6_MG_QP` | 52+1=53 | 53 | 52 | 1 | - | 52 | 51 | 51 | 51 | 79,391,760 |
| `SUM2_MG_QP` | 45+1=46 | 46 | 45 | 1 | - | 45 | 45 | 45 | 45 | 13,231,960 |
| `SUM4_MG_QP` | 44+1=45 | 45 | 44 | 1 | - | 44 | 44 | 44 | 44 | 13,231,960 |
| `SUM2_QP_MG` | 44+1=45 | 45 | 44 | 1 | - | 44 | 44 | 44 | 44 | 119,087,640 |
| `SUM2_MG_K` | 48+1=49 | 49 | 48 | 1 | - | 48 | 48 | 47 | 48 | 112,471,660 |
| `SUM2_K_QP` | 44+1=45 | 45 | 44 | 1 | - | 44 | 44 | 44 | 44 | 52,927,840 |
| `SUM4_K_QP` | 44+1=45 | 45 | 44 | 1 | - | 44 | 44 | 44 | 44 | 26,463,920 |
| `SUM8_K_QP` | 43+1=44 | 44 | 43 | 1 | - | 43 | 43 | 43 | 43 | 13,231,960 |

### Single cast-product into each sum type

Policy: **single cast-product width = observed max + 1**. ITEM shares the same typedef as the matching SUM row, so `chosen` is the common production width for that pair.

| variable / site | policy | chosen | obs max | margin | why chosen | `input_root` | `FPGA_ROS2` | `FPGA_UDP` | `MPC_10Laps` | samples |
|---|---|---|---|---|---|---|---|---|---|---|
| `QP_ITEM   single product` | 44+1=45 | 45 | 44 | 1 | - | 44 | 44 | 44 | 44 | 238,175,280 |
| `P_QP_ITEM single product` | 58+1=59 | 59 | 58 | 1 | - | 58 | 57 | 57 | 58 | 1,508,443,440 |
| `P_MIX_ITEM single product` | 59+1=60 | 60 | 59 | 1 | - | 59 | 58 | 58 | 59 | 1,429,051,680 |
| `MG_QP_ITEM single product` | 52+1=53 | 53 | 52 | 1 | - | 52 | 51 | 51 | 51 | 476,350,560 |
| `K_QP_ITEM single product` | 45+1=46 | 46 | 45 | 1 | - | 45 | 44 | 44 | 44 | 105,855,680 |

### Stored family widths — full `W = 1 + INT + FRAC` decomposition

For each family: **chosen** is the production format. **INT need** = integer bits the data actually required (dynamic-range high end); **INT have** = `WIDTH-FRAC-1`. **FRAC used** = deepest fractional bit ever set (`FRAC - dead`); **dead** = low fractional bits no nonzero sample ever set (resolution the format provides but the data never needs). **range** = max / smallest-nonzero magnitude in real units. Margins must be >= 0 (WARN flags overflow risk).

| family | chosen W=1+INT+FRAC | INT need | INT have | INT slack | FRAC | FRAC used | dead | min |x| | max |x| | nz samples | note |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `QP_STORE` | 32 = 1+13+18 | 13 | 13 | 0 | 18 | 18 | 0 | 3.815e-06 | 6.695e+03 | 612,752,161 |  |
| `FN_STORE` | 26 = 1+8+17 | 8 | 8 | 0 | 17 | 17 | 0 | 7.629e-06 | 1.921e+02 | 1,247,257,364 |  |
| `P_STORE` | 40 = 1+21+18 | 21 | 21 | 0 | 18 | 18 | 0 | 3.815e-06 | 1.320e+06 | 754,429,847 |  |
| `MG_STORE` | 34 = 1+15+18 | 15 | 15 | 0 | 18 | 18 | 0 | 3.815e-06 | 1.957e+04 | 217,001,786 |  |
| `K_STORE` | 26 = 1+7+18 | 7 | 7 | 0 | 18 | 18 | 0 | 3.815e-06 | 1.157e+02 | 116,073,086 |  |

**Reading it for the report.** *INT need < INT have* proves the format cannot overflow on the recorded operating envelope, so the worst-case integer span is unnecessary. *dead > 0* means the bottom `dead` fractional bits are never exercised by any sample across any bag, so `FRAC` (hence `WIDTH`) carries resolution the physical data does not contain — direct evidence that a smaller `FRAC_BITS` is lossless for this data. *INT need* and *FRAC used* together give the minimal correct `WIDTH = 1 + INT_need + FRAC_used`; the chosen width's surplus over that is the engineering margin.

> Reproduce: `tools/mpc_replay/run_width_report.sh` (rebuilds the probe with wide measurement typedefs, re-runs every bag under `tools/input/`).
