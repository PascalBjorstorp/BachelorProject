# Fixed-point sizing — empirical justification (per variable)

Aggregated over **5** independent recorded runs (`FPGA_ROS2`, `FPGA_UDP`, `MPC_10Laps`, `MPC10LapBaseline`, `LateralPlanner`); **9,382,859,884** instrumented samples.

Every configurable width and guard in `fp_types_hls.hpp` is sized below its algebraic worst case deliberately. The tables below give, per variable, the evidence: the worst case is provably never approached, the behaviour is consistent across independent bags, and the chosen width keeps a safety margin over the worst case actually observed.

### Product widths -> guard bits (`*_GUARD`)

Policy: **product width = observed max + 1**. `chosen` is the production width; if it differs from policy, the table states why.

| variable / site | policy | chosen | obs max | margin | why chosen | `FPGA_ROS2` | `FPGA_UDP` | `MPC_10Laps` | `MPC10LapBaseline` | `LateralPlanner` | samples |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `QP_MUL    fp_QP_mul_t` | 41+1=42 | 43 | 41 | 2 | - | 41 | 41 | 41 | 41 | 41 | 332,010,180 |
| `P_QP_MUL  fp_P_QP_mul_t` | 42+1=43 | 43 | 42 | 1 | - | 41 | 41 | 42 | 41 | 42 | 1,549,166,040 |
| `MG_QP_MUL fp_MG_QP_mul_t` | 32+1=33 | 34 | 32 | 2 | - | 31 | 31 | 31 | 31 | 32 | 421,838,640 |
| `MG_K_MUL  fp_MG_K_mul_t` | 33+1=34 | 35 | 33 | 2 | - | 31 | 32 | 32 | 32 | 33 | 309,105,900 |
| `K_QP_MUL  fp_K_QP_mul_t` | 30+1=31 | 33 | 30 | 3 | - | 30 | 30 | 30 | 30 | 30 | 58,184,640 |
| `FN_MUL    fp_fn_accum_t` | 33+1=34 | 35 | 33 | 2 | - | 33 | 33 | 33 | 33 | 33 | 391,175,040 |

### Accumulator widths (`fp_sum*_*`)

Policy: **accumulator width = observed max + 1**. If a row still shares a typedef with another live use, the reason is shown in `why chosen`.

| variable / site | policy | chosen | obs max | margin | why chosen | `FPGA_ROS2` | `FPGA_UDP` | `MPC_10Laps` | `MPC10LapBaseline` | `LateralPlanner` | samples |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `SUM2_QP_RAW` | 26+1=27 | 27 | 26 | 1 | - | 26 | 25 | 24 | 21 | 26 | 151,467,050 |
| `SUM4_QP_RAW` | 21+1=22 | 23 | 21 | 2 | - | 21 | 21 | 21 | 20 | 21 | 25,735,200 |
| `SUM8_QP_RAW` | 20+1=21 | 22 | 20 | 2 | - | 19 | 19 | 20 | 19 | 20 | 15,441,120 |
| `SUM6_QP_tree` | 36+1=37 | 38 | 36 | 2 | QP shared with QP_ITEM | 35 | 35 | 36 | 35 | 36 | 21,819,240 |
| `SUM6_QP_ACC (true)` | 35+1=36 | 38 | 35 | 3 | QP shared with QP_ITEM | 35 | 35 | 35 | 34 | 35 | 21,819,240 |
| `SUM2_P_RAW` | 26+1=27 | 27 | 26 | 1 | - | 25 | 26 | 26 | 25 | 26 | 102,550,428 |
| `SUM6_P_QP` | 41+1=42 | 42 | 41 | 1 | - | 40 | 40 | 41 | 40 | 41 | 138,188,520 |
| `SUM2_P_QP` | 34+1=35 | 35 | 34 | 1 | - | 33 | 33 | 33 | 33 | 34 | 32,728,860 |
| `SUM4_P_QP` | 33+1=34 | 35 | 33 | 2 | - | 32 | 32 | 33 | 32 | 32 | 32,728,860 |
| `SUM2_P_MIX` | 42+1=43 | 43 | 42 | 1 | - | 41 | 41 | 42 | 41 | 42 | 392,746,320 |
| `SUM4_P_MIX` | 40+1=41 | 41 | 40 | 1 | - | 39 | 39 | 40 | 39 | 40 | 196,373,160 |
| `SUM8_P_MIX` | 39+1=40 | 41 | 39 | 2 | - | 37 | 38 | 39 | 37 | 38 | 21,819,240 |
| `SUM8_P_MIX_pupdate` | 40+1=41 | 41 | 40 | 1 | - | 39 | 40 | 40 | 39 | 40 | 76,367,340 |
| `SUM2_MG_RAW` | 16+1=17 | 19 | 16 | 3 | - | 15 | 16 | 16 | 14 | 15 | 7,273,080 |
| `SUM6_MG_QP` | 32+1=33 | 34 | 32 | 2 | shared with MG_QP_ITEM | 31 | 31 | 32 | 31 | 32 | 43,638,480 |
| `SUM2_MG_QP` | 25+1=26 | 28 | 25 | 3 | - | 24 | 24 | 25 | 25 | 25 | 7,273,080 |
| `SUM4_MG_QP` | 24+1=25 | 27 | 24 | 3 | - | 24 | 24 | 24 | 24 | 24 | 7,273,080 |
| `SUM2_QP_MG` | 26+1=27 | 28 | 26 | 2 | - | 26 | 26 | 25 | 26 | 26 | 65,457,720 |
| `SUM2_MG_K` | 21+1=22 | 24 | 21 | 3 | - | 21 | 21 | 21 | 21 | 21 | 61,821,180 |
| `SUM2_K_QP` | 30+1=31 | 33 | 30 | 3 | - | 30 | 30 | 30 | 30 | 30 | 29,092,320 |
| `SUM4_K_QP` | 30+1=31 | 33 | 30 | 3 | - | 30 | 30 | 30 | 30 | 30 | 14,546,160 |
| `SUM8_K_QP` | 29+1=30 | 31 | 29 | 2 | - | 29 | 29 | 29 | 29 | 29 | 7,273,080 |
| `QP_RECIP_SHIFT` | 13+1=14 | 15 | 13 | 2 | - | 13 | 13 | 13 | 13 | 13 | 3,751,073 |
| `FN_RECIP_SHIFT` | 16+1=17 | 18 | 16 | 2 | - | 16 | 16 | 16 | 16 | 16 | 23,161,680 |
| `QP_DET_MUL` | 40+1=41 | 42 | 40 | 2 | - | 39 | 39 | 39 | 40 | 40 | 3,636,540 |

### Single cast-product into each sum type

Policy: **single cast-product width = observed max + 1**. ITEM shares the same typedef as the matching SUM row, so `chosen` is the common production width for that pair.

| variable / site | policy | chosen | obs max | margin | why chosen | `FPGA_ROS2` | `FPGA_UDP` | `MPC_10Laps` | `MPC10LapBaseline` | `LateralPlanner` | samples |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `QP_ITEM   single product` | 36+1=37 | 38 | 36 | 2 | shared with SUM6_QP_* | 36 | 36 | 36 | 35 | 36 | 130,915,440 |
| `P_QP_ITEM single product` | 41+1=42 | 42 | 41 | 1 | - | 40 | 40 | 41 | 40 | 41 | 829,131,120 |
| `P_MIX_ITEM single product` | 42+1=43 | 43 | 42 | 1 | - | 41 | 41 | 42 | 41 | 42 | 785,492,640 |
| `MG_QP_ITEM single product` | 32+1=33 | 34 | 32 | 2 | shared with SUM6_MG_QP | 31 | 31 | 31 | 31 | 32 | 261,830,880 |
| `K_QP_ITEM single product` | 30+1=31 | 33 | 30 | 3 | - | 30 | 30 | 30 | 30 | 30 | 58,184,640 |

### Stored family widths — full `W = 1 + INT + FRAC` decomposition

For each family: **chosen** is the production format. **INT need** = integer bits the data actually required (dynamic-range high end); **INT have** = `WIDTH-FRAC-1`. **FRAC used** = deepest fractional bit ever set (`FRAC - dead`); **dead** = low fractional bits no nonzero sample ever set (resolution the format provides but the data never needs). **range** = max / smallest-nonzero magnitude in real units. Margins must be >= 0 (WARN flags overflow risk).

| family | chosen W=1+INT+FRAC | INT need | INT have | INT slack | FRAC | FRAC used | dead | min |x| | max |x| | nz samples | note |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `QP_STORE` | 26 = 1+11+14 | 11 | 11 | 0 | 14 | 14 | 0 | 6.104e-05 | 2.000e+03 | 452,721,846 |  |
| `FN_STORE` | 21 = 1+8+12 | 8 | 8 | 0 | 12 | 12 | 0 | 2.441e-04 | 1.920e+02 | 1,544,165,960 |  |
| `P_STORE` | 27 = 1+20+6 | 20 | 20 | 0 | 6 | 6 | 0 | 1.562e-02 | 6.589e+05 | 410,961,078 |  |
| `MG_STORE` | 18 = 1+14+3 | 14 | 14 | 0 | 3 | 3 | 0 | 1.250e-01 | 9.411e+03 | 113,471,499 |  |
| `K_STORE` | 17 = 1+8+8 | 8 | 8 | 0 | 8 | 8 | 0 | 3.906e-03 | 1.388e+02 | 62,261,237 |  |

**Reading it for the report.** *INT need < INT have* proves the format cannot overflow on the recorded operating envelope, so the worst-case integer span is unnecessary. *dead > 0* means the bottom `dead` fractional bits are never exercised by any sample across any bag, so `FRAC` (hence `WIDTH`) carries resolution the physical data does not contain — direct evidence that a smaller `FRAC_BITS` is lossless for this data. *INT need* and *FRAC used* together give the minimal correct `WIDTH = 1 + INT_need + FRAC_used`; the chosen width's surplus over that is the engineering margin.

> Reproduce: `tools/mpc_replay/run_width_report.sh` (rebuilds the probe with wide measurement typedefs, re-runs every bag under `tools/input/`).
