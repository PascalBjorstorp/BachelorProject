/* Emits the PRODUCTION fixed-point widths straight from fp_types_hls.hpp.
 * Built WITHOUT the wide measurement overrides, so the aggregator can score
 * observed maxima against the real typedef widths (the probe binary itself
 * is compiled with -DMPC_HLS_SUM*_WIDTH=64 and cannot report them). */
#include "fp_types_hls.hpp"
#include <cstdio>

int main() {
  /* key,production_width  — key matches the leading token of the probe
   * site name (see fp_width_probe.hpp table). */
  std::printf("QP_MUL,%d\n",   MPC_HLS_QP_WIDTH + MPC_HLS_QP_GUARD);
  std::printf("P_QP_MUL,%d\n", MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD);
  std::printf("MG_QP_MUL,%d\n", MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD);
  std::printf("MG_K_MUL,%d\n",  MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD);
  std::printf("K_QP_MUL,%d\n",  MPC_HLS_K_WIDTH + MPC_HLS_K_QP_GUARD);
  std::printf("FN_MUL,%d\n",    MPC_HLS_FN_WIDTH + MPC_HLS_FN_GUARD);
  std::printf("SUM2_QP_RAW,%d\n", MPC_HLS_SUM2_QP_RAW_WIDTH);
  std::printf("SUM4_QP_RAW,%d\n", MPC_HLS_SUM4_QP_RAW_WIDTH);
  std::printf("SUM8_QP_RAW,%d\n", MPC_HLS_SUM8_QP_RAW_WIDTH);
  std::printf("SUM6_QP_tree,%d\n", MPC_HLS_SUM6_QP_MUL_WIDTH);
  std::printf("SUM6_QP_ACC,%d\n",  MPC_HLS_SUM6_QP_MUL_WIDTH);
  std::printf("SUM2_P_RAW,%d\n",   MPC_HLS_SUM2_P_RAW_WIDTH);
  std::printf("SUM6_P_QP,%d\n",    MPC_HLS_SUM6_P_QP_WIDTH);
  std::printf("SUM2_P_QP,%d\n",    MPC_HLS_SUM2_P_QP_WIDTH);
  std::printf("SUM4_P_QP,%d\n",    MPC_HLS_SUM4_P_QP_WIDTH);
  std::printf("SUM2_P_MIX,%d\n",   MPC_HLS_SUM2_P_MIX_WIDTH);
  std::printf("SUM4_P_MIX,%d\n",   MPC_HLS_SUM4_P_MIX_WIDTH);
  std::printf("SUM8_P_MIX,%d\n",   MPC_HLS_SUM8_P_MIX_WIDTH);
  std::printf("SUM8_P_MIX_pupdate,%d\n", MPC_HLS_SUM8_P_MIX_PUP_WIDTH);
  std::printf("SUM2_MG_RAW,%d\n",  MPC_HLS_SUM2_MG_RAW_WIDTH);
  std::printf("SUM6_MG_QP,%d\n",   MPC_HLS_SUM6_MG_QP_WIDTH);
  std::printf("SUM2_MG_QP,%d\n",   MPC_HLS_SUM2_MG_QP_WIDTH);
  std::printf("SUM4_MG_QP,%d\n",   MPC_HLS_SUM4_MG_QP_WIDTH);
  std::printf("SUM2_QP_MG,%d\n",   MPC_HLS_SUM2_QP_MG_WIDTH);
  std::printf("SUM2_MG_K,%d\n",    MPC_HLS_SUM2_MG_K_WIDTH);
  std::printf("SUM2_K_QP,%d\n",    MPC_HLS_SUM2_K_QP_WIDTH);
  std::printf("SUM4_K_QP,%d\n",    MPC_HLS_SUM4_K_QP_WIDTH);
  std::printf("SUM8_K_QP,%d\n",    MPC_HLS_SUM8_K_QP_WIDTH);
  std::printf("QP_RECIP_SHIFT,%d\n", MPC_HLS_QP_RECIP_SHIFT_WIDTH);
  std::printf("FN_RECIP_SHIFT,%d\n", MPC_HLS_FN_RECIP_SHIFT_WIDTH);
  std::printf("QP_DET_MUL,%d\n",     MPC_HLS_QP_DET_MUL_WIDTH);
  std::printf("QP_ITEM,%d\n",   MPC_HLS_SUM6_QP_MUL_WIDTH);
  std::printf("P_QP_ITEM,%d\n", MPC_HLS_SUM6_P_QP_WIDTH);
  std::printf("P_MIX_ITEM,%d\n", MPC_HLS_P_MIX_ITEM_WIDTH);
  std::printf("MG_QP_ITEM,%d\n", MPC_HLS_SUM6_MG_QP_WIDTH);
  std::printf("K_QP_ITEM,%d\n",  MPC_HLS_K_QP_ITEM_WIDTH);
  std::printf("QP_STORE,%d\n",  MPC_HLS_QP_WIDTH);
  std::printf("FN_STORE,%d\n",  MPC_HLS_FN_WIDTH);
  std::printf("P_STORE,%d\n",   MPC_HLS_P_WIDTH);
  std::printf("MG_STORE,%d\n",  MPC_HLS_MG_WIDTH);
  std::printf("K_STORE,%d\n",   MPC_HLS_K_WIDTH);
  return 0;
}
