// Copyright (c) 2026 Pascal - MIT License
#pragma once

#include <cstddef>

namespace f1tenth_localization::system_monitor_config
{
inline constexpr const char * kOutputDir = "f1tenth_localization/Benchmark/Matlab/csv";
inline constexpr const char * kLongCsvFileName = "SystemUsageLong.csv";
inline constexpr const char * kShortCsvFileName = "SystemUsageShort.csv";
inline constexpr const char * kPerCoreCsvFileName = "SystemUsagePerCore.csv";
inline constexpr const char * kGpuCsvFileName = "SystemUsageGpu.csv";
inline constexpr const char * kNodeProcessCsvFileName = "SystemUsageNodeProcesses.csv";
inline constexpr double kCpuSampleHz = 100.0;
inline constexpr double kGpuSampleHz = 25.0;
inline constexpr double kNodeProcessSampleHz = 5.0;
inline constexpr double kNodeProcessDiscoveryHz = 1.0;
inline constexpr double kShortCsvLogHz = 50.0;
inline constexpr double kLongCsvLogHz = 1.0;
inline constexpr double kPrintHz = 1.0;
inline constexpr double kRollingWindowLongSec = 1.0;
inline constexpr double kRollingWindowShortSec = 0.1;
}  // namespace f1tenth_localization::system_monitor_config
