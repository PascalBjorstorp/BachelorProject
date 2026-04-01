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
inline constexpr double kCpuSampleHz = 400.0;
inline constexpr double kGpuSampleHz = 200.0;
inline constexpr double kShortCsvLogHz = 200;
inline constexpr double kLongCsvLogHz = 1.0;
inline constexpr double kPrintHz = 1.0;
inline constexpr double kRollingWindowLongSec = 1.0;
inline constexpr double kRollingWindowShortSec = 0.005;
}  // namespace f1tenth_localization::system_monitor_config
