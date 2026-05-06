function summaryTable = printOptiTrackBenchmarkSummary(results)
%PRINTOPTITRACKBENCHMARKSUMMARY Print and return per-bag lap-sample summary statistics.

bagName = string({results.bagName})';
sourceBagName = string({results.sourceBagName})';
particle_count = [results.particleCount]';
duration_s = [results.durationS]';
nSamples = [results.nSamples]';
nMetricSamples = [results.nMetricSamples]';
nExcludedSamples = [results.nExcludedSamples]';
nLaps = [results.nLaps]';
lap_mean_duration_s = arrayfun(@(r) mean(r.lapDurationsS, 'omitnan'), results)';
lap_std_duration_s = arrayfun(@(r) std(r.lapDurationsS, 0, 'omitnan'), results)';
optitrack_samples_raw = [results.optitrackSamplesRaw]';
optitrack_samples_valid = [results.optitrackSamplesValid]';
optitrack_samples_rejected = [results.optitrackSamplesRejected]';
n_yaw_isolated_samples_removed = [results.nYawIsolatedSamplesRemoved]';
n_yaw_outlier_laps_excluded = [results.nYawOutlierLapsExcluded]';
yaw_outlier_laps_excluded = strings(numel(results), 1);
for i = 1:numel(results)
    yaw_outlier_laps_excluded(i) = strjoin(string(results(i).yawOutlierLapsExcluded), ';');
end
mean_x_error_m = [results.meanXError]';
std_x_error_m = [results.stdXError]';
mean_y_error_m = [results.meanYError]';
std_y_error_m = [results.stdYError]';
mean_xy_error_m = [results.meanXYError]';
std_xy_error_m = [results.stdXYError]';
rmse_xy_error_m = [results.rmseXYError]';
std_rmse_xy_error_m = [results.stdRmseXYError]';
max_xy_error_m = [results.maxXYError]';
mean_abs_yaw_error_rad = [results.meanAbsYawError]';
std_mean_abs_yaw_error_rad = [results.stdYawError]';
rmse_yaw_error_rad = [results.rmseYawError]';
std_rmse_yaw_error_rad = [results.stdRmseYawError]';
max_abs_yaw_error_rad = [results.maxAbsYawError]';
trimApplied = [results.trimApplied]';

summaryTable = table(bagName, sourceBagName, particle_count, duration_s, nSamples, ...
    nMetricSamples, nExcludedSamples, ...
    nLaps, lap_mean_duration_s, lap_std_duration_s, trimApplied, ...
    optitrack_samples_raw, optitrack_samples_valid, optitrack_samples_rejected, ...
    n_yaw_isolated_samples_removed, ...
    n_yaw_outlier_laps_excluded, yaw_outlier_laps_excluded, ...
    mean_x_error_m, std_x_error_m, mean_y_error_m, std_y_error_m, ...
    mean_xy_error_m, std_xy_error_m, rmse_xy_error_m, std_rmse_xy_error_m, ...
    max_xy_error_m, mean_abs_yaw_error_rad, std_mean_abs_yaw_error_rad, ...
    rmse_yaw_error_rad, std_rmse_yaw_error_rad, max_abs_yaw_error_rad);

fprintf('\n=== OptiTrack Benchmark Summary ===\n');
disp(summaryTable);

fprintf('Aggregate lap mean x error       : %.4f m\n', mean(mean_x_error_m, 'omitnan'));
fprintf('Aggregate lap std x metric       : %.4f m\n', std(mean_x_error_m, 0, 'omitnan'));
fprintf('Aggregate lap mean y error       : %.4f m\n', mean(mean_y_error_m, 'omitnan'));
fprintf('Aggregate lap std y metric       : %.4f m\n', std(mean_y_error_m, 0, 'omitnan'));
fprintf('Aggregate lap mean XY error      : %.4f m\n', mean(mean_xy_error_m, 'omitnan'));
fprintf('Aggregate lap std XY metric      : %.4f m\n', std(mean_xy_error_m, 0, 'omitnan'));
fprintf('Aggregate lap mean XY RMSE       : %.4f m\n', mean(rmse_xy_error_m, 'omitnan'));
fprintf('Aggregate lap mean |yaw| error   : %.4f rad (%.3f deg)\n', ...
    mean(mean_abs_yaw_error_rad, 'omitnan'), ...
    rad2deg(mean(mean_abs_yaw_error_rad, 'omitnan')));
fprintf('Aggregate lap std |yaw| metric   : %.4f rad\n', std(mean_abs_yaw_error_rad, 0, 'omitnan'));
end
