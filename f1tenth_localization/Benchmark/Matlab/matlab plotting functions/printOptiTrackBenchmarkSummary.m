function summaryTable = printOptiTrackBenchmarkSummary(results)
%PRINTOPTITRACKBENCHMARKSUMMARY Print and return per-bag summary statistics.

bagName = string({results.bagName})';
duration_s = [results.durationS]';
nSamples = [results.nSamples]';
mean_x_error_m = [results.meanXError]';
std_x_error_m = [results.stdXError]';
mean_y_error_m = [results.meanYError]';
std_y_error_m = [results.stdYError]';
mean_xy_error_m = [results.meanXYError]';
std_xy_error_m = [results.stdXYError]';
rmse_xy_error_m = [results.rmseXYError]';
max_xy_error_m = [results.maxXYError]';
mean_abs_yaw_error_rad = [results.meanAbsYawError]';
std_yaw_error_rad = [results.stdYawError]';
rmse_yaw_error_rad = [results.rmseYawError]';
max_abs_yaw_error_rad = [results.maxAbsYawError]';
trimApplied = [results.trimApplied]';

summaryTable = table(bagName, duration_s, nSamples, trimApplied, ...
    mean_x_error_m, std_x_error_m, mean_y_error_m, std_y_error_m, ...
    mean_xy_error_m, std_xy_error_m, rmse_xy_error_m, max_xy_error_m, ...
    mean_abs_yaw_error_rad, std_yaw_error_rad, rmse_yaw_error_rad, ...
    max_abs_yaw_error_rad);

fprintf('\n=== OptiTrack Benchmark Summary ===\n');
disp(summaryTable);

fprintf('Aggregate mean x error       : %.4f m\n', mean(mean_x_error_m, 'omitnan'));
fprintf('Aggregate std x error        : %.4f m\n', std(mean_x_error_m, 0, 'omitnan'));
fprintf('Aggregate mean y error       : %.4f m\n', mean(mean_y_error_m, 'omitnan'));
fprintf('Aggregate std y error        : %.4f m\n', std(mean_y_error_m, 0, 'omitnan'));
fprintf('Aggregate mean XY RMSE       : %.4f m\n', mean(rmse_xy_error_m, 'omitnan'));
fprintf('Aggregate std XY RMSE        : %.4f m\n', std(rmse_xy_error_m, 0, 'omitnan'));
fprintf('Aggregate mean |yaw| error   : %.4f rad (%.3f deg)\n', ...
    mean(mean_abs_yaw_error_rad, 'omitnan'), ...
    rad2deg(mean(mean_abs_yaw_error_rad, 'omitnan')));
fprintf('Aggregate std |yaw| metric   : %.4f rad\n', std(mean_abs_yaw_error_rad, 0, 'omitnan'));
end
