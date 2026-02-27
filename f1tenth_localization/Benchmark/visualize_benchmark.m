%% visualize_benchmark.m — Localization Benchmark Visualisation
%  Usage: Run this script from MATLAB. A file picker will open if no CSV
%         path is hard-coded below.
%
%  Columns expected (from localization_benchmark.py):
%    time_s, gt_x, gt_y, gt_theta, gt_vx, gt_vy, gt_omega,
%    ekf_x, ekf_y, ekf_theta, ekf_cov_xx, ekf_cov_yy, ekf_cov_tt,
%    amcl_x, amcl_y, amcl_theta, amcl_cov_xx, amcl_cov_yy, amcl_cov_tt,
%    error_x, error_y, error_theta, error_euclidean,
%    amcl_error_x, amcl_error_y, amcl_error_theta, amcl_error_euclidean,
%    amcl_processing_ms, scan_to_amcl_ms, scan_to_ekf_ms

clear; clc; close all;

%% ── 1. Load CSV ──────────────────────────────────────────────────────
% Change this path or leave empty to use a file picker.
csv_path = '';

if isempty(csv_path)
    [file, folder] = uigetfile('*.csv', 'Select benchmark CSV');
    if isequal(file, 0), disp('Cancelled.'); return; end
    csv_path = fullfile(folder, file);
end

fprintf('Loading: %s\n', csv_path);
T = readtable(csv_path);

% Normalise time to start at zero
t = T.time_s - T.time_s(1);



%% ── 3. Position Error Over Time ─────────────────────────────────────
figure('Name', 'Trajectory', 'Position', [100 100 900 700]);

subplot(2,1,1); hold on; grid on;
valid = ~isnan(T.error_euclidean);
plot(t(valid), T.error_euclidean(valid), 'b-', 'LineWidth', 0.8);

% Rolling mean (1-second window @ 200 Hz)
window = min(200, sum(valid));
if window > 1
    err_smooth = movmean(T.error_euclidean(valid), window);
    plot(t(valid), err_smooth, 'r-', 'LineWidth', 1.5);
    legend('Instant', sprintf('Rolling mean (%ds)', round(window/200)), ...
        'Location', 'best');
else
    legend('Instant', 'Location', 'best');
end

xlabel('Time [s]'); ylabel('Position Error [m]');
title('EKF vs Ground Truth — Euclidean Error');

% Compute and display stats
mean_err = mean(T.error_euclidean(valid), 'omitnan');
max_err  = max(T.error_euclidean(valid));
rms_err  = sqrt(mean(T.error_euclidean(valid).^2, 'omitnan'));
text(0.02, 0.95, sprintf('Mean: %.3f m\nMax:  %.3f m\nRMS:  %.3f m', ...
    mean_err, max_err, rms_err), ...
    'Units', 'normalized', 'VerticalAlignment', 'top', ...
    'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', [.7 .7 .7]);

%% ── 4. Heading Error Over Time ──────────────────────────────────────
subplot(2,1,2); hold on; grid on;
valid_th = ~isnan(T.error_theta);
heading_err_deg = rad2deg(T.error_theta(valid_th));
plot(t(valid_th), heading_err_deg, 'b-', 'LineWidth', 0.8);
if window > 1
    th_smooth = movmean(heading_err_deg, min(window, numel(heading_err_deg)));
    plot(t(valid_th), th_smooth, 'r-', 'LineWidth', 1.5);
end
xlabel('Time [s]'); ylabel('Heading Error [deg]');
title('EKF vs Ground Truth — Heading Error');

%% ── 5. AMCL Processing Time ─────────────────────────────────────────
figure('Name', 'AMCL Timing', 'Position', [120 80 900 500]);

subplot(2,1,1); hold on; grid on;
has_timing = any(~isnan(T.amcl_processing_ms));
if has_timing
    valid_t = ~isnan(T.amcl_processing_ms);
    plot(t(valid_t), T.amcl_processing_ms(valid_t), 'b.', 'MarkerSize', 3);

    % Rolling mean (every 40 samples ~ 1 sec at PF rate)
    pf_window = min(40, sum(valid_t));
    if pf_window > 1
        timing_smooth = movmean(T.amcl_processing_ms(valid_t), pf_window);
        plot(t(valid_t), timing_smooth, 'r-', 'LineWidth', 1.5);
    end

    mean_ms = mean(T.amcl_processing_ms(valid_t), 'omitnan');
    max_ms  = max(T.amcl_processing_ms(valid_t));
    min_ms  = min(T.amcl_processing_ms(valid_t));
    xlabel('Time [s]'); ylabel('Processing Time [ms]');
    title('AMCL Particle Filter Processing Time');
    text(0.02, 0.95, sprintf('Mean: %.2f ms\nMax:  %.2f ms\nMin:  %.2f ms', ...
        mean_ms, max_ms, min_ms), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', [.7 .7 .7]);

    % Histogram inset
    subplot(2,1,2); hold on; grid on;
    histogram(T.amcl_processing_ms(valid_t), 50, 'FaceColor', [0.2 0.4 0.8], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.7);
    xline(mean_ms, 'r--', 'LineWidth', 1.5);
    xline(prctile(T.amcl_processing_ms(valid_t), 99), 'm--', 'LineWidth', 1.5);
    legend('Samples', sprintf('Mean (%.2f ms)', mean_ms), ...
        sprintf('P99 (%.2f ms)', prctile(T.amcl_processing_ms(valid_t), 99)), ...
        'Location', 'best');
    xlabel('Processing Time [ms]'); ylabel('Count');
    title('AMCL Processing Time Distribution');
else
    text(0.5, 0.5, 'No AMCL timing data', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', 'FontSize', 14);
end

%% ── 6. Pipeline Latency (Scan→AMCL and Scan→EKF) ───────────────────
figure('Name', 'Pipeline Latency', 'Position', [140 60 900 600]);

has_lat_amcl = any(~isnan(T.scan_to_amcl_ms));
has_lat_ekf  = ismember('scan_to_ekf_ms', T.Properties.VariableNames) && ...
               any(~isnan(T.scan_to_ekf_ms));

subplot(2,1,1); hold on; grid on;
if has_lat_amcl
    valid_l = ~isnan(T.scan_to_amcl_ms);
    plot(t(valid_l), T.scan_to_amcl_ms(valid_l), 'b.', 'MarkerSize', 3);
    lat_window = min(200, sum(valid_l));
    if lat_window > 1
        lat_smooth = movmean(T.scan_to_amcl_ms(valid_l), lat_window);
        plot(t(valid_l), lat_smooth, 'r-', 'LineWidth', 1.5);
    end
    mean_lat_amcl = mean(T.scan_to_amcl_ms(valid_l), 'omitnan');
    max_lat_amcl  = max(T.scan_to_amcl_ms(valid_l));
    ylabel('Latency [ms]');
    title('Scan → AMCL Pipeline Latency');
    text(0.02, 0.95, sprintf('Mean: %.2f ms\nMax:  %.2f ms', mean_lat_amcl, max_lat_amcl), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', [.7 .7 .7]);
else
    text(0.5, 0.5, 'No scan-to-AMCL data', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', 'FontSize', 14);
    mean_lat_amcl = NaN; max_lat_amcl = NaN;
end

subplot(2,1,2); hold on; grid on;
if has_lat_ekf
    valid_e = ~isnan(T.scan_to_ekf_ms);
    plot(t(valid_e), T.scan_to_ekf_ms(valid_e), 'b.', 'MarkerSize', 3);
    ekf_lat_window = min(200, sum(valid_e));
    if ekf_lat_window > 1
        ekf_lat_smooth = movmean(T.scan_to_ekf_ms(valid_e), ekf_lat_window);
        plot(t(valid_e), ekf_lat_smooth, 'r-', 'LineWidth', 1.5);
    end
    mean_lat_ekf = mean(T.scan_to_ekf_ms(valid_e), 'omitnan');
    max_lat_ekf  = max(T.scan_to_ekf_ms(valid_e));
    xlabel('Time [s]'); ylabel('Latency [ms]');
    title('Scan → EKF Full Pipeline Latency');
    text(0.02, 0.95, sprintf('Mean: %.2f ms\nMax:  %.2f ms', mean_lat_ekf, max_lat_ekf), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontSize', 9, 'BackgroundColor', 'w', 'EdgeColor', [.7 .7 .7]);
else
    text(0.5, 0.5, 'No scan-to-EKF data', 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', 'FontSize', 14);
    mean_lat_ekf = NaN; max_lat_ekf = NaN;
end

%% ── 7. P vs R Covariance Evolution ──────────────────────────────────
has_cov = any(~isnan(T.ekf_cov_xx));
has_amcl_cov = ismember('amcl_cov_xx', T.Properties.VariableNames) && ...
               any(~isnan(T.amcl_cov_xx));
if has_cov
    figure('Name', 'P vs R Covariance', 'Position', [200 10 900 600]);

    % ── σ_x ──
    subplot(3,1,1); hold on; grid on;
    plot(t, sqrt(T.ekf_cov_xx), 'b-', 'LineWidth', 0.8, 'DisplayName', 'P (EKF)');
    if has_amcl_cov
        va_c = ~isnan(T.amcl_cov_xx);
        plot(t(va_c), sqrt(T.amcl_cov_xx(va_c)), 'r.', 'MarkerSize', 3, ...
            'DisplayName', 'R (AMCL)');
    end
    legend('Location', 'best');
    ylabel('\sigma_x [m]');
    title('Covariance Evolution — P (EKF posterior) vs R (AMCL measurement)');

    % ── σ_y ──
    subplot(3,1,2); hold on; grid on;
    plot(t, sqrt(T.ekf_cov_yy), 'b-', 'LineWidth', 0.8, 'DisplayName', 'P (EKF)');
    if has_amcl_cov
        va_c2 = ~isnan(T.amcl_cov_yy);
        plot(t(va_c2), sqrt(T.amcl_cov_yy(va_c2)), 'r.', 'MarkerSize', 3, ...
            'DisplayName', 'R (AMCL)');
    end
    legend('Location', 'best');
    ylabel('\sigma_y [m]');

    % ── σ_θ ──
    subplot(3,1,3); hold on; grid on;
    plot(t, rad2deg(sqrt(T.ekf_cov_tt)), 'b-', 'LineWidth', 0.8, 'DisplayName', 'P (EKF)');
    if has_amcl_cov
        va_c3 = ~isnan(T.amcl_cov_tt);
        plot(t(va_c3), rad2deg(sqrt(T.amcl_cov_tt(va_c3))), 'r.', 'MarkerSize', 3, ...
            'DisplayName', 'R (AMCL)');
    end
    legend('Location', 'best');
    xlabel('Time [s]'); ylabel('\sigma_\theta [deg]');
end

%% ── 8. Summary table to console ────────────────────────────────────
% Compute bias values for summary
valid_ex = ~isnan(T.error_x);
valid_ey = ~isnan(T.error_y);
ekf_bx = mean(T.error_x(valid_ex), 'omitnan');
ekf_by = mean(T.error_y(valid_ey), 'omitnan');
has_amcl_err = ismember('amcl_error_x', T.Properties.VariableNames);

fprintf('\n===== Localization Benchmark Summary =====\n');
fprintf('  Samples:          %d\n', height(T));
fprintf('  Duration:         %.1f s\n', t(end));
fprintf('  Position error:   mean=%.4f m, max=%.4f m, RMS=%.4f m\n', ...
    mean_err, max_err, rms_err);
fprintf('  EKF bias:         X=%+.4f m, Y=%+.4f m, |bias|=%.4f m\n', ...
    ekf_bx, ekf_by, sqrt(ekf_bx^2 + ekf_by^2));
if has_amcl_err
    va = ~isnan(T.amcl_error_x) & ~isnan(T.amcl_error_y);
    if any(va)
        amcl_bx = mean(T.amcl_error_x(va), 'omitnan');
        amcl_by = mean(T.amcl_error_y(va), 'omitnan');
        fprintf('  AMCL bias:        X=%+.4f m, Y=%+.4f m, |bias|=%.4f m\n', ...
            amcl_bx, amcl_by, sqrt(amcl_bx^2 + amcl_by^2));
    end
end
if has_timing
    fprintf('  AMCL processing:  mean=%.2f ms, max=%.2f ms, min=%.2f ms\n', ...
        mean_ms, max_ms, min_ms);
    fprintf('  AMCL P99:         %.2f ms\n', ...
        prctile(T.amcl_processing_ms(valid_t), 99));
end
if has_lat_amcl && ~isnan(mean_lat_amcl)
    fprintf('  Scan→AMCL latency: mean=%.2f ms, max=%.2f ms\n', mean_lat_amcl, max_lat_amcl);
end
if has_lat_ekf && ~isnan(mean_lat_ekf)
    fprintf('  Scan→EKF  latency: mean=%.2f ms, max=%.2f ms\n', mean_lat_ekf, max_lat_ekf);
end
fprintf('==========================================\n');
