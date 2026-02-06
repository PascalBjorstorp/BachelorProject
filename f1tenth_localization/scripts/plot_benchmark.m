%% AMCL Benchmark Visualization Script
% Compares performance across different AMCL configurations
% 
% Usage:
%   1. Place all CSV files in one folder
%   2. Set 'data_dir' to that folder path
%   3. Run this script
%
% CSV expected columns:
%   amcl_type, min_particles, max_particles, max_beams, amcl_cpu_percent,
%   scan_to_pose_latency_ms, pose_rate_hz, gpu_percent, etc.

clear; clc; close all;

%% Configuration
data_dir = '/home/pascal/Documents/BachelorProject/benchmark_results';  % Change to your benchmark folder
output_dir = fullfile(data_dir, 'plots');

% Create output directory
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% Load all CSV files
csv_files = dir(fullfile(data_dir, '*.csv'));
if isempty(csv_files)
    error('No CSV files found in %s', data_dir);
end

fprintf('Found %d benchmark files:\n', length(csv_files));
for i = 1:length(csv_files)
    fprintf('  %d. %s\n', i, csv_files(i).name);
end

%% Parse and combine data
all_data = [];
config_labels = {};

for i = 1:length(csv_files)
    filepath = fullfile(csv_files(i).folder, csv_files(i).name);
    data = readtable(filepath);
    
    % Skip empty files
    if height(data) < 2
        fprintf('  Skipping empty file: %s\n', csv_files(i).name);
        continue;
    end
    
    % Extract config from first row
    if ismember('amcl_type', data.Properties.VariableNames)
        amcl_type = string(data.amcl_type(1));
        min_p = data.min_particles(1);
        max_p = data.max_particles(1);
        beams = data.max_beams(1);
        config_label = sprintf('%s p:%d-%d b:%d', amcl_type, min_p, max_p, beams);
        config_label_multiline = sprintf('%s\np:%d-%d b:%d', amcl_type, min_p, max_p, beams);
    else
        % Fallback: extract from filename
        config_label = csv_files(i).name;
        config_label_multiline = config_label;
        amcl_type = "unknown";
        min_p = 0; max_p = 0; beams = 0;
    end
    
    % Create time vector (seconds from start)
    data.time_sec = (data.timestamp_sec - data.timestamp_sec(1)) + ...
                    (data.timestamp_nsec - data.timestamp_nsec(1)) / 1e9;
    
    % Store metadata
    data.config_id = repmat(i, height(data), 1);
    data.config_label = repmat({config_label}, height(data), 1);
    
    all_data = [all_data; data];
    config_labels{end+1} = config_label;
    
    fprintf('  Loaded: %s (%d samples)\n', config_label, height(data));
end

num_configs = length(config_labels);
colors = lines(num_configs);

%% Figure 1: CPU Usage Comparison
figure('Name', 'CPU Usage Comparison', 'Position', [100, 100, 1200, 800]);

subplot(2,2,1);
hold on;
for i = 1:num_configs
    idx = all_data.config_id == i;
    t = all_data.time_sec(idx);
    cpu = all_data.amcl_cpu_percent(idx);
    plot(t, cpu, 'Color', colors(i,:), 'DisplayName', config_labels{i});
end
xlabel('Time (s)');
ylabel('AMCL CPU (%)');
title('AMCL Process CPU Usage Over Time');
legend('Location', 'best');
grid on;

subplot(2,2,2);
cpu_means = zeros(1, num_configs);
cpu_stds = zeros(1, num_configs);
for i = 1:num_configs
    idx = all_data.config_id == i;
    cpu_means(i) = mean(all_data.amcl_cpu_percent(idx), 'omitnan');
    cpu_stds(i) = std(all_data.amcl_cpu_percent(idx), 'omitnan');
end
bar(cpu_means);
hold on;
errorbar(1:num_configs, cpu_means, cpu_stds, 'k.', 'LineWidth', 1.5);
set(gca, 'XTick', 1:num_configs, 'XTickLabel', config_labels, 'XTickLabelRotation', 45);
ylabel('AMCL CPU (%)');
title('Average AMCL CPU Usage');
grid on;

subplot(2,2,3);
hold on;
for i = 1:num_configs
    idx = all_data.config_id == i;
    t = all_data.time_sec(idx);
    cpu = all_data.system_cpu_percent(idx);
    plot(t, cpu, 'Color', colors(i,:), 'DisplayName', config_labels{i});
end
xlabel('Time (s)');
ylabel('System CPU (%)');
title('System CPU Usage Over Time');
legend('Location', 'best');
grid on;

subplot(2,2,4);
% Boxplot for AMCL CPU
cpu_data = [];
cpu_groups = [];
for i = 1:num_configs
    idx = all_data.config_id == i;
    cpu = all_data.amcl_cpu_percent(idx);
    cpu_data = [cpu_data; cpu];
    cpu_groups = [cpu_groups; repmat(i, length(cpu), 1)];
end
boxplot(cpu_data, cpu_groups, 'Labels', config_labels);
ylabel('AMCL CPU (%)');
title('AMCL CPU Distribution');
xtickangle(45);
grid on;

saveas(gcf, fullfile(output_dir, 'cpu_comparison.png'));

%% Figure 2: Latency Comparison
figure('Name', 'Latency Comparison', 'Position', [100, 100, 1200, 600]);

subplot(1,2,1);
hold on;
for i = 1:num_configs
    idx = all_data.config_id == i;
    t = all_data.time_sec(idx);
    lat = all_data.scan_to_pose_latency_ms(idx);
    plot(t, lat, 'Color', colors(i,:), 'DisplayName', config_labels{i});
end
xlabel('Time (s)');
ylabel('Latency (ms)');
title('Scan → Pose Latency Over Time');
legend('Location', 'best');
grid on;

subplot(1,2,2);
lat_means = zeros(1, num_configs);
lat_stds = zeros(1, num_configs);
for i = 1:num_configs
    idx = all_data.config_id == i;
    lat_means(i) = mean(all_data.scan_to_pose_latency_ms(idx), 'omitnan');
    lat_stds(i) = std(all_data.scan_to_pose_latency_ms(idx), 'omitnan');
end
bar(lat_means);
hold on;
errorbar(1:num_configs, lat_means, lat_stds, 'k.', 'LineWidth', 1.5);
set(gca, 'XTick', 1:num_configs, 'XTickLabel', config_labels, 'XTickLabelRotation', 45);
ylabel('Latency (ms)');
title('Average Latency');
grid on;

saveas(gcf, fullfile(output_dir, 'latency_comparison.png'));

%% Figure 3: Pose Rate (Throughput) Comparison
figure('Name', 'Pose Rate Comparison', 'Position', [100, 100, 1200, 600]);

subplot(1,2,1);
hold on;
for i = 1:num_configs
    idx = all_data.config_id == i;
    t = all_data.time_sec(idx);
    rate = all_data.pose_rate_hz(idx);
    plot(t, rate, 'Color', colors(i,:), 'DisplayName', config_labels{i});
end
xlabel('Time (s)');
ylabel('Pose Rate (Hz)');
title('AMCL Output Rate Over Time');
ylim([0, 50]);
legend('Location', 'best');
grid on;

subplot(1,2,2);
rate_means = zeros(1, num_configs);
for i = 1:num_configs
    idx = all_data.config_id == i;
    rate_means(i) = mean(all_data.pose_rate_hz(idx), 'omitnan');
end
bar(rate_means);
set(gca, 'XTick', 1:num_configs, 'XTickLabel', config_labels, 'XTickLabelRotation', 45);
ylabel('Pose Rate (Hz)');
title('Average Pose Output Rate');
ylim([0, 50]);
grid on;

saveas(gcf, fullfile(output_dir, 'pose_rate_comparison.png'));

%% Figure 4: GPU Usage (if available)
if ismember('gpu_percent', all_data.Properties.VariableNames)
    figure('Name', 'GPU Usage Comparison', 'Position', [100, 100, 1200, 600]);
    
    subplot(1,2,1);
    hold on;
    for i = 1:num_configs
        idx = all_data.config_id == i;
        t = all_data.time_sec(idx);
        gpu = all_data.gpu_percent(idx);
        plot(t, gpu, 'Color', colors(i,:), 'DisplayName', config_labels{i});
    end
    xlabel('Time (s)');
    ylabel('GPU (%)');
    title('GPU Usage Over Time');
    legend('Location', 'best');
    grid on;
    
    subplot(1,2,2);
    gpu_means = zeros(1, num_configs);
    for i = 1:num_configs
        idx = all_data.config_id == i;
        gpu_means(i) = mean(all_data.gpu_percent(idx), 'omitnan');
    end
    bar(gpu_means);
    set(gca, 'XTick', 1:num_configs, 'XTickLabel', config_labels, 'XTickLabelRotation', 45);
    ylabel('GPU (%)');
    title('Average GPU Usage');
    grid on;
    
    saveas(gcf, fullfile(output_dir, 'gpu_comparison.png'));
end

%% Figure 5: Per-Core CPU Heatmap (for all configs)
if ismember('cpu_core_0_percent', all_data.Properties.VariableNames)
    % Get number of cores from column names
    core_cols = startsWith(all_data.Properties.VariableNames, 'cpu_core_');
    num_cores = sum(core_cols);
    
    figure('Name', 'Per-Core CPU Heatmap', 'Position', [100, 100, 1200, 300 * num_configs]);
    
    for cfg = 1:num_configs
        subplot(num_configs, 1, cfg);
        
        idx = all_data.config_id == cfg;
        t = all_data.time_sec(idx);
        
        core_data = zeros(num_cores, sum(idx));
        for c = 0:num_cores-1
            col_name = sprintf('cpu_core_%d_percent', c);
            if ismember(col_name, all_data.Properties.VariableNames)
                core_data(c+1, :) = all_data.(col_name)(idx)';
            end
        end
        
        imagesc(t, 1:num_cores, core_data);
        colormap(hot);
        cb = colorbar;
        ylabel(cb, 'CPU %');
        caxis([0 100]);
        xlabel('Time (s)');
        ylabel('CPU Core');
        title(sprintf('Per-Core CPU: %s', strrep(config_labels{cfg}, newline, ' ')));
        set(gca, 'YDir', 'normal');
    end
    
    saveas(gcf, fullfile(output_dir, 'per_core_heatmap.png'));
end

%% Figure 6: Summary Table
figure('Name', 'Summary Table', 'Position', [100, 100, 900, 400]);

% Build summary data
summary_headers = {'Config', 'Avg CPU (%)', 'Peak CPU (%)', 'Avg Latency (ms)', 'Avg Pose Rate (Hz)', 'Avg GPU (%)'};
summary_values = cell(num_configs, 6);

for i = 1:num_configs
    idx = all_data.config_id == i;
    summary_values{i, 1} = strrep(config_labels{i}, newline, ' ');  % Remove newlines
    summary_values{i, 2} = sprintf('%.1f', mean(all_data.amcl_cpu_percent(idx), 'omitnan'));
    summary_values{i, 3} = sprintf('%.1f', max(all_data.amcl_cpu_percent(idx)));
    summary_values{i, 4} = sprintf('%.1f', mean(all_data.scan_to_pose_latency_ms(idx), 'omitnan'));
    summary_values{i, 5} = sprintf('%.1f', mean(all_data.pose_rate_hz(idx), 'omitnan'));
    
    if ismember('gpu_percent', all_data.Properties.VariableNames)
        summary_values{i, 6} = sprintf('%.1f', mean(all_data.gpu_percent(idx), 'omitnan'));
    else
        summary_values{i, 6} = 'N/A';
    end
end

% Display as text instead of uitable (compatible with saveas)
axis off;
text(0.5, 0.95, 'Benchmark Summary', 'FontSize', 16, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'Units', 'normalized');

% Create table as text
y_start = 0.85;
y_step = 0.12;
x_positions = linspace(0.02, 0.98, 6);

% Headers
for j = 1:6
    text(x_positions(j), y_start, summary_headers{j}, 'FontSize', 10, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'Units', 'normalized');
end

% Data rows
for i = 1:num_configs
    y_pos = y_start - i * y_step;
    for j = 1:6
        text(x_positions(j), y_pos, summary_values{i, j}, 'FontSize', 9, ...
             'HorizontalAlignment', 'center', 'Units', 'normalized');
    end
end

saveas(gcf, fullfile(output_dir, 'summary_table.png'));

%% Done
fprintf('\n=== Benchmark Analysis Complete ===\n');
fprintf('Plots saved to: %s\n', output_dir);
fprintf('Configs analyzed: %d\n', num_configs);
