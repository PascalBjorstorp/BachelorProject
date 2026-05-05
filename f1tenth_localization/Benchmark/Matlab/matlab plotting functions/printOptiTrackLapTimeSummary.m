function lapTimeTable = printOptiTrackLapTimeSummary(results)
%PRINTOPTITRACKLAPTIMESUMMARY Print and return per-bag lap-time statistics.

bagName = string({results.bagName})';
particle_count = [results.particleCount]';
nLaps = [results.nLaps]';
lap_mean_s = arrayfun(@(r) mean(r.lapDurationsS, 'omitnan'), results)';
lap_std_s = arrayfun(@(r) std(r.lapDurationsS, 0, 'omitnan'), results)';
lap_variance_s2 = arrayfun(@(r) var(r.lapDurationsS, 0, 'omitnan'), results)';
lap_min_s = arrayfun(@(r) min(r.lapDurationsS), results)';
lap_max_s = arrayfun(@(r) max(r.lapDurationsS), results)';
lap_range_s = lap_max_s - lap_min_s;

lapTimeTable = table(bagName, particle_count, nLaps, lap_mean_s, lap_std_s, ...
    lap_variance_s2, lap_min_s, lap_max_s, lap_range_s);

fprintf('\n=== OptiTrack Lap-Time Summary ===\n');
disp(lapTimeTable);
end
