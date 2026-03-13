#!/usr/bin/env python3
"""
Parameter sweep for trajectory optimization.
Runs multiple combinations and compares results.
"""
import subprocess
import re
import os
import shutil
import csv

PYTHON = "f1tenth_sim/.venv/bin/python3"
SCRIPT = "f1tenth_planning/scripts/optimize_trajectory.py"
MAP = "f1tenth_sim/maps/my_track_map.yaml"
TRAJ_DIR = "f1tenth_planning/trajectories"

# Parameter combinations to try
# (name, centerline_points, smooth_factor, waypoint_spacing, wall_clearance, opt_type)
CONFIGS = [
    # Baseline mintime
    ("mt_baseline",         300, 2.0, 0.15, 0.02, "mintime"),
    # More centerline points
    ("mt_pts_500",          500, 2.0, 0.15, 0.02, "mintime"),
    ("mt_pts_200",          200, 2.0, 0.15, 0.02, "mintime"),
    ("mt_pts_150",          150, 2.0, 0.15, 0.02, "mintime"),
    # Different smooth factors
    ("mt_smooth_0.5",       300, 0.5, 0.15, 0.02, "mintime"),
    ("mt_smooth_1.0",       300, 1.0, 0.15, 0.02, "mintime"),
    ("mt_smooth_5.0",       300, 5.0, 0.15, 0.02, "mintime"),
    ("mt_smooth_10.0",      300, 10.0, 0.15, 0.02, "mintime"),
    # Denser waypoints
    ("mt_wps_0.10",         300, 2.0, 0.10, 0.02, "mintime"),
    ("mt_wps_0.05",         300, 2.0, 0.05, 0.02, "mintime"),
    # More wall clearance
    ("mt_clearance_0.05",   300, 2.0, 0.15, 0.05, "mintime"),
    ("mt_clearance_0.10",   300, 2.0, 0.15, 0.10, "mintime"),
    # Combined: more points + lower smooth
    ("mt_pts500_sm1",       500, 1.0, 0.10, 0.02, "mintime"),
    # Combined: clearance + smooth
    ("mt_cl0.05_sm1",       300, 1.0, 0.15, 0.05, "mintime"),
    # High smooth + fine waypoints
    ("mt_sm5_wps0.10",      300, 5.0, 0.10, 0.02, "mintime"),
]


def parse_output(output):
    """Extract key metrics from optimizer output."""
    metrics = {}
    
    # Laptime from IPOPT mintime solver
    m = re.search(r'INFO: Laptime:\s*([\d.]+)s', output)
    if m:
        metrics['laptime_ipopt'] = float(m.group(1))
    
    # Laptime (estimated, from velocity profile)
    m = re.search(r'Estimated laptime:\s*([\d.]+)s', output)
    if m:
        metrics['laptime'] = float(m.group(1))
    
    # Curvature limit exceeded
    m = re.search(r'Curvature limit is exceeded:\s*([\d.]+)rad/m', output)
    if m:
        metrics['max_curv_warn'] = float(m.group(1))
    
    # Track length
    m = re.search(r'Track length:\s*([\d.]+)\s*m', output)
    if m:
        metrics['track_length'] = float(m.group(1))
    
    # Waypoints
    m = re.search(r'Waypoints:\s*(\d+)\s*$', output, re.MULTILINE)
    if m:
        metrics['waypoints'] = int(m.group(1))
    
    # Velocity range
    m = re.search(r'Velocity range:\s*\[([\d.]+),\s*([\d.]+)\]', output)
    if m:
        metrics['v_min'] = float(m.group(1))
        metrics['v_max'] = float(m.group(2))
    
    # Kappa range
    m = re.search(r'Kappa range:\s*\[([-\d.]+),\s*([-\d.]+)\]', output)
    if m:
        metrics['kappa_min'] = float(m.group(1))
        metrics['kappa_max'] = float(m.group(2))
    
    # Left/right wall distances
    m = re.search(r'Left wall:\s*\[([\d.]+),\s*([\d.]+)\]', output)
    if m:
        metrics['wall_left_min'] = float(m.group(1))
        metrics['wall_left_max'] = float(m.group(2))
    m = re.search(r'Right wall:\s*\[([\d.]+),\s*([\d.]+)\]', output)
    if m:
        metrics['wall_right_min'] = float(m.group(1))
        metrics['wall_right_max'] = float(m.group(2))
    
    # Heading error
    m = re.search(r'Heading deviates.*?up to\s*([\d.]+)\s*deg', output)
    if m:
        metrics['psi_err_deg'] = float(m.group(1))
    
    # Spline deviation
    m = re.search(r'Spline approximation: mean deviation\s*([\d.]+)m.*maximum deviation\s*([\d.]+)m', output)
    if m:
        metrics['spline_mean_dev'] = float(m.group(1))
        metrics['spline_max_dev'] = float(m.group(2))
    
    # Success
    metrics['success'] = 'SUCCESS' in output
    
    return metrics


def run_config(name, centerline_pts, smooth, waypoint_spacing, wall_clearance, opt_type):
    """Run one configuration and return metrics."""
    cmd = [
        PYTHON, SCRIPT,
        '--map', MAP,
        '--centerline-points', str(centerline_pts),
        '--smooth-factor', str(smooth),
        '--waypoint-spacing', str(waypoint_spacing),
        '--wall-clearance', str(wall_clearance),
        '--opt-type', opt_type,
        '--with-walls',
    ]
    
    print(f"\n{'='*60}")
    print(f"  Config: {name}")
    print(f"  pts={centerline_pts} smooth={smooth} ws={waypoint_spacing} "
          f"clear={wall_clearance} opt={opt_type}")
    print(f"{'='*60}")
    
    env = os.environ.copy()
    env['MPLBACKEND'] = 'Agg'
    
    result = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=300)
    output = result.stdout + result.stderr
    
    metrics = parse_output(output)
    metrics['name'] = name
    metrics['centerline_pts'] = centerline_pts
    metrics['smooth_factor'] = smooth
    metrics['waypoint_spacing'] = waypoint_spacing
    metrics['wall_clearance'] = wall_clearance
    metrics['opt_type'] = opt_type
    
    if metrics['success']:
        # Save trajectory with config name
        src = os.path.join(TRAJ_DIR, 'my_track_raceline.csv')
        dst = os.path.join(TRAJ_DIR, f'my_track_raceline_{name}.csv')
        if os.path.exists(src):
            shutil.copy2(src, dst)
        # Also save viz
        src_viz = os.path.join(TRAJ_DIR, 'my_track_raceline_viz.png')
        dst_viz = os.path.join(TRAJ_DIR, f'my_track_raceline_{name}_viz.png')
        if os.path.exists(src_viz):
            shutil.copy2(src_viz, dst_viz)
    else:
        # Print error
        print(f"  FAILED!")
        # Print last 20 lines
        lines = output.strip().split('\n')
        for line in lines[-20:]:
            print(f"    {line}")
    
    return metrics


def main():
    results = []
    
    for config in CONFIGS:
        try:
            m = run_config(*config)
            results.append(m)
        except subprocess.TimeoutExpired:
            print(f"  TIMEOUT: {config[0]}")
            results.append({'name': config[0], 'success': False, 'error': 'timeout'})
        except Exception as e:
            print(f"  ERROR: {config[0]}: {e}")
            results.append({'name': config[0], 'success': False, 'error': str(e)})
    
    # Print comparison table
    print(f"\n\n{'='*100}")
    print(f"  PARAMETER SWEEP RESULTS")
    print(f"{'='*100}")
    
    header = f"{'Config':<25} {'OK':>3} {'Lap(s)':>7} {'IPOPT':>7} {'WP':>4} {'Length':>7} " \
             f"{'V_max':>6} {'κ_max':>7} {'Wall_min':>9} {'Ψ_err':>6} {'Spl_dev':>8}"
    print(header)
    print('-' * 110)
    
    for m in results:
        if not m.get('success', False):
            print(f"{m['name']:<25} {'NO':>3}")
            continue
        
        wall_min = min(m.get('wall_left_min', 99), m.get('wall_right_min', 99))
        kappa_max = max(abs(m.get('kappa_min', 0)), abs(m.get('kappa_max', 0)))
        ipopt_str = f"{m['laptime_ipopt']:>7.2f}" if 'laptime_ipopt' in m else f"{'---':>7}"
        
        print(f"{m['name']:<25} "
              f"{'OK':>3} "
              f"{m.get('laptime', 0):>7.2f} "
              f"{ipopt_str} "
              f"{m.get('waypoints', 0):>4d} "
              f"{m.get('track_length', 0):>7.1f} "
              f"{m.get('v_max', 0):>6.2f} "
              f"{kappa_max:>7.3f} "
              f"{wall_min:>9.3f} "
              f"{m.get('psi_err_deg', 0):>6.1f} "
              f"{m.get('spline_max_dev', 0):>8.3f}")
    
    # Find best laptime
    successful = [m for m in results if m.get('success') and (m.get('laptime_ipopt') or m.get('laptime'))]
    if successful:
        def get_best_lap(m):
            return m.get('laptime_ipopt', m.get('laptime', 999))
        best = min(successful, key=get_best_lap)
        best_time = get_best_lap(best)
        print(f"\n  BEST LAPTIME: {best['name']} = {best_time:.2f}s")
        
        # Also find best that's safe (wall_min > 0.15m)
        safe = [m for m in successful 
                if min(m.get('wall_left_min', 99), m.get('wall_right_min', 99)) > 0.15]
        if safe:
            best_safe = min(safe, key=get_best_lap)
            safe_time = get_best_lap(best_safe)
            print(f"  BEST SAFE (wall>0.15m): {best_safe['name']} = {safe_time:.2f}s")
    
    # Save results to CSV
    csv_path = os.path.join(TRAJ_DIR, 'parameter_sweep_results.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'name', 'success', 'centerline_pts', 'smooth_factor', 'waypoint_spacing',
            'wall_clearance', 'opt_type', 'laptime', 'waypoints', 'track_length',
            'v_min', 'v_max', 'kappa_min', 'kappa_max',
            'wall_left_min', 'wall_right_min', 'psi_err_deg', 'spline_max_dev',
        ])
        writer.writeheader()
        for m in results:
            writer.writerow({k: m.get(k, '') for k in writer.fieldnames})
    print(f"\n  Results saved to: {csv_path}")


if __name__ == '__main__':
    main()
