#!/usr/bin/env python3
"""
Racing Line Generator for F1Tenth.

This script generates an optimized racing line from a track map file.
The algorithm:
1. Extracts track boundaries from the occupancy grid map
2. Computes the centerline between boundaries
3. Optimizes lateral deviation to minimize curvature
4. Generates a velocity profile based on vehicle dynamics

Output: CSV and NPZ files with trajectory waypoints including position,
heading, curvature, velocity, and acceleration at each point.

Usage:
    python3 generate_raceline.py --map <path_to_map.yaml> --output <output_dir>

Example:
    python3 generate_raceline.py --map f1tenth_sim/maps/Spielberg_map.yaml \
                                  --output f1tenth_planning/trajectories
"""

import argparse
import os
import sys
import csv
import numpy as np
import cv2
import yaml
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for server/headless
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
from scipy.spatial import cKDTree


# ============================================================================
# Vehicle Parameters (can be loaded from YAML)
# ============================================================================
class VehicleParams:
    """Vehicle dynamic parameters for velocity profiling."""
    
    def __init__(self, config_path: str = None):
        # Default values for F1Tenth (actual simulation values)
        self.friction_coeff = 1.0489   # Tire-track friction coefficient
        self.max_speed = 20.0          # Maximum velocity [m/s]
        self.max_accel = 9.51          # Maximum acceleration [m/s²]
        self.max_decel = 10.0          # Maximum deceleration [m/s²]
        self.safety_margin = 0.155     # Distance from walls [m]
        self.wheelbase = 0.3302        # Wheelbase [m]
        self.track_width = 0.31        # Vehicle width [m]
        
        if config_path and os.path.exists(config_path):
            self._load_config(config_path)
    
    def _load_config(self, path: str):
        """Load parameters from YAML file."""
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        
        if 'vehicle' in config:
            v = config['vehicle']
            self.friction_coeff = v.get('friction_coeff', self.friction_coeff)
            self.max_speed = v.get('max_speed', self.max_speed)
            # Handle both naming conventions
            self.max_accel = v.get('max_acceleration', v.get('max_accel', self.max_accel))
            self.max_decel = v.get('max_deceleration', v.get('max_decel', self.max_decel))
            self.wheelbase = v.get('wheelbase', self.wheelbase)
            self.track_width = v.get('width', v.get('track_width', self.track_width))
        
        if 'optimization' in config:
            p = config['optimization']
            self.safety_margin = p.get('safety_margin', self.safety_margin)
        elif 'planning' in config:
            p = config['planning']
            self.safety_margin = p.get('safety_margin', self.safety_margin)


# ============================================================================
# Track Processing
# ============================================================================
def load_map(map_yaml_path: str):
    """Load occupancy grid map from YAML configuration."""
    with open(map_yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    resolution = config['resolution']
    origin = np.array(config['origin'])
    
    image_path = config['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
    
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Could not load map image: {image_path}")
    
    return img, resolution, origin


def extract_boundaries(img: np.ndarray, resolution: float, origin: np.ndarray):
    """
    Extract track boundaries from occupancy grid.
    
    Uses contour detection with hierarchy analysis to find the outer and inner
    track walls. Handles maps where boundary lines have thickness.
    
    Returns:
        outer_world: Nx2 array of outer boundary in world coordinates
        inner_world: Nx2 array of inner boundary in world coordinates
    """
    # Threshold to get free space
    free_space = (img > 205).astype(np.uint8)
    
    # Find contours with hierarchy
    contours, hierarchy = cv2.findContours(
        free_space, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
    )
    
    if len(contours) < 2:
        raise ValueError("Could not find track boundaries in map")
    
    # Sort by area to identify hierarchy
    # Contour 0: Image boundary (largest)
    # Contour 1: Outer track edge
    # Contour 4 (innermost): Inner track edge
    areas = [(i, cv2.contourArea(c)) for i, c in enumerate(contours)]
    areas.sort(key=lambda x: x[1], reverse=True)
    
    # Find outer boundary (first child of image boundary)
    outer_idx = None
    for idx, _ in areas[1:]:
        parent = hierarchy[0][idx][3]
        if parent == areas[0][0]:
            outer_idx = idx
            break
    
    # Find inner boundary (innermost contour - no children)
    inner_idx = None
    for idx, _ in reversed(areas):
        if hierarchy[0][idx][2] == -1:  # No children
            inner_idx = idx
            break
    
    if outer_idx is None or inner_idx is None:
        # Fallback: use second and smallest contours
        outer_idx = areas[1][0]
        inner_idx = areas[-1][0]
    
    outer_contour = contours[outer_idx].squeeze()
    inner_contour = contours[inner_idx].squeeze()
    
    # Convert to world coordinates
    def pixel_to_world(points):
        world = np.zeros((len(points), 2))
        world[:, 0] = points[:, 0] * resolution + origin[0]
        world[:, 1] = (img.shape[0] - points[:, 1]) * resolution + origin[1]
        return world
    
    return pixel_to_world(outer_contour), pixel_to_world(inner_contour)


def compute_centerline(outer_world: np.ndarray, inner_world: np.ndarray, 
                       num_points: int = 1000):
    """
    Compute track centerline from boundaries.
    
    For each point on the outer boundary, finds the closest point on the inner
    boundary and computes the midpoint.
    
    Returns:
        centerline: Nx2 array of centerline coordinates
        normals: Nx2 array of normal vectors
        inner_tree: KD-tree for inner boundary
        outer_tree: KD-tree for outer boundary
    """
    # Build KD-trees for fast queries
    outer_tree = cKDTree(outer_world)
    inner_tree = cKDTree(inner_world)
    
    # Sample outer boundary
    step = max(1, len(outer_world) // num_points)
    outer_sampled = outer_world[::step]
    
    centerline_raw = []
    for outer_pt in outer_sampled:
        d_inner, idx = inner_tree.query(outer_pt)
        inner_pt = inner_world[idx]
        mid = (inner_pt + outer_pt) / 2
        centerline_raw.append(mid)
    
    centerline_raw = np.array(centerline_raw)
    
    # Remove duplicates
    diff = np.diff(centerline_raw, axis=0)
    dist = np.linalg.norm(diff, axis=1)
    mask = np.concatenate([[True], dist > 0.01])
    centerline_raw = centerline_raw[mask]
    
    # Smooth with periodic spline
    centerline_closed = np.vstack([centerline_raw, centerline_raw[0]])
    tck, u = splprep(
        [centerline_closed[:, 0], centerline_closed[:, 1]], 
        s=len(centerline_raw) * 0.01, 
        per=True
    )
    u_new = np.linspace(0, 1, num_points, endpoint=False)
    centerline = np.array(splev(u_new, tck)).T
    
    # Compute normals (perpendicular to tangent)
    tangents = np.zeros_like(centerline)
    tangents[:-1] = np.diff(centerline, axis=0)
    tangents[-1] = centerline[0] - centerline[-1]
    tangent_lengths = np.linalg.norm(tangents, axis=1, keepdims=True)
    tangents = tangents / np.maximum(tangent_lengths, 1e-6)
    
    normals = np.zeros_like(tangents)
    normals[:, 0] = -tangents[:, 1]
    normals[:, 1] = tangents[:, 0]
    
    return centerline, normals, inner_tree, outer_tree


# ============================================================================
# Racing Line Optimization
# ============================================================================
def optimize_racing_line(centerline: np.ndarray, normals: np.ndarray,
                         inner_tree: cKDTree, outer_tree: cKDTree,
                         params: VehicleParams, verbose: bool = True):
    """
    Optimize racing line using minimum curvature approach.
    
    Parameterizes lateral deviation from centerline and minimizes
    total curvature squared subject to boundary constraints.
    
    Returns:
        raceline: Nx2 array of optimized racing line coordinates
    """
    n = len(centerline)
    
    # Compute actual distances to boundaries for each centerline point
    widths_inner = np.array([inner_tree.query(pt)[0] for pt in centerline])
    widths_outer = np.array([outer_tree.query(pt)[0] for pt in centerline])
    
    if verbose:
        print(f"  Centerline points: {n}")
        print(f"  Distance to inner: {widths_inner.min():.2f} - {widths_inner.max():.2f} m")
        print(f"  Distance to outer: {widths_outer.min():.2f} - {widths_outer.max():.2f} m")
    
    # Optimization parameters
    safety = params.safety_margin
    curvature_weight = 1.0
    smoothness_weight = 0.5
    
    def get_bounds(i):
        """Get allowed alpha range for point i."""
        w_inner = max(widths_inner[i] - safety, 0.05)
        w_outer = max(widths_outer[i] - safety, 0.05)
        alpha_min = -w_inner / max(widths_inner[i], 0.1)
        alpha_max = w_outer / max(widths_outer[i], 0.1)
        return max(alpha_min, -0.9), min(alpha_max, 0.9)
    
    def path_from_alpha(alpha):
        """Convert alpha parameters to path coordinates."""
        path = np.zeros_like(centerline)
        for i in range(n):
            if alpha[i] < 0:
                displacement = alpha[i] * widths_inner[i]
            else:
                displacement = alpha[i] * widths_outer[i]
            path[i] = centerline[i] + displacement * normals[i]
        return path
    
    def compute_curvature(path):
        """Compute curvature at each path point."""
        dx = np.gradient(path[:, 0])
        dy = np.gradient(path[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        num = dx * ddy - dy * ddx
        denom = (dx**2 + dy**2)**1.5
        return num / np.maximum(denom, 1e-10)
    
    def objective(alpha):
        """Objective function: curvature + smoothness."""
        path = path_from_alpha(alpha)
        kappa = compute_curvature(path)
        
        curvature_cost = curvature_weight * np.sum(kappa**2)
        
        dalpha = np.diff(alpha)
        dalpha = np.append(dalpha, alpha[0] - alpha[-1])
        smoothness_cost = smoothness_weight * np.sum(dalpha**2)
        
        return curvature_cost + smoothness_cost
    
    # Build bounds and optimize
    bounds = [get_bounds(i) for i in range(n)]
    alpha0 = np.zeros(n)
    
    if verbose:
        print(f"  Optimizing with {n} variables...")
    
    result = minimize(
        objective,
        alpha0,
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 200, 'disp': False}
    )
    
    raceline = path_from_alpha(result.x)
    
    # Verify safety margins
    min_dist = float('inf')
    for pt in raceline:
        d_inner, _ = inner_tree.query(pt)
        d_outer, _ = outer_tree.query(pt)
        min_dist = min(min_dist, d_inner, d_outer)
    
    if verbose:
        kappa_opt = compute_curvature(raceline)
        kappa_center = compute_curvature(centerline)
        reduction = 100 * (1 - np.sum(kappa_opt**2) / np.sum(kappa_center**2))
        
        print(f"  Optimization {'converged' if result.success else 'did not converge'}")
        print(f"  Min wall distance: {min_dist:.3f} m")
        print(f"  Curvature reduction: {reduction:.1f}%")
    
    return raceline


# ============================================================================
# Wall Bounds Computation for MPC
# ============================================================================

def compute_track_bounds(raceline: np.ndarray, normals: np.ndarray,
                         inner_tree: cKDTree, outer_tree: cKDTree,
                         params: VehicleParams, verbose: bool = True):
    """
    Compute distance to left/right walls for each raceline point.
    
    This is used by the MPC controller to enforce track boundary constraints.
    The bounds represent the maximum lateral deviation allowed from the raceline
    to avoid hitting walls, accounting for vehicle width.
    
    Args:
        raceline: Racing line coordinates (N x 2)
        normals: Normal vectors at each point (N x 2)
        inner_tree: KD-tree of inner boundary
        outer_tree: KD-tree of outer boundary
        params: Vehicle parameters
        verbose: Print statistics
        
    Returns:
        left_bound: Distance to outer wall (negative normal direction) [m]
        right_bound: Distance to inner wall (positive normal direction) [m]
    """
    n = len(raceline)
    half_width = params.track_width / 2.0
    
    left_bound = np.zeros(n)
    right_bound = np.zeros(n)
    
    for i in range(n):
        # Get distances to both walls
        d_outer, _ = outer_tree.query(raceline[i])
        d_inner, _ = inner_tree.query(raceline[i])
        
        # Subtract vehicle half-width and safety margin to get available space
        # Left bound = distance to outer wall (can deviate left by this amount)
        left_bound[i] = max(d_outer - params.track_width / 2.0, 0.01)
        
        # Right bound = distance to inner wall (can deviate right by this amount)  
        right_bound[i] = max(d_inner - params.track_width / 2.0, 0.01)
    
    if verbose:
        print(f"  Track bounds analysis:")
        print(f"    Left bound (outer): {left_bound.min():.3f} - {left_bound.max():.3f} m")
        print(f"    Right bound (inner): {right_bound.min():.3f} - {right_bound.max():.3f} m")
        
        # Find tightest constraint points
        min_left_idx = np.argmin(left_bound)
        min_right_idx = np.argmin(right_bound)
        print(f"    Tightest left: {left_bound.min():.3f} m at waypoint {min_left_idx}")
        print(f"    Tightest right: {right_bound.min():.3f} m at waypoint {min_right_idx}")
        
        # Check if any constraint is too tight
        if np.any(left_bound < 0.05):
            print(f"    WARNING: Left bound < 5cm at {np.sum(left_bound < 0.05)} points")
        if np.any(right_bound < 0.05):
            print(f"    WARNING: Right bound < 5cm at {np.sum(right_bound < 0.05)} points")
    
    return left_bound, right_bound


# ============================================================================

def compute_velocity_profile(raceline: np.ndarray, params: VehicleParams,
                             verbose: bool = True, use_friction_circle: bool = True):
    """
    Generate velocity profile based on vehicle dynamics.
    
    Basic approach:
        1. Curvature-limited speed: v = sqrt(mu * g / |kappa|)
        2. Forward pass: acceleration limited
        3. Backward pass: braking limited
    
    With friction circle (use_friction_circle=True):
        The friction circle constraint limits TOTAL tire grip:
            sqrt(a_x^2 + a_y^2) <= mu * g
        
        Where a_y = v^2 * kappa (lateral/centripetal)
        
        This couples longitudinal and lateral acceleration:
        - In corners, we're using lateral grip, so less is available for accel/brake
        - On straights, we have full longitudinal grip
        
        The available longitudinal acceleration at each point is:
            a_x_available = sqrt((mu*g)^2 - (v^2 * kappa)^2)
    
    Returns:
        velocities: N array of velocities
        accelerations: N array of accelerations
        arc_length: N array of cumulative arc length
        curvature: N array of curvature values
        heading: N array of heading angles
    """
    n = len(raceline)
    G = 9.81
    MU = params.friction_coeff
    A_MAX_LONG = params.max_accel
    A_BRAKE_MAX = params.max_decel
    V_MAX = params.max_speed
    
    # Compute path properties
    dx = np.gradient(raceline[:, 0])
    dy = np.gradient(raceline[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # Curvature
    num = dx * ddy - dy * ddx
    denom = (dx**2 + dy**2)**1.5
    curvature = num / np.maximum(denom, 1e-10)
    
    # Heading
    heading = np.arctan2(dy, dx)
    
    # Arc length
    diff = np.diff(raceline, axis=0)
    segment_lengths = np.linalg.norm(diff, axis=1)
    arc_length = np.concatenate([[0], np.cumsum(segment_lengths)])
    
    # Maximum grip available (friction circle radius)
    max_total_accel = MU * G
    
    # Step 1: Curvature-limited velocity
    # At maximum cornering, a_lat = v^2 * kappa = mu * g
    kappa_abs = np.maximum(np.abs(curvature), 0.001)
    v_curvature = np.sqrt(max_total_accel / kappa_abs)
    v_curvature = np.minimum(v_curvature, V_MAX)
    
    velocities = v_curvature.copy()
    
    if use_friction_circle:
        # ================================================================
        # FRICTION CIRCLE VELOCITY PROFILING
        # ================================================================
        # The key insight: when cornering, lateral accel uses part of the
        # friction budget, leaving less for longitudinal (accel/brake).
        #
        # Friction circle: a_x^2 + a_y^2 <= (mu*g)^2
        # Rearranged: a_x_max = sqrt((mu*g)^2 - a_y^2)
        #           = sqrt((mu*g)^2 - (v^2 * kappa)^2)
        # ================================================================
        
        def get_available_accel(v, kappa):
            """Get available longitudinal acceleration given current cornering."""
            a_lateral = v**2 * abs(kappa)
            if a_lateral >= max_total_accel:
                return 0.0  # All grip used for cornering
            a_long_available = np.sqrt(max_total_accel**2 - a_lateral**2)
            return min(a_long_available, A_MAX_LONG)
        
        def get_available_decel(v, kappa):
            """Get available braking given current cornering."""
            a_lateral = v**2 * abs(kappa)
            if a_lateral >= max_total_accel:
                return 0.0  # All grip used for cornering
            a_long_available = np.sqrt(max_total_accel**2 - a_lateral**2)
            return min(a_long_available, A_BRAKE_MAX)
        
        # Step 2: Forward pass with friction circle
        # Can only accelerate with remaining grip after cornering
        for i in range(1, n):
            ds = arc_length[i] - arc_length[i-1]
            if ds > 0:
                # Use curvature at current position to estimate required lateral accel
                kappa_local = (kappa_abs[i-1] + kappa_abs[i]) / 2
                a_available = get_available_accel(velocities[i-1], kappa_local)
                
                # v_f^2 = v_i^2 + 2*a*ds
                v_max = np.sqrt(velocities[i-1]**2 + 2 * a_available * ds)
                velocities[i] = min(velocities[i], v_max)
        
        # Step 3: Backward pass with friction circle
        # Can only brake with remaining grip after cornering
        for i in range(n-2, -1, -1):
            ds = arc_length[i+1] - arc_length[i]
            if ds > 0:
                kappa_local = (kappa_abs[i] + kappa_abs[i+1]) / 2
                a_available = get_available_decel(velocities[i+1], kappa_local)
                
                v_max = np.sqrt(velocities[i+1]**2 + 2 * a_available * ds)
                velocities[i] = min(velocities[i], v_max)
        
        # Additional passes: enforce friction circle constraint
        # This is needed because forward/backward passes may still violate
        # the combined acceleration constraint
        for iteration in range(5):  # Multiple iterations for convergence
            changed = False
            
            # Enforce curvature limit on velocity
            for i in range(n):
                a_lateral = velocities[i]**2 * kappa_abs[i]
                if a_lateral > max_total_accel * 0.99:  # 1% margin
                    v_new = np.sqrt(0.99 * max_total_accel / max(kappa_abs[i], 0.001))
                    if v_new < velocities[i]:
                        velocities[i] = v_new
                        changed = True
            
            # Forward pass again with updated velocities
            for i in range(1, n):
                ds = arc_length[i] - arc_length[i-1]
                if ds > 0:
                    kappa_local = (kappa_abs[i-1] + kappa_abs[i]) / 2
                    a_available = get_available_accel(velocities[i-1], kappa_local)
                    v_max = np.sqrt(velocities[i-1]**2 + 2 * a_available * ds)
                    if v_max < velocities[i]:
                        velocities[i] = v_max
                        changed = True
            
            # Backward pass again
            for i in range(n-2, -1, -1):
                ds = arc_length[i+1] - arc_length[i]
                if ds > 0:
                    kappa_local = (kappa_abs[i] + kappa_abs[i+1]) / 2
                    a_available = get_available_decel(velocities[i+1], kappa_local)
                    v_max = np.sqrt(velocities[i+1]**2 + 2 * a_available * ds)
                    if v_max < velocities[i]:
                        velocities[i] = v_max
                        changed = True
            
            if not changed:
                break
    
    else:
        # Original simple approach (without friction circle coupling)
        # Step 2: Forward pass (acceleration limited)
        for i in range(1, n):
            ds = arc_length[i] - arc_length[i-1]
            if ds > 0:
                v_max = np.sqrt(velocities[i-1]**2 + 2 * A_MAX_LONG * ds)
                velocities[i] = min(velocities[i], v_max)
        
        # Step 3: Backward pass (braking limited)
        for i in range(n-2, -1, -1):
            ds = arc_length[i+1] - arc_length[i]
            if ds > 0:
                v_max = np.sqrt(velocities[i+1]**2 + 2 * A_BRAKE_MAX * ds)
                velocities[i] = min(velocities[i], v_max)
    
    # Compute actual accelerations from final velocity profile
    accelerations = np.zeros(n)
    for i in range(n - 1):
        ds = arc_length[i+1] - arc_length[i]
        if ds > 0:
            accelerations[i] = (velocities[i+1]**2 - velocities[i]**2) / (2 * ds)
    accelerations[-1] = accelerations[-2]
    accelerations = np.clip(accelerations, -A_BRAKE_MAX, A_MAX_LONG)
    
    # Compute combined accelerations for friction circle analysis
    a_lateral = velocities**2 * kappa_abs
    a_total = np.sqrt(accelerations**2 + a_lateral**2)
    
    if verbose:
        # Estimate lap time
        lap_time = 0
        for i in range(n - 1):
            ds = arc_length[i+1] - arc_length[i]
            avg_v = (velocities[i] + velocities[i+1]) / 2
            if avg_v > 0:
                lap_time += ds / avg_v
        
        print(f"  Speed range: {velocities.min():.2f} - {velocities.max():.2f} m/s")
        print(f"  Acceleration range: {accelerations.min():.2f} - {accelerations.max():.2f} m/s²")
        print(f"  Track length: {arc_length[-1]:.1f} m")
        print(f"  Estimated lap time: {lap_time:.2f} s")
        print(f"  Average speed: {arc_length[-1]/lap_time:.2f} m/s")
        
        # Friction circle analysis
        print(f"\n  --- Friction Circle Analysis ---")
        print(f"  Max grip (μg): {max_total_accel:.2f} m/s²")
        print(f"  Lateral accel range: {a_lateral.min():.2f} - {a_lateral.max():.2f} m/s²")
        
        # Lateral utilization (what matters for stability)
        lat_utilization = a_lateral / max_total_accel * 100
        print(f"  Lateral grip util:  {lat_utilization.min():.1f}% - {lat_utilization.max():.1f}%")
        
        # Combined (informational - can exceed 100% at transitions)
        grip_utilization = a_total / max_total_accel * 100
        print(f"  Combined (info):    {grip_utilization.min():.1f}% - {grip_utilization.max():.1f}%")
        
        # Points where lateral is near limit (what causes spins)
        lat_near_limit = np.sum(lat_utilization > 90)
        print(f"  Points at >90% lat: {lat_near_limit}/{n} ({100*lat_near_limit/n:.1f}%)")
        
        if use_friction_circle:
            print(f"  Mode: Friction circle ENABLED (coupled dynamics)")
        else:
            print(f"  Mode: Simple (decoupled lateral/longitudinal)")
    
    return velocities, accelerations, arc_length, curvature, heading


# ============================================================================
# Output Functions
# ============================================================================
def save_trajectory(output_path: str, raceline: np.ndarray, 
                    arc_length: np.ndarray, heading: np.ndarray,
                    curvature: np.ndarray, velocities: np.ndarray,
                    accelerations: np.ndarray, params: VehicleParams,
                    track_name: str, left_bound: np.ndarray = None,
                    right_bound: np.ndarray = None):
    """Save trajectory in CSV (TUM format) and NPZ formats.
    
    Args:
        left_bound: Distance to left/outer wall at each point [m]
        right_bound: Distance to right/inner wall at each point [m]
    """
    
    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # Save CSV
    csv_path = output_path.replace('.npz', '.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        if left_bound is not None and right_bound is not None:
            writer.writerow(['# s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2', 'left_bound_m', 'right_bound_m'])
            for i in range(len(raceline)):
                writer.writerow([
                    f"{arc_length[i]:.6f}",
                    f"{raceline[i, 0]:.6f}",
                    f"{raceline[i, 1]:.6f}",
                    f"{heading[i]:.6f}",
                    f"{curvature[i]:.6f}",
                    f"{velocities[i]:.6f}",
                    f"{accelerations[i]:.6f}",
                    f"{left_bound[i]:.6f}",
                    f"{right_bound[i]:.6f}"
                ])
        else:
            writer.writerow(['# s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'])
            for i in range(len(raceline)):
                writer.writerow([
                    f"{arc_length[i]:.6f}",
                    f"{raceline[i, 0]:.6f}",
                    f"{raceline[i, 1]:.6f}",
                    f"{heading[i]:.6f}",
                    f"{curvature[i]:.6f}",
                    f"{velocities[i]:.6f}",
                    f"{accelerations[i]:.6f}"
                ])
    
    # Save NPZ
    save_dict = dict(
        positions=raceline,
        arc_length=arc_length,
        heading=heading,
        curvature=curvature,
        velocities=velocities,
        accelerations=accelerations,
        track_name=track_name,
        friction_coeff=params.friction_coeff,
        max_accel=params.max_accel,
        max_decel=params.max_decel,
        max_speed=params.max_speed
    )
    
    # Add bounds if available
    if left_bound is not None:
        save_dict['left_bound'] = left_bound
    if right_bound is not None:
        save_dict['right_bound'] = right_bound
    
    np.savez(output_path, **save_dict)
    
    print(f"  Saved CSV: {csv_path}")
    print(f"  Saved NPZ: {output_path}")


def visualize_raceline(map_yaml_path: str, raceline: np.ndarray,
                       velocities: np.ndarray, output_path: str):
    """Create visualization of racing line on track map."""
    # Load map
    with open(map_yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    img_path = config['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml_path), img_path)
    
    img = cv2.imread(img_path)
    resolution = config['resolution']
    origin = np.array(config['origin'])
    img_height = img.shape[0]
    
    # Convert to pixel coordinates
    raceline_px = np.zeros_like(raceline, dtype=int)
    raceline_px[:, 0] = ((raceline[:, 0] - origin[0]) / resolution).astype(int)
    raceline_px[:, 1] = (img_height - (raceline[:, 1] - origin[1]) / resolution).astype(int)
    
    # Plot
    fig, ax = plt.subplots(figsize=(16, 16))
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    
    # Racing line colored by velocity
    scatter = ax.scatter(raceline_px[:, 0], raceline_px[:, 1],
                         c=velocities, cmap='RdYlGn', s=20, 
                         vmin=velocities.min(), vmax=velocities.max())
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.6)
    cbar.set_label('Velocity (m/s)', fontsize=12)
    
    # Mark start
    ax.scatter(raceline_px[0, 0], raceline_px[0, 1], 
               c='blue', s=300, marker='*', label='Start', zorder=10)
    
    # Direction arrows
    for i in range(0, len(raceline_px), 80):
        if i+1 < len(raceline_px):
            dx = raceline_px[i+1, 0] - raceline_px[i, 0]
            dy = raceline_px[i+1, 1] - raceline_px[i, 1]
            length = np.sqrt(dx**2 + dy**2)
            if length > 0:
                dx, dy = dx/length * 15, dy/length * 15
                ax.annotate('', xy=(raceline_px[i, 0]+dx, raceline_px[i, 1]+dy),
                           xytext=(raceline_px[i, 0], raceline_px[i, 1]),
                           arrowprops=dict(arrowstyle='->', color='white', lw=2))
    
    ax.legend(loc='upper right', fontsize=12)
    ax.set_title('Optimized Racing Line', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    
    print(f"  Saved visualization: {output_path}")


def visualize_friction_circle(velocities: np.ndarray, accelerations: np.ndarray,
                               curvature: np.ndarray, params: VehicleParams,
                               output_path: str):
    """
    Create friction circle visualization showing how grip is utilized.
    
    This shows a_x (longitudinal) vs a_y (lateral) for each point,
    with the friction circle limit overlaid.
    """
    G = 9.81
    max_grip = params.friction_coeff * G
    
    # Compute lateral accelerations
    a_lateral = velocities**2 * np.abs(curvature)
    a_longitudinal = accelerations
    
    # Create figure with 2 subplots
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    
    # === Subplot 1: Friction Circle (g-g diagram) ===
    ax1 = axes[0]
    
    # Draw friction circle
    theta = np.linspace(0, 2*np.pi, 100)
    circle_x = max_grip * np.cos(theta)
    circle_y = max_grip * np.sin(theta)
    ax1.plot(circle_x, circle_y, 'r--', linewidth=2, label=f'Friction limit (μg={max_grip:.1f} m/s²)')
    ax1.fill(circle_x, circle_y, color='red', alpha=0.1)
    
    # Plot actual accelerations
    scatter = ax1.scatter(a_lateral, a_longitudinal, c=velocities, cmap='plasma', 
                          s=5, alpha=0.6, label='Trajectory points')
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Velocity (m/s)')
    
    ax1.axhline(y=0, color='gray', linewidth=0.5)
    ax1.axvline(x=0, color='gray', linewidth=0.5)
    
    ax1.set_xlabel('Lateral Acceleration (m/s²)', fontsize=12)
    ax1.set_ylabel('Longitudinal Acceleration (m/s²)', fontsize=12)
    ax1.set_title('Friction Circle (G-G Diagram)', fontsize=14)
    ax1.set_xlim(-max_grip*1.2, max_grip*1.2)
    ax1.set_ylim(-max_grip*1.2, max_grip*1.2)
    ax1.set_aspect('equal')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Add annotations
    ax1.annotate('Pure Cornering\n(right)', xy=(max_grip*0.8, 0), fontsize=9, ha='center')
    ax1.annotate('Pure Accel', xy=(0, max_grip*0.7), fontsize=9, ha='center')
    ax1.annotate('Pure Braking', xy=(0, -max_grip*0.7), fontsize=9, ha='center')
    
    # === Subplot 2: Grip utilization along track ===
    ax2 = axes[1]
    
    # Lateral vs Combined utilization
    lat_utilization = a_lateral / max_grip * 100
    a_total = np.sqrt(a_lateral**2 + a_longitudinal**2)
    combined_utilization = a_total / max_grip * 100
    
    arc_length = np.arange(len(velocities))  # Simple index for x-axis
    
    # Plot both lateral and combined
    ax2.fill_between(arc_length, 0, lat_utilization, alpha=0.3, color='green', label='Lateral (stability critical)')
    ax2.fill_between(arc_length, lat_utilization, combined_utilization, alpha=0.3, color='blue', label='+ Longitudinal')
    ax2.plot(arc_length, combined_utilization, color='blue', linewidth=1)
    ax2.plot(arc_length, lat_utilization, color='green', linewidth=1)
    ax2.axhline(y=100, color='red', linewidth=2, linestyle='--', label='100% limit')
    
    ax2.set_xlabel('Waypoint Index', fontsize=12)
    ax2.set_ylabel('Grip Utilization (%)', fontsize=12)
    ax2.set_title('Friction Circle Utilization Along Track', fontsize=14)
    ax2.set_ylim(0, max(combined_utilization.max() * 1.1, 110))
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Statistics
    stats_text = (f"Lateral: {lat_utilization.mean():.1f}% avg, {lat_utilization.max():.1f}% max\n"
                  f"Combined: {combined_utilization.mean():.1f}% avg, {combined_utilization.max():.1f}% max\n"
                  f"Note: Combined >100% at corner entry/exit is normal")
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes, fontsize=9,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    
    print(f"  Saved friction circle viz: {output_path}")


# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description='Generate optimized racing line')
    parser.add_argument('--map', '-m', required=True, help='Path to map YAML file')
    parser.add_argument('--output', '-o', required=True, help='Output directory')
    parser.add_argument('--config', '-c', help='Path to vehicle config YAML')
    parser.add_argument('--points', '-n', type=int, default=1000,
                        help='Number of waypoints (default: 1000)')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Generate visualization')
    parser.add_argument('--no-friction-circle', action='store_true',
                        help='Disable friction circle constraints (use simple model)')
    args = parser.parse_args()
    
    # Extract track name from map path
    track_name = os.path.splitext(os.path.basename(args.map))[0].replace('_map', '')
    
    print("=" * 60)
    print("Racing Line Generator")
    print("=" * 60)
    print(f"Map: {args.map}")
    print(f"Track: {track_name}")
    
    # Load vehicle parameters
    config_path = args.config
    if not config_path:
        # Look for default config
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'vehicle_params.yaml')
    
    params = VehicleParams(config_path if os.path.exists(config_path) else None)
    print(f"\nVehicle Parameters:")
    print(f"  Friction: {params.friction_coeff}")
    print(f"  Max speed: {params.max_speed} m/s")
    print(f"  Max accel: {params.max_accel} m/s²")
    print(f"  Max decel: {params.max_decel} m/s²")
    print(f"  Safety margin: {params.safety_margin} m")
    
    # Step 1: Load map and extract boundaries
    print("\n" + "=" * 60)
    print("Step 1: Extracting track boundaries")
    print("=" * 60)
    img, resolution, origin = load_map(args.map)
    outer_world, inner_world = extract_boundaries(img, resolution, origin)
    print(f"  Outer boundary: {len(outer_world)} points")
    print(f"  Inner boundary: {len(inner_world)} points")
    
    # Step 2: Compute centerline
    print("\n" + "=" * 60)
    print("Step 2: Computing centerline")
    print("=" * 60)
    centerline, normals, inner_tree, outer_tree = compute_centerline(
        outer_world, inner_world, num_points=args.points
    )
    
    # Step 3: Optimize racing line
    print("\n" + "=" * 60)
    print("Step 3: Optimizing racing line")
    print("=" * 60)
    raceline = optimize_racing_line(
        centerline, normals, inner_tree, outer_tree, params, verbose=True
    )
    
    # Step 4: Compute velocity profile
    print("\n" + "=" * 60)
    print("Step 4: Computing velocity profile")
    use_friction_circle = not args.no_friction_circle
    print(f"  Friction circle model: {'ENABLED' if use_friction_circle else 'DISABLED'}")
    print("=" * 60)
    velocities, accelerations, arc_length, curvature, heading = compute_velocity_profile(
        raceline, params, verbose=True, use_friction_circle=use_friction_circle
    )
    
    # Step 5: Compute track bounds (for MPC constraints)
    print("\n" + "=" * 60)
    print("Step 5: Computing track bounds for MPC")
    print("=" * 60)
    left_bound, right_bound = compute_track_bounds(
        raceline, normals, inner_tree, outer_tree, params, verbose=True
    )
    
    # Step 6: Save output
    print("\n" + "=" * 60)
    print("Step 6: Saving trajectory")
    print("=" * 60)
    
    # Ensure output is a file path
    if os.path.isdir(args.output) or not args.output.endswith(('.csv', '.npz')):
        output_dir = args.output
        output_path = os.path.join(output_dir, f"{track_name}_raceline.npz")
    else:
        output_path = args.output
        if output_path.endswith('.csv'):
            output_path = output_path.replace('.csv', '.npz')
    
    save_trajectory(
        output_path, raceline, arc_length, heading, curvature,
        velocities, accelerations, params, track_name,
        left_bound, right_bound
    )
    
    # Optional visualization
    if args.visualize:
        print("\n" + "=" * 60)
        print("Creating visualizations")
        print("=" * 60)
        viz_path = output_path.replace('.npz', '_viz.png')
        visualize_raceline(args.map, raceline, velocities, viz_path)
        
        # Friction circle visualization
        fc_viz_path = output_path.replace('.npz', '_friction_circle.png')
        visualize_friction_circle(velocities, accelerations, curvature, params, fc_viz_path)
    
    print("\n" + "=" * 60)
    print("Racing line generation complete!")
    print("=" * 60)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
