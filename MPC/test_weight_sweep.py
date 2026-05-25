#!/usr/bin/env python3
import os
import subprocess
import sys

def run_sim(q_lat, q_hdg, q_vel, r_steer, w_jerk, speed_gain=1.0):
    env = os.environ.copy()
    env.update({
        "Q_LAT": str(q_lat),
        "Q_HDG": str(q_hdg),
        "Q_VEL": str(q_vel),
        "R_STEER": str(r_steer),
        "W_JERK": str(w_jerk),
        "TRAJECTORY_SPEED_GAIN": str(speed_gain) # if read in future
    })
    
    # Run the compiled simulator binary in the MPC folder
    proc = subprocess.run(
        ["./test_sim_drive"],
        env=env,
        capture_output=True,
        text=True
    )
    
    stdout = proc.stdout
    
    # Parse the outputs
    crashed = "WALL CRASH" in stdout or "[FAIL] No wall collisions" in stdout
    passed_speed = "[FAIL] Reaches driving speed" not in stdout
    
    max_lat_err = 9.9
    avg_speed = 0.0
    for line in stdout.splitlines():
        if "Max lateral error:" in line:
            parts = line.strip().split()
            max_lat_err = float(parts[-2])
        if "Avg velocity:" in line:
            parts = line.strip().split()
            avg_speed = float(parts[-2])
            
    return crashed, passed_speed, max_lat_err, avg_speed, stdout

def main():
    print("--- Starting MPC Cost Weight Sweep for Aligned Plant ---")
    
    # We will test combinations of Q_LAT, Q_HDG, R_STEER, and W_JERK
    q_lat_options = [1500.0, 2000.0, 2500.0, 3000.0]
    q_hdg_options = [5.0, 10.0, 20.0, 30.0]
    r_steer_options = [0.1, 0.5, 1.0, 2.0]
    w_jerk_options = [0.5, 1.0, 2.0, 5.0]
    
    best_weights = None
    min_lat_err = 9.9
    
    for q_lat in q_lat_options:
        for q_hdg in q_hdg_options:
            for r_steer in r_steer_options:
                for w_jerk in w_jerk_options:
                    crashed, passed_speed, max_lat_err, avg_speed, stdout = run_sim(
                        q_lat, q_hdg, 200.0, r_steer, w_jerk
                    )
                    
                    status = "PASSED" if (not crashed and passed_speed) else "FAILED"
                    if status == "PASSED":
                        print(f"[SUCCESS] Q_LAT={q_lat} Q_HDG={q_hdg} R_STEER={r_steer} W_JERK={w_jerk} | Max Lat Err={max_lat_err:.3f}m, Avg Speed={avg_speed:.2f}m/s")
                        if max_lat_err < min_lat_err:
                            min_lat_err = max_lat_err
                            best_weights = (q_lat, q_hdg, r_steer, w_jerk)
                    else:
                        crash_reason = "CRASH" if crashed else ""
                        speed_reason = "SLOW" if not passed_speed else ""
                        reasons = ", ".join(filter(None, [crash_reason, speed_reason]))
                        # print(f"  Failed: Q_LAT={q_lat} Q_HDG={q_hdg} R_STEER={r_steer} W_JERK={w_jerk} | {reasons}")
                        
    if best_weights:
        print("\n=== OPTIMAL WEIGHTS FOUND ===")
        print(f"  Q_LAT={best_weights[0]}")
        print(f"  Q_HDG={best_weights[1]}")
        print(f"  R_STEER={best_weights[2]}")
        print(f"  W_JERK={best_weights[3]}")
    else:
        print("\n[ALERT] No single weight set passed both speed and safety constraints. Let's try wider sweeps or tuning other parameters.")

if __name__ == "__main__":
    main()
