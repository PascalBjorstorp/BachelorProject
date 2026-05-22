#!/usr/bin/env python3
import os
import subprocess
import sys
import random
from concurrent.futures import ThreadPoolExecutor, as_completed

def run_sim(q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel=10.0, q_yaw=1.5):
    env = os.environ.copy()
    env.update({
        "Q_LAT": str(q_lat),
        "Q_HDG": str(q_hdg),
        "Q_VEL": str(q_vel),
        "R_STEER": str(r_steer),
        "W_JERK": str(w_jerk),
        "Q_LAT_VEL": str(q_lat_vel),
        "Q_YAW": str(q_yaw)
    })
    
    proc = subprocess.run(
        ["./test_sim_drive"],
        env=env,
        capture_output=True,
        text=True
    )
    
    stdout = proc.stdout
    
    crashed = "WALL CRASH" in stdout or "[FAIL] No wall collisions" in stdout
    
    max_lat_err = 9.9
    avg_speed = 0.0
    for line in stdout.splitlines():
        if "Max lateral error:" in line:
            parts = line.strip().split()
            max_lat_err = float(parts[-2])
        if "Avg velocity:" in line:
            parts = line.strip().split()
            avg_speed = float(parts[-2])
            
    return crashed, max_lat_err, avg_speed, q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel, q_yaw

def main():
    print("--- Starting High-Speed Collision-Free MPC Weight Explorer ---")
    print("Plant Mode: Realistic Real-Car Calibration default")
    
    candidates = []
    
    # Focus search on high speed tracking (Q_VEL >= 100) and agile steering
    random.seed(42)
    for _ in range(800):
        q_lat = random.uniform(1000.0, 5000.0)
        q_hdg = random.uniform(5.0, 100.0)
        q_vel = random.uniform(100.0, 450.0)    # High Q_VEL to push speed limits
        r_steer = random.uniform(0.1, 1.5)     # Low steer cost for agility
        w_jerk = random.uniform(0.1, 3.0)      # Low jerk cost for fast corrections
        q_lat_vel = random.uniform(2.0, 30.0)
        q_yaw = random.uniform(0.5, 5.0)
        candidates.append((q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel, q_yaw))

    print(f"Evaluating {len(candidates)} high-speed candidates in parallel...")
    
    passing = []
    
    with ThreadPoolExecutor(max_workers=12) as executor:
        futures = {executor.submit(run_sim, *c): c for c in candidates}
        
        count = 0
        for future in as_completed(futures):
            count += 1
            crashed, max_lat_err, avg_speed, q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel, q_yaw = future.result()
            
            if not crashed:
                weights_str = f"Q_LAT={q_lat:.1f} Q_HDG={q_hdg:.1f} Q_VEL={q_vel:.1f} R_STEER={r_steer:.3f} W_JERK={w_jerk:.3f} Q_LAT_VEL={q_lat_vel:.2f} Q_YAW={q_yaw:.2f}"
                print(f"\n[PASS] {weights_str} | Avg Speed={avg_speed:.3f} m/s, Max Lat Err={max_lat_err:.3f} m")
                passing.append((avg_speed, max_lat_err, q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel, q_yaw))
                
            if count % 100 == 0:
                sys.stdout.write(f"\rProgress: {count}/{len(candidates)} tested...")
                sys.stdout.flush()
                
    if passing:
        # Sort by average speed in descending order to get the fastest passing weight configuration
        passing.sort(key=lambda x: x[0], reverse=True)
        best = passing[0]
        print("\n\n=== ABSOLUTE FASTEST COLLISION-FREE WEIGHTS FOUND ===")
        print(f"export Q_LAT={best[2]:.4f}")
        print(f"export Q_HDG={best[3]:.4f}")
        print(f"export Q_VEL={best[4]:.4f}")
        print(f"export R_STEER={best[5]:.4f}")
        print(f"export W_JERK={best[6]:.4f}")
        print(f"export Q_LAT_VEL={best[7]:.4f}")
        print(f"export Q_YAW={best[8]:.4f}")
        print(f"Average Speed: {best[0]:.3f} m/s")
        print(f"Max lateral error: {best[1]:.3f} m")
    else:
        print("\n\n[FAIL] No collision-free combinations found. The real-car physics bounds are highly constrained.")

if __name__ == "__main__":
    main()
