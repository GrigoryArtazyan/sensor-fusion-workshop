"""
Multi-Sensor Fusion Simulation for FRC
=======================================
Fuse multiple sensor types: IMU, Odometry, Camera (AprilTags), and LiDAR.

Each sensor has different characteristics:
- IMU: Very fast, low noise, but drifts over time
- Odometry: Fast, low noise, drifts (wheel slip)
- Camera/AprilTags: Slow, noisy, but NO drift (absolute reference)
- LiDAR: Medium speed, low noise, no drift (when landmarks visible)

Run: python src/simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter1D

np.random.seed(42)

# ============================================
# SENSOR PARAMETERS - Experiment with these!
# ============================================

SIMULATION_TIME = 150  # Time steps
ROBOT_VELOCITY = 0.3   # meters per time step

# IMU (gyro/accelerometer integration)
# Fast and smooth, but drifts over time
IMU_ENABLED = True
IMU_NOISE = 0.01       # Very low noise
IMU_DRIFT = 0.02       # Accumulates! Try 0.05

# Wheel Odometry (encoder integration)  
# Fast, but wheel slip causes drift
ODOM_ENABLED = True
ODOM_NOISE = 0.02      # Low noise
ODOM_DRIFT = 0.03      # Wheel slip drift. Try 0.08

# Camera / AprilTags (visual odometry)
# Slower, noisier, but gives absolute position
CAMERA_ENABLED = True
CAMERA_NOISE = 0.4     # Higher noise per reading
CAMERA_INTERVAL = 10   # Updates every N steps. Try 20, 30

# LiDAR (landmark detection)
# When available, very accurate
LIDAR_ENABLED = False  # Try enabling this!
LIDAR_NOISE = 0.15     # Low noise
LIDAR_INTERVAL = 8     # Updates every N steps

# Kalman filter tuning
Q = 0.02  # Process noise (trust in motion model)
R = 0.20  # Measurement noise (trust in absolute sensors)


# ============================================
# SENSOR SIMULATION
# ============================================

def generate_ground_truth(num_steps, velocity):
    """True robot position (what we're trying to estimate)."""
    positions = np.zeros(num_steps)
    for i in range(1, num_steps):
        # Varying velocity to make it interesting
        v = velocity * (1 + 0.3 * np.sin(i / 20))
        positions[i] = positions[i-1] + v + np.random.normal(0, 0.005)
    return positions


def simulate_imu(ground_truth, noise, drift_rate):
    """
    IMU: Integrates acceleration to get position.
    - Very fast updates (every step)
    - Low instantaneous noise
    - DRIFTS over time (integration error)
    """
    n = len(ground_truth)
    readings = np.zeros(n)
    accumulated_drift = 0.0
    
    for i in range(n):
        accumulated_drift += np.random.normal(0, drift_rate)
        readings[i] = ground_truth[i] + np.random.normal(0, noise) + accumulated_drift
    
    return readings


def simulate_odometry(ground_truth, noise, drift_rate):
    """
    Wheel Odometry: Integrates wheel rotations.
    - Fast updates
    - Low noise normally
    - DRIFTS due to wheel slip, carpet, etc.
    """
    n = len(ground_truth)
    readings = np.zeros(n)
    accumulated_drift = 0.0
    
    for i in range(n):
        # Wheel slip events (occasional larger errors)
        if np.random.random() < 0.05:  # 5% chance of slip
            accumulated_drift += np.random.normal(0, drift_rate * 3)
        else:
            accumulated_drift += np.random.normal(0, drift_rate)
        
        readings[i] = ground_truth[i] + np.random.normal(0, noise) + accumulated_drift
    
    return readings


def simulate_camera(ground_truth, noise, interval):
    """
    Camera/AprilTags: Visual position from landmarks.
    - Slower updates (image processing takes time)
    - Noisy (lighting, motion blur, distance)
    - NO DRIFT (absolute position reference)
    """
    n = len(ground_truth)
    readings = np.full(n, np.nan)
    available = np.zeros(n, dtype=bool)
    
    for i in range(0, n, interval):
        # Sometimes tags aren't visible
        if np.random.random() > 0.1:  # 90% detection rate
            readings[i] = ground_truth[i] + np.random.normal(0, noise)
            available[i] = True
    
    return readings, available


def simulate_lidar(ground_truth, noise, interval):
    """
    LiDAR: Distance to known landmarks.
    - Medium update rate
    - Low noise (very precise)
    - NO DRIFT (absolute reference)
    """
    n = len(ground_truth)
    readings = np.full(n, np.nan)
    available = np.zeros(n, dtype=bool)
    
    for i in range(0, n, interval):
        if np.random.random() > 0.15:  # 85% detection rate
            readings[i] = ground_truth[i] + np.random.normal(0, noise)
            available[i] = True
    
    return readings, available


# ============================================
# FUSION
# ============================================

def fuse_sensors(imu, odom, camera, camera_avail, lidar, lidar_avail):
    """
    Kalman filter fusion of all enabled sensors.
    
    Strategy:
    - PREDICT: Blend IMU and odometry velocities
    - UPDATE: Use camera and/or LiDAR when available
    """
    n = len(imu)
    kf = KalmanFilter1D(process_noise=Q, measurement_noise=R)
    
    fused = np.zeros(n)
    uncertainty = np.zeros(n)
    
    for i in range(n):
        # PREDICT using motion sensors
        if i > 0:
            velocities = []
            if IMU_ENABLED:
                velocities.append(imu[i] - imu[i-1])
            if ODOM_ENABLED:
                velocities.append(odom[i] - odom[i-1])
            
            if velocities:
                avg_velocity = np.mean(velocities)
                kf.predict(velocity=avg_velocity, dt=1.0)
        
        # UPDATE with absolute position sensors
        updated = False
        
        if CAMERA_ENABLED and camera_avail[i]:
            kf.update(camera[i])
            updated = True
        
        if LIDAR_ENABLED and lidar_avail[i]:
            kf.update(lidar[i])
            updated = True
        
        fused[i], uncertainty[i] = kf.get_state()
    
    return fused, uncertainty


# ============================================
# VISUALIZATION
# ============================================

def plot_results(ground_truth, imu, odom, camera, camera_avail, 
                 lidar, lidar_avail, fused, uncertainty):
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
    t = np.arange(len(ground_truth))
    
    # === Position Plot ===
    ax1 = axes[0]
    ax1.plot(t, ground_truth, 'k-', lw=2.5, label='Ground Truth', zorder=10)
    
    if IMU_ENABLED:
        ax1.plot(t, imu, 'c-', alpha=0.5, lw=1, label='IMU (drifts)')
    if ODOM_ENABLED:
        ax1.plot(t, odom, 'b-', alpha=0.5, lw=1, label='Odometry (drifts)')
    if CAMERA_ENABLED:
        ax1.scatter(t[camera_avail], camera[camera_avail], c='green', s=50, 
                    marker='^', label='Camera/AprilTag', zorder=5)
    if LIDAR_ENABLED:
        ax1.scatter(t[lidar_avail], lidar[lidar_avail], c='orange', s=40,
                    marker='s', label='LiDAR', zorder=5)
    
    ax1.plot(t, fused, 'r-', lw=2, label='Kalman Fusion')
    ax1.fill_between(t, fused - 2*np.sqrt(uncertainty), fused + 2*np.sqrt(uncertainty),
                     color='red', alpha=0.15)
    
    ax1.set_ylabel('Position (m)', fontsize=11)
    ax1.set_title('Multi-Sensor Fusion: Combining Drifting + Absolute Sensors', fontsize=12)
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # === Error Plot ===
    ax2 = axes[1]
    
    errors = {}
    if IMU_ENABLED:
        errors['IMU'] = ('c', imu - ground_truth)
    if ODOM_ENABLED:
        errors['Odometry'] = ('b', odom - ground_truth)
    errors['Fused'] = ('r', fused - ground_truth)
    
    for name, (color, err) in errors.items():
        rmse = np.sqrt(np.mean(err**2))
        lw = 2 if name == 'Fused' else 1
        alpha = 1.0 if name == 'Fused' else 0.5
        ax2.plot(t, err, color=color, lw=lw, alpha=alpha,
                 label=f'{name} (RMSE: {rmse:.3f}m)')
    
    ax2.axhline(0, color='k', ls='--', alpha=0.3)
    ax2.set_xlabel('Time Step', fontsize=11)
    ax2.set_ylabel('Error (m)', fontsize=11)
    ax2.set_title('Position Error Over Time', fontsize=12)
    ax2.legend(loc='upper left', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def print_config():
    print("=" * 55)
    print("MULTI-SENSOR FUSION SIMULATION")
    print("=" * 55)
    print("\nEnabled Sensors:")
    print(f"  IMU:      {'ON' if IMU_ENABLED else 'OFF':3}  (noise={IMU_NOISE}, drift={IMU_DRIFT})")
    print(f"  Odometry: {'ON' if ODOM_ENABLED else 'OFF':3}  (noise={ODOM_NOISE}, drift={ODOM_DRIFT})")
    print(f"  Camera:   {'ON' if CAMERA_ENABLED else 'OFF':3}  (noise={CAMERA_NOISE}, interval={CAMERA_INTERVAL})")
    print(f"  LiDAR:    {'ON' if LIDAR_ENABLED else 'OFF':3}  (noise={LIDAR_NOISE}, interval={LIDAR_INTERVAL})")
    print(f"\nKalman: Q={Q}, R={R}")


def print_results(ground_truth, imu, odom, fused):
    print(f"\n{'=' * 55}")
    print("RESULTS")
    print(f"{'=' * 55}")
    
    results = []
    if IMU_ENABLED:
        rmse = np.sqrt(np.mean((imu - ground_truth)**2))
        results.append(('IMU only', rmse))
    if ODOM_ENABLED:
        rmse = np.sqrt(np.mean((odom - ground_truth)**2))
        results.append(('Odometry only', rmse))
    
    fused_rmse = np.sqrt(np.mean((fused - ground_truth)**2))
    results.append(('FUSED', fused_rmse))
    
    for name, rmse in results:
        marker = '→' if name == 'FUSED' else ' '
        print(f"  {marker} {name:15} RMSE: {rmse:.4f} m")
    
    # Calculate improvement
    if len(results) > 1:
        best_single = min(r[1] for r in results[:-1])
        improvement = (1 - fused_rmse / best_single) * 100
        if improvement > 0:
            print(f"\n✓ Fusion improved accuracy by {improvement:.1f}%")
        else:
            print(f"\n→ Fusion didn't help here. Try increasing drift rates!")


# ============================================
# MAIN
# ============================================

if __name__ == "__main__":
    print_config()
    
    # Generate data
    ground_truth = generate_ground_truth(SIMULATION_TIME, ROBOT_VELOCITY)
    imu = simulate_imu(ground_truth, IMU_NOISE, IMU_DRIFT)
    odom = simulate_odometry(ground_truth, ODOM_NOISE, ODOM_DRIFT)
    camera, camera_avail = simulate_camera(ground_truth, CAMERA_NOISE, CAMERA_INTERVAL)
    lidar, lidar_avail = simulate_lidar(ground_truth, LIDAR_NOISE, LIDAR_INTERVAL)
    
    # Fuse
    fused, uncertainty = fuse_sensors(imu, odom, camera, camera_avail, lidar, lidar_avail)
    
    # Results
    print_results(ground_truth, imu, odom, fused)
    
    print(f"\n{'=' * 55}")
    print("EXPERIMENTS TO TRY")
    print(f"{'=' * 55}")
    print("""
1. ENABLE LIDAR: Set LIDAR_ENABLED = True
   → Another absolute reference helps!

2. WORSE DRIFT: Set IMU_DRIFT = 0.08, ODOM_DRIFT = 0.1  
   → Watch fusion shine with bad encoders

3. LOSE VISION: Set CAMERA_INTERVAL = 30
   → What if you rarely see AprilTags?

4. CAMERA ONLY: Set IMU_ENABLED = False, ODOM_ENABLED = False
   → See why you need motion sensors too

5. TUNE FILTER: Adjust Q (model trust) and R (sensor trust)
""")
    
    # Plot
    fig = plot_results(ground_truth, imu, odom, camera, camera_avail,
                       lidar, lidar_avail, fused, uncertainty)
    plt.savefig('results.png', dpi=150, bbox_inches='tight')
    print("Saved plot to results.png")
    plt.show()
