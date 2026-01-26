"""
Sensor Fusion Simulation - Workshop Demo
=========================================

This simulation demonstrates why sensor fusion is valuable and how a 
Kalman filter combines data from multiple sensors.

Scenario:
---------
A robot moves along a 1D path. We have two sensors:

1. IMU-like sensor (velocity/acceleration integration):
   - LOW NOISE: Readings are smooth and precise moment-to-moment
   - HIGH DRIFT: Errors accumulate over time (integration drift)
   - FAST: Updates every time step
   
2. Camera-like sensor (visual odometry/landmarks):
   - HIGH NOISE: Readings are jumpy/inconsistent
   - LOW DRIFT: Provides absolute position reference
   - SLOWER: Updates less frequently (every N time steps)

By fusing these sensors, we get the best of both worlds:
- Smooth tracking from IMU
- Drift correction from camera

Run this file to see the visualization!

EXPERIMENTATION:
Modify the parameters in the "TUNING PARAMETERS" section to see how
different settings affect the filter's performance.
"""

import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter1D

# Set random seed for reproducibility
np.random.seed(42)


# =============================================================================
# TUNING PARAMETERS - Students: Experiment with these!
# =============================================================================

# Simulation settings
SIMULATION_TIME = 100       # Total time steps to simulate
ROBOT_VELOCITY = 0.5        # True robot velocity (units per time step)

# IMU-like sensor characteristics
IMU_NOISE_STD = 0.02        # Low noise (standard deviation)
IMU_DRIFT_RATE = 0.01       # Drift accumulates over time

# Camera-like sensor characteristics  
CAMERA_NOISE_STD = 0.5      # Higher noise than IMU
CAMERA_UPDATE_INTERVAL = 5  # Camera only updates every N steps (slower)

# Kalman filter tuning
KF_PROCESS_NOISE = 0.01     # Q: Trust in motion model (lower = trust model more)
KF_MEASUREMENT_NOISE = 0.25 # R: Trust in camera sensor (lower = trust sensor more)


# =============================================================================
# SIMULATION FUNCTIONS
# =============================================================================

def generate_ground_truth(num_steps, velocity):
    """
    Generate the true robot position over time.
    
    This is what we're trying to estimate - but in the real world,
    we never have direct access to ground truth!
    """
    positions = np.zeros(num_steps)
    for i in range(1, num_steps):
        # Simple constant velocity motion with small random variations
        positions[i] = positions[i-1] + velocity + np.random.normal(0, 0.01)
    return positions


def simulate_imu_sensor(ground_truth, noise_std, drift_rate):
    """
    Simulate an IMU-like sensor that integrates velocity.
    
    Characteristics:
    - Low instantaneous noise (smooth readings)
    - Drift that accumulates over time (the big problem with IMUs!)
    
    In real life, this is like integrating accelerometer readings to get
    velocity, then integrating velocity to get position. Each integration
    step adds a tiny error that compounds over time.
    """
    num_steps = len(ground_truth)
    imu_readings = np.zeros(num_steps)
    accumulated_drift = 0.0
    
    for i in range(num_steps):
        # Drift accumulates over time (random walk)
        accumulated_drift += np.random.normal(0, drift_rate)
        
        # IMU reading = true position + small noise + accumulated drift
        imu_readings[i] = ground_truth[i] + np.random.normal(0, noise_std) + accumulated_drift
    
    return imu_readings


def simulate_camera_sensor(ground_truth, noise_std, update_interval):
    """
    Simulate a camera-like sensor that provides absolute position.
    
    Characteristics:
    - Higher noise (vision processing is imperfect)
    - No drift (it measures absolute position from landmarks)
    - Slower update rate (image processing takes time)
    
    Returns both the readings and a mask indicating when readings are available.
    """
    num_steps = len(ground_truth)
    camera_readings = np.full(num_steps, np.nan)  # NaN when no reading
    camera_available = np.zeros(num_steps, dtype=bool)
    
    for i in range(0, num_steps, update_interval):
        # Camera reading = true position + noise (no drift!)
        camera_readings[i] = ground_truth[i] + np.random.normal(0, noise_std)
        camera_available[i] = True
    
    return camera_readings, camera_available


def run_kalman_fusion(imu_readings, camera_readings, camera_available,
                      process_noise, measurement_noise):
    """
    Run the Kalman filter to fuse IMU and camera data.
    
    Strategy:
    - PREDICT: Use IMU velocity estimate to predict next position
    - UPDATE: When camera reading available, correct the estimate
    
    This is a simplified version - real systems use the full state
    including velocity, and may have more sophisticated prediction models.
    """
    num_steps = len(imu_readings)
    
    # Initialize filter
    kf = KalmanFilter1D(
        initial_position=0.0,
        initial_uncertainty=1.0,
        process_noise=process_noise,
        measurement_noise=measurement_noise
    )
    
    # Storage for results
    fused_estimates = np.zeros(num_steps)
    uncertainties = np.zeros(num_steps)
    kalman_gains = np.zeros(num_steps)
    
    for i in range(num_steps):
        # PREDICT step: Use velocity from IMU
        if i > 0:
            # Estimate velocity from IMU readings
            imu_velocity = imu_readings[i] - imu_readings[i-1]
            kf.predict(velocity=imu_velocity, dt=1.0)
        
        # UPDATE step: Only when camera reading is available
        if camera_available[i]:
            est, unc, K = kf.update(camera_readings[i])
            kalman_gains[i] = K
        else:
            est, unc = kf.get_state()
            kalman_gains[i] = 0  # No update this step
        
        fused_estimates[i] = est
        uncertainties[i] = unc
    
    return fused_estimates, uncertainties, kalman_gains


def calculate_errors(ground_truth, estimates):
    """Calculate various error metrics."""
    errors = estimates - ground_truth
    rmse = np.sqrt(np.mean(errors**2))
    max_error = np.max(np.abs(errors))
    final_error = np.abs(errors[-1])
    return {
        'rmse': rmse,
        'max_error': max_error,
        'final_error': final_error,
        'errors': errors
    }


# =============================================================================
# VISUALIZATION
# =============================================================================

def plot_results(ground_truth, imu_readings, camera_readings, camera_available,
                 fused_estimates, uncertainties, kalman_gains):
    """Create a comprehensive visualization of the simulation results."""
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    time = np.arange(len(ground_truth))
    
    # --- Plot 1: Position Comparison ---
    ax1 = axes[0]
    ax1.plot(time, ground_truth, 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
    ax1.plot(time, imu_readings, 'b-', linewidth=1, alpha=0.6, label='IMU Only (drifts!)')
    
    # Camera readings (only where available)
    camera_times = time[camera_available]
    camera_vals = camera_readings[camera_available]
    ax1.scatter(camera_times, camera_vals, c='g', s=30, alpha=0.7, 
                label='Camera (noisy but no drift)', zorder=5)
    
    ax1.plot(time, fused_estimates, 'r-', linewidth=2, label='Kalman Fusion')
    
    # Add uncertainty band around fused estimate
    std = np.sqrt(uncertainties)
    ax1.fill_between(time, fused_estimates - 2*std, fused_estimates + 2*std,
                     color='red', alpha=0.1, label='95% Confidence')
    
    ax1.set_ylabel('Position')
    ax1.set_title('Sensor Fusion: Combining IMU (low noise, high drift) with Camera (high noise, no drift)')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # --- Plot 2: Estimation Errors ---
    ax2 = axes[1]
    
    imu_error = imu_readings - ground_truth
    fused_error = fused_estimates - ground_truth
    
    ax2.plot(time, imu_error, 'b-', alpha=0.6, label=f'IMU Error (RMSE: {np.sqrt(np.mean(imu_error**2)):.3f})')
    ax2.plot(time, fused_error, 'r-', linewidth=2, label=f'Fused Error (RMSE: {np.sqrt(np.mean(fused_error**2)):.3f})')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    ax2.set_ylabel('Error')
    ax2.set_title('Estimation Error Over Time')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)
    
    # --- Plot 3: Kalman Gain and Uncertainty ---
    ax3 = axes[2]
    
    ax3_twin = ax3.twinx()
    
    # Kalman gain (shows when camera updates happen)
    ax3.bar(time[camera_available], kalman_gains[camera_available], 
            color='green', alpha=0.6, label='Kalman Gain (at camera updates)')
    ax3.set_ylabel('Kalman Gain', color='green')
    ax3.tick_params(axis='y', labelcolor='green')
    
    # Uncertainty over time
    ax3_twin.plot(time, uncertainties, 'purple', linewidth=1.5, label='Uncertainty')
    ax3_twin.set_ylabel('Uncertainty (Variance)', color='purple')
    ax3_twin.tick_params(axis='y', labelcolor='purple')
    
    ax3.set_xlabel('Time Step')
    ax3.set_title('Kalman Gain (trust in camera) and Uncertainty Over Time')
    ax3.grid(True, alpha=0.3)
    
    # Combined legend
    lines1, labels1 = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_twin.get_legend_handles_labels()
    ax3.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    plt.tight_layout()
    return fig


def print_summary(ground_truth, imu_readings, fused_estimates):
    """Print a summary of the results."""
    imu_metrics = calculate_errors(ground_truth, imu_readings)
    fused_metrics = calculate_errors(ground_truth, fused_estimates)
    
    print("\n" + "="*60)
    print("SENSOR FUSION RESULTS SUMMARY")
    print("="*60)
    
    print("\n┌─────────────────┬─────────────┬─────────────┐")
    print("│     Metric      │   IMU Only  │ Kalman Fused│")
    print("├─────────────────┼─────────────┼─────────────┤")
    print(f"│ RMSE            │   {imu_metrics['rmse']:8.4f}  │   {fused_metrics['rmse']:8.4f}  │")
    print(f"│ Max Error       │   {imu_metrics['max_error']:8.4f}  │   {fused_metrics['max_error']:8.4f}  │")
    print(f"│ Final Error     │   {imu_metrics['final_error']:8.4f}  │   {fused_metrics['final_error']:8.4f}  │")
    print("└─────────────────┴─────────────┴─────────────┘")
    
    improvement = (1 - fused_metrics['rmse'] / imu_metrics['rmse']) * 100
    print(f"\n✓ Sensor fusion reduced RMSE by {improvement:.1f}%")
    
    print("\n" + "-"*60)
    print("KEY OBSERVATIONS:")
    print("-"*60)
    print("1. IMU alone drifts significantly over time")
    print("2. Camera readings are noisy but provide absolute reference")
    print("3. Kalman fusion gives smooth tracking with drift correction")
    print("4. Uncertainty grows between camera updates, shrinks after")
    print("\nTry adjusting the parameters at the top of this file!")


# =============================================================================
# MAIN SIMULATION
# =============================================================================

def main():
    """Run the complete sensor fusion simulation."""
    
    print("="*60)
    print("SENSOR FUSION SIMULATION")
    print("="*60)
    print(f"\nSimulation Parameters:")
    print(f"  - Time steps: {SIMULATION_TIME}")
    print(f"  - Robot velocity: {ROBOT_VELOCITY}")
    print(f"\nIMU Sensor:")
    print(f"  - Noise std: {IMU_NOISE_STD} (low)")
    print(f"  - Drift rate: {IMU_DRIFT_RATE}")
    print(f"\nCamera Sensor:")
    print(f"  - Noise std: {CAMERA_NOISE_STD} (high)")
    print(f"  - Update interval: every {CAMERA_UPDATE_INTERVAL} steps")
    print(f"\nKalman Filter:")
    print(f"  - Process noise (Q): {KF_PROCESS_NOISE}")
    print(f"  - Measurement noise (R): {KF_MEASUREMENT_NOISE}")
    
    # Generate simulation data
    print("\n[1/4] Generating ground truth...")
    ground_truth = generate_ground_truth(SIMULATION_TIME, ROBOT_VELOCITY)
    
    print("[2/4] Simulating IMU sensor...")
    imu_readings = simulate_imu_sensor(ground_truth, IMU_NOISE_STD, IMU_DRIFT_RATE)
    
    print("[3/4] Simulating camera sensor...")
    camera_readings, camera_available = simulate_camera_sensor(
        ground_truth, CAMERA_NOISE_STD, CAMERA_UPDATE_INTERVAL
    )
    
    print("[4/4] Running Kalman filter fusion...")
    fused_estimates, uncertainties, kalman_gains = run_kalman_fusion(
        imu_readings, camera_readings, camera_available,
        KF_PROCESS_NOISE, KF_MEASUREMENT_NOISE
    )
    
    # Print results
    print_summary(ground_truth, imu_readings, fused_estimates)
    
    # Create visualization
    print("\nGenerating visualization...")
    fig = plot_results(
        ground_truth, imu_readings, camera_readings, camera_available,
        fused_estimates, uncertainties, kalman_gains
    )
    
    plt.savefig('sensor_fusion_results.png', dpi=150, bbox_inches='tight')
    print("Saved plot to 'sensor_fusion_results.png'")
    
    plt.show()
    
    return ground_truth, imu_readings, camera_readings, fused_estimates


# =============================================================================
# EXERCISES FOR STUDENTS
# =============================================================================
"""
EXERCISE 1: Tune the Filter
---------------------------
Modify the TUNING PARAMETERS section and observe:
a) What happens when you increase KF_PROCESS_NOISE (Q)?
b) What happens when you increase KF_MEASUREMENT_NOISE (R)?
c) What's the best balance for this scenario?

EXERCISE 2: Change Sensor Characteristics
----------------------------------------
a) Make the IMU drift worse (increase IMU_DRIFT_RATE to 0.05)
   - How does the fused estimate handle it?
b) Make camera updates less frequent (CAMERA_UPDATE_INTERVAL = 20)
   - What happens to drift between camera updates?
c) Make camera very accurate (CAMERA_NOISE_STD = 0.1)
   - Does the filter rely more on camera?

EXERCISE 3: Different Motion Patterns
------------------------------------
Modify generate_ground_truth() to simulate:
a) Accelerating robot: velocity increases over time
b) Stopping robot: velocity becomes 0 halfway through
c) Reversing robot: velocity becomes negative
How well does the constant-velocity model handle these cases?

EXERCISE 4: Add a Third Sensor
-----------------------------
Create a new function simulate_odometry_sensor() that:
- Has medium noise (between IMU and camera)
- Has medium drift (less than IMU)
- Updates every time step
Modify run_kalman_fusion() to incorporate this third sensor.

EXERCISE 5: Real-World Considerations
------------------------------------
Think about these questions:
a) How would you handle a camera that occasionally gives completely wrong
   readings (outliers)?
b) What if you don't know the true noise characteristics of your sensors?
c) How would you extend this to 2D or 3D position tracking?
"""


if __name__ == "__main__":
    # Run the simulation
    results = main()
    
    print("\n" + "="*60)
    print("WORKSHOP EXERCISES")
    print("="*60)
    print("""
Try these experiments:

1. TUNE THE FILTER
   - Increase KF_PROCESS_NOISE (Q): Filter trusts measurements more
   - Increase KF_MEASUREMENT_NOISE (R): Filter trusts model more
   
2. BREAK THE IMU
   - Set IMU_DRIFT_RATE = 0.05 (worse drift)
   - See how fusion still works!
   
3. SLOW DOWN CAMERA
   - Set CAMERA_UPDATE_INTERVAL = 20
   - Watch uncertainty grow between updates
   
4. PERFECT CAMERA
   - Set CAMERA_NOISE_STD = 0.01
   - The filter will trust it almost completely

Modify the parameters at the top of this file and re-run!
""")
