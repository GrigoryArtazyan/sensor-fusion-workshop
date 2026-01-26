"""
Sensor Fusion Simulation for FRC
================================
Simulates fusing odometry (drifts) with AprilTag vision (noisy but absolute).

This mirrors what WPILib's PoseEstimator classes do internally!

Run: python src/simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter1D

np.random.seed(42)

# ============================================
# PARAMETERS - Experiment with these!
# ============================================
SIMULATION_TIME = 100  # Time steps (like 2 seconds at 50Hz)
ROBOT_VELOCITY = 0.5   # meters per time step

# Odometry: fast but drifts (wheel slip, encoder error)
ODOMETRY_NOISE = 0.02   # Low instantaneous noise
ODOMETRY_DRIFT = 0.05   # Accumulates over time - try 0.1!

# AprilTag Vision: slower, noisier, but no drift
VISION_NOISE = 0.5      # Higher noise per reading
VISION_INTERVAL = 5     # Only get pose every N steps - try 20!

# Kalman filter tuning (like WPILib std devs)
Q = 0.01  # State/odometry trust (lower = trust more)
R = 0.25  # Vision trust (lower = trust more)


# ============================================
# SIMULATION
# ============================================
def run_simulation():
    # Ground truth: robot driving forward
    ground_truth = np.cumsum([ROBOT_VELOCITY + np.random.normal(0, 0.01) 
                              for _ in range(SIMULATION_TIME)])
    ground_truth = np.insert(ground_truth, 0, 0)[:-1]
    
    # Odometry: integrates velocity but drifts
    drift = np.cumsum(np.random.normal(0, ODOMETRY_DRIFT, SIMULATION_TIME))
    odometry = ground_truth + np.random.normal(0, ODOMETRY_NOISE, SIMULATION_TIME) + drift
    
    # AprilTag vision: absolute position but noisy and intermittent
    vision = np.full(SIMULATION_TIME, np.nan)
    vision_mask = np.zeros(SIMULATION_TIME, dtype=bool)
    for i in range(0, SIMULATION_TIME, VISION_INTERVAL):
        vision[i] = ground_truth[i] + np.random.normal(0, VISION_NOISE)
        vision_mask[i] = True
    
    # Kalman filter fusion (like WPILib PoseEstimator)
    kf = KalmanFilter1D(process_noise=Q, measurement_noise=R)
    fused = np.zeros(SIMULATION_TIME)
    uncertainty = np.zeros(SIMULATION_TIME)
    
    for i in range(SIMULATION_TIME):
        # PREDICT with odometry
        if i > 0:
            velocity = odometry[i] - odometry[i-1]
            kf.predict(velocity=velocity, dt=1.0)
        
        # UPDATE with vision (when AprilTag visible)
        if vision_mask[i]:
            est, unc, _ = kf.update(vision[i])
        else:
            est, unc = kf.get_state()
        
        fused[i] = est
        uncertainty[i] = unc
    
    return ground_truth, odometry, vision, vision_mask, fused, uncertainty


def plot_results(ground_truth, odometry, vision, vision_mask, fused, uncertainty):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    t = np.arange(len(ground_truth))
    
    # Position plot
    ax1.plot(t, ground_truth, 'k-', lw=2, label='Ground Truth')
    ax1.plot(t, odometry, 'b-', alpha=0.6, label='Odometry Only (drifts!)')
    ax1.scatter(t[vision_mask], vision[vision_mask], c='g', s=40, 
                label='AprilTag Vision (noisy)', zorder=5, marker='^')
    ax1.plot(t, fused, 'r-', lw=2, label='Kalman Fusion')
    ax1.fill_between(t, fused - 2*np.sqrt(uncertainty), fused + 2*np.sqrt(uncertainty),
                     color='red', alpha=0.1, label='95% confidence')
    ax1.set_ylabel('Position (m)')
    ax1.set_title('FRC Sensor Fusion: Odometry + AprilTag Vision')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # Error plot
    odom_rmse = np.sqrt(np.mean((odometry - ground_truth)**2))
    fused_rmse = np.sqrt(np.mean((fused - ground_truth)**2))
    
    ax2.plot(t, odometry - ground_truth, 'b-', alpha=0.6, 
             label=f'Odometry Error (RMSE: {odom_rmse:.2f}m)')
    ax2.plot(t, fused - ground_truth, 'r-', lw=2,
             label=f'Fused Error (RMSE: {fused_rmse:.2f}m)')
    ax2.axhline(0, color='k', ls='--', alpha=0.3)
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Position Error Over Time')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


if __name__ == "__main__":
    print("=" * 50)
    print("FRC SENSOR FUSION SIMULATION")
    print("=" * 50)
    print(f"\nOdometry (like wheel encoders):")
    print(f"  Drift rate: {ODOMETRY_DRIFT}")
    print(f"\nAprilTag Vision:")
    print(f"  Noise: {VISION_NOISE}")
    print(f"  Update interval: every {VISION_INTERVAL} steps")
    print(f"\nKalman Filter (like WPILib std devs):")
    print(f"  Q (state): {Q}")
    print(f"  R (vision): {R}")
    
    results = run_simulation()
    ground_truth, odometry, vision, vision_mask, fused, uncertainty = results
    
    # Results
    odom_rmse = np.sqrt(np.mean((odometry - ground_truth)**2))
    fused_rmse = np.sqrt(np.mean((fused - ground_truth)**2))
    improvement = (1 - fused_rmse / odom_rmse) * 100
    
    print(f"\n{'=' * 50}")
    print("RESULTS")
    print(f"{'=' * 50}")
    print(f"  Odometry RMSE:  {odom_rmse:.3f} m")
    print(f"  Fused RMSE:     {fused_rmse:.3f} m")
    print(f"  Improvement:    {improvement:.1f}%")
    
    if improvement > 0:
        print(f"\n✓ Fusion reduced error by {improvement:.1f}%!")
    else:
        print(f"\n→ Try increasing ODOMETRY_DRIFT to see fusion benefits")
    
    print(f"\n{'=' * 50}")
    print("EXERCISES")
    print(f"{'=' * 50}")
    print("""
1. WORSE DRIFT: Set ODOMETRY_DRIFT = 0.1
   → Watch fusion handle encoder slip!

2. LOSE VISION: Set VISION_INTERVAL = 20  
   → What happens when you can't see AprilTags?

3. TUNE FILTER: Adjust Q and R
   → Q↑ = trust vision more, R↑ = trust odometry more

4. PERFECT VISION: Set VISION_NOISE = 0.1
   → Filter relies heavily on AprilTags
""")
    
    fig = plot_results(*results)
    plt.savefig('results.png', dpi=150, bbox_inches='tight')
    print("Saved plot to results.png")
    plt.show()
