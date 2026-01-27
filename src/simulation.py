"""
Sensor Fusion Demo
==================
Shows why combining sensors beats using any single sensor alone.

Key concept:
  LOW NOISE + HIGH DRIFT (Odometry/IMU)
           +
  HIGH NOISE + NO DRIFT (Camera/AprilTags)
           =
  BEST OF BOTH!

Run: python src/simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# === SETTINGS (experiment with these!) ===
TIME_STEPS = 150

# Odometry: smooth but drifts (wheel slip, encoder errors)
ODOM_NOISE = 0.02      # Low noise
ODOM_DRIFT = 0.1       # Drift accumulates over time!

# Camera/AprilTags: noisy but no drift
CAMERA_NOISE = 0.3     # Noisy
CAMERA_INTERVAL = 8    # Only updates every N steps

# Kalman filter
Q = 0.08  # Process noise
R = 0.15  # Measurement noise

# === KALMAN FILTER ===
class KalmanFilter:
    def __init__(self):
        self.x = 0
        self.P = 1.0
    
    def predict(self, velocity):
        self.x += velocity
        self.P += Q
    
    def update(self, measurement):
        K = self.P / (self.P + R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)

# === SIMULATION ===
# Ground truth
truth = np.cumsum(np.full(TIME_STEPS, 0.3) + np.random.normal(0, 0.01, TIME_STEPS))

# Odometry: drifts over time
odom_drift = np.cumsum(np.random.normal(0, ODOM_DRIFT, TIME_STEPS))
odom = truth + np.random.normal(0, ODOM_NOISE, TIME_STEPS) + odom_drift

# Camera: noisy but no drift
camera = np.full(TIME_STEPS, np.nan)
for i in range(0, TIME_STEPS, CAMERA_INTERVAL):
    camera[i] = truth[i] + np.random.normal(0, CAMERA_NOISE)

# Fused estimate using Kalman filter
kf = KalmanFilter()
fused = []
for i in range(TIME_STEPS):
    if i > 0:
        kf.predict(odom[i] - odom[i-1])
    if not np.isnan(camera[i]):
        kf.update(camera[i])
    fused.append(kf.x)
fused = np.array(fused)

# === RESULTS ===
odom_rmse = np.sqrt(np.mean((odom - truth)**2))
fused_rmse = np.sqrt(np.mean((fused - truth)**2))
improvement = (1 - fused_rmse / odom_rmse) * 100

print("=" * 50)
print("SENSOR FUSION RESULTS")
print("=" * 50)
print(f"  Odometry only: {odom_rmse:.3f}m RMSE  (drifts!)")
print(f"  FUSED:         {fused_rmse:.3f}m RMSE")
print(f"  Improvement:   {improvement:.0f}%")
print("=" * 50)
print("\nKey insight:")
print("  Odometry = Low noise, High drift")
print("  Camera   = High noise, No drift")
print("  Fused    = Low noise, No drift!")

# === PLOT ===
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(truth, 'k-', lw=2, label='Ground Truth')
ax.plot(odom, 'b-', alpha=0.6, label=f'Odometry only ({odom_rmse:.2f}m) - drifts!')
ax.scatter(np.where(~np.isnan(camera))[0], camera[~np.isnan(camera)], 
           c='g', s=60, marker='^', label='Camera (noisy but accurate)', zorder=5)
ax.plot(fused, 'r-', lw=2, label=f'FUSED ({fused_rmse:.2f}m) - best!')

ax.set_xlabel('Time Step')
ax.set_ylabel('Position (m)')
ax.set_title('Sensor Fusion: Odometry (drifts) + Camera (noisy) = Best Estimate')
ax.legend(loc='upper left')
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig('results.png', dpi=150)
print("\nSaved results.png")
plt.show()
