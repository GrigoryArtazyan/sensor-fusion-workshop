"""
Sensor Fusion Demo - Simplified
===============================
See how combining IMU + Camera beats either alone.

Run: python src/simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

# === SETTINGS (experiment with these!) ===
TIME_STEPS = 100
IMU_DRIFT = 0.06      # Try 0.1 for worse drift
CAMERA_NOISE = 0.5    # Try 1.0 for noisier camera
CAMERA_INTERVAL = 8   # Try 20 for fewer updates
KALMAN_Q = 0.02       # Process noise
KALMAN_R = 0.25       # Measurement noise

# === SIMPLE KALMAN FILTER ===
class KalmanFilter:
    def __init__(self):
        self.x = 0      # Position estimate
        self.P = 1.0    # Uncertainty
    
    def predict(self, velocity):
        self.x += velocity
        self.P += KALMAN_Q
    
    def update(self, measurement):
        K = self.P / (self.P + KALMAN_R)  # Kalman gain
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)

# === RUN SIMULATION ===
ground_truth = np.cumsum(np.full(TIME_STEPS, 0.3) + np.random.normal(0, 0.01, TIME_STEPS))

# IMU: drifts over time
imu_drift = np.cumsum(np.random.normal(0, IMU_DRIFT, TIME_STEPS))
imu = ground_truth + imu_drift

# Camera: noisy but no drift
camera = np.full(TIME_STEPS, np.nan)
for i in range(0, TIME_STEPS, CAMERA_INTERVAL):
    camera[i] = ground_truth[i] + np.random.normal(0, CAMERA_NOISE)

# Kalman fusion
kf = KalmanFilter()
fused = []
for i in range(TIME_STEPS):
    if i > 0: kf.predict(imu[i] - imu[i-1])
    if not np.isnan(camera[i]): kf.update(camera[i])
    fused.append(kf.x)
fused = np.array(fused)

# === PLOT ===
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(ground_truth, 'k-', lw=2, label='Ground Truth')
ax.plot(imu, 'c-', alpha=0.6, label=f'IMU (RMSE: {np.sqrt(np.mean((imu-ground_truth)**2)):.2f}m)')
ax.scatter(np.where(~np.isnan(camera))[0], camera[~np.isnan(camera)], 
           c='g', s=40, marker='^', label='Camera', zorder=5)
ax.plot(fused, 'r-', lw=2, label=f'Fused (RMSE: {np.sqrt(np.mean((fused-ground_truth)**2)):.2f}m)')
ax.set_xlabel('Time'); ax.set_ylabel('Position (m)')
ax.set_title('Sensor Fusion: IMU (drifts) + Camera (noisy) = Best estimate')
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig('results.png', dpi=150)
print("Saved results.png")
plt.show()
