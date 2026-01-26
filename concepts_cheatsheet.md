# Sensor Fusion & Kalman Filter Cheatsheet

## Why Sensor Fusion?

**Single sensors have limitations:**
- IMU: Drifts over time (integration errors accumulate)
- Camera: Noisy, affected by lighting, slower
- Odometry: Wheel slip causes errors

**Combining sensors = Best of both worlds!**

---

## Sensor Characteristics

| Sensor | Noise | Drift | Update Rate | Good For |
|--------|-------|-------|-------------|----------|
| IMU (Accel/Gyro) | Low | High | Fast (100-1000 Hz) | Short-term motion |
| Camera/Vision | High | Low | Slow (30-60 Hz) | Absolute position |
| Wheel Odometry | Medium | High | Fast | Relative motion |
| LiDAR | Low | Low | Medium (10-40 Hz) | Distance mapping |

**Key Insight:** Fuse high-noise/low-drift with low-noise/high-drift sensors!

---

## The Kalman Filter

### Core Idea
A continuous **predict-update** loop that optimally combines:
- What we **expect** (from motion model)
- What we **observe** (from sensors)

```
┌─────────────┐     ┌─────────────┐
│   PREDICT   │ ──▶ │   UPDATE    │
│ (use model) │     │ (use sensor)│
└─────────────┘     └─────────────┘
       ▲                   │
       └───────────────────┘
           (repeat)
```

### State Variables
- `x` = State estimate (what we're tracking: position, velocity, etc.)
- `P` = Uncertainty/covariance (how confident we are)

### Noise Parameters (TUNING THESE IS KEY!)
- `Q` = Process noise (trust in model)
  - Higher Q → Trust measurements more
  - Lower Q → Trust model more
  
- `R` = Measurement noise (trust in sensors)
  - Higher R → Trust model more
  - Lower R → Trust sensor more

---

## The Math (1D Simplified)

### Predict Step
```
x_predicted = x + velocity × dt      # State prediction
P_predicted = P + Q                  # Uncertainty grows!
```

### Update Step
```
K = P / (P + R)                      # Kalman gain
x = x + K × (measurement - x)        # Correct state
P = (1 - K) × P                      # Uncertainty shrinks!
```

### Kalman Gain Intuition
- K ≈ 1 → Trust measurement (sensor is accurate OR prediction is uncertain)
- K ≈ 0 → Trust prediction (sensor is noisy OR prediction is confident)

---

## Matrix Form (2D: Position + Velocity)

### State Vector
```
x = [position]
    [velocity]
```

### Predict
```
x = F × x              where F = [1  dt]
P = F × P × Fᵀ + Q               [0   1]
```

### Update
```
K = P × Hᵀ × (H × P × Hᵀ + R)⁻¹    # Kalman gain
x = x + K × (z - H × x)             # z = measurement
P = (I - K × H) × P
```

Where H = [1  0] (we only measure position, not velocity)

---

## Tuning Guide

### Problem: Filter is too slow to respond
**Solution:** Increase Q (process noise)

### Problem: Filter is too jumpy/noisy
**Solution:** Increase R (measurement noise)

### Problem: Filter drifts over time
**Solution:** 
- Ensure you have absolute reference sensor (camera, GPS)
- Check if sensors are providing updates
- Reduce time between updates if possible

### Problem: Filter diverges (goes crazy)
**Solution:**
- Check matrix dimensions
- Verify P stays positive definite
- Look for sensor outliers

---

## Code Quick Reference

```python
# Create filter
kf = KalmanFilter1D(
    initial_position=0.0,
    initial_uncertainty=1.0,
    process_noise=0.1,      # Q
    measurement_noise=1.0    # R
)

# Predict (when you have motion/IMU data)
kf.predict(velocity=1.0, dt=0.1)

# Update (when you have sensor measurement)
estimate, uncertainty, gain = kf.update(measurement=5.2)

# Get current state
position, uncertainty = kf.get_state()
```

---

## Key Takeaways

1. **Predict increases uncertainty** (we're less sure over time)
2. **Update decreases uncertainty** (sensors give us information)
3. **Kalman gain balances** prediction vs measurement
4. **Tuning Q and R** is crucial for real-world performance
5. **Sensor fusion** combines complementary sensor strengths

---

## Going Further

- **Extended Kalman Filter (EKF):** For nonlinear systems
- **Unscented Kalman Filter (UKF):** Better for highly nonlinear systems
- **Particle Filter:** For non-Gaussian distributions

### Resources
- kalmanfilter.net - Great visual explanations
- filterpy library - Production Python implementation
- "Probabilistic Robotics" book - Deep dive into robot state estimation
