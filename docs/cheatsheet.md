# Kalman Filter Cheatsheet for FRC

## The Predict-Update Loop

```
┌──────────────┐         ┌──────────────┐
│   PREDICT    │ ──────► │    UPDATE    │
│  (Odometry)  │         │  (AprilTag)  │
│ uncertainty↑ │         │ uncertainty↓ │
└──────────────┘         └──────────────┘
        ▲                       │
        └───────────────────────┘
```

## Key Variables

| Variable | WPILib Equivalent | Meaning |
|----------|-------------------|---------|
| `x` | Pose2d | Position estimate |
| `P` | (internal) | Uncertainty |
| `Q` | stateStdDevs | Trust in odometry |
| `R` | visionStdDevs | Trust in vision |
| `K` | (internal) | Kalman gain |

## The Equations

**Predict (odometry update):**
```
x = x + velocity × dt
P = P + Q              ← uncertainty grows
```

**Update (AprilTag visible):**
```
K = P / (P + R)        ← Kalman gain
x = x + K × (vision - x)
P = (1 - K) × P        ← uncertainty shrinks
```

## Kalman Gain Intuition

- **K ≈ 1**: Trust AprilTag (haven't seen tags in a while)
- **K ≈ 0**: Trust odometry (just updated with vision)

## FRC Sensor Characteristics

| Sensor | Noise | Drift | Rate | Role in Fusion |
|--------|-------|-------|------|----------------|
| **IMU/Gyro** | Very Low | High | 200+ Hz | Predict (motion) |
| **Encoders** | Low | High (slip) | 50-100 Hz | Predict (motion) |
| **AprilTags** | Medium | None | 30-90 Hz | Update (absolute) |
| **LiDAR** | Low | None | 10-40 Hz | Update (absolute) |

**Key insight:** Fuse drifting sensors (IMU, encoders) with absolute sensors (AprilTags, LiDAR)

## WPILib Standard Deviations

```java
// Odometry trust (state)
VecBuilder.fill(0.1, 0.1, 0.1)  // x, y, theta

// Vision trust
VecBuilder.fill(0.5, 0.5, 0.5)  // x, y, theta
```

**Tuning:**
- Robot drifts → Lower vision std devs
- Jumpy with tags → Raise vision std devs
- Good encoders → Lower state std devs

## Quick Reference

| Problem | Solution |
|---------|----------|
| Robot teleports when seeing tags | Raise vision std devs |
| Robot drifts in auto | Lower vision std devs |
| Pose is jittery | Raise vision std devs |
| Slow to correct | Lower vision std devs |
