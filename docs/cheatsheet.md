# Sensor Fusion Cheatsheet

## The Core Concept

```
LOW NOISE + HIGH DRIFT     HIGH NOISE + NO DRIFT
   (IMU, Encoders)     +    (Camera, AprilTags)
          │                        │
          └──────────┬─────────────┘
                     ▼
           LOW NOISE + NO DRIFT
              (Fused result!)
```

## Kalman Filter Loop

```
PREDICT (motion) ──► UPDATE (sensor) ──► PREDICT ──► ...
   uncertainty ↑       uncertainty ↓
```

## Key Equations

```
Kalman Gain:  K = P / (P + R)
Update:       x = x + K × (measurement - x)
```

- **K ≈ 1**: Trust sensor more
- **K ≈ 0**: Trust prediction more

## Tuning

| Parameter | What it does |
|-----------|--------------|
| Q (process noise) | Higher = trust sensors more |
| R (measurement noise) | Higher = trust prediction more |

## Sensor Characteristics

| Sensor | Noise | Drift | Role |
|--------|-------|-------|------|
| IMU | Low | High | Predict |
| Encoders | Low | High | Predict |
| AprilTags | High | None | Update |
| Camera | High | None | Update |

## WPILib Equivalent

```java
VecBuilder.fill(0.1, 0.1, 0.1)  // stateStdDevs = Q
VecBuilder.fill(0.5, 0.5, 0.5)  // visionStdDevs = R
```
