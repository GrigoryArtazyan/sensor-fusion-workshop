# Sensor Fusion Cheatsheet

## The Core Concept

```
LOW NOISE + HIGH DRIFT     HIGH NOISE + NO DRIFT
   (IMU, Encoders)     +    (Camera, AprilTags)
          │                        │
          └──────────┬─────────────┘
                     ▼
           LOW NOISE + NO DRIFT
              (Fused result)
```

![Sensor Fusion Path](../images/circle_path.png)
- IMU provides smooth, high-frequency updates but drifts over time
- Camera gives absolute position but with measurement noise
- Fusion uses algorithms like Kalman filtering to combine both
- Result: Smooth tracking without long-term drift

## Kalman Filter Loop

This loop repeats continuously, balancing predictions with sensor measurements
```
PREDICT (motion) ──► UPDATE (sensor) ──► PREDICT ──► ...
   uncertainty ↑       uncertainty ↓
```
- PREDICT: Robot moves forward → uncertainty increases (we're less sure where it is)

- UPDATE: Sensor measures position → uncertainty decreases (we're more sure where it is)


## Key Equations

```
Kalman Gain:  K = P / (P + R)
> Kalman Gain: Balances trust between prediction (P) and sensor (R)
----
Update:       x = x + K × (measurement - x)
> State Update: Moves estimate toward measurement by K percent
```
- **x (State):** The value you are trying to estimate (e.g., your robot's actual position).
- **z (Measurement):** The "noisy" data coming directly from your sensor.
- **K (Kalman Gain):** The **weighting factor** (0 to 1) that decides which source to trust more.
- **P (Prediction Uncertainty):** How much you trust your **model/math** (the "internal guess").
- **R (Measurement Noise):** How much you trust your **sensor/hardware** (the "external input").

## The Two-Step Logic

The filter runs in a loop: first **predict**, then **correct** using sensor data.

### 1. Calculating the Gain (K)

- **High P (low trust in model):** K → 1 → rely more on the sensor.
- **High R (low trust in sensor):** K → 0 → rely more on the prediction.

### 2. Updating the State (x)

- **Innovation:** `z - x` → the difference between the sensor reading and your current estimate.
- **Update rule:**  


## Sensor Characteristics

| Sensor | Noise | Drift | Role |
|--------|-------|-------|------|
| IMU / Gyro | Low | High | Predict |
| Encoders / Odometry | Low | High | Predict |
| AprilTags / Camera | High | None | Update |

