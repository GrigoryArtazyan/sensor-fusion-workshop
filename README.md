# Sensor Fusion Workshop

> **A hands-on workshop for VISST School robotics students**

Learn how robots combine multiple sensors to know where they are!

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/5/5c/Kalman_filter_animation.gif/220px-Kalman_filter_animation.gif" alt="Kalman Filter Animation" width="300">
  <br>
  <em>Kalman filter tracking a moving object through noisy measurements</em>
</p>

## The Problem

Every sensor lies a little bit:

<p align="center">
  <img src="https://miro.medium.com/v2/resize:fit:1400/1*QhPC5BPn2G_ielY9FR0YCw.png" alt="Sensor Fusion Concept" width="500">
</p>

| Sensor | Strength | Weakness |
|--------|----------|----------|
| **IMU/Encoders** | Smooth, fast | Drifts over time |
| **Camera/AprilTags** | Accurate position | Noisy, sometimes blocked |

## The Solution: Sensor Fusion

**Sensor fusion** is a filtering technique that combines multiple sensors to get better accuracy than any single sensor alone.

### Why does combining sensors work?

1. **Complementary weaknesses** - IMU drifts but camera doesn't. Camera is noisy but IMU isn't. They cover each other's flaws!

2. **Noise cancellation** - When IMU says "10.2m" and camera says "9.8m", the average (10.0m) is often closer to truth.

3. **Redundancy** - If camera is blocked, IMU keeps tracking. When camera returns, it corrects the drift.

```
LOW NOISE + HIGH DRIFT      HIGH NOISE + NO DRIFT
   (IMU, Odometry)      +     (Camera, AprilTags)
         │                          │
         └────────────┬─────────────┘
                      ▼
            LOW NOISE + NO DRIFT
               (Best of both!)
```

<p align="center">
  <img src="images/simulation.png" alt="Fusion Demo" width="600">
  <br>
  <em>Our simulation: Fusion (red) beats IMU alone (cyan)</em>
</p>

## The Kalman Filter

A simple predict-update loop:

```
┌──────────────┐              ┌──────────────┐
│   PREDICT    │  ─────────►  │    UPDATE    │
│  Use IMU to  │              │ Correct with │
│  estimate    │              │   camera     │
│ (uncertainty │              │ (uncertainty │
│   grows)     │              │   shrinks)   │
└──────────────┘              └──────────────┘
        ▲                            │
        └────────────────────────────┘
```

### The Math (Brief!)

```python
# Predict: move estimate, uncertainty grows
x = x + velocity
P = P + Q

# Update: correct with measurement, uncertainty shrinks  
K = P / (P + R)           # Kalman gain (0 to 1)
x = x + K * (measurement - x)
P = P * (1 - K)
```

**K (Kalman gain)** decides who to trust:
- K ≈ 1 → Trust sensor more
- K ≈ 0 → Trust prediction more

## Quick Start

```bash
git clone https://github.com/GrigoryArtazyan/sensor-fusion-workshop.git
cd sensor-fusion-workshop
pip install numpy matplotlib

python src/simulation.py        # See fusion in action
python src/localization_game.py # Interactive game!
python src/particle_demo.py     # Particle filter demo
```

## Workshop Outline (1 hour)

| Part | Topic | Time |
|------|-------|------|
| 1 | Why Sensor Fusion? | 10 min |
| 2 | Kalman Filter basics | 15 min |
| 3 | Hands-on demos | 25 min |
| 4 | FRC integration | 10 min |

## Files

```
src/
├── simulation.py        # Simple fusion demo (60 lines!)
├── localization_game.py # Move robot with arrow keys
├── particle_demo.py     # Watch particles converge
└── kalman_filter.py     # Reusable filter class

docs/
├── cheatsheet.md        # Quick reference
└── frc_integration.md   # WPILib code examples
```

## For FRC Robots

WPILib does sensor fusion for you:

```java
// Create pose estimator
SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(...);

// Every loop: predict with odometry
estimator.update(gyro.getRotation2d(), modulePositions);

// When AprilTag visible: update with vision
estimator.addVisionMeasurement(visionPose, timestamp);
```

## Experiment!

Change these in `src/simulation.py`:

```python
IMU_DRIFT = 0.1       # Worse drift → fusion helps more
CAMERA_INTERVAL = 20  # Fewer updates → watch uncertainty grow
CAMERA_NOISE = 1.0    # Noisier → IMU smooths it out
```

---

<p align="center">
  <em>Workshop materials for VISST School by Grigory Artazyan</em>
</p>
