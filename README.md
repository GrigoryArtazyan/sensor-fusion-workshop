# Sensor Fusion Workshop

> **A hands-on workshop for VISST School robotics students**

Learn how robots combine multiple sensors to know where they are - even when each sensor is imperfect!

![Sensor Fusion Simulation](images/simulation.png)

## The Big Idea

Every sensor has weaknesses:

| Sensor | Good At | Bad At |
|--------|---------|--------|
| **IMU/Encoders** | Smooth, fast readings | Drifts over time |
| **Camera/AprilTags** | Accurate position | Noisy, not always visible |

**Solution: Combine them!**

```
Low noise + High drift   +   High noise + No drift   =   Best of both!
    (IMU/Encoders)              (Camera/AprilTags)
```

## Quick Start

```bash
git clone https://github.com/GrigoryArtazyan/sensor-fusion-workshop.git
cd sensor-fusion-workshop
pip install -r requirements.txt

# Try these demos:
python src/simulation.py        # See fusion beat single sensors
python src/localization_game.py # Control a robot, watch sensors work
python src/particle_demo.py     # Visualize particle filter
```

## What's Included

| File | Description |
|------|-------------|
| `src/simulation.py` | Multi-sensor fusion demo with plots |
| `src/localization_game.py` | Interactive game - move with arrow keys! |
| `src/particle_demo.py` | Animated particle filter visualization |
| `src/kalman_filter.py` | Simple Kalman filter implementation |
| `docs/cheatsheet.md` | Quick reference with formulas |
| `docs/frc_integration.md` | WPILib code examples for FRC robots |

## Workshop Outline (1 hour)

1. **Why Sensor Fusion?** (10 min) - Why combining sensors works
2. **The Kalman Filter** (15 min) - Predict-update loop, brief math
3. **Hands-on Demos** (25 min) - Run simulations, experiment
4. **FRC Integration** (10 min) - How to use this on your robot

## The Kalman Filter in 30 Seconds

```
┌──────────┐         ┌──────────┐
│ PREDICT  │ ──────► │  UPDATE  │
│  (IMU)   │         │ (Camera) │
└──────────┘         └──────────┘
      ▲                    │
      └────────────────────┘
```

1. **Predict**: Use motion sensors → uncertainty grows
2. **Update**: See landmark → uncertainty shrinks
3. Repeat forever!

## Try These Experiments

In `src/simulation.py`, change these values and re-run:

```python
IMU_DRIFT = 0.12      # Make drift worse - watch fusion save the day!
CAMERA_INTERVAL = 25  # See fewer AprilTags - what happens?
CAMERA_NOISE = 1.0    # Noisier camera - IMU smooths it out
```

## For FRC Teams

WPILib has built-in sensor fusion:

```java
SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics, gyro.getRotation2d(), positions, startPose,
    VecBuilder.fill(0.1, 0.1, 0.1),  // Trust in odometry
    VecBuilder.fill(0.5, 0.5, 0.5)   // Trust in vision
);

// Every loop: update with odometry
poseEstimator.update(gyro.getRotation2d(), positions);

// When you see an AprilTag: add vision
poseEstimator.addVisionMeasurement(visionPose, timestamp);
```

## Resources

- [WPILib Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [AprilTag Docs](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)

---

*Workshop materials by Grigory Artazyan for VISST School*
