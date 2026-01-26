# Sensor Fusion Workshop for FRC

Learn how to combine multiple sensors using a Kalman filter for better robot localization.

## Why Sensor Fusion?

In FRC, accurate robot positioning is critical for autonomous routines and game piece alignment. Single sensors have limitations:

| Sensor | Strengths | Weaknesses |
|--------|-----------|------------|
| **IMU/Gyro** | Fast, smooth, precise short-term | Drifts over time |
| **AprilTags** | Absolute field position, no drift | Noisy, slower, requires line-of-sight |
| **Wheel Odometry** | Always available, fast | Wheel slip, accumulates error |

**Solution**: Fuse them together using a Kalman filter!

## Quick Start

```bash
git clone https://github.com/GrigoryArtazyan/sensor-fusion-workshop.git
cd sensor-fusion-workshop
pip install -r requirements.txt
python src/simulation.py
```

## Repository Structure

```
sensor-fusion-workshop/
├── src/
│   ├── kalman_filter.py   # Kalman filter implementation
│   └── simulation.py      # Interactive demo
├── docs/
│   ├── cheatsheet.md      # Quick reference
│   └── frc_integration.md # WPILib integration guide
├── requirements.txt
└── README.md
```

## Workshop Outline (1 hour)

### Part 1: Why Sensor Fusion? (10 min)
- Single sensor limitations
- Complementary sensor characteristics
- Real FRC examples

### Part 2: The Kalman Filter (15 min)
- Predict-update loop
- Brief math overview
- Tuning Q and R parameters

### Part 3: Hands-on Simulation (25 min)
- Run the Python simulation
- Experiment with parameters
- See fusion outperform single sensors

### Part 4: FRC Integration (10 min)
- WPILib pose estimation classes
- AprilTags + odometry fusion
- Next steps for your robot

## Key Concepts

### The Kalman Filter Loop

```
┌──────────┐         ┌──────────┐
│ PREDICT  │ ──────► │  UPDATE  │
│ (Odometry)│         │(AprilTag)│
└──────────┘         └──────────┘
      ▲                    │
      └────────────────────┘
```

1. **Predict**: Use wheel odometry/IMU → uncertainty grows
2. **Update**: See AprilTag → uncertainty shrinks

### Core Equations

```
Kalman Gain:  K = P / (P + R)
State Update: x = x + K × (measurement - x)
```

- **K ≈ 1**: Trust the measurement (AprilTag visible, good pose)
- **K ≈ 0**: Trust the prediction (no tags visible, rely on odometry)

## Exercises

Modify `src/simulation.py`:

1. **Increase drift** (`IMU_DRIFT = 0.1`) - simulates encoder slip
2. **Reduce camera updates** (`CAMERA_INTERVAL = 20`) - like losing AprilTag sight
3. **Tune the filter** - adjust Q and R for best performance

## FRC Resources

- [WPILib Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [AprilTag Documentation](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)
- [Odometry Classes](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/index.html)

## WPILib Integration

WPILib provides built-in pose estimators that do sensor fusion for you:

```java
// Java example
SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics,
    gyro.getRotation2d(),
    modulePositions,
    initialPose,
    stateStdDevs,    // Trust in odometry (like Q)
    visionStdDevs    // Trust in vision (like R)
);

// In periodic: add odometry
poseEstimator.update(gyro.getRotation2d(), modulePositions);

// When AprilTag detected: add vision measurement
poseEstimator.addVisionMeasurement(visionPose, timestamp);
```

The `stateStdDevs` and `visionStdDevs` are like our Q and R parameters!
