# FRC Integration Guide

How to use sensor fusion on your FRC robot with WPILib.

## WPILib Pose Estimators

WPILib provides built-in sensor fusion classes:

- `DifferentialDrivePoseEstimator`
- `SwerveDrivePoseEstimator`  
- `MecanumDrivePoseEstimator`

These fuse **odometry + gyro** with **AprilTag vision** automatically.

## Basic Usage

```cpp
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <wpi/array.h>

// Create pose estimator
frc::SwerveDrivePoseEstimator<4> poseEstimator{
    kinematics,
    gyro.GetRotation2d(),
    GetModulePositions(),
    frc::Pose2d{},
    {0.1, 0.1, 0.1},  // stateStdDevs - Trust in odometry
    {0.5, 0.5, 0.5}   // visionStdDevs - Trust in vision
};

// Every loop: PREDICT with odometry
poseEstimator.Update(gyro.GetRotation2d(), GetModulePositions());

// When AprilTag visible: UPDATE with vision
poseEstimator.AddVisionMeasurement(visionPose, timestamp);

// Get fused result
frc::Pose2d robotPose = poseEstimator.GetEstimatedPosition();
```

## Tuning the Standard Deviations

| Parameter | What it means | Adjustment |
|-----------|--------------|------------|
| `stateStdDevs` | Trust in odometry | Lower = trust odometry more |
| `visionStdDevs` | Trust in vision | Lower = trust AprilTags more |

**Starting values:**
- Odometry: `(0.1, 0.1, 0.1)` - meters, meters, radians
- Vision: `(0.5, 0.5, 0.5)`

**Tuning tips:**
- Robot drifts in auto? → Lower vision std devs
- Robot jumps when seeing tags? → Raise vision std devs
- Good encoders? → Lower state std devs

## Sensor Characteristics

| Sensor | Update Rate | Noise | Drift |
|--------|-------------|-------|-------|
| Wheel Encoders | 50-100 Hz | Low | High (slip) |
| NavX/Pigeon | 200 Hz | Low | Medium |
| AprilTags | 30-90 Hz | Medium | None |

## Common Issues

| Problem | Cause | Fix |
|---------|-------|-----|
| Robot "teleports" | Vision trusted too much | Raise vision std devs |
| Drifts in auto | Odometry trusted too much | Lower vision std devs |
| Jittery pose | Vision too noisy | Raise vision std devs |

## Resources

- [WPILib Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [PhotonVision Docs](https://docs.photonvision.org/)
- [AprilTag Intro](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)
