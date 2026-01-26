# FRC Integration Guide

How to apply sensor fusion concepts to your FRC robot using WPILib.

## WPILib Pose Estimators

WPILib provides pose estimator classes that implement Kalman-filter-based sensor fusion:

- `DifferentialDrivePoseEstimator`
- `SwerveDrivePoseEstimator`  
- `MecanumDrivePoseEstimator`

These fuse **wheel odometry + gyro** with **vision measurements** (AprilTags).

## Basic Setup (Java)

```java
// 1. Create the pose estimator
SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics,
    gyro.getRotation2d(),
    getModulePositions(),
    new Pose2d(),  // Starting pose
    VecBuilder.fill(0.1, 0.1, 0.1),    // State std devs (x, y, theta)
    VecBuilder.fill(0.5, 0.5, 0.5)     // Vision std devs (x, y, theta)
);

// 2. Update with odometry every loop (50Hz)
public void periodic() {
    poseEstimator.update(
        gyro.getRotation2d(),
        getModulePositions()
    );
}

// 3. Add vision measurements when available
public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
}

// 4. Get the fused pose
public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
}
```

## Understanding Standard Deviations

The std dev parameters are like Q and R from the workshop:

| Parameter | Meaning | Effect |
|-----------|---------|--------|
| `stateStdDevs` | Trust in odometry | Lower = trust odometry more |
| `visionStdDevs` | Trust in vision | Lower = trust AprilTags more |

### Tuning Tips

**Start with these values:**
```java
// Odometry: trust it moderately
VecBuilder.fill(0.1, 0.1, 0.1)  // meters, meters, radians

// Vision: trust it less (AprilTags can be noisy)
VecBuilder.fill(0.5, 0.5, 0.5)
```

**Adjust based on testing:**
- Robot drifting? Lower vision std devs
- Jumpy when seeing tags? Raise vision std devs
- Good encoders? Lower state std devs

## AprilTag Pose Estimation

### Getting Pose from AprilTags

```java
// Using PhotonVision
var result = camera.getLatestResult();
if (result.hasTargets()) {
    var target = result.getBestTarget();
    
    // Get camera-to-target transform
    Transform3d camToTarget = target.getBestCameraToTarget();
    
    // Calculate field-relative robot pose
    Pose3d tagPose = fieldLayout.getTagPose(target.getFiducialId()).get();
    Pose3d cameraPose = tagPose.transformBy(camToTarget.inverse());
    Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());
    
    // Add to pose estimator
    poseEstimator.addVisionMeasurement(
        robotPose.toPose2d(),
        result.getTimestampSeconds()
    );
}
```

### Handling Ambiguity

AprilTag pose estimation can have ambiguity (two possible poses). Solutions:

1. **Reject unlikely poses** (outside field, in the air)
2. **Use multiple tags** when visible
3. **Compare to odometry** - reject if too far from expected
4. **Use pose ambiguity metric** - reject high-ambiguity results

```java
// Reject if ambiguity is too high
if (target.getPoseAmbiguity() < 0.2) {
    poseEstimator.addVisionMeasurement(pose, timestamp);
}
```

## Sensor Characteristics for FRC

| Sensor | Update Rate | Noise | Drift |
|--------|-------------|-------|-------|
| Wheel Encoders | 50-100 Hz | Low | High (slip) |
| NavX/Pigeon Gyro | 200 Hz | Low | Medium |
| AprilTags | 30-90 Hz | Medium-High | None |
| Limelight | 90 Hz | Medium | None |

## Architecture Diagram

```
┌─────────────┐     ┌─────────────┐
│   Encoders  │     │    Gyro     │
└──────┬──────┘     └──────┬──────┘
       │                   │
       └─────────┬─────────┘
                 ▼
         ┌──────────────┐
         │   Odometry   │
         │  (predict)   │
         └──────┬───────┘
                │
                ▼
    ┌───────────────────────┐
    │   Pose Estimator      │
    │   (Kalman Filter)     │ ◄─── AprilTag Pose
    └───────────┬───────────┘      (update)
                │
                ▼
        ┌───────────────┐
        │  Fused Pose   │
        │  (best estimate)
        └───────────────┘
```

## Common Issues

### Robot "teleports" when seeing AprilTags
- Vision std devs too low (trusting vision too much)
- Bad camera calibration
- Pose ambiguity not being filtered

### Robot drifts during auto
- State std devs too low (trusting odometry too much)
- Wheel slip not accounted for
- Not seeing enough AprilTags

### Pose is jittery
- Vision std devs too low
- Camera framerate too high without filtering
- Multiple tags giving conflicting poses

## Resources

- [WPILib Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [PhotonVision Docs](https://docs.photonvision.org/)
- [AprilTag Intro](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)
