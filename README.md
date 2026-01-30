# Sensor Fusion Workshop

**A hands-on workshop for VISST School robotics students**

Learn how robots combine multiple sensors to know where they are!

---

## Try It Live

**[Launch Interactive Demo](https://sensorfusionworkshop.streamlit.app/)**

---

## The Problem

**Every sensor lies.** No sensor is perfect:

| Sensor | Strength | Weakness |
|--------|----------|----------|
| IMU / Gyro | Smooth, fast | Drifts over time |
| Wheel Encoders | Always available | Wheel slip |
| Camera / AprilTags | Absolute position | Noisy |

## The Solution

Combine sensors with **opposite weaknesses**:

| | IMU / Odometry | Camera / AprilTags |
|---|:---:|:---:|
| **Noise** | Low | High |
| **Drift** | High | None |
| **Combined** | **Low** | **None** |

---

## The Kalman Filter

```
PREDICT (odometry) ──► UPDATE (camera) ──► PREDICT ──► ...
  uncertainty ↑          uncertainty ↓
```

---

## Workshop Agenda (1 hour)

| Time | Topic |
|------|-------|
| 10 min | Why Sensor Fusion? |
| 15 min | The Kalman Filter |
| 25 min | Hands-on Demos |
| 10 min | FRC Integration |

---

## Resources

- [WPILib Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [AprilTag Intro](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)

---

*Workshop by Grigory Artazyan & Claude*
