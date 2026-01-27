# Sensor Fusion Workshop

<p align="center">
  <strong>A hands-on workshop for VISST School robotics students</strong><br>
  <em>Learn how robots combine multiple sensors to know where they are!</em>
</p>

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/5/5c/Kalman_filter_animation.gif/220px-Kalman_filter_animation.gif" alt="Kalman Filter Animation" width="300">
  <br>
  <sub>Kalman filter tracking through noisy measurements</sub>
</p>

---

## Overview

| | |
|---|---|
| **Duration** | 1 hour |
| **Audience** | Robotics students |
| **Prerequisites** | Basic Python |
| **Topics** | Sensor fusion, Kalman filter, Particle filter |

---

## The Problem

**Every sensor lies.** No sensor is perfect:

| Sensor | Strength | Weakness |
|--------|----------|----------|
| IMU / Gyro | Smooth, fast | Drifts over time |
| Wheel Encoders | Always available | Wheel slip |
| Camera / AprilTags | Absolute position | Noisy, needs line-of-sight |

---

## The Solution: Sensor Fusion

Combine sensors with **opposite weaknesses**:

| | IMU / Odometry | Camera / AprilTags |
|---|:---:|:---:|
| **Noise** | Low | High |
| **Drift** | High | None |
| **Combined** | **Low** | **None** |

<p align="center">
  <img src="images/localization_Gemini_Generate.png" alt="Sensor Fusion" width="600">
</p>

> **Blue** = Camera (noisy but accurate) | **Red** = IMU (smooth but drifts) | **Green** = Fused (best of both!)

---

## The Kalman Filter

A two-step loop that runs continuously:

```
┌─────────────┐              ┌─────────────┐
│   PREDICT   │  ─────────►  │   UPDATE    │
│  (Odometry) │              │  (Camera)   │
│ uncertainty │              │ uncertainty │
│    GROWS    │              │   SHRINKS   │
└─────────────┘              └─────────────┘
       ▲                            │
       └────────────────────────────┘
```

**PREDICT**: Use motion sensors - uncertainty grows  
**UPDATE**: Use position sensors - uncertainty shrinks

---

## Try It Live

**[Launch Interactive Demo](https://sensorfusionworkshop.streamlit.app/)**

Play with Kalman filters and sensor fusion in your browser - no installation required!

---

## Contents

```
sensor-fusion-workshop/
├── src/
│   ├── streamlit_app.py      # Interactive web demo (recommended)
│   ├── simulation.py         # Command-line demo
│   ├── localization_game.py  # Arrow-key robot game
│   └── particle_demo.py      # Particle filter visualization
├── docs/
│   ├── cheatsheet.md         # Quick reference
│   └── frc_integration.md    # WPILib code examples
└── images/
```

---

## Workshop Agenda

| Time | Topic |
|------|-------|
| 10 min | Why Sensor Fusion? |
| 15 min | The Kalman Filter |
| 25 min | Hands-on Demos |
| 10 min | FRC Integration |

---

## Resources

- [WPILib Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [AprilTag Documentation](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)

---

<p align="center">
  <strong>VISST School Workshop</strong><br>
  <sub>by Grigory Artazyan & Claude</sub>
</p>
