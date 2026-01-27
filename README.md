# Sensor Fusion Workshop

> **A hands-on workshop for VISST School robotics students**

Learn how robots combine multiple sensors to accurately know where they are - even when each sensor is imperfect!

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/5/5c/Kalman_filter_animation.gif/220px-Kalman_filter_animation.gif" alt="Kalman Filter Animation" width="280">
</p>

---

## The Problem: Every Sensor Lies

No sensor is perfect. Each has strengths and weaknesses:

| Sensor | Strength | Weakness |
|--------|----------|----------|
| **IMU / Gyro** | Smooth, fast readings | Drifts over time |
| **Wheel Encoders** | Always available | Wheel slip causes errors |
| **Camera / AprilTags** | Accurate absolute position | Noisy, requires line-of-sight |

**The challenge:** How do we get accurate position when every sensor has flaws?

---

## The Solution: Sensor Fusion

**Sensor fusion** combines multiple sensors to get better accuracy than any single sensor alone.

### Why Combining Sensors Works

1. **Complementary weaknesses** - IMU drifts but camera doesn't. Camera is noisy but IMU isn't. They cover each other's flaws.

2. **Noise reduction** - When sensors disagree, averaging often gets closer to truth.

3. **Redundancy** - If one sensor fails, others keep working.

### The Key Insight

Fuse sensors with **opposite** characteristics:

| | IMU / Odometry | Camera / AprilTags |
|---|---|---|
| **Noise** | Low (smooth) | High (jumpy) |
| **Drift** | High (accumulates) | None (absolute) |

Combined result: **Low noise AND no drift!**

<p align="center">
  <img src="images/localization_Gemini_Generate.png" alt="Sensor Fusion Visualization" width="650">
</p>

**What this shows:**
- **Blue (Camera)**: Noisy but centered on truth - no drift
- **Red (IMU)**: Smooth but drifts away over time  
- **Green (Fused)**: Smooth AND accurate - best of both!

---

## The Kalman Filter

The Kalman filter is the algorithm that makes sensor fusion work. It runs a continuous two-step loop:

**1. PREDICT** - Use motion sensors (IMU, encoders) to estimate where you moved
   - Uncertainty *grows* (things can go wrong)

**2. UPDATE** - Use position sensors (camera, AprilTags) to correct the estimate
   - Uncertainty *shrinks* (we got real information)

The filter automatically decides how much to trust each sensor based on their reliability.

<p align="center">
  <img src="images/simulation.png" alt="Fusion Demo" width="550">
</p>

---

## Workshop Outline

| Part | Topic | Duration |
|------|-------|----------|
| 1 | Why Sensor Fusion? | 10 min |
| 2 | The Kalman Filter | 15 min |
| 3 | Hands-on Python Demos | 25 min |
| 4 | FRC Integration | 10 min |

**Total: 1 hour**

---

## What's Included

| Folder | Contents |
|--------|----------|
| `src/` | Python simulations and interactive demos |
| `docs/` | Cheatsheet and FRC integration guide |
| `images/` | Visualizations |

### Interactive Demos

- **streamlit_app.py** - Web-based interactive demo (recommended!)
- **simulation.py** - Command-line fusion demo
- **localization_game.py** - Control a robot with arrow keys
- **particle_demo.py** - Particle filter visualization

---

## Quick Start

```
git clone https://github.com/GrigoryArtazyan/sensor-fusion-workshop.git
cd sensor-fusion-workshop
pip install -r requirements.txt

# Run the interactive web app (recommended!)
streamlit run src/streamlit_app.py

# Or run individual demos
python src/simulation.py
```

---

## Resources

- [WPILib Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [AprilTag Documentation](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html)

---

<p align="center">
  <em>Workshop materials for VISST School</em><br>
  <em>by Grigory Artazyan & Claude</em>
</p>
