# Sensor Fusion Workshop

A hands-on workshop on sensor fusion and Kalman filtering for robotics teams.

## Quick Start

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Run the simulation:
   ```bash
   python sensor_fusion_simulation.py
   ```

3. Experiment with parameters at the top of `sensor_fusion_simulation.py`

## Workshop Files

| File | Description |
|------|-------------|
| `kalman_filter.py` | Well-commented Kalman filter implementation |
| `sensor_fusion_simulation.py` | Main simulation with visualization |
| `concepts_cheatsheet.md` | One-page reference with formulas |
| `workshop_slides.md` | Presentation outline (20 slides) |

## What You'll Learn

- Why combining sensors improves accuracy
- How the Kalman filter works (predict-update loop)
- Brief math behind the filter
- Hands-on tuning and experimentation

## Key Concepts

**Sensor Fusion:** Combining high-noise/low-drift sensors (camera) with low-noise/high-drift sensors (IMU) to get smooth AND accurate estimates.

**Kalman Filter:** An optimal estimation algorithm that automatically balances trust between predictions and measurements based on their uncertainties.

## Experiments to Try

1. Increase `IMU_DRIFT_RATE` to see how fusion handles worse drift
2. Increase `CAMERA_UPDATE_INTERVAL` to see uncertainty grow between updates
3. Tune `KF_PROCESS_NOISE` (Q) and `KF_MEASUREMENT_NOISE` (R) to find the best balance

## Resources

- [kalmanfilter.net](https://www.kalmanfilter.net) - Visual explanations
- filterpy library - Production Python implementation
- "Probabilistic Robotics" book - Deep dive
