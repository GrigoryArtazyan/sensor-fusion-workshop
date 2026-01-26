# Sensor Fusion Workshop - Presentation Outline

---

## Slide 1: Title

**Sensor Fusion & Kalman Filtering**

*Getting the best from your robot's sensors*

Workshop for [Team Name]

---

## Slide 2: What We'll Cover

1. Why sensor fusion matters
2. Understanding sensor characteristics
3. Filter types overview (focus on Kalman)
4. The math behind Kalman filters
5. Hands-on: Python simulation

**Goal:** Understand how to combine sensors for better state estimation

---

## Slide 3: The Problem - Single Sensors Have Limitations

**IMU (Accelerometer/Gyroscope)**
- Fast updates, smooth readings
- But: Drifts over time (errors accumulate)

**Camera/Vision**
- Absolute position reference
- But: Noisy, affected by lighting, slower

**Wheel Odometry**
- Simple, always available
- But: Wheel slip causes errors

*No single sensor is perfect!*

---

## Slide 4: The Solution - Sensor Fusion

**Combine multiple sensors to get the best of each**

```
IMU (low noise, high drift)
         \
          → FUSION → Best Estimate
         /
Camera (high noise, low drift)
```

**Key insight:**
- IMU: Great for short-term, smooth tracking
- Camera: Great for long-term, drift correction
- Together: Smooth AND accurate!

---

## Slide 5: Sensor Characteristics Table

| Sensor | Noise | Drift | Update Rate | Best For |
|--------|-------|-------|-------------|----------|
| IMU | Low | High | Fast | Short-term motion |
| Camera | High | Low | Slow | Absolute position |
| Odometry | Medium | High | Fast | Relative motion |
| LiDAR | Low | Low | Medium | Distance/mapping |

**Your robot:** Identify which sensors you have and their characteristics

---

## Slide 6: Filter Types Overview

**1. Moving Average**
- Simple smoothing
- Adds lag, doesn't combine sensors well

**2. Complementary Filter**
- Weighted blend of two sensors
- Simple but limited

**3. Kalman Filter** ← Our focus!
- Optimal estimation
- Handles uncertainty mathematically
- Works with multiple sensors

**4. Extended Kalman Filter (EKF)**
- Kalman for nonlinear systems
- What most robots actually use

---

## Slide 7: Kalman Filter - The Core Idea

**Two-step loop that runs continuously:**

```
┌─────────────┐         ┌─────────────┐
│   PREDICT   │ ──────▶ │   UPDATE    │
│ "Where do   │         │ "Correct    │
│  we think   │         │  with       │
│  we are?"   │         │  sensor"    │
└─────────────┘         └─────────────┘
       ▲                       │
       └───────────────────────┘
```

- **Predict:** Use motion model → Uncertainty GROWS
- **Update:** Use sensor data → Uncertainty SHRINKS

---

## Slide 8: Kalman Filter - Intuition

**You have:**
- A belief about your state (position, velocity)
- Uncertainty about that belief

**Prediction:**
- "Based on physics, I think I moved here"
- But uncertainty increases (things could go wrong)

**Measurement:**
- "My sensor says I'm here"
- Use this to correct prediction
- Uncertainty decreases (gained information)

**The filter automatically balances which to trust more!**

---

## Slide 9: The Math - Variables

**State:**
- `x` = What we're estimating (position, velocity, etc.)
- `P` = How uncertain we are (covariance)

**Noise parameters (you tune these!):**
- `Q` = Process noise (trust in model)
- `R` = Measurement noise (trust in sensor)

**The key output:**
- `K` = Kalman gain (how much to trust measurement vs prediction)

---

## Slide 10: The Math - Equations

**Predict Step:**
```
x_predicted = F × x + B × u     (state prediction)
P_predicted = F × P × Fᵀ + Q    (uncertainty grows)
```

**Update Step:**
```
K = P × Hᵀ × (H × P × Hᵀ + R)⁻¹  (Kalman gain)
x = x + K × (z - H × x)          (correct state)
P = (I - K × H) × P              (uncertainty shrinks)
```

**Don't memorize this!** Understand the concept:
- Kalman gain balances prediction vs measurement
- High K → trust sensor more
- Low K → trust prediction more

---

## Slide 11: Kalman Gain Intuition

**K = P / (P + R)** (simplified 1D)

**When P is large (uncertain prediction):**
- K approaches 1
- Trust the measurement more

**When R is large (noisy sensor):**
- K approaches 0  
- Trust the prediction more

**The filter automatically figures out the optimal balance!**

---

## Slide 12: Tuning Tips

**Process Noise (Q) - Trust in your model**
- Increase Q → Filter follows measurements more closely
- Decrease Q → Filter is smoother, trusts physics model

**Measurement Noise (R) - Trust in your sensor**
- Increase R → Filter ignores noisy measurements
- Decrease R → Filter reacts quickly to measurements

**Start with sensor datasheets, then tune experimentally!**

---

## Slide 13: Hands-On Demo Setup

**Files in your workshop folder:**

1. `kalman_filter.py` - Kalman filter implementation
2. `sensor_fusion_simulation.py` - Main simulation
3. `concepts_cheatsheet.md` - Quick reference

**To run:**
```bash
python sensor_fusion_simulation.py
```

**Requirements:**
- Python 3
- numpy
- matplotlib

---

## Slide 14: What the Simulation Shows

**Scenario:** Robot moving in 1D with two sensors

1. **Ground truth** - Actual position (black line)
2. **IMU only** - Drifts over time (blue line)
3. **Camera readings** - Noisy dots (green)
4. **Kalman fusion** - Best estimate (red line)

**Watch how:**
- IMU drifts away from truth
- Camera corrections pull it back
- Fusion stays close to truth!

---

## Slide 15: Experiments to Try

**1. Break the IMU**
```python
IMU_DRIFT_RATE = 0.05  # Increase from 0.01
```
See how fusion still works!

**2. Slow down camera**
```python
CAMERA_UPDATE_INTERVAL = 20  # Increase from 5
```
Watch uncertainty grow between updates

**3. Tune the filter**
```python
KF_PROCESS_NOISE = 0.1      # Try different values
KF_MEASUREMENT_NOISE = 0.5   # Try different values
```

---

## Slide 16: Key Takeaways

1. **Single sensors have weaknesses** - drift, noise, blind spots

2. **Sensor fusion combines strengths** - smooth + accurate

3. **Kalman filter is optimal** - mathematically balances uncertainty

4. **Predict increases uncertainty** - we're less sure over time

5. **Update decreases uncertainty** - sensors give information

6. **Tuning Q and R is crucial** - experiment!

---

## Slide 17: For Your Robot

**Questions to consider:**

1. What sensors do you have available?
2. What state do you need to estimate?
3. What are each sensor's characteristics?
4. How fast do you need updates?

**Starting point:**
- Identify your "IMU-like" sensors (fast, drifty)
- Identify your "camera-like" sensors (slow, absolute)
- Start with 1D Kalman, then expand

---

## Slide 18: Resources

**Learn more:**
- kalmanfilter.net - Great visual explanations
- "Probabilistic Robotics" book
- filterpy Python library

**Code from today:**
- All files in workshop folder
- Well-commented for reference

**Questions?**

---

## Slide 19: Workshop Exercises

**Exercise 1:** Run the simulation, observe the plots

**Exercise 2:** Increase IMU drift - how does fusion handle it?

**Exercise 3:** Tune Q and R - find the best balance

**Exercise 4:** Change camera update rate - what happens?

**Challenge:** Add a third sensor type to the simulation

---

## Slide 20: Thank You!

**Sensor Fusion Workshop**

Key formula to remember:
```
Optimal Estimate = Prediction + Kalman_Gain × (Measurement - Prediction)
```

*The Kalman gain automatically balances trust based on uncertainty*

Good luck with your robot!
