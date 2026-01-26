# Sensor Fusion: Why It Works

## What is Filtering?

In signal processing, **filtering** means extracting useful information from noisy data. Common filtering approaches:

| Filter Type | How It Works | Limitations |
|-------------|--------------|-------------|
| Moving Average | Average recent readings | Adds lag, single sensor only |
| Low-Pass Filter | Smooth out high-frequency noise | Can't handle drift |
| Complementary Filter | Weighted blend of two sensors | Only works for specific pairs |
| **Kalman Filter** | Optimal fusion based on uncertainty | More complex, but most flexible |

**Sensor fusion** is filtering that combines multiple sensor sources.

## Why Combining Sensors Improves Accuracy

### 1. Complementary Error Characteristics

Different sensors fail in different ways:

```
Odometry:     ████████████████░░░░░░░░░░░░░░░░  (good short-term, drifts)
AprilTags:    ░░░░████░░░░████░░░░████░░░░████  (intermittent, but accurate)
FUSED:        ████████████████████████████████  (best of both!)
```

- **Odometry** accumulates error over time (integration drift)
- **Vision** has no drift but is noisy and sometimes unavailable
- **Fusion** uses odometry for smooth tracking, vision to reset drift

### 2. Statistical Noise Reduction

When combining independent measurements, noise partially cancels:

```
Sensor A reads:  10.3  (true value: 10.0, error: +0.3)
Sensor B reads:   9.8  (true value: 10.0, error: -0.2)
Average:         10.05  (error: +0.05) ← closer to truth!
```

**Mathematically**: Combining N independent measurements with variance σ² gives combined variance σ²/N.

This is why the Kalman filter weights measurements by their uncertainty - it's finding the optimal combination.

### 3. Different Update Rates

Sensors operate at different speeds:

| Sensor | Update Rate | Best For |
|--------|-------------|----------|
| Encoders | 50-200 Hz | Frame-to-frame motion |
| Gyro | 100-400 Hz | Rotation tracking |
| AprilTags | 30-90 Hz | Absolute position correction |

**Fast sensors** (encoders, gyro) → Use for prediction between updates  
**Slower sensors** (vision) → Use to correct accumulated error

### 4. Graceful Degradation

When one sensor fails, others keep working:

- **Lost sight of AprilTags?** → Odometry continues tracking
- **Wheel slip on carpet?** → AprilTags correct when visible
- **Gyro drifting?** → Vision heading resets it

## The Kalman Filter Approach

The Kalman filter is optimal because it:

1. **Tracks uncertainty**: Knows how confident to be in each estimate
2. **Weights by reliability**: Trusts accurate sensors more
3. **Propagates uncertainty**: Knows prediction gets worse over time
4. **Updates optimally**: Mathematically minimizes expected error

### The Predict-Update Cycle

```
        ┌─────────────────────────────────────┐
        │         PREDICT (Odometry)          │
        │  • Use motion model to estimate     │
        │  • Uncertainty INCREASES            │
        │  • "I think I moved 0.5m forward"   │
        └──────────────────┬──────────────────┘
                           │
                           ▼
        ┌─────────────────────────────────────┐
        │         UPDATE (AprilTag)           │
        │  • Correct with measurement         │
        │  • Uncertainty DECREASES            │
        │  • "Tag says I'm at (3.2, 1.5)"     │
        └──────────────────┬──────────────────┘
                           │
                           ▼
                    [Repeat forever]
```

### Why This Works

Between vision updates, uncertainty grows:
```
Time:        t=0    t=1    t=2    t=3    t=4    t=5
Uncertainty: 0.1 → 0.15 → 0.2 → 0.25 → 0.3 → 0.35
             (growing because we're just predicting)
```

When AprilTag is seen, uncertainty drops:
```
Time:        t=5    t=6
Uncertainty: 0.35 → 0.12  ← Vision update!
             (reset because we got real information)
```

This "sawtooth" pattern is exactly what you see in the simulation.

## Practical Example: Robot Auto

Imagine a 15-second autonomous routine:

**Without fusion (odometry only):**
```
Start:    Position error = 0 cm
After 5s: Position error = 5 cm (drift accumulating)
After 10s: Position error = 15 cm (getting worse)
After 15s: Position error = 30 cm (missed the target!)
```

**With fusion (odometry + AprilTags):**
```
Start:    Position error = 0 cm
After 5s: Position error = 3 cm → sees tag → corrects to 1 cm
After 10s: Position error = 4 cm → sees tag → corrects to 1 cm
After 15s: Position error = 2 cm (stayed accurate!)
```

## Key Takeaways

1. **Single sensors have weaknesses** - no sensor is perfect
2. **Different sensors fail differently** - this is what makes fusion work
3. **Fusion combines strengths** - smooth + accurate + reliable
4. **Kalman filter is optimal** - automatically balances trust
5. **Uncertainty matters** - knowing confidence enables smart decisions

## Connection to WPILib

WPILib's `PoseEstimator` classes implement exactly this:
- `update()` → Predict step (odometry)
- `addVisionMeasurement()` → Update step (AprilTags)
- `stateStdDevs` / `visionStdDevs` → Tuning parameters (like Q and R)
