# Workshop Teaching Guide

## Recommended Order

| # | File | Time | Purpose |
|---|------|------|---------|
| 1 | `kalman_filter.py` | 5 min | Introduce the core concept |
| 2 | `simulation.py` | 5 min | Visual proof that fusion works |
| 3 | `streamlit_app.py` | 15 min | Students explore interactively |
| 4 | `localization_game.py` | 10 min | Hands-on game (most engaging) |
| 5 | `particle_demo.py` | 5 min | Optional - different approach |

---

## 1. `kalman_filter.py` - Start Here

**Run:** `python src/kalman_filter.py`

**Teaching points:**
- Walk through the class structure (only ~60 lines)
- Highlight the two key methods: `predict()` and `update()`
- Show output table - watch the Kalman Gain (K) change

**Key questions to ask:**
- "What happens to uncertainty during predict?" → It grows
- "What happens to uncertainty during update?" → It shrinks
- "When is K high vs low?" → High K = trust sensor, Low K = trust prediction

**Hint:** Lines 34-56 are the entire algorithm. It's just 10 lines of math!

---

## 2. `simulation.py` - Proof It Works

**Run:** `python src/simulation.py`

**Teaching points:**
- Show the printed RMSE comparison (fused always beats odometry)
- Point to the plot: blue drifts away, green is noisy, red stays close

**Experiment:** Change line 26 `ODOM_DRIFT = 0.1` → Try 0.05 and 0.2

**Key insight:** "Low noise + high drift" + "High noise + no drift" = "Low noise + no drift"

---

## 3. `streamlit_app.py` - Student Exploration

**Run:** `streamlit run src/streamlit_app.py` or use https://sensorfusionworkshop.streamlit.app/

**This is where students spend most time!**

**Guided experiments:**

| Change This | What Happens |
|-------------|--------------|
| Increase odometry drift | Blue line diverges more, fusion helps more |
| Increase camera noise | Green dots scatter more, but fusion still works |
| Increase camera interval | Fewer green dots, uncertainty grows between updates |
| Q too high | Filter overreacts, jumpy estimate |
| R too high | Filter ignores camera, drifts like odometry |

**Challenge question:** "Can you make the fused estimate worse than odometry alone?" → Very hard! That's the point.

---

## 4. `localization_game.py` - Game Time

**Run:** `python src/localization_game.py`

**Controls:**
- Arrow keys: move robot
- Space: toggle sensor display
- R: reset
- 1/2/3: change filter trust level

**Teaching activity:**
1. Move in a big square
2. Watch cyan (IMU) drift away from black (truth)
3. Watch red (fused) stay close because of camera corrections
4. Press 1 (low camera trust) - red follows IMU more
5. Press 3 (high camera trust) - red jumps to camera readings

**Key observation:** Red circle (uncertainty) shrinks when camera is visible!

---

## 5. `particle_demo.py` - Alternative Approach (Optional)

**Run:** `python src/particle_demo.py`

**When to use:**
- If students finish early
- If someone asks "are there other filters?"
- Shows non-Gaussian approach

**Key difference from Kalman:**
- Kalman: tracks mean + variance (Gaussian assumption)
- Particle: tracks many "guesses", no Gaussian assumption
- Both do: predict → update → estimate

---

## Quick Reference for Students

```
PREDICT (odometry step):
  x = x + velocity × dt
  uncertainty GROWS

UPDATE (camera step):
  K = P / (P + R)          ← Kalman Gain
  x = x + K × (measurement - x)
  uncertainty SHRINKS

Key insight:
  K ≈ 1  →  Trust camera
  K ≈ 0  →  Trust prediction
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| matplotlib window doesn't appear | Run from terminal, not Jupyter |
| Streamlit won't start | `pip install streamlit` first |
| Game controls don't work | Click on the plot window first |
