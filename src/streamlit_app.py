"""
Sensor Fusion Interactive Demo
==============================
A Streamlit app to explore sensor fusion, Kalman filtering, and particle filters.

Run: streamlit run src/streamlit_app.py
"""

import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

st.set_page_config(page_title="Sensor Fusion Workshop", page_icon="🤖", layout="wide")

# === KALMAN FILTER CLASS ===
class KalmanFilter:
    def __init__(self, Q, R):
        self.x = 0
        self.P = 1.0
        self.Q = Q
        self.R = R
        self.history = {'x': [], 'P': [], 'K': []}
    
    def predict(self, velocity):
        self.x += velocity
        self.P += self.Q
        
    def update(self, measurement):
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        self.history['K'].append(K)
        return K
    
    def record(self):
        self.history['x'].append(self.x)
        self.history['P'].append(self.P)

# === PARTICLE FILTER CLASS ===
class ParticleFilter:
    def __init__(self, n_particles, field_size):
        self.n = n_particles
        self.field_size = field_size
        self.particles_x = np.random.uniform(0, field_size, n_particles)
        self.particles_y = np.random.uniform(0, field_size, n_particles)
        self.weights = np.ones(n_particles) / n_particles
    
    def predict(self, dx, dy, noise):
        self.particles_x += dx + np.random.normal(0, noise, self.n)
        self.particles_y += dy + np.random.normal(0, noise, self.n)
        self.particles_x = np.clip(self.particles_x, 0, self.field_size)
        self.particles_y = np.clip(self.particles_y, 0, self.field_size)
    
    def update(self, meas_x, meas_y, sensor_noise):
        distances = np.sqrt((self.particles_x - meas_x)**2 + (self.particles_y - meas_y)**2)
        self.weights = np.exp(-distances**2 / (2 * sensor_noise**2))
        self.weights /= np.sum(self.weights)
    
    def resample(self):
        indices = np.random.choice(self.n, size=self.n, p=self.weights)
        self.particles_x = self.particles_x[indices] + np.random.normal(0, 0.1, self.n)
        self.particles_y = self.particles_y[indices] + np.random.normal(0, 0.1, self.n)
        self.weights = np.ones(self.n) / self.n
    
    def estimate(self):
        return np.average(self.particles_x, weights=self.weights), np.average(self.particles_y, weights=self.weights)

# === MAIN APP ===
st.title("🤖 Sensor Fusion Workshop")
st.markdown("**Interactive demos for VISST School**")

tab1, tab2, tab3 = st.tabs(["📊 Kalman Filter", "🎯 Particle Filter", "🔀 Sensor Fusion"])

# === TAB 1: KALMAN FILTER ===
with tab1:
    st.header("Kalman Filter Demo")
    st.markdown("""
    The Kalman filter combines **predictions** (from motion) with **measurements** (from sensors).
    
    **Key insight**: It automatically balances trust based on uncertainty!
    """)
    
    col1, col2 = st.columns([1, 2])
    
    with col1:
        st.subheader("Parameters")
        kf_steps = st.slider("Time steps", 20, 200, 100, key="kf_steps")
        kf_Q = st.slider("Q (process noise)", 0.01, 0.5, 0.1, 0.01, 
                         help="Higher = trust measurements more")
        kf_R = st.slider("R (measurement noise)", 0.1, 2.0, 0.5, 0.1,
                         help="Higher = trust predictions more")
        odom_drift = st.slider("Odometry drift", 0.01, 0.2, 0.08, 0.01)
        camera_noise = st.slider("Camera noise", 0.1, 1.5, 0.4, 0.1)
        camera_interval = st.slider("Camera update interval", 1, 30, 8)
        
        if st.button("Run Simulation", key="kf_run"):
            st.session_state.kf_run = True
    
    with col2:
        if st.session_state.get('kf_run', False):
            np.random.seed(42)
            
            # Ground truth
            truth = np.cumsum(np.full(kf_steps, 0.3) + np.random.normal(0, 0.01, kf_steps))
            
            # Odometry (drifts)
            odom_drift_acc = np.cumsum(np.random.normal(0, odom_drift, kf_steps))
            odom = truth + odom_drift_acc
            
            # Camera (noisy, intermittent)
            camera = np.full(kf_steps, np.nan)
            for i in range(0, kf_steps, camera_interval):
                camera[i] = truth[i] + np.random.normal(0, camera_noise)
            
            # Kalman filter
            kf = KalmanFilter(kf_Q, kf_R)
            fused = []
            for i in range(kf_steps):
                if i > 0:
                    kf.predict(odom[i] - odom[i-1])
                if not np.isnan(camera[i]):
                    kf.update(camera[i])
                kf.record()
                fused.append(kf.x)
            fused = np.array(fused)
            
            # Plot
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            
            ax1.plot(truth, 'k-', lw=2, label='Ground Truth')
            ax1.plot(odom, 'b-', alpha=0.6, label='Odometry (drifts)')
            ax1.scatter(np.where(~np.isnan(camera))[0], camera[~np.isnan(camera)], 
                       c='g', s=40, marker='^', label='Camera', zorder=5)
            ax1.plot(fused, 'r-', lw=2, label='Kalman Fused')
            ax1.set_ylabel('Position')
            ax1.legend(loc='upper left')
            ax1.set_title('Position Tracking')
            ax1.grid(alpha=0.3)
            
            ax2.fill_between(range(kf_steps), 0, kf.history['P'], alpha=0.3, color='purple')
            ax2.plot(kf.history['P'], 'purple', label='Uncertainty')
            ax2.set_xlabel('Time Step')
            ax2.set_ylabel('Uncertainty')
            ax2.set_title('Uncertainty Over Time (grows during predict, shrinks during update)')
            ax2.legend()
            ax2.grid(alpha=0.3)
            
            plt.tight_layout()
            st.pyplot(fig)
            
            # Metrics
            odom_rmse = np.sqrt(np.mean((odom - truth)**2))
            fused_rmse = np.sqrt(np.mean((fused - truth)**2))
            improvement = (1 - fused_rmse / odom_rmse) * 100
            
            col_a, col_b, col_c = st.columns(3)
            col_a.metric("Odometry RMSE", f"{odom_rmse:.3f} m")
            col_b.metric("Fused RMSE", f"{fused_rmse:.3f} m")
            col_c.metric("Improvement", f"{improvement:.1f}%", delta=f"{improvement:.1f}%")

# === TAB 2: PARTICLE FILTER ===
with tab2:
    st.header("Particle Filter Demo")
    st.markdown("""
    Particle filters represent uncertainty with many "guesses" (particles).
    
    **Process**: Predict → Measure → Weight → Resample → Repeat
    """)
    
    col1, col2 = st.columns([1, 2])
    
    with col1:
        st.subheader("Parameters")
        n_particles = st.slider("Number of particles", 50, 500, 200, 50)
        pf_steps = st.slider("Simulation steps", 10, 100, 50)
        motion_noise = st.slider("Motion noise", 0.01, 0.3, 0.1, 0.01)
        sensor_noise = st.slider("Sensor noise", 0.1, 1.0, 0.3, 0.1)
        
        if st.button("Run Particle Filter", key="pf_run"):
            st.session_state.pf_run = True
    
    with col2:
        if st.session_state.get('pf_run', False):
            np.random.seed(42)
            field_size = 10.0
            
            # True robot trajectory (moves in a pattern)
            robot_x, robot_y = 5.0, 5.0
            trajectory_x, trajectory_y = [robot_x], [robot_y]
            
            # Initialize particle filter
            pf = ParticleFilter(n_particles, field_size)
            estimates_x, estimates_y = [], []
            
            # Simulate
            for step in range(pf_steps):
                # Robot moves
                dx = 0.15 * np.cos(step * 0.2)
                dy = 0.15 * np.sin(step * 0.2)
                robot_x = np.clip(robot_x + dx, 0.5, field_size - 0.5)
                robot_y = np.clip(robot_y + dy, 0.5, field_size - 0.5)
                trajectory_x.append(robot_x)
                trajectory_y.append(robot_y)
                
                # Particle filter steps
                pf.predict(dx, dy, motion_noise)
                meas_x = robot_x + np.random.normal(0, sensor_noise)
                meas_y = robot_y + np.random.normal(0, sensor_noise)
                pf.update(meas_x, meas_y, sensor_noise)
                pf.resample()
                
                est_x, est_y = pf.estimate()
                estimates_x.append(est_x)
                estimates_y.append(est_y)
            
            # Plot final state
            fig, ax = plt.subplots(figsize=(8, 8))
            
            # Particles
            ax.scatter(pf.particles_x, pf.particles_y, s=pf.weights*5000+5, 
                      c='blue', alpha=0.3, label='Particles')
            
            # Trajectory
            ax.plot(trajectory_x, trajectory_y, 'k--', alpha=0.5, label='True path')
            ax.plot(estimates_x, estimates_y, 'r-', lw=2, alpha=0.7, label='Estimated path')
            
            # Current positions
            ax.plot(robot_x, robot_y, 'go', markersize=15, label='True position')
            ax.plot(estimates_x[-1], estimates_y[-1], 'rs', markersize=12, label='Estimate')
            
            ax.set_xlim(0, field_size)
            ax.set_ylim(0, field_size)
            ax.set_aspect('equal')
            ax.legend(loc='upper right')
            ax.set_title(f'Particle Filter with {n_particles} particles')
            ax.grid(alpha=0.3)
            
            st.pyplot(fig)
            
            # Error metric
            final_error = np.sqrt((estimates_x[-1] - robot_x)**2 + (estimates_y[-1] - robot_y)**2)
            st.metric("Final Position Error", f"{final_error:.3f} m")

# === TAB 3: SENSOR FUSION COMPARISON ===
with tab3:
    st.header("Sensor Fusion: The Key Concept")
    
    st.markdown("""
    ### Why Sensor Fusion Works
    
    | Sensor Type | Noise | Drift | Example |
    |-------------|-------|-------|---------|
    | **Motion sensors** | Low | High | IMU, Encoders |
    | **Position sensors** | High | None | Camera, AprilTags |
    
    **Combined**: Low noise + No drift = Best of both!
    """)
    
    import os
    img_path = os.path.join(os.path.dirname(__file__), "..", "images", "localization_Gemini_Generate.png")
    if os.path.exists(img_path):
        st.image(img_path, caption="Sensor Fusion Visualization")
    else:
        st.info("Visualization image not found - run from project root directory")
    
    st.markdown("""
    ### The Kalman Filter Loop
    
    ```
    ┌─────────────┐              ┌─────────────┐
    │   PREDICT   │  ─────────►  │   UPDATE    │
    │  (Odometry) │              │  (Camera)   │
    │ uncertainty │              │ uncertainty │
    │   GROWS     │              │  SHRINKS    │
    └─────────────┘              └─────────────┘
           ▲                            │
           └────────────────────────────┘
    ```
    
    ### Key Equations
    
    **Kalman Gain**: `K = P / (P + R)`
    
    - K ≈ 1 → Trust sensor (camera) more
    - K ≈ 0 → Trust prediction (odometry) more
    
    **State Update**: `x = x + K × (measurement - x)`
    """)
    
    st.info("💡 **Tip**: Play with the parameters in the other tabs to see how Q and R affect the filter!")

# === SIDEBAR ===
st.sidebar.title("About")
st.sidebar.markdown("""
**Sensor Fusion Workshop**

For VISST School robotics students.

Learn how robots combine multiple sensors to know where they are!

---

**Demos:**
- 📊 Kalman Filter
- 🎯 Particle Filter  
- 🔀 Sensor Fusion Concept

---

*by Grigory Artazyan*
""")
