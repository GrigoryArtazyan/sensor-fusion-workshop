"""
Sensor Fusion Interactive Demo
==============================
A Streamlit app to explore sensor fusion, Kalman filtering, and particle filters.
No external APIs - runs completely offline!

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
        self.history_P = []
    
    def predict(self, velocity):
        self.x += velocity
        self.P += self.Q
        
    def update(self, measurement):
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return K
    
    def record(self):
        self.history_P.append(self.P)

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
        self.weights /= np.sum(self.weights) + 1e-10
    
    def resample(self):
        indices = np.random.choice(self.n, size=self.n, p=self.weights)
        self.particles_x = self.particles_x[indices] + np.random.normal(0, 0.05, self.n)
        self.particles_y = self.particles_y[indices] + np.random.normal(0, 0.05, self.n)
        self.weights = np.ones(self.n) / self.n
    
    def estimate(self):
        return np.average(self.particles_x, weights=self.weights), np.average(self.particles_y, weights=self.weights)

# === MAIN APP ===
st.title("🤖 Sensor Fusion Workshop")
st.markdown("**Interactive demos for VISST School** - No internet required!")

tab1, tab2, tab3 = st.tabs(["📊 Kalman Filter", "🎯 Particle Filter", "📚 Concepts"])

# === TAB 1: KALMAN FILTER ===
with tab1:
    st.header("Kalman Filter Demo")
    
    with st.expander("📖 How to Use", expanded=False):
        st.markdown("""
        **Try these experiments:**
        1. **Increase Odometry drift** → Watch blue line diverge from truth
        2. **Increase Camera noise** → Green dots scatter more
        3. **Increase Camera interval** → Fewer updates, uncertainty grows longer
        4. **Adjust Q (process noise)** → Higher = filter reacts faster but noisier
        5. **Adjust R (measurement noise)** → Higher = filter ignores camera more
        
        **Goal:** Can you make fused estimate *worse* than odometry? (Hint: it's hard!)
        """)
    
    st.markdown("Adjust parameters and see results instantly!")
    
    col1, col2 = st.columns([1, 2])
    
    with col1:
        st.subheader("⚙️ Parameters")
        kf_steps = st.slider("Time steps", 50, 200, 100)
        odom_drift = st.slider("Odometry drift", 0.02, 0.15, 0.08, 0.01)
        camera_noise = st.slider("Camera noise", 0.1, 1.0, 0.4, 0.1)
        camera_interval = st.slider("Camera interval", 3, 25, 8)
        kf_Q = st.slider("Q (process noise)", 0.01, 0.3, 0.05, 0.01)
        kf_R = st.slider("R (measurement noise)", 0.1, 1.0, 0.3, 0.1)
    
    with col2:
        # Run simulation with current parameters
        np.random.seed(42)
        
        # Ground truth
        truth = np.cumsum(np.full(kf_steps, 0.3) + np.random.normal(0, 0.01, kf_steps))
        
        # Odometry (drifts)
        odom = truth + np.cumsum(np.random.normal(0, odom_drift, kf_steps))
        
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
        ax1.plot(odom, 'b-', alpha=0.5, label='Odometry (drifts)')
        cam_idx = np.where(~np.isnan(camera))[0]
        ax1.scatter(cam_idx, camera[cam_idx], c='g', s=50, marker='^', label='Camera', zorder=5)
        ax1.plot(fused, 'r-', lw=2, label='Fused')
        ax1.set_ylabel('Position (m)')
        ax1.legend(loc='upper left')
        ax1.set_title('Sensor Fusion: Odometry + Camera → Best Estimate')
        ax1.grid(alpha=0.3)
        
        ax2.fill_between(range(kf_steps), 0, kf.history_P, alpha=0.4, color='purple')
        ax2.set_xlabel('Time Step')
        ax2.set_ylabel('Uncertainty')
        ax2.set_title('Uncertainty: grows during predict, shrinks during update')
        ax2.grid(alpha=0.3)
        
        plt.tight_layout()
        st.pyplot(fig)
        plt.close()
        
        # Metrics
        odom_rmse = np.sqrt(np.mean((odom - truth)**2))
        fused_rmse = np.sqrt(np.mean((fused - truth)**2))
        
        # Metrics with color-coded improvement
        improvement = (1 - fused_rmse/odom_rmse) * 100
        
        c1, c2, c3 = st.columns(3)
        c1.metric("Odometry RMSE", f"{odom_rmse:.2f} m")
        c2.metric("Fused RMSE", f"{fused_rmse:.2f} m", delta=f"{-improvement:.0f}%" if improvement > 0 else f"+{-improvement:.0f}%", delta_color="inverse")
        if improvement > 0:
            c3.metric("Improvement", f"{improvement:.0f}%", delta="Fusion wins!", delta_color="normal")
        else:
            c3.metric("Improvement", f"{improvement:.0f}%", delta="Odometry wins", delta_color="inverse")

# === TAB 2: PARTICLE FILTER ===
with tab2:
    st.header("Particle Filter Demo")
    
    with st.expander("📖 How to Use", expanded=False):
        st.markdown("""
        **Try these experiments:**
        1. **Fewer particles** → Less accurate, but faster
        2. **More particles** → More accurate, shows convergence better
        3. **Increase motion noise** → Particles spread out more during predict
        4. **Increase sensor noise** → Takes longer to converge
        
        **Watch:** Blue particles cluster around green (true position) over time!
        """)
    
    st.markdown("Watch particles converge to the true position!")
    
    col1, col2 = st.columns([1, 2])
    
    with col1:
        st.subheader("⚙️ Parameters")
        n_particles = st.slider("Particles", 50, 400, 150, 50)
        pf_steps = st.slider("Steps", 20, 80, 40)
        motion_noise = st.slider("Motion noise", 0.05, 0.25, 0.1, 0.01)
        pf_sensor_noise = st.slider("Sensor noise (PF)", 0.1, 0.8, 0.3, 0.05)
    
    with col2:
        np.random.seed(123)
        field_size = 10.0
        
        robot_x, robot_y = 5.0, 5.0
        traj_x, traj_y = [robot_x], [robot_y]
        
        pf = ParticleFilter(n_particles, field_size)
        est_x, est_y = [], []
        
        for step in range(pf_steps):
            dx = 0.12 * np.cos(step * 0.15)
            dy = 0.12 * np.sin(step * 0.15)
            robot_x = np.clip(robot_x + dx, 1, field_size - 1)
            robot_y = np.clip(robot_y + dy, 1, field_size - 1)
            traj_x.append(robot_x)
            traj_y.append(robot_y)
            
            pf.predict(dx, dy, motion_noise)
            mx = robot_x + np.random.normal(0, pf_sensor_noise)
            my = robot_y + np.random.normal(0, pf_sensor_noise)
            pf.update(mx, my, pf_sensor_noise)
            pf.resample()
            
            ex, ey = pf.estimate()
            est_x.append(ex)
            est_y.append(ey)
        
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.scatter(pf.particles_x, pf.particles_y, s=20, c='blue', alpha=0.4, label='Particles')
        ax.plot(traj_x, traj_y, 'k--', alpha=0.5, lw=1, label='True path')
        ax.plot(est_x, est_y, 'r-', lw=2, label='Estimated path')
        ax.plot(robot_x, robot_y, 'go', markersize=15, label='True position', zorder=10)
        ax.plot(est_x[-1], est_y[-1], 'rs', markersize=12, label='Estimate', zorder=10)
        ax.set_xlim(0, field_size)
        ax.set_ylim(0, field_size)
        ax.set_aspect('equal')
        ax.legend(loc='upper right')
        ax.set_title(f'Particle Filter Localization ({n_particles} particles)')
        ax.grid(alpha=0.3)
        st.pyplot(fig)
        plt.close()
        
        error = np.sqrt((est_x[-1] - robot_x)**2 + (est_y[-1] - robot_y)**2)
        st.metric("Final Error", f"{error:.3f} m")

# === TAB 3: CONCEPTS ===
with tab3:
    st.header("Sensor Fusion Concepts")
    
    st.subheader("Why Combine Sensors?")
    st.markdown("""
    | Sensor | Noise | Drift | Example |
    |--------|-------|-------|---------|
    | **Motion sensors** | Low | High | IMU, Wheel encoders |
    | **Position sensors** | High | None | Camera, AprilTags |
    
    **Result**: Low noise + No drift = Best of both!
    """)
    
    st.subheader("The Kalman Filter Loop")
    st.code("""
    ┌─────────────┐              ┌─────────────┐
    │   PREDICT   │  ─────────►  │   UPDATE    │
    │  (Odometry) │              │  (Camera)   │
    │ uncertainty │              │ uncertainty │
    │   GROWS     │              │  SHRINKS    │
    └─────────────┘              └─────────────┘
           ▲                            │
           └────────────────────────────┘
    """)
    
    st.subheader("Key Equations")
    st.latex(r"K = \frac{P}{P + R}")
    st.markdown("- **K ≈ 1**: Trust measurement (camera)")
    st.markdown("- **K ≈ 0**: Trust prediction (odometry)")
    
    st.latex(r"x_{new} = x + K \times (measurement - x)")
    
    st.info("💡 Go to the other tabs and adjust parameters to see these concepts in action!")

# === SIDEBAR ===
st.sidebar.title("About")
st.sidebar.markdown("""
**Sensor Fusion Workshop**

---

**Features:**
- 📊 Kalman Filter demo
- 🎯 Particle Filter demo
- 📚 Concept explanations


---

*by Grigory Artazyan & Claude*
""")
