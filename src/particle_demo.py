"""
Particle Filter Demo
====================
A simple visualization of how particle filters work for localization.

Particles represent "guesses" about where the robot might be.
- Start with particles spread out (uncertain)
- When sensor data comes in, particles near the true position survive
- Over time, particles cluster around the true position

Run: python src/particle_demo.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Settings
NUM_PARTICLES = 200
FIELD_SIZE = 10.0
SENSOR_NOISE = 0.5
MOTION_NOISE = 0.1


class ParticleFilter:
    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.reset()
    
    def reset(self):
        # Initialize particles randomly across the field
        self.particles_x = np.random.uniform(0, FIELD_SIZE, self.num_particles)
        self.particles_y = np.random.uniform(0, FIELD_SIZE, self.num_particles)
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def predict(self, dx, dy):
        """Move all particles based on motion command (with noise)."""
        self.particles_x += dx + np.random.normal(0, MOTION_NOISE, self.num_particles)
        self.particles_y += dy + np.random.normal(0, MOTION_NOISE, self.num_particles)
        
        # Keep particles in bounds
        self.particles_x = np.clip(self.particles_x, 0, FIELD_SIZE)
        self.particles_y = np.clip(self.particles_y, 0, FIELD_SIZE)
    
    def update(self, measured_x, measured_y):
        """
        Update particle weights based on sensor measurement.
        Particles closer to the measurement get higher weights.
        """
        # Calculate distance from each particle to measurement
        distances = np.sqrt((self.particles_x - measured_x)**2 + 
                           (self.particles_y - measured_y)**2)
        
        # Convert distances to weights (closer = higher weight)
        # Using Gaussian likelihood
        self.weights = np.exp(-distances**2 / (2 * SENSOR_NOISE**2))
        
        # Normalize weights
        self.weights /= np.sum(self.weights)
    
    def resample(self):
        """
        Resample particles based on weights.
        Particles with higher weights get duplicated.
        Particles with lower weights disappear.
        """
        indices = np.random.choice(
            self.num_particles, 
            size=self.num_particles, 
            p=self.weights
        )
        
        self.particles_x = self.particles_x[indices]
        self.particles_y = self.particles_y[indices]
        
        # Add small noise to prevent particle depletion
        self.particles_x += np.random.normal(0, 0.05, self.num_particles)
        self.particles_y += np.random.normal(0, 0.05, self.num_particles)
        
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def estimate(self):
        """Get weighted average position estimate."""
        est_x = np.average(self.particles_x, weights=self.weights)
        est_y = np.average(self.particles_y, weights=self.weights)
        return est_x, est_y


class ParticleFilterDemo:
    def __init__(self):
        # True robot position
        self.robot_x = FIELD_SIZE / 2
        self.robot_y = FIELD_SIZE / 2
        self.robot_trail = [(self.robot_x, self.robot_y)]
        
        # Particle filter
        self.pf = ParticleFilter(NUM_PARTICLES)
        
        # Movement pattern (will move in a square)
        self.step = 0
        self.movements = [
            (0.2, 0),   # Right
            (0.2, 0),
            (0.2, 0),
            (0, 0.2),   # Up
            (0, 0.2),
            (0, 0.2),
            (-0.2, 0),  # Left
            (-0.2, 0),
            (-0.2, 0),
            (0, -0.2),  # Down
            (0, -0.2),
            (0, -0.2),
        ]
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Particle Filter Demo')
        
    def update_frame(self, frame):
        self.ax.clear()
        
        # Get movement
        dx, dy = self.movements[self.step % len(self.movements)]
        self.step += 1
        
        # Move true robot
        self.robot_x = np.clip(self.robot_x + dx, 0.5, FIELD_SIZE - 0.5)
        self.robot_y = np.clip(self.robot_y + dy, 0.5, FIELD_SIZE - 0.5)
        self.robot_trail.append((self.robot_x, self.robot_y))
        if len(self.robot_trail) > 50:
            self.robot_trail.pop(0)
        
        # Particle filter steps
        # 1. Predict (move particles)
        self.pf.predict(dx, dy)
        
        # 2. Get sensor measurement (with noise)
        measured_x = self.robot_x + np.random.normal(0, SENSOR_NOISE)
        measured_y = self.robot_y + np.random.normal(0, SENSOR_NOISE)
        
        # 3. Update weights based on measurement
        self.pf.update(measured_x, measured_y)
        
        # 4. Resample
        self.pf.resample()
        
        # 5. Get estimate
        est_x, est_y = self.pf.estimate()
        
        # === PLOTTING ===
        self.ax.set_xlim(0, FIELD_SIZE)
        self.ax.set_ylim(0, FIELD_SIZE)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # Draw particles (size based on weight)
        sizes = self.pf.weights * 1000 + 5
        self.ax.scatter(self.pf.particles_x, self.pf.particles_y, 
                       s=sizes, c='blue', alpha=0.3, label='Particles')
        
        # Draw robot trail
        if len(self.robot_trail) > 1:
            trail = np.array(self.robot_trail)
            self.ax.plot(trail[:, 0], trail[:, 1], 'k-', alpha=0.3, lw=1)
        
        # Draw true robot
        self.ax.plot(self.robot_x, self.robot_y, 'ko', markersize=20)
        self.ax.plot(self.robot_x, self.robot_y, 'go', markersize=15, label='True position')
        
        # Draw sensor measurement
        self.ax.plot(measured_x, measured_y, 'r*', markersize=15, 
                    label='Sensor reading (noisy)', alpha=0.7)
        
        # Draw estimate
        self.ax.plot(est_x, est_y, 'rs', markersize=12, label='Particle estimate')
        
        # Error
        error = np.sqrt((est_x - self.robot_x)**2 + (est_y - self.robot_y)**2)
        
        # Title and legend
        self.ax.set_title(f'Particle Filter Localization\n'
                         f'Step {self.step} | Particles: {NUM_PARTICLES} | '
                         f'Error: {error:.2f}m', fontsize=12)
        self.ax.legend(loc='upper right')
        
        # Explanation
        explanation = """
How it works:
1. PREDICT: Move all particles based on motion command
2. SENSE: Get noisy sensor reading (red star)
3. WEIGHT: Particles near reading get higher weight
4. RESAMPLE: Duplicate good particles, remove bad ones
5. ESTIMATE: Weighted average of all particles

Watch: Particles cluster around the true position!
        """
        self.ax.text(0.02, 0.02, explanation, transform=self.ax.transAxes,
                    fontsize=9, verticalalignment='bottom', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        return []
    
    def run(self):
        ani = FuncAnimation(self.fig, self.update_frame, frames=200, 
                           interval=300, blit=False, repeat=True)
        plt.show()


def main():
    print("=" * 55)
    print("PARTICLE FILTER DEMO")
    print("=" * 55)
    print("""
Watch how particle filters handle localization!

CONCEPT:
  - Blue dots = Particles (guesses about robot location)
  - Green dot = True robot position  
  - Red star  = Noisy sensor reading
  - Red square = Particle filter estimate

PROCESS (each step):
  1. PREDICT: Particles move with the robot (but with noise)
  2. SENSE: Get a noisy position measurement
  3. WEIGHT: Particles near the measurement are "better"
  4. RESAMPLE: Keep good particles, discard bad ones
  5. ESTIMATE: Average of all particles

Watch how particles cluster around the true position!

Starting animation...
""")
    
    demo = ParticleFilterDemo()
    demo.run()


if __name__ == "__main__":
    main()
