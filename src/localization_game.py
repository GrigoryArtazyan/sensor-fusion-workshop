"""
Robot Localization Game
=======================
A simple interactive game to understand sensor fusion and filtering!

The robot moves on a 2D field. You can only "see" through noisy sensors.
Watch how the filter estimates position despite uncertainty.

Controls:
- Arrow keys: Move the robot
- SPACE: Toggle sensor readings on/off
- R: Reset position
- 1/2/3: Change filter aggressiveness
- Q: Quit

Run: python src/localization_game.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow
from matplotlib.widgets import Button
import matplotlib.animation as animation

# ============================================
# GAME SETTINGS
# ============================================
FIELD_SIZE = 10.0  # meters
ROBOT_SPEED = 0.3  # meters per step

# Sensor characteristics
IMU_NOISE = 0.02      # Low noise
IMU_DRIFT_RATE = 0.01 # Accumulates over time

CAMERA_NOISE = 0.5    # High noise  
CAMERA_AVAILABLE = 0.7  # 70% chance camera sees landmarks

# Filter settings
FILTER_TRUST_CAMERA = 0.3  # How much to trust camera vs prediction


class Robot:
    """The actual robot (ground truth)."""
    def __init__(self):
        self.x = FIELD_SIZE / 2
        self.y = FIELD_SIZE / 2
        self.trail = [(self.x, self.y)]
    
    def move(self, dx, dy):
        self.x = np.clip(self.x + dx, 0.5, FIELD_SIZE - 0.5)
        self.y = np.clip(self.y + dy, 0.5, FIELD_SIZE - 0.5)
        self.trail.append((self.x, self.y))
        if len(self.trail) > 100:
            self.trail.pop(0)
    
    def reset(self):
        self.x = FIELD_SIZE / 2
        self.y = FIELD_SIZE / 2
        self.trail = [(self.x, self.y)]


class IMUSensor:
    """
    IMU: LOW NOISE, HIGH DRIFT
    Measures movement, but errors accumulate over time.
    """
    def __init__(self):
        self.estimated_x = FIELD_SIZE / 2
        self.estimated_y = FIELD_SIZE / 2
        self.drift_x = 0
        self.drift_y = 0
        self.trail = [(self.estimated_x, self.estimated_y)]
    
    def update(self, dx, dy):
        # Drift accumulates
        self.drift_x += np.random.normal(0, IMU_DRIFT_RATE)
        self.drift_y += np.random.normal(0, IMU_DRIFT_RATE)
        
        # Measure movement with low noise but accumulated drift
        measured_dx = dx + np.random.normal(0, IMU_NOISE) + self.drift_x
        measured_dy = dy + np.random.normal(0, IMU_NOISE) + self.drift_y
        
        self.estimated_x += measured_dx
        self.estimated_y += measured_dy
        
        self.trail.append((self.estimated_x, self.estimated_y))
        if len(self.trail) > 100:
            self.trail.pop(0)
        
        return self.estimated_x, self.estimated_y
    
    def reset(self):
        self.estimated_x = FIELD_SIZE / 2
        self.estimated_y = FIELD_SIZE / 2
        self.drift_x = 0
        self.drift_y = 0
        self.trail = [(self.estimated_x, self.estimated_y)]


class CameraSensor:
    """
    Camera/AprilTags: HIGH NOISE, NO DRIFT
    Gives absolute position, but noisy and not always available.
    """
    def __init__(self):
        self.last_reading = None
        self.available = False
    
    def update(self, true_x, true_y):
        # Camera doesn't always see landmarks
        self.available = np.random.random() < CAMERA_AVAILABLE
        
        if self.available:
            # Noisy but absolute position
            self.last_reading = (
                true_x + np.random.normal(0, CAMERA_NOISE),
                true_y + np.random.normal(0, CAMERA_NOISE)
            )
        else:
            self.last_reading = None
        
        return self.last_reading
    
    def reset(self):
        self.last_reading = None
        self.available = False


class FusedEstimate:
    """
    Kalman-like fusion of IMU and Camera.
    Combines low-noise/high-drift with high-noise/no-drift.
    """
    def __init__(self):
        self.x = FIELD_SIZE / 2
        self.y = FIELD_SIZE / 2
        self.uncertainty = 1.0
        self.trail = [(self.x, self.y)]
        self.trust_camera = FILTER_TRUST_CAMERA
    
    def update(self, imu_x, imu_y, camera_reading):
        # Predict using IMU (uncertainty grows)
        pred_x = imu_x
        pred_y = imu_y
        self.uncertainty += 0.05
        
        if camera_reading is not None:
            # Update with camera (uncertainty shrinks)
            cam_x, cam_y = camera_reading
            
            # Kalman-like weighted average
            K = self.trust_camera  # Simplified Kalman gain
            self.x = (1 - K) * self.x + K * cam_x
            self.y = (1 - K) * self.y + K * cam_y
            
            # But also blend with IMU prediction
            self.x = 0.7 * self.x + 0.3 * pred_x
            self.y = 0.7 * self.y + 0.3 * pred_y
            
            self.uncertainty *= 0.5  # Shrinks with camera update
        else:
            # No camera - follow IMU but uncertainty grows
            self.x = 0.3 * self.x + 0.7 * pred_x
            self.y = 0.3 * self.y + 0.7 * pred_y
        
        self.uncertainty = np.clip(self.uncertainty, 0.1, 3.0)
        
        self.trail.append((self.x, self.y))
        if len(self.trail) > 100:
            self.trail.pop(0)
        
        return self.x, self.y, self.uncertainty
    
    def set_trust(self, trust):
        self.trust_camera = trust
    
    def reset(self):
        self.x = FIELD_SIZE / 2
        self.y = FIELD_SIZE / 2
        self.uncertainty = 1.0
        self.trail = [(self.x, self.y)]


class LocalizationGame:
    def __init__(self):
        self.robot = Robot()
        self.imu = IMUSensor()
        self.camera = CameraSensor()
        self.fused = FusedEstimate()
        
        self.show_sensors = True
        self.step_count = 0
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Robot Localization Game')
        
        # Connect keyboard
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        self.setup_plot()
        self.update_plot()
    
    def setup_plot(self):
        self.ax.set_xlim(0, FIELD_SIZE)
        self.ax.set_ylim(0, FIELD_SIZE)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Robot Localization Game\n'
                          'Arrow keys: Move | Space: Toggle sensors | R: Reset | 1/2/3: Filter trust', 
                          fontsize=11)
        
        # Draw "landmarks" (AprilTag positions)
        landmarks = [(1, 1), (1, 9), (9, 1), (9, 9), (5, 5)]
        for lx, ly in landmarks:
            self.ax.plot(lx, ly, 's', color='purple', markersize=15, alpha=0.3)
            self.ax.text(lx, ly-0.5, 'Tag', ha='center', fontsize=8, color='purple', alpha=0.5)
    
    def update_plot(self):
        self.ax.clear()
        self.setup_plot()
        
        # Draw trails
        if self.show_sensors and len(self.imu.trail) > 1:
            imu_trail = np.array(self.imu.trail)
            self.ax.plot(imu_trail[:, 0], imu_trail[:, 1], 'c-', alpha=0.3, lw=1, label='IMU trail')
        
        if len(self.fused.trail) > 1:
            fused_trail = np.array(self.fused.trail)
            self.ax.plot(fused_trail[:, 0], fused_trail[:, 1], 'r-', alpha=0.5, lw=2, label='Fused trail')
        
        if len(self.robot.trail) > 1:
            robot_trail = np.array(self.robot.trail)
            self.ax.plot(robot_trail[:, 0], robot_trail[:, 1], 'k-', alpha=0.3, lw=1)
        
        # Draw robot (ground truth)
        self.ax.plot(self.robot.x, self.robot.y, 'ko', markersize=20, label='Robot (truth)')
        self.ax.plot(self.robot.x, self.robot.y, 'go', markersize=15)
        
        if self.show_sensors:
            # Draw IMU estimate
            self.ax.plot(self.imu.estimated_x, self.imu.estimated_y, 'c^', 
                        markersize=12, label=f'IMU (drifts)')
            
            # Draw camera reading if available
            if self.camera.last_reading:
                cx, cy = self.camera.last_reading
                self.ax.plot(cx, cy, 'g*', markersize=15, label='Camera (noisy)')
                # Draw line from camera to truth to show noise
                self.ax.plot([self.robot.x, cx], [self.robot.y, cy], 'g--', alpha=0.3)
        
        # Draw fused estimate with uncertainty circle
        circle = Circle((self.fused.x, self.fused.y), self.fused.uncertainty,
                        fill=False, color='red', linestyle='--', linewidth=2, alpha=0.5)
        self.ax.add_patch(circle)
        self.ax.plot(self.fused.x, self.fused.y, 'rs', markersize=14, label='Fused estimate')
        
        # Stats
        imu_error = np.sqrt((self.imu.estimated_x - self.robot.x)**2 + 
                           (self.imu.estimated_y - self.robot.y)**2)
        fused_error = np.sqrt((self.fused.x - self.robot.x)**2 + 
                             (self.fused.y - self.robot.y)**2)
        
        stats_text = f'Step: {self.step_count}\n'
        stats_text += f'IMU Error: {imu_error:.2f}m\n'
        stats_text += f'Fused Error: {fused_error:.2f}m\n'
        stats_text += f'Uncertainty: {self.fused.uncertainty:.2f}\n'
        stats_text += f'Camera: {"✓ visible" if self.camera.available else "✗ blocked"}'
        
        self.ax.text(0.02, 0.98, stats_text, transform=self.ax.transAxes,
                    fontsize=10, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Legend
        self.ax.legend(loc='upper right', fontsize=9)
        
        # Concept reminder
        concept = 'LOW noise + HIGH drift (IMU) + HIGH noise + LOW drift (Camera) = Best estimate!'
        self.ax.text(0.5, 0.02, concept, transform=self.ax.transAxes,
                    ha='center', fontsize=9, style='italic', alpha=0.7)
        
        self.fig.canvas.draw()
    
    def move_robot(self, dx, dy):
        # Move actual robot
        self.robot.move(dx, dy)
        
        # Update sensors
        self.imu.update(dx, dy)
        camera_reading = self.camera.update(self.robot.x, self.robot.y)
        
        # Update fused estimate
        self.fused.update(self.imu.estimated_x, self.imu.estimated_y, camera_reading)
        
        self.step_count += 1
        self.update_plot()
    
    def on_key(self, event):
        if event.key == 'up':
            self.move_robot(0, ROBOT_SPEED)
        elif event.key == 'down':
            self.move_robot(0, -ROBOT_SPEED)
        elif event.key == 'left':
            self.move_robot(-ROBOT_SPEED, 0)
        elif event.key == 'right':
            self.move_robot(ROBOT_SPEED, 0)
        elif event.key == ' ':
            self.show_sensors = not self.show_sensors
            self.update_plot()
        elif event.key == 'r':
            self.robot.reset()
            self.imu.reset()
            self.camera.reset()
            self.fused.reset()
            self.step_count = 0
            self.update_plot()
        elif event.key == '1':
            self.fused.set_trust(0.1)
            print("Filter: Low camera trust (follows IMU more)")
        elif event.key == '2':
            self.fused.set_trust(0.3)
            print("Filter: Medium camera trust (balanced)")
        elif event.key == '3':
            self.fused.set_trust(0.6)
            print("Filter: High camera trust (jumps to camera)")
        elif event.key == 'q':
            plt.close()
    
    def run(self):
        plt.show()


def main():
    print("=" * 55)
    print("ROBOT LOCALIZATION GAME")
    print("=" * 55)
    print("""
Learn sensor fusion by playing!

CONCEPT:
  IMU    = Low noise, but DRIFTS over time (cyan triangle)
  Camera = High noise, but NO drift (green star)
  Fused  = Combines both for best estimate (red square)

CONTROLS:
  Arrow keys  - Move the robot
  Space       - Toggle sensor display
  R           - Reset position
  1/2/3       - Change how much filter trusts camera
  Q           - Quit

WATCH:
  - IMU estimate (cyan) slowly drifts away from truth
  - Camera readings (green) are noisy but centered on truth
  - Fused estimate (red) stays close with low uncertainty!
  - Red circle shows uncertainty (shrinks when camera visible)

""")
    
    game = LocalizationGame()
    game.run()


if __name__ == "__main__":
    main()
