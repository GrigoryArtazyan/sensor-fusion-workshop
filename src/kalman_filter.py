"""
Kalman Filter Implementation
============================
A simple, well-documented 1D Kalman filter for educational purposes.
"""

import numpy as np


class KalmanFilter1D:
    """
    1D Kalman filter for position estimation.
    
    Example:
        >>> kf = KalmanFilter1D(process_noise=0.1, measurement_noise=1.0)
        >>> kf.predict(velocity=1.0, dt=0.1)
        >>> estimate, uncertainty, gain = kf.update(measurement=0.12)
    """
    
    def __init__(self, initial_position=0.0, initial_uncertainty=1.0, 
                 process_noise=0.1, measurement_noise=1.0):
        """
        Args:
            initial_position: Starting position estimate
            initial_uncertainty: Starting uncertainty
            process_noise (Q): How much uncertainty grows each step
            measurement_noise (R): How noisy sensor readings are
        """
        self.x = initial_position      # State estimate
        self.P = initial_uncertainty   # Uncertainty
        self.Q = process_noise         # Process noise
        self.R = measurement_noise     # Measurement noise
    
    def predict(self, velocity=0.0, dt=1.0):
        """
        Predict next state based on motion model.
        Uncertainty INCREASES (we're less sure over time).
        """
        self.x = self.x + velocity * dt
        self.P = self.P + self.Q
        return self.x, self.P
    
    def update(self, measurement):
        """
        Update estimate using sensor measurement.
        Uncertainty DECREASES (sensor gives us information).
        
        Returns: (estimate, uncertainty, kalman_gain)
        """
        # Kalman gain: balance between prediction and measurement
        K = self.P / (self.P + self.R)
        
        # Update state
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * self.P
        
        return self.x, self.P, K
    
    def get_state(self):
        """Return current state and uncertainty."""
        return self.x, self.P


if __name__ == "__main__":
    # Simple demo
    print("Kalman Filter Demo")
    print("=" * 50)
    
    kf = KalmanFilter1D(process_noise=0.1, measurement_noise=1.0)
    
    true_positions = [0, 1, 2, 3, 4, 5]
    measurements = [0.1, 1.3, 1.8, 3.2, 4.1, 4.9]
    
    print("\nStep | True | Measured | Estimate | Gain")
    print("-" * 50)
    
    for i, (true_pos, meas) in enumerate(zip(true_positions, measurements)):
        if i > 0:
            kf.predict(velocity=1.0, dt=1.0)
        est, unc, K = kf.update(meas)
        print(f"  {i}  | {true_pos:.1f}  |   {meas:.1f}    |   {est:.2f}   | {K:.3f}")
