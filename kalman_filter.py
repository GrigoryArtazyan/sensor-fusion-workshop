"""
Kalman Filter Implementation for Sensor Fusion Workshop
========================================================

This module provides a clean, well-documented implementation of the 
1D Kalman filter for educational purposes. The concepts extend directly
to multi-dimensional cases used in real robotics applications.

Key Concepts:
- State: What we're trying to estimate (e.g., position, velocity)
- Covariance: How uncertain we are about our state estimate
- Process noise (Q): How much we trust our motion model
- Measurement noise (R): How much we trust our sensors

The Kalman filter runs a continuous predict-update loop:
1. PREDICT: Use motion model to predict next state (uncertainty grows)
2. UPDATE: Use sensor measurement to correct prediction (uncertainty shrinks)
"""

import numpy as np


class KalmanFilter1D:
    """
    A simple 1D Kalman filter for position estimation.
    
    This filter estimates position using:
    - A motion model (predict step) 
    - Sensor measurements (update step)
    
    Attributes:
        x (float): Current state estimate (position)
        P (float): Current uncertainty (variance) in state estimate
        Q (float): Process noise - how much uncertainty the motion model adds
        R (float): Measurement noise - how noisy the sensor readings are
    
    Example:
        >>> kf = KalmanFilter1D(initial_position=0.0, initial_uncertainty=1.0)
        >>> kf.predict(velocity=1.0, dt=0.1)  # Robot moved
        >>> kf.update(measurement=0.12)        # Sensor says we're at 0.12
        >>> print(kf.x)                        # Fused estimate
    """
    
    def __init__(self, initial_position=0.0, initial_uncertainty=1.0, 
                 process_noise=0.1, measurement_noise=1.0):
        """
        Initialize the Kalman filter.
        
        Args:
            initial_position: Starting position estimate
            initial_uncertainty: How uncertain we are about initial position
            process_noise (Q): Variance added each prediction step
                              Higher = trust motion model less
            measurement_noise (R): Variance of sensor measurements
                                   Higher = trust sensor less
        """
        # State estimate
        self.x = initial_position
        
        # Uncertainty (covariance) in state estimate
        self.P = initial_uncertainty
        
        # Process noise: How much does uncertainty grow each time step?
        # This accounts for unmodeled dynamics, control errors, etc.
        # TUNING TIP: Increase Q if your model is inaccurate
        self.Q = process_noise
        
        # Measurement noise: How noisy are sensor readings?
        # This should match the actual variance of your sensor
        # TUNING TIP: Increase R if sensor is unreliable
        self.R = measurement_noise
    
    def predict(self, velocity=0.0, dt=1.0):
        """
        Predict step: Estimate where we'll be based on motion model.
        
        This step INCREASES uncertainty because we're less sure about
        our state after time passes (things can go wrong).
        
        For a simple constant-velocity model:
            new_position = old_position + velocity * dt
        
        Args:
            velocity: Estimated velocity (could come from IMU integration)
            dt: Time step since last prediction
            
        Returns:
            tuple: (predicted_position, predicted_uncertainty)
        """
        # State prediction: x = x + v * dt
        # In matrix form: x = F * x + B * u, where F=1, B=dt, u=velocity
        self.x = self.x + velocity * dt
        
        # Uncertainty prediction: P = P + Q
        # In matrix form: P = F * P * F^T + Q
        # Uncertainty always GROWS in prediction step
        self.P = self.P + self.Q
        
        return self.x, self.P
    
    def update(self, measurement):
        """
        Update step: Correct our prediction using a sensor measurement.
        
        This step DECREASES uncertainty because the sensor gives us
        information about our actual state.
        
        The Kalman gain (K) determines how much we trust the measurement
        vs our prediction:
        - K close to 1: Trust measurement more (sensor is accurate)
        - K close to 0: Trust prediction more (sensor is noisy)
        
        Args:
            measurement: Sensor reading of position
            
        Returns:
            tuple: (updated_position, updated_uncertainty, kalman_gain)
        """
        # Calculate Kalman gain
        # K = P / (P + R)
        # This is the key formula that optimally weights prediction vs measurement
        # 
        # Intuition:
        # - If P is large (uncertain prediction), K -> 1, trust measurement
        # - If R is large (noisy sensor), K -> 0, trust prediction
        K = self.P / (self.P + self.R)
        
        # Calculate innovation (measurement residual)
        # This is the difference between what we measured and what we expected
        innovation = measurement - self.x
        
        # Update state estimate
        # x = x + K * (measurement - x)
        # We move our estimate toward the measurement, weighted by K
        self.x = self.x + K * innovation
        
        # Update uncertainty
        # P = (1 - K) * P
        # Uncertainty always SHRINKS in update step (we gained information)
        self.P = (1 - K) * self.P
        
        return self.x, self.P, K
    
    def get_state(self):
        """Return current state estimate and uncertainty."""
        return self.x, self.P


class KalmanFilter2D:
    """
    A 2D Kalman filter that estimates both position and velocity.
    
    State vector: [position, velocity]
    
    This is more realistic for robotics - we often want to estimate
    velocity as well as position, since velocity helps predict future position.
    
    The math is the same as 1D but uses matrices:
    - State transition matrix F models physics (position += velocity * dt)
    - Measurement matrix H extracts what sensors can see
    """
    
    def __init__(self, initial_state=None, initial_covariance=None,
                 process_noise=None, measurement_noise=1.0):
        """
        Initialize 2D Kalman filter.
        
        Args:
            initial_state: [position, velocity] initial estimate
            initial_covariance: 2x2 uncertainty matrix
            process_noise: 2x2 process noise matrix
            measurement_noise: Scalar variance of position measurements
        """
        # State vector: [position, velocity]
        if initial_state is None:
            self.x = np.array([0.0, 0.0])
        else:
            self.x = np.array(initial_state)
        
        # Covariance matrix (uncertainty in state)
        if initial_covariance is None:
            self.P = np.eye(2) * 1.0  # Start with some uncertainty
        else:
            self.P = np.array(initial_covariance)
        
        # Process noise matrix
        if process_noise is None:
            self.Q = np.array([
                [0.1, 0.0],   # Position process noise
                [0.0, 0.1]    # Velocity process noise
            ])
        else:
            self.Q = np.array(process_noise)
        
        # Measurement noise (we only measure position)
        self.R = measurement_noise
        
        # Measurement matrix: we only observe position, not velocity
        # H extracts position from state: observed = H @ state = position
        self.H = np.array([[1.0, 0.0]])
    
    def predict(self, dt=1.0):
        """
        Predict step using constant velocity model.
        
        Physics model:
            new_position = old_position + velocity * dt
            new_velocity = old_velocity (constant velocity assumption)
            
        In matrix form: x_new = F @ x_old
        """
        # State transition matrix
        # [pos_new]   [1  dt] [pos_old]
        # [vel_new] = [0   1] [vel_old]
        F = np.array([
            [1.0, dt],
            [0.0, 1.0]
        ])
        
        # Predict state
        self.x = F @ self.x
        
        # Predict covariance: P = F @ P @ F^T + Q
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x.copy(), self.P.copy()
    
    def update(self, measurement):
        """
        Update step: correct prediction with position measurement.
        
        Args:
            measurement: Observed position (scalar)
            
        Returns:
            tuple: (state, covariance, kalman_gain)
        """
        # Innovation (measurement residual)
        y = measurement - (self.H @ self.x)
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain: K = P @ H^T @ S^(-1)
        K = self.P @ self.H.T / S
        
        # Update state
        self.x = self.x + (K * y).flatten()
        
        # Update covariance: P = (I - K @ H) @ P
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x.copy(), self.P.copy(), K.flatten()
    
    def get_state(self):
        """Return current state [position, velocity] and covariance."""
        return self.x.copy(), self.P.copy()


# =============================================================================
# EXPERIMENTATION SECTION
# Students: Try modifying the parameters below to see how the filter behaves!
# =============================================================================

if __name__ == "__main__":
    print("Kalman Filter Demo")
    print("=" * 50)
    
    # Create a simple 1D filter
    kf = KalmanFilter1D(
        initial_position=0.0,
        initial_uncertainty=1.0,
        process_noise=0.1,      # Try changing this!
        measurement_noise=1.0    # Try changing this!
    )
    
    # Simulate some motion and measurements
    true_positions = [0, 1, 2, 3, 4, 5]
    measurements = [0.1, 1.3, 1.8, 3.2, 4.1, 4.9]  # Noisy sensor readings
    
    print("\nStep | True Pos | Measurement | Estimate | Uncertainty | Kalman Gain")
    print("-" * 75)
    
    for i, (true_pos, meas) in enumerate(zip(true_positions, measurements)):
        # Predict (assume constant velocity of 1)
        if i > 0:
            kf.predict(velocity=1.0, dt=1.0)
        
        # Update with measurement
        est, unc, K = kf.update(meas)
        
        print(f"  {i}  |   {true_pos:.1f}    |    {meas:.1f}      |   {est:.2f}   |    {unc:.3f}     |    {K:.3f}")
    
    print("\n" + "=" * 50)
    print("Try modifying process_noise and measurement_noise to see how")
    print("the filter balances between trusting the model vs the sensor!")
