import math
from typing import Tuple

class ComplementaryFilter:
    """
    A simple Complementary Filter for sensor fusion between Accelerometer and Gyroscope.

    The filter uses a weighted average to combine the stable, long-term angle 
    from the accelerometer with the reactive, short-term rotation from the gyroscope.
    """
    
    # Filter coefficient, or 'trust factor'.
    # Alpha (a) determines how much we trust the Gyroscope vs. the Accelerometer.
    # A value close to 1.0 (e.g., 0.98) means we trust the Gyroscope heavily.
    # The time constant (tau = alpha * dt / (1 - alpha)) is typically around 0.5s.
    ALPHA = 0.98  
    
    def __init__(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        """
        Initializes the filter with starting angles.
        
        Args:
            roll (float): Initial roll angle (degrees).
            pitch (float): Initial pitch angle (degrees).
            yaw (float): Initial yaw angle (degrees).
        """
        self.roll_angle = roll
        self.pitch_angle = pitch
        self.yaw_angle = yaw
        
        self.gyro_roll_angle = roll
        self.gyro_pitch_angle = pitch
        
    def update_roll_pitch(self, accel_g: Tuple[float, float, float], gyro_dps: Tuple[float, float, float], dt: float) -> Tuple[float, float]:
        """
        Applies the Complementary Filter to update the Roll and Pitch angles.

        Args:
            accel_g (Tuple[float, float, float]): Accelerations (Ax, Ay, Az) in g-force.
            gyro_dps (Tuple[float, float, float]): Angular rates (Gx, Gy, Gz) in degrees per second (dps).
            dt (float): The time elapsed since the last update in seconds.

        Returns:
            tuple: (roll_angle, pitch_angle) in degrees.
        """
        # 1. READ Gyroscope data
        gx, gy, _ = gyro_dps # We ignore Gz for Roll/Pitch in the basic filter
        
        # 2. Integrate Gyroscope to find the predicted new angle
        # Angle_new = Angle_old + Rate * dt
        self.gyro_roll_angle += gx * dt
        self.gyro_pitch_angle += gy * dt
        
        # 3. Calculate Accelerometer angles (Static correction)
        # Note: These formulas were already defined in your previous step.
        ax, ay, az = accel_g
        
        # Static Roll from Accelerometer
        accel_roll = math.degrees(math.atan2(ay, az))
        
        # Static Pitch from Accelerometer
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        # 4. Apply the Complementary Filter formula
        # Fused_Angle = ALPHA * (Gyro_Angle + Gyro_Rate * dt) + (1 - ALPHA) * Accel_Angle
        
        self.roll_angle = self.ALPHA * (self.roll_angle + gx * dt) + (1.0 - self.ALPHA) * accel_roll
        self.pitch_angle = self.ALPHA * (self.pitch_angle + gy * dt) + (1.0 - self.ALPHA) * accel_pitch
        
        # Alternative formula, simpler form (integrating the difference)
        # roll_angle = ALPHA * (self.roll_angle + gx * dt) + (1.0 - ALPHA) * accel_roll

        return self.roll_angle, self.pitch_angle

    def update_yaw(self, gyro_z_dps: float, dt: float) -> float:
        """
        Updates the Yaw angle (Lacet) using simple integration of the Gyroscope's Z-axis.

        Note: Yaw typically requires a Magnetometer (or external heading correction) 
        to prevent long-term drift, as the Gyroscope's error is integrated over time.
        
        Args:
            gyro_z_dps (float): Angular rate around the Z-axis in dps.
            dt (float): The time elapsed since the last update in seconds.

        Returns:
            float: The updated yaw angle in degrees.
        """
        # Simple integration: Angle = Angle + Rate * dt
        self.yaw_angle += gyro_z_dps * dt
        
        # We can add a simple normalization to keep Yaw between -180 and 180 degrees
        if self.yaw_angle > 180.0:
            self.yaw_angle -= 360.0
        elif self.yaw_angle < -180.0:
            self.yaw_angle += 360.0
            
        return self.yaw_angle