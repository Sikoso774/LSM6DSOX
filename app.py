import sys
import time
from driver.lsm6dsox import LSM6DSOX_IMU
from driver.settings import I2C_BUS_NUM
from driver.complementary_filter import ComplementaryFilter

class IMU_Application:
    """
    Main application controller for running the LSM6DSOX data acquisition loop.

    This class handles the initialization flow, the continuous data reading loop, 
    and proper cleanup of system resources (I2C bus).
    """

    def __init__(self) -> None:
        """
        Initializes the application and creates the IMU sensor object.
        """
        # Sensor instantiation
        self.sensor = LSM6DSOX_IMU(bus_num=I2C_BUS_NUM)
        
        # Complementary Filter instance
        self.filter = ComplementaryFilter()

    def initialize(self) -> bool:
        """
        Initializes the sensor hardware and configuration via the LSM6DSOX_IMU class.

        Returns:
            bool: True if sensor initialization was successful.
        """
        print("Initializing LSM6DSOX IMU...")
        return self.sensor.initialize()

    def run(self, delay_s=0.1) -> None:
        """
        Starts the main loop to continuously read and print IMU data.

        Args:
            delay_s (float): The time delay in seconds between each reading (default is 0.1s).
        
        Raises:
            KeyboardInterrupt: Raised when the user stops the script with Ctrl+C.
        """
        if not self.sensor._is_initialized:
            print("Error: Application not initialized. Run initialize() first.")
            return

        print(f"\nStarting IMU reading loop (delay={delay_s}s, Press Ctrl+C to stop)...")
        
        # Variable to track time elapsed (Delta Time)
        last_time = time.time()
        
        try:
            while True:
                current_time = time.time()
                dt = current_time - last_time 
                last_time = current_time
                
                # 1. Read Raw Data
                accel_data = self.sensor.get_acceleration_g() # (ax, ay, az)
                gyro_data = self.sensor.get_angular_rate_dps() # (gx, gy, gz)
                
                # 2. Update Roll and Pitch using the Complementary Filter (Fusion)
                roll_deg, pitch_deg = self.filter.update_roll_pitch(accel_data, gyro_data, dt)
                
                # 3. Update Yaw (Simple integration)
                yaw_deg = self.filter.update_yaw(gyro_data[2], dt) # gyro_data[2] is Gz
                
                # 4. Display all data
                print(f"Accel (g): X={accel_data[0]:+.3f}, Y={accel_data[1]:+.3f}, Z={accel_data[2]:+.3f} | Gyro (dps): X={gyro_data[0]:+.3f}, Y={gyro_data[1]:+.3f}, Z={gyro_data[2]:+.3f}")
                # print(f"| Fused R/P/Y (deg): R={roll_deg:+.1f}, P={pitch_deg:+.1f}, Y={yaw_deg:+.1f}")
                
                time.sleep(delay_s)

        except KeyboardInterrupt:
            print("\nScript terminated by user.")
            
        except Exception as e:
            print(f"\nAn unexpected error occurred during the loop: {e}")


    def cleanup(self) -> None:
        """
        Closes the I2C bus and performs final application shutdown.
        """
        del self.sensor 
        print("Cleanup complete. Application exit.")