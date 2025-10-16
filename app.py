import sys
import time
from lsm6dsox import LSM6DSOX_IMU
from settings import I2C_BUS_NUM

class IMU_Application:
    """
    Main application controller for running the LSM6DSOX data acquisition loop.

    This class handles the initialization flow, the continuous data reading loop, 
    and proper cleanup of system resources (I2C bus).
    """

    def __init__(self):
        """
        Initializes the application and creates the IMU sensor object.
        """
        self.sensor = LSM6DSOX_IMU(bus_num=I2C_BUS_NUM)

    def initialize(self):
        """
        Initializes the sensor hardware and configuration via the LSM6DSOX_IMU class.

        Returns:
            bool: True if sensor initialization was successful.
        """
        print("Initializing LSM6DSOX IMU...")
        return self.sensor.initialize()

    def run(self, delay_s=0.1):
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
        
        try:
            while True:
                ax, ay, az = self.sensor.get_acceleration_g()
                gx, gy, gz = self.sensor.get_angular_rate_dps()
                
                print(f"Accel (g): X={ax:+.3f}, Y={ay:+.3f}, Z={az:+.3f} | Gyro (dps): X={gx:+.3f}, Y={gy:+.3f}, Z={gz:+.3f}")
                
                time.sleep(delay_s)

        except KeyboardInterrupt:
            print("\nScript terminated by user.")
            
        except Exception as e:
            print(f"\nAn unexpected error occurred during the loop: {e}")

    def cleanup(self):
        """
        Closes the I2C bus and performs final application shutdown.
        """
        del self.sensor 
        print("Cleanup complete. Application exit.")