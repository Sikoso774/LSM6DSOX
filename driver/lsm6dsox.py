from smbus2 import SMBus
import time
import math
from typing import Tuple
from driver.settings import *

class LSM6DSOX_IMU:
    """
    Interface driver for the STMicroelectronics LSM6DSOX Inertial Measurement Unit (IMU).

    This class provides methods to initialize the sensor via I2C,
    configure the Accelerometer and Gyroscope, and read raw and converted
    data (g-force and degrees per second)..
    """
    
    def __init__(self, bus_num=I2C_BUS_NUM) -> None:
        """
        Initializes the LSM6DSOX_IMU instance.

        Args:
            bus_num (int): The I2C bus number (e.g., 1 for Raspberry Pi).
        """
        self.bus_num = bus_num
        self.bus = None
        self._is_initialized = False
    
    def __del__(self) -> None:
        """
        Destructor: Ensures the I2C bus is closed when the object is deleted.
        """
        if self.bus:
            self.bus.close()
            print("I2C bus closed.")

    def initialize(self) -> bool:
        """
        Opens the I2C bus, performs a soft reset, verifies the chip ID (0x6C), 
        and configures both the Accelerometer and Gyroscope.

        The current configuration is set in settings.py (26 Hz ODR, +/-2g, +/-250dps).

        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        if self._is_initialized:
            print("Sensor is already initialized.")
            return True
            
        try:
            self.bus = SMBus(self.bus_num)
            device_id = self.bus.read_byte_data(LSM6DSOX_ADDR, WHO_AM_I_REG)
            print(f"LSM6DSOX IMU detected. ID : {device_id:#x}")
            if device_id != WHO_AM_I_VALUE:
                raise IOError(f"LSM6DSOX ID mismatch. Expected: {WHO_AM_I_VALUE:#x}, Read: {device_id:#x}")

            print("Applying soft reset...")
            self.bus.write_byte_data(LSM6DSOX_ADDR, CTRL3_C_REG, CTRL3_C_SW_RESET)
            time.sleep(0.05)

            self.bus.write_byte_data(LSM6DSOX_ADDR, CTRL1_XL_REG, CTRL1_XL_CONFIG)
            print("Accelerometer configured (26 Hz, +/-2g).")
            
            self.bus.write_byte_data(LSM6DSOX_ADDR, CTRL2_G_REG, CTRL2_G_CONFIG)
            print("Gyroscope configured (26 Hz, +/-250dps).")
            
            self._is_initialized = True
            return True

        except Exception as e:
            print(f"Failed to initialize LSM6DSOX: {e}")
            if self.bus:
                self.bus.close()
            return False
    
    def _read_raw_data(self, start_reg) -> Tuple[float, float, float]:
        data = self.bus.read_i2c_block_data(LSM6DSOX_ADDR, start_reg, 6)
        
        raw_x = (data[1] << 8 | data[0])
        raw_y = (data[3] << 8 | data[2])
        raw_z = (data[5] << 8 | data[4])
        
        # Convert to signed 16-bit value (Two's Complement)
        if raw_x > 32767:
            raw_x -= 65536
        if raw_y > 32767:
            raw_y -= 65536
        if raw_z > 32767: 
            raw_z -= 65536
        
        return raw_x, raw_y, raw_z

    def get_acceleration_g(self) -> Tuple[float, float, float]:
        """
        Reads raw data from the Accelerometer and converts it to g-force values.

        The conversion uses the SENSITIVITY_ACCEL_2G constant defined in settings.py.

        Returns:
            tuple: (ax, ay, az) where each value is the acceleration in g-force.
        
        Raises:
            Exception: If the sensor has not been successfully initialized.
        """
        if not self._is_initialized:
            raise Exception('Sensor must be initialized before reading data')
            
        raw_x, raw_y, raw_z = self._read_raw_data(OUT_XL_L)
        
        accel_x = raw_x / SENSITIVITY_ACCEL_2G
        accel_y = raw_y / SENSITIVITY_ACCEL_2G
        accel_z = raw_z / SENSITIVITY_ACCEL_2G
        
        return accel_x, accel_y, accel_z
    
    def get_angular_rate_dps(self) -> Tuple[float, float, float]:
        """
        Reads raw data from the Gyroscope and converts it to degrees per second (dps).

        The conversion uses the SENSITIVITY_GYRO_250DPS constant defined in settings.py.

        Returns:
            tuple: (gx, gy, gz) where each value is the angular rate in dps.
            
        Raises:
            Exception: If the sensor has not been successfully initialized.
        """
        if not self._is_initialized:
            raise Exception("Sensor must be initialized before reading data.")
            
        raw_x, raw_y, raw_z = self._read_raw_data(OUT_GYRO_L)
        
        angular_rate_x = raw_x / SENSITIVITY_GYRO_250DPS
        angular_rate_y = raw_y / SENSITIVITY_GYRO_250DPS
        angular_rate_z = raw_z / SENSITIVITY_GYRO_250DPS
        
        return angular_rate_x, angular_rate_y, angular_rate_z

    def get_roll_pitch_angle(self) -> Tuple[float, float]:
        """
        Calculates the static Roll and Pitch angles in degrees using Accelerometer data.
        
        Note: These angles are reliable only when the sensor is static.

        Returns:
            tuple: (roll, pitch) in degrees.
        """
        if not self._is_initialized:
            raise Exception("Sensor must be initialized before reading data.")
            
        # 1. Read accelerations in g
        ax, ay, az = self.get_acceleration_g()

        # 2. Calculate Roll (rotation around X-axis) in radians
        # Formula: atan2(Ay, Az)
        roll_rad = math.atan2(ay, az)
        
        # 3. Calculate Pitch (rotation around Y-axis) in radians
        # Formula: atan2(-Ax, sqrt(Ay^2 + Az^2))
        pitch_rad = math.atan2(-ax, math.sqrt(ay*ay + az*az))

        # 4. Convert Radians to Degrees
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        
        return roll_deg, pitch_deg

