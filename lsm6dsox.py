from smbus2 import SMBus
import time
from settings import *

class LSM6DSOX_IMU:
    """
    Interface driver for the STMicroelectronics LSM6DSOX Inertial Measurement Unit (IMU).

    This class provides methods to initialize the sensor via I2C,
    configure the Accelerometer and Gyroscope, and read raw and converted
    data (g-force and degrees per second)..
    """
    

    def __init__(self, bus_num=I2C_BUS_NUM): # Utilisation de la constante par défaut
        """
        Initializes the LSM6DSOX_IMU instance.

        Args:
            bus_num (int): The I2C bus number (e.g., 1 for Raspberry Pi).
        """
        self.bus_num = bus_num
        self.bus = None
        self._is_initialized = False
    
    def __del__(self):
        """
        Destructor: Ensures the I2C bus is closed when the object is deleted.
        """
        if self.bus:
            self.bus.close()
            print("I2C bus closed.")

    def initialize(self):
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
            # 1. Open the I2C Bus
            self.bus = SMBus(self.bus_num)

            # 2. Verify communication (WHO_AM_I)
            device_id = self.bus.read_byte_data(LSM6DSOX_ADDR, WHO_AM_I_REG)
            # ... (vérification de l'ID inchangée) ...
            print(f"LSM6DSOX IMU detected. ID : {device_id:#x}")

            # 3. NOUVEAU : Soft Reset pour s'assurer que le capteur est propre
            print("Applying soft reset...")
            self.bus.write_byte_data(LSM6DSOX_ADDR, CTRL3_C_REG, CTRL3_C_SW_RESET)
            time.sleep(0.05) # Attendre que le reset soit effectué (typiquement 50ms suffisent)

            # 4. Configure the Accelerometer 
            self.bus.write_byte_data(LSM6DSOX_ADDR, CTRL1_XL_REG, CTRL1_XL_CONFIG)
            print("Accelerometer configured (26 Hz, +/-2g).")
            
            # 5. Configure the Gyroscope 
            self.bus.write_byte_data(LSM6DSOX_ADDR, CTRL2_G_REG, CTRL2_G_CONFIG)
            print("Gyroscope configured (26 Hz, +/-250dps).")
            
            self._is_initialized = True
            return True

        except Exception as e:
            print(f"Failed to initialize LSM6DSOX: {e}")
            if self.bus:
                self.bus.close()
            return False
    

    def _read_raw_data(self, start_reg):
        # Start reading from the specified register and read 6 consécutive bytes
        data = self.bus.read_i2c_block_data(LSM6DSOX_ADDR, start_reg, 6)
        
        # Combne the Low and High bytes (L + H*256)
        raw_x = (data[1] << 8 | data[0])
        raw_y = (data[3] << 8 | data[2])
        raw_z = (data[5] << 8 | data[4])
        
        if raw_x > 32767:
            raw_x -= 65536
        if raw_y > 32767:
            raw_y -= 65536
        if raw_y > 32767:
            raw_y -= 65536
        
        return raw_x, raw_y, raw_z

    def get_acceleration_g(self):
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
        
        # Convert raw values to 'g' by dividing by the sensitivity factor
        # Utilisation de la constante importée
        accel_x = raw_x / SENSITIVITY_ACCEL_2G
        accel_y = raw_y / SENSITIVITY_ACCEL_2G
        accel_z = raw_z / SENSITIVITY_ACCEL_2G
        
        return accel_x, accel_y, accel_z
    
    def get_angular_rate_dps(self):
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
            
        # Appel à la méthode refactorisée avec le registre du gyroscope
        raw_x, raw_y, raw_z = self._read_raw_data(OUT_GYRO_L)
        
        # Convert raw values to 'dps' 
        angular_rate_x = raw_x / SENSITIVITY_GYRO_250DPS
        angular_rate_y = raw_y / SENSITIVITY_GYRO_250DPS
        angular_rate_z = raw_z / SENSITIVITY_GYRO_250DPS
        
        return angular_rate_x, angular_rate_y, angular_rate_z