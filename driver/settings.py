# --- Hardware and I2C Configuration ---
I2C_BUS_NUM = 1                # The I2C bus number on the Raspberry Pi Zero W (typically 1)
LSM6DSOX_ADDR = 0x6A           # The I2C address of the LSM6DSOX sensor (confirmed to be 0x6A)
WHO_AM_I_VALUE = 0x6C          # Expected ID value for the LSM6DSOX sensor (Read from WHO_AM_I register 0x0F)

# --- LSM6DSOX Register Addresses ---
WHO_AM_I_REG = 0x0F            # Register for chip identification
CTRL1_XL_REG = 0x10            # Register for Accelerometer configuration (Output Data Rate / Full Scale)
CTRL2_G_REG = 0x11             # Register for Gyroscope configuration
OUT_XL_L = 0x28                # First register address for Accelerometer data output (6 bytes sequence)
OUT_GYRO_L = 0x22              # First register address for Gyroscope data output (6 bytes sequence)
CTRL3_C_REG = 0x12             # Register for sensor control, reboot, and block data update (BDU)
CTRL3_C_SW_RESET = 0x01        # Configuration Value for soft reset (Bit 0: SW_RESET = 1)

# --- Accelerometer Specific Configuration (26 Hz, +/-2g) ---
CTRL1_XL_CONFIG = 0x50         # Configuration Value: 26 Hz ODR, +/-2g FS (0x50)
SENSITIVITY_ACCEL_2G = 16384.0 # Sensitivity Factor for +/- 2g range (in LSB/g)

# --- Gyroscope Specific Configuration (26 Hz, +/-250dps) ---
CTRL2_G_CONFIG = 0x50          # Configuration Value: 26 Hz ODR, +/-250dps FS (0x50)
SENSITIVITY_GYRO_250DPS = 114.28 # Sensitivity Factor for +/- 250 dps range (in LSB/dps)

