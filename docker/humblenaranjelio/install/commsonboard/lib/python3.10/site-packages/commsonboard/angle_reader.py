import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import Adafruit_PCA9685



import math
from smbus import SMBus
from time import sleep

# I2C device addresses
LIS3MDL_ADDR = 0x1e      # Magnetometer
LPS25H_ADDR = 0x5d      # Barometric pressure sensor
LSM6DS33_ADDR = 0x6b      # Gyrometer / accelerometer

# LSM6DS33 gyroscope and accelerometer control registers
LSM6DS33_CTRL1_XL = 0x10  # Acceleration sensor control
LSM6DS33_CTRL2_G = 0x11  # Angular rate sensor (gyroscope) control

# LSM6DS33 Gyroscope and accelerometer output registers
LSM6DS33_OUTX_L_G = 0x22  # Gyroscope pitch axis (X) output, low byte
LSM6DS33_OUTX_H_G = 0x23  # Gyroscope pitch axis (X) output, high byte
LSM6DS33_OUTY_L_G = 0x24  # Gyroscope roll axis (Y) output, low byte
LSM6DS33_OUTY_H_G = 0x25  # Gyroscope roll axis (Y) output, high byte
LSM6DS33_OUTZ_L_G = 0x26  # Gyroscope yaw axis (Z) output, low byte
LSM6DS33_OUTZ_H_G = 0x27  # Gyroscope yaw axis (Z) output, high byte
LSM6DS33_OUTX_L_XL = 0x28  # Accelerometer pitch axis (X) output, low byte
LSM6DS33_OUTX_H_XL = 0x29  # Accelerometer pitch axis (X) output, high byte
LSM6DS33_OUTY_L_XL = 0x2A  # Accelerometer roll axis (Y) output, low byte
LSM6DS33_OUTY_H_XL = 0x2B  # Accelerometer roll axis (Y) output, high byte
LSM6DS33_OUTZ_L_XL = 0x2C  # Accelerometer yaw axis (Z) output, low byte
LSM6DS33_OUTZ_H_XL = 0x2D  # Accelerometer yaw axis (Z) output, high byte

# Control registers for magnetometer
# Enable device, set operating modes and rates for X and Y axes
LIS3MDL_CTRL_REG1 = 0x20
LIS3MDL_CTRL_REG2 = 0x21   # Set gauss scale
LIS3MDL_CTRL_REG3 = 0x22   # Set operating/power modes
LIS3MDL_CTRL_REG4 = 0x23   # Set operating mode and rate for Z-axis

# Output registers for magnetometer
LIS3MDL_OUT_X_L = 0x28   # X output, low byte
LIS3MDL_OUT_X_H = 0x29   # X output, high byte
LIS3MDL_OUT_Y_L = 0x2A   # Y output, low byte
LIS3MDL_OUT_Y_H = 0x2B   # Y output, high byte
LIS3MDL_OUT_Z_L = 0x2C   # Z output, low byte
LIS3MDL_OUT_Z_H = 0x2D   # Z output, high byte

# Control registers for the digital barometer
LPS25H_CTRL_REG1 = 0x20  # Set device power mode / ODR / BDU

# Output registers for the digital barometer
LPS25H_PRESS_OUT_XL = 0x28  # Pressure output, loweste byte
LPS25H_PRESS_OUT_L = 0x29   # Pressure output, low byte
LPS25H_PRESS_OUT_H = 0x2A   # Pressure output, high byte

# Gyroscope dps/LSB for 1000 dps full scale
GYRO_GAIN = 35.0

# Accelerometer conversion factor for +/- 4g full scale
ACCEL_CONVERSION_FACTOR = 0.122


class I2C(object):
    """ Class to set up and access I2C devices.
    """

    def __init__(self, bus_id=1):
        """ Initialize the I2C bus. """
        self._i2c = SMBus(bus_id)

    def __del__(self):
        """ Clean up. """
        try:
            del(self._i2c)
        except:
            pass

    def write_register(self, address, register, value):
        """ Write a single byte to a I2C register. Return the value the
            register had before the write.
        """
        value_old = self.read_register(address, register)
        self._i2c.write_byte_data(address, register, value)
        return value_old

    def read_register(self, address, register):
        """ Read a single I2C register. """
        return self._i2c.read_byte_data(address, register)

    def combine_lo_hi(self, lo_byte, hi_byte):
        """ Combine low and high bytes to an unsigned 16 bit value. """
        return (hi_byte << 8) | lo_byte

    def combine_signed_lo_hi(self, lo_byte, hi_byte):
        """ Combine low and high bytes to a signed 16 bit value. """
        combined = self.combine_lo_hi(lo_byte, hi_byte)
        return combined if combined < 32768 else (combined - 65536)

    def combine_xlo_lo_hi(self, xlo_byte, lo_byte, hi_byte):
        """ Combine extra low, low, and high bytes to an unsigned
            24 bit value.
        """
        return (xlo_byte | lo_byte << 8 | hi_byte << 16)

    def combine_signed_xlo_lo_hi(self, xlo_byte, lo_byte, hi_byte):
        """ Combine extra low, low, and high bytes to a signed 24 bit value. """
        combined = self.combine_xlo_lo_hi(xlo_byte, lo_byte, hi_byte)
        return combined if combined < 8388608 else (combined - 16777216)

    def read_1d_sensor(self, address, registers):
        """ Return a vector with the combined raw signed 24 bit values
            of the output registers of a 1d sensor.
        """

        xlo_byte = self.read_register(address, registers[0])
        lo_byte = self.read_register(address, registers[1])
        hi_byte = self.read_register(address, registers[2])

        return self.combine_signed_xlo_lo_hi(xlo_byte, lo_byte, hi_byte)

    def read_3d_sensor(self, address, registers):
        """ Return a vector with the combined raw signed 16 bit values
            of the output registers of a 3d sensor.
        """

        # Read register outputs and combine low and high byte values
        x_low = self.read_register(address, registers[0])
        x_hi = self.read_register(address, registers[1])
        y_low = self.read_register(address, registers[2])
        y_hi = self.read_register(address, registers[3])
        z_low = self.read_register(address, registers[4])
        z_hi = self.read_register(address, registers[5])

        x_val = self.combine_signed_lo_hi(x_low, x_hi)
        y_val = self.combine_signed_lo_hi(y_low, y_hi)
        z_val = self.combine_signed_lo_hi(z_low, z_hi)

        return [x_val, y_val, z_val]


class LSM6DS33(I2C):
    """ Set up and access LSM6DS33 accelerometer and gyroscope.
    """

    # Output registers used by the gyroscope
    gyro_registers = [
        LSM6DS33_OUTX_L_G,  # low byte of X value
        LSM6DS33_OUTX_H_G,  # high byte of X value
        LSM6DS33_OUTY_L_G,  # low byte of Y value
        LSM6DS33_OUTY_H_G,  # high byte of Y value
        LSM6DS33_OUTZ_L_G,  # low byte of Z value
        LSM6DS33_OUTZ_H_G,  # high byte of Z value
    ]

    # Output registers used by the accelerometer
    accel_registers = [
        LSM6DS33_OUTX_L_XL,  # low byte of X value
        LSM6DS33_OUTX_H_XL,  # high byte of X value
        LSM6DS33_OUTY_L_XL,  # low byte of Y value
        LSM6DS33_OUTY_H_XL,  # high byte of Y value
        LSM6DS33_OUTZ_L_XL,  # low byte of Z value
        LSM6DS33_OUTZ_H_XL,  # high byte of Z value
    ]

    def __init__(self, bus_id=1):
        """ Set up I2C connection and initialize some flags and values.
        """

        super(LSM6DS33, self).__init__(bus_id)
        self.is_accel_enabled = False
        self.is_gyro_enabled = False

        self.is_gyro_calibrated = False
        self.gyro_cal = [0, 0, 0]

        self.is_accel_calibrated = False
        self.accel_angle_cal = [0, 0]

    def __del__(self):
        """ Clean up."""
        try:
            # Power down accelerometer and gyro
            self.writeRegister(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x00)
            self.writeRegister(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, 0x00)
            super(LSM6DS33, self).__del__()
            print('Destroying')
        except:
            pass

    def enable(self, accelerometer=True, gyroscope=True, calibration=True):
        """ Enable and set up the given sensors in the IMU."""
        if accelerometer:
            # 1.66 kHz (high performance) / +/- 4g
            # binary value -> 0b01011000, hex value -> 0x58
            self.write_register(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x58)
            self.is_accel_enabled = True
        if gyroscope:
            # 208 Hz (high performance) / 1000 dps
            # binary value -> 0b01011000, hex value -> 0x58
            self.write_register(LSM6DS33_ADDR, LSM6DS33_CTRL2_G, 0x58)
            self.is_gyro_enabled = True
        if calibration:
            self.calibrate()
            self.is_gyro_calibrated = True
            self.is_accel_calibrated = True

    def calibrate(self, iterations=2000):
        """ Calibrate the gyro's raw values."""
        print('Calibrating Gryo and Accelerometer...')

        for i in range(iterations):
            gyro_raw = self.get_gyroscope_raw()
            accel_angles = self.get_accelerometer_angles()

            self.gyro_cal[0] += gyro_raw[0]
            self.gyro_cal[1] += gyro_raw[1]
            self.gyro_cal[2] += gyro_raw[2]

            self.accel_angle_cal[0] += accel_angles[0]
            self.accel_angle_cal[1] += accel_angles[1]

            sleep(0.004)

        self.gyro_cal[0] /= iterations
        self.gyro_cal[1] /= iterations
        self.gyro_cal[2] /= iterations

        self.accel_angle_cal[0] /= iterations
        self.accel_angle_cal[1] /= iterations

        print('Calibration Done')

    def get_gyroscope_raw(self):
        """ Return a 3D vector of raw gyro data.
        """
        # Check if gyroscope has been enabled
        if not self.is_gyro_enabled:
            raise(Exception('Gyroscope is not enabled!'))

        sensor_data = self.read_3d_sensor(LSM6DS33_ADDR, self.gyro_registers)

        # Return the vector
        if self.is_gyro_calibrated:
            calibrated_gyro_data = sensor_data
            calibrated_gyro_data[0] -= self.gyro_cal[0]
            calibrated_gyro_data[1] -= self.gyro_cal[1]
            calibrated_gyro_data[2] -= self.gyro_cal[2]
            return calibrated_gyro_data
        else:
            return sensor_data

    def get_gyro_angular_velocity(self):
        """ Return a 3D vector of the angular velocity measured by the gyro
            in degrees/second.
        """
        # Check if gyroscope has been enabled
        if not self.is_gyro_enabled:
            raise(Exception('Gyroscope is not enabled!'))

        # Check if gyroscope has been calibrated
        if not self.is_gyro_calibrated:
            raise(Exception('Gyroscope is not calibrated!'))

        gyro_data = self.get_gyroscope_raw()

        gyro_data[0] = (gyro_data[0] * GYRO_GAIN) / 1000
        gyro_data[1] = (gyro_data[1] * GYRO_GAIN) / 1000
        gyro_data[2] = (gyro_data[2] * GYRO_GAIN) / 1000

        return gyro_data

    def get_accelerometer_raw(self):
        """ Return a 3D vector of raw accelerometer data.
        """

        # Check if accelerometer has been enabled
        if not self.is_accel_enabled:
            raise(Exception('Accelerometer is not enabled!'))

        return self.read_3d_sensor(LSM6DS33_ADDR, self.accel_registers)

    def get_accelerometer_g_forces(self):
        """ Return a 3D vector of the g forces measured by the accelerometer"""
        [x_val, y_val, z_val] = self.get_accelerometer_raw()

        x_val = (x_val * ACCEL_CONVERSION_FACTOR) / 1000
        y_val = (y_val * ACCEL_CONVERSION_FACTOR) / 1000
        z_val = (z_val * ACCEL_CONVERSION_FACTOR) / 1000

        return [x_val, y_val, z_val]

    def get_accelerometer_angles(self, round_digits=0):
        """ Return a 2D vector of roll and pitch angles,
            based on accelerometer g forces
        """

        # Get raw accelerometer g forces
        [acc_xg_force, acc_yg_force, acc_zg_force] = self.get_accelerometer_g_forces()

        # Calculate angles
        xz_dist = self._get_dist(acc_xg_force, acc_zg_force)
        yz_dist = self._get_dist(acc_yg_force, acc_zg_force)
        accel_roll_angle = math.degrees(math.atan2(acc_yg_force, xz_dist))
        accel_pitch_angle = -math.degrees(math.atan2(acc_xg_force, yz_dist))

        if self.is_accel_calibrated:
            accel_roll_angle -= self.accel_angle_cal[0]
            accel_pitch_angle -= self.accel_angle_cal[1]
            if round_digits != 0:
                return [round(accel_roll_angle, round_digits), round(accel_pitch_angle, round_digits)]
            else:
                return [accel_roll_angle, accel_pitch_angle]
        else:
            return [accel_roll_angle, accel_pitch_angle]

    def _get_dist(self, a, b):
        return math.sqrt((a * a) + (b * b))
    
class LIS3MDL(I2C):
    """ Set up and access LIS3MDL magnetometer.
    """

    # Output registers used by the magnetometer
    magnetometer_registers = [
        LIS3MDL_OUT_X_L,  # low byte of X value
        LIS3MDL_OUT_X_H,  # high byte of X value
        LIS3MDL_OUT_Y_L,  # low byte of Y value
        LIS3MDL_OUT_Y_H,  # high byte of Y value
        LIS3MDL_OUT_Z_L,  # low byte of Z value
        LIS3MDL_OUT_Z_H,  # high byte of Z value
    ]

    def __init__(self, bus_id=1):
        """ Set up I2C connection and initialize some flags and values.
        """

        super(LIS3MDL, self).__init__(bus_id)
        self.is_magnetometer_enabled = False

    def __del__(self):
        """ Clean up. """
        try:
            # Power down magnetometer
            self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x03)
            super(LIS3MDL, self).__del__()
        except:
            pass

    def enable(self):
        """ Enable and set up the the magnetometer and determine
            whether to auto increment registers during I2C read operations.
        """

        # Disable magnetometer and temperature sensor first
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x00)
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x03)

        # Enable device in continuous conversion mode
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00)

        # Initial value for CTRL_REG1
        ctrl_reg1 = 0x00

        # Ultra-high-performance mode for X and Y
        # Output data rate 10Hz
        # binary value -> 01110000b, hex value -> 0x70
        ctrl_reg1 += 0x70

        # +/- 4 gauss full scale
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x00)

        # Ultra-high-performance mode for Z
        # binary value -> 00001100b, hex value -> 0x0c
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4, 0x0c)

        self.is_magnetometer_enabled = True

        # Write calculated value to the CTRL_REG1 register
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, ctrl_reg1)

    def get_magnetometer_raw(self):
        """ Return 3D vector of raw magnetometer data.
        """
        # Check if magnetometer has been enabled
        if not self.is_magnetometer_enabled:
            raise(Exception('Magnetometer is not enabled'))

        return self.read_3d_sensor(LIS3MDL_ADDR, self.magnetometer_registers)
    






pwm = Adafruit_PCA9685.PCA9685()
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print(f"{pulse_length}us per period")
    pulse_length //= 4096     # 12 bits of resolution
    print(f"{pulse_length}us per bit")
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
# Function to move servo
def move_servo(channel, angle):
    servo_min = 150  # Min pulse length out of 4096
    servo_max = 600  # Max pulse length out of 4096
    
    # Convert angle to pulse
    pulse = servo_min + (servo_max - servo_min) * angle / 180.0
    pwm.set_pwm(channel, 0, int(pulse))



class MotorAngleSubscriber(Node):
    def __init__(self):
        super().__init__('motor_angle_subscriber')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'IMU', 10)
        self.subscription = self.create_subscription(Float64MultiArray, 'motorangles', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.compass=LIS3MDL()
        self.compass.enable()    
        self.lsm6ds33 = LSM6DS33()
        self.lsm6ds33.enable()
        
        self.timer = self.create_timer(0.01, self.IMUdata)
        self.pca = Adafruit_PCA9685.PCA9685()
        

    def listener_callback(self, msg):
        self.get_logger().info('Received motor angles: "%s"' % msg.data)
        # Example: set motor 0 to the first angle value
        FR = [msg.data[0],msg.data[1],msg.data[2]]
        FL= [msg.data[3],msg.data[4],msg.data[5]]
        BR= [msg.data[6],msg.data[7],msg.data[8]]
        BL= [msg.data[9],msg.data[10],msg.data[11]]
        
        move_servo(0,FR[0])
        move_servo(1,FR[1])
        move_servo(2,FR[2])
        
        move_servo(4,FL[0])
        move_servo(5,FL[1])
        move_servo(6,FL[2])
        
        move_servo(8,BR[0])
        move_servo(9,BR[1])
        move_servo(10,BR[2])
        
        move_servo(12,BL[0])
        move_servo(13,BL[1])
        move_servo(14,BL[2])
        
    def IMUdata(self):
        
        accel=self.lsm6ds33.get_accelerometer_g_forces()

        gyro=self.lsm6ds33.get_gyro_angular_velocity()

        compass=self.compass.get_magnetometer_raw()
        print([round(accel[0],3),round(accel[1],3),round(accel[2],3),round(gyro[0],3),round(gyro[1],3),round(gyro[2],3),round(compass[0],3),round(compass[1],3),round(compass[2],3)])

        imu=Float64MultiArray()
        imu.data=[float(round(accel[0],3)),float(round(accel[1],3)),float(round(accel[2],3)),
                  float(round(gyro[0],3)),float(round(gyro[1],3)),float(round(gyro[2],3)),
                  float(round(compass[0],3)),float(round(compass[1],3)),float(round(compass[2],3))]
        print(imu.data)
        self.publisher_.publish(imu)
            
        
        

def main(args=None):
    rclpy.init(args=args)
    motor_angle_subscriber = MotorAngleSubscriber()
    rclpy.spin(motor_angle_subscriber)
    motor_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()