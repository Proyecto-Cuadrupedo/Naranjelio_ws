import Adafruit_PCA9685
import time

# Initialize the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
print("HOLA")
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
print("HOLA2")
# Function to move servo
def move_servo(channel, angle):
    servo_min = 150  # Min pulse length out of 4096
    servo_max = 600  # Max pulse length out of 4096
    
    # Convert angle to pulse
    pulse = servo_min + (servo_max - servo_min) * angle / 180.0
    pwm.set_pwm(channel, 0, int(pulse))

# Example usage
# Uncomment the lines below to test the functions with your servos and PCA9685
#for i in range(1,180):
#    move_servo(0, i) # Move servo on channel 0 to 90 degrees
#    time.sleep(100)
#    print(i)


#Front Right
move_servo(0, 145)
move_servo(1, 90)
move_servo(2, 90)

#Front Left
move_servo(4, 65)
move_servo(5, 90)
move_servo(6, 90)

#Back Right
move_servo(8, 65)
move_servo(9, 90)
move_servo(10, 90)

#Back Left
move_servo(12, 145)
move_servo(13, 90)
move_servo(14, 90)

tiempo=0.3
while True:
    move_servo(12, 160)
    move_servo(8,80)
    
    #Tibias
    move_servo(2,140)
    move_servo(6,60)
    
    #Femurs
    move_servo(1,80)
    move_servo(5,100)
    
    time.sleep(tiempo)
    move_servo(12, 145)
    move_servo(8,65)
    time.sleep(tiempo)
    move_servo(12, 130)
    move_servo(8,50)
    
    move_servo(2,80)
    move_servo(6,120)
    
    move_servo(1,100)
    move_servo(5,80)
    
    time.sleep(tiempo)
    move_servo(12, 145)
    move_servo(8,65)
    time.sleep(tiempo)
    
    
#while True:
#    grados=int(input("Grados Tibia: "))
#    move_servo(2, grados)
 #   grados=int(input("Grados Tibia: "))
 #   move_servo(6, grados)
 #   # Move servo on channel 1 to 45 degrees
# move_servo(2, 180) # Move servo on channel 2 to 180 degrees

# Note: Adjust the servo_min and servo_max values according to your servo's specifications.
# The example assumes a standard servo with 180 degrees of movement.
