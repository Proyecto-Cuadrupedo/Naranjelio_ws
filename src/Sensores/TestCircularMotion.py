import Adafruit_PCA9685
import time
import math
from multiprocessing import Process
import numpy as np
#import keyboard

# Initialize the PCA9685 using the default address (0x40).
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



class Leg:
    def __init__(self) -> None:

        self.angfemurmax=100
        self.angfemurmin=50
        self.angtibiamax=130
        self.angtibiamin=75
        
        self.servos=[[0,1,2],[4,5,6],[8,9,10],[12,13,14]]
        
        #self.minfemur=[40,90,40,90]
        self.minfemur=[50,120,120,50]
        
        
        #self.maxfemur=[90,140,90,140]
        self.maxfemur=[80,150,150,80]
        
        self.ampfemur=[30,30,30,30]
        
        #self.mintibia=[50,50,50,50]
        #self.maxtibia=[110,110,110,110]
        
        self.mintibia=[75,75,75,75]
        self.maxtibia=[125,125,125,125]
        self.amptibia=[50,50,50,50]

        
        self.speed=0.25
        self.samples=50
        
        self.compfemur=[0,0,-10,0]
        self.comptibia=[0,0,-30,20]
        
    def initial_values(self,leg):
        femurnowinit=90
        dirfeminit=1
        
        tibianowinit=90
        dirtibiainit=0
        
        return femurnowinit,dirfeminit,tibianowinit,dirtibiainit
    def initfront(self, leg):
        if leg == 1:
            move_servo(1,60)
            move_servo(2,120)
            
            move_servo(0,135)
        if leg == 2:
            move_servo(5,130)
            move_servo(6,70)
            
            move_servo(4,60)
        if leg == 3:
            move_servo(9,120)
            move_servo(10,70)
            
            move_servo(8,60)
        if leg == 4:
            move_servo(13,60)
            move_servo(14,120)
            
            move_servo(12,145)
            
    def front(self,leg,point):
        if point > 2*math.pi/self.speed:
            point = point - 2*math.pi/self.speed
        
        if leg==1 or leg==3:
            femurnow=(self.ampfemur[leg-1]/2)*math.cos(point*self.speed)   +   (self.maxfemur[leg-1]-(self.ampfemur[leg-1]/2))+self.compfemur[leg-1]
        elif leg==2 or leg==4:
            femurnow=-(self.ampfemur[leg-1]/2)*math.cos(point*self.speed)   +   (self.maxfemur[leg-1]-(self.ampfemur[leg-1]/2))+self.compfemur[leg-1]
        
        if leg == 2 or leg == 3:
            if point <= self.samples*0.68:
                tibianow=1-(-self.amptibia[leg-1]/(1+math.exp((-5*self.speed*point)+1.6*6)))+self.mintibia[leg-1]-1+self.comptibia[leg-1]
            elif point > self.samples*0.68:
                tibianow=-1+(-self.amptibia[leg-1]/(1+math.exp((-6.8*self.speed*point)+6.2*6)))+self.maxtibia[leg-1]+1+self.comptibia[leg-1]
        elif leg == 1 or leg == 4:
            if point <= self.samples*0.68:
                tibianow=1-(self.amptibia[leg-1]/(1+math.exp((-5*self.speed*point)+1.6*6)))+self.maxtibia[leg-1]-1+self.comptibia[leg-1]
            elif point > self.samples*0.68:
                tibianow=-1+(self.amptibia[leg-1]/(1+math.exp((-6.8*self.speed*point)+6.2*6)))+self.mintibia[leg-1]+1+self.comptibia[leg-1]

        move_servo(self.servos[leg-1][1],femurnow)
        move_servo(self.servos[leg-1][2],tibianow)
    
        
    
    def back_fr_bl(self, femurnum,tibianum,femurnow,dirfem, tibianow, dirtibia ):
        if dirfem==1:
            if femurnow<self.angfemurmax:
                femurnow+=1
                move_servo(femurnum,femurnow)
                
                if dirtibia==1:
                    if tibianow<self.angtibiamax:
                        tibianow+=6
                        move_servo(tibianum,tibianow)
                    else:
                        dirtibia=0     
                
            else:
                dirfem=0
                

        if dirfem==0:
            if femurnow>self.angfemurmin:
                femurnow-=1
                move_servo(femurnum,femurnow)
                
                if dirtibia==0:
                    if tibianow>self.angtibiamin:
                        tibianow-=3
                        move_servo(tibianum,tibianow)
                    else:
                        dirtibia=1
                
            else:
                dirfem=1
                

        time.sleep(0.02)
        return femurnow,dirfem,tibianow,dirtibia

frontr=Leg()
frontr.initfront(1)
backl=Leg()
backl.initfront(4)

frontl=Leg()
frontl.initfront(2)
backr=Leg()
backr.initfront(3)

time.sleep(2)
steps=np.linspace(0,(2*math.pi/frontr.speed),num=frontr.samples)
cuarto=round((2*math.pi/frontr.speed)/4)

dt=0.015
#"""
for reps in range(1,10):    
    for i in steps:
        frontr.front(2,i)
        time.sleep(dt)
        frontr.front(3,i+cuarto)
        time.sleep(dt)
        frontr.front(1,i+cuarto*2)
        time.sleep(dt)
        frontr.front(4,i+cuarto*3)
        time.sleep(dt)
    
#"""

