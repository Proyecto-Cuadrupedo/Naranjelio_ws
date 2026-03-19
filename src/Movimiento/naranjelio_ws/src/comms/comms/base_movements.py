import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math
import numpy as np

class MotorAnglePublisher(Node):
    def __init__(self):
        super().__init__('motor_angle_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motorangles', 10)
        #self.timer = self.create_timer(0.5, self.publish_motor_angles)
        #self.i = 0
        
        # Initial positions
        # Hip, Femur Tibia        
        
        self.speed=0.2
        self.samples=100      
        
        #self.minfemur=[40,90,40,90]
        self.minfemur=[50,120,120,50]
        
        
        #self.maxfemur=[90,140,90,140]
        self.maxfemur=[80,150,150,80]
        
        self.ampfemur=[30,30,30,30]
        
        #self.mintibia=[50,50,50,50]
        #self.maxtibia=[110,110,110,110]
        
        self.mintibia=[100,60,60,100]
        self.maxtibia=[140,100,100,140]
        self.amptibia=[40,40,40,40]
        
        self.compfemur=[0,0,-12,0]
        self.comptibia=[0,0,-30,20]
        
        input("Avise pa empezar")
        gaits=[self.frontA, self.frontB]
        while True:
            print("1. Gait A \n 2. Gait B \n 3. Gait C \n")
            gaitn=int(input("Select: "))-1
            if gaitn<3:
                self.samples=(11-int(input("Speed (1-10): ")))*10
                try:
                    loops=int(input("Loops: "))
                except:
                    loops=10
                gait=gaits[gaitn]
                self.walkinggait(gait,loops)
            else:
                self.POS=[[145.1, 60.1+self.compfemur[0], 120.1+self.comptibia[0]], 
                [55.1, 130.1+self.compfemur[1], 70.1+self.comptibia[1]],
                [50.1, 130.1+self.compfemur[2], 70.1+self.comptibia[2]],
                [145.1,60.1+self.compfemur[3],120.1+self.comptibia[3]]]
                self.publish_motor_angles()
        
    def frontA(self,leg,point):
        if point > 2*math.pi/self.speed:
            point = point - 2*math.pi/self.speed
        
        if leg==1 or leg==3:
            femurnow=(self.ampfemur[leg-1]/2)*math.cos(point*self.speed)   +   (self.maxfemur[leg-1]-(self.ampfemur[leg-1]/2))+self.compfemur[leg-1]
        elif leg==2 or leg==4:
            femurnow=-(self.ampfemur[leg-1]/2)*math.cos(point*self.speed)   +   (self.maxfemur[leg-1]-(self.ampfemur[leg-1]/2))+self.compfemur[leg-1]
        
        if leg == 2 or leg == 3:
            if point <= self.samples*0.53:
                tibianow=1-(-self.amptibia[leg-1]/(1+math.exp((-5*self.speed*point)+1.6*6)))+self.mintibia[leg-1]-1+self.comptibia[leg-1]
            elif point > self.samples*0.53:
                tibianow=-1+(-self.amptibia[leg-1]/(1+math.exp((-6.8*self.speed*point)+6.2*6)))+self.maxtibia[leg-1]+1+self.comptibia[leg-1]
        elif leg == 1 or leg == 4:
            if point <= self.samples*0.53:
                tibianow=1-(self.amptibia[leg-1]/(1+math.exp((-5*self.speed*point)+1.6*6)))+self.maxtibia[leg-1]-1+self.comptibia[leg-1]
            elif point > self.samples*0.53:
                tibianow=-1+(self.amptibia[leg-1]/(1+math.exp((-6.8*self.speed*point)+6.2*6)))+self.mintibia[leg-1]+1+self.comptibia[leg-1]

        
        
        self.POS[leg-1][1]=round(femurnow,3)
        self.POS[leg-1][2]=round(tibianow,3)
        self.publish_motor_angles()
        
    def frontB(self,leg,point):
        
        self.minfemurb=[60,110,120,50]
        
        self.maxfemurb=[90,140,150,80]
        
        self.ampfemurb=[30,30,30,30]

        
        self.mintibiab=[75,75,75,75]
        self.maxtibiab=[125,125,125,125]
        self.amptibiab=[50,50,50,50]
        
        
        
        if point > 2*math.pi/self.speed:
            point = point - 2*math.pi/self.speed
        
        if leg==1 or leg==3:
            femurnow=(self.ampfemurb[leg-1]/2)*math.cos(point*self.speed)   +   (self.maxfemurb[leg-1]-(self.ampfemurb[leg-1]/2))+self.compfemur[leg-1]
        elif leg==2 or leg==4:
            femurnow=-(self.ampfemurb[leg-1]/2)*math.cos(point*self.speed)   +   (self.maxfemurb[leg-1]-(self.ampfemurb[leg-1]/2))+self.compfemur[leg-1]
        
        point+=(2*math.pi/self.speed)/10
        if leg == 2  or leg == 3:
            tibianow=(self.amptibiab[leg-1]/2)*math.cos(point*self.speed) + (self.maxtibiab[leg-1]-(self.amptibiab[leg-1]/2))+self.comptibia[leg-1]
        elif leg == 1 or leg == 4:
            tibianow=-(self.amptibiab[leg-1]/2)*math.cos(point*self.speed) + (self.maxtibiab[leg-1]-(self.amptibiab[leg-1]/2))+self.comptibia[leg-1]
        
        
        
        
        self.POS[leg-1][1]=round(femurnow,3)
        self.POS[leg-1][2]=round(tibianow,3)
        print(self.POS)
        self.publish_motor_angles()
    
    def walkinggait(self,selectedgait,loops):
        steps=np.linspace(1,(2*math.pi/self.speed),num=self.samples)
        cuarto=round((2*math.pi/self.speed)/4)

        dt=0.015
        #"""
        for reps in range(0,loops):    
            for i in steps:
                selectedgait(2,i)
                time.sleep(dt)
                selectedgait(3,i+cuarto)
                time.sleep(dt)
                selectedgait(1,i+cuarto*2)
                time.sleep(dt)
                selectedgait(4,i+cuarto*3)
                time.sleep(dt)
            print(reps+1)
        

    def publish_motor_angles(self):
        
        msg = Float64MultiArray()
        msg.data = [self.POS[0][0],self.POS[0][1],self.POS[0][2],
                    self.POS[1][0],self.POS[1][1],self.POS[1][2],
                    self.POS[2][0],self.POS[2][1],self.POS[2][2],
                    self.POS[3][0],self.POS[3][1],self.POS[3][2]]  # data
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        
        
        
        
        
        
        
        
        
        

def main(args=None):
    rclpy.init(args=args)
    motor_angle_publisher = MotorAnglePublisher()
    rclpy.spin(motor_angle_publisher)
    motor_angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
