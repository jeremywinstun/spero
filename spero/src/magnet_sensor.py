#!/usr/bin/python

from __future__ import division
import rospy
import RPi.GPIO as GPIO
from spero.msg import FollowLine
import time
from platform import python_version
from simple_pid import PID


class MagnetSensor():
    def __init__(self):
        #Setting up ROS
        rospy.init_node("magnet_sensor")
        rospy.loginfo("Seting Up the node...")
        
        self.ros_pub_follow_line = rospy.Publisher("/follow_line", FollowLine, queue_size = 1)
        rospy.loginfo("> Publisher servos_absolute corrrectly initialized")
        
        #Setting up GPIO
        #GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(4, GPIO.IN)
        GPIO.setup(17, GPIO.IN)
        GPIO.setup(27, GPIO.IN)
        GPIO.setup(16, GPIO.IN)
        GPIO.setup(20, GPIO.IN)
        GPIO.setup(9, GPIO.IN)
        GPIO.setup(11, GPIO.IN)
        GPIO.setup(0, GPIO.IN)
        GPIO.setup(5, GPIO.IN)
        GPIO.setup(6, GPIO.IN)
        GPIO.setup(13, GPIO.IN)
        GPIO.setup(19, GPIO.IN)
        GPIO.setup(12, GPIO.IN)
        GPIO.setup(18, GPIO.IN)
        GPIO.setup(23, GPIO.IN)
        GPIO.setup(24, GPIO.IN)
        rospy.loginfo("> GPIO corrrectly initialized")
        
        #Create variable for FollowLine message 
        self.msg_follow_line = FollowLine()
        
        
        #Create variable for store magnet sensor data
        self.value_out = 0
        self.left_speed = 0
        self.right_speed = 0
        self.ratio = 0
        
        self.ms = []
        for i in range(16):
            self.ms.append(0)
            
        self.pid = PID(3, 1, 0.5, setpoint=0)
        self.pid.output_limits = (-256, 256)
        
        rospy.loginfo("Initialization complete")
    
    def Scansensor(self):
        #Get input from sensor
        self.ms[0] = GPIO.input(4)
        self.ms[1] = GPIO.input(17)
        self.ms[2] = GPIO.input(27)
        self.ms[3] = GPIO.input(16)
        self.ms[4] = GPIO.input(20)
        self.ms[5] = GPIO.input(9)
        self.ms[6] = GPIO.input(11)
        self.ms[7] = GPIO.input(0)
        self.ms[8] = GPIO.input(5)
        self.ms[9] = GPIO.input(6)
        self.ms[10] = GPIO.input(13)
        self.ms[11] = GPIO.input(19)
        self.ms[12] = GPIO.input(12)
        self.ms[13] = GPIO.input(18)
        self.ms[14] = GPIO.input(23)
        self.ms[15] = GPIO.input(24)
	print(self.ms) 
	
	'''
        #convert value to -256 to 256
        self.msCounted = 0
        self.accumulator = 0
        self.x = 15

        for i in self.ms :
            if i == 0 :
                self.msCounted +=1
                if self.x >= 8 :
                      self.accumulator += (-32 * (self.x - 7))
                elif self.x < 8 :
                    self.accumulator += (-32 * (self.x - 8))
            self.x -= 1
        
        #print(msCounted)
        
        #----------------------PID-----------------------
        if self.msCounted == 0 :
            print("---Magnet not detected---")
            self.value_out = 9999
        else :
            self.value_out = self.pid(self.accumulator/self.msCounted)
            p, i, d = self.pid.components
            print("%f\t%f\t%f" %(p,i,d))
            #rospy.loginfo("%d \t %d" %(self.accumulator, self.msCounted))
            #rospy.loginfo(self.value_out)
            
        if self.value_out == 9999 :
            self.left_speed = 0
            self.right_speed = 0
                    
        elif self.value_out == 0 :
            self.left_speed = 1.0
            self.right_speed = 1.0
                
        elif self.value_out < 0 :
            
            if self.value_out <= -256 :
                self.ratio = 1
            else :
            
            self.ratio = abs(self.value_out)/256
            self.right_speed = 1
            self.left_speed = 1.0 - self.ratio
                    
        elif self.value_out > 0 :
            
            if self.value_out >= 256 :
                self.ratio = 1
            else :
            
            self.ratio = self.value_out/256
            self.left_speed = 1.0
            self.right_speed = 1.0 - self.ratio
                
        self.msg_follow_line.left = self.left_speed
        self.msg_follow_line.right = self.right_speed
        #self.ros_pub_follow_line.publish(self.msg_follow_line)
        #print(self.ms)
        print("Value out = %2.1f \t L = %2.1f \t R = %2.1f" %(self.value_out, self.left_speed, self.right_speed))
        '''
        
    def run(self):
        rate = rospy.Rate(10)
        self.Scansensor()
        while not rospy.is_shutdown():
            self.Scansensor()
            rate.sleep()

if __name__ == "__main__" :
    magnet_sensor = MagnetSensor()
    magnet_sensor.run()
