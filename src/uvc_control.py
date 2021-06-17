#!/usr/bin/python

import rospy
import RPi.GPIO as GPIO
from spero.msg import Operator
from std_msgs.msg import Bool, Int8
import time 

class UVCControl():
    def __init__(self):
        #Setting up ROS
        rospy.init_node("uvc_control")
        rospy.loginfo("Seting Up the node...")
        
        #Setting up GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(21, GPIO.OUT)
        GPIO.output(21, 0)
        rospy.loginfo("> GPIO corrrectly initialized")
        
        self.robot_status = "STOP"
        self.robot_mode = "MANUAL"
        self.robot_uvc = "OFF"
        
        #--- Create the Subscriber to motion_status topic
        self.ros_sub_motion_status   = rospy.Subscriber("/motion_status", Bool, self.set_uvc_from_motion_sensor)
        rospy.loginfo("> Subscriber motion_status corrrectly initialized")
        
        #--- Create the Subscriber to Operator commands
        self.ros_sub_operator          = rospy.Subscriber("/operator_status", Operator, self.set_uvc_from_operator)
        rospy.loginfo("> Subscriber operator_status corrrectly initialized")
        
        '''
        #--- Create the Subscriber to distance_status topic
        self.ros_sub_distance_status = rospy.Subscriber("/distance_status", Int8, self.set_uvc_from_distance_sensor)
        rospy.loginfo("> Subscriber distance_status corrrectly initialized")
        '''
        
        rospy.loginfo("Initialization complete")
    
    def set_uvc_from_operator(self, message):
        #rospy.loginfo("----Opperator callback---")
        self.robot_status = message.status
        self.robot_mode = message.mode
        self.robot_uvc = message.uvc
        
        rospy.loginfo("UV-C > OPERATOR MESSAGE \t %s %s %s" %(self.robot_status, self.robot_mode, self.robot_uvc))
        if self.robot_uvc == "ON" :
            GPIO.output(21, 1)
        else :
            GPIO.output(21, 0)
            
        if self.robot_status == "STOP":
            GPIO.output(21, 0)
            
    
    def set_uvc_from_motion_sensor(self, message):
        #rospy.loginfo("----Motion sensor callback----")
        if self.robot_status == "GO" and self.robot_mode == "AUTO" and self.robot_uvc == "ON" :

            if message.data == True :
                rospy.loginfo("UV-C > UV OFF (MOTION SENSOR)")
                GPIO.output(21, 0)
            else :
                rospy.loginfo("UV-C > ON")
                GPIO.output(21, 1)
                    
        else :
            pass
    
    '''     
    def set_uvc_from_distance_sensor(self, message):
        rospy.loginfo("----distance sensor callback----")
        rospy.loginfo(message.data)
    '''
    
    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__" :
    uvc_control = UVCControl()
    uvc_control.run()
