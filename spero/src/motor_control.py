#!/usr/bin/python

"""
Class for low level control of our robot. It assumes ros-12cpwmboard has been
installed
"""
from __future__ import division
import RPi.GPIO as GPIO
import rospy
import rosservice
from spero.msg import FollowLine, Operator
from i2cpwm_board.msg import Servo, ServoArray
from i2cpwm_board.srv import IntValue 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int8
from simple_pid import PID
import time

class ServoConvert():
    def __init__(self, id=1):
        self.value = 0
        self.max_speed  = 4095
        self.value_out  = 0
        self.id         = id

    def get_value_out(self, value_in):
        #value_in is in [-1, 1] value_out is in [0, 4095]
        self.value      = int(value_in)
        self.value_out  = int(value_in*self.max_speed)
        return self.value_out
    
    def set_max_speed(self, value_in):
        self.max_speed = value_in
        return self.max_speed
    
class MagnetSensor():
    def __init__(self):
        #Setting up GPIO to read 16 digital output from magnet sensor
        GPIO.setwarnings(False)  
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
        
        '''
        #Create variable for FollowLine message 
        self.msg_follow_line = FollowLine()
        '''
        
        #Create variable for store magnet sensor data
        self.value_out = 0
        self.last_value_out = 0
        self.left_speed = 0
        self.right_speed = 0
        self.ratio = 0
        
        self.forward_left   = 0
        self.forward_right  = 0
        self.backward_left  = 0
        self.backward_right = 0
        
        #create variable for PID calculation
        self.Kp=1.5
        self.Ki=0.0
        self.Kd=0.0
        self.windup_guard = 256
        self.last_error = 0.0
        self.last_time = time.time()
        self.sample_time = 0.1
        
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        
        #create array variable to store 16 input data from magnet sensor
        self.ms = []
        for i in range(16):
            self.ms.append(0)
        
    def run(self):
        #Get input from magnet sensor
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
        
        #self.ms = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
        #rospy.loginfo(self.ms)
        
        #convert sensor readings to a value with range -256 to 256
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
        
        #print(self.msCounted)
        
        #if no magnet detected
        if self.msCounted == 0 :
            rospy.loginfo("---Magnet not detected---")
            if self.last_value_out == 0 :
                rospy.loginfo("---Magnet not detected---")
            elif self.last_value_out > 45:
                rospy.loginfo("---GO RIGHT---")
                self.forward_left = 1
                self.backward_right = 0.6
                self.backward_left = 0
                self.forward_right = 0
            
            elif self. last_value_out < -45:
                rospy.loginfo("---GO LEFT---")
                self.forward_left  = 1
                self.backward_right  = 0.6
                self.forward_right   = 0
                self.backward_left = 0
            
            else: 
                self.value_out = 9999
                self.forward_left   = 0.0
                self.forward_right  = 0.0
                self.backward_left  = 0.0
                self.backward_right = 0.0
	
        #if magnet detected > calculate PID
        else :
            self.error = self.accumulator/self.msCounted
            
            self.current_time = time.time()
            self.delta_time = self.current_time - self.last_time
            self.delta_error = self.error - self.last_error

            if (self.delta_time >= self.sample_time):
                self.PTerm = self.Kp * self.error
                self.ITerm += self.Ki * self.error * self.delta_time

                if (self.ITerm < -self.windup_guard):
                    self.ITerm = -self.windup_guard
                elif (self.ITerm > self.windup_guard):
                    self.ITerm = self.windup_guard

                self.DTerm = 0.0
                if self.delta_time > 0:
                    self.DTerm = self.Kd * self.delta_error / self.delta_time

                # Remember last time and last error for next calculation
                self.last_time = self.current_time
                self.last_error = self.error

                self.value_out = self.PTerm + self.ITerm + self.DTerm
                print("P=%f\tI=%f\tD=%f\tV OUT=%f" %(self.PTerm, self.ITerm, self. DTerm, self.value_out))
            
            #value_out = map(value_in, -256, 256, -1, 1)
            if self.value_out == 0 :
                self.forward_left   = 1.0
                self.forward_right  = 1.0
                self.backward_left  = 0
                self.backward_right = 0
                        
            elif self.value_out < 0 :
                if self.value_out < -256 :
                    self.forward_right  = 1
                    self.backward_left  = 0.5
                    self.forward_left   = 0
                    self.backward_right = 0
                
                elif self.value_out < -170 :
                    self.ratio = abs((self.value_out + 170)/172)
                    self.forward_right  = 1
                    self.backward_left  = self.ratio
                    self.forward_left   = 0
                    self.backward_right = 0
            
                else :
                    self.ratio = abs(self.value_out)/170
                    self.forward_right  = 1
                    self.forward_left   = 1.0 - self.ratio
                    self.backward_left  = 0
                    self.backward_right = 0
                            
            elif self.value_out > 0 :
                if self.value_out > 256 :
                    self.forward_left = 1
                    self.backward_right = 0.5
                    self.backward_left = 0
                    self.forward_right = 0
                
                elif self.value_out > 170 :
                    self.ratio = abs((self.value_out - 170)/172)
                    self.forward_left = 1
                    self.backward_right = self.ratio
                    self.backward_left = 0
                    self.forward_right = 0
                else:
                    self.ratio = self.value_out/170
                    self.forward_left   = 1.0
                    self.forward_right  = 1.0 - self.ratio
                    self.backward_left  = 0
                    self.backward_right = 0
                    
        '''
        self.msg_follow_line.FLeft  = self.forward_left
        self.msg_follow_line.FRight = self.forward_right
        self.msg_follow_line.BLeft  = self.backward_left
        self.msg_follow_line.BRight = self.backward_right
        self.ros_pub_follow_line.publish(self.msg_follow_line)
        '''
        #print(self.ms)
        #print("Value out = %2.1f\tFL = %2.1f\tFR = %2.1f\tBL = %2.1f\tBR = %2.1f" %(self.value_out, self.forward_left, self.forward_right, self.backward_left, self.backward_right))
        #return("%f2.3,%f2.3" %(self.left_speed, self.right_speed))
        self.last_value_out = self.value_out
        return(self.forward_left, self.forward_right, self.backward_left, self.backward_right)

class MotorControl():
    def __init__(self):
        rospy.init_node("motor_control")
        
        rospy.loginfo("Setting Up the Node...")

        rospy.loginfo("Set PWM freqeuncy to 1600Hz")
        rospy.wait_for_service('set_pwm_frequency')
        pwm_freq = rospy.ServiceProxy('set_pwm_frequency', IntValue)
        set_pwm_freq = pwm_freq(1024)
        
        #Setting up GPIO for motor encoder
        self.A1 = 25
        self.B1 = 7
        self.A2 = 8
        self.B2 = 1
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.A1, GPIO.IN) 
        GPIO.setup(self.B1, GPIO.IN)  
        GPIO.setup(self.A2, GPIO.IN)  
        GPIO.setup(self.B2, GPIO.IN)  
        
        #variable for robot status and mode
        self.robot_status = "GO"
        self.robot_mode = "AUTO"
        self.robot_uvc = "OFF"
        self.motion_status = False
        
        #variable for joy stick axis
        self.last_x_axis = 0
        self.last_y_axis = 0
        self.last_theta_axis = 0
        
        #variable for encoder data
        self.counter1 = 0
        self.encState1 = 0
        self.encLastState1 = GPIO.input(self.A1)
        
        self.counter2 = 0
        self.encState2 = 0
        self.encLastState2 = GPIO.input(self.A2)
        
        #dictionary variable that keeps each individual motor speed
        self.actuators = {}
        self.actuators['1forward'] = ServoConvert(id=1)
        self.actuators['2forward'] = ServoConvert(id=4)
        self.actuators['3forward'] = ServoConvert(id=5)
        self.actuators['4forward'] = ServoConvert(id=8)
        self.actuators['1backward'] = ServoConvert(id=2)
        self.actuators['2backward'] = ServoConvert(id=3)
        self.actuators['3backward'] = ServoConvert(id=6)
        self.actuators['4backward'] = ServoConvert(id=7)
        rospy.loginfo("> Actuators corrrectly initialized")
        
        self._servo_msg = ServoArray()
        for i in range(16): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array     = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher servos_absolute corrrectly initialized")
        
        #--- Create the Subscriber to Joy commands
        self.ros_sub_joy             = rospy.Subscriber("/joy", Joy, self.set_actuators_from_joy)
        rospy.loginfo("> Subscriber joy corrrectly initialized")
        
        #--- Create the Subscriber to distance_status topic
        self.ros_sub_distance_status = rospy.Subscriber("/distance_status", Int8, self.set_actuators_from_distance_sensor)
        rospy.loginfo("> Subscriber distance_status corrrectly initialized")
        
        #--- Create the Subscriber to Operator commands
        self.ros_sub_operator        = rospy.Subscriber("/operator_status", Operator, self.set_actuators_from_operator)
        rospy.loginfo("> Subscriber operator_statuus corrrectly initialized")
        
        #--- Create the Subscriber to FollowLine commands
        self.ros_sub_follow_line     = rospy.Subscriber("/follow_line", FollowLine, self.set_actuators_from_magnet_sensor)
        rospy.loginfo("> Subscriber follow_line corrrectly initialized")
        
        '''
        #--- Create the Subscriber to motion_status topic
        self.ros_sub_motion_status   = rospy.Subscriber("/motion_status", Bool, self.set_actuators_from_motion_sensor)
        rospy.loginfo("> Subscriber motion_status corrrectly initialized")

        #--- Create the Subscriber to Twist commands
        self.ros_sub_twist           = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        rospy.loginfo("> Subscriber cmd_vel corrrectly initialized")
        '''
        
        #--- Get the last time e got a commands
        self._last_time_cmd_rcv     = time.time()
        self._timeout_s             = 0.5
        
        self.line_following = MagnetSensor()
        
        self.set_actuators_idle()
        rospy.loginfo("Initialization complete")
    
    def set_actuators_from_operator(self, message):
        self.robot_status = message.status
        self.robot_mode = message.mode
        self.robot_uvc = message.uvc
        
        if self.robot_status == "STOP" :
            self.set_actuators_idle()
        
        if self.robot_mode == "AUTO" or self.robot_mode == "MANUAL":
            self.set_actuators_idle()
            
        rospy.loginfo("MOTOR > OPERATOR MESSAGE \t %s %s %s" %(self.robot_status, self.robot_mode, self.robot_uvc))
    
    #def set_actuators_from_magnet_sensor(self, message):
    def set_actuators_from_magnet_sensor(self):
        """
        Get a message from follow_line, left value for throttle1 & throttle2 right value for throttle3 and throttle4
        
        Buat program buat jalan setiap beberapa detik dan berhenti setiap beberapa detik
        Yang udh jadi selama ini baru ngikutin garis aja
        
        """
        global motion_status
        
        if self.robot_status == "GO" and self.robot_mode == "AUTO":
            #rospy.loginfo("---MAGNET---")
            (x,y,i,j) = self.line_following.run()
            
            #rospy.loginfo("Got Command L = %2.3f \t R = %2.3f" %(x, y))
            #rospy.loginfo("Got a command L = %2.5f R = %2.5f" %(message.left, message.right))
            
            #-- Convert vel into servo values
            self.actuators['1forward'].get_value_out(y)
            self.actuators['2forward'].get_value_out(x)
            self.actuators['3forward'].get_value_out(y)
            self.actuators['4forward'].get_value_out(x)
            self.actuators['1backward'].get_value_out(j)
            self.actuators['2backward'].get_value_out(i)
            self.actuators['3backward'].get_value_out(j)
            self.actuators['4backward'].get_value_out(i)
            
            '''
            #-- Convert vel into servo values
            self.actuators['1forward'].get_value_out(message.left)
            self.actuators['2forward'].get_value_out(message.right)
            self.actuators['3forward'].get_value_out(0)
            self.actuators['4forward'].get_value_out(0)
            self.actuators['1backward'].get_value_out(0)
            self.actuators['2backward'].get_value_out(0)
            self.actuators['3backward'].get_value_out(0)
            self.actuators['4backward'].get_value_out(0)
            '''
            self.send_servo_msg()
             
        else :
            pass
            '''
            elif motion_status == True :
                rospy.loginfo("MOTION DETECTED --- wait for 10 seconds")
                for i in range(10):
                    rospy.loginfo(i+1)
                    time.sleep(1)
            '''
        
    def set_actuators_from_distance_sensor(self, message):
        rospy.loginfo("DISTANCE SENSOR ACTIVITY")
        if self.robot_status == "GO" and self.robot_mode == "AUTO" :
            if message.data == 0 :
                self.actuators['1forward'].set_max_speed(0)
                self.actuators['2forward'].set_max_speed(0)
                self.actuators['3forward'].set_max_speed(0)
                self.actuators['4forward'].set_max_speed(0)
                self.actuators['1backward'].set_max_speed(0)
                self.actuators['2backward'].set_max_speed(0)
                self.actuators['3backward'].set_max_speed(0)
                self.actuators['4backward'].set_max_speed(0)
                
                rospy.loginfo("--> set max speed to 0")
                
            elif message.data == 1 :
                self.actuators['1forward'].set_max_speed(2000)
                self.actuators['2forward'].set_max_speed(2000)
                self.actuators['3forward'].set_max_speed(2000)
                self.actuators['4forward'].set_max_speed(2000)
                self.actuators['1backward'].set_max_speed(2000)
                self.actuators['2backward'].set_max_speed(2000)
                self.actuators['3backward'].set_max_speed(2000)
                self.actuators['4backward'].set_max_speed(2000)
                rospy.loginfo("--> set max speed to 2000")
                
            elif message.data == 2 :
                self.actuators['1forward'].set_max_speed(4095)
                self.actuators['2forward'].set_max_speed(4095)
                self.actuators['3forward'].set_max_speed(4095)
                self.actuators['4forward'].set_max_speed(4095)
                self.actuators['1backward'].set_max_speed(4095)
                self.actuators['2backward'].set_max_speed(4095)
                self.actuators['3backward'].set_max_speed(4095)
                self.actuators['4backward'].set_max_speed(4095)
                rospy.loginfo("--> set max speed to 4095")
                
            self.send_servo_msg()
            
        else:
            pass

    def set_actuators_from_motion_sensor(self, message):
        global motion_status

        if robot.status == "GO" and robot.mode == "AUTO" and robot.uvc == "ON" :
            motion_status = message.data
            rospy.loginfo("PIR SENSOR ACTIVITY")
            rospy.loginfo(message)
            if message.data == True:
                self.actuators['1forward'].set_max_speed(0)
                self.actuators['2forward'].set_max_speed(0)
                self.actuators['3forward'].set_max_speed(0)
                self.actuators['4forward'].set_max_speed(0)
                self.actuators['1backward'].set_max_speed(0)
                self.actuators['2backward'].set_max_speed(0)
                self.actuators['3backward'].set_max_speed(0)
                self.actuators['4backward'].set_max_speed(0)
                rospy.loginfo("--> set max speed to 0")
            
            else:
                self.actuators['1forward'].set_max_speed(4095)
                self.actuators['2forward'].set_max_speed(4095)
                self.actuators['3forward'].set_max_speed(4095)
                self.actuators['4forward'].set_max_speed(4095)
                self.actuators['1backward'].set_max_speed(4095)
                self.actuators['2backward'].set_max_speed(4095)
                self.actuators['3backward'].set_max_speed(4095)
                self.actuators['4backward'].set_max_speed(4095)
                rospy.loginfo("--> PIR = set max speed to 4095")
	
            self.send_servo_msg()
            
        else :
            pass
    
            
    def set_actuators_from_joy(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        if self.robot_status == "GO" and self.robot_mode == "MANUAL":
            rospy.loginfo("---JOY STICK---")
            
            #-- Save the time
            self._last_time_cmd_rcv = time.time()
            
            #-- Save each axis to a variable
            self.x_axis = message.axes[1]
            self.y_axis = message.axes[0]
            self.theta_axis = message.axes[3]
            
            """
            3 possible movement
                - forward + backward (linearx axis)
                - sliding right + sliding left (y axis)
                - rotate right + rotate left 
            
            Wheel position
             __________
            |          |
            |  1    2  |
            |          |
            |  3    4  |
            |__________|
                   
            """
            # 3a. Rotate Right
            if self.x_axis == 0 and self.y_axis == 0 and self.theta_axis < 0 :
                rospy.loginfo("Rotate right")
                self.actuators['1forward'].get_value_out(self.theta_axis)
                self.actuators['2backward'].get_value_out(self.theta_axis)
                self.actuators['3forward'].get_value_out(self.theta_axis)
                self.actuators['4backward'].get_value_out(self.theta_axis)
                self.actuators['1backward'].get_value_out(0)
                self.actuators['2forward'].get_value_out(0)
                self.actuators['3backward'].get_value_out(0)
                self.actuators['4forward'].get_value_out(0)
            
            # 3b. Rotate Right
            elif self.x_axis == 0 and self.y_axis == 0 and self.theta_axis > 0 :
                rospy.loginfo("Rotate left")
                self.actuators['1backward'].get_value_out(self.theta_axis)
                self.actuators['2forward'].get_value_out(self.theta_axis)
                self.actuators['3backward'].get_value_out(self.theta_axis)
                self.actuators['4forward'].get_value_out(self.theta_axis)
                self.actuators['1forward'].get_value_out(0)
                self.actuators['2backward'].get_value_out(0)
                self.actuators['3forward'].get_value_out(0)
                self.actuators['4backward'].get_value_out(0)
            
            # 1a. Forward
            elif abs(self.x_axis) > abs(self.y_axis) and self.x_axis > 0 :
                rospy.loginfo("Forward")
                self.actuators['1forward'].get_value_out(self.x_axis)
                self.actuators['2forward'].get_value_out(self.x_axis)
                self.actuators['3forward'].get_value_out(self.x_axis)
                self.actuators['4forward'].get_value_out(self.x_axis)
                self.actuators['1backward'].get_value_out(0)
                self.actuators['2backward'].get_value_out(0)
                self.actuators['3backward'].get_value_out(0)
                self.actuators['4backward'].get_value_out(0)
                
            # 1a. Backward
            elif abs(self.x_axis) > abs(self.y_axis) and self.x_axis < 0 :
                rospy.loginfo("Backward")
                self.actuators['1backward'].get_value_out(self.x_axis)
                self.actuators['2backward'].get_value_out(self.x_axis)
                self.actuators['3backward'].get_value_out(self.x_axis)
                self.actuators['4backward'].get_value_out(self.x_axis)
                self.actuators['1forward'].get_value_out(0)
                self.actuators['2forward'].get_value_out(0)
                self.actuators['3forward'].get_value_out(0)
                self.actuators['4forward'].get_value_out(0)
                
                '''
            # 2a. Sliding right
            elif abs(self.y_axis) > abs(self.x_axis) and self.y_axis > 0 :
                rospy.loginfo("Sliding right")
                self.actuators['1forward'].get_value_out(self.y_axis)
                self.actuators['2backward'].get_value_out(self.y_axis)
                self.actuators['3backward'].get_value_out(self.y_axis)
                self.actuators['4forward'].get_value_out(self.y_axis)
                self.actuators['1backward'].get_value_out(0)
                self.actuators['2forward'].get_value_out(0)
                self.actuators['3forward'].get_value_out(0)
                self.actuators['4backward'].get_value_out(0)
            
            # 2b. Sliding left
            elif abs(self.y_axis) > abs(self.x_axis) and self.y_axis < 0 :
                rospy.loginfo("Sliding left")
                self.actuators['1backward'].get_value_out(self.y_axis)
                self.actuators['2forward'].get_value_out(self.y_axis)
                self.actuators['3forward'].get_value_out(self.y_axis)
                self.actuators['4backward'].get_value_out(self.y_axis)
                self.actuators['1forward'].get_value_out(0)
                self.actuators['2backward'].get_value_out(0)
                self.actuators['3backward'].get_value_out(0)
                self.actuators['4forward'].get_value_out(0)
                '''
            
            else :
                self.set_actuators_idle()
            
            #rospy.loginfo("Got a command x = %2.1f  y = %2.1f theta = %2.1f" %(self.x_axis, self.y_axis, self.theta_axis))
            self.last_x_axis = self.x_axis
            self.last_y_axis = self.y_axis
            self.last_theta_axis = self.theta_axis
            
            rospy.loginfo("Send data joy stick")
            self.send_servo_msg()
        
        else :
            pass
    
    @property
    def is_controller_connected(self):
        print time.time() - self._last_time_cmd_rcv
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)
    
    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['1forward'].get_value_out(0)
        self.actuators['2forward'].get_value_out(0)
        self.actuators['3forward'].get_value_out(0)
        self.actuators['4forward'].get_value_out(0)
        self.actuators['1backward'].get_value_out(0)
        self.actuators['2backward'].get_value_out(0)
        self.actuators['3backward'].get_value_out(0)
        self.actuators['4backward'].get_value_out(0)

        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id].value = abs(servo_obj.value_out)
            #rospy.loginfo("Sending to %s command %d"%(actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)
        #rospy.loginfo("Publish to ServoArray")

    @property
    def is_controller_connected(self):
        print time.time() - self._last_time_cmd_rcv
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):
        #--- Set the control rate
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            '''
            if self.robot_mode == "MANUAL" :
                print self._last_time_cmd_rcv, self.is_controller_connected
                if not self.is_controller_connected:
                    self.set_actuators_idle()
                    self.send_servo_msg()
            '''
            
            #program for automated desinfection
            #stop for 1 minute every 10 meter and so on
            if self.robot_status == "GO" and self.robot_mode == "AUTO" :
                A1 = GPIO.input(25)
                B1 = GPIO.input(7)
                A2 = GPIO.input(8)
                B2 = GPIO.inout(1)
                
                self.encState1 = GPIO.input(self.A1)
                self.encState2 = GPIO.input(self.A2)
                
                #calculate encoder
                if not self.encState1 == self.encLastState1 :
                    rospy.loginfo("Motor rorating")
                    if not GPIO.input(self.B1) == self.encState1 :
                        self.counter1 += 1
                    else :
                        self.counter1 -= 1
                    
                    self.encLastState1 = self.encState1
                    
                if not self.encState2 == self.encLastState2 :
                    if not GPIO.input(self.B2) == self.encState2:
                        self.counter2 += 1
                    else :
                        self.counter2 -= 1
                    
                    self.encLastState2 = self.encState2
                
                self.combine_counter = (self.counter1 + self.counter2)/2 
                
                rospy.loginfo("Counter > %2.1f \t %2.1f \t %2.1f" %(self.counter1, self.counter2, self.combine_counter))
                
                if self.combine_counter >= 10000 :
                    self.distance = self.combine_counter/1500
                    rospy.loginfo("Distance = %2.1f" %(self.distance))
                    self.set_actuators_from_magnet_sensor()
                    
                else :
                    self.set_actuators_idle()
                    rospy.loginfo("Stop for 60 Seconds")
                    for i in range(10) :
                        rospy.loginfo(i+1)
                        time.sleep(1)
                        
                    self.combine_counter = 0
                    self.distance = 0
                
            rate.sleep()

if __name__ == "__main__":
    motor_control = MotorControl()
    motor_control.run()
