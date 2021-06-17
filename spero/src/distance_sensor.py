#!/usr/bin/python

import rospy
import paho.mqtt.client as paho
from std_msgs.msg import Int8
from spero.msg import Operator
import time

broker = '192.168.100.35'
port = 1883
topic = "distance/#"

robot_status = "GO"
robot_mode = "AUTO"

distance = [0,0,0,0,0,0,0,0]

def on_connect(client, userdata, flag, rc):
    if rc == 0:
        rospy.loginfo("  Connected to mqtt broker = " + broker)
        client.connected_flag = True
        rospy.loginfo("> Subscribe to mqtt topic = " + topic)
        rospy.loginfo("Initialization complete")
    else:
        rospy.loginfo("Connection failed return code=" + rc)

### Distance sensor 1
def on_message_1(client, obj, msg):
    global distance
    distance[0] = int(msg.payload)

### Distance sensor 2
def on_message_2(client, obj, msg):
    global distance
    distance[1] = int(msg.payload)

### Distance sensor 3
def on_message_3(client, obj, msg):
    global distance
    distance[2] = int(msg.payload)

### Distance sensor 4
def on_message_4(client, obj, msg):
    global distance
    distance[3] = int(msg.payload)

### Distance sensor 5
def on_message_5(client, obj, msg):
    global distance
    distance[4] = int(msg.payload)

### Distance sensor 6
def on_message_6(client, obj, msg):
    global distance
    distance[5] = int(msg.payload)

### Distance sensor 7
def on_message_7(client, obj, msg):
    global distance
    distance[6] = int(msg.payload)

### Distance sensor 8
def on_message_8(client, obj, msg):
    global distance
    distance[7] = int(msg.payload)
    
### topic message
def on_message(mosq, obj, msg):
    print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))

def callback_operator_status(message):
    global robot_status, robot_mode
    robot_status = message.status
    robot_mode = message.mode

def run():
    global distance, robot_status, robot_mode
    #---------------------------------------SET UP---------------------------------------
    #----------Setting up ROS
    rospy.init_node("distance_sensor")
    rospy.loginfo("Setting up ROS Node and MQTT Client")
        
    ros_pub_distance = rospy.Publisher("/distance_status", Int8, queue_size=1)
    rospy.loginfo("> Publisher distance_status corrrectly initialized")
    
    ros_sub_operator = rospy.Subscriber("/operator_status", Operator, callback_operator_status)
    rospy.loginfo("> Subscriber opearotr_status corrrectly initialized")
    
    #Setting up MQTT
    rospy.loginfo("> Connecting to mqtt broker = "+ broker)
    client = paho.Client("Distance_status")
    
    #Add message callbacks that will only trigger on a specific   subscription    match
    client.message_callback_add('distance/1', on_message_1)
    client.message_callback_add('distance/2', on_message_2)
    client.message_callback_add('distance/3', on_message_3)
    client.message_callback_add('distance/4', on_message_4)
    client.message_callback_add('distance/5', on_message_5)
    client.message_callback_add('distance/6', on_message_6)
    client.message_callback_add('distance/7', on_message_7)
    client.message_callback_add('distance/8', on_message_8)
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    client.connected_flag = False
    client.connect(broker, port, 60)
    client.loop_start()
     
    while not client.connected_flag :
        rospy.loginfo("  Connecting to mqtt broker = "+ broker)
        time.sleep(1)
    
    #------------------------------------Main program------------------------------------
    client.subscribe("distance/1")
    client.subscribe("distance/2")
    client.subscribe("distance/3")
    client.subscribe("distance/4")
    client.subscribe("distance/5")
    client.subscribe("distance/6")
    client.subscribe("distance/7")
    client.subscribe("distance/8")
    
    rate = rospy.Rate(10)
    last_state = 3
    
    while not rospy.is_shutdown():
        if robot_status == "STOP" or robot_mode == "MANUAL" :
            print("Waiting for mode = AUTO")
        else :
            print("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t" %(distance[0],distance[1],distance[2],distance[3],distance[4],distance[5],distance[6],distance[7]))
            
            x = 1
            which_sensor = 0
            smallest = 8000
            
            for i in distance :
                if smallest > i :
                    smallest = i
                    which_sensor = x
                x += 1
                 
            if smallest <= 150 :
                distance_status = 0
                
            elif smallest > 150  and smallest < 250 :
                distance_status = 1
                
            else :
                distance_status = 2
            
            if not last_state == distance_status :
                ros_pub_distance.publish(distance_status)
                last_state = distance_status
                rospy.loginfo("Distance status = %d because sensor %d" %(distance_status, which_sensor))
        rate.sleep()
        

if __name__ == "__main__" :
    run()
