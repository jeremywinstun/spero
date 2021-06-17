#!/usr/bin/python

import rospy
import paho.mqtt.client as paho
from spero.msg import Operator
import time

broker = "192.168.0.196"
port = 1883
topic = "operator_status"

status = "STOP"
mode = "MANUAL"
uvc = "OFF"

def on_connect(client, userdata, flag, rc):
    if rc == 0:
        rospy.loginfo("  Connected to mqtt broker = " + broker)
        client.connected_flag = True
        client.subscribe(topic)
        rospy.loginfo("> Subscribe to mqtt topic = "+ topic)
        rospy.loginfo("Initialization complete")
    else:
        rospy.loginfo("Connection failed return code=" + rc)

def on_message(client, userdata, message):
    global send, status, mode, uvc
    
    recieve = str(message.payload.decode("utf-8"))
    
    #rospy.loginfo("Data from mqtt = "+ recieve)
    if recieve == "GO" :
        status = recieve
    elif recieve == "STOP" :
        status = recieve
    elif recieve == "AUTO" :
        mode = recieve
    elif recieve == "MANUAL" :
        mode = recieve
    elif recieve == "ON" :
        uvc = recieve
    elif recieve == "OFF" :
        uvc = recieve
        
    ros_pub_motion = rospy.Publisher("/operator_status", Operator, queue_size=1)
    
    msg_operator = Operator()
    msg_operator.status = status
    msg_operator.mode = mode
    msg_operator.uvc = uvc
    ros_pub_motion.publish(msg_operator)
    rospy.loginfo("Publishing %s %s %s" %(status, mode, uvc))

def run():
    #---------------------------------------SET UP---------------------------------------
    #Setting up ROS
    global send, status, mode, uvc
    
    rospy.init_node("operator")
    rospy.loginfo("Setting up ROS Node and MQTT Client")
        
    rospy.loginfo("> Subscriber operator_status corrrectly initialized")
        
    #Setting up MQTT
    rospy.loginfo("> Connecting to mqtt broker = "+ broker)
    client = paho.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connected_flag = False

    client.connect(broker, port, 60)
    client.loop_start()
    
    while not client.connected_flag :
        rospy.loginfo("  Connecting to mqtt broker = " + broker)
        time.sleep(1)
    
    #------------------------------------Main program------------------------------------
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        
if __name__ == "__main__" :
    run();


