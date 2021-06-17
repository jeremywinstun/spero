#!/usr/bin/python

import rospy
import paho.mqtt.client as paho
from std_msgs.msg import Bool
import time

broker = '192.168.100.35'
port = 1883
topic = "motion_status"

last_state = False

#Setting up ROS
rospy.init_node("motion_sensor")
rospy.loginfo("Setting up ROS Node and MQTT Client")
        
ros_pub_motion = rospy.Publisher("/motion_status", Bool, queue_size=1)
rospy.loginfo("> Publisher motion_status corrrectly initialized")


def on_connect(client, userdata, flag, rc):
    if rc == 0:
        rospy.loginfo("  Connected to mqtt broker = " + broker)
        client.connected_flag = True
        client.subscribe(topic)
        rospy.loginfo("> Subscribe to mqtt topic = " + topic)
        rospy.loginfo("Initialization complete")
    else:
        rospy.loginfo("Connection failed return code=" + rc)

def on_message(client, userdata, message):
    motion_detected = False
    recieve = str(message.payload.decode("utf-8"))
    #rospy.loginfo("Data from mqtt = "+ recieve)
    pir = [0,0,0,0,0,0]
    pir[0] = recieve[0]
    pir[1] = recieve[1]
    pir[2] = recieve[2]
    pir[3] = recieve[3]
    pir[4] = recieve[4]
    pir[5] = recieve[5]
    rospy.loginfo(pir)
    
    j=0
    
    for i in pir :
        j = int(i)
        #rospy.loginfo("j in PIR = %d" %j)
        if j == 1 :
            motion_detected = True
        else :
            pass
    #rospy.loginfo(motion_detected)
    update(motion_detected)
    
def update(status):
    global last_state
    if not status == last_state:
        if status == True:
            ros_pub_motion.publish(status)
            rospy.loginfo("MOTION DETECTED wait for 5 second")
            for i in range(5): #coba test ini apakah masih delay 10 detik klo ada pesan baru masuk
                rospy.loginfo(i+1)
                time.sleep(1)
            last_state = status
        else :
            ros_pub_motion.publish(status)
            rospy.loginfo("ALL CLEAR")
            last_state = status

def run():
    #Setting up MQTT
    rospy.loginfo("> Connecting to mqtt broker = "+ broker)
    client = paho.Client("PIR_sensor")
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
    run()