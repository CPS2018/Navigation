#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from sensor_msgs.msg import LaserScan

import threading
import sys
import time 
import signal
import numpy as np
import math

#### FILES IN PACKAGE  ######
import mavros_driver  # File Mavros drive
import drive        # File state

# Destionation pose % relative pose
pose_rel = [0.0,0.0,2.0]
pose_rel2 = [0.0,0.0,0.0]
des_pose = PoseStamped()

# Reset pose
def reset(ar):
    for i,x in enumerate(ar):
        ar[i] = 0.0

# Useless crap
def if_isNum(arg):
    try:
        float(arg)
        return True
    except ValueError:
        return False

# Move thread: needs to sent position > 2 Hz (wth?)
def move_pub(rate, go, run_event):
    
    # frequency of publishing
    rate = rospy.Rate(rate)
    # publish desire pose
    while run_event.is_set() and not rospy.is_shutdown():
       go.pub.publish(go.msg)
       rate.sleep()
        
# Print UI
def usage():
    print "usage: [ p [x] [y] [z] ]"


# Main loop
def run_tests():

    # Init
    # Ros node initalization
    rospy.init_node('position', anonymous=True)

    # Create driver for receiving mavros msg
    driver = mavros_driver.mavros_driver()
    lock = threading.Lock()
    go = drive.drive(lock)

    # Pose publisher rate
    rate = 20

    # Signal flag for running threads
    run_event = threading.Event()
    run_event.set()
    move_t = threading.Thread(target=move_pub, args=(rate, go, run_event))
    move_t.start()

    # SET MODE
    driver.set_mode("OFFBOARD")
    
    # ARM
    driver.arm(True)

    # Wait for drone to reach hover position
    time.sleep(2.0)
    print "Start Altitude = 2 meter"

    # UI loop
    usage()
    while 1:

        # READ USER INPUT
        user_input = sys.stdin.readline()

        # SPLIT INPUT
        args = user_input.split()

        # SET POSITION
        if str(args[0]) == "p":

            # Reser relative pose
            reset(pose_rel)

            # Set new relative position
            if len(args[1:]) > 3:
                print "too many arguments"
                usage()

            elif len(args[1:]) < 3:
                print "not enough arguments"
                usage()

            else:

                # Starting position
                start_pose = driver.get_pose()

                # Set starting pose
                pose_rel[0] = start_pose.pose.position.x
                pose_rel[1] = start_pose.pose.position.y
                pose_rel[2] = start_pose.pose.position.z
                print "Start position set"
                time.sleep(0.5)

                # Set destination pose
                des_pose.pose.position.x = float(args[1])
                des_pose.pose.position.y = float(args[2])
                des_pose.pose.position.z = float(args[3])
                print "Waypoint set"
                time.sleep(0.5)

                # Set distance
                distance = go.dist2wp(des_pose)
                print"Distance set"
                print distance
                time.sleep(0.5)

                # Calculate distance step
                dx = ((float(args[1]) - start_pose.pose.position.x) / (distance * 1000))
                dy = ((float(args[2]) - start_pose.pose.position.y) / (distance * 1000))
                dz = ((float(args[3]) - start_pose.pose.position.z) / (distance * 1000))
                print "Step size set"
                time.sleep(0.5)

                count = 0
                pose_rel2[0] = float(args[1])
                pose_rel2[1] = float(args[2])
                pose_rel2[2] = float(args[3])
                go.set_orient(pose_rel2)

                while distance > 0.5:
                    #[new_x,new_y]=go.detect_th1(pose_rel[0] + dx,pose_rel[1] + dy, start_pose)
                    #new_alt = go.holdalt(des_pose.pose.position.z)
                    pose_rel[0] = pose_rel[0] + dx
                    pose_rel[1] = pose_rel[1] + dy
                    pose_rel[2] = pose_rel[2] + dz
                    go.set_msg(pose_rel)
                    time.sleep(0.001)
                    distance = go.dist2wp(des_pose)
                    print go.dist2wp(des_pose)
                print "Destination reached"
                #reset(pose_rel)

        usage()
        # Dont waste cpu!
        time.sleep(1)

if __name__ == '__main__':
    try:
        
        run_tests()

    except rospy.ROSInterruptException:
        pass
