#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State
import math
import Sensor
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

detect_data = LaserScan()
height_data = LaserScan()
class drive():
    # Init
    def __init__(self, lock):
        self._lock = lock
        self._curr_pose = PoseStamped()
        self.sense = Sensor.sensor()

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._curr_pose_callback)
        self._pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self._pose_msg = PoseStamped()
        self._pose_state = "posctr"
        self.set_state("posctr")
        ps = [0.0,0.0,2.0]
        self.set_msg(ps)

    # Current position callback
    def _curr_pose_callback(self, pos):
        self._curr_pose = pos

    # State
    def set_state(self, arg):
        self._lock.acquire()
        if arg == "posctr":
            self.state = self._pose_state
            self.msg = self._pose_msg
            self.pub = self._pose_pub
        self._lock.release()

    # Set msg (ish)?
    def set_msg(self, arg):
        if  self.state == "posctr" and len(arg) == 3:
            self._lock.acquire()
            self._pose_msg.pose.position.x = arg[0]
            self._pose_msg.pose.position.y = arg[1]
            self._pose_msg.pose.position.z = arg[2]
            self._lock.release()

    # Set orientation/heading
    def set_orient(self, arg):

        curr_x = self._curr_pose.pose.position.x
        curr_y = self._curr_pose.pose.position.y

        azimuth = math.atan2(arg[1] - curr_y, arg[0] - curr_x)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, azimuth)

        self._lock.acquire()
        self._pose_msg.pose.orientation.z = quaternion[2]
        self._pose_msg.pose.orientation.w = quaternion[3]
        self._lock.release()

        self.orientation = False
        while not self.orientation:

            self.z = self._curr_pose.pose.orientation.z
            self.w = self._curr_pose.pose.orientation.w

            self.qz = quaternion[2]
            self.qw =quaternion[3]

            if (abs(self.z) < (abs(self.qz)-0.1)) or (abs(self.w) < (abs(self.qw)-0.1)):
                self._lock.acquire()
                self._pose_msg.pose.orientation.z = quaternion[2]
                self._pose_msg.pose.orientation.w = quaternion[3]
                self._lock.release()

            else:
                print "Orientation set"
                self.orientation=True

    # Hold altitude function
    def holdalt(self, des_z):
        temp_height = des_z
        height_data = self.sense.get_height()
        if height_data:
            if (height_data.ranges[1]>0.0) and (height_data.ranges[1]<40.0):
                des_z = des_z + ((des_z - height_data.ranges[1]))
        return des_z

    # Sensor detection function #1
    def detect_th(self, des_x, des_y): #Return 0 if ostacle is detected
        detect_data = self.sense.get_detection()
        if detect_data:
            #2=left
            #1=front
            #0=right
            if (detect_data.ranges[0] <= 7) and (detect_data.ranges[0] >= 0): #RIGHT RAY
                des_y = des_y + abs(detect_data.ranges[0] - 7)
                if detect_data.header.seq%10==0:
                    print detect_data.ranges[0]
            if (detect_data.ranges[1] <= 7) and (detect_data.ranges[1] >= 0): #FRONT RAY
                des_x = des_x + (detect_data.ranges[1] - 7)
                if detect_data.header.seq % 10==0:
                    print detect_data.ranges[1]
            if (detect_data.ranges[2] <= 7) and (detect_data.ranges[2] >= 0): #LEFT RAY
                des_y = des_y - abs(detect_data.ranges[0] - 7)
                if detect_data.header.seq % 10==0:
                    print detect_data.ranges[2]
        return des_x, des_y

    # Sensor detection function #2
    def detect_th1(self, des_x, des_y, start):  # Return 0 if ostacle is detected
        detect_data = self.sense.get_detection()
        if detect_data:
            # 2=left
            # 1=front
            # 0=right
            if (detect_data.ranges[1] <= 4) and (detect_data.ranges[1] >= 0):  # FRONT RAY
                Sensrange = detect_data.ranges[1]

                des_x = des_x - self.pose_controlFront(des_x, Sensrange, start.pose.position.x)
                des_y = des_y - self.pose_controlFront(des_y, Sensrange, start.pose.position.y)

             # 0=right
            if (detect_data.ranges[0] <= 4) and (detect_data.ranges[0] >= 0):  # FRONT RAY
                Sensrange = detect_data.ranges[0]

                des_x = des_x - self.pose_controlFront(des_x, Sensrange, start.pose.position.x)
                des_y = des_y - self.pose_controlFront(des_y, Sensrange, start.pose.position.y)

            if (detect_data.ranges[2] <= 4) and (detect_data.ranges[2] >= 0):  # FRONT RAY
                Sensrange = detect_data.ranges[2]

                des_x = des_x - self.pose_controlFront(des_x, Sensrange, start.pose.position.x)
                des_y = des_y - self.pose_controlFront(des_y, Sensrange, start.pose.position.y)


        #print des_x
        return des_x, des_y

    # Something wierd to decde negative/positive orientation
    def pose_controlFront(self, des, range, start):

        if des < start:
            return -(4 - range)

        return (4 - range)

    # Calculate distance to waypoint
    def dist2wp(self, goal):
        cx = float(self._curr_pose.pose.position.x)
        cy = float(self._curr_pose.pose.position.y)
        cz = float(self._curr_pose.pose.position.z)
        dx = float(goal.pose.position.x)
        dy = float(goal.pose.position.y)
        dz = float(goal.pose.position.z)
        distance = math.sqrt(math.pow(cx - dx, 2) + math.pow(cy - dy, 2) + math.pow(cz - dz, 2))
        return distance






















                
            
       
                
        
            
