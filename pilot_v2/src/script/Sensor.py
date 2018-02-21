#!/usr/bin/env python

import rospy
import math
import numpy
import sys
import drive
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class sensor():
    def __init__(self):

        self.current_height = LaserScan()
        self.send_height = LaserScan()
        rospy.Subscriber('/laser_height', LaserScan, self.height_callback)

        self.detection = LaserScan()
        self.send_det = LaserScan()
        rospy.Subscriber('/laser_scan', LaserScan,  self.scan_callback)
        rospy.Rate(20)

    def scan_callback(self,data):
        self.detection = data

    def height_callback(self,data):
        self.current_height = data

    def get_height(self):
        return self.current_height

    def get_detection(self):
        #if self.detection.ranges[0] == float('inf'):
        #    self.send_det.ranges[0] = 40
        #if self.detection.ranges[1] == float('inf'):
        #   self.send_det.ranges[1] = 40
        #if self.detection.ranges[2] == float('inf'):
        #    self.send_det.ranges[2] = 40
        return self.detection
