#!/usr/bin/env python
"""
Created on Fri Sep 16 23:28:53 2016

@author: dennis
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest, CommandBoolResponse, CommandTOL, CommandTOLRequest
import time
from tf.transformations import *
import numpy as np


### constant
RATE_STATE = 1 # state rate subscription


### class for mavros subscription ###
class mavros_driver():
    def __init__(self):
   
        ### subscriber ###
        
        # state subscriber 
        self._rate_state = rospy.Rate(RATE_STATE)
        self.current_state = State()
        rospy.Subscriber('/mavros/state', State , self._current_state_callback)

        #  Position subscriber,
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        # pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # wait until connection with FCU 
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.Rate(20)     
        print 'FCU connection successful'

### callback functions ###
    
    def _current_state_callback(self, data):
        self.current_state = data

    def _local_pose_callback(self, data):
        self.local_pose = data

    def get_pose(self):
        return self.local_pose

### service function ###
        
    def set_mode(self, mode):
        if not self.current_state.connected:
            print "No FCU connection"
        
        elif self.current_state.mode == mode:
            print "Already in " + mode + " mode"
        
        else:

            # wait for service
            rospy.wait_for_service("mavros/set_mode")   


            # service client
            set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
            
        
            # set request object
            req = SetModeRequest()
            req.custom_mode = mode

            
            # zero time 
            t0 = rospy.get_time()
            
            
            # check response
            while not rospy.is_shutdown() and (self.current_state.mode != req.custom_mode):
                if rospy.get_time() - t0 > 2.0: # check every 5 seconds
                
                    try:
                        # request 
                        set_mode.call(req)
                        
                    except rospy.ServiceException, e:
                        print "Service did not process request: %s"%str(e)
  
                    t0 = rospy.get_time()
                    
                
            print "Mode: "+self.current_state.mode + " established"

    def arm(self, do_arming):
        
        if self.current_state.armed and do_arming:
            print "already armed"
            
        else:
            # wait for service
            rospy.wait_for_service("mavros/cmd/arming")   
            
            
            # service client
            set_arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
            
            
            # set request object
            req = CommandBoolRequest()
            req.value = do_arming
            
            
             # zero time 
            t0 = rospy.get_time()
            
            
            # check response
            if do_arming:
                while not rospy.is_shutdown() and not self.current_state.armed:
                    if rospy.get_time() - t0 > 2.0: # check every 5 seconds
                    
                        try:
                            # request 
                            set_arm.call(req)
                            
                        except rospy.ServiceException, e:
                            print "Service did not process request: %s"%str(e)
      
                        t0 = rospy.get_time()
                
                print "armed: ", self.current_state.armed
                
            else: 
                while not rospy.is_shutdown() and self.current_state.armed:
                    if rospy.get_time() - t0 > 0.5: # check every 5 seconds
                    
                        try:
                            # request 
                            set_arm.call(req)
                            
                        except rospy.ServiceException, e:
                            print "Service did not process request: %s"%str(e)
      
                        t0 = rospy.get_time()




                    
                
            
    
            
            
