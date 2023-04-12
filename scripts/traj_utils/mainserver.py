"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#! /usr/bin/env python

# mainserver.py

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool
import numpy
import visualization_msgs.msg


        
class mainserver():
        
    def __init__(self):
        
        rospy.loginfo("INSTANTIATED!!!")
        
        self.start_buttom_sub = rospy.Subscriber("/starto", Bool, callback = self.start_buttom_callback)
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback)
        self.uav_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.pose_callback)
       
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
                        
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        self.command_timer = rospy.Timer(rospy.Duration(0.02), self.command_callback)
        
        self.fsm = "IDLE"        
        self.current_state = State()

        self.take_off_hoverpt = PoseStamped()
        self.set_point = PoseStamped()
        self.uav_pose = PoseStamped()
        
        self.start_or_not = Bool()
        
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True
        
        self.last_request = rospy.Time.now()
        
        
    # def take_off():
    
    def set_straight():
        return 
    def set_traj(points):
        return
    
    
    def start_buttom_callback(self, msg):
        
        return 
    
    def state_callback(self, msg):
        self.current_state = msg
        # self.cr
        
    def pose_callback(self, msg):
        self.uav_pose = msg
        
    def command_callback(self, event=None):
        if self.fsm == "IDLE":
            rospy.loginfo("IDLE")
            
            if self.get_ready() == True:
                self.fsm = "ARMED"
                
                self.take_off_hoverpt.pose.position.x = self.uav_pose.pose.position.x
                self.take_off_hoverpt.pose.position.y = self.uav_pose.pose.position.y
                self.take_off_hoverpt.pose.position.z = 1.0
                
                self.set_point = self.take_off_hoverpt
                
                print("TAKEOFF TO\n")
                print(self.take_off_hoverpt.pose.position.x)
                print("\n")
                print(self.take_off_hoverpt.pose.position.y)
                print("\n")
                print(self.take_off_hoverpt.pose.position.z)
                print("\n")
                                            
        elif self.fsm == "ARMED":            
            rospy.loginfo("ARMED")
            
            if self.taking_off() == True:
                self.fsm = "GOSTART"
                self.last_request = rospy.Time.now()
                
            
        elif self.fsm == "GOSTART":
            rospy.loginfo("GOSTART")
            
            if self.arrived() == True and self.start_or_not.data == True:
                pass
            
        
        else:
            pass
        
        
        self.local_pos_pub.publish(self.set_point)
    
    def get_ready(self):
        return_state = False
        
        
        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_request) > rospy.Duration(2.0)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            self.last_request = rospy.Time.now()
        else:
            if(not self.current_state.armed and (rospy.Time.now() - self.last_request) > rospy.Duration(2.0)):
                if(self.arming_client.call(self.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    return_state = True
            
                self.last_request = rospy.Time.now()
                
        if return_state:
            return True
        else:
            return False
        
    def taking_off(self):
        return False
        pass