#! /usr/bin/env python

# mainserver.py

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool
import numpy as np
import math
import tf
from tf.transformations import quaternion_matrix
        
class mainserver():
        
    def __init__(self):
        
        rospy.loginfo("MAINSERVER INSTANTIATED!!!")
        
        # subscribers for UAV
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_callback)
        self.uav_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = self.pose_callback)

        # subscriber for receive starting command from another launch file
        self.start_buttom_sub = rospy.Subscriber("/starto", Bool, callback = self.start_buttom_callback)
        
        # publisher for commanding the UAV, here publish setpoint to onboard controller
        # where the outer loop control algorithm of default px4 is adopted
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # services for setting up the UAV                                    
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)        
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        # Timer, which would be called at ros::Spin();        
        self.command_timer = rospy.Timer(rospy.Duration(0.02), self.command_callback)
        
        # getparam
        self.yaw_offset = 0
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        
        if rospy.has_param('~yaw_offset'):

            self.yaw_offset = rospy.get_param("~yaw_offset")
            self.yaw_offset = self.yaw_offset / 360 * 2 * math.pi
            self.x_offset   = rospy.get_param("~x_offset")
            self.y_offset   = rospy.get_param("~y_offset")
            self.z_offset   = rospy.get_param("~z_offset")                    
                    
        
        # private objects for mainserver
        self.fsm = "IDLE"        
        
        self.current_state = State()
        self.uav_pose = PoseStamped()
        self.start_or_not = Bool()

        self.take_off_hoverpt = [0,0,0, 2 * math.pi * 270 / 360]
        self.set_point = PoseStamped()                        
        
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True        
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        
        self.last_request = rospy.Time.now()
        
        self.print_or_not = True
        
        ############
        self.wp = np.array([
            [0.0, 0.0, 0.0, 2 * math.pi * 270 / 360],
            [0.5, 0.0, 2.0, 2 * math.pi * 270 / 360],
            [1.0, 0.0, 0.0, 2 * math.pi * 270 / 360],                    
            [1.5, 0.0, 2.0, 2 * math.pi * 270 / 360],
            [2.0, 0.0, 0.0, 2 * math.pi * 270 / 360],
            [3.0, 0.0, 0.0, 2 * math.pi * 270 / 360],
            [2.0, 0.0, 0.0, 2 * math.pi * 270 / 360],
            [2.0, 0.0, 2.0, 2 * math.pi * 270 / 360],
            [3.0, 0.0, 2.0, 2 * math.pi * 270 / 360],
            [2.0, 0.0, 2.0, 2 * math.pi * 270 / 360],
            [2.0, 0.0, 1.0, 2 * math.pi * 270 / 360],
            [3.0, 0.0, 1.0, 2 * math.pi * 270 / 360],
            [0.0, 0.0, 1.0, 2 * math.pi * 270 / 360]
        ])                
        
        self.sizeof_wp = len(self.wp)
        self.wp_indi = 0        
        self.starting_pt = self.wp[self.wp_indi]
        
        self.target_pt = [0,0,0,0]
        
        # for i in range(self.sizeof_wp):
        #     self.wp[i][2] = self.wp[i][2] + 0.5
        
        print(self.sizeof_wp)
        print(self.wp[0][3])
        print(math.pi)

        ############
        
    
    def state_callback(self, msg):
        self.current_state = msg
        return
        
    def pose_callback(self, msg):
        self.uav_pose = msg
        return
        
    def start_buttom_callback(self, msg):
        self.start_or_not = msg        
        return
        
    def command_callback(self, event=None):
        if self.fsm == "IDLE":
            
            if(self.print_or_not):
                rospy.loginfo("IDLE")
                self.print_or_not = False
            
            if self.get_ready() == True:
                
                
                self.take_off_hoverpt[0] = self.uav_pose.pose.position.x
                self.take_off_hoverpt[1] = self.uav_pose.pose.position.y
                self.take_off_hoverpt[2] = 1.0    
                                                            
                print("\nTAKEOFF TO")
                print(self.take_off_hoverpt[0])                
                print(self.take_off_hoverpt[1])
                print(self.take_off_hoverpt[2])
                print("\n")
                
                self.fsm = "ARMED"
                self.last_request = rospy.Time.now()
                self.print_or_not = True
                self.target_pt = self.take_off_hoverpt
                                            
        elif self.fsm == "ARMED":    
            
            if(self.print_or_not):
                rospy.loginfo("ARMED")
                self.print_or_not = False        
            
            
            if self.arrived_or_not() == True:
                self.fsm = "GOSTART"
                self.last_request = rospy.Time.now()
                self.print_or_not = True
                self.target_pt = self.starting_pt
                            
        elif self.fsm == "GOSTART":
            
            if(self.print_or_not):
                rospy.loginfo("GOSTART")
                self.print_or_not = False   
            
            
            if self.arrived_or_not() == True:# and self.start_or_not.data == True:
                self.fsm = "EXECUTE"
                self.last_request = rospy.Time.now()
                self.print_or_not = True                
                self.wp_indi = self.wp_indi + 1
                self.target_pt = self.wp[self.wp_indi]
                
        
        elif self.fsm == "EXECUTE":
            
            if(self.print_or_not):
                rospy.loginfo("EXECUTE")
                rospy.loginfo("%i th WAYPOINT...", self.wp_indi)
                self.print_or_not = False
                
                
            if(self.arrived_or_not() == True):
                self.wp_indi = self.wp_indi + 1
                print(self.wp_indi)
                print(self.sizeof_wp)
                print(self.wp_indi >= self.sizeof_wp)
                if self.wp_indi >= self.sizeof_wp:                                        
                    self.fsm = "TERMINATE"                
                    self.last_request = rospy.Time.now()
                    self.print_or_not = True
                else:
                    self.last_request = rospy.Time.now()
                    self.print_or_not = True                    
                    self.target_pt = self.wp[self.wp_indi]
        
        elif self.fsm == "TERMINATE":
            
            if(self.print_or_not):
                rospy.loginfo("TERMINATE")
                self.print_or_not = True
                rospy.signal_shutdown("MISSION COMPLETE!")                                                        
        
        else:
            pass
        
        
        self.local_pos_pub.publish(self.set_point)
        
        return
    
    def get_ready(self) -> bool:
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
        
                
    def arrived_or_not(self) -> bool:
        
        self.np2posestamped(self.target_pt)
        
        desired = np.array([
            self.target_pt[0],
            self.target_pt[1],
            self.target_pt[2]
        ])
        
        current = np.array([
            self.uav_pose.pose.position.x,
            self.uav_pose.pose.position.y,
            self.uav_pose.pose.position.z
        ])
        
        delta = np.linalg.norm(current - desired)
        # print(delta)
        
        if(delta < 0.14):
            return True
        else: 
            return False
    
    def np2posestamped(self, pose_np : list) -> None:  
        
        p_temp = [pose_np[0], pose_np[1], pose_np[2]]
        q_temp = self.rpy2q(pose_np[3])
                
        
        
        pos_temp = [0,0,0]
        if(not self.fsm == "ARMED"):
            p_temp = self.se3_transform(p_temp)
            q_temp = so3_transform(q_temp)
            
            self.set_point.pose.position.x = p_temp[0]
            self.set_point.pose.position.y = p_temp[1]
            self.set_point.pose.position.z = p_temp[2]
            
            self.set_point.pose.orientation.w = q_temp[0]
            self.set_point.pose.orientation.x = q_temp[1]
            self.set_point.pose.orientation.y = q_temp[2]
            self.set_point.pose.orientation.z = q_temp[3]
            
        else:
            # without apply SE(3) transformation
            self.set_point.pose.position.x = p_temp[0]
            self.set_point.pose.position.y = p_temp[1]
            self.set_point.pose.position.z = p_temp[2]
            
            self.set_point.pose.orientation.w = q_temp[0]
            self.set_point.pose.orientation.x = q_temp[1]
            self.set_point.pose.orientation.y = q_temp[2]
            self.set_point.pose.orientation.z = q_temp[3]
            
    def so3_transform(self, q) -> list:
        R = tf.transformations.rotation_matrix(self.yaw_offset, (0, 0, 1))
        M = rf.transformations.quaternion_matrix(q)
        
        new_M = np.dot(R, M)
        
        new_q = tf.transformations.quaternion_from_matrix(new_M)
    
        return [new_q[3], new_q[0], new_q[1], new_q[2]]
        
    def se3_transform(self, p) -> list:
        
        t = [self.x_offset, self.y_offset, self.z_offset]
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw_offset)
        R = quaternion_matrix(q)[:3, :3]
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        p_transformed = tf.transformations.concatenate_matrices(T, [p[0], p[1], p[2], 1.0])
                
        return [p_transformed[0], p_transformed[1], p_transformed[2]]
        
        
    def rpy2q(self, yaw) -> list:
        
        roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, yaw
        rpy = np.array([roll_deg, pitch_deg, yaw_deg])
        
        q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        
        return [q[3], q[0], q[1], q[2]]
    
    def q2rpy(self, q : list) -> list:
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        rpy = tf.transformations.euler_from_quaternion([x, y, z, w])

        return [rpy[0], rpy[1], rpy[2]]
                
        
    
    def set_straight():
        return 
    def set_traj(points):
        return
    
    
    