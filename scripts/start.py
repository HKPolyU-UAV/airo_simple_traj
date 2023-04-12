#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool

global start_pub

def pub_timer_func():
    pub_obj = Bool()
    pub_obj.data = True
    
    start_pub.publish(pub_obj)

if __name__ == '__main__':
    rospy.init_node("START")
    rospy.loginfo("START BY SENDING MSG")
    
    start_pub = rospy.Publisher("/starto", Bool, queue_size=10)
    pub_timer = rospy.Timer(rospy.Duration(0.1), pub_timer_func)
    
    rospy.spin()