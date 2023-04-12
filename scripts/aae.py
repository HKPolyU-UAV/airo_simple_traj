#! /usr/bin/env python3

import rospy
# from mainserver import Mainserver
import traj_utils.mainserver as ms

if __name__ == '__main__':
    
    rospy.init_node("AAE_TRAJ_PY")
    rospy.loginfo("SCRIPT FOR AAE TRAJ!")

    ts = ms.mainserver()
    
    # point = Point(21, 42)
        
    rospy.spin()