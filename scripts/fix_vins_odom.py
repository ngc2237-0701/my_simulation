#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    # Fix the child_frame_id to be "body" instead of the hardcoded "world"
    msg.child_frame_id = "body"
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('fix_vins_odom')
    
    # Publish the corrected odometry
    pub = rospy.Publisher('/vins_estimator/odometry_corrected', Odometry, queue_size=10)

    # Subscribe to the original VINS odometry
    rospy.Subscriber('/vins_estimator/odometry', Odometry, callback)
    
    rospy.loginfo("Started VINS Odometry Fixer Node: /vins_estimator/odometry -> /vins_estimator/odometry_corrected (child_frame_id='body')")
    
    rospy.spin()
