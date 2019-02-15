#!/usr/bin/env python

""" odom_transformer.py 2017-08-09

    this is used for publish the tf between odom and base_footprint
    
"""

import rospy
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster


class odomTransformer():
    def __init__(self):
        # naming
        rospy.init_node('Odom_Transformer', anonymous=False)
        
        # broadcaster
        self.tf_broadcaster = TransformBroadcaster()
        
        # subscribe the odom topic
        rospy.Subscriber('odom', Odometry, self.pub_odom_tf)
        
    def pub_odom_tf(self, msg):
        self.tf_broadcaster.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(),
            "base_footprint",
            "odom"
        )
        
if __name__ == '__main__':
    try:
        odomTransformer()
        rospy.spin()
    except:
        rospy.loginfo("OdomTransformer exception.")
