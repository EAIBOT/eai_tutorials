#!/usr/bin/env python
#import time,os,math,struct
#from math import pi

import rospy,math
from geometry_msgs.msg import Pose,Point32
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

class VitualWallDetector:
    def __init__(self):
        rospy.init_node('costmap_filter', anonymous=True)
        rospy.Subscriber('robot_pose', Pose, self.Robot_Pose_Callback)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.Costmap_Callback)
        self.costmap_filter = rospy.Publisher('costmap_filter', Bool, queue_size=1)
        
        self.r_x = 0.0
        self.r_y = 0.0
        self.r_z = 0.0
        self.r_qx = 0.0
        self.r_qy = 0.0
        self.r_qz = 1.0
        self.r_qw = 1.0

        self.min_x = rospy.get_param("/laser_filter/scan_filter_chain")[0]['params']['min_x']
        self.min_y = rospy.get_param("/laser_filter/scan_filter_chain")[0]['params']['min_y']
        self.min_z = rospy.get_param("/laser_filter/scan_filter_chain")[0]['params']['min_z']
        self.max_x = rospy.get_param("/laser_filter/scan_filter_chain")[0]['params']['max_x']
        self.max_y = rospy.get_param("/laser_filter/scan_filter_chain")[0]['params']['max_y']
        self.max_z = rospy.get_param("/laser_filter/scan_filter_chain")[0]['params']['max_z']
        self.box_frame = rospy.get_param("/laser_filter/scan_filter_chain")[0]['params']['box_frame']

        self.distance = (self.min_x+self.min_y+self.max_x+self.max_y)/4.0
        self.cost_max = 80

    def Robot_Pose_Callback(self,msg):
        self.r_x = msg.position.x
        self.r_y = msg.position.y
        self.r_z = msg.position.z
        self.r_qx = msg.orientation.x
        self.r_qy = msg.orientation.y
        self.r_qz = msg.orientation.z
        self.r_qw = msg.orientation.w

    def Costmap_Callback(self,msg):
        angle = math.atan2(2.0*(self.r_qx*self.r_qy+self.r_qz*self.r_qw), 1.0-2.0*(self.r_qy*self.r_qy+self.r_qz*self.r_qz))
        msg_pub = Bool()
        resolution=msg.info.resolution
        x_min=self.r_x-self.min_y*math.sin(angle)+self.min_x*math.cos(angle)
        y_min=self.r_y+self.min_x*math.sin(angle)+self.min_y*math.cos(angle)
        z_min=self.min_z
        x_max=self.r_x-self.max_y*math.sin(angle)+self.max_x*math.cos(angle)
        y_max=self.r_y+self.max_x*math.sin(angle)+self.max_y*math.cos(angle)
        z_max=self.max_z
        x_center=(x_min+x_max)/2
        y_center=(y_min+y_max)/2
        flag = 0
        print len(msg.data)
        for i in range(0,msg.info.width):
            #if abs(self.x_min)>self.distance or abs(self.max_x)>self.distance:
            #    continue
            for j in range(0,msg.info.height):
            #    if abs(self.y_min)>self.distance or abs(self.max_y)>self.distance:
            #ji        continue
                #r=msg.data[i]-yy
                if (i*i+j*j<self.distance*self.distance):
                    if msg.data[i][j]>self.cost_max:
                        flag = flag+1
        if flag:  # obstacles detected
            msg_pub.data = True
        self.costmap_filter.publish(msg_pub)
        pass

    def Run(self):
        self.Costmap_Callback()
        while not rospy.is_shutdown():
            pass
        #rospy.loginfo("ROS was shutdown")

if __name__ == '__main__':
    VitualWallDetector().Run()
    rospy.spin()
