#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class microphone():

    def __init__(self):
        rospy.Subscriber("recognizer/output", String, self.callback)
        self.voice_control_pub = rospy.Publisher('check', String, queue_size=1) 
        self.rate = rospy.Rate(10)    

    def callback(self,data):
        rospy.loginfo("voice_control")
        voice_control_str = "line true 1 0.1 0.1"
        self.voice_control_pub.publish(voice_control_str)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('microphone')
        server = microphone()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


