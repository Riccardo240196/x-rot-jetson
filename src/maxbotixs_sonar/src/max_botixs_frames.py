#!/usr/bin/env python
import rospy
from time import time, sleep
from serial import Serial
import numpy as np
from sensor_msgs.msg import Range

torad = 3.1416/180.0
todeg = 1.0/torad
PI = 3.141592


class sonar:
    def __init__(self):
        self.sub_sonar1_range = rospy.Subscriber('max_botix1_range', Range, self.max_botix1_range_callback)
        self.sub_sonar2_range = rospy.Subscriber('max_botix2_range', Range, self.max_botix2_range_callback)

        # publisher
        self.pub_sonar1_range = rospy.Publisher('max_botix1_range_frame', Range, queue_size=10)
        self.pub_sonar2_range = rospy.Publisher('max_botix2_range_frame', Range, queue_size=10)
        print('max botix sonar frames initialized')

    def max_botix1_range_callback(self,msg):
        msg.header.frame_id = 'sonar1'
        msg.field_of_view = 0.2
        msg.min_range = 0 
        msg.max_range = 10
        msg.range = msg.range/1000 
        self.pub_sonar1_range.publish(msg)
    
    def max_botix2_range_callback(self,msg):
        msg.header.frame_id = 'sonar2'
        msg.field_of_view = 0.2
        msg.min_range = 0 
        msg.max_range = 10
        msg.range = msg.range/1000
        self.pub_sonar2_range.publish(msg)

    def run(self):
        rospy.init_node('max_botix_frames', anonymous=True)

        rospy.spin()

if __name__ == '__main__':
    try:
        sonar_handle = sonar()
        sonar_handle.run()
    except rospy.ROSInterruptException:
        pass