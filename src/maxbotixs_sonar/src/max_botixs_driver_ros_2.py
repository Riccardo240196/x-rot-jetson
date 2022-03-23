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
        self.noValue = 0
        self.minValue = 300
        self.sonar_range = 0

        # self.device = rospy.get_param("maxbotix_sonar2/usb_dev")
        
        # publisher
        self.pub_sonar_range = rospy.Publisher('max_botix_range', Range, queue_size=10)
        print('max botix sonar driver initialized')

    def measure(self):
        timeStart = time()
        valueCount = 0
        mm = -1
        maxwait=3
        self.ser = Serial("/dev/ttyUSB0", 9600, 8, 'N', 1, timeout=1)
        while time() < timeStart + maxwait:
            if self.ser.inWaiting():
                bytesToRead = self.ser.inWaiting()
                valueCount += 1
                if valueCount < 2: # 1st reading may be partial number; throw it out
                    continue
                testData = self.ser.read(bytesToRead)
                if not testData.startswith(b'R'):
                    # data received did not start with R
                    continue
                try:
                    sensorData = testData.decode('utf-8').lstrip('R')
                except UnicodeDecodeError:
                    # data received could not be decoded properly
                    continue
                try:
                    mm = int(sensorData)
                except ValueError:
                    # value is not a number
                    continue

        self.ser.close()
        self.sonar_range = mm


    def pub_topic(self):
        
        ros_msg = Range()
        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.range = self.sonar_range
        self.pub_sonar_range.publish(ros_msg)

        # print('max botix sonar driver topic published')

    def run(self):
        rospy.init_node('max_botix_driver', anonymous=True)

        rateHz = 10  # >= 10 Hz
        rate = rospy.Rate(rateHz)

        while not rospy.is_shutdown():
            self.measure()
            self.pub_topic()            
            rate.sleep()

if __name__ == '__main__':
    try:
        sonar_handle = sonar()
        sonar_handle.run()
    except rospy.ROSInterruptException:
        pass