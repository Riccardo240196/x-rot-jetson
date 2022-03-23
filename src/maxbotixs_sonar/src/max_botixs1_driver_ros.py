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
        self.ser = Serial("/dev/ttyUSB0", 9600, 8, 'N', 1, timeout=1)

        # publisher
        self.pub_sonar_range = rospy.Publisher('max_botix1_range', Range, queue_size=10)
        print('max botix sonar driver initialized')

    def measure(self):
        
        timeOut = 1
        timeStart = time()
        endTime = timeStart + timeOut

        # eat any existing data in the serial pipe
        while time() < endTime:
            bytesToRead = self.ser.inWaiting()
            if bytesToRead > 0:
                self.ser.read(bytesToRead)
            else:
                break
        # now try to get a measurement
        result = self.noValue
        guaranteedRead = 8
        fullReadSize = 6
        digitsReadSize = 4
        while time() < endTime:
            bytesToRead = self.ser.inWaiting()
            # print('after inwaiting ', rospy.Time.now().to_sec())
            if bytesToRead >= guaranteedRead:
                readBytes = self.ser.read(bytesToRead)
                # print('after read bytes ', rospy.Time.now().to_sec())
                rIndex = readBytes.find (b'R')
                fullReadCount = int ((len (readBytes) - rIndex) / fullReadSize) - 1
                readIndex = rIndex + (fullReadCount * fullReadSize) + 1
                readEnd = readIndex + digitsReadSize
                while readBytes[readIndex] == b'0'[0]:
                    readIndex += 1
                # print('after reindex ', rospy.Time.now().to_sec())
                readSlice = readBytes[readIndex:readEnd]
                result = int (readSlice)
                break
        # print('max botix sonar driver measure done')

        self.sonar_range = result


    def pub_topic(self):
        
        ros_msg = Range()
        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = 'sonar1' 
        ros_msg.range = self.sonar_range
        self.pub_sonar_range.publish(ros_msg)

        # print('max botix sonar driver topic published')

    def run(self):
        rospy.init_node('max_botix_driver', anonymous=True)

        rateHz = 10  # >= 10 Hz
        rate = rospy.Rate(rateHz)

        while not rospy.is_shutdown():
            self.measure()
            print('max botix 1: ', self.sonar_range)
            self.pub_topic()            
            rate.sleep()

        self.ser.close()

if __name__ == '__main__':
    try:
        sonar_handle = sonar()
        sonar_handle.run()
    except rospy.ROSInterruptException:
        pass