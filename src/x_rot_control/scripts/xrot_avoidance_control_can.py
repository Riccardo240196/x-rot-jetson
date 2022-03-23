#!/usr/bin/env python

import os
import rospy
import math 
import numpy as np
import rospkg
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import can
import cantools
import struct
from time import sleep

torad = 3.1416/180.0
todeg = 1.0/torad
PI = 3.141592

# Front 0x03F2
# Rear 0x07DA

class xrot_position_control:
    def __init__(self):
        self.state_msg = LinkState()
        self.state_msg.link_name = 'footprint'
        self.state_msg.reference_frame = 'world'
        self.path_point_distance = 20 # [cm]
        self.Allarm_ON_prev = 0
        
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
        # self.bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
        self.bus.set_filters([{"can_id": 0x104, "can_mask":0xFFFFFFF },{"can_id": 0x304, "can_mask":0xFFFFFFF },{"can_id": 0x690, "can_mask":0xFFFFFFF }]) 
        self.pub_can_odometry = rospy.Publisher('can_odometry', Odometry, queue_size=1)
        self.can_odometry = Odometry()
        self.pub_path_point = rospy.Publisher('path_point', Odometry, queue_size=1)
        self.path_point = Odometry()
        self.pub_speed_request = rospy.Publisher('speed_request', Twist, queue_size=1)
        self.speed_request = Twist()

        # load dbc file
        rospack = rospkg.RosPack()
        path_to_pack = rospack.get_path('xrot_gazebo_sim')
        Laserline_db_file_path = path_to_pack + '/database/Laserline_ultrasonic.dbc'
        XROT_db_file_path = path_to_pack + '/database/X-ROT_CAN1.dbc'
        self.db = cantools.database.load_file(Laserline_db_file_path)
        self.db.add_dbc_file(XROT_db_file_path)

        self.vehicle_x = 10.0
        self.vehicle_y = -20.0
        self.vehicle_ang = 120

        self.Request_Control = 0
        self.Allarm_ON = 0
        self.Closes_Obst_Dist = 0
        self.Closes_Obst_Orient = 0
        self.Speed_Request = 128
        self.Steering_Request = 128
        self.received_path = False

    def cmd_vel_cbk(self,msg):
        self.Allarm_ON = msg.linear.z
        self.Request_Control = self.received_path
        self.Closes_Obst_Dist = 0
        self.Closes_Obst_Orient = 0
        self.Speed_Request = int(128 + msg.linear.x*127)
        self.Steering_Request = int((-msg.angular.z + 0.75)*255/1.5) # 0 full left (positive omega), 255 full rigth (negative omega)
        # print(self.Speed_Request, self.Steering_Request)
        self.send_local_planner()

        if(self.Allarm_ON_prev==0 and self.Allarm_ON==1):
            self.received_path = False
            self.send_path_request()

        self.Allarm_ON_prev = self.Allarm_ON   

        self.speed_request.linear.x = self.Speed_Request
        self.speed_request.angular.z = self.Steering_Request
        self.pub_speed_request.publish(self.speed_request)

    def send_local_planner(self):
        message_def = self.db.get_message_by_name('LocalPlanner')
        data = message_def.encode({ 'Request_Control': self.Request_Control,
                                    'Allarm_ON': self.Allarm_ON,
                                    'Closes_Obst_Dist': self.Closes_Obst_Dist,
                                    'Closes_Obst_Orient': self.Closes_Obst_Orient,
                                    'Speed_Request': self.Speed_Request,
                                    'Steering_Request': self.Steering_Request
                                    })
        # print(message_def.frame_id)
        message = can.Message(arbitration_id=message_def.frame_id,extended_id=False, data=data, timestamp=rospy.get_time())
        
        self.bus.send(message)

    def send_path_request(self):
        message_def = self.db.get_message_by_name('LocalPlannerPathRequest')
        data = message_def.encode({ 'Distance': self.path_point_distance})
        message = can.Message(arbitration_id=message_def.frame_id,extended_id=False, data=data, timestamp=rospy.get_time())
        self.bus.send(message)
        # print(message)

    def set_state(self):
        try:
            service_client = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
            self.state_resp = service_client(self.state_msg)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
    
    def read_pos_from_can(self):
        # self.bus.reset()
        msg = self.bus.recv(0.1)

        try:
            if msg.arbitration_id == 0x304:

                data = list(msg.data)

                # X position                                    
                n0 = data[0]
                n1 = data[1]
                n2 = data[2]

                if(n0 & 0b10000000):
                    num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                    num = 0xffffffff - num + 0b1
                    pos_x = - np.float32(num)/100
                else:
                    num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                    pos_x = np.float32(num)/100

                # Y position                    
                n0 = data[3]
                n1 = data[4]
                n2 = data[5]

                if(n0 & 0b10000000):
                    num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                    num = 0xffffffff - num + 0b1
                    pos_y = -np.float32(num)/100
                else:
                    num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                    pos_y = np.float32(num)/100
                
                # angle
                n0 = data[6]
                n1 = data[7]
                if(n0 & 0b10000000):
                    num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                    num = 0xffffffff - num + 0b1
                    angle = - np.float32(num)/100
                else:
                    num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                    angle = np.float32(num)/100

                quat = quaternion_from_euler(0, 0, angle)
                
                self.state_msg.pose.position.x = pos_x
                self.state_msg.pose.position.y = pos_y

                self.state_msg.pose.orientation.x = quat[0]
                self.state_msg.pose.orientation.y = quat[1]
                self.state_msg.pose.orientation.z = quat[2]
                self.state_msg.pose.orientation.w = quat[3]

                self.can_odometry.pose.pose.position.x = pos_x
                self.can_odometry.pose.pose.position.y = pos_y
                self.can_odometry.pose.pose.orientation.x = quat[0]
                self.can_odometry.pose.pose.orientation.y = quat[1]
                self.can_odometry.pose.pose.orientation.z = quat[2]
                self.can_odometry.pose.pose.orientation.w = quat[3]

                self.pub_can_odometry.publish(self.can_odometry)
            
            if msg.arbitration_id == 0x690:

                data = list(msg.data)
            
                # X position                                    
                n0 = data[0]
                n1 = data[1]
                n2 = data[2]

                if(n0 & 0b10000000):
                    num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                    num = 0xffffffff - num + 0b1
                    pos_x = - np.float32(num)/100
                else:
                    num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                    pos_x = np.float32(num)/100

                # Y position                    
                n0 = data[3]
                n1 = data[4]
                n2 = data[5]

                if(n0 & 0b10000000):
                    num = (0xff << 24) | (n0 << 16) | (n1 << 8) | n2
                    num = 0xffffffff - num + 0b1
                    pos_y = -np.float32(num)/100
                else:
                    num = (0x00 << 24) | (n0 << 16) | (n1 << 8) | n2
                    pos_y = np.float32(num)/100
                
                # angle
                n0 = data[6]
                n1 = data[7]
                if(n0 & 0b10000000):
                    num = (0xff << 24) | (0xff << 16) | (n0 << 8) | n1
                    num = 0xffffffff - num + 0b1
                    angle = - np.float32(num)/100
                else:
                    num = (0x00 << 24) | (0x00 << 16) | (n0 << 8) | n1
                    angle = np.float32(num)/100

                
                quat = quaternion_from_euler(0, 0, angle)

                self.path_point.pose.pose.position.x = pos_x
                self.path_point.pose.pose.position.y = pos_y
                self.path_point.pose.pose.orientation.x = quat[0]
                self.path_point.pose.pose.orientation.y = quat[1]
                self.path_point.pose.pose.orientation.z = quat[2]
                self.path_point.pose.pose.orientation.w = quat[3]

                self.pub_path_point.publish(self.path_point)

                if(self.received_path==False):
                    self.received_path = True

        except:
            print("Nothing received this time")

        sleep(0.01)    
        
    def run(self):
        rospy.init_node('xrot_position_control', anonymous=True)
        self.sub_cmd_vel = rospy.Subscriber('/yape/cmd_vel',Twist,self.cmd_vel_cbk)
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            rate.sleep()
            self.read_pos_from_can()
            self.set_state()
            self.send_local_planner()
          

        self.bus.shutdown()

if __name__ == '__main__':
    try:
        xrot_position_control_handle = xrot_position_control()
        xrot_position_control_handle.run()
    except rospy.ROSInterruptException:
        pass 