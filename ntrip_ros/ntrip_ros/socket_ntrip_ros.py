#!/usr/bin/env python3

import sys
import time
import socket
import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rtcm_msgs.msg import Message

# 전역
factor = 2  # How much the sleep time increases with each failed attempt
maxReconnect = 1
maxReconnectTime = 1200
sleepTime = 1  # So the first one is 1 second
V2 = True

class SocketNtrip(Node):

    def __init__(self):
        super().__init__('socket_ntrip')

        self.declare_parameter('rtcm_topic')
        self.declare_parameter('target_host')
        # self.declare_parameter('target_host', "fkp.ngii.go.kr")
        self.declare_parameter('target_port')
        # self.declare_parameter('mountpoint', "VRS_V32")
        # self.declare_parameter('useragent', "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36")
        # self.declare_parameter('user', "tge1375@naver.com:gnss")
        # self.declare_parameter('verbose', True)
        
        rtcm_topic = Parameter('rtcm_topic', Parameter.Type.STRING, '/rtcm')
        target_port = Parameter('target_port', Parameter.Type.INTEGER, 2201)
        # param_int = Parameter('my_int', Parameter.Type.INTEGER, 12)
        # param_double_array = Parameter('my_double_array', Parameter.Type.DOUBLE_ARRAY, [1.1, 2.2])

        self.set_parameters([rtcm_topic, target_port])

        self.rtcm_topic = self.get_parameter('rtcm_topic').value
        self.target_port = self.get_parameter('target_port').value
        print(self.rtcm_topic, type(self.rtcm_topic))
        print(self.target_port, type(self.target_port))


        # self.rtcm_topic = str(self.get_parameter('rtcm_topic'))
        # self.target_host = str(self.get_parameter('target_host'))
        # self.target_port = int(self.get_parameter('target_port'))
        # self.mountpoint = str(self.get_parameter('mountpoint'))
        # self.useragent = str(self.get_parameter('useragent'))
        # self.user = str(self.get_parameter('user'))
        # self.verbose = bool(self.get_parameter('verbose'))
        
        # self.publisher = self.create_publisher(Message, 'self.rtcm_topic', 10)

        # self.get_logger().info(
        #     self.rtcm_topic + 
        #     self.target_host +
        #     str(self.target_port) +
        #     self.mountpoint +
        #     self.useragent +
        #     self.user +
        #     str(self.verbose)
        # )

def main(args=None):
    rclpy.init(args=args)

    socket_ntrip_node = SocketNtrip()
    rclpy.spin(socket_ntrip_node)
    socket_ntrip_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
