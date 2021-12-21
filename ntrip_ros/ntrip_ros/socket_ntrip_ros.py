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
        self.declare_parameter('target_port')
        self.declare_parameter('mountpoint')
        self.declare_parameter('useragent', "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36")
        self.declare_parameter('user', "tge1375@naver.com:gnss")
        self.declare_parameter('verbose', True)
        
        rtcm_topic = Parameter('rtcm_topic', Parameter.Type.STRING, '/rtcm')
        target_host = Parameter('target_host', Parameter.Type.STRING, 'fkp.ngii.go.kr')
        target_port = Parameter('target_port', Parameter.Type.INTEGER, 2201)
        mountpoint = Parameter('mountpoint', Parameter.Type.STRING, 'VRS_V32')
        useragent = Parameter('useragent', Parameter.Type.STRING, "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36")
        user = Parameter('user', Parameter.Type.STRING, "tge1375@naver.com:gnss")
        verbose = Parameter('verbose', Parameter.Type.BOOL, True)

        self.set_parameters([
            rtcm_topic, 
            target_host,
            target_port,
            mountpoint,
            useragent,
            user,
            verbose,
        ])

        self.rtcm_topic = self.get_parameter('rtcm_topic').value
        self.target_host = self.get_parameter('target_host').value
        self.target_port = self.get_parameter('target_port').value
        self.mountpoint = self.get_parameter('mountpoint').value
        self.useragent = self.get_parameter('useragent').value
        self.user = self.get_parameter('user').value
        self.verbose = self.get_parameter('verbose').value
        
        self.publisher = self.create_publisher(Message, self.rtcm_topic, 10)
        self.pub_msg = Message()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)

    def publish_callback(self):
        self.pub_msg.header.frame_id = ""
        self.pub_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_msg.message = bytes([100, 101, 102])

        self.publisher.publish(self.pub_msg)

def main(args=None):
    rclpy.init(args=args)

    socket_ntrip_node = SocketNtrip()
    rclpy.spin(socket_ntrip_node)
    socket_ntrip_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
