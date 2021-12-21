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
        
        self.force_stop = False
        self.ssl = False

        self.maxConnectTime = 0
        self.found_header = False

        self.reconnectTry = 1
        self.sleepTime = 1
        self.RTCM_ARR = []
        self.data = "Initial data"

        # Socket Client
        self.client = None

        self.publisher = self.create_publisher(Message, self.rtcm_topic, 10)
        self.pub_msg = Message()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)

        # Create Fisrt Connection!!
        self.ntripConnection()

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def getGGABytes(self):
        now = datetime.datetime.utcnow()
        ggaString = (
            "GPGGA,%02d%02d%04.2f,3734.087,N,12702.603,E,1,12,1.0,0.0,M,0.0,M,,"
            % (now.hour, now.minute, now.second)
        )

        checksum = self.calcultateCheckSum(ggaString)
        if self.verbose:
            self.get_logger().info("$%s*%s\r\n" % (ggaString, checksum))
        return bytes("$%s*%s\r\n" % (ggaString, checksum), "ascii")

    def getMountPointBytes(self):
        mountPointString = (
            "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n"
            % (self.mountpoint, self.useragent, self.user)
        )

        # if self.host or self.V2:
        #    hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
        #    mountPointString += hostString

        if V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"

        if self.verbose:
            self.get_logger().info(mountPointString)
        return mountPointString.encode()

    def createSocket(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.ssl:
            self.client = self.ssl.wrap_socket(self.client)

        error_indicator = self.client.connect_ex(
            (self.target_host, self.target_port)
        )

        return error_indicator

    def handleClientException(self):
        self.client = None

        if self.verbose:
            self.get_logger().info("Error indicator: ", error_indicator)

        if self.reconnectTry < maxReconnect:
            sys.stderr.write(
                "%s No Connection to NtripCaster.  Trying again in %i seconds\n"
                % (datetime.datetime.now(), sleepTime)
            )

            time.sleep(sleepTime)

            sleepTime *= factor
            if sleepTime > maxReconnectTime:
                sleepTime = maxReconnectTime

        self.reconnectTry += 1

    def ntripConnection(self):
        if self.maxConnectTime > 0:
            self.EndConnect = datetime.timedelta(seconds=self.maxConnectTime)
        try:
            while self.reconnectTry <= maxReconnect:
                self.found_header = False
                if self.verbose:
                    self.get_logger().info("Connection {0} of {1}\n".format(self.reconnectTry, maxReconnect))

                error_indicator = self.createSocket()

                if error_indicator == 0:
                    self.sleepTime = 1
                    self.connectTime = datetime.datetime.now()

                    self.client.settimeout(10)
                    self.client.send(self.getMountPointBytes())

                    # casterRequest
                    self.casterRequest()
                    self.reconnectTry += 1
                else:
                    self.handleClientException()
        except KeyboardInterrupt:
            if self.client:
                self.client.close()
            sys.exit()

    def casterRequest(self):
        while not self.found_header:
            casterResponse = self.client.recv(4096)  # All the data
            header_lines = casterResponse.decode("utf-8").split("\r\n")

            if self.verbose:
                self.get_logger().info("".join(header_lines))

            for line in header_lines:
                if line == "":
                    if not self.found_header:
                        self.found_header = True
                        if self.verbose:
                            self.get_logger().info("End Of Header" + "\n")
                else:
                    if self.verbose:
                        self.get_logger().info("Header: " + line + "\n")

            request_output = None
            for line in header_lines:
                if line.find("SOURCETABLE") >= 0:
                    sys.stderr.write("Mount point does not exist")
                    sys.exit(1)
                elif line.find("401 Unauthorized") >= 0:
                    sys.stderr.write("Unauthorized request\n")
                    sys.exit(1)
                elif line.find("404 Not Found") >= 0:
                    sys.stderr.write("Mount Point does not exist\n")
                    sys.exit(2)
                elif line.find("ICY 200 OK") >= 0:
                    # Request was valid
                    self.get_logger().warn("Valid Request : ICY 200 OK")
                    self.client.sendall(self.getGGABytes())
                elif line.find("HTTP/1.0 200 OK") >= 0:
                    # Request was valid
                    self.get_logger().warn("Valid Request : HTTP/1.0 200 OK")
                    self.client.sendall(self.getGGABytes())
                elif line.find("HTTP/1.1 200 OK") >= 0:
                    # Request was valid
                    self.get_logger().warn("Valid Request : HTTP/1.1 200 OK")
                    self.client.sendall(self.getGGABytes())

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
