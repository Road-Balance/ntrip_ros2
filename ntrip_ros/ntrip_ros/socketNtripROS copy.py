#!/usr/bin/env python3

import sys
import time
import rospy
import socket
import datetime

import rclpy
from rclpy.node import Node

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

        self.declare_parameter('rtcm_topic', "/rtcm")
        self.declare_parameter('target_host', "fkp.ngii.go.kr")
        self.declare_parameter('target_port', "2201")
        self.declare_parameter('mountpoint', "VRS_V32")
        self.declare_parameter('useragent', "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36")
        self.declare_parameter('user', "tge1375@naver.com:gnss")
        self.declare_parameter('verbose', True)
        
        self.rtcm_topic = str(self.get_parameter('rtcm_topic'))
        self.target_host = str(self.get_parameter('target_host'))
        self.target_port = int(self.get_parameter('target_port'))
        self.mountpoint = str(self.get_parameter('mountpoint'))
        self.useragent = str(self.get_parameter('useragent'))
        self.user = str(self.get_parameter('user'))
        self.verbose = bool(self.get_parameter('verbose'))
        
        self.publisher = self.create_publisher(Message, 'self.rtcm_topic', 10)

        self.get_logger().info(
            self.rtcm_topic + 
            self.target_host +
            str(self.target_port) +
            self.mountpoint +
            self.useragent +
            self.user +
            str(self.verbose)
        )

        self.rtcm_publisher = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

        if type(self.verbose) is str:
            self.verbose = bool(self.verbose)

        if type(self.target_port) is str:
            self.target_port = int(self.target_port)

        self.force_stop = False

        self.ssl = False

        self.maxConnectTime = 0
        self.found_header = False

        self.reconnectTry = 1
        self.sleepTime = 1
        self.RTCM_ARR = []
        self.data = "Initial data"

        self.client = None
        self.rmsg = Message()

        self.ntripConnection()

        rospy.logwarn("{%s}:{%d} Connection Succeed..." % (self.target_host, self.target_port))

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
            print("$%s*%s\r\n" % (ggaString, checksum))
        
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
            print(mountPointString)
        return mountPointString.encode()

    def ntripConnection(self):
        if self.maxConnectTime > 0:
            self.EndConnect = datetime.timedelta(seconds=self.maxConnectTime)
        try:
            while self.reconnectTry <= maxReconnect:
                self.found_header = False
                if self.verbose:
                    print("Connection {0} of {1}\n".format(self.reconnectTry, maxReconnect))

                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.ssl:
                    self.client = self.ssl.wrap_socket(self.client)

                error_indicator = self.client.connect_ex(
                    (self.target_host, self.target_port)
                )
                if error_indicator == 0:
                    self.sleepTime = 1
                    self.connectTime = datetime.datetime.now()

                    self.client.settimeout(10)
                    self.client.send(self.getMountPointBytes())

                    # casterRequest
                    self.casterRequest()
                    self.reconnectTry += 1
                else:
                    self.client = None
                    if self.verbose:
                        print("Error indicator: ", error_indicator)

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

        except KeyboardInterrupt:
            if self.client:
                self.client.close()
            sys.exit()

    def casterRequest(self):

        while not self.found_header:
            casterResponse = self.client.recv(4096)  # All the data
            header_lines = casterResponse.decode("utf-8").split("\r\n")

            if self.verbose:
                print(header_lines)

            for line in header_lines:
                if line == "":
                    if not self.found_header:
                        self.found_header = True
                        if self.verbose:
                            print("End Of Header" + "\n")
                else:
                    if self.verbose:
                        print("Header: " + line + "\n")

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
                    self.client.sendall(self.getGGABytes())
                elif line.find("HTTP/1.0 200 OK") >= 0:
                    # Request was valid
                    self.client.sendall(self.getGGABytes())
                elif line.find("HTTP/1.1 200 OK") >= 0:
                    # Request was valid
                    self.client.sendall(self.getGGABytes())
                rospy.loginfo("getGGABytes")

    def pubRTCM(self):
        self.RTCM_ARR = []
        while self.data:
            try:
                self.data = self.client.recv(1)
                if ord(self.data) == 211:
                    self.RTCM_ARR.append(ord(self.data))

                    new_data = self.client.recv(2)
                    print(new_data, type(new_data), new_data[0], new_data[1])

                    self.RTCM_ARR.append(new_data[0])
                    self.RTCM_ARR.append(new_data[1])
                    print(self.RTCM_ARR)

                    cnt = new_data[0] * 256 + new_data[1]
                    cnt = cnt + 1
                    print("cnt : ", cnt)

                    if cnt > 255:  # Just Abstract Value
                        print("Over Msg Error")
                        self.RTCM_ARR = []
                        continue
                        # raise IndexError

                    for i in range(cnt):
                        new_data = self.client.recv(1)
                        self.RTCM_ARR.append(ord(new_data))

                    if self.force_stop:
                        return

                    print("Final Array : ", self.RTCM_ARR)
                    # ros_message = bytes(self.RTCM_ARR)
                    self.rmsg.message = bytes(self.RTCM_ARR)
                    self.rmsg.header.seq += 1
                    self.rmsg.header.stamp = rospy.get_rostime()

                    self.rtcm_publisher.publish(self.rmsg)

                    self.RTCM_ARR = []

                if self.maxConnectTime:
                    if datetime.datetime.now() > self.connectTime + self.EndConnect:
                        if self.verbose:
                            sys.stderr.write("Connection Time exceeded\n")
                        sys.exit(0)

            except KeyboardInterrupt:
                if self.client:
                    self.client.close()
                sys.exit()

            # TODO : TypeError: catching classes that do not inherit from BaseException is not allowed
            except self.client.timeout:
                if self.verbose:
                    sys.stderr.write("Connection TimedOut\n")
                self.data = False

            except self.client.error:
                if self.verbose:
                    sys.stderr.write("Connection Error\n")
                self.data = False

            except IndexError:
                self.RTCM_ARR = []
                pass

            except Exception as e:
                print("Unknown Error : ", e)
                pass



if __name__ == "__main__":
    rospy.init_node("maze_action_server")

    ntrip_ros_node = SocketNtrip("maze_action_server")
    rate = rospy.Rate(2)

    ntrip_ros_node.pubRTCM()