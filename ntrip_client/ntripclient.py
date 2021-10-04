# Copyright 2021 Salzburg Research
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
# See the License for the specific language governing permissions and
# limitations under the License.

import datetime
import time
import socket
from http.client import HTTPConnection
import base64
from threading import Thread

from ament_index_python.packages import get_package_prefix

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import RTCM


class NTripClient(Node):
    """
    NTRIP client that sends NMEA-GGA messages to caster in return for RTCM correction messages
    Code based on XSens Knowledgebase article: 'Using an NTRIP client with the XSens ROS driver'
    https://base.xsens.com/knowledgebase/s/article/Using-an-NTRIP-client-with-the-Xsens-ROS-driver
    https://github.com/dayjaby/ntrip_ros
    Implemented changes on the original code:
     * Code converted from ROS1 to ROS2
     * Added continuous sending of GGA message to NTRIP caster
    """
    
    longitude = None
    latitude = None
    
    def __init__(self):
        super().__init__('ntripclient')
        
        self.rtcm_topic = '/rtcm'
        # self.nmea_topic = '/nmea'
        
        parameter_descriptor_server = ParameterDescriptor(name="NTRIP server", type=ParameterType.PARAMETER_STRING)
        parameter_descriptor_user = ParameterDescriptor(name="NTRIP username for authentication", type=ParameterType.PARAMETER_STRING)
        parameter_descriptor_password = ParameterDescriptor(name="NTRIP password for authentication", type=ParameterType.PARAMETER_STRING)
        parameter_descriptor_stream = ParameterDescriptor(name="NTRIP stream", type=ParameterType.PARAMETER_STRING)
        
        self.declare_parameter('server', 'ntripserver.com:7801', parameter_descriptor_server)
        self.declare_parameter('user', 'myuser', parameter_descriptor_user)
        self.declare_parameter('password', 'mypassword', parameter_descriptor_password)
        self.declare_parameter('stream', 'mystream', parameter_descriptor_stream)
        
        self.ntrip_server = self.get_parameter('server').get_parameter_value().string_value
        self.ntrip_user = self.get_parameter('user').get_parameter_value().string_value
        self.ntrip_pass = self.get_parameter('password').get_parameter_value().string_value
        self.ntrip_stream = self.get_parameter('stream').get_parameter_value().string_value
        
        self.nmea_gga = 'GPGGA,%02d%02d%04.1f,%09.4f,%s,%010.4f,%s,1,24,0.6,38.9,M,84.4,M,0,0'
        
        self.subscription_gnss = self.create_subscription(
            NavSatFix,
            '/gnss',
            self.listener_callback_gnss,
            10)
        self.subscription_gnss  # prevent unused variable warning
        
        self.publisher = self.create_publisher(RTCM, '/rtcm', 10)

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()
        
    def listener_callback_gnss(self, msg):
        """
        Reads gnss data from XSens device
        """
        
        self.longitude = msg.longitude
        self.latitude = msg.latitude
        
    def log(self, msg):
        self.get_logger().info(str(msg))

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.connection = None
        self.stop = False

    def run(self):
        
        response = self.send_gga_message()
        
        while not response:
            time.sleep(10)
            self.send_gga_message()

        if response.status != 200:
            raise Exception("Response status " + str(response.status) + " > " + str(response.reason))
        else:
            self.ntc.log("Connected to NTRIP server!")
        
        self.ntc.log("Waiting for RTK data ...")
        
        last_send_time = time.time()
        previous_data = "".encode("latin-1")
        story = ""
        
        buf = ""
        rmsg = RTCM()
        while not self.stop:
            if last_send_time + 10 < time.time():
                response = self.send_gga_message()
                last_send_time = time.time()
            data = response.read(1)
            
            if data != "".encode("latin-1") and previous_data == "".encode("latin-1"):
                self.ntc.log("Receiving data ...")
                previous_data = data
            elif data == "".encode("latin-1") and previous_data != "".encode("latin-1"):
                self.ntc.log("Receiving NO data ...")
                previous_data = data
            
            if data!=chr(211).encode("latin-1"):
                continue
            
            l1 = ord(response.read(1))
            l2 = ord(response.read(1))
            pkt_len = ((l1&0x3)<<8)+l2
    
            pkt = response.read(pkt_len)
            parity = response.read(3)
            if len(pkt) != pkt_len:
                self.ntc.log("Length error: {} {}".format(len(pkt), pkt_len))
                continue
            # self.ntc.log(str(rmsg))
            # rmsg.header.seq += 1
            t = time.time()
            rmsg.header.stamp.sec = int(t)
            rmsg.header.stamp.nanosec = int((t - int(t)) * 1000000)
            rmsg.data = data + chr(l1).encode("latin-1") + chr(l2).encode("latin-1") + pkt + parity
            self.ntc.publisher.publish(rmsg)
            self.ntc.log("NTRIP message published")
        
        if self.connection is not None:
            self.connection.close()
        self.ntc.log("NTRIP connection closed")
        
    def send_gga_message(self):
        """
        Sends GGA message to the NTRIP caster via the specified connection
        """
    
        # Wait for current location
        waiting = False
        while self.ntc.longitude is None:
            if not waiting:
                self.ntc.log('Waiting for GNSS location ...')
            waiting = True
            time.sleep(1)
        self.ntc.log('Sending GGA message using available GNSS location ...')
    
        if self.connection is not None:
            # close previous connection
            self.connection.close()
        
        connection = HTTPConnection(self.ntc.ntrip_server)
        
        concatenated = str(self.ntc.ntrip_user) + ':' + self.ntc.ntrip_pass
        
        data = base64.encodebytes(concatenated.encode("utf-8")).replace('\n'.encode('utf-8'), ''.encode('utf-8'))
        
        try:
            connection.putrequest('GET', '/' + self.ntc.ntrip_stream)
            connection.putheader('Ntrip-Version', 'Ntrip/2.0')
            connection.putheader('User-Agent', 'NTRIP ntrip_ros')
            connection.putheader('Connection', 'close')
            connection.putheader('Authorization', 'Basic ' + data.decode('utf-8'))
            connection.endheaders()
        except socket.gaierror:
            self.ntc.log('Cannot create GET request on server ' + self.ntc.ntrip_server + '. Please check HTTP connection!')
            self.ntc.log('- Server: ' + self.ntc.ntrip_server)
            self.ntc.log('- User: ' + self.ntc.ntrip_user)
            self.ntc.log('- Stream: ' + self.ntc.ntrip_stream)
            return None
        
        now = datetime.datetime.utcnow()
        # nmeadata = self.ntc.nmea_gga % (now.hour, now.minute, now.second)
        latitude_value = int(self.ntc.latitude) * 100 + (self.ntc.latitude - int(self.ntc.latitude)) * 60
        longitude_value = int(self.ntc.longitude) * 100 + (self.ntc.longitude - int(self.ntc.longitude)) * 60
        latitude_sign = 'N' if self.ntc.latitude >= 0 else 'S'
        longitude_sign = 'E' if self.ntc.longitude >= 0 else 'W'
        nmeadata = self.ntc.nmea_gga % (now.hour, now.minute, now.second, latitude_value, latitude_sign, longitude_value, longitude_sign)
        
        csum = 0
        for c in nmeadata:
               # XOR'ing value of csum against the next char in line
               # and storing the new XOR value in csum
               if ord(c)!=',':
                  csum ^= ord(c)

        #convert hex characters to upper case
        csum = hex(csum).upper() 

        #add 0x0 if checksum value is less than 0x10
        if len(csum)==3:	
            csum='0'+csum[2]
        else:
            csum=csum[2:4]

        nmeastring = '$'+nmeadata+'*'+csum+'\r\n'
        
        self.ntc.log('Nmeastring: ' + nmeastring)
        
        connection.send(nmeastring.encode("latin-1"))
        
        self.ntc.longitude = None
        self.ntc.latitude = None
        
        return connection.getresponse()

def main(args=None):
    rclpy.init(args=args)

    ntrip_client = NTripClient()

    try:
        rclpy.spin(ntrip_client)
    except KeyboardInterrupt:
        pass

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # bicycle_cam_creator.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
