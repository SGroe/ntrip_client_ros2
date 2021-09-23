# ntrip_client

NTRIP client for ROS2 that sends NMEA-GGA messages to caster in return for RTCM correction messages.

Code is based on XSens Knowledgebase article
[Using an NTRIP client with the XSens ROS driver|https://base.xsens.com/knowledgebase/s/article/Using-an-NTRIP-client-with-the-Xsens-ROS-driver] 
and
[https://github.com/dayjaby/ntrip_ros]

Implemented changes on the original code:

 * Code converted from ROS1 to ROS2
 * Added continuous sending of GGA message to NTRIP caster (after first coordinate is received)

## Launch ROS2 node

Server properties are set in file ´´´resource/ntrip_client.properties´´´

´´´ros2 launch ntrip_client start_ntrip_client.launch.py´´´

## Requirements

pip install jproperties
