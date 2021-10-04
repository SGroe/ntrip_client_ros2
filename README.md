# ntrip_client (ROS2 node)

NTRIP client for ROS2 that sends NMEA-GGA messages to caster in return for RTCM correction messages.

Code is based on XSens Knowledgebase article 'Using an NTRIP client with the XSens ROS driver'.
It is available at https://base.xsens.com/knowledgebase/s/article/Using-an-NTRIP-client-with-the-Xsens-ROS-driver.
The driver by XSens is based on https://github.com/dayjaby/ntrip_ros.

Implemented changes on the original code:

 * Code converted from ROS1 to ROS2
 * Added continuous sending of GGA message to NTRIP caster (after first coordinate is received)

## Configure and launch ROS2 node

NTRIP server properties are set with the following ROS2 parameters:

* server
* user
* password
* stream

This script can be used to set the parameter values after launching the node.

```shell script
ros2 param set /ntripclient server my_nrtk_server.com:7801
ros2 param set /ntripclient user myuser
ros2 param set /ntripclient password mypassword
ros2 param set /ntripclient stream mystream
```

Dump the parameters, copy the yaml-file into the /config directory of this node and rebuild the node.

```
ros2 param dump /ntripclient
```

The node is launched with the following command:

```shell script
ros2 launch ntrip_client start_ntrip_client.launch.py
```

## Requirements

```shell script
pip install jproperties
```
