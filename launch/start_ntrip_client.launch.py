# ntrip client launch file

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Start ntrip client
    ntrip_client = Node(
            package='ntrip_client',
            executable='ntrip_client',
    )

    return LaunchDescription([
        ntrip_client
    ])
