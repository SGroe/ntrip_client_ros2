# ntrip client launch file

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ntrip_client'),
        # 'config',
        'ntripclient.yaml'
        )

    # Start ntrip client
    ntrip_client = Node(
            package='ntrip_client',
            executable='ntrip_client',
            parameters = [config]
    )

    return LaunchDescription([
        ntrip_client
    ])
