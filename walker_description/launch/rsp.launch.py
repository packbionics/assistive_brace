# Copyright (c) 2024 Pack Bionics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""Adds a robot_state_publisher to manage robot configuration."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # synchronizes ROS clock with simulation clock if true
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # specifies the XACRO/URDF file to utilize
    model = LaunchConfiguration('model')
    robot_urdf = Command(['xacro', ' ', model])

    # Stages the robot state publisher for execution
    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_urdf}],
    )
    ld.add_action(rsp)

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_robot_broadcaster",
        arguments=[
            '--x', '0', 
            '--y', '0', 
            '--z', '0', 
            '--qx', '0', 
            '--qy', '0', 
            '--qz', '0', 
            '--qw', '1', 
            '--frame-id', 'map', 
            '--child-frame-id', 'body'
        ]
    )
    ld.add_action(static_transform_publisher)

    return ld
