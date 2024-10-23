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

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    # Toggle Gazebo GUI client
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false"
    )
    ld.add_action(rviz_arg)

    # Add Gazebo to launch step
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"),
        "launch/gazebo.launch.py"
    ])

    # Load walker into the world
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="simple_walker"
    )

    # Start the controller manager
    ros2_control_params_file = PathJoinSubstitution([
        FindPackageShare("walker_description"),
        "config", "ros2_controllers.yaml"
    ])
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control_params_file],
        remappings=[('/controller_manager/robot_description', '/robot_description')]
    )
    ld.add_action(controller_manager)

    # Add RVIZ to launch step
    rviz_launch_path = PathJoinSubstitution([
        FindPackageShare("walker_description"),
        "launch", "rviz.launch.py"
    ])
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path)
    )
    ld.add_action(rviz)

    return ld
