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

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    rviz_toggle_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([
            FindPackageShare("walker_description"),
            "config", "visualize_walker.rviz"
        ]),
        description='Config file used to load RVIZ configs on startup'
    )
    ld.add_action(rviz_toggle_arg)

    robot_description_path_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution([
            FindPackageShare("walker_description"),
            'urdf', 'simple_walker.urdf.xacro'
        ])
    )
    ld.add_action(robot_description_path_arg)
    rsp_launch_path = PathJoinSubstitution([
        FindPackageShare("walker_description"),
        "launch/rsp.launch.py"
    ])
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path)
    )
    ld.add_action(rsp)

    robot_system_path = PathJoinSubstitution([
        FindPackageShare("walker_description"),
        'launch', 'fake_system.launch.py'
    ])
    robot_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_system_path)
    )
    ld.add_action(robot_system)

    spawn_controllers_path = PathJoinSubstitution([
        FindPackageShare("walker_description"),
        'launch', 'spawn_controllers.launch.py'
    ])
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_controllers_path)
    )
    ld.add_action(spawn_controllers)

    return ld
