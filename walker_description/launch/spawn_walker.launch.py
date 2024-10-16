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
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription()

    # Retrieve the name of the model
    model_name = LaunchConfiguration("model_name")

    # Arguments describing how to spawn model
    model_description = ['-entity', model_name, '-topic', '/robot_description']
    model_pos = ['-x', '0.0', '-y', '0.0', '-z', '1.5']
    model_orientation = ['-R', '0.0', '-P', '0.0', '-Y', '0.0']

    spawn_params = model_description + model_pos + model_orientation

    # Spawn Jetleg in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=spawn_params,
        output='screen'
    )
    ld.add_action(spawn_entity)

    return ld
