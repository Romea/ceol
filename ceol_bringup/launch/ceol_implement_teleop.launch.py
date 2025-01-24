# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from romea_teleop_description import get_default_joystick_implement_remapping, joystick_remapping


def get_joystick_type(context):
    return LaunchConfiguration("joystick_type").perform(context)


def get_joystick_driver(context):
    return LaunchConfiguration("joystick_driver").perform(context)


def get_joystick_topic(context):
    return LaunchConfiguration("joystick_topic").perform(context)


def launch_setup(context, *args, **kwargs):

    joystick_type = get_joystick_type(context)
    joystick_driver = get_joystick_driver(context)
    joystick_topic = get_joystick_topic(context)

    implement_teleop_configuration = {}
    implement_teleop_configuration["joystick_mapping"] = joystick_remapping(
        joystick_type, joystick_driver, get_default_joystick_implement_remapping(joystick_type)
    )
    implement_teleop = Node(
        package="romea_teleop_drivers",
        executable="implement_teleop_node",
        name="implement_teleop",
        parameters=[implement_teleop_configuration],
        output="screen",
        remappings=[("joystick/joy", joystick_topic)],
    )

    return [implement_teleop]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("joystick_type"))

    declared_arguments.append(
        DeclareLaunchArgument("joystick_driver", default_value="joy")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_topic", default_value="joystick/joy")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
