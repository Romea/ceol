# Copyright 2023 Agreenculture
# Copyright 2023 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from romea_mobile_base_description import get_mobile_base_description
from romea_teleop_description import complete_teleop_configuration
import yaml


def get_teleop_configuration(context):

    teleop_configuration_file_path = LaunchConfiguration(
        "teleop_configuration_file_path"
    ).perform(context)

    with open(teleop_configuration_file_path) as f:
        return yaml.safe_load(f)


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
    teleop_configuration = get_teleop_configuration(context)

    # print("robot_type", robot_type)
    # print("robot_model", robot_model)
    # print("joystick_type", joystick_type)
    # print("joystick_driver", joystick_driver)

    mobile_base_info = get_mobile_base_description("ceol", "")
    teleop_configuration = complete_teleop_configuration(
        teleop_configuration, mobile_base_info, joystick_type, joystick_driver
    )

    teleop = Node(
        package="romea_teleop_drivers",
        executable="skid_steering_teleop_node",
        name="teleop",
        parameters=[teleop_configuration],
        output="screen",
        remappings=[("joystick/joy", joystick_topic)],
    )

    return [teleop]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("joystick_type"))

    declared_arguments.append(
        DeclareLaunchArgument("joystick_driver", default_value="joy")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_topic", default_value="joystick/joy")
    )

    declared_arguments.append(DeclareLaunchArgument("teleop_configuration_file_path"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
