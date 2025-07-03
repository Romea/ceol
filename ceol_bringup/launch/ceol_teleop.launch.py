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

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

from ament_index_python.packages import get_package_share_directory
from ceol_description import get_specifications_path_file


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    joystick_topic = LaunchConfiguration("joystick_topic").perform(context)

    mobile_base_configuration_file_path = get_specifications_path_file()

    joystick_configuration_file_path = LaunchConfiguration(
        "joystick_configuration_file_path"
    ).perform(context)

    teleop_configuration_file_path = LaunchConfiguration(
        "teleop_configuration_file_path"
    ).perform(context)

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_mobile_base_teleop") + "/launch/teleop.launch.py"
        ),
        launch_arguments={
            "mobile_base_configuration_file_path": mobile_base_configuration_file_path,
            "joystick_configuration_file_path": joystick_configuration_file_path,
            "teleop_configuration_file_path": teleop_configuration_file_path,
            "joystick_topic": joystick_topic,
        }.items(),
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                teleop,
            ]
        )
    ]


def generate_launch_description():

    default_teleop_configuration_file_path = (
        get_package_share_directory("ceol_description") + "/config/teleop.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode"),
            DeclareLaunchArgument("joystick_configuration_file_path"),
            DeclareLaunchArgument("joystick_topic"),
            DeclareLaunchArgument(
                "teleop_configuration_file_path",
                default_value=default_teleop_configuration_file_path
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
