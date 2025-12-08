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

from ament_index_python.packages import get_package_share_directory
from ceol_description import get_specifications_path_file

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


import romea_common_meta_bringup.ros_launch as common
import romea_joystick_meta_bringup.ros_launch as joystick
# import romea_teleop_meta_bringup.launch as teleop


def launch_setup(context, *args, **kwargs):

    mode = common.get_mode(context)
    joystick_topic = joystick.get_joystick_topic(context)
    joystick_configuration_file_path = joystick.get_joystick_configuration_file_path(context)
    # teleop_configuration_file_path = teleop.get_teleop_configuration_file_path(context)
    mobile_base_configuration_file_path = get_specifications_path_file()

    teleop_configuration_file_path = LaunchConfiguration(
        "teleop_configuration_file_path"
    ).perform(context)

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_mobile_base_teleop")
            + "/launch/teleop.launch.py"
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
            common.declare_mode(),
            joystick.declare_joystick_topic(),
            joystick.declare_joystick_configuration_file_path(),
            DeclareLaunchArgument(
                "teleop_configuration_file_path",
                default_value=default_teleop_configuration_file_path
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
