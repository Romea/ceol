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

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

import romea_common_meta_bringup.ros_launch as common
import romea_joystick_meta_bringup.ros_launch as joystick


def launch_setup(context, *args, **kwargs):

    mode = common.get_mode(context)

    joystick_configuration_file_path = (
        get_package_share_directory("romea_joystick_utils")
        + "/config/" + joystick.get_joystick_model(context) + ".yaml"
    )

    robot = []

    if "simulation" in mode:

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("ceol_bringup")
                    + "/launch/ceol_gazebo.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "robot_namespace": "ceol",
                    "base_name": "base",
                }.items(),
            )
        )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("ceol_bringup")
            + "/launch/ceol_base.launch.py"
        ),
        launch_arguments={
            "mode": mode,
            "robot_namespace": "ceol",
            "base_name": "base",
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("ceol_bringup")
            + "/launch/ceol_teleop.launch.py"
        ),
        launch_arguments={
            "mode": mode,
            "joystick_configuration_file_path": joystick_configuration_file_path,
            "joystick_topic": "/ceol/joystick/joy",
        }.items(),
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("ceol"),
                PushRosNamespace("base"),
                base,
                teleop,
            ]
        )
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("ceol"),
                PushRosNamespace("joystick"),
                Node(package="joy", executable="joy_node"),
            ]
        )
    )

    return robot


def generate_launch_description():

    return LaunchDescription(
        [
            common.declare_mode("simulation"),
            joystick.declare_joystick_model("microsoft_xbox"),
            OpaqueFunction(function=launch_setup),
        ]
    )
