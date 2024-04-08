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


import yaml

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    joystick_device = LaunchConfiguration("joystick_device").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)

    robot = []

    if mode == "simulation":
        mode += "_gazebo_classic"

    if mode == "simulation_gazebo_classic":

        world = PathJoinSubstitution(
            [
                FindPackageShare("romea_simulation_gazebo_worlds"),
                "worlds",
                "friction_cone.world",
            ]
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("gazebo_ros"),
                                "launch",
                                "gzserver.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={"world": world, "verbose": "false"}.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("gazebo_ros"),
                                "launch",
                                "gzclient.launch.py",
                            ]
                        )
                    ]
                )
            )
        )

        robot_description_file = "/tmp/ceol_description.urdf"
        with open(robot_description_file, "w") as f:
            f.write(robot_urdf_description)

        robot.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file",
                    robot_description_file,
                    "-entity",
                    "ceol",
                ],
                output={
                    'stdout': 'log',
                    'stderr': 'log',
                },
            )
        )

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ceol_bringup"),
                            "launch",
                            "ceol_base.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={"mode": mode}.items(),
        )
    )

    teleop_configuration_file_path = (
        get_package_share_directory("ceol_description") + "/config/teleop.yaml"
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("ceol"),
                PushRosNamespace("base"),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        get_package_share_directory("romea_teleop_drivers")
                        + "/launch/teleop.launch.py"
                    ),
                    launch_arguments={
                        "robot_type": "ceol",
                        "joystick_type": joystick_type,
                        "joystick_driver": "joy",
                        "joystick_topic": "/ceol/joystick/joy",
                        "teleop_configuration_file_path": teleop_configuration_file_path,
                    }.items(),
                )
            ]
        )
    )

    joy_params_path = '/tmp/joy_parameters.yaml'
    joy_params = {
        'dead_zone': 0.05,
        'autorepeat_rate': 10.0,
        'frame_id': 'joy',
    }
    with open(joy_params_path, 'w') as file:
        file.write(yaml.safe_dump(joy_params))

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("ceol"),
                PushRosNamespace("joystick"),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("romea_joystick_bringup"),
                                    "launch",
                                    "drivers/joy.launch.py",
                                ]
                            )
                        ]
                    ),
                    launch_arguments={
                        'executable': 'joy_node',
                        'config_path': joy_params_path,
                        'frame_id': joy_params['frame_id'],
                    }.items(),
                )
            ]
        )
    )

    return robot


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    urdf_description = Command(
        [
            ExecutableInPackage("urdf_description.py", "ceol_bringup"),
            " robot_namespace:ceol",
            " base_name:base",
            " mode:",
            LaunchConfiguration("mode"),
        ],
        on_stderr="ignore",
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_urdf_description", default_value=urdf_description)
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_type", default_value="xbox")
    )

    declared_arguments.append(
        DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
