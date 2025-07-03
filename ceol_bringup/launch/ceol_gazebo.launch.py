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
)

from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)

    robot = []

    if mode == "simulation":
        mode += "_gazebo_classic"

    if mode == "simulation_gazebo_classic":

        world = (
            get_package_share_directory("romea_simulation_gazebo_worlds")
            + "/worlds/friction_cone.world"
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros") + "/launch/gzserver.launch.py"
                ),
                launch_arguments={"world": world, "verbose": "false"}.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros") + "/launch/gzclient.launch.py"
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
                exec_name="gazebo_spawn_entity",
                arguments=["-file", robot_description_file, "-entity", "ceol"],
                output={"stdout": "log", "stderr": "log"},
            )
        )

    return robot


def generate_launch_description():

    urdf_description = Command(
        [
            ExecutableInPackage("generate_urdf_description.py", "ceol_bringup"),
            " robot_namespace:ceol",
            " base_name:base",
            " mode:",
            LaunchConfiguration("mode"),
        ],
        on_stderr="ignore",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="simulation"),
            DeclareLaunchArgument("robot_urdf_description", default_value=urdf_description),
            OpaqueFunction(function=launch_setup),
        ]
    )
