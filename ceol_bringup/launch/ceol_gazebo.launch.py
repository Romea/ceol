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
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import romea_common_meta_bringup.ros_launch as common
import romea_mobile_base_meta_bringup.ros_launch as mobile_base


def launch_setup(context, *args, **kwargs):

    mode = common.get_mode(context)
    robot_namespace = common.get_robot_namespace(context)
    robot_urdf_description = common.get_robot_urdf_description(context)

    robot = []

    if mode == "simulation_gazebo_classic":

        world = (
            get_package_share_directory("romea_simulation_gazebo_worlds")
            + "/worlds/friction_cone.world"
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros")
                    + "/launch/gzserver.launch.py"
                ),
                launch_arguments={"world": world, "verbose": "false"}.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros")
                    + "/launch/gzclient.launch.py"
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

    else:

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("ros_gz_sim")
                    + "/launch/gz_sim.launch.py"
                ),
                launch_arguments={
                    # 'gz_args': '/tmp/gazebo_world.world',
                    "gz_args": "-g",
                    "on_exit_shutdown": "True",
                }.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("ros_gz_sim")
                    + "/launch/gz_server.launch.py"
                ),
                launch_arguments={
                    "world_sdf_file": "empty.sdf",
                    "world_sdf_string": "world",
                }.items(),
            )
        )

        robot_description_file = "/tmp/ceol_description.urdf"
        with open(robot_description_file, "w") as f:
            f.write(robot_urdf_description)

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("ros_gz_sim")
                    + "/launch/gz_spawn_model.launch.py"
                ),
                launch_arguments=[
                    ("file", "/tmp/ceol_description.urdf"),
                    ("entity_name", robot_namespace),
                ],
            )
        )

        robot.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen'
            )
        )

    return robot


def generate_launch_description():

    return LaunchDescription(
        [
            common.declare_mode("simulation"),
            common.declare_robot_namespace("ceol"),
            mobile_base.declare_base_name("base"),
            common.declare_robot_urdf_description(
                common.generate_robot_urdf_description("ceol_bringup")
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
