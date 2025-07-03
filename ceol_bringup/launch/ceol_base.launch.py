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

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from ceol_bringup import generate_ros2_control_description


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    if "replay" in mode:
        return []

    if mode == "simulation":
        mode += "_gazebo_classic"

    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    base_name = LaunchConfiguration("base_name").perform(context)

    base_configuration_file_path = (
        f'{get_package_share_directory("ceol_description")}/config/ceol.yaml'
    )

    controller_manager_configuration_file_path = (
        f'{get_package_share_directory("ceol_bringup")}/config/controller_manager.yaml'
    )

    if "live" in mode:
        base_controller_configuration_file_path = (
            get_package_share_directory("ceol_bringup") + 
            "/config/mobile_base_controller_live.yaml"
        )
    else:
        base_controller_configuration_file_path = (
            get_package_share_directory("ceol_bringup")
            + "/config/mobile_base_controller_simulation.yaml"
        )

    ros2_control_description_node = Node(
        package="romea_common_meta_bringup",
        executable="urdf_broadcaster_node",
        name="ros2_control_description",
        parameters=[
            {
                "robot_description":
                generate_ros2_control_description(tf_prefix, mode, base_name),
            }
        ],
    )

    controller_manager = Node(
        condition=IfCondition(PythonExpression(["'gazebo' not in '", mode, "'"])),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # {"robot_description": base_ros2_control_description},
            controller_manager_configuration_file_path,
        ],
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_mobile_base_controllers")
            + "/launch/mobile_base_controller.launch.py"
        ),
        launch_arguments={
            "joints_prefix": tf_prefix,
            "controller_name": "mobile_base_controller",
            "base_configuration_file_path": base_configuration_file_path,
            "base_controller_configuration_file_path": base_controller_configuration_file_path,
        }.items(),
    )

    cmd_mux = Node(
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": "romea_mobile_base_msgs/SkidSteeringCommand"}],
        remappings=[("~/out", "controller/cmd_skid_steering")],
        output="screen",
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                # can_receiver,
                ros2_control_description_node,
                controller_manager,
                controller,
                cmd_mux,
            ]
        )
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("base_name", default_value="base"),
            OpaqueFunction(function=launch_setup),
        ]
    )
