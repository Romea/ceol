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
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    base_name = LaunchConfiguration("base_name").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    if "replay" in mode:
        return []

    if mode == "simulation":
        mode += "_gazebo_classic"

    if robot_namespace:
        controller_manager_name = "/" + robot_namespace + "/base/controller_manager"
        robot_prefix = robot_namespace + "_"
    else:
        controller_manager_name = "/base/controller_manager"
        robot_prefix = ""

    base_description_yaml_file = (
        get_package_share_directory("ceol_description") + "/config/ceol.yaml"
    )

    controller_manager_yaml_file = (
        get_package_share_directory("ceol_bringup") + "/config/controller_manager.yaml"
    )

    if "live" in mode:
        base_controller_yaml_file = (
            get_package_share_directory("ceol_bringup") + "/config/mobile_base_controller_live.yaml"
        )
    else:
        base_controller_yaml_file = (
            get_package_share_directory("ceol_bringup")
            + "/config/mobile_base_controller_simulation.yaml"
        )

    base_ros2_control_description_file = "/tmp/" + robot_prefix + base_name + "_ros2_control.urdf"
    with open(base_ros2_control_description_file, "r") as f:
        base_ros2_control_description = f.read()

    controller_manager = Node(
        condition=IfCondition(PythonExpression(["'gazebo' not in '", mode, "'"])),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": base_ros2_control_description},
            controller_manager_yaml_file,
        ],
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_mobile_base_controllers"),
                        "launch",
                        "mobile_base_controller.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "joints_prefix": robot_prefix,
            "controller_name": "mobile_base_controller",
            "controller_manager_name": controller_manager_name,
            "base_description_yaml_filename": base_description_yaml_file,
            "base_controller_yaml_filename": base_controller_yaml_file,
            "angular_speed_input_topic": f"/{robot_namespace}/imu/data",
        }.items(),
        condition=LaunchConfigurationNotEquals("mode", "replay"),
    )

    cmd_mux = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
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
                PushRosNamespace(robot_namespace),
                PushRosNamespace(base_name),
                controller_manager,
                controller,
                cmd_mux,
            ]
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("mode", default_value="simulation"),
        DeclareLaunchArgument("robot_namespace", default_value="ceol"),
        DeclareLaunchArgument("base_name", default_value="base"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
