#!/usr/bin/env python3
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


import xacro

from ament_index_python.packages import get_package_share_directory


def urdf(prefix, mode, base_name, controller_manager_config_yaml_file, ros_prefix):

    if mode == "simulation":
        mode += "_gazebo_classic"

    ros2_control_xacro_file = (
        get_package_share_directory("ceol_description")
        + "/ros2_control/ceol.ros2_control.urdf.xacro"
    )

    ros2_control_urdf_xml = xacro.process_file(
        ros2_control_xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
        },
    )

    ros2_control_config_urdf_file = "/tmp/"+prefix+base_name+"_ros2_control.urdf"

    with open(ros2_control_config_urdf_file, "w") as f:
        f.write(ros2_control_urdf_xml.toprettyxml())

    base_xacro_file = (
        get_package_share_directory("ceol_description") + "/urdf/ceol.urdf.xacro"
    )

    base_urdf_xml = xacro.process_file(
        base_xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "base_name": base_name,
            "controller_manager_config_yaml_file": controller_manager_config_yaml_file,
            "ros2_control_config_urdf_file": ros2_control_config_urdf_file,
            "ros_prefix": ros_prefix
        },
    )

    return base_urdf_xml.toprettyxml()
