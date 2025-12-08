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


import subprocess
import xml.etree.ElementTree as ET

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory


def urdf_xml(mode):

    exe = (
        get_package_prefix("ceol_bringup") + "/lib/ceol_bringup/generate_urdf_description.py"
    )

    return ET.fromstring(
        subprocess.check_output(
            [exe, "mode:" + mode, "base_name:base", "robot_namespace:robot"],
            encoding="utf-8",
        )
    )


def ros2_control_xml(mode):

    exe = (
        get_package_prefix("ceol_bringup")
        + "/lib/ceol_bringup/generate_ros2_control_description.py"
    )

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "mode:" + mode,
                "base_name:base",
                "robot_namespace:robot",
            ],
            encoding="utf-8",
        )
    )


def test_footprint_link_name():
    assert urdf_xml("live").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert ros2_control_xml("live").find(
        "ros2_control/hardware/plugin"
    ).text == "ceol_hardware/CeolHardware"

    assert ros2_control_xml("simulation").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2THD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation_gazebo_classic").find("gazebo/plugin/parameters").text
        == get_package_share_directory("ceol_bringup") + "/config/controller_manager.yaml"
    )
