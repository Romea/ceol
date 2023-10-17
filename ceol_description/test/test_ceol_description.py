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


# import pytest
import xml.etree.ElementTree as ET
from ceol_description import urdf


def urdf_xml(mode):
    prefix = "robot_"
    ros_prefix = "/robot/"
    base_name = "base"
    controller_conf_yaml_file = mode + "_controller.yaml"
    return ET.fromstring(urdf(prefix, mode, base_name, controller_conf_yaml_file, ros_prefix))


def ros2_control_urdf_xml(mode):
    urdf_xml(mode)
    return ET.parse("/tmp/robot_base_ros2_control.urdf")


def test_footprint_link_name():
    assert urdf_xml("live").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert ros2_control_urdf_xml("live").find(
        "ros2_control/hardware/plugin"
    ).text == "ceol_hardware/CeolHardware"

    assert ros2_control_urdf_xml("simulation").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface2THD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation").find("gazebo/plugin/controller_manager_config_file").text
        == "simulation_controller.yaml"
    )
