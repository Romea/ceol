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
from ceol_description import urdf


def urdf_description(prefix, mode, base_name, ros_prefix):

    controller_manager_yaml_file = (
        get_package_share_directory("ceol_bringup") + "/config/controller_manager.yaml"
    )

    return urdf(prefix, mode, base_name, controller_manager_yaml_file, ros_prefix)
