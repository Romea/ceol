# ceol

## Overview

`ceol` groups the ROS2 packages that describe, launch and control the Ceol mobile base in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `ceol` | Metapackage that groups the Ceol ROS2 packages. |
| `ceol_description` | Robot-specific description layer for Ceol, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `ceol_bringup` | Main integration entry point for generating Ceol configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `ceol_hardware` | Live `ros2_control` hardware plugin for the Ceol mobile base, built on the continuous-track hardware abstraction. |
| `ceol_msgs` | Ceol-specific message definitions used by the live hardware and robot interface. |

## Usage

In most cases, start with `ceol_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when a Ceol model is selected from a mobile base meta-description.

The Ceol stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `2THD` and it is commanded as a skid-steering robot; `ceol_description` provides the concrete geometry and generated descriptions, `ceol_hardware` provides the live hardware implementation, and `ceol_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `ceol` project was developed by Jean Laneurit in the context of the TIRREX ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
