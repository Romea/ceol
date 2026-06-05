# ceol_description

## 1) Overview

`ceol_description` provides the robot-specific description layer for the Ceol mobile base.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate the Ceol robot.

The Ceol mobile base uses:

| Configuration file | Mobile base architecture | Command type |
|---|---|---|
| `config/ceol.yaml` | `2THD` | `skid_steering` |

This architecture has two continuous tracks driven by left and right sprocket wheels. It is controlled as a skid-steering mobile base.

## 2) Robot configuration

The `config/` directory contains:

| File | Purpose |
|---|---|
| `ceol.yaml` | full Ceol mobile base configuration |
| `teleop.yaml` | default skid-steering teleoperation configuration |

The robot configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`2THD`);
* track geometry, sprocket, idler and roller wheel dimensions;
* track speed command and feedback information;
* the equivalent wheelbase used by skid-steering control models;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/ceol.urdf.xacro` | main URDF entry point |
| `urdf/ceol.xacro` | Ceol mobile base macro |
| `urdf/ceol.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `urdf/visual/` | visual Xacro fragments for chassis and wheels |
| `meshes/` | chassis and wheel visual meshes |

The Ceol macro reuses the `base2THD.chassis.xacro` template from `romea_mobile_base_description` and specializes it with Ceol geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/ceol.ros2_control.urdf.xacro` | main `ros2_control` entry point |
| `ros2_control/ceol.ros2_control.xacro` | Ceol `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects:

| Mode | Hardware plugin |
|---|---|
| `live` | `ceol_hardware/CeolHardware` |
| `simulation`, `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface2THD` |
| `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface2THD` |
| `simulation_4dv`, `simulation_isaac` | `romea_mobile_base_hardware/GenericHardwareSystemInterface2THD` |

## 4) Python API

The installed Python module provides helper functions used by `ceol_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file()` | returns the Ceol configuration file path |
| `get_specifications_configuration()` | loads the full robot configuration |
| `get_configuration()` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the Ceol URDF description |
| `generate_ros2_control_description(...)` | generates the Ceol `ros2_control` description |

Example:

```python
from ceol_description import get_configuration

configuration = get_configuration()
```

When `mode` is set to `simulation`, the Python API maps it to `simulation_gazebo` before generating the URDF or `ros2_control` description.

## 5) Relation with generic mobile base packages

`ceol_description` is the Ceol specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `2THD` description and `ros2_control` templates;
* `ceol_description` provides the Ceol configuration, meshes and Xacro specialization;
* `ceol_hardware` provides the live `ros2_control` hardware plugin;
* `ceol_bringup` uses this package to generate configuration, URDF and `ros2_control` artifacts for live and simulation modes.
