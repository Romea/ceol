# ceol_bringup

## 1) Overview

`ceol_bringup` connects the Ceol description, hardware, simulation and teleoperation packages to the generic `romea_mobile_base_meta_bringup` workflow.

It provides:

* robot-specific generation functions for configuration, URDF and `ros2_control` descriptions;
* launch files for live control, Gazebo simulation and teleoperation;
* controller manager and mobile base controller parameter files.

The Ceol mobile base uses the `2THD` architecture and is commanded as a skid-steering robot. Its default controller is `romea_mobile_base_controllers/MobileBaseEnhancedController2TD`, which can use IMU angular speed feedback in addition to track speed feedback.

## 2) Generated artifacts

The Python module `ceol_bringup` delegates most generation work to `ceol_description` and adds bringup-specific configuration such as the controller manager parameter file.

It provides the functions expected by `romea_mobile_base_meta_bringup`:

| Function | Purpose |
|---|---|
| `get_configuration()` | returns the compact mobile base configuration |
| `generate_configuration_file(extended)` | generates the mobile base configuration file |
| `generate_urdf_description(prefix, mode, base_name, ros_prefix)` | generates the Ceol URDF description |
| `generate_ros2_control_description(prefix, mode, base_name)` | generates the Ceol `ros2_control` description |

The executable scripts in `scripts/` expose these functions from the command line.

The configuration generator writes the compact `2THD` mobile base configuration used by controllers, teleoperation and launch files. It is derived from `ceol_description/config/ceol.yaml`.

```bash
ros2 run ceol_bringup generate_configuration_file.py \
  extended:false
```

The URDF generator writes the Ceol robot description. It contains the `2THD` continuous-track link and joint structure, inertial data, collision geometry, visual meshes and the simulator plugin block when a simulation mode is selected.

```bash
ros2 run ceol_bringup generate_urdf_description.py \
  robot_namespace:ceol \
  base_name:base \
  mode:simulation_gazebo
```

The `ros2_control` generator writes the hardware description consumed by `controller_manager`. It declares the hardware plugin selected by the mode, the geometric hardware parameters and the command/state interfaces for the sprocket wheel joints.

```bash
ros2 run ceol_bringup generate_ros2_control_description.py \
  robot_namespace:ceol \
  base_name:base \
  mode:live
```

## 3) Launch files

### 3.1) Base launch

`launch/ceol_base.launch.py` starts the Ceol mobile base control stack.

It:

* receives the generated robot URDF and `ros2_control` description from the meta-bringup launch context;
* starts `controller_manager/ros2_control_node` in non-Gazebo modes;
* loads `joint_state_broadcaster`;
* loads `mobile_base_controller` using `romea_mobile_base_controllers/MobileBaseEnhancedController2TD`;
* starts `romea_cmd_mux` and remaps its output to `controller/cmd_skid_steering`.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | execution mode, such as `live`, `simulation_gazebo` or `simulation_gazebo_classic` |
| `robot_namespace` | namespace of the robot |
| `base_name` | namespace of the mobile base, usually `base` |

### 3.2) Teleoperation launch

`launch/ceol_teleop.launch.py` starts the mobile base teleoperation stack through `romea_mobile_base_teleop`.

It uses:

* the Ceol robot configuration from `ceol_description/config/ceol.yaml`;
* the joystick configuration file, usually selected from the `config/` directory of `romea_joystick_utils` according to the joystick type;
* the teleoperation configuration from `ceol_description/config/teleop.yaml` by default.

The teleoperation node publishes `romea_mobile_base_msgs/SkidSteeringCommand`, consistent with the Ceol skid-steering command type.

To move the robot, the operator must hold either the slow mode or turbo mode button. The joystick axes then command the longitudinal and angular speeds.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | execution mode, used to configure simulation time |
| `joystick_topic` | joystick `sensor_msgs/msg/Joy` topic |
| `joystick_configuration_file_path` | joystick configuration file, usually selected from `romea_joystick_utils/config/` |
| `teleop_configuration_file_path` | teleoperation configuration file, defaulting to `ceol_description/config/teleop.yaml` |

![Ceol teleoperation mapping](doc/teleop.jpg)

### 3.3) Gazebo launch

`launch/ceol_gazebo.launch.py` starts a Gazebo or Gazebo Classic simulation and spawns the Ceol entity from the generated URDF.

It supports:

* `simulation_gazebo`, using `ros_gz_sim` and `gz_ros2_control`;
* `simulation_gazebo_classic`, using `gazebo_ros` and `gazebo_ros2_control`.

The `ros2_control` hardware plugin used in simulation is selected by `ceol_description` from the generated `mode`.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | simulation mode, usually `simulation_gazebo` or `simulation_gazebo_classic` |
| `robot_namespace` | namespace of the robot and simulation entity |
| `base_name` | namespace of the mobile base, usually `base` |

### 3.4) Implement teleoperation launch

`launch/ceol_implement_teleop.launch.py` starts the teleoperation stack used to command the Ceol rear implement actuators.

This launch file is separate from the mobile base teleoperation launch because it controls the implement side of the robot, not the skid-steering motion controller.

### 3.5) Test launch

`launch/ceol_test.launch.py` starts a compact test setup with:

* the Ceol simulation when the selected mode contains `simulation`;
* the Ceol base launch;
* the Ceol teleoperation launch;
* a joystick node using the selected joystick model.

In simulation mode, the controller manager is provided by the Gazebo integration. In live mode, the base launch starts the standard `controller_manager/ros2_control_node`.

Main launch arguments are:

| Argument | Description |
|---|---|
| `mode` | execution mode, usually `simulation_gazebo` for this test setup |
| `joystick_model` | joystick model used to select the default joystick configuration, such as `microsoft_xbox` or `sony_dualshock4` |

The following diagram gives an overview of the control pipeline started by this test launch file.

![Ceol test pipeline](doc/test_pipeline.jpg)

## 4) Configuration files

The `config/` directory contains:

| File | Purpose |
|---|---|
| `controller_manager.yaml` | declares `joint_state_broadcaster` and `MobileBaseEnhancedController2TD` |
| `mobile_base_controller_live.yaml` | provides common runtime parameters for the live mobile base controller |
| `mobile_base_controller_simulation.yaml` | provides common runtime parameters for the simulation mobile base controller |

The robot geometry, inertia, joint names and teleoperation defaults are stored in `ceol_description/config/`.

## 5) Relation with the meta-bringup workflow

`ceol_bringup` is the robot-specific extension used when a mobile base meta-description selects:

```yaml
configuration:
  manufacturer: agreenculture
  model: ceol
  version: ""
```

In that workflow:

* `romea_mobile_base_meta_bringup` reads the mobile base meta-description;
* `ceol_bringup` generates Ceol-specific configuration, URDF, `ros2_control` and launch artifacts;
* `ceol_description` provides the concrete robot model;
* `ceol_hardware` is used in `live` mode;
* `romea_mobile_base_gazebo` or `romea_mobile_base_gazebo_classic` is used in Gazebo simulation modes;
* `romea_mobile_base_teleop` starts the matching skid-steering teleoperation node.
