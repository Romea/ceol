// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <memory>
#include <thread>
#include <string>
#include <fstream>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

// ros
#include "rclcpp/rclcpp.hpp"


// local
#include "ceol_hardware/ceol_hardware.hpp"

namespace
{
const double WHEEL_LINEAR_SPEED_EPSILON = 0.01;
const double WHEEL_STEERING_ANGLE_EPSILON = 0.03;

const uint32_t LEFT_SPROCKET_WHEEL_MEASUREMENTS_ID = 0x186;
const uint32_t RIGHT_SPROCKET_WHEEL_MEASUREMENTS_ID = 0x187;
const uint32_t IMU_ACCELERATION_X_MEASUREMENT_ID = 0x30F;
const uint32_t IMU_ACCELERATION_Y_MEASUREMENT_ID = 0x40F;
const uint32_t IMU_ACCELERATION_Z_MEASUREMENT_ID = 0x50F;
const uint32_t IMU_ANGULAR_SPEED_X_MEASUREMENT_ID = 0x48F;
const uint32_t IMU_ANGULAR_SPEED_Y_MEASUREMENT_ID = 0x58F;
const uint32_t IMU_ANGULAR_SPEED_Z_MEASUREMENT_ID = 0x20F;
const uint32_t ENS_CONTROL_ID = 0x211;

const int LOWEST_TOOL_VALUE_ = 1050;  // Valeur de position basse de l'outil
const int HIGHEST_TOOL_VALUE_ = 1;  // Valeur de position haute de l'outil


const std::chrono::milliseconds TIMEOUT(5);

}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
CeolHardware::CeolHardware()
: HardwareSystemInterface2TD(),
  can_receiver_thread_(nullptr),
  can_receiver_thread_run_(false),
  can_sender_("can0"),
  can_receiver_("can0"),
  sended_frame_data_(),
  received_frame_data_(),
  virtual_sprocket_wheel_radius_(0),
  left_sprocket_wheel_angular_speed_command_(0),
  right_sprocket_wheel_angular_speed_command_(0),
  left_sprocket_wheel_angular_speed_measure_(0),
  right_sprocket_wheel_angular_speed_measure_(0),
  left_sprocket_wheel_torque_measure_(0),
  right_sprocket_wheel_torque_measure_(0),
  caius_auto_(false),
  pending_activation_(false),
  speed_limitation_(false)
{
#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CeolHardware::connect_()
{
  RCLCPP_INFO(rclcpp::get_logger("CeolHardware"), "Init communication with robot");

  send_null_command_();
  start_can_receiver_thread_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type CeolHardware::disconnect_()
{
  RCLCPP_INFO(rclcpp::get_logger("CeolHardware"), "Close communication with robot");

  send_null_command_();
  stop_can_receiver_thread_();
  return hardware_interface::return_type::OK;
}
//-----------------------------------------------------------------------------
hardware_interface::return_type CeolHardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    double sprocket_wheel_radius = get_parameter<double>(hardware_info, "sprocket_wheel_radius");
    double track_thickness = get_parameter<double>(hardware_info, "track_thickness");
    virtual_sprocket_wheel_radius_ = sprocket_wheel_radius + track_thickness;
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CeolHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CeolHardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (this->load_info_(hardware_info) == hardware_interface::return_type::OK &&
    this->load_interface_(hardware_info) == hardware_interface::return_type::OK)
  {
    std::string ns = "_" + hardware_info.name;
    std::replace(ns.begin(), ns.end(), '_', '/');
    std::cout << " hardware node_name " << ns << std::endl;
    node_ = rclcpp::Node::make_shared("hardware", ns);

    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      ns + "/imu/data", sensor_data_qos());
    // joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    //   ns + "/bridge/vehicle_controller/joint_states", best_effort(1), callback);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
}


//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type CeolHardware::read()
#else
hardware_interface::return_type CeolHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  RCLCPP_INFO(rclcpp::get_logger("CeolHardware"), "Read data from robot");

  set_hardware_state_();

  std::cout << "spocket wheels speeds " <<
    left_sprocket_wheel_angular_speed_measure_.load() << " " <<
    right_sprocket_wheel_angular_speed_measure_.load() << std::endl;

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type CeolHardware::write()
#else
hardware_interface::return_type CeolHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  RCLCPP_INFO(rclcpp::get_logger("CeolHardware"), "Send command to robot");
  return hardware_interface::return_type::OK;

  get_hardware_command_();

  // if (is_drive_enable_()) {
  //   std::cout << " send command " << std::endl;
  //   send_command_();

  //   std::cout << "sprocket wheels speeds command " <<
  //     left_sprocket_wheel_angular_speed_command_ << " " <<
  //     right_sprocket_wheel_angular_speed_command_ << std::endl;

  // }

#ifndef NDEBUG
  write_log_data_();
#endif

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void CeolHardware::get_hardware_command_()
{
  HardwareCommand2TD command = hardware_interface_->get_command();
  left_sprocket_wheel_angular_speed_command_ = command.leftSprocketWheelSpinningSetPoint;
  right_sprocket_wheel_angular_speed_command_ = command.rightSprocketWheelSpinningSetPoint;

}

//-----------------------------------------------------------------------------
void CeolHardware::set_hardware_state_()
{
  HardwareState2TD state;
  state.leftSprocketWheelSpinningMotion.velocity = left_sprocket_wheel_angular_speed_measure_;
  state.leftSprocketWheelSpinningMotion.torque = left_sprocket_wheel_torque_measure_;
  state.rightSprocketWheelSpinningMotion.velocity = right_sprocket_wheel_angular_speed_measure_;
  state.rightSprocketWheelSpinningMotion.torque = right_sprocket_wheel_torque_measure_;
  hardware_interface_->set_state(state);
}


//-----------------------------------------------------------------------------
void CeolHardware::receive_data_()
{
  while (rclcpp::ok() && can_receiver_thread_run_) {
    try {
      drivers::socketcan::CanId receive_id = can_receiver_.
        receive(received_frame_data_.data(), TIMEOUT);

      switch (receive_id.identifier()) {
        case LEFT_SPROCKET_WHEEL_MEASUREMENTS_ID:
          decode_left_track_measurements_();
          break;
        case RIGHT_SPROCKET_WHEEL_MEASUREMENTS_ID:
          decode_left_track_measurements_();
          break;
        case IMU_ACCELERATION_X_MEASUREMENT_ID:
          decode_imu_acceleration_(imu_acceleration_x_stamp_, imu_acceleration_x_measure_);
          break;
        case IMU_ACCELERATION_Y_MEASUREMENT_ID:
          decode_imu_acceleration_(imu_acceleration_y_stamp_, imu_acceleration_y_measure_);
          break;
        case IMU_ACCELERATION_Z_MEASUREMENT_ID:
          decode_imu_acceleration_(imu_acceleration_z_stamp_, imu_acceleration_z_measure_);
          break;
        case IMU_ANGULAR_SPEED_X_MEASUREMENT_ID:
          decode_imu_angular_speed_(
            imu_angular_speed_x_stamp_, imu_angular_speed_x_measure_, imu_angle_x_measure_);
          break;
        case IMU_ANGULAR_SPEED_Y_MEASUREMENT_ID:
          decode_imu_angular_speed_(
            imu_angular_speed_y_stamp_, imu_angular_speed_y_measure_, imu_angle_y_measure_);
          break;
        case IMU_ANGULAR_SPEED_Z_MEASUREMENT_ID:
          decode_imu_angular_speed_(
            imu_angular_speed_y_stamp_, imu_angular_speed_y_measure_, imu_angle_y_measure_);
          break;
        case ENS_CONTROL_ID:
          ens_control_callback_();
          break;
          // case 0x18FF00C6:
          //   callbackRightToolActuator_(frame);
          //   break;
          // case 0x18FF00C7:
          //   callbackLeftToolActuator_(frame);
          //   break;
          // default:
          // break;
      }

    } catch (const std::exception & ex) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("CeolHardware"), "Error receiving CAN message");
    }
  }
}

//-----------------------------------------------------------------------------
bool CeolHardware::send_command_()
{
  return send_command_(
    left_sprocket_wheel_angular_speed_command_,
    right_sprocket_wheel_angular_speed_command_);
}

//-----------------------------------------------------------------------------
bool CeolHardware::send_null_command_()
{
  return send_command_(0., 0.);
}

//-----------------------------------------------------------------------------
bool CeolHardware::send_command_(
  const float & left_sprocket_angular_speed_command,
  const float & right_sprocket_angular_speed_command)
{
  //  BO_ 657 SteeringCommands: 5 AGC_ENS
  //  SG_ GuidanceRightSpeed : 16|16@1- (1,0) [-32768|32767] "mm/s"
  //  SG_ GuidanceLeftSpeed : 0|16@1- (1,0) [-32768|32767] "mm/s"
  //  SG_ SafetyCounterENS : 32|8@1+ (1,0) [0|255] ""
  int16_t int_left_track_speed = static_cast<int16_t>(
    left_sprocket_angular_speed_command * virtual_sprocket_wheel_radius_ * 1000);
  int16_t int_right_track_speed = static_cast<int16_t>(
    right_sprocket_angular_speed_command * virtual_sprocket_wheel_radius_ * 1000);

  sended_frame_data_[0] = int_left_track_speed & 0xFF;
  sended_frame_data_[1] = (int_left_track_speed >> 8) & 0xFF;
  sended_frame_data_[2] = int_right_track_speed & 0xFF;
  sended_frame_data_[3] = (int_right_track_speed >> 8) & 0xFF;
  return send_data_(657, 4);
}


//-----------------------------------------------------------------------------
bool CeolHardware::send_data_(uint32_t id, uint32_t length)
{
  try {
    drivers::socketcan::CanId can_id(id, 0, length);
    can_sender_.send(sended_frame_data_.data(), length, can_id, TIMEOUT);
    return true;
  } catch (drivers::socketcan::SocketCanTimeout & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("CeolHardware"),
      "Send can data" << std::hex << id << " : timeout");
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("CeolHardware"),
      "Send can data" << std::hex << id << " : " << e.what());
  }
  return false;
}


//----------------------------------------------------------------------------
void CeolHardware::decode_left_track_measurements_()
{
//  BO_ 390 InverterLeftDrivingStatus: 8 INVERTER_LEFT
//  SG_ InverterLeftStatusWord : 0|16@1+ (1,0) [0|65535] ""  EMBEDDED_CONTROLLER
//  SG_ InverterLeftVelocityRPM : 16|32@1- (1,0) [-2147483648|2147483647] "RPM"  EMBEDDED_CONTROLLER
//  SG_ InverterLeftTorque : 48|16@1- (0.1,0) [-32768|32767] "% of peak"  EMBEDDED_CONTROLLER  // % of peak = 25Nm
  left_sprocket_wheel_angular_speed_measure_ = -static_cast<float>(static_cast<int32_t>(
      (received_frame_data_[5] << 24) + (received_frame_data_[4] << 16) +
      (received_frame_data_[3] << 8) + received_frame_data_[2])) / 2736.61;

  left_sprocket_wheel_torque_measure_ = -static_cast<float>(static_cast<int16_t>(
      (received_frame_data_[7] << 8) + received_frame_data_[6])) * 25 / 1000;

}

//-----------------------------------------------------------------------------
void CeolHardware::decode_right_track_measurements_()
{
//  BO_ 391 InverterRightDrivingStatus: 8 INVERTER_RIGHT
//  SG_ InverterRightTorque : 48|16@1- (0.1,0) [-32768|32767] "% of peak"  EMBEDDED_CONTROLLER // % of peak = 25Nm
//  SG_ InverterRightVelocityRPM : 16|32@1- (1,0) [-2147483648|2147483647] "RPM"  EMBEDDED_CONTROLLER
//  SG_ InverterRightStatusWord : 0|16@1+ (1,0) [0|65535] ""  EMBEDDED_CONTROLLER
  right_sprocket_wheel_angular_speed_measure_ = static_cast<float>(static_cast<int32_t>(
      (received_frame_data_[5] << 24) + (received_frame_data_[4] << 16) +
      (received_frame_data_[3] << 8) + received_frame_data_[2])) / 2736.61;

  right_sprocket_wheel_torque_measure_ = static_cast<float>(static_cast<int16_t>(
      (received_frame_data_[7] << 8) + received_frame_data_[6])) * 25 / 1000;
}

//-----------------------------------------------------------------------------
void CeolHardware::decode_imu_acceleration_(
  uint32_t & stamp,
  float & acceleration)
{
//   SG_ IMUAcc : 48|16@1- (1,0) [-32768|32767] "mm/s"  EMBEDDED_CONTROLLER
//   SG_ IMUDV : 32|16@1- (1,0) [-32768|32767] "mm/s"  EMBEDDED_CONTROLLER
//   SG_ IMUTimestamp : 0|32@1+ (1,0) [0|4294967295] "ms"  EMBEDDED_CONTROLLER
  stamp = ((received_frame_data_[3] & 0xFF) << 24) + ((received_frame_data_[2] & 0xFF) << 16) +
    ((received_frame_data_[1] & 0xFF) << 8) + received_frame_data_[0];

  acceleration = static_cast<float>(static_cast<int16_t>(
      (received_frame_data_[7] << 8) + received_frame_data_[6])) / 1000.;

  try_publish_imu_data_(stamp);
}

//-----------------------------------------------------------------------------
void CeolHardware::decode_imu_angular_speed_(
  uint32_t & stamp,
  float & angular_speed,
  float & angle)
{
//  SG_ IMURot : 48|16@1- (0.0001,0) [-3.2768|3.2767] "rad/s"  EMBEDDED_CONTROLLER
//   SG_ IMUAngle: 32|16@1- (0.0001,0) [-3.2768|3.2767] "radian"  EMBEDDED_CONTROLLER
//   SG_ IMUTimestamp : 0|32@1+ (1,0) [0|4294967295] "ms"  EMBEDDED_CONTROLLER
  stamp = ((received_frame_data_[3] & 0xFF) << 24) + ((received_frame_data_[2] & 0xFF) << 16) +
    ((received_frame_data_[1] & 0xFF) << 8) + received_frame_data_[0];

  angular_speed = static_cast<float>(static_cast<int16_t>(
      (received_frame_data_[7] << 8) + received_frame_data_[6])) / 10000.;

  angle = static_cast<float>(static_cast<int16_t>(
      (received_frame_data_[5] << 8) + received_frame_data_[4])) / 10000.;

  try_publish_imu_data_(stamp);
}

//-----------------------------------------------------------------------------
void CeolHardware::ens_control_callback_()
{
  //  BO_ 529 ENSControl: 1
  //  SG_ MotionAuthorized : 0|1@1+ (1,0) [0|1] ""  AGC_ENS
  //  SG_ GoToAuto : 1|1@1+ (1,0) [0|1] ""  AGC_ENS
  //  SG_ SpeedLimitationENS : 2|1@1+ (1,0) [0|1] ""  AGC_ENS
  int go_to_auto = (received_frame_data_[0] & 0x02) >> 1;
  int motion_authorized = received_frame_data_[0] & 0x01;
  speed_limitation_ = (received_frame_data_[0] & 0x04) >> 2;
  send_nmt_();

  if (go_to_auto && !motion_authorized && !pending_activation_ && !caius_auto_) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "TRYING TO ENTER IN AUTO MODE");
    pending_activation_ = true;
    send_activate_auto_mode_();
  } else if (!go_to_auto && !motion_authorized && caius_auto_) {
    caius_auto_ = false;
    send_deactivate_auto_mode_();
    RCLCPP_WARN_STREAM(node_->get_logger(), "BACK TO MANUAL");
  } else if (go_to_auto && motion_authorized && !caius_auto_) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "AUTO MODE OK");
    caius_auto_ = true;
    pending_activation_ = false;
  }
}

//-----------------------------------------------------------------------------
void CeolHardware::send_activate_auto_mode_()
{
  using namespace std::chrono_literals;

  sended_frame_data_[0] = 0;
  sended_frame_data_[1] = 1;
  send_data_(401, 2);
  std::this_thread::sleep_for(500ms);

  sended_frame_data_[0] = 1;
  sended_frame_data_[1] = 1;
  send_data_(401, 2);
  std::this_thread::sleep_for(500ms);
}

//-----------------------------------------------------------------------------
void CeolHardware::send_deactivate_auto_mode_()
{
  send_null_command_();
  sended_frame_data_[0] = 0;
  sended_frame_data_[1] = 0;
  sended_frame_data_[2] = 2;
  send_data_(401, 3);
}

//-----------------------------------------------------------------------------
void CeolHardware::send_nmt_()
{
  sended_frame_data_[0] = 1;
  sended_frame_data_[1] = 15;
  send_data_(0, 2);
}


//-----------------------------------------------------------------------------
void CeolHardware::try_publish_imu_data_(const uint32_t & stamp)
{
  std::lock_guard<std::mutex> guard(imu_mutex_);
  if (imu_acceleration_x_stamp_ == stamp &&
    imu_acceleration_y_stamp_ == stamp &&
    imu_acceleration_y_stamp_ == stamp &&
    imu_angular_speed_x_stamp_ == stamp &&
    imu_angular_speed_y_stamp_ == stamp &&
    imu_angular_speed_z_stamp_ == stamp)
  {

    sensor_msgs::msg::Imu msg;
    double roll = imu_angle_x_measure_;
    double pitch = imu_angle_y_measure_;
    double yaw = imu_angle_z_measure_;
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "imu_link";
    msg.angular_velocity.x = imu_angular_speed_x_measure_;
    msg.angular_velocity.y = imu_angular_speed_y_measure_;
    msg.angular_velocity.z = imu_angular_speed_z_measure_;
    msg.linear_acceleration.x = imu_acceleration_x_measure_;
    msg.linear_acceleration.y = imu_acceleration_y_measure_;
    msg.linear_acceleration.z = imu_acceleration_z_measure_;
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;
    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_pub_->publish(msg);
  }
}


//-----------------------------------------------------------------------------
void CeolHardware::start_can_receiver_thread_()
{
  can_receiver_thread_run_ = true;
  can_receiver_thread_ = std::make_unique<std::thread>(&CeolHardware::receive_data_, this);
}

//-----------------------------------------------------------------------------
void CeolHardware::stop_can_receiver_thread_()
{
  can_receiver_thread_run_ = false;
  if (can_receiver_thread_->joinable()) {
    can_receiver_thread_->join();
  }
}

// //-----------------------------------------------------------------------------
// bool CeolHardware::is_drive_enable_() const
// {
//   float speed_measure = 0.25 * (front_left_wheel_linear_speed_measure_ +
//     front_right_wheel_linear_speed_measure_ +
//     rear_left_wheel_linear_speed_measure_ +
//     rear_right_wheel_linear_speed_measure_);

//   float speed_command = 0.25 * (front_left_wheel_linear_speed_command_ +
//     front_right_wheel_linear_speed_command_ +
//     rear_left_wheel_linear_speed_command_ +
//     rear_right_wheel_linear_speed_command_);

//   return !(std::abs(front_axle_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
//          std::abs(front_axle_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
//          std::abs(rear_axle_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
//          std::abs(rear_axle_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
//          std::abs(speed_measure) < WHEEL_LINEAR_SPEED_EPSILON &&
//          std::abs(speed_command) < WHEEL_LINEAR_SPEED_EPSILON);
// }

#ifndef NDEBUG
//-----------------------------------------------------------------------------
void CeolHardware::open_log_file_()
{
  debug_file_.open(
    std::string("ceol.dat").c_str(),
    std::fstream::in | std::fstream::out | std::fstream::trunc);
}
//-----------------------------------------------------------------------------
void CeolHardware::write_log_header_()
{
  if (debug_file_.is_open()) {
    debug_file_ << " time, ";
    debug_file_ << " LS, " << " RS, ";
    debug_file_ << " LT, " << " RT, ";
    debug_file_ << " LS_cmd, " << " RS_cmd " << "\n";
  }
}

//-----------------------------------------------------------------------------
void CeolHardware::write_log_data_()
{
  if (debug_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count();
    debug_file_ << left_sprocket_wheel_angular_speed_measure_ << " ";
    debug_file_ << right_sprocket_wheel_angular_speed_measure_ << " ";
    debug_file_ << left_sprocket_wheel_torque_measure_ << " ";
    debug_file_ << right_sprocket_wheel_torque_measure_ << " ";
    debug_file_ << left_sprocket_wheel_angular_speed_command_ << " ";
    debug_file_ << right_sprocket_wheel_angular_speed_command_ << " \n";
  }
}
#endif

}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::CeolHardware, hardware_interface::SystemInterface)
