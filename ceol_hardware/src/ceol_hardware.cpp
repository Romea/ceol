// Copyright 2023 Agreenculture
// Copyright 2023 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// std
#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

// ros
#include "rclcpp/rclcpp.hpp"

// ceol
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

const uint32_t IMPLEMENT_LEFT_ACTUATOR_MEASUREMENT_ID = 0x18FF00C7;
const uint32_t IMPLEMENT_RIGHT_ACTUATOR_MEASUREMENT_ID = 0x18FF00C6;
const uint32_t IMPLEMENT_LEFT_ACTUATOR_COMMAND_ID = 0x18EFC781;
const uint32_t IMPLEMENT_RIGHT_ACTUATOR_COMMAND_ID = 0x18EFC681;
const uint16_t IMPLEMENT_LOWEST_ACTUATORS_POSITION_ = 1050;
const uint16_t IMPLEMENT_HIGHEST_ACTUATORS_POSITION_ = 1;
const uint16_t IMPLEMENT_POSITION_MSG_MAXIMAL_VALUE = 10000;

const std::chrono::milliseconds TIMEOUT(200);

}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
CeolHardware::CeolHardware()
: HardwareSystemInterface2THD("CeolHardware"),
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
  speed_limitation_(false),
  desired_implement_position_(std::numeric_limits<uint16_t>::quiet_NaN())
{
#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif
}

//-----------------------------------------------------------------------------
CeolHardware::~CeolHardware()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
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
  std::cout << "load_info_ " << std::endl;
  try {
    // double sprocket_wheel_radius = get_parameter<double>(hardware_info, "sprocket_wheel_radius");
    // double track_thickness = get_parameter<double>(hardware_info, "track_thickness");
    double sprocket_wheel_radius = get_sprocket_wheel_radius(hardware_info);
    double track_thickness = get_track_thickness(hardware_info);
    virtual_sprocket_wheel_radius_ = sprocket_wheel_radius + track_thickness;

    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CeolHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CeolHardware::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (
    hardware_interface::SystemInterface::on_init(hardware_info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (
    this->load_info_(hardware_info) == hardware_interface::return_type::OK &&
    this->load_interface_(hardware_info) == hardware_interface::return_type::OK) {
    std::string ns = "_" + hardware_info.name;
    std::replace(ns.begin(), ns.end(), '_', '/');
    std::cout << " hardware node_name " << ns << std::endl;
    node_ = rclcpp::Node::make_shared("hardware", ns);

    imu_frame_id_ = hardware_info.name + "_imu_link";

    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(ns + "/imu/data", sensor_data_qos());

    auto callback =
      std::bind(&CeolHardware::implement_position_callback_, this, std::placeholders::_1);

    implement_position_sub_ = node_->create_subscription<ImplementPositionMsg>(
      ns + "/implement_position", best_effort(1), callback);

    // RCLCPP_INFO_STREAM(
    //   rclcpp::get_logger("CeolHardware"),
    //   implement_position_sub_->get_topic_name()
    //     << " " << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
#endif
{
  // RCLCPP_INFO(rclcpp::get_logger("CeolHardware"), "Read data from robot");

  // rclcpp::spin_some(node_);
  set_hardware_state_();

  std::cout << "spocket wheels speeds " << left_sprocket_wheel_angular_speed_measure_.load() << " "
            << right_sprocket_wheel_angular_speed_measure_.load() << std::endl;

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type CeolHardware::write()
#else
hardware_interface::return_type CeolHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
#endif
{
  // RCLCPP_INFO(rclcpp::get_logger("CeolHardware"), "Send command to robot");
  get_hardware_command_();
  send_command_();

#ifndef NDEBUG
  write_log_data_();
#endif

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void CeolHardware::get_hardware_command_()
{
  core::HardwareCommand2TD command = hardware_interface_->get_hardware_command();
  left_sprocket_wheel_angular_speed_command_ = command.leftSprocketWheelSpinningSetPoint;
  right_sprocket_wheel_angular_speed_command_ = command.rightSprocketWheelSpinningSetPoint;
}

//-----------------------------------------------------------------------------
void CeolHardware::set_hardware_state_()
{
  core::HardwareState2TD state;
  state.leftSprocketWheelSpinningMotion.velocity = left_sprocket_wheel_angular_speed_measure_;
  state.leftSprocketWheelSpinningMotion.torque = left_sprocket_wheel_torque_measure_;
  state.rightSprocketWheelSpinningMotion.velocity = right_sprocket_wheel_angular_speed_measure_;
  state.rightSprocketWheelSpinningMotion.torque = right_sprocket_wheel_torque_measure_;
  hardware_interface_->set_feedback(state);
}

//-----------------------------------------------------------------------------
void CeolHardware::receive_data_()
{
  while (rclcpp::ok() && can_receiver_thread_run_) {
    try {
      drivers::socketcan::CanId receive_id =
        can_receiver_.receive(received_frame_data_.data(), TIMEOUT);

      static auto previous_time = std::chrono::system_clock::now();
      static std::size_t count = 0;
      auto current_time = std::chrono::system_clock::now();
      count++;
      if (std::chrono::duration_cast<std::chrono::seconds>(current_time - previous_time).count()) {
        //RCLCPP_INFO(node_->get_logger(), "CAN freq: %4lu Hz", count);
        count = 0;
        previous_time = current_time;
      }

      switch (receive_id.identifier()) {
        case LEFT_SPROCKET_WHEEL_MEASUREMENTS_ID:
          decode_left_track_measurements_();
          break;
        case RIGHT_SPROCKET_WHEEL_MEASUREMENTS_ID:
          decode_right_track_measurements_();
          break;
        case IMPLEMENT_RIGHT_ACTUATOR_MEASUREMENT_ID:
          control_implement_actuator_position_(IMPLEMENT_RIGHT_ACTUATOR_COMMAND_ID);
          break;
        case IMPLEMENT_LEFT_ACTUATOR_MEASUREMENT_ID:
          control_implement_actuator_position_(IMPLEMENT_LEFT_ACTUATOR_COMMAND_ID);
          break;
        case IMU_ACCELERATION_X_MEASUREMENT_ID:
          // RCLCPP_INFO_STREAM(node_->get_logger(), "ax-axis");
          decode_imu_acceleration_(imu_acceleration_x_stamp_, imu_acceleration_x_measure_);
          break;
        case IMU_ACCELERATION_Y_MEASUREMENT_ID:
          // RCLCPP_INFO_STREAM(node_->get_logger(), "ay-axis");
          decode_imu_acceleration_(imu_acceleration_y_stamp_, imu_acceleration_y_measure_);
          break;
        case IMU_ACCELERATION_Z_MEASUREMENT_ID:
          // RCLCPP_INFO_STREAM(node_->get_logger(), "az-axis");
          decode_imu_acceleration_(imu_acceleration_z_stamp_, imu_acceleration_z_measure_);
          break;
        case IMU_ANGULAR_SPEED_X_MEASUREMENT_ID:
          // RCLCPP_INFO_STREAM(node_->get_logger(), "sx-axis");
          decode_imu_angular_speed_(
            imu_angular_speed_x_stamp_, imu_angular_speed_x_measure_, imu_angle_x_measure_);
          break;
        case IMU_ANGULAR_SPEED_Y_MEASUREMENT_ID:
          // RCLCPP_INFO_STREAM(node_->get_logger(), "sy-axis");
          decode_imu_angular_speed_(
            imu_angular_speed_y_stamp_, imu_angular_speed_y_measure_, imu_angle_y_measure_);
          break;
        case IMU_ANGULAR_SPEED_Z_MEASUREMENT_ID:
          // RCLCPP_INFO_STREAM(node_->get_logger(), "sz-axis");
          decode_imu_angular_speed_(
            imu_angular_speed_z_stamp_, imu_angular_speed_z_measure_, imu_angle_z_measure_);
          break;
        case ENS_CONTROL_ID:
          ens_control_callback_();
          break;
        default:
          break;
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
    left_sprocket_wheel_angular_speed_command_, right_sprocket_wheel_angular_speed_command_);
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
bool CeolHardware::send_data_(uint32_t id, uint32_t length, bool extended)
{
  try {
    drivers::socketcan::CanId can_id(id, 0, length);
    if (extended) {
      can_sender_.send(sended_frame_data_.data(), length, can_id.extended(), TIMEOUT);
    } else {
      can_sender_.send(sended_frame_data_.data(), length, can_id, TIMEOUT);
    }
    return true;
  } catch (drivers::socketcan::SocketCanTimeout & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("CeolHardware"), "Send can data" << std::hex << id << " : timeout");
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("CeolHardware"), "Send can data" << std::hex << id << " : " << e.what());
  }
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("CeolHardware"), "Send data failed");

  return false;
}

//----------------------------------------------------------------------------
void CeolHardware::decode_left_track_measurements_()
{
  //  BO_ 390 InverterLeftDrivingStatus: 8 INVERTER_LEFT
  //  SG_ InverterLeftStatusWord : 0|16@1+ (1,0) [0|65535] ""  EMBEDDED_CONTROLLER
  //  SG_ InverterLeftVelocityRPM : 16|32@1- (1,0) [-2147483648|2147483647] "RPM"  EMBEDDED_CONTROLLER // NOLINT
  //  SG_ InverterLeftTorque : 48|16@1- (0.1,0) [-32768|32767] "% of peak"  EMBEDDED_CONTROLLER  // % of peak = 25Nm  // NOLINT
  left_sprocket_wheel_angular_speed_measure_ =
    -static_cast<float>(static_cast<int32_t>(
      (received_frame_data_[5] << 24) + (received_frame_data_[4] << 16) +
      (received_frame_data_[3] << 8) + received_frame_data_[2])) /
    2736.61 / virtual_sprocket_wheel_radius_;

  left_sprocket_wheel_torque_measure_ =
    -static_cast<float>(
      static_cast<int16_t>((received_frame_data_[7] << 8) + received_frame_data_[6])) *
    25 / 1000;
}

//-----------------------------------------------------------------------------
void CeolHardware::decode_right_track_measurements_()
{
  //  BO_ 391 InverterRightDrivingStatus: 8 INVERTER_RIGHT
  //  SG_ InverterRightTorque : 48|16@1- (0.1,0) [-32768|32767] "% of peak"  EMBEDDED_CONTROLLER // % of peak = 25Nm // NOLINT
  //  SG_ InverterRightVelocityRPM : 16|32@1- (1,0) [-2147483648|2147483647] "RPM"  EMBEDDED_CONTROLLER // NOLINT
  //  SG_ InverterRightStatusWord : 0|16@1+ (1,0) [0|65535] ""  EMBEDDED_CONTROLLER
  right_sprocket_wheel_angular_speed_measure_ =
    static_cast<float>(static_cast<int32_t>(
      (received_frame_data_[5] << 24) + (received_frame_data_[4] << 16) +
      (received_frame_data_[3] << 8) + received_frame_data_[2])) /
    2736.61 / virtual_sprocket_wheel_radius_;

  right_sprocket_wheel_torque_measure_ =
    static_cast<float>(
      static_cast<int16_t>((received_frame_data_[7] << 8) + received_frame_data_[6])) *
    25 / 1000;
}

//-----------------------------------------------------------------------------
void CeolHardware::decode_imu_acceleration_(uint32_t & stamp, float & acceleration)
{
  //   SG_ IMUAcc : 48|16@1- (1,0) [-32768|32767] "mm/s"  EMBEDDED_CONTROLLER
  //   SG_ IMUDV : 32|16@1- (1,0) [-32768|32767] "mm/s"  EMBEDDED_CONTROLLER
  //   SG_ IMUTimestamp : 0|32@1+ (1,0) [0|4294967295] "ms"  EMBEDDED_CONTROLLER
  stamp = ((received_frame_data_[3] & 0xFF) << 24) + ((received_frame_data_[2] & 0xFF) << 16) +
          ((received_frame_data_[1] & 0xFF) << 8) + received_frame_data_[0];

  acceleration = static_cast<float>(
                   static_cast<int16_t>((received_frame_data_[7] << 8) + received_frame_data_[6])) /
                 1000.;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "a " << " " << stamp);

  try_publish_imu_data_(stamp);
}

//-----------------------------------------------------------------------------
void CeolHardware::decode_imu_angular_speed_(uint32_t & stamp, float & angular_speed, float & angle)
{
  //  SG_ IMURot : 48|16@1- (0.0001,0) [-3.2768|3.2767] "rad/s"  EMBEDDED_CONTROLLER
  //   SG_ IMUAngle: 32|16@1- (0.0001,0) [-3.2768|3.2767] "radian"  EMBEDDED_CONTROLLER
  //   SG_ IMUTimestamp : 0|32@1+ (1,0) [0|4294967295] "ms"  EMBEDDED_CONTROLLER
  stamp = ((received_frame_data_[3] & 0xFF) << 24) + ((received_frame_data_[2] & 0xFF) << 16) +
          ((received_frame_data_[1] & 0xFF) << 8) + received_frame_data_[0];

  angular_speed = static_cast<float>(static_cast<int16_t>(
                    (received_frame_data_[7] << 8) + received_frame_data_[6])) /
                  10000.;

  angle = static_cast<float>(
            static_cast<int16_t>((received_frame_data_[5] << 8) + received_frame_data_[4])) /
          10000.;

  // RCLCPP_INFO_STREAM(node_->get_logger(), "w " << " " << stamp);

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

  // std::cout << "go_to_auto" << go_to_auto << " " << "MotionAuthorized" << motion_authorized <<
  //   " speed limitation" << speed_limitation_ << std::endl;
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
void CeolHardware::try_publish_imu_data_(const uint32_t & /* stamp */)
{
  {
    std::lock_guard<std::mutex> guard(imu_mutex_);

    // if (imu_acceleration_x_stamp_ == stamp &&
    //   imu_acceleration_y_stamp_ == stamp &&
    //   imu_acceleration_z_stamp_ == stamp &&
    //   imu_angular_speed_x_stamp_ == stamp &&
    //   imu_angular_speed_y_stamp_ == stamp &&
    //   imu_angular_speed_z_stamp_ == stamp)
    // {
    if (
      imu_acceleration_y_stamp_ == imu_acceleration_x_stamp_ &&
      imu_acceleration_z_stamp_ == imu_acceleration_x_stamp_ &&
      imu_angular_speed_y_stamp_ == imu_angular_speed_x_stamp_ &&
      imu_angular_speed_z_stamp_ == imu_angular_speed_x_stamp_) {
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
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = imu_frame_id_;
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
  rclcpp::spin_some(node_);
  // }
}

//-----------------------------------------------------------------------------
void CeolHardware::implement_position_callback_(ImplementPositionMsg::ConstSharedPtr msg)
{
  double percentage = std::min(msg->position, IMPLEMENT_POSITION_MSG_MAXIMAL_VALUE) / 10000.;

  uint16_t desired_implement_position = static_cast<uint16_t>(
    percentage * IMPLEMENT_HIGHEST_ACTUATORS_POSITION_ +
    (1 - percentage) * IMPLEMENT_LOWEST_ACTUATORS_POSITION_);

  if (desired_implement_position != desired_implement_position_.load() && caius_auto_) {
    start_implement_actuator_control_(IMPLEMENT_LEFT_ACTUATOR_COMMAND_ID);
    start_implement_actuator_control_(IMPLEMENT_RIGHT_ACTUATOR_COMMAND_ID);
  }

  desired_implement_position_.store(desired_implement_position);
}

//-----------------------------------------------------------------------------
void CeolHardware::start_implement_actuator_control_(uint32_t actuator_id)
{
  sended_frame_data_[0] = 0x00;
  sended_frame_data_[1] = 0xFB;
  sended_frame_data_[2] = 0x50;
  sended_frame_data_[3] = 0xC8;
  sended_frame_data_[4] = 0x00;
  sended_frame_data_[5] = 0x00;
  sended_frame_data_[6] = 0x00;
  sended_frame_data_[7] = 0x00;
  send_data_(actuator_id, 8, true);
}

//-----------------------------------------------------------------------------
void CeolHardware::control_implement_actuator_position_(uint32_t actuator_id)
{
  //  BO_ 2565850753 ActuatorLeft_Cmd: 8 ACTUATOR_LEFT
  //  SG_ Cmd_ActuatorLeft_Position : 0|16@1+ (0.1,0) [0|65535] "mm per bit" Vector__XXX
  //  SG_ Cmd_ActuatorLeft_MaxCurrent : 16|8@1+ (0.25,0) [0|255] "250mA per bit" Vector__XXX
  //  SG_ Cmd_ActuatorLeft_Speed : 24|8@1+ (0.5,0) [0|255] "0.5 percent per bit" Vector__XXX
  //  SG_ Cmd_ActuatorLeft_StartRamp : 32|8@1+ (0.05,0) [0|255] "50ms per bit" Vector__XXX
  //  SG_ Cmd_ActuatorLeft_EndRamp : 40|8@1+ (0.05,0) [0|255] "50ms per bit" Vector__XXX
  uint16_t actuator_position_measure =
    static_cast<uint16_t>(((received_frame_data_[1] << 8) + received_frame_data_[0]));

  auto desired_implement_position = desired_implement_position_.load();

  if (std::isfinite(desired_implement_position) && caius_auto_) {
    if (std::abs(desired_implement_position - actuator_position_measure) > 10) {
      sended_frame_data_[0] = desired_implement_position & 0xFF;
      sended_frame_data_[1] = (desired_implement_position >> 8) & 0xFF;
      sended_frame_data_[2] = 0x50;
      sended_frame_data_[3] = 0xC8;
      sended_frame_data_[4] = 0x00;
      sended_frame_data_[5] = 0x00;
      sended_frame_data_[6] = 0x00;
      sended_frame_data_[7] = 0x00;
      send_data_(actuator_id, 8, true);
    }
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

#ifndef NDEBUG
//-----------------------------------------------------------------------------
void CeolHardware::open_log_file_()
{
  debug_file_.open(
    std::string("ceol.dat").c_str(), std::fstream::in | std::fstream::out | std::fstream::trunc);
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

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::CeolHardware, hardware_interface::SystemInterface)
