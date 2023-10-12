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


#ifndef CEOL_HARDWARE__CEOL_HARDWARE_HPP_
#define CEOL_HARDWARE__CEOL_HARDWARE_HPP_

// std
#include <array>
#include <atomic>
#include <memory>
#include <thread>
#include <fstream>


// ros
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/macros.hpp"

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"


namespace romea
{

class CeolHardware : public HardwareSystemInterface2THD
{
public:
  using ImuMsg = sensor_msgs::msg::Imu;
  using ImplementPositionMsg = ceol_msgs::msg::ImplementPosition;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CeolHardware);

  CeolHardware();
  #if ROS_DISTRO == ROS_GALACTIC
  hardware_interface::return_type read()override;

  hardware_interface::return_type write()override;
#else
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)override;
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

private:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;


  bool send_data_(uint32_t id, uint32_t length, bool extended = false);

  bool send_command_(
    const float & left_sprocket_angular_speed_command,
    const float & right_sprocket_angular_speed_command
  );

  bool send_null_command_();

  bool send_command_();

  void send_activate_auto_mode_();

  void send_deactivate_auto_mode_();

  void send_nmt_();


  void receive_data_();

  void decode_left_track_measurements_();

  void decode_right_track_measurements_();

  void decode_imu_acceleration_(
    uint32_t & stamp,
    float & acceleration);

  void decode_imu_angular_speed_(
    uint32_t & stamp,
    float & angular_speed,
    float & angle);

  void ens_control_callback_();


  void try_publish_imu_data_(const uint32_t & stamp);

  // bool is_drive_enable_() const;


  void start_can_receiver_thread_();

  void stop_can_receiver_thread_();

  void get_hardware_command_();

  void set_hardware_state_();

#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

private:
  std::unique_ptr<std::thread> can_receiver_thread_;
  std::atomic<bool> can_receiver_thread_run_;

  drivers::socketcan::SocketCanSender can_sender_;
  drivers::socketcan::SocketCanReceiver can_receiver_;
  std::array<uint8_t, 16> sended_frame_data_;
  std::array<uint8_t, 16> received_frame_data_;

  float virtual_sprocket_wheel_radius_;
  float left_sprocket_wheel_angular_speed_command_;
  float right_sprocket_wheel_angular_speed_command_;

  std::atomic<float> left_sprocket_wheel_angular_speed_measure_;
  std::atomic<float> right_sprocket_wheel_angular_speed_measure_;
  std::atomic<float> left_sprocket_wheel_torque_measure_;
  std::atomic<float> right_sprocket_wheel_torque_measure_;

  bool caius_auto_;
  bool pending_activation_;
  bool speed_limitation_;

  rclcpp::Node::SharedPtr node_;

  std::mutex imu_mutex_;
  std::string imu_frame_id_;
  uint32_t imu_acceleration_x_stamp_;
  float imu_acceleration_x_measure_;
  uint32_t imu_acceleration_y_stamp_;
  float imu_acceleration_y_measure_;
  uint32_t imu_acceleration_z_stamp_;
  float imu_acceleration_z_measure_;
  uint32_t imu_angular_speed_x_stamp_;
  float imu_angular_speed_x_measure_;
  float imu_angle_x_measure_;
  uint32_t imu_angular_speed_y_stamp_;
  float imu_angular_speed_y_measure_;
  float imu_angle_y_measure_;
  uint32_t imu_angular_speed_z_stamp_;
  float imu_angular_speed_z_measure_;
  float imu_angle_z_measure_;
  rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub_;

  std::atomic<uint16_t> desired_implement_position_;
  void start_implement_actuator_control_(uint32_t actuator_id);
  void control_implement_actuator_position_(uint32_t actuator_id);
  void implement_position_callback_(ImplementPositionMsg::ConstSharedPtr msg);
  rclcpp::Subscription<ImplementPositionMsg>::SharedPtr implement_position_sub_;


#ifndef NDEBUG
  std::fstream debug_file_;
#endif
};

}  // namespace romea

#endif  // CEOL_HARDWARE__CEOL_HARDWARE_HPP_
