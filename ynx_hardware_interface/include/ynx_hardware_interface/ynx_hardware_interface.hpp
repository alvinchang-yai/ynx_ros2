#ifndef YNX_HARDWARE_INTERFACE__YNX_HARDWARE_INTERFACE_HPP_
#define YNX_HARDWARE_INTERFACE__YNX_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <grpcpp/grpcpp.h>

// Include your generated gRPC headers here
#include "rcs/v1/alarm_api.grpc.pb.h"
#include "rcs/v1/monitor_api.grpc.pb.h"
#include "rcs/v1/motion_api.grpc.pb.h"
// #include "rcs/v1/event_api.grpc.pb.h"
// #include "rcs/v1/file_api.grpc.pb.h"
// #include "rcs/v1/io_api.grpc.pb.h"
// #include "rcs/v1/job_control.grpc.pb.h"
// #include "rcs/v1/mode_get.grpc.pb.h"
// #include "rcs/v1/position_types.grpc.pb.h"
#include "rcs/v1/system_info.grpc.pb.h"
// #include "rcs/v1/timestamp.grpc.pb.h"
// #include "rcs/v1/variable_api.grpc.pb.h"

namespace ynx_hardware_interface
{

class YnxHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(YnxHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- Parameters ---
  std::string ip_;
  std::string port_;
  int group_no_ = 0;
  int task_no_ = -1;
  uint32_t control_group_bit_ = 1;

  // --- State and Command variables ---
  std::vector<double> position_commands_;
  std::vector<double> previous_position_commands_;
  std::vector<double> position_states_;
  std::vector<double> previous_position_states_;
  std::vector<double> velocity_states_;

  // --- gRPC Objects ---
  std::shared_ptr<grpc::Channel> grpc_channel_;
  std::unique_ptr<rcs::v1::RealtimeMonitorService::Stub> monitor_stub_;
  std::unique_ptr<rcs::v1::IncrementMoveService::Stub> motion_stub_;
  std::unique_ptr<rcs::v1::ServoPowerControlService::Stub> servo_stub_;
  std::unique_ptr<rcs::v1::AlarmControlService::Stub> alarm_stub_;
  std::unique_ptr<rcs::v1::SystemInfoService::Stub> system_stub_;

  // --- Full-trajectory recording ---
  // Streams four checkpoints of every control cycle, for the whole movement, so
  // they can be recorded (e.g. `ros2 bag record`) and plotted against each other:
  //   joint_command_sent - the position ros2_control wants, timestamped right
  //     before it's handed to the gRPC call (what "moveit sent this cycle").
  //   joint_command      - the same position, timestamped right after the ACU
  //     acknowledges the gRPC call (what "the ACU received").
  //   joint_command_acu  - the ACU's own internal command/setpoint stream
  //     (GetAxesPos), i.e. what the ACU's interpolator is itself currently
  //     driving toward - distinct from joint_command (our record of what we
  //     sent) and from joint_feedback (physical encoder readback).
  //   joint_feedback     - the physical encoder position read back from the ACU
  //     (GetFeedbackAxesPos).
  std::vector<std::string> joint_names_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_sent_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_acu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_feedback_pub_;
};

}  // namespace ynx_hardware_interface

#endif  // YNX_HARDWARE_INTERFACE__YNX_HARDWARE_INTERFACE_HPP_
