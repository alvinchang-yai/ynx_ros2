#ifndef YNX_HARDWARE_INTERFACE__YNX_HARDWARE_INTERFACE_HPP_
#define YNX_HARDWARE_INTERFACE__YNX_HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
#include "rcs/v1/io_api.grpc.pb.h"
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

  // --- General digital I/O (ros2_control <gpio name="gpio_io">) ---
  // Pins 0-7 map to controller general input #00010-#00017 and general output
  // #10010-#10017. The exported interfaces are only touched from read()/write()
  // (the RT thread); the background thread below is the only one that talks
  // gRPC and hands data over through io_mutex_ / the cache below.
  static constexpr size_t kNumGpio = 8;
  static constexpr uint32_t kGpioInputBaseAddress = 10;
  static constexpr uint32_t kGpioOutputBaseAddress = 10010;
  static constexpr int kIoPollIntervalMs = 20;
  static constexpr int kIoDeadlineMs = 200;

  std::array<double, kNumGpio> gpio_input_states_{};
  std::array<double, kNumGpio> gpio_output_states_{};
  std::array<double, kNumGpio> gpio_output_commands_{};

  // --- gRPC Objects ---
  std::shared_ptr<grpc::Channel> grpc_channel_;
  std::unique_ptr<rcs::v1::RealtimeMonitorService::Stub> monitor_stub_;
  std::unique_ptr<rcs::v1::IncrementMoveService::Stub> motion_stub_;
  std::unique_ptr<rcs::v1::ServoPowerControlService::Stub> servo_stub_;
  std::unique_ptr<rcs::v1::AlarmControlService::Stub> alarm_stub_;
  std::unique_ptr<rcs::v1::SystemInfoService::Stub> system_stub_;
  std::unique_ptr<rcs::v1::IOService::Stub> io_stub_;

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

  // --- Async write: coalescing buffer + background sender thread ---
  // write() only accumulates the commanded delta and returns immediately; a
  // background thread drains the accumulated delta and does the actual
  // SetIncrementMove gRPC call, so the RT control loop never blocks on the
  // network round-trip. See the design plan discussed before implementing this.
  std::mutex write_mutex_;
  std::vector<double> pending_delta_rad_;         // accumulated since the last send, protected by write_mutex_
  std::vector<double> last_commanded_position_rad_;  // snapshot of position_commands_, protected by write_mutex_
  std::thread write_thread_;
  std::atomic<bool> write_thread_running_{false};
  std::atomic<bool> write_fault_{false};
  int write_fault_count_ = 0;  // background thread only, no cross-thread access

  static constexpr int kMaxConsecutiveWriteFailures = 5;

  void writeSenderLoop();

  // --- Async read: streaming feedback/ACU-setpoint + background alarm polling ---
  // Mirrors the write-side design: read() only copies cached data and checks
  // staleness, never blocks on gRPC. Feedback and the ACU-setpoint stream use
  // persistent server-streaming RPCs (no per-sample handshake); alarms have no
  // streaming API in the RCS SDK, so they're polled on their own background
  // thread instead. See the design plan discussed before implementing this.
  std::mutex feedback_mutex_;
  std::vector<double> cached_feedback_rad_;
  std::chrono::steady_clock::time_point cached_feedback_time_;  // monotonic, staleness watchdog only
  rclcpp::Time cached_feedback_stamp_;  // ROS clock at capture, published as-is so recorded latency isn't inflated by cache age
  std::mutex feedback_ctx_mutex_;
  std::shared_ptr<grpc::ClientContext> feedback_stream_ctx_;
  std::atomic<bool> feedback_thread_running_{false};
  std::thread feedback_thread_;

  std::mutex acu_mutex_;
  std::vector<double> cached_acu_rad_;
  std::chrono::steady_clock::time_point cached_acu_time_;  // monotonic, staleness watchdog only
  rclcpp::Time cached_acu_stamp_;  // ROS clock at capture, published as-is
  std::mutex acu_ctx_mutex_;
  std::shared_ptr<grpc::ClientContext> acu_stream_ctx_;
  std::atomic<bool> acu_thread_running_{false};
  std::thread acu_thread_;

  std::mutex alarm_mutex_;
  std::chrono::steady_clock::time_point last_alarm_check_time_;
  bool alarm_has_data_ = false;  // true once the poller has completed at least one poll; protected by alarm_mutex_
  std::atomic<bool> alarm_fault_{false};
  std::atomic<bool> alarm_thread_running_{false};
  std::thread alarm_thread_;

  // --- Async I/O: background poll + write thread ---
  // read() copies the cached input/output levels, write() only records which
  // outputs changed. The background thread does the actual GetIOStatus /
  // SetIOStatus calls, so the RT loop never blocks on the network. read() and
  // write() use try_lock: on contention they skip one cycle rather than wait.
  std::mutex io_mutex_;
  std::array<uint32_t, kNumGpio> io_cached_inputs_{};    // protected by io_mutex_
  std::array<uint32_t, kNumGpio> io_cached_outputs_{};   // protected by io_mutex_
  std::array<int, kNumGpio> io_pending_{};                // -1 = nothing to send, else 0/1; protected by io_mutex_
  std::array<double, kNumGpio> io_last_commanded_{};      // RT thread only (write())
  std::thread io_thread_;
  std::atomic<bool> io_thread_running_{false};

  bool readIoStatus(std::array<uint32_t, kNumGpio> & inputs, std::array<uint32_t, kNumGpio> & outputs);
  void ioLoop();

  static constexpr double kFeedbackStreamRateHz = 250.0;
  // Loosened after real-hardware testing: 3x the ~4ms sample interval (12ms) tripped
  // on ordinary startup/stream jitter (observed 12.8ms), and read() has zero retry
  // tolerance during normal operation (a single miss deactivates the whole hardware
  // component), unlike the 5x-retry activation path. 50ms is still a large
  // improvement over the old synchronous design's effective ~158Hz refresh.
  static constexpr int kFeedbackStalenessMs = 50;
  static constexpr int kAlarmPollIntervalMs = 50;
  static constexpr int kAlarmStalenessMs = 500;    // loosened alongside kFeedbackStalenessMs, same reasoning

  void feedbackStreamLoop();
  void acuStreamLoop();
  void alarmPollLoop();
};

}  // namespace ynx_hardware_interface

#endif  // YNX_HARDWARE_INTERFACE__YNX_HARDWARE_INTERFACE_HPP_
