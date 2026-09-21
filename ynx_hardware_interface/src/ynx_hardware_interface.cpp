#include "ynx_hardware_interface/ynx_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <chrono>

namespace ynx_hardware_interface
{

  hardware_interface::CallbackReturn YnxHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    ip_= info_.hardware_parameters["ip"];
    port_= info_.hardware_parameters["port"];
    if (ip_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"), "[INIT] Ip address not provided!");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (port_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"), "[INIT] Port not provided!");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize to 0.0 for the mock interface to avoid NaN errors in controllers
    position_states_.resize(info_.joints.size(), 0.0);
    previous_position_states_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);
    position_commands_.resize(info_.joints.size(), 0.0);
    previous_position_commands_.resize(info_.joints.size(), 0.0);

    joint_names_.clear();
    for (const auto & joint : info_.joints) {
      joint_names_.push_back(joint.name);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn YnxHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Create gRPC Channel
    RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[CONFIG] Connecting to %s:%s ...", ip_.c_str(), port_.c_str());
    grpc_channel_ = grpc::CreateChannel(ip_ + ":" + port_, grpc::InsecureChannelCredentials());
    // Create gRPC stubs
    monitor_stub_ = rcs::v1::RealtimeMonitorService::NewStub(grpc_channel_);
    motion_stub_ = rcs::v1::IncrementMoveService::NewStub(grpc_channel_);
    servo_stub_ = rcs::v1::ServoPowerControlService::NewStub(grpc_channel_);
    alarm_stub_ = rcs::v1::AlarmControlService::NewStub(grpc_channel_);
    system_stub_ = rcs::v1::SystemInfoService::NewStub(grpc_channel_);
    io_stub_ = rcs::v1::IOService::NewStub(grpc_channel_);

    // 2. Perform Connection Check (Handshake)
    grpc::ClientContext context;
    // Set a 2-second deadline for the connection check
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(2);
    context.set_deadline(deadline);

    rcs::v1::GetFirmwareVersionRequest version_req;
    rcs::v1::GetFirmwareVersionResponse version_res;
    grpc::Status status = system_stub_->GetFirmwareVersion(&context, version_req, &version_res);

    if (status.ok() && version_res.status() == rcs::v1::GetFirmwareVersionResponse::STATUS_SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[CONFIG] Connection established!");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"), "[CONFIG] Failed to connect to controller! gRPC Error: %s (%d)", status.error_message().c_str(), status.error_code());
      return hardware_interface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[CONFIG] Connection established!");

    // Full-rate trajectory streams for plotting commanded vs. actual motion.
    joint_command_sent_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_command_sent", rclcpp::SensorDataQoS());
    joint_command_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_command", rclcpp::SensorDataQoS());
    joint_command_acu_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_command_acu", rclcpp::SensorDataQoS());
    joint_feedback_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_feedback", rclcpp::SensorDataQoS());

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn YnxHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    grpc::ClientContext alarm_context;
    rcs::v1::ClearAlarmErrorRequest alarm_req;
    rcs::v1::ClearAlarmErrorResponse alarm_res;
    grpc::Status alarm_status = alarm_stub_->ClearAlarmError(&alarm_context, alarm_req, &alarm_res);
    if (alarm_status.ok() && alarm_res.status() == rcs::v1::ClearAlarmErrorResponse::STATUS_SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Alarms cleared successfully!");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Clear alarm command failed or returned non-success. Status: %d", alarm_res.status());
    }

    // Power on servos
    grpc::ClientContext servo_context;
    rcs::v1::PowerOnServosRequest servo_req;
    rcs::v1::PowerOnServosResponse servo_res;

    grpc::Status servo_status = servo_stub_->PowerOnServos(&servo_context, servo_req, &servo_res);

    if (servo_status.ok() && servo_res.status() == rcs::v1::PowerOnServosResponse::STATUS_SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Servos powered on successfully!");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Failed to turn on servos. Status: %d", servo_res.status());
      return hardware_interface::CallbackReturn::ERROR; 
    }

    // Start the background feedback/ACU-setpoint streams and the alarm poller
    // before the sync-states read below - read() now depends on these threads
    // having produced at least one sample.
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      cached_feedback_rad_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(acu_mutex_);
      cached_acu_rad_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(alarm_mutex_);
      last_alarm_check_time_ = std::chrono::steady_clock::time_point();
      alarm_has_data_ = false;
    }
    alarm_fault_.store(false);
    feedback_thread_running_.store(true);
    acu_thread_running_.store(true);
    alarm_thread_running_.store(true);
    feedback_thread_ = std::thread(&YnxHardwareInterface::feedbackStreamLoop, this);
    acu_thread_ = std::thread(&YnxHardwareInterface::acuStreamLoop, this);
    alarm_thread_ = std::thread(&YnxHardwareInterface::alarmPollLoop, this);

    // Sync States
    // 10x100ms (was 5x100ms): the background streams/poller need a moment to
    // warm up after being (re)started above, and the old 500ms budget wasn't
    // reliably enough margin on real hardware.
    int max_retries = 10;
    bool read_success = false;

    for (int i = 0; i < max_retries; i++) {
      if (read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.0)) == hardware_interface::return_type::OK) {
        read_success = true;
        break;
      }
      RCLCPP_WARN(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Failed to read initial state. Retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!read_success) {
      RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Could not read starting position. Aborting activation.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (uint i = 0; i < position_states_.size(); i++) {
      position_commands_[i] = position_states_[i];
      previous_position_commands_[i] = position_states_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] States synced succesfully!");

    // Seed the I/O commands from the robot's current output levels so that
    // activating doesn't turn any output that is currently ON back OFF. This is
    // a one-off blocking call (activation isn't RT); on failure the commands
    // stay 0 and nothing is written until a controller commands a change.
    {
      std::array<uint32_t, kNumGpio> inputs{};
      std::array<uint32_t, kNumGpio> outputs{};
      if (readIoStatus(inputs, outputs)) {
        std::lock_guard<std::mutex> lock(io_mutex_);
        io_cached_inputs_ = inputs;
        io_cached_outputs_ = outputs;
        io_pending_.fill(-1);
        for (size_t i = 0; i < kNumGpio; i++) {
          gpio_input_states_[i] = inputs[i];
          gpio_output_states_[i] = outputs[i];
          gpio_output_commands_[i] = outputs[i];
          io_last_commanded_[i] = outputs[i];
        }
        RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] I/O states synced successfully!");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Could not read initial I/O state; outputs will only be written on change.");
        std::lock_guard<std::mutex> lock(io_mutex_);
        io_pending_.fill(-1);
        io_last_commanded_.fill(0.0);
        gpio_output_commands_.fill(0.0);
      }
    }
    io_thread_running_.store(true);
    io_thread_ = std::thread(&YnxHardwareInterface::ioLoop, this);

    // Start Increment Move
    grpc::ClientContext motion_context;
    rcs::v1::StartIncrementMoveRequest motion_req;
    rcs::v1::StartIncrementMoveResponse motion_res;
    motion_req.set_control_group_bit(control_group_bit_);
    grpc::Status status = motion_stub_->StartIncrementMove(&motion_context, motion_req, &motion_res);

    if (status.ok() && motion_res.status() == rcs::v1::StartIncrementMoveResponse::STATUS_SUCCESS) {
      task_no_ = motion_res.task_no();
      RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Increment Motion started succesfully!");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"), "[ACTIVATION] Failed to start Increment Move. Status: %d", motion_res.status());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Start the background sender thread: write() will only accumulate deltas
    // from here on, this thread does the actual (blocking) SetIncrementMove calls.
    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      pending_delta_rad_.assign(info_.joints.size(), 0.0);
      last_commanded_position_rad_ = position_commands_;
    }
    write_fault_count_ = 0;
    write_fault_.store(false);
    write_thread_running_.store(true);
    write_thread_ = std::thread(&YnxHardwareInterface::writeSenderLoop, this);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn YnxHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Stop the background sender first, before telling the ACU to stop
    // increment mode, so nothing races a SetIncrementMove against StopIncrementMove.
    if (write_thread_running_.exchange(false)) {
      if (write_thread_.joinable()) {
        write_thread_.join();
      }
    }

    // Stop the feedback/ACU-setpoint streams. Their background threads are
    // blocked inside a streaming Read() call, so just clearing the running
    // flag won't wake them - cancel the gRPC context to unblock Read() first.
    feedback_thread_running_.store(false);
    {
      std::lock_guard<std::mutex> lock(feedback_ctx_mutex_);
      if (feedback_stream_ctx_) {
        feedback_stream_ctx_->TryCancel();
      }
    }
    if (feedback_thread_.joinable()) {
      feedback_thread_.join();
    }

    acu_thread_running_.store(false);
    {
      std::lock_guard<std::mutex> lock(acu_ctx_mutex_);
      if (acu_stream_ctx_) {
        acu_stream_ctx_->TryCancel();
      }
    }
    if (acu_thread_.joinable()) {
      acu_thread_.join();
    }

    // The alarm poller is unary + sleeps between polls, so clearing the flag
    // is enough - it's checked at the top of each loop iteration.
    alarm_thread_running_.store(false);
    if (alarm_thread_.joinable()) {
      alarm_thread_.join();
    }

    // The I/O thread only makes unary calls with a short deadline, so it exits
    // promptly once the flag is cleared.
    io_thread_running_.store(false);
    if (io_thread_.joinable()) {
      io_thread_.join();
    }

    if (task_no_ >= 0) {
      grpc::ClientContext context;
      rcs::v1::StopIncrementMoveRequest req;
      rcs::v1::StopIncrementMoveResponse res;

      req.set_task_no(task_no_);

      motion_stub_->StopIncrementMove(&context, req, &res);
      task_no_ = -1;
    }

    grpc::ClientContext servo_context;
    rcs::v1::PowerOffServosRequest servo_req;
    rcs::v1::PowerOffServosResponse servo_res;

    grpc::Status servo_status = servo_stub_->PowerOffServos(&servo_context, servo_req, &servo_res);

    if (servo_status.ok() && servo_res.status() == rcs::v1::PowerOffServosResponse::STATUS_SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[DEACTIVATION] Servos powered off successfully.");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("YnxHardwareInterface"), "[DEACTIVATION] Failed to turn off servos. Status: %d", servo_res.status());
    }

    RCLCPP_INFO(rclcpp::get_logger("YnxHardwareInterface"), "[DEACTIVATION] Succesfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type YnxHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
    // Everything here just copies cached data written by the background
    // threads (feedbackStreamLoop/acuStreamLoop/alarmPollLoop) and checks
    // staleness - no gRPC calls happen on this (RT) thread anymore.

    // Alarm status: fail safe if the poller itself is stuck/unreachable (can't
    // confirm "no alarm" if we can't reach the ACU at all), or if it reported
    // an active fault.
    {
      std::lock_guard<std::mutex> lock(alarm_mutex_);
      if (!alarm_has_data_) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *this->get_clock(), 1000,
            "[READ] No alarm status received yet from the poller thread.");
        return hardware_interface::return_type::ERROR;
      }
      auto age = std::chrono::steady_clock::now() - last_alarm_check_time_;
      if (age > std::chrono::milliseconds(kAlarmStalenessMs)) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *this->get_clock(), 1000,
            "[READ] Alarm status stale (age %.1f ms) - cannot confirm safety, halting.",
            std::chrono::duration<double, std::milli>(age).count());
        return hardware_interface::return_type::ERROR;
      }
    }
    if (alarm_fault_.load()) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *this->get_clock(), 1000,
          "[READ] Robot has active alarms/errors! Halting.");
      return hardware_interface::return_type::ERROR;
    }

    // Physical feedback: fail safe if the stream hasn't produced a fresh
    // sample recently, rather than silently feeding the control loop stale
    // position data.
    std::vector<double> feedback_snapshot;
    rclcpp::Time feedback_stamp;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (cached_feedback_rad_.empty()) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *this->get_clock(), 1000,
            "[READ] No feedback received yet from the streaming thread.");
        return hardware_interface::return_type::ERROR;
      }
      auto age = std::chrono::steady_clock::now() - cached_feedback_time_;
      if (age > std::chrono::milliseconds(kFeedbackStalenessMs)) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *this->get_clock(), 1000,
            "[READ] Feedback stale (age %.1f ms) - halting.",
            std::chrono::duration<double, std::milli>(age).count());
        return hardware_interface::return_type::ERROR;
      }
      feedback_snapshot = cached_feedback_rad_;
      feedback_stamp = cached_feedback_stamp_;
    }

    for (uint i = 0; i < info_.joints.size(); i++) {
      previous_position_states_[i] = position_states_[i];
      position_states_[i] = feedback_snapshot[i];
      if (period.seconds() > 0.0) {
        velocity_states_[i] = (position_states_[i] - previous_position_states_[i]) / period.seconds();
      }
    }

    if (joint_feedback_pub_) {
      sensor_msgs::msg::JointState feedback_msg;
      feedback_msg.header.stamp = feedback_stamp;
      feedback_msg.name = joint_names_;
      feedback_msg.position = position_states_;
      feedback_msg.velocity = velocity_states_;
      joint_feedback_pub_->publish(feedback_msg);
    }

    // ACU internal setpoint: recording-only, never fails read() - just skip
    // publishing if there's nothing fresh cached yet.
    if (joint_command_acu_pub_) {
      std::vector<double> acu_snapshot;
      rclcpp::Time acu_stamp;
      bool acu_fresh = false;
      {
        std::lock_guard<std::mutex> lock(acu_mutex_);
        if (!cached_acu_rad_.empty()) {
          auto age = std::chrono::steady_clock::now() - cached_acu_time_;
          if (age <= std::chrono::milliseconds(kFeedbackStalenessMs)) {
            acu_snapshot = cached_acu_rad_;
            acu_stamp = cached_acu_stamp_;
            acu_fresh = true;
          }
        }
      }
      if (acu_fresh) {
        sensor_msgs::msg::JointState acu_msg;
        acu_msg.header.stamp = acu_stamp;
        acu_msg.name = joint_names_;
        acu_msg.position = acu_snapshot;
        joint_command_acu_pub_->publish(acu_msg);
      }
    }

    // I/O levels: best effort, never fails read(). If the I/O thread holds the
    // lock right now, keep last cycle's values instead of waiting.
    {
      std::unique_lock<std::mutex> lock(io_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        for (size_t i = 0; i < kNumGpio; i++) {
          gpio_input_states_[i] = io_cached_inputs_[i];
          gpio_output_states_[i] = io_cached_outputs_[i];
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type YnxHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // Safety check to ensure we have a valid task handle
    if (task_no_ < 0) {
      return hardware_interface::return_type::ERROR;
    }

    if (write_fault_.load()) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *this->get_clock(), 1000,
          "[WRITE] Background sender has a persistent fault (see earlier errors) - halting.");
      return hardware_interface::return_type::ERROR;
    }

    // Only accumulate the delta and publish "sent" here - the actual gRPC call
    // happens on the background sender thread (writeSenderLoop), so this never
    // blocks the RT control loop on the network round-trip. Publish "sent" on
    // this thread since it should reflect every commanded cycle, not just the
    // (less frequent) cycles the background thread actually manages to send.
    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      for (uint i = 0; i < info_.joints.size(); i++) {
        double delta_rad = position_commands_[i] - previous_position_commands_[i];
        previous_position_commands_[i] = position_commands_[i];
        pending_delta_rad_[i] += delta_rad;
      }
      last_commanded_position_rad_ = position_commands_;
    }

    // Record output changes for the I/O thread. try_lock: if it is busy, the
    // change is picked up next cycle (io_last_commanded_ is only updated once
    // the change was handed over).
    {
      std::unique_lock<std::mutex> lock(io_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        for (size_t i = 0; i < kNumGpio; i++) {
          if (gpio_output_commands_[i] != io_last_commanded_[i]) {
            io_pending_[i] = gpio_output_commands_[i] > 0.5 ? 1 : 0;
            io_last_commanded_[i] = gpio_output_commands_[i];
          }
        }
      }
    }

    if (joint_command_sent_pub_) {
      sensor_msgs::msg::JointState sent_msg;
      sent_msg.header.stamp = get_clock()->now();
      sent_msg.name = joint_names_;
      sent_msg.position = position_commands_;
      joint_command_sent_pub_->publish(sent_msg);
    }

    return hardware_interface::return_type::OK;
  }

  void YnxHardwareInterface::writeSenderLoop() {
    while (write_thread_running_.load()) {
      std::vector<double> delta_to_send;
      std::vector<double> position_snapshot;
      {
        std::lock_guard<std::mutex> lock(write_mutex_);
        delta_to_send = pending_delta_rad_;
        std::fill(pending_delta_rad_.begin(), pending_delta_rad_.end(), 0.0);
        position_snapshot = last_commanded_position_rad_;
      }

      grpc::ClientContext context;
      rcs::v1::SetIncrementMoveRequest req;
      rcs::v1::SetIncrementMoveResponse res;
      req.set_task_no(task_no_);
      req.set_timeout(100);
      rcs::v1::IncrementMoveGroupRequest* group_req = req.add_requests();
      group_req->set_group_no(group_no_);
      rcs::v1::AxesPos* angle_pos = group_req->mutable_angle();
      for (double delta_rad : delta_to_send) {
        angle_pos->add_pos(delta_rad * (180.0 / M_PI));
      }

      grpc::Status status = motion_stub_->SetIncrementMove(&context, req, &res);

      if (status.ok() && res.status() == rcs::v1::SetIncrementMoveResponse::STATUS_SUCCESS) {
        write_fault_count_ = 0;
        write_fault_.store(false);

        if (joint_command_pub_) {
          sensor_msgs::msg::JointState command_msg;
          command_msg.header.stamp = get_clock()->now();
          command_msg.name = joint_names_;
          command_msg.position = position_snapshot;
          joint_command_pub_->publish(command_msg);
        }
      } else {
        // Don't drop the delta on failure - merge it back in with whatever has
        // accumulated since, so a transient failure doesn't silently lose
        // commanded motion. It'll be included in the next send attempt.
        {
          std::lock_guard<std::mutex> lock(write_mutex_);
          for (uint i = 0; i < delta_to_send.size(); i++) {
            pending_delta_rad_[i] += delta_to_send[i];
          }
        }

        write_fault_count_++;
        RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"),
            "[WRITE] Failed to set Increment Move. gRPC ok: %d, Response status: %d (consecutive failures: %d)",
            status.ok(), res.status(), write_fault_count_);

        if (write_fault_count_ >= kMaxConsecutiveWriteFailures) {
          write_fault_.store(true);
        }
        // Avoid hammering a genuinely unreachable ACU in a tight retry loop.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  bool YnxHardwareInterface::readIoStatus(
      std::array<uint32_t, kNumGpio> & inputs, std::array<uint32_t, kNumGpio> & outputs) {
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(kIoDeadlineMs));
    rcs::v1::GetIOStatusRequest req;
    rcs::v1::GetIOStatusResponse res;
    for (uint32_t i = 0; i < kNumGpio; i++) {
      req.add_addresses(kGpioInputBaseAddress + i);
    }
    for (uint32_t i = 0; i < kNumGpio; i++) {
      req.add_addresses(kGpioOutputBaseAddress + i);
    }

    grpc::Status status = io_stub_->GetIOStatus(&context, req, &res);
    if (!status.ok()) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *get_clock(), 2000,
          "[IO] GetIOStatus gRPC call failed: %s", status.error_message().c_str());
      return false;
    }
    if (res.status() != rcs::v1::GetIOStatusResponse::STATUS_SUCCESS ||
        res.io_response_size() != static_cast<int>(2 * kNumGpio)) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *get_clock(), 2000,
          "[IO] GetIOStatus returned status %d with %d values (expected %zu).",
          static_cast<int>(res.status()), res.io_response_size(), 2 * kNumGpio);
      return false;
    }
    for (size_t i = 0; i < kNumGpio; i++) {
      inputs[i] = res.io_response(static_cast<int>(i)).value();
      outputs[i] = res.io_response(static_cast<int>(i + kNumGpio)).value();
    }
    return true;
  }

  void YnxHardwareInterface::ioLoop() {
    while (io_thread_running_.load()) {
      // 1. Send output changes recorded by write().
      std::array<int, kNumGpio> to_send;
      {
        std::lock_guard<std::mutex> lock(io_mutex_);
        to_send = io_pending_;
        io_pending_.fill(-1);
      }

      rcs::v1::SetIOStatusRequest set_req;
      for (size_t i = 0; i < kNumGpio; i++) {
        if (to_send[i] >= 0) {
          auto * io = set_req.add_io_request();
          io->set_address(kGpioOutputBaseAddress + static_cast<uint32_t>(i));
          io->set_value(static_cast<uint32_t>(to_send[i]));
        }
      }
      if (set_req.io_request_size() > 0) {
        grpc::ClientContext context;
        context.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(kIoDeadlineMs));
        rcs::v1::SetIOStatusResponse set_res;
        grpc::Status status = io_stub_->SetIOStatus(&context, set_req, &set_res);
        if (!status.ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"),
              "[IO] SetIOStatus gRPC call failed: %s - will retry.", status.error_message().c_str());
          // Transport failure: put the changes back unless a newer command
          // for the same pin arrived in the meantime.
          std::lock_guard<std::mutex> lock(io_mutex_);
          for (size_t i = 0; i < kNumGpio; i++) {
            if (to_send[i] >= 0 && io_pending_[i] < 0) {
              io_pending_[i] = to_send[i];
            }
          }
        } else if (set_res.status() != rcs::v1::SetIOStatusResponse::STATUS_SUCCESS) {
          // The controller rejected it; retrying the same request won't help.
          RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"),
              "[IO] Controller rejected SetIOStatus (status %d, %d of %d applied).",
              static_cast<int>(set_res.status()), set_res.set_num(), set_req.io_request_size());
        }
      }

      // 2. Refresh the cached levels for read().
      std::array<uint32_t, kNumGpio> inputs{};
      std::array<uint32_t, kNumGpio> outputs{};
      if (readIoStatus(inputs, outputs)) {
        std::lock_guard<std::mutex> lock(io_mutex_);
        io_cached_inputs_ = inputs;
        io_cached_outputs_ = outputs;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kIoPollIntervalMs));
    }
  }

  void YnxHardwareInterface::feedbackStreamLoop() {
    while (feedback_thread_running_.load()) {
      auto context = std::make_shared<grpc::ClientContext>();
      {
        std::lock_guard<std::mutex> lock(feedback_ctx_mutex_);
        feedback_stream_ctx_ = context;
      }

      rcs::v1::GetFeedbackAxesPosStreamRequest request;
      request.set_group_no(group_no_);
      request.set_rate(kFeedbackStreamRateHz);
      auto reader = monitor_stub_->GetFeedbackAxesPosStream(context.get(), request);

      rcs::v1::GetFeedbackAxesPosStreamResponse response;
      while (feedback_thread_running_.load() && reader->Read(&response)) {
        if (response.status() == rcs::v1::GetFeedbackAxesPosStreamResponse::STATUS_SUCCESS) {
          std::vector<double> pos_rad(info_.joints.size());
          for (uint i = 0; i < info_.joints.size(); i++) {
            pos_rad[i] = response.axes_pos().pos(i) * (M_PI / 180.0);
          }
          rclcpp::Time capture_stamp = get_clock()->now();
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          cached_feedback_rad_ = std::move(pos_rad);
          cached_feedback_time_ = std::chrono::steady_clock::now();
          cached_feedback_stamp_ = capture_stamp;
        }
        // else: bad sample - skip it, keep the previous cache, staleness
        // watchdog in read() catches a prolonged failure to get good data.
      }

      grpc::Status status = reader->Finish();
      {
        std::lock_guard<std::mutex> lock(feedback_ctx_mutex_);
        feedback_stream_ctx_.reset();
      }

      if (!feedback_thread_running_.load()) {
        break;  // intentional shutdown (on_deactivate cancelled the context)
      }

      RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"),
          "[READ] Feedback stream ended unexpectedly (code %d: %s) - reconnecting...",
          status.error_code(), status.error_message().c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void YnxHardwareInterface::acuStreamLoop() {
    while (acu_thread_running_.load()) {
      auto context = std::make_shared<grpc::ClientContext>();
      {
        std::lock_guard<std::mutex> lock(acu_ctx_mutex_);
        acu_stream_ctx_ = context;
      }

      rcs::v1::GetAxesPosStreamRequest request;
      request.set_group_no(group_no_);
      request.set_rate(kFeedbackStreamRateHz);
      auto reader = monitor_stub_->GetAxesPosStream(context.get(), request);

      rcs::v1::GetAxesPosStreamResponse response;
      while (acu_thread_running_.load() && reader->Read(&response)) {
        if (response.status() == rcs::v1::GetAxesPosStreamResponse::STATUS_SUCCESS) {
          std::vector<double> pos_rad(info_.joints.size());
          for (uint i = 0; i < info_.joints.size(); i++) {
            pos_rad[i] = response.axes_pos().pos(i) * (M_PI / 180.0);
          }
          rclcpp::Time capture_stamp = get_clock()->now();
          std::lock_guard<std::mutex> lock(acu_mutex_);
          cached_acu_rad_ = std::move(pos_rad);
          cached_acu_time_ = std::chrono::steady_clock::now();
          cached_acu_stamp_ = capture_stamp;
        }
      }

      grpc::Status status = reader->Finish();
      {
        std::lock_guard<std::mutex> lock(acu_ctx_mutex_);
        acu_stream_ctx_.reset();
      }

      if (!acu_thread_running_.load()) {
        break;
      }

      // Diagnostic-only stream - warn, not error, and keep retrying.
      RCLCPP_WARN(rclcpp::get_logger("YnxHardwareInterface"),
          "[READ] ACU-setpoint stream ended unexpectedly (code %d: %s) - reconnecting...",
          status.error_code(), status.error_message().c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void YnxHardwareInterface::alarmPollLoop() {
    while (alarm_thread_running_.load()) {
      grpc::ClientContext context;
      rcs::v1::GetAlarmErrorRequest request;
      rcs::v1::GetAlarmErrorResponse response;
      grpc::Status status = alarm_stub_->GetAlarmError(&context, request, &response);

      if (status.ok() && response.status() == rcs::v1::GetAlarmErrorResponse::STATUS_SUCCESS) {
        bool has_fault = (response.alarms_size() > 0 || response.errors_size() > 0);
        alarm_fault_.store(has_fault);
        if (has_fault) {
          RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("YnxHardwareInterface"), *this->get_clock(), 1000,
              "[READ] Robot has %d active alarms and %d active errors!",
              response.alarms_size(), response.errors_size());
        }
        std::lock_guard<std::mutex> lock(alarm_mutex_);
        last_alarm_check_time_ = std::chrono::steady_clock::now();
        alarm_has_data_ = true;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("YnxHardwareInterface"),
            "[READ] GetAlarmError failed - gRPC ok: %d, status: %d", status.ok(), response.status());
        // Don't update last_alarm_check_time_ - the staleness watchdog in
        // read() will trip if this keeps failing.
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kAlarmPollIntervalMs));
    }
  }

  std::vector<hardware_interface::StateInterface> YnxHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
    for (size_t i = 0; i < kNumGpio; i++) {
      state_interfaces.emplace_back("gpio_io", "digital_input_" + std::to_string(i), &gpio_input_states_[i]);
      state_interfaces.emplace_back("gpio_io", "digital_output_" + std::to_string(i), &gpio_output_states_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> YnxHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }
    for (size_t i = 0; i < kNumGpio; i++) {
      command_interfaces.emplace_back("gpio_io", "digital_output_" + std::to_string(i), &gpio_output_commands_[i]);
    }
    return command_interfaces;
  }

}  // namespace ynx_hardware_interface

// Export the class to pluginlib so it can be dynamically loaded
PLUGINLIB_EXPORT_CLASS(
    ynx_hardware_interface::YnxHardwareInterface, hardware_interface::SystemInterface)
