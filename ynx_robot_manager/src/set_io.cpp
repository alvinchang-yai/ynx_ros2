#include "ynx_robot_manager/ynx_robot_manager.hpp"

namespace ynx_robot_manager
{
  void YnxRobotManager::set_io_service_callback(
      const std::shared_ptr<SetIo::Request> request,
      std::shared_ptr<SetIo::Response> response) 
  {
    // Extract io parameters from the request
    int pin = request->pin;
    int state = request->state;

    // Pin needs to be in range 1-8 (pin N maps to the hardware interface's digital_output_(N-1))
    if (pin < 1 || pin > 8) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Invalid Pin: %d. Needs to be in between 1 and 8.", pin);
      response->success = false;
      response->message = "Io update failed: Invalid pin.";
      return;
    }
    if (state != 0 && state != 1) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Invalid State: %d. Needs to be 0 or 1", state);
      response->success = false;
      response->message = "Io update failed: Invalid state.";
      return;
    }

    // Nobody listening means the gpio_command_controller isn't active, so the command would be dropped.
    if (gpio_command_publisher_->get_subscription_count() == 0) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] gpio_command_controller is not available.");
      response->success = false;
      response->message = "Io update failed: gpio_command_controller is not active.";
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[Set Io Service] Attempting to set io - Pin: %d, state: %s", 
                pin, state ? "true" : "false");

    // Command a single output interface of the "gpio_io" group
    control_msgs::msg::DynamicInterfaceGroupValues msg;
    msg.interface_groups.push_back("gpio_io");
    control_msgs::msg::InterfaceValue values;
    values.interface_names.push_back("digital_output_" + std::to_string(pin - 1));
    values.values.push_back(static_cast<double>(state));
    msg.interface_values.push_back(values);
    gpio_command_publisher_->publish(msg);

    // The command is applied asynchronously by the hardware interface.
    response->success = true;
    response->message = "Io command sent.";
  }
}  // namespace ynx_robot_manager
