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

    // Pin needs to be in range 0-7
    if (pin < 0 || pin > 7) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Invalid Pin: %d. Needs to be in between 0 and 7.", pin);
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
    RCLCPP_INFO(this->get_logger(), "[Set Io Service] Attempting to set io - Pin: %d, state: %s", 
                pin, state ? "true" : "false");

    // TODO: Implement IO service

    RCLCPP_INFO(this->get_logger(), "[Set Io Service] Successfully set io.");
    response->success = true;
    response->message = "Io successfully updated.";
  }
}  // namespace ynx_robot_managr
