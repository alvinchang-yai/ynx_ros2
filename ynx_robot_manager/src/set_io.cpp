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

    // General outputs start at #10010, one address per pin (0-7)
    rcs::v1::SetIOStatusRequest io_request;
    auto * io = io_request.add_io_request();
    io->set_address(10010 + pin);
    io->set_value(state);

    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(1));
    rcs::v1::SetIOStatusResponse io_response;
    grpc::Status status = io_stub_->SetIOStatus(&context, io_request, &io_response);

    if (!status.ok()) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] gRPC call failed: %s", status.error_message().c_str());
      response->success = false;
      response->message = "Io update failed: " + status.error_message();
      return;
    }
    if (io_response.status() != rcs::v1::SetIOStatusResponse::STATUS_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "[Set Io Service] Controller rejected request (status %d).", static_cast<int>(io_response.status()));
      response->success = false;
      response->message = "Io update failed: Controller returned status " + std::to_string(static_cast<int>(io_response.status())) + ".";
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[Set Io Service] Successfully set io.");
    response->success = true;
    response->message = "Io successfully updated.";
  }
}  // namespace ynx_robot_managr
