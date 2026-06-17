#include "ynx_robot_manager/ynx_robot_manager.hpp"

namespace ynx_robot_manager
{
  void YnxRobotManager::input_wrench_subscription_callback_(const WrenchStamped::SharedPtr msg) 
  {
    WrenchStamped output_msg;
    output_msg.header = msg->header;
    output_msg.wrench.force.x =   msg->wrench.force.x;
    output_msg.wrench.force.y =  -msg->wrench.force.y;
    output_msg.wrench.force.z =   msg->wrench.force.z;
    output_msg.wrench.torque.x = -msg->wrench.torque.x;
    output_msg.wrench.torque.y = -msg->wrench.torque.y;
    output_msg.wrench.torque.z =  msg->wrench.torque.z;
    wrench_publisher_->publish(output_msg);
  }
}  // namespace ynx_robot_manager
