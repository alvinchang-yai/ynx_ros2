import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import threading
import time

from robot_manager_interfaces.action import PoseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class PoseGoalExample(Node):
    def __init__(self):
        super().__init__('pose_goal_example')

        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value) + "/"
        
        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "pose_goal")
        self.pose_goal_client.wait_for_server()

def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalExample()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # Create Goal
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = Pose(
                position=Point(x=0.6, y=0.0, z=0.6),
                orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
                )
        goal_msg.velocity_scaling = 0.1
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = "" # If empty -> base_link used
        goal_msg.target_id = "" # If empty -> tool0 used
        goal_msg.method = "PTP"

        # Blocking Execution without Feedback
        node.get_logger().info("Starting Execution")
        node.pose_goal_client.send_goal(goal_msg)
        node.get_logger().info("Finished Execution")

        # Asynchronous Execution with Feedback
        goal_msg.target_pose.position.y = 0.3
        goal_msg.method = "LIN"
        node.get_logger().info("Starting Execution")
        send_future = node.pose_goal_client.send_goal_async(goal_msg, feedback_callback=lambda msg: node.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        while not send_future.done(): continue # Wait for Goal to be accepted
        result = send_future.result().get_result() # Wait for Goal to be executed
        if not result.result.success: node.get_logger().error(result.result.message)
        else: node.get_logger().info("Finished Execution")

        # Cancelling Execution (works but errors the robot)
        goal_msg.target_pose.position.y = -0.3
        node.get_logger().info("Starting Execution")
        send_future = node.pose_goal_client.send_goal_async(goal_msg, feedback_callback=lambda msg: node.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        while not send_future.done(): continue # Wait for Goal to be accepted
        # time.sleep(1.0)
        # send_future.result().cancel_goal()
        result = send_future.result().get_result() # Wait for Goal to be executed
        if not result.result.success: node.get_logger().error(result.result.message)
        else: node.get_logger().info("Finished Execution")

    except KeyboardInterrupt:
        node.get_logger().info("Script interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
