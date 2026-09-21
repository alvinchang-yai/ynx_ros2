import rclpy
import time
from robot_manager_interfaces.srv import SetIo

class Io():
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('io_example')
        self.node.declare_parameter('ns', '')
        self.ns = self.node.get_parameter("ns").value
        self.topic = ""
        if self.ns != "":
            self.topic = "/" + str(self.ns)
        self.io_client = self.node.create_client(SetIo, self.topic + '/set_io')
        while not self.io_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

def log_result(io, future, state):
    res = future.result()
    if res is None:
        io.node.get_logger().error('Service call failed: %s' % future.exception())
    elif res.success:
        io.node.get_logger().info('Set pin to %d: %s' % (state, res.message))
    else:
        io.node.get_logger().error('Set pin to %d failed: %s' % (state, res.message))

def main(args=None):
    io = Io()
    req = SetIo.Request()
    req.pin = 1 # 1 for do1
    req.state = 1 # 1 for ON, 0 for OFF
    
    # Send the async request
    future = io.io_client.call_async(req)
    rclpy.spin_until_future_complete(io.node, future)
    log_result(io, future, req.state)

    time.sleep(5)

    req.state = 0 # 1 for ON, 0 for OFF
    future = io.io_client.call_async(req)
    rclpy.spin_until_future_complete(io.node, future)
    log_result(io, future, req.state)

    io.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
