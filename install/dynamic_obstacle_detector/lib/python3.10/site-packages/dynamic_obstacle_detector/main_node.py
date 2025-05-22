import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Example message type

class DynamicObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_detector_node')
        self.publisher_ = self.create_publisher(String, 'obstacle_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot status: All clear! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communication

    node = DynamicObstacleDetectorNode() # Create the node

    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        pass # Allow graceful exit on Ctrl+C

    node.destroy_node() # Destroy the node
    rclpy.shutdown() # Shutdown ROS 2 communication