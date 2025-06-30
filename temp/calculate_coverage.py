import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class CoverageNode(Node):
    def __init__(self):
        super().__init__('coverage_node')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.total_cells = 40000  # 10m x 10m at 0.05m resolution
        self.get_logger().info('Coverage node started')

    def map_callback(self, msg):
        non_negative_cells = sum(1 for cell in msg.data if cell >= 0)
        coverage = (non_negative_cells / self.total_cells) * 100
        self.get_logger().info(f'Coverage: {coverage:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
