import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import numpy as np

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.start_time = time.time()
        self.phase = 'forward'  # Phases: 'forward', 'backward', 'explore'
        self.forward_start = time.time()
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.min_distance = 0.3  # Minimum distance to wall
        self.get_logger().info('Explorer node started')

    def lidar_callback(self, msg):
        if time.time() - self.start_time > 600:  # 10 minutes max
            self.stop()
            self.get_logger().info('Time limit reached, stopping')
            return

        # Get LIDAR ranges in [-45°, 45°]
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        front_ranges = ranges[(angles >= -np.pi/4) & (angles <= np.pi/4)]
        right_ranges = ranges[(angles >= -np.pi/2) & (angles < -np.pi/4)]
        left_ranges = ranges[(angles > np.pi/4) & (angles <= np.pi/2)]

        cmd = Twist()

        if self.phase == 'forward':
            # Move forward for 60 seconds
            if time.time() - self.forward_start < 60:
                cmd.linear.x = self.linear_speed
                if len(front_ranges) > 0 and min(front_ranges) < self.min_distance:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.angular_speed  # Turn left if obstacle
                self.get_logger().info('Moving forward')
            else:
                self.phase = 'backward'
                self.forward_start = time.time()
            self.publisher.publish(cmd)

        elif self.phase == 'backward':
            # Move backward for 60 seconds
            if time.time() - self.forward_start < 60:
                cmd.linear.x = -self.linear_speed
                self.get_logger().info('Moving backward')
            else:
                self.phase = 'explore'
                self.forward_start = time.time()
            self.publisher.publish(cmd)

        else:
            # Wall-following exploration (right wall)
            if len(right_ranges) > 0 and len(front_ranges) > 0:
                min_right = min(right_ranges) if min(right_ranges) < msg.range_max else msg.range_max
                min_front = min(front_ranges) if min(front_ranges) < msg.range_max else msg.range_max

                if min_front < self.min_distance:
                    cmd.angular.z = self.angular_speed  # Turn left to avoid obstacle
                elif min_right < self.min_distance:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = -0.2  # Turn slightly left to move away from wall
                elif min_right > self.min_distance + 0.2:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.2  # Turn slightly right to approach wall
                else:
                    cmd.linear.x = self.linear_speed  # Move forward along wall
                self.get_logger().info(f'Exploring: min_right={min_right:.2f}, min_front={min_front:.2f}')
            else:
                cmd.linear.x = self.linear_speed  # Move forward if no data
            self.publisher.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.publisher.publish(cmd)
        self.get_logger().info('Stopped')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
