import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class DualControl(Node):

    def __init__(self):
        super().__init__('dual_control')

        # Gazebo publisher
        self.gz_pub = self.create_publisher(
            Twist,
            '/model/iris_with_standoffs/cmd_vel',
            10
        )

        # MAVROS publisher
        self.mavros_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        # subscriber
        self.sub = self.create_subscription(
            Twist,
            '/control/cmd_vel',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):

        self.gz_pub.publish(msg)
        self.mavros_pub.publish(msg)

def main():
    rclpy.init()

    node = DualControl()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
