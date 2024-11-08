import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Color

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.subscription = self.create_subscription(
            Color,
            '/turtle1/color_sensor',
            self.color_callback,
            10
        )

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0 
        msg.angular.z = 0.5  
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def color_callback(self, msg):
        self.get_logger().info('Received color sensor data: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
