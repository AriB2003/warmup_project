'''This script is for driving a Neato robot in a 1 meter by 1 meter square.'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3

class SendMessageNode(Node):
    """This is a mesasge sending node which inherits from Node."""
    def __init__(self):
        super().__init__("send_message_node")
        #Create timer that calls the loop every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        #Create counter to track time
        self.counter = 0
        #Create publisher
        self.publisher = self.create_publisher(Twist, "neato_square", 10)

    def run_loop(self):
        "Drives the neato in a square."
        # Increment the timer
        cmd_vel = Twist()
        # it takes 1 second for the neato to go 1 m so go for 10 increments
        if (self.counter % 30 < 10):
            cmd_vel.linear.x = 1.0
            cmd_vel.angular.z = 0.0
        else:
            # Set velocity to pi/4 so it will turn 90 degrees in exactly 2 seconds
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = .7854
        self.counter += 1
        # my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id = "odom")
        print(cmd_vel)
        print(self.counter)
        self.publisher.publish(cmd_vel)

def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = SendMessageNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
