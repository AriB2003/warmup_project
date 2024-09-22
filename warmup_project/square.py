'''This script is for driving a Neato robot in a 1 meter by 1 meter square.'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveSquareNode(Node):
    """This is a node for driving the neato in a 1m by 1m square. Inherits from Node."""

    def __init__(self):
        super().__init__("drive_square_node")
        # Create timer that calls the loop every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_square_loop)
        # Create counter to track time
        self.counter = 0
        # Create publisher to neato's velocity topic
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def run_square_loop(self):
        "Drives the neato in a square."
        # initialize a velocity message for the neato
        cmd_vel = Twist()
        # it takes 5 seconds for the neato to go 1 m so go for 50 increments
        if (self.counter % 100 < 50):
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.0
        # wait for a second to allow the neato to stop fully
        elif (self.counter % 100 < 60):
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        elif (self.counter % 100 < 90):
            # Set velocity to pi/6 so it will turn 90 degrees in exactly 3
            # seconds
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = .54
        # wait for a second to allow the neato to stop fully
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        # increment counter
        self.counter += 1

        # publish message
        print(cmd_vel)
        print(self.counter)
        self.publisher.publish(cmd_vel)


def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
