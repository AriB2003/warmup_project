import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import math
import sys
import os
import subprocess
import signal
import time


class FiniteStateControllerNode(Node):
    """This is a finite state controller node which inherits from Node."""

    def __init__(self):
        super().__init__("finite_state_controller_node")
        # Create timer
        timer_period = 0.1
        self.timer0 = self.create_timer(timer_period, self.run_loop)
        # Read the bump
        self.sub = self.create_subscription(Bump, '/bump', self.read_bump, 10)
        # Save state variables
        self.bump_state = 0
        self.last_time = -10
        self.state = 1

        # Person Follower
        # Create timer
        timer_period = 0.1
        self.timer1 = self.create_timer(timer_period, self.run_loop)
        self.timer2 = self.create_timer(timer_period, self.run_back_away)
        self.timer3 = self.create_timer(timer_period, self.run_person_follower)
        # Create subscriptions to LaserScan
        self.sub1 = self.create_subscription(
            LaserScan, 'scan', self.process_scan, 10)
        # Create publisher for Twist commands
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_results = None  # save the scan to a list
        # save the most resent directions of interest
        self.last_direction = [0]
        self.front_scan = None  # save the front subset of the scan
        self.angular = 0  # save the angular speed

    def run_loop(self):
        """Main body of the FSM for switching states."""
        print(f"{time.time()},{self.state}")
        # If person followering and a bump occurs, switch state
        if not self.state and self.bump_state:
            self.last_time = time.time()
            self.state = 1
        # If backing away and have been for more than 4 seconds, switch state
        elif self.state and time.time() - self.last_time > 4:
            self.state = 0

    def read_bump(self, msg):
        self.bump_state = 8 * msg.left_front + 4 * \
            msg.left_side + 2 * msg.right_front + msg.right_side

    # Person Follower
    def process_scan(self, msg):
        """Get the scan from the robot and pre-process it."""
        # Make the scan weigh the closer values higher
        self.scan_results = [
            8 - 2 * v if not math.isinf(v) else 0 for v in msg.ranges]
        # Bias the +-15 degree window at the front of the robot so that it
        # doesn't wander
        for i in range(-15, 15):
            self.scan_results[i] += (25 - abs(i)) / 25
        # Save the +-20 degree front scan
        self.front_scan = [msg.ranges[i] if not math.isinf(
            msg.ranges[i]) else 0.25 for i in range(-20, 20)]
        # print(self.scan_results)

    def run_person_follower(self):
        """Determine the movement of the robot to follow a person."""
        # Ensures a scan has been completed before processing data
        if (self.scan_results is not None):
            # Detect likeliest direction of the person.
            direction = self.detect_person()
            # Calculate the difference in heading based on set speed for 0.1
            # seconds
            hd = round(self.angular * 180 / math.pi / 10)
            # Correct the list of directions of interest using the heading
            # difference and only keep  historical values
            self.last_direction = [abs((d + hd))
                                   for d in self.last_direction[-4:]]
            # Save the current direction to the list of recent directions of
            # interest
            self.last_direction.append(direction)

            # Calculate the average of the last headings of interest to
            # determine direction of rotation
            if sum(self.last_direction) / 5 > 180:
                # Perform the angle to speed calculation (180 fastest, 0/360
                # slowest)
                speed = [-(180 - abs(d - 180)) / 180 *
                         5 for d in self.last_direction]
            else:
                speed = [(180 - abs(d - 180)) / 180 *
                         5 for d in self.last_direction]

            # Build the Twist command
            msg = Twist()
            # Average the requested speeds and scale down
            self.angular = sum(speed) / 10
            # Bound the speed to a reasonable range
            msg.angular.z = max(min(0.7, self.angular), -0.7)
            # Calculate forward velocity based on forward scan proximity
            forward = sum(self.front_scan) / len(self.front_scan) / 4 - 0.07
            msg.linear.x = forward
            if not self.state:
                self.publisher.publish(msg)

    def detect_person(self):
        """Detect a person in the scan with high likelihood."""
        # Bias the scan where the person was last seen
        for i, d in enumerate(self.last_direction):
            self.scan_results[min(d, len(self.scan_results) - 1)] += 2 + i
        # Apply a kernel and convolve it through the scan
        # The kernel biases the robot to be afraid of very large objects
        kernel = [-4, -4, -4, -3, -2, -1, -1, -1, -1, 1, 1, 1, 1, 2, 2, 4,
                  6, 8, 6, 4, 2, 2, 1, 1, 1, 1, -1, -1, -1, -2, -3, -4, -4, -4, -4]
        o = len(kernel) // 2
        self.scan_results = self.scan_results[-o:] + \
            self.scan_results + self.scan_results[:o]
        convolution = []
        for i in range(o, len(self.scan_results) - o):
            convolution.append(
                sum([k * v for k, v in zip(kernel, self.scan_results[i - o:i + o])]))
        # Return the largest value from the convolution to correspond with
        # direction
        return convolution.index(max(convolution))

    # Back Away
    def run_back_away(self):
        "Drives the neato backwards."
        # initialize a velocity message for the neato
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.1
        cmd_vel.angular.z = 0.0
        if self.state:
            self.publisher.publish(cmd_vel)


def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = FiniteStateControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
