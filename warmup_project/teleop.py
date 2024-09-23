""" This script explores publishing ROS messages in ROS using Python """
import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist

# Pull the settings from the terminal and then override them
settings = termios.tcgetattr(sys.stdin)
tty.setraw(sys.stdin.fileno())


def getKey():
    """Get non-blocking key press in terminal."""
    key = None
    # Check for data present and read data
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        key = sys.stdin.read(1)
    # Flush the data from the input buffer
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)
    return key


class TeleopNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""

    def __init__(self):
        """Initializes the Teleop Node. No inputs."""
        super().__init__('teleop_node')
        # Create a timer that fires ten times per second
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        # Create a Twist publisher
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Track linear and angular speed
        self.lin = 0
        self.ang = 0
        self.lin_inc = 0.15
        self.ang_inc = 0.2

    def run_loop(self):
        """Perform teleop processing and send commands."""
        key = getKey()
        # Raise KeyboardInterrupt when ctrl+C
        if key == '\x03':
            global settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            raise KeyboardInterrupt
        # Standard WASD control schema
        if key == 'w':
            self.lin += self.lin_inc
            self.lin = max(self.lin, 0)
        if key == 's':
            self.lin -= self.lin_inc
            self.lin = min(self.lin, 0)
        if key == 'a':
            self.ang += self.ang_inc
            self.ang = max(self.ang, 0)
        if key == 'd':
            self.ang -= self.ang_inc
            self.ang = min(self.ang, 0)
        self.control()
        print(key)

    def control(self):
        """Manage the Neato speed control."""
        # Bound the speeds to the range of -0.5 to 0.5
        self.lin = min(0.5, max(-0.5, self.lin))
        self.ang = min(0.5, max(-0.5, self.ang))
        # Calculate the total speed
        total = self.lin+self.ang or 1
        # Scale the speeds based on their contribution for normalization
        self.lin = self.lin * min(1, self.lin / total)
        self.ang = self.ang * min(1, self.ang / total)
        # Build and send the Twist command
        cmd_vel = Twist()
        cmd_vel.linear.x = float(self.lin)
        cmd_vel.angular.z = float(self.ang)
        # print(cmd_vel)
        self.publisher.publish(cmd_vel)
        # Decay the speed exponentially so that it slows down naturally
        self.lin /= 1.2
        self.ang /= 1.2


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = TeleopNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup
    # restore terminal settings
    global settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
