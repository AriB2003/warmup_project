""" This script explores publishing ROS messages in ROS using Python """
import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    key = None
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Teleop(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendPointNode. No inputs."""
        super().__init__('teleop')
        # Create a timer that fires ten times per second
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.publisher = self.create_publisher(Marker, 'my_marker', 10)

    def run_loop(self):
        """Does Teleop."""
        key = getKey()
        if key == '\x03':
            raise KeyboardInterrupt   
        print(key)     

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = Teleop()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
