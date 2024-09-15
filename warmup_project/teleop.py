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

settings = termios.tcgetattr(sys.stdin)
tty.setraw(sys.stdin.fileno())

def getKey():
    key = None
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        key = sys.stdin.read(1)
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)
    return key

class TeleopNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendPointNode. No inputs."""
        super().__init__('teleop_node')
        # Create a timer that fires ten times per second
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.lin = 0
        self.ang = 0
        self.lin_inc = 0.15
        self.ang_inc = 0.2
    
    def run_loop(self):
        """Does Teleop."""
        key = getKey()
        if key == '\x03':
            global settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            raise KeyboardInterrupt
        if key == 'w':
            self.lin+=self.lin_inc
            self.lin = max(self.lin, 0)
        if key == 's':
            self.lin-=self.lin_inc
            self.lin = min(self.lin, 0)
        if key == 'a':
            self.ang+=self.ang_inc
            self.ang = max(self.ang, 0)
        if key == 'd':
            self.ang-=self.ang_inc
            self.ang = min(self.ang, 0)
        self.control()
        print(key)     

    def control(self):
        self.lin = min(0.5, max(-0.5, self.lin))
        self.ang = min(0.5, max(-0.5, self.ang))
        total = self.lin+self.ang or 1
        self.lin = self.lin*min(1,self.lin/total)
        self.ang = self.ang*min(1,self.ang/total)
        cmd_vel = Twist()
        cmd_vel.linear.x = float(self.lin)
        cmd_vel.angular.z = float(self.ang)
        # print(cmd_vel)
        self.publisher.publish(cmd_vel)
        self.lin/=1.2
        self.ang/=1.2


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = TeleopNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup
    global settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
