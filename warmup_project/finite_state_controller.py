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
print(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from person_follower import PersonFollowerNode
from obstacle_avoider import ObstacleAvoiderNode
import time
       

class FiniteStateControllerNode(Node):
    """This is a finite state controller node which inherits from Node."""
    def __init__(self):
        super().__init__("finite_state_controller_node")
        #Create timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.sub = self.create_subscription(Bump, '/bump', self.read_bump, 10)
        self.bump_state = 0
        self.last_time = -10
        self.state = 1
        self.person_follower_node = None
        self.obstacle_avoider_node = None
        self.process = None

    def run_person_follower(self):

        if self.process is not None:
            # self.process.send_signal(signal.SIGINT)
            # self.process.kill()
            os.kill(self.process.pid, signal.SIGKILL)
            # self.process.wait(timeout=0.1)
        self.process = subprocess.Popen(["ros2", "run", "warmup_project", "person_follower"], text=True)
        
        
    def run_obstacle_avoider(self):

        if self.process is not None:
            # self.process.send_signal(signal.SIGINT)
            # self.process.kill()
            os.kill(self.process.pid, signal.SIGKILL)
            # self.process.wait(timeout=0.1)
        self.process = subprocess.Popen(["ros2", "run", "warmup_project", "obstacle_avoider"], text=True)

    def run_loop(self):
        # check bump and switch between running nodes
        print(f"{time.time()},{self.state}")
        if not self.state and self.bump_state:
            self.run_obstacle_avoider()
            self.last_time = time.time()
            self.state = 1
        if self.state and time.time()-self.last_time>10:
            self.run_person_follower()
            self.state = 0
        
    def read_bump(self, msg):
        self.bump_state = 8*msg.left_front+4*msg.left_side+2*msg.right_front+msg.right_side

def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = FiniteStateControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
