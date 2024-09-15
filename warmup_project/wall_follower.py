import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math

bump_status = None
        

class WallFollowerNode(Node):
    """This is a wall following node which inherits from Node."""
    def __init__(self):
        super().__init__("estop_node")
        #Create timer
        timer_period = 0.1
        self.scan_results = None
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def process_scan(self, msg):
        self.scan_results = msg.ranges
        print(msg.ranges)
        
    def run_loop(self):
        "Prints a message to terminal."
        #Ensures a scan has been completed before writing data
        if (self.scan_results != None):
            
            # Determine if wall is on right or left side of neato
            # min_scan_dist = 99999999999
            # min_scan_value = None
            # for i in range(len(self.scan_results)):
            #     if (0 < self.scan_results[i] < min_scan_dist):
            #         min_scan_dist = self.scan_results[i]
            #         min_scan_value = i
            # write try/except statement to account for cases where all values are inf and 0?
            # Calculate the angle between neato heading and wall direction
            a = None
            b = None
            
            for i in range(215,235):
                if (self.scan_results[i] != math.inf and self.scan_results[i] != 0):
                    a = i
                    # print("a = ")
                    # print(a)
                    # print(self.scan_results[a])
                    break
            for j in range(305,325):
                if (0 < self.scan_results[j] < 10):
                    b = j
                    # print("b = ")
                    # print(b)
                    # print(self.scan_results[b])
                    break
            # try:
            # print("made it")        
            heading_numerator = (self.scan_results[a] * math.cos(math.radians(a)) - self.scan_results[b] * math.cos(math.radians(b)))
            heading_denominator = (self.scan_results[a] * math.sin(math.radians(a)) - self.scan_results[b] * math.sin(math.radians(b)))
            heading = math.atan(heading_numerator/heading_denominator)

            print(heading)
        
            # Publish correction to neato by changing angular velocity
            if math.isnan(heading) == False:
                msg = Twist()
                msg.linear.x = 0.2
                msg.angular.z = float(heading/5)
                self.publisher.publish(msg)
                print(msg)
            # except UnboundLocalError or TypeError:
            #     msg = Twist()
            #     msg.linear.x = 0.05
            #     msg.angular.z = 0.0
            #     self.publisher.publish(msg)
            #     print(msg)
        

def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
