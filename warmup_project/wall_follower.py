import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math      

class WallFollowerNode(Node):
    """This is a wall following node which inherits from Node."""
    def __init__(self):
        super().__init__("wall_follower_node")
        #Create timer
        timer_period = 0.1
        self.scan_results = None
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.Kp = 0.2 # For proportional control

    def process_scan(self, msg):
        """Reads scan data from the Neato's LIDAR sensor"""
        self.scan_results = msg.ranges
        # print(msg.ranges)
        
    def run_loop(self):
        """Compares the heading of the neato to the heading of the closest wall,
        and sends a velocity messsage to the neato to correct the heading."""

        #Ensures a scan has been completed before writing data
        if (self.scan_results != None):
            # Initialize variables for heading calculation
            a = None
            b = None

            a_min = 35
            a_max = 89
            b_min = 91
            b_max = 145

            # Determine if wall is on right or left side of neato
            min_scan_dist = 99999999999
            min_scan_value = -1
            for i in range(a_min, b_max + 180):
                if (0 < self.scan_results[i] < min_scan_dist):
                    min_scan_dist = self.scan_results[i]
                    min_scan_value = i
                    # print(min_scan_value)
            if 0 <= min_scan_value < 180:
                right_wall = False # wall is on the left in this case
            else:
                right_wall = True

            # print(right_wall)
                

            # If right wall, use right side data to determine heading. Default is to left side data
            if right_wall == True:
                a_min += 180
                a_max += 180
                b_min += 180
                b_max += 180


            # Find two points that have a legitimate data point from the scan
            for i in range(a_min,a_max):
                if (self.scan_results[i] != math.inf and self.scan_results[i] != 0):
                    a = i
                    # print("a = ")
                    # print(a)
                    # print(self.scan_results[a])
                    break
            for j in range(b_min,b_max):
                if (0 < self.scan_results[j] < 10):
                    b = j
                    # print("b = ")
                    # print(b)
                    # print(self.scan_results[b])
                    break

            # Change the neato's angle to keep parallel with the wall
            try:
                # Calculate the angle between neato heading and wall direction 
                heading_numerator = (self.scan_results[a] * math.cos(math.radians(a)) - self.scan_results[b] * math.cos(math.radians(b)))
                heading_denominator = (self.scan_results[a] * math.sin(math.radians(a)) - self.scan_results[b] * math.sin(math.radians(b)))
                heading = math.atan(heading_numerator/heading_denominator)
            
                # Publish correction to neato by changing angular velocity
                if math.isnan(heading) == False:
                    msg = Twist()
                    msg.linear.x = 0.2
                    msg.angular.z = float(heading* self.Kp)
                    self.publisher.publish(msg)
                    print(True)
            except (UnboundLocalError, TypeError): # If the sensor doesn't find any values in the given range, do nothing
                print(False)
        

def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
