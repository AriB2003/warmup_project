import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

bump_status = None
        

class ObstacleAvoiderNode(Node):
    """This is an obstacle avoiding node which inherits from Node."""
    def __init__(self):
        super().__init__("obstacle_avoider_node")
        
        # Create variable to store LIDAR data
        self.scan_results = None
        
        #Create timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # Create a subscriber to collect LIDAR data
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        # Create a publisher to control the Neato
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def process_scan(self, msg):
        """Reads scan data from the Neato's LIDAR sensor"""
        self.scan_results = msg.ranges
        # print(msg.ranges)
        
    def run_loop(self):
        """Calculate a vector for the Neato's velocity based on the repellant vectors 
        of all relevant LIDAR data points and its current direction of travel."""

        #Ensures a scan has been completed before writing data
        if (self.scan_results != None):
            
            sum_of_vectors = [0.0,0.0] # [x,y] - x is forward and y is left
            
            # Generate vectors from all relevant points (from -35 to 35 degrees)
            for i in range(35):
                current_vector = calculate_potential_vector(i, self.scan_results[i])
                # Add vector to Neato velocity calculation
                sum_of_vectors[0] += current_vector[0]
                sum_of_vectors[1] += current_vector[1]
                
            for i in range(325,360):
                current_vector = calculate_potential_vector(i, self.scan_results[i])
                # Add vector to Neato velocity calculation
                sum_of_vectors[0] += current_vector[0]
                sum_of_vectors[1] += current_vector[1]
            

            # Add vector to pull the robot forward
            sum_of_vectors[0] += .4

            # print(sum_of_vectors)

            # calculate linear and angular velocity from vector
            gradient_r = (sum_of_vectors[0]**2) + (sum_of_vectors[1]**2)
            gradient_theta = math.atan2(sum_of_vectors[1],sum_of_vectors[0])

            # Constrain to reasonable speeds and distances
            gradient_r = float(constrain(gradient_r, -0.1, 0.5))
            gradient_theta = float(constrain(gradient_theta, -1, 1))
            # print(gradient_theta)

            # Publish to robot
            msg = Twist()
            msg.linear.x = gradient_r
            msg.angular.z = gradient_theta
            self.publisher.publish(msg)

            
def calculate_potential_vector(angle, distance):
    """Calculates a vector that will repel the robot in a specific direction 
    based on the angle and magnitude of a LIDAR data point."""

    angle_off_heading = min(abs(360-angle), abs(0-angle)) 
    relative_weight = min((-angle_off_heading**2 + 2025), 1800) # points in the middle should be more heavily weighted
    r_scaling_factor = 4 * (10**-6)

    # Calculate direction and magnitude of repellant vector
    theta = angle+180
    r = ((r_scaling_factor)/(distance**2)) * relative_weight
    # convert polar to cartesian
    potential_vector = [r*math.cos(math.radians(theta)), r*math.sin(math.radians(theta))] # [x,y] - x is forward and y is left
    # print(potential_vector)
    return potential_vector

def constrain(val, min_val, max_val):
    """Constrains a given value to fall within a range of acceptable values."""
    if val < min_val: return min_val
    if val > max_val: return max_val
    return val

def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
