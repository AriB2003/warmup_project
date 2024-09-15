import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
       

class PersonFollowerNode(Node):
    """This is a wall following node which inherits from Node."""
    def __init__(self):
        super().__init__("person_follower_node")
        #Create timer
        timer_period = 0.1
        self.scan_results = None
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.last_direction = [0]
        self.front_scan = None

    def process_scan(self, msg):
        self.scan_results = [4-v if not math.isinf(v) else 0 for v in msg.ranges]
        self.front_scan = [msg.ranges[i] if not math.isinf(msg.ranges[i]) else 0.25 for i in range(-15,15)]
        # print(self.scan_results)
        
    def run_loop(self):
        "Prints a message to terminal."
        #Ensures a scan has been completed before writing data
        if (self.scan_results != None):
            direction = self.detect_person()
            self.last_direction.append(direction)
            self.last_direction = self.last_direction[-5:]
            # print(direction)
            # Determine if wall is on right or left side of neato
            # min_scan_dist = 99999999999
            # min_scan_value = None
            # for i in range(len(self.scan_results)):
            #     if (0 < self.scan_results[i] < min_scan_dist):
            #         min_scan_dist = self.scan_results[i]
            #         min_scan_value = i
            # write try/except statement to account for cases where all values are inf and 0?
            # Calculate the angle between neato heading and wall direction
        
            # Publish correction to neato by changing angular velocity
            speed = [i/2*(180-d)/180 for i,d in enumerate(self.last_direction)]
            # print(speed)
            msg = Twist()
            # msg.linear.x = 0.2
            msg.angular.z = sum(speed)/5
            forward = sum(self.front_scan)/len(self.front_scan)-0.5
            print(self.front_scan)
            if abs(sum(speed)/5)<0.35:
                msg.linear.x = forward
            else:
                msg.linear.x = 0.0
            self.publisher.publish(msg)
            # print(msg)
            # except UnboundLocalError or TypeError:
            #     msg = Twist()
            #     msg.linear.x = 0.05
            #     msg.angular.z = 0.0
            #     self.publisher.publish(msg)
            #     print(msg)

    def detect_person(self):
        # print(self.last_direction)
        for i, d in enumerate(self.last_direction):
            self.scan_results[min(d, len(self.scan_results)-1)] = 2+i
        kernel = [-8,-7,-6,-5,-4,-3,-2,-1,-1,1,1,1,2,4,5,4,2,1,1,1,-1,-1,-2,-3,-4,-5,-6,-7,-8]
        o = len(kernel)//2
        self.scan_results = self.scan_results[-o:]+self.scan_results+self.scan_results[:o]
        convolution = []
        for i in range(o,len(self.scan_results)-o):
            convolution.append(sum([k*v for k,v in zip(kernel, self.scan_results[i-o:i+o])]))
        return convolution.index(max(convolution))
        

def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
