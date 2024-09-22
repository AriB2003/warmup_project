import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
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
        self.sub2 = self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.last_direction = [0]
        self.front_scan = None
        self.last_heading = 0
        self.current_heading = 0
        self.angular = 0

    def process_odom(self, msg):
        self.current_heading = 360*msg.pose.pose.orientation.z
        # print(self.current_heading)

    def process_scan(self, msg):
        self.scan_results = [8-2*v if not math.isinf(v) else 0 for v in msg.ranges]
        for i in range(-15,15):
            self.scan_results[i]+=(25-abs(i))/25
        self.front_scan = [msg.ranges[i] if not math.isinf(msg.ranges[i]) else 0.25 for i in range(-20,20)]
        # print(self.scan_results)
        
    def run_loop(self):
        "Prints a message to terminal."
        #Ensures a scan has been completed before writing data
        if (self.scan_results != None):
            direction = self.detect_person()
            hd = round(self.angular*180/math.pi/10)
            self.last_direction = [abs((d+hd)) for d in self.last_direction[-4:]]
            self.last_direction.append(direction)
            self.last_heading = self.current_heading
            # print(self.last_direction)
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
            # speed = [i/5*(180-d)/180 for i,d in enumerate(self.last_direction)]
            if sum(self.last_direction)/5>180:
                speed = [-(180-abs(d-180))/180*5 for i,d in enumerate(self.last_direction)]
            else:
                speed = [(180-abs(d-180))/180*5 for i,d in enumerate(self.last_direction)]
            # print(sum(speed)/5)
            msg = Twist()
            # self.angular = 0.35
            self.angular = sum(speed)/10
            msg.angular.z = max(min(0.7,self.angular),-0.7)
            forward = sum(self.front_scan)/len(self.front_scan)/4-0.07
            # print(self.front_scan)
            # if abs(sum(speed)/5)<0.7:
            if True:
                msg.linear.x = forward
                pass
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
            self.scan_results[min(d, len(self.scan_results)-1)] += 2+i
        kernel = [-4,-4,-4,-3,-2,-1,-1,-1,-1,1,1,1,1,2,2,4,6,8,6,4,2,2,1,1,1,1,-1,-1,-1,-2,-3,-4,-4,-4,-4]
        o = len(kernel)//2
        self.scan_results = self.scan_results[-o:]+self.scan_results+self.scan_results[:o]
        convolution = []
        for i in range(o,len(self.scan_results)-o):
            convolution.append(sum([k*v for k,v in zip(kernel, self.scan_results[i-o:i+o])]))
        # print(convolution)
        return convolution.index(max(convolution))
        

def main(args=None):
    """Initialize our node, run it, and cleanup on shutdown."""
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
