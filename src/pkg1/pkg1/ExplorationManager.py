import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import math
import time
import subprocess
 
class ExplorationManager(Node):
    def __init__(self):
        super().__init__('exploration_manager')
        self.setup_publishers()
        self.setup_subscribers()
        self.initialize_flags()
        self.get_logger().info("Exploration Manager initialized.")
 
    def setup_publishers(self):
        self.get_logger().info("Setting up publishers...")
        self.spin_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.explore_control_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.state_pub = self.create_publisher(Bool, 'working/state', 10)
 
    def setup_subscribers(self):
        self.get_logger().info("Setting up subscribers...")
        self.start_subscription = self.create_subscription(
            Bool, 'exploration_control/start', self.handle_start_exploration, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.state_sub = self.create_subscription(
            Bool, 'working/state', self.handle_state_change, 10)

 
    def initialize_flags(self):
        self.get_logger().info("Initializing flags...")
        self.is_spinning = False  
        self.is_exploring = False  
        self.eplisrunning = False

        self.initial_spin_done = False
        self.last_position = None
        self.total_distance = 0.0
        self.distance_threshold = 0.35  # in meters
        
 #================================================================================
# Done >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def handle_start_exploration(self, msg):
        self.get_logger().info("Handling start exploration launched")

        if msg.data:
            self.get_logger().info("Received starting exploration signal")

            if self.initial_spin_done is False:
                self.get_logger().info("Attempting to initial spin")
                self.initial_spin()

            if self.eplisrunning is False :
                self.get_logger().info("Attempting to explore")
                self.is_exploring = True  
                self.state_pub.publish(Bool(data=True))
            
        else:
            self.get_logger().info("Signal of stsart exploration not received")
            self.get_logger().info("Stopping all activities")
            self.is_spinning = False
            self.is_exploring = False
            self.spin_pub.publish(Twist())  
            self.state_pub.publish(Bool(data=False))


    def initial_spin(self):
        self.get_logger().info("Initial spinning")
        self.spin_robot()
        self.initial_spin_done = True

    def spin_robot(self):
        self.is_spinning = True
        self.get_logger().info("Starting Spin")
        twist = Twist()
        twist.angular.z = 0.3

        try:
            for i in range(30):
                self.spin_pub.publish(twist)
                time.sleep(0.5) # IMPORTANT: DON'T REMOVE THIS LINE
        finally:
            self.get_logger().info('Spin finished or interrupted.')
            self.is_spinning = False
# Done <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    def handle_state_change(self, msg):
        if not msg.data:
            self.get_logger().info("External stop received.")
            self.is_spinning = False
            self.is_exploring = False
            
            stop_spinning = Twist()
            stop_spinning.angular.z = 0.0
            stop_spinning.linear.x = 0.0
            self.spin_pub.publish(stop_spinning)  
            self.explore_control_pub.publish(Bool(data=False))  
           
        else:
            self.get_logger().info("External resume received.")
            if self.eplisrunning: 
                self.get_logger().info("Explore lite already launched, spinning and resuming exploration")
                self.is_exploring = True
                for i in range(5):
                    self.explore_control_pub.publish(Bool(data=True)) 

            else:
                self.get_logger().info("Explore lite never launched, starting exploration for the first time")
                self.start_exploration() 

    def start_exploration(self):
        self.get_logger().info("Launching Explore Lite")
        self.is_exploring = True
        try:
            self.eplisrunning = True
            process = subprocess.Popen(['ros2', 'launch', 'explore_lite', 'explore.launch.py'])
            self.get_logger().info("Explore Lite launched successfully, PID: {}".format(process.pid))
        except Exception as e:
            self.get_logger().error(f"Failed to launch Explore Lite: {str(e)}")
            self.is_exploring = False
            self.eplisrunning = False

# Done >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        #self.get_logger().info(f"Received new position: {current_position}")
        if self.last_position is not None:
            distance = self.calculate_distance(self.last_position, current_position)
            #self.get_logger().info(f"Calculated distance: {distance}, Total: {self.total_distance}")
            self.total_distance += distance
            if self.total_distance >= self.distance_threshold:
                self.get_logger().info("Threshold exceeded, initiating spin.")
                self.manage_exploration_spin()
                self.total_distance = 0.0
        self.last_position = current_position

    def calculate_distance(self, last_pos, current_pos):
        return math.sqrt((last_pos.x - current_pos.x) ** 2 + (last_pos.y - current_pos.y) ** 2)
    
    def manage_exploration_spin(self):
        if self.is_exploring:
            self.explore_control_pub.publish(Bool(data=False)) 
            self.is_exploring = False
            self.spin_robot()
            self.explore_control_pub.publish(Bool(data=True)) 
            self.is_exploring = True

# Done <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 
def main(args=None):
    rclpy.init(args=args)
    node = ExplorationManager()
    try:
        rclpy.spin(node)
    except:
        pass
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()