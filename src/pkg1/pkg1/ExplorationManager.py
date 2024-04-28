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
        self.is_spinning = False  # Indicates if the robot is currently spinning
        self.is_exploring = False  # Indicates if the robot is currently exploring
        self.eplisrunning = False
        self.robot_active = False  # Represents the overall activity state of the robot
        self.should_continue_spinning =True
        self.should_continue_exploring = True
        self.initial_spin_done = False
        self.last_position = None
        self.total_distance = 0.0
        self.distance_threshold = 0.55  # in meters
 

    def handle_start_exploration(self, msg):
        self.get_logger().info(f"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa{msg.data}")
        if msg.data:
            self.get_logger().info("Received starting exploration signal")
            self.should_continue_spinning = True
            if self.initial_spin_done is False:
                self.initial_spin()
            if self.eplisrunning is False:
                self.start_exploration()
            self.is_spinning = True  
            self.is_exploring = True  
            self.state_pub.publish(Bool(data=True))
        else:
            self.get_logger().info("FUCK YOUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU")
 
    def publish_explore_control(self, should_resume):         
        msg = Bool()        
        msg.data = should_resume        
        self.explore_control_pub.publish(msg)
 
    def manage_exploration_spin(self):
        if self.is_exploring:
            self.get_logger().info("Checking if spinning is necessary...")
            self.publish_explore_control(False)  # Pause exploration for spinning
            self.spin_robot()
            self.publish_explore_control(True)  # Resume exploration after spinning
    
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
    
    def handle_state_change(self, msg):
        if not msg.data:
            self.get_logger().info("External stop received.")
            self.should_continue_spinning = False
            self.should_continue_exploring = False
            self.is_exploring = False
            self.spin_pub.publish(Twist())  # Send zero velocities to stop spinning.
            self.explore_control_pub.publish(Bool(data=False))  # Signal exploration to pause.
            self.state_pub.publish(Bool(data=False))  # Confirm the stopped state.
        else:
            self.get_logger().info("External resume received.")
            self.should_continue_spinning = True
            self.should_continue_exploring = True
            if not self.is_exploring:
                self.start_exploration()
 
    def initial_spin(self):
        self.get_logger().info("Checking condition for initial spinning")
        self.get_logger().info("Initial spinning")
        self.spin_robot()  # Make sure spin_robot handles spinning appropriately.
        self.publish_explore_control(True)  # Consider the purpose of this control. Does it resume exploration?
        self.initial_spin_done = True
        self.should_continue_spinning = False 
 
    def start_exploration(self):
        self.get_logger().info("Launching Explore Lite")
        self.is_exploring = True
        self.eplisrunning = True
        self.state_pub.publish(Bool(data=True))  # Notify system that exploration has started
        try:
            process = subprocess.Popen(['ros2', 'launch', 'explore_lite', 'explore.launch.py'])
            self.get_logger().info("Explore Lite launched successfully, PID: {}".format(process.pid))
        except Exception as e:
            self.get_logger().error(f"Failed to launch Explore Lite: {str(e)}")
            self.is_exploring = False
            self.state_pub.publish(Bool(data=False))
 
    def spin_robot(self):
        if self.should_continue_spinning:
            self.is_spinning = True
            self.state_pub.publish(Bool(data=True))
            self.get_logger().info("Starting Spin")
            twist = Twist()
            twist.angular.z = 0.5
        try:
            for i in range(25):
                if not self.should_continue_spinning:  # Define this flag in initialize_flags if used
                    self.get_logger().info(f"Interrupting Spin at iteration {i}")
                    break
                self.spin_pub.publish(twist)
                time.sleep(0.5) # IMPORTANT: DON'T REMOVE THIS LINE
        finally:
            self.cleanup_after_spin()
 
    def cleanup_after_spin(self):
        self.get_logger().info('Spin finished or interrupted.')
        self.is_spinning = False
        # self.state_pub.publish(Bool(data=False))
 
    def calculate_distance(self, last_pos, current_pos):
        return math.sqrt((last_pos.x - current_pos.x) ** 2 + (last_pos.y - current_pos.y) ** 2)
 
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