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
        self.spin_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.explore_control_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.state_pub = self.create_publisher(Bool, 'robot/state', 10)
 
    def setup_subscribers(self):
        self.start_subscription = self.create_subscription(
            Bool, 'exploration_control/start', self.handle_start_exploration, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.state_sub = self.create_subscription(
            Bool, 'robot/state', self.handle_state_change, 10)
 
    def initialize_flags(self):
        self.is_spinning = False  # Indicates if the robot is currently spinning
        self.is_exploring = False  # Indicates if the robot is currently exploring
        self.robot_active = False  # Represents the overall activity state of the robot
        
        self.initial_spin_done = False
        self.last_position = None
        self.total_distance = 0.0
        self.distance_threshold = 0.35  # in meters
 
# ======================================Call Backs==================================================
 
    def handle_start_exploration(self, msg):
        self.get_logger().info(f"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa{msg.data}")
        if msg.data:
            self.get_logger().info("Received starting exploration signal")
            if self.initial_spin_done is False:
                self.initial_spin()
            self.is_spinning = True  
            self.is_exploring = True  
            self.state_pub.publish(Bool(data=True))
            self.start_exploration()
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
        # React to the state message indicating whether to stop or resume activities.
        if msg.data:
            # Resume all previously active operations.
            self.get_logger().info("State change received: Resume all activities.")
            if not self.is_spinning:
                self.is_spinning = True
                self.spin_robot()  # Ensure this method sets up spinning properly.
            if not self.is_exploring:
                self.is_exploring = True
                self.publish_explore_control(True)  # Resume exploration.
            self.state_pub.publish(Bool(data=True))  # Confirm the active state.
        else:
            # Stop all currently active operations.
            self.get_logger().info("State change received: Stop all activities.")
            self.is_spinning = False
            self.is_exploring = False
            self.spin_pub.publish(Twist())  # Stop any spinning.
            self.publish_explore_control(False)  # Signal exploration to pause.
            self.state_pub.publish(Bool(data=False))  # Confirm the stopped state.
 
 
# ======================================Actrual Functions==================================================
 
    def initial_spin(self):
        self.get_logger().info("Checking condition for initial spinning")
        self.get_logger().info("Initial spinning")
        self.spin_robot()  # Make sure spin_robot handles spinning appropriately.
        self.publish_explore_control(True)  # Consider the purpose of this control. Does it resume exploration?
        self.initial_spin_done = True
 
    def start_exploration(self):
        self.get_logger().info("Launching Explore Lite")
        self.is_exploring = True
        self.exploration_state_pub.publish(Bool(data=True))  # Notify system that exploration has started
        try:
            process = subprocess.Popen(['ros2', 'launch', 'explore_lite', 'explore.launch.py'])
            self.get_logger().info("Explore Lite launched successfully, PID: {}".format(process.pid))
        except Exception as e:
            self.get_logger().error(f"Failed to launch Explore Lite: {str(e)}")
            self.is_exploring = False
            self.exploration_state_pub.publish(Bool(data=False))
 
    def spin_robot(self):
        self.is_spinning = True
        self.spin_state_pub.publish(Bool(data=True))
        self.get_logger().info("Starting Spin")
        twist = Twist()
        twist.angular.z = 0.5
        try:
            for i in range(30):
                if not self.should_continue_spinning:  # Define this flag in initialize_flags if used
                    self.get_logger().info(f"Interrupting Spin at iteration {i}")
                    break
                self.spin_pub.publish(twist)
        finally:
            self.cleanup_after_spin()
 
    def cleanup_after_spin(self):
        self.get_logger().info('Spin finished or interrupted.')
        self.is_spinning = False
        self.spin_state_pub.publish(Bool(data=False))
 
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