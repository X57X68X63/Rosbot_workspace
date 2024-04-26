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
 
    def setup_publishers(self):
        self.spin_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.explore_control_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.spin_state_pub = self.create_publisher(Bool, 'robot/spinning', 10) # Tells you if the robot is spinning
        self.exploration_state_pub = self.create_publisher(Bool, 'robot/exploring', 10) # tells your if the robot is exploring
 
    def setup_subscribers(self):
        self.subscription = self.create_subscription(
            Bool, 'exploration_control/start', self.handle_start_exploration, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.stop_command_sub = self.create_subscription(
            Bool, 'command/stop', self.handle_stop_command, 10) # handles when you tell me to stop
        self.resume_command_sub = self.create_subscription(
            Bool, 'command/resume', self.handle_resume_command, 10) # handles when you tell me to continue
 
    def initialize_flags(self):
        self.is_spinning = False
        self.is_exploring = False
        self.initial_spin_done = False
        self.should_continue_spinning = True
        self.last_position = None
        self.total_distance = 0.0
        self.distance_threshold = 0.35  # in meters
 
 
# ========================================================================================
 
    def initial_spin(self):
        self.get_logger().info("Checking condition for initial spinning")
        self.get_logger().info("Initial spinning")
        self.spin_robot()
        self.publish_explore_control(True)
        self.initial_spin_done = True
 
 
    def handle_start_exploration(self, msg):
        if msg.data:
            self.get_logger().info("Received starting exploration signal")
            self.should_continue_spinning = True  # Enable spinning at the start
            if self.initial_spin_done is False:
                self.initial_spin()
            self.start_exploration()
 
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
 
    def manage_exploration_spin(self):
        if self.is_exploring:
            self.get_logger().info("Checking if spinning is necessary...")
            self.publish_explore_control(False)  # Pause exploration for spinning
            self.spin_robot()
            self.publish_explore_control(True)  # Resume exploration after spinning
 
    def publish_explore_control(self, data):
        msg = Bool()
        msg.data = data
        self.explore_control_pub.publish(msg)
 
 
    def spin_robot(self):
        if self.should_continue_spinning:
            self.is_spinning = True
            self.spin_state_pub.publish(Bool(data=True))
            self.get_logger().info("Starting Spin")
            twist = Twist()
            twist.angular.z = 1.0
        try:
            for i in range(30):
                if not self.should_continue_spinning:
                    self.get_logger().info(f"Interrupting Spin at iteration {i}")
                    break
                self.spin_pub.publish(twist)
                time.sleep(0.2)
        finally:
            self.cleanup_after_spin()
 
    def cleanup_after_spin(self):
        self.get_logger().info('Spin finished or interrupted.')
        self.is_spinning = False
        self.spin_state_pub.publish(Bool(data=False))
 
    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        # self.get_logger().info(f"Received new position: {current_position}")
        if self.last_position is not None:
            distance = self.calculate_distance(self.last_position, current_position)
            # self.get_logger().info(f"Calculated distance: {distance}, Total: {self.total_distance}")
            self.total_distance += distance
            if self.total_distance >= self.distance_threshold:
                self.get_logger().info("Threshold exceeded, initiating spin.")
                self.manage_exploration_spin()
                self.total_distance = 0.0
        self.last_position = current_position
 
 
    def calculate_distance(self, last_pos, current_pos):
        return math.sqrt((last_pos.x - current_pos.x) ** 2 + (last_pos.y - current_pos.y) ** 2)
 
    def handle_stop_command(self, msg): # stopping spin
        if msg.data:
            self.should_continue_spinning = False
            self.get_logger().info('Stop command received, interrupting spin.')
 
    def handle_resume_command(self, msg):
        if msg.data:
            self.should_continue_spinning = True  # Resume spinning when commanded
            self.get_logger().info('Resume command received, resuming operations.')
 
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