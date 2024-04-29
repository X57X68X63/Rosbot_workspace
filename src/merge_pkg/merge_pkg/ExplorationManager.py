import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose, Vector3
from std_msgs.msg import Bool, Header, ColorRGBA
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from tf2_ros import TransformListener, Buffer
from find_object_2d.msg import ObjectsStamped 
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
import math
import time
import subprocess

# Global variables: markers
markers = {
    1: "Explosives",
    2: "Flammable Gas",
    3: "Non-Flammable Gas",
    4: "Dangous When Wet",
    5: "Flammable Solid",
    6: "Combustible",
    7: "Oxidizer",
    8: "Orgnic Peroxide",
    9: "Inhalation Hazards",
    10: "Posion",
    11: "Radioactive"
}


class ExplorationManager(Node):
    def __init__(self):
        super().__init__('exploration_manager')
        self.setup_subscribers()
        self.setup_publishers()
        self.initial_flags()
        
    # -------------------------------------------SETUP FUNCTIONS-------------------------------------------
    def setup_subscribers(self):
        # receive start signal from start_check node
        self.start_sub = self.create_subscription(
            Bool, 'exploration_control/start', self.start_all_callback, 10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.obj_sub = self.create_subscription(
            ObjectsStamped, '/objectsStamped', self.object_callback, 10)
        
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
    def setup_publishers(self):
        self.explore_lite_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hazard_pub = self.create_publisher(Marker, '/hazards', 10)
    
    def initial_flags(self):
        # Flags only change when start signal is received
        self.initial_spin_done = False
        self.explore_lite_launched = False
        self.robot_started = False
        # Flags that change during exploration
        self.is_spinning = False
        self.is_exploring = False
        
        self.last_position = None
        self.total_distance = 0.0
        self.distance_threshold = 0.35  # in meters
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.object_checked = []
        self.marked_objects = {}
        self.hazard_markers = []
        
        self.object_distence = -1
        
    # -------------------------------------------CALLBACK FUNCTIONS-------------------------------------------
    # Only true will receive from start_check node
    def start_all_callback(self, msg):
        self.get_logger().info('Single received from start_check node.')
       
        # if singal is true
        if msg.data: 
            self.get_logger().info('Start signal received, handling start signal.')
            self.robot_started = True
            
            # if never do initial spin
            if self.initial_spin_done is False:
                self.get_logger().info('Initial spin not done, doing initial spin.')
                self.initial_spin_done = True
                self.spin_robot()
            
            # check if explore lite launched
            if self.explore_lite_launched is False:
                self.get_logger().info('Explore lite not launched, launching explore lite.')
                self.launch_explore_lite() # <--- explore_lite launched true if successful else false
        
        # if signal is false, something went wrong, ignore signal
        else: 
            self.get_logger().info('False received, ignoring signal.')
    
    
    # Callback for distance calculation
    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        
        if self.last_position is not None:
            distance = math.sqrt((self.last_position.x - current_position.x) ** 2 + (self.last_position.y - current_position.y) ** 2)
            self.total_distance += distance
            
            if self.total_distance >= self.distance_threshold:
                self.get_logger().info("Threshold exceeded, initiating spin.")
                
                if self.is_exploring and not self.is_spinning: # if exploring and not spinning
                    # stop explore lite
                    self.explore_lite_pub.publish(Bool(data=False))
                    self.is_exploring = False
                    
                    # spin robot
                    self.spin_robot()
                    
                    # resume explore lite
                    self.explore_lite_pub.publish(Bool(data=True))
                    self.is_exploring = True
                    
                self.total_distance = 0.0
        self.last_position = current_position
        
        
    def laser_callback(self, msg):
        self.object_distence = msg.ranges[0]
        self.last_scan_time = msg.header.stamp
        
    
    def object_callback(self, msg):
        if self.robot_started is False:
            self.get_logger().info('Robot not started, ignoring object detection.')
            return
        
        data = []
        if 12 >= len(msg.objects.data) > 0:
            data = msg.objects.data
        elif len(msg.objects.data) > 12:
            data = msg.objects.data[:12]
            
        if len(data) > 0 and self.object_availabile_to_mark(data[0]):
            self.get_logger().info('Object detected, marking object.')
            
            # check if robot running other tasks
            if self.is_exploring and not self.is_spinning:
                # stop explore lite
                self.get_logger().info('Pausing explore lite.')
                self.explore_lite_pub.publish(Bool(data=False))
                self.is_exploring = False
            elif self.is_spinning and not self.is_exploring:
                self.get_logger().info('Pausing spin.')
                self.is_spinning = False
            
            obj = {
                'id': data[0],
                'width': data[1],
                'height': data[2],
                'dx': data[9],
                'dy': data[10],
                'h': data[3:12]
            }
            self.rotate_to_center_object(obj)
        
        else: 
            if not self.is_exploring and not self.is_spinning and self.explore_lite_launched:
                self.get_logger().info('Resuming explore lite.')
                self.get_logger().info('Current object checked: {}'.format(self.object_checked))
                self.explore_lite_pub.publish(Bool(data=True))
                self.is_exploring = True
            elif not self.explore_lite_launched:
                self.get_logger().info('Explore lite not launched, launching explore lite.')
                self.launch_explore_lite()
            else:
                self.get_logger().info('Object not avilable or already marked, ignoring.')
    
    
    # -------------------------------------------HELPER FUNCTIONS-------------------------------------------
    def spin_robot(self):
        self.get_logger().info('Spinning robot.')
        self.is_spinning = True
        twist = Twist()
        twist.angular.z = 0.3
        
        cycle = 0
        while cycle < 30 and self.is_spinning:
            self.twist_pub.publish(twist)
            time.sleep(0.5)
            cycle += 1
       
        twist.angular.z = 0.0
        self.twist_pub.publish(twist)
        self.is_spinning = False
        
        
    def launch_explore_lite(self):
        self.get_logger().info("Launching Explore Lite")
        try:
            process = subprocess.Popen(['ros2', 'launch', 'explore_lite', 'explore_lite.launch.py'])
            self.get_logger().info("Explore Lite launched successfully, PID: {}".format(process.pid))
            self.explore_lite_launched = True
            self.is_exploring = True
        except Exception as e:
            self.get_logger().error("Failed to launch Explore Lite: {}".format(str(e)))
            self.explore_lite_launched = False
            self.is_exploring = False
            
    
    def object_availabile_to_mark(self, obj_id):
        if obj_id in markers.keys() and obj_id not in self.object_checked:
            return True
        else:
            return False
        
    
    def rotate_to_center_object(self, obj):
        self.get_logger().info("Starting rotation in rotate_to_center_object")

        # QTransform 
        H = np.array(obj['h']).reshape(3, 3)

        obj_center_x_ = obj['width'] / 2
        obj_center_y_ = obj['height'] / 2

        obj_center_x = obj_center_x_ * H[0][0] + obj_center_y_ * H[1][0] + obj['dx']
        obj_center_y = obj_center_y_ * H[1][1] + obj_center_x_ * H[0][1] + obj['dy']

        w_ = obj_center_x_ * H[0][2] + obj_center_y_ * H[1][2] + H[2][2]

        obj_center_x /= w_
        obj_center_y /= w_
        
        center_x = obj_center_x

        # center_x = obj['dx'] + obj['width'] // 2

        err_x = 319 - center_x
        self.get_logger().info(f"Object center: {center_x}, error: {err_x}")

        k = 1/319
        threshold = 3
        twist_msg = Twist()
        if abs(err_x) > threshold:
            speed = k * err_x
            if speed > 0:
                speed = max(min(speed, 1.0), 0.2)
            else:
                speed = min(max(speed, -1.0), -0.2)
            self.get_logger().info(f"Speed: {speed}")
            twist_msg.angular.z = speed
        else:
            twist_msg.angular.z = 0.0
        
        self.twist_pub.publish(twist_msg)
        
        if twist_msg.angular.z == 0.0:
            self.object_checked.append(obj['id'])
            self.mark_object(obj)
            self.get_logger().info("Object marked, recorded and posted to /hazards. resuming normal operations.")
        else:
            self.get_logger().info(f"Rotating to center object: {center_x}")
            
        
    def mark_object(self, obj):
        self.get_logger().info("Marking object, recording and resuming normal operation")
        if self.object_distence != -1:
            point = PointStamped()
            point.header.frame_id = "laser"
            point.header.stamp = self.last_scan_time
            point.point.x = - self.object_distence
            point.point.y = 0.0
            point.point.z = 0.0
            
            try:
                transform = self.tf_buffer.transform(point, "odom")
                self.get_logger().info(f"Object marked at: {transform.point}")
            
                self.marked_objects[obj['id']] = transform.point
                
                self.publish_hazard_marker(obj)

            except Exception as e:
                self.get_logger().error(f"Error transforming point: {str(e)}")
    
    
    def publish_hazard_marker(self, obj):
        marker = Marker()
        marker.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        marker.ns = "hazards"
        marker.id = int(obj['id'])
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        self.get_logger().info(f"Available keys: {list(self.marked_objects.keys())}")
        position = self.marked_objects[obj['id']]
        marker.pose = Pose(position=Point(x=position.x, y=position.y, z=position.z))
        marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker.lifetime = Duration(sec=0, nanosec=0)
        self.hazard_markers.append(marker)

        for _marker in self.hazard_markers:
            self.hazard_pub.publish(_marker)


def main(args=None):
    rclpy.init(args=args)
    exploration_manager = ExplorationManager()
    try:
        rclpy.spin(exploration_manager)
    except KeyboardInterrupt:
        pass
    exploration_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()