import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Header, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, Vector3
from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from tf2_ros import TransformListener, Buffer
from find_object_2d.msg import ObjectsStamped 
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


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


class ImageLocalizer(Node):
    def __init__(self):
        super().__init__('image_localizer')
        
        # Variables
        self.is_spinning = False
        self.is_exploring = False
        
        # Subscribers
        self.spin_sub = self.create_subscription(Bool, 'robot/spinning',
            self.spin_callback, 10)
        self.explore_sub = self.create_subscription(Bool, 'robot/exploring',
            self.explore_callback, 10)
        self.obj_sub = self.create_subscription(ObjectsStamped, '/objectsStamped', 
            self.object_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', 
            self.laser_callback, 10)
        
        # Publishers
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.spin_stop_pub = self.create_publisher(Bool, 'command/stop', 10)
        self.spin_resume_pub = self.create_publisher(Bool, 'command/resume', 10)
        self.explore_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.harzard_pub = self.create_publisher(Marker, '/hazards', 10)
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Marked objects
        self.marked_objects = {}
        self.object_distence = -1
        

    def object_callback(self, msg):
        self.get_logger().info(f"Object detected: {self.marked_objects}")
        if len(msg.objects.data) > 0:    
            obj = {
                'id': msg.objects.data[0],
                'width': msg.objects.data[1],
                'height': msg.objects.data[2],
                'dx': msg.objects.data[9],
                'dy': msg.objects.data[10]
            }
            if self.object_available_to_mark(obj['id']):
                if self.is_spinning:
                    self.spin_stop_pub.publish(Bool(data=True))
                elif self.is_exploring:
                    self.explore_pub.publish(Bool(data=False))
                self.get_logger().info(f"New object detected: {obj['id']}")
                self.rotate_to_center_object(obj)
            else:
                self.get_logger().warn(f"Object already marked or not in the target list, resuming spinning")
                if not self.is_spinning and not self.is_exploring:
                    self.explore_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn("No object detected")
        
    
    def laser_callback(self, msg):
        self.last_scan_time = msg.header.stamp
        self.object_distence = msg.ranges[0]
        
    
    def spin_callback(self, msg):
        is_spinning = msg.data
    
    
    def explore_callback(self, msg):
        is_exploring = msg.data
    
        
    def object_available_to_mark(self, object_id):
        if not (object_id in self.marked_objects) and (object_id in markers):
            return True
        return False  

    
    
    # TODO: Implement the checker for current action status: spinning or exploring
    # idea: use another node to record the status and publish to the topic
    def rotate_to_center_object(self, obj):
        center_x = obj['dx'] + obj['width'] // 2
        
        err_x = 319 - center_x
        k = 1/319
        threshold = 3
        
        twist_msg = Twist()
        if abs(err_x) > threshold:
            speed = k * err_x
            if speed > 0:
                speed = max(min(speed, 1.0), 0.3)
            else:
                speed = min(max(speed, -1.0), -0.3)
            self.get_logger().info(f"Speed: {speed}: negative: turning right, positive: turning left")
            twist_msg.angular.z = speed
        else:
            twist_msg.angular.z = 0.0
        
        self.twist_pub.publish(twist_msg)
        if twist_msg.angular.z == 0.0:
            self.mark_object(obj)
            self.get_logger().info("Object marked")
        else:
            self.get_logger().info(f"Rotating to center object: {center_x}")
    
    
    def publish_harzard_marker(self, obj):
        marker = Marker()
        marker.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        marker.ns = "hazards"
        marker.id = int(obj['id'])
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        self.get_logger().info(f"Available keys: {list(self.marked_objects.keys())}")
        position = self.marked_objects[obj['id']]['position']
        marker.pose = Pose(position=Point(x=position.x, y=position.y, z=position.z))
        marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker.lifetime = Duration(sec=0, nanosec=0)

        self.harzard_pub.publish(marker)
        self.get_logger().info('Published marker for hazard: ' + obj['name'])

        
        

    def mark_object(self, obj):
        # Record the object's position
        if self.object_distence != -1:
            point = PointStamped()
            point.header.frame_id = "laser"
            point.header.stamp = self.last_scan_time
            point.point.x = self.object_distence
            point.point.y = 0.0
            point.point.z = 0.0
            
            try:
                transform = self.tf_buffer.transform(point, "odom")
                self.get_logger().info(f"Object marked at: {transform.point}")
                
                self.marked_objects[obj['id']] = {
                    'name': markers[obj['id']],
                    'position': transform.point
                }
                # TODO: Publish the marker for the object
                self.publish_harzard_marker(obj)

            except Exception as e:
                self.get_logger().error(f"Error transforming point: {str(e)}")
        
        


def main(args=None):
    rclpy.init(args=args)
    node = ImageLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.get_logger().info("Keyboard interrupt, shutting down")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
