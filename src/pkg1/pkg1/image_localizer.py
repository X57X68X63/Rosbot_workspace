import numpy as np
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

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.harzard_pub = self.create_publisher(Marker, '/hazards', 10)
        self.state_pub = self.create_publisher(Bool, 'working/state', 10)
        self.explorecontrol_pub = self.create_publisher(Bool, 'explore/resume', 10)

        self.obj_sub = self.create_subscription(ObjectsStamped, '/objectsStamped', 
            self.object_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', 
            self.laser_callback, 10)
        self.state_sub = self.create_subscription(Bool, 'working/state', 
            self.state_callback, 10)
 
        self.working_state = None 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.check_list = []
        self.marked_objects = {}
        self.object_distence = -1
        

    def object_callback(self, msg):

        if self.working_state is None:
            self.get_logger().warn("Working state is unknown; waiting for state update.")
            return
        data = []
        if 0 < len(msg.objects.data) <= 12:
            data = msg.objects.data
        elif len(msg.objects.data) > 12:
            data = msg.objects.data[:12]
        if len(data) > 0 and self.object_available_to_mark(data[0]):
            self.get_logger().info(f"Object detected, interruption signal sent")
            if not self.working_state:
                
                self.get_logger().info("Interrupting other activities due to object detection.")
                obj = {
                    'id': data[0],
                    'width': data[1],
                    'height': data[2],
                    'dx': data[9],
                    'dy': data[10],
                    'h': data[3:12]
                }
                self.get_logger().info(f"Object width: {obj['width']}, height: {obj['height']}")
                self.get_logger().info(f"Object center: {obj['dx'] + obj['width'] // 2}")
                self.get_logger().info(f"Rotating to object")
                self.rotate_to_center_object(obj)
            else:
                self.state_pub.publish(Bool(data=False))

        else:
            if self.working_state is False:
                self.get_logger().warn(f"Object already marked or not in the target list.")
                #self.state_pub.publish(Bool(data=True))
                self.explorecontrol_pub.publish(Bool(data=True))

    def laser_callback(self, msg):
        self.last_scan_time = msg.header.stamp
        self.object_distence = msg.ranges[0]

    def state_callback(self, msg):
        self.get_logger().info(f"State received: {msg.data}")
        self.working_state = msg.data

        
    def object_available_to_mark(self, object_id):
        if not (object_id in self.check_list) and (object_id in markers.keys()):
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
                speed = max(min(speed, 1.0), 0.3)
            else:
                speed = min(max(speed, -1.0), -0.3)
            self.get_logger().info(f"Speed: {speed}: negative: turning right, positive: turning left")
            twist_msg.angular.z = speed
        else:
            twist_msg.angular.z = 0.0
        
        self.twist_pub.publish(twist_msg)
        
        if twist_msg.angular.z == 0.0:
            self.check_list.append(obj['id'])
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
                
                self.publish_harzard_marker(obj)

            except Exception as e:
                self.get_logger().error(f"Error transforming point: {str(e)}")
            
            finally:
                #self.state_pub.publish(Bool(data=True))
                self.explorecontrol_pub.publish(Bool(data=True))
        
        
    def publish_harzard_marker(self, obj):
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

        self.harzard_pub.publish(marker)




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
