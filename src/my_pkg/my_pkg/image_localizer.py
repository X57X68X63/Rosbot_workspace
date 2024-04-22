import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from find_object_2d.msg import ObjectsStamped 


markers = {
    1: "start",
    7: "radioactive",
    11: "explosives",
    21: "flammable",
    22: "non-flammable gas",
    41: "dangous when wet",
    42: "combustible",
    43: "flammable solid",
    51: "oxidizer",
    52: "orgnic peroxide",
    61: "posion",
    62: "inhalation hazards"
}



class ImageLocalizer(Node):
    def __init__(self):
        super().__init__('image_localizer')
        self.subscription = self.create_subscription(
            ObjectsStamped, 
            '/objectsStamped', 
            self.object_callback, 
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marked_objects = {}

    def object_callback(self, msg):
        if not self.object_available_to_mark(msg.id):
            self.get_logger().info(f"New object detected: {msg.id}")
            self.rotate_to_center_object(msg)
            self.mark_object(msg)

    def object_available_to_mark(self, object_id):
        if object_id not in self.marked_objects and object_id in markers:
            return True
        return False  # Simplified for example

    def rotate_to_center_object(self, msg):
        # Command robot to rotate so that the object is centered
        twist_msg = Twist()
        twist_msg.angular.z = 1.0  # Simplified rotation command
        self.publisher.publish(twist_msg)
        # You would include logic to stop rotation based on image position

    def mark_object(self, msg):
        # Record the object's position
        try:
            trans = self.tf_buffer.lookup_transform('map', 'camera', rclpy.time.Time())
            self.get_logger().info(f"Object {msg.id} is at {trans.transform.translation}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
