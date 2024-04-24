import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
from tf2_ros import TransformListener, Buffer
from find_object_2d.msg import ObjectsStamped 


markers = {
    7: "radioactive",
    11: "explosives",
    21: "flammable gas",
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
        self.obj_sub = self.create_subscription(ObjectsStamped, '/objectsStamped', 
            self.object_callback, 10)
        # self.laser_sub = self.create_subscription(LaserScan, '/scan', 
        #     self.laser_callback, 10)
        
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.object_distence = -1
        
        self.marked_objects = {}
        

    def object_callback(self, msg):
        if len(msg.objects.data) > 0:    
            obj = {
                'id': msg.objects.data[0],
                'width': msg.objects.data[1],
                'height': msg.objects.data[2],
                'dx': msg.objects.data[9],
                'dy': msg.objects.data[10]
            }
            if not self.object_available_to_mark(obj['id']):
                self.get_logger().info(f"New object detected: {obj['id']}")
                self.rotate_to_center_object(obj)
        else:
            self.get_logger().warn("No object detected")
        
    
    # def laser_callback(self, msg):
    #     center_index = len(msg.ranges) // 2
    #     self.object_distence = msg.ranges[center_index]
    
        
    def object_available_to_mark(self, object_id):
        if not (object_id in self.marked_objects) and (object_id in markers):
            return True
        return False  


    def rotate_to_center_object(self, obj):
        center_x = obj['dx'] + obj['width'] // 2
        
        err_x = 320 - center_x
        k = 0.05
        threshold = 5
        
        twist_msg = Twist()
        if abs(err_x) > threshold:
            speed = k * err_x
            speed = max(min(speed, 3), 0.3)
            twist_msg.angular.z = speed
        else:
            twist_msg.angular.z = 0
        
        self.twist_pub.publish(twist_msg)
        if twist_msg.angular.z == 0:
            self.mark_object(obj)
            self.get_logger().info("Object marked")
        else:
            self.get_logger().info(f"Rotating to center object: {err_x}")
        pass
        

    def mark_object(self, obj):
        # Record the object's position
        if self.object_distence != -1:
            point = PointStamped()
            point.header.frame_id = "laser" # TODO: Change to the correct frame
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = self.object_distence
            point.point.y = 0
            point.point.z = 0
            
            try:
                transform = self.tf_buffer.transform(point, "odom")
                self.get_logger().info(f"Object marked at: {transform.point}")
            except Exception as e:
                self.get_logger().error(f"Error transforming point: {str(e)}")
        
        


def main(args=None):
    rclpy.init(args=args)
    node = ImageLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
