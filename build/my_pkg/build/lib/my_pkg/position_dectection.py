import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
from tf2_ros import TransformListener, Buffer
from find_object_2d.msg import ObjectsStamped


class PositionDetection(Node):
    def __init__(self):
        super().__init__('position_detection')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', 
            self.laser_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.object_distence = -1
    
    
    def laser_callback(self, msg):
        front_distence = msg.ranges[0]
        self.object_distence = msg.ranges[0]
        self.get_logger().info(f"Center distence: {self.object_distence}")
        

def main(args=None):
    rclpy.init(args=args)
    node = PositionDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
    