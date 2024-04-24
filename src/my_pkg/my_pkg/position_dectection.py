import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from tf2_ros import TransformListener, Buffer
import tf2_ros


class PositionDetection(Node):
    def __init__(self):
        super().__init__('position_detection')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', 
            self.laser_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.object_distence = -1
    
    
    def laser_callback(self, msg):
        self.last_scan_time = msg.header.stamp
        self.object_distence = msg.ranges[0]
        self.get_logger().info(f"Center distence: {self.object_distence}")
        self.position_calculation()
        
    
    def position_calculation(self):
        point = PointStamped()
        point.header.frame_id = 'laser'
        point.header.stamp = self.last_scan_time
        point.point.x = self.object_distence
        point.point.y = 0.0
        point.point.z = 0.0
        
        try:
            transform = self.tf_buffer.transform(point, 'odom')
            self.get_logger().info(f"Object odom position: {transform}")
        except tf2_ros.LookupException:
            self.get_logger().error("No transformation found")
        except tf2_ros.ConnectivityException:
            self.get_logger().error("Problems with connectivity")
        except tf2_ros.ExtrapolationException:
            self.get_logger().error("Extrapolation problem")
        except Exception as e:
            self.get_logger().error(f"Error transforming point: {str(e)}")
        

def main(args=None):
    rclpy.init(args=args)
    node = PositionDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.get_logger().info("Keyboard interrupt, shutting down")
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
    