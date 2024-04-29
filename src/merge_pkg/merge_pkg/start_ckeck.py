#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from find_object_2d.msg import ObjectsStamped
from std_msgs.msg import Bool
import subprocess

class ObjectDetectionMover(Node):
    def __init__(self):
        super().__init__('object_detection_mover')
        # Flags
        self.start_signal_sent = False
        self.markerchecker = False
        
        # Subcribers
        self.subscription = self.create_subscription(ObjectsStamped, '/objectsStamped',
            self.detection_callback, 10)
        
        # Publishers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start_exploration_pub =  self.create_publisher(Bool, 
                                                        'exploration_control/start', 10 )

    def detection_callback(self, msg):
        try:
            if msg.objects.data[0] == 12 and not self.start_signal_sent:
                self.get_logger().info("Start detected, publishing start exploration command.")
                START_msg = Bool()
                START_msg.data = True
                self.start_exploration_pub.publish(START_msg)
                self.start_signal_sent = True  # Prevent further signals until reset
                self.get_logger().info("Published start exploration.")
                self.markerchecker = True
        except Exception as e:
            self.get_logger().error(f"PUBLISH FAILED: {str(e)}")
 
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionMover()
    try:
        rclpy.spin(node)  # Corrected to spin, not spin_once
    except KeyboardInterrupt:
        pass  # Gracefully handle a keyboard interrupt
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()