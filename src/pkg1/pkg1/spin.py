import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
 
class PeriodicSpinner(Node):
    def __init__(self):
        super().__init__('periodic_spinner')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pause_pub = self.create_publisher(Bool, "explore/resume", 10)
        self.timer = self.create_timer(2, self.send_spin_command)  # Adjust timer interval
 
    def send_spin_command(self):

        pause_msg = Bool()
        pause_msg.data = False
        self.pause_pub.publish(pause_msg)
        self.get_logger().info('i am pausing')


        self.get_logger().info('starts spinning')
        for i in range(60):
            twist = Twist()
            twist.angular.z = 0.5  # Adjust spin speed
            self.publisher.publish(twist)
            time.sleep(0.2)
        self.get_logger().info('stopped spinning')
    

 
        pause_msg = Bool()
        pause_msg.data = True
        self.pause_pub.publish(pause_msg)
        self.get_logger().info('i am resuming')

def main(args=None):
    rclpy.init(args=args)
    node = PeriodicSpinner()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()