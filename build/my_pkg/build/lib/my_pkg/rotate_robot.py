import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class RobotRotator(Node):
    def __init__(self):
        super().__init__('robot_rotator')
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = time.time()
        self.duration = 0
        self.angle = 0
        self.speed = 0
    
    def rotate(self, angle, speed):
        self.angle = angle
        self.speed = speed
        self.duration = abs(angle) / speed
        self.start_time = time.time()
        
    def timer_callback(self):
        now = time.time()
        if now - self.start_time < self.duration:
            move_cmd = Twist()
            move_cmd.angular.z = math.copysign(self.speed, self.angle)
            self.publisher_.publish(move_cmd)
        else:
            move_cmd = Twist()
            move_cmd.angular.z = 0
            self.publisher_.publish(move_cmd)
            self.timer.cancel()
            

def main(args=None):
    rclpy.init(args=args)
    rotator = RobotRotator()
    rotator.rotate(math.radians(90), 0.5)
    rclpy.spin(rotator)
    rotator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

