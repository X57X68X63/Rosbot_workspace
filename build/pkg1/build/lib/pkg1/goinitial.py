#!/usr/bin/env python  


import rclpy
from rclpy.node import Node

import geometry_msgs.msg

class GoToPose(Node):
    def __init__(self):
        super().__init__('aiil_gotopose')
        
        # Config
        self.map_frame = "map"
        
        # Goal position publisher
        self.topic = "/goal_pose"
        self.pub = self.create_publisher(geometry_msgs.msg.PoseStamped, self.topic, 10)
        
        # Setup timer callback
        self.timer = self.create_timer(1.0, self.goToPose)
    
    def goToPose(self):
        time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.5)
        
        self.get_logger().info("--------------------------------")
        self.get_logger().info("Time:" + str(time))
        
        # Construct the destination pose
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        self.get_logger().info(f" - Destination Pose: {pose}")
        
        # Construct Message
        poseS = geometry_msgs.msg.PoseStamped()
        poseS.header.frame_id = self.map_frame
        poseS.header.stamp = time.to_msg()
        poseS.pose = pose
        
        # Publish
        self.pub.publish(poseS)
        self.get_logger().info(f" - Published")


def main():
    rclpy.init()
    node = GoToPose()
    
    try:
        # Execute once
        rclpy.spin_once(node)
        
        # Execute on repeat
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    exit(0)


