#!/bin/usr/python3
# coding=UTF-8
# Copyright (c) 2022 Wei-Chih Lin(weichih.lin@protonmail.com)
# Apache 2.0 License
# 
# This node will get the distance from specific direction

import sys, time, threading, math, re
path = "/opt/ros/noetic/lib/python3/dist-packages" 
if path in sys.path:
    sys.path.remove(path)

import rclpy
from multiprocessing import cpu_count
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from rosky2_interfaces.srv import DirectionUpdate
from rclpy.qos import ReliabilityPolicy, QoSProfile
from functools import partial

class DetectDistance(Node):

    def __init__(self):
        """Initialize Node"""
        super().__init__("detect_distance")
        self.get_logger().info("Initializing...")

        # local parameter
        self.cbg = ReentrantCallbackGroup()
        self.direction_list = ["back", "right", "front", "left"]
        self.direction = "back"

        # create subscription
        self.subscriber_ = self.create_subscription(
            msg_type=LaserScan, 
            topic="~/scan", 
            callback=self.callback_scan, 
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.cbg
        )

        # creat service server
        self.service_server_ = self.create_service(
            srv_type=DirectionUpdate,
            srv_name="~/update_direction",
            callback=self.callback_update_direction,
            callback_group=self.cbg
        )

        self.get_logger().info("Initialized successfully!")

    
    def callback_scan(self, msg: LaserScan):
        """Get the distance from msg.ranges, assume the angle range is -180 degrees ~ +180 degrees"""
        direction_increment = len(msg.ranges) // 4
        direction = {"back": 1}
        direction["right"] = direction["back"] + direction_increment
        direction["front"] = direction["right"] + direction_increment
        direction["left"] = direction["front"] + direction_increment
        self.get_logger().info(f"{self.direction.upper()}: {msg.ranges[direction[self.direction]]:.3f} m")

    def callback_update_direction(self, request: DirectionUpdate.Request, response: DirectionUpdate.Response):
        """Update local parameter direction"""
        if request.data in self.direction_list:
            self.direction = request.data
            response.message = f"Update direction successfully! Now detect direction: {self.direction.upper()}"
        else:
            response.message = f"Failed to update! You just can use these direction: {self.direction_list}"
        return response

    def on_shutdown(self):
        self.get_logger().info("Shutdown successfully.")


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # declare the node constructor
    detect_distance = DetectDistance()      
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    executor = MultiThreadedExecutor(num_threads=cpu_count())
    executor.add_node(detect_distance)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # Explicity destroy the node
    detect_distance.on_shutdown()
    # shutdown the ROS communication
    executor.shutdown()
    detect_distance.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()