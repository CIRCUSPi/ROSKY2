#!/bin/usr python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosky2_interfaces.action import DesireSpeed

class ActionClientExample(Node):
    def __init__(self):
        super().__init__("action_client_example")
        self.action_client = ActionClient(
            node=self, 
            action_type=DesireSpeed, 
            action_name="/action_server_example/desire_speed",
        )
        self.get_logger().info(f"Start!")
        future = self.send_goal(0.18)

    def send_goal(self, speed):
        goal = DesireSpeed.Goal()
        goal.speed = speed 
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal=goal, 
            feedback_callback=self.feedback_back
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("No! Goal rejected :(")
            return
        
        self.get_logger().info("Goal accepted :)")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.completed}")
        rclpy.shutdown()
    
    def feedback_back(self, feedback_msg):
        feedback = feedback_msg.feedback.feedback
        self.get_logger().info(f"Feedback: {feedback}")

def main(args=None):
    rclpy.init(args=args)
    node = ActionClientExample()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
