#!/bin/usr python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rosky2_interfaces.action import DesireSpeed
from geometry_msgs.msg import Twist

class ActionServerExample(Node):
    def __init__(self):
        super().__init__('action_server_example')

        # Publisher
        self.publisher_ = self.create_publisher(
            msg_type=Twist,
            topic="~/cmd_vel",
            qos_profile=10,
        )

        # Action
        self.action_server = ActionServer(
            node=self, 
            action_type=DesireSpeed, 
            action_name='~/desire_speed', 
            execute_callback=self.callback_action_server_desire_distance,
        )
        self.get_logger().info("Start")

    def callback_action_server_desire_distance(self, goal_handle):
        self.get_logger().info("Executing goal")
        self.cmd_vel = Twist()
        difference, limit = 0.01, 0.2
        feedback_msg = DesireSpeed.Feedback()
        desire_speed = goal_handle.request.speed
        while True:
            if self.cmd_vel.linear.x >= limit:
                feedback_msg.feedback = f"It's my limit! The speed now is {limit}"
                goal_handle.publish_feedback(feedback_msg)
                break
            else:
                if self.cmd_vel.linear.x > desire_speed:
                    feedback_msg.feedback = f"Done! Now the speed is {desire_speed}(m/s)"
                    goal_handle.publish_feedback(feedback_msg)
                    break
                else:
                    feedback_msg.feedback = f"Current speed: {self.cmd_vel.linear.x}(m/s)! I will speed up by {difference}(m/s)" 
                    goal_handle.publish_feedback(feedback_msg)
                    self.cmd_vel.linear.x += difference
                    self.publisher_.publish(self.cmd_vel)
                    time.sleep(0.5)

        goal_handle.succeed()
        result = DesireSpeed.Result()
        result.completed = "Now I will stop moving!"
        return result

    def shutdown(self):
        self.cmd_vel.linear.x = 0.0
        self.publisher_.publish(self.cmd_vel)
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    node = ActionServerExample()
    rclpy.spin(node)
    node.shutdown()

if __name__ == '__main__':
    main()

