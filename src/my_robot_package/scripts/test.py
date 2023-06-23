#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

def move_robot():
    rclpy.init()
    node = rclpy.create_node('robot_movement')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def model_states_callback(msg):
        # Find the index of your robot model in the list of names
        model_index = msg.name.index('YOUR_ROBOT_MODEL_NAME')
        print(f"Model index: {model_index}")

        # Set the linear and angular velocities
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Set the desired linear velocity
        twist_msg.angular.z = 0.1  # Set the desired angular velocity

        # Publish the twist message
        publisher.publish(twist_msg)
        print("Twist message published")

    node.create_subscription(
        ModelStates,
        '/gazebo/model_states',
        model_states_callback,
        10
    )

    print("Subscribed to model states topic")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    move_robot()
