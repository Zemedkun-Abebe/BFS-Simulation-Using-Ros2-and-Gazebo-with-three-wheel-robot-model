#!/usr/bin/env python3

import rclpy
from gazebo_msgs.msg import ModelStates

def print_model_names():
    rclpy.init()
    node = rclpy.create_node('model_names')
    
    def model_states_callback(msg):
        model_names = msg.name
        print("Model names:")
        for name in model_names:
            print(name)
    
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
    print_model_names()
