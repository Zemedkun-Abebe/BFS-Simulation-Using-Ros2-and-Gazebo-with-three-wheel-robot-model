#!/usr/bin/env python3
import rclpy
from collections import deque
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from rclpy.qos import qos_profile_sensor_data

print(f"Importing Libraries DONE")

class RobotControl:
    print(f"Class Started")
    def __init__(self):
        self.landmark_positions = {}
        self.node = rclpy.create_node('bfs_robot_control')
        print(f"Creating Node DONE")
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        print(f"Creating Publisher DONE")
        self.linear_velocity = 0.2
        self.angular_velocity = 0.1
        self.rate = None
        print(f"Variable Declaration DONE")
        self.node.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            qos_profile=qos_profile_sensor_data
        )
        print(f"Node creation Subscription DONE")

    def model_states_callback(self, msg):
        print("Model states callback triggered")  # Debugging statement
        for i, name in enumerate(msg.name):
            if name.startswith("STATE_"):
                self.landmark_positions[name] = msg.pose[i].position

    def bfs(self, graph, start, goal):
        print(f"BFS Running")
        visited = set()
        queue = deque([(start, [])])

        while queue:
            node, path = queue.popleft()

            if node == goal:
                print(f"Path Finding DONE")
                return path + [node]

            if node not in visited:
                visited.add(node)
                queue.extend((neighbor, path + [node]) for neighbor in graph[node])

        return None

    def move_to_landmark(self, landmark_name):
        landmark_position = self.landmark_positions.get(landmark_name)

        if landmark_position:
            target_x = landmark_position.x
            target_y = landmark_position.y

            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = self.angular_velocity

            print(f"Moving to landmark {landmark_name} at position ({landmark_position.x}, {landmark_position.y})")

            while not self.reached_target(target_x, target_y):
                self.publisher.publish(twist_msg)
                rclpy.spin_once(self.node)
                self.rate.sleep()

            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)

            print(f"Reached landmark {landmark_name}")
        else:
            print(f"Landmark {landmark_name} not found!")

    def reached_target(self, target_x, target_y, tolerance=0.1):
        current_x = 0.0  
        current_y = 0.0  

        if abs(target_x - current_x) <= tolerance and abs(target_y - current_y) <= tolerance:
            return True
        else:
            return False

    def run(self):
        graph = {
            'STATE_A': ['STATE_B', 'STATE_D'],
            'STATE_B': ['STATE_A', 'STATE_D'],
            'STATE_C': ['STATE_D', 'STATE_F'],
            'STATE_D': ['STATE_A', 'STATE_B', 'STATE_C', 'STATE_F']
        }

        start_state = 'STATE_A'
        goal_state = 'STATE_C'

        print("Waiting for landmark positions...")

        while len(self.landmark_positions) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        print("Landmark positions received.")

        self.rate = self.node.create_rate(10)
        path = self.bfs(graph, start_state, goal_state)

        if path:
            print("Path found:")
            print(" -> ".join(path))

            for landmark in path:
                self.move_to_landmark(landmark)
                rclpy.sleep(1.0)  # Delay to observe the movement in Gazebo

        else:
            print("No path found.")

        self.node.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    robot_control = RobotControl()
    robot_control.run()

if __name__ == '__main__':
    main()
