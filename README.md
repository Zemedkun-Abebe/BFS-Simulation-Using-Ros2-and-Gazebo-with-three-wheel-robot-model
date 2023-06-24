# BFS Simulation Using Ros2 and Gazebo with three wheel robot model

This repository contains code for simulating BFS (Breadth-First Search) path planning using ROS2 and Gazebo with a three-wheel robot model.

## Description

The purpose of this project is to demonstrate how to implement BFS path planning algorithm in a simulated environment using ROS2 and Gazebo. The three-wheel robot model is used as the mobile platform, and the Gazebo simulator provides a realistic simulation environment.

The main components of the project are:

- **RobotControl**: This class controls the movement of the robot and implements the BFS algorithm for path planning.
- **ModelStates Subscriber**: It subscribes to the `/gazebo/model_states` topic to get the positions of the robot and landmarks in the simulation.
- **Path Planner**: It uses the BFS algorithm to find the shortest path between two given states in the simulation environment.
- **Gazebo Simulation**: The simulation environment consists of the robot model, landmarks, and obstacles defined in the Gazebo world file.

## Getting Started

To run the simulation and test the BFS path planning algorithm, follow these steps:

1. Prerequisites: Make sure you have ROS2 and Gazebo installed on your system.
2. Clone this repository: Use the following command to clone this repository to your local machine:
   git clone https://github.com/Zemedkun-Abebe/BFS-Simulation-Using-Ros2-and-Gazebo-with-three-wheel-robot-model/)>](https://github.com/Zemedkun-Abebe/BFS-Simulation-Using-Ros2-and-Gazebo-with-three-wheel-robot-model/
3. Build the ROS2 packages: Navigate to the cloned repository and build the ROS2 packages using the following commands:
   cd <repository-directory>
colcon build
4. Launch the simulation: Run the launch file to launch the Gazebo simulation environment:
   source install/setup.bash
ros2 launch my_robot_package my_simulation.launch.py
5. Execute the BFS path planning: Open a new terminal, navigate to the repository directory, and run the following command:
   source install/setup.bash
ros2 run my_robot_package path_planner.py
7. Observe the simulation: The robot will start moving in the simulation environment, following the calculated shortest path based on the BFS algorithm.

## Contributing

Contributions to this project are welcome. If you have any ideas, suggestions, or bug fixes, please submit a pull request or open an issue.

## License

This project is licensed under the [MIT License](LICENSE).

