import os
import launch
import launch_ros.actions

def generate_launch_description():
    package_dir = launch_ros.substitutions.FindPackageShare(package='my_robot_package').find('my_robot_package')
    world_file = os.path.join(package_dir, 'worlds', 'my_world.world')

    gazebo_launch = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )

    return launch.LaunchDescription([gazebo_launch])

if __name__ == '__main__':
    generate_launch_description()
