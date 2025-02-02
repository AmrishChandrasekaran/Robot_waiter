import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_waiter',
            executable='robot_waiter_node',
            name='robot_waiter',
            output='screen',
            parameters=[{'timeout': 5}],  # Set your timeout parameter here if needed
            remappings=[('/order_received', '/order_received')]  # Remap topic if needed
        )
    ])
