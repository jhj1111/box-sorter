import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='box_sorter',
            executable='job_publisher',
            name='job_publisher',
            output='screen'
        ),
        
        launch_ros.actions.Node(
            package='box_sorter',
            executable='conveyor',
            name='conveyor',
            output='screen'
        )
    ])
