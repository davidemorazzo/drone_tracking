import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='drone_tracker_py',
            name='drone_tracker_py'),
        

  ])