import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # ------ XRCE-Agent -------- #
        launch.actions.ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent ',
            'udp4 ',
            '-p ',
            '8888'
        ]],
        shell=True),

        # ------- DRONE 2 ---------- #
        launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='drone_tracker_py',
            name='drone_tracker_py',
            parameters=[
                {'namespace':"drone2"}
            ]),

        launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='mocap_gazebo',
            name='mocap_node',
            parameters=[
                {'namespace':"drone2"}
            ]),

        # ------- DRONE 3 ---------- #
        launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='drone_tracker_py',
            name='drone_tracker_py',
            parameters=[
                {'namespace':"drone3"}
            ]),

        launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='mocap_gazebo',
            name='mocap_node',
            parameters=[
                {'namespace':"drone3"}
            ])
  ])