import launch
import launch_ros.actions

def generate_launch_description():
    DRONE_NUMBERS = 1

    nodes_list = []
    # ------ XRCE-Agent -------- #
    nodes_list.append(
        launch.actions.ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent ',
            'udp4 ',
            '-p ',
            '8888'
        ]],
        shell=True))

    for i in range(1,DRONE_NUMBERS+1):
        # ---- PILOT NODE --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='pilot',
            namespace=f'/drone{i+1}',
            name=f'pilot{i+1}',
            parameters=[
                {'simulation':True}])
        )
        # ---- MOTION CAPTURE --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='mocap_gazebo',
            name=f'mocap_node{i+1}',
            namespace=f'/drone{i+1}')
        )
        # ---- CAMERA SIMULATION --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='camera_sim',
            name=f'drone_tracker_py{i+1}',
            namespace=f'/drone{i+1}')
        )

    return launch.LaunchDescription(nodes_list)