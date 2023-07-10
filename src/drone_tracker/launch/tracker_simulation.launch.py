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
    # ------- COMMANDER -------- #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='vehicle',
        namespace=f'/drone2',
        name=f'vehicle')
    )
    # ---- MOTION CAPTURE --- #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker_py',
        executable='mocap_gazebo',
        name=f'mocap_node',
        namespace=f'/drone2')
    )
    # ---- CAMERA NODE --- #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='camera',
        name=f'camera',
        namespace=f'/drone2')
    )
    # ------ ESTIMATOR ------ #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='estimator',
        name=f'estimator',
        namespace=f'/drone2',
        parameters=[
            {"self_id":"0"},
            {"nA_id":"1"},
            {"nB_id":"2"}
        ])
    )

    return launch.LaunchDescription(nodes_list)