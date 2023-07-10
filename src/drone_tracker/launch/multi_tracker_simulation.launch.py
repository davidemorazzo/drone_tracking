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
    
    for i in range(2,5):
        # ------- COMMANDER -------- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker',
            executable='vehicle',
            namespace=f'/drone{i}',
            name=f'vehicle{i}')
        )
        # ---- MOTION CAPTURE --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker_py',
            executable='mocap_gazebo',
            name=f'mocap_node{i}',
            namespace=f'/drone{i}')
        )
        # ---- CAMERA NODE --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker',
            executable='camera',
            name=f'camera{i}',
            namespace=f'/drone{i}')
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
        ]))

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='estimator',
        name=f'estimator',
        namespace=f'/drone3',
        parameters=[
            {"self_id":"1"},
            {"nA_id":"0"},
            {"nB_id":"2"}
        ]))
    
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='estimator',
        name=f'estimator',
        namespace=f'/drone4',
        parameters=[
            {"self_id":"2"},
            {"nA_id":"1"},
            {"nB_id":"0"}
        ]))

    return launch.LaunchDescription(nodes_list)