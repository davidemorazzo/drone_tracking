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
        shell=True,
        name="XRCE-Agent"))
    
    # ------ ROSBAG2 -------- #
    nodes_list.append(
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 ', 'bag ', 'record ', 
            '-a ',
            '-x ', "'\/(drone[0-9]\/camera\/(?!marker_pos).*)' ",
            '--use-sim-time '
        ]],
        shell=True,
        name="rosbag2"))
    
    for i in range(2,4):
        # ------- COMMANDER -------- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker',
            executable='vehicle',
            namespace=f'/drone{i}',
            name=f'vehicle{i}',
            parameters=[
                {'use_sim_time':True},
                {'height' : -0.5}])
        )
        # ---- MOTION CAPTURE --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='gazebo_mocap',
            executable='mocap',
            name=f'mocap_node{i}',
            namespace=f'/drone{i}',
            parameters=[{'use_sim_time':True}])
        )
        # ---- GAZEBO CAMERA DRIVER NODE --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker',
            executable='gazebo_camera_driver',
            name=f'gazebo_camera_driver{i}',
            namespace=f'/drone{i}',
            parameters=[{'use_sim_time':True}])
        )
        # ---- AURCO NODE --- #
        nodes_list.append(
            launch_ros.actions.Node(
            package='drone_tracker',
            executable='aruco_pose',
            name=f'aruco_pose{i}',
            namespace=f'/drone{i}',
            parameters=[
                {'use_sim_time':True},
                {'fake_measure':False}])
        )
    # ------ ESTIMATOR ------ #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='estimator2',
        name=f'estimator',
        namespace=f'/drone2',
        parameters=[
            {"self_id":"0"},
            {"nA_id":"1"},
            {'use_sim_time':True}
        ]))

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='estimator2',
        name=f'estimator',
        namespace=f'/drone3',
        parameters=[
            {"self_id":"1"},
            {"nA_id":"0"},
            {'use_sim_time':True}
        ]))

    return launch.LaunchDescription(nodes_list)