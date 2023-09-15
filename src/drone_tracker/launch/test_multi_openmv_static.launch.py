import launch
import launch_ros.actions

def generate_launch_description():

    nodes_list = []
    nodes_list.append(
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher ',
            '--x 0 --y 0 --z 0.75 --roll 0 --pitch 0 --yaw 1.5708 ',
            '--frame-id map --child-frame-id drone2'
        ]],
        name="tf2_drone2",
        shell=True))
    
    nodes_list.append(
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher ',
            '--x 0.4 --y 0 --z 0.75 --roll 0 --pitch 0 --yaw 1.5708 ',
            '--frame-id map --child-frame-id drone3'
        ]],
        name="tf2_drone3",
        shell=True))
    
    nodes_list.append(
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher ',
            '--x 0.07 --y 0 --z 0 --qx 0 --qy 0 --qz 1 --qw 0 ',
            '--frame-id drone2 --child-frame-id drone2/camera'
        ]],
        name="tf2_camera2",
        shell=True))
    
    nodes_list.append(
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher ',
            '--x 0.07 --y 0 --z 0 --qx 0 --qy 0 --qz 1 --qw 0 ',
            '--frame-id drone3 --child-frame-id drone3/camera'
        ]],
        name="tf2_camera3",
        shell=True))
    
    # ---- ARUCO NODE --- #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='aruco_pose',
        name=f'aruco_pose_2',
        namespace=f'/drone2')
    )

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='aruco_pose',
        name=f'aruco_pose_3',
        namespace=f'/drone3')
    )
    # ------ ESTIMATOR ------ #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='estimator2',
        name=f'estimator2',
        namespace=f'/drone2',
        parameters=[
            {"self_id":"0"},
            {"nA_id":"1"},
        ])
    )
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='estimator2',
        name=f'estimator2',
        namespace=f'/drone3',
        parameters=[
            {"self_id":"1"},
            {"nA_id":"0"},
        ])
    )

    return launch.LaunchDescription(nodes_list)