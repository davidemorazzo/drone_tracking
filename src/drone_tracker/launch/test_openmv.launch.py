import launch
import launch_ros.actions

def generate_launch_description():

    nodes_list = []
    # nodes_list.append(
    #     launch.actions.ExecuteProcess(
    #     cmd=[[
    #         'ros2 run tf2_ros static_transform_publisher ',
    #         '--x 0 --y 0 --z 0.75 --roll 0 --pitch 0 --yaw 0 ',
    #         '--frame-id map --child-frame-id drone2'
    #     ]],
    #     shell=True))

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='vehicle',
        namespace=f'/drone2',
        name=f'vehicle')
    )
    
    # ---- ARUCO NODE --- #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='aruco_pose',
        name=f'aruco_pose_2',
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