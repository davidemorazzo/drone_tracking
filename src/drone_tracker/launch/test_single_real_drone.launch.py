import launch
import launch_ros.actions

def generate_launch_description():

    nodes_list = []
    # ------- COMMANDER -------- #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='vehicle',
        namespace=f'/drone2',
        name=f'vehicle')
    )

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='vehicle',
        namespace=f'/drone3',
        name=f'vehicle')
    )

    # nodes_list.append(
    #     launch_ros.actions.Node(
    #     package='drone_tracker',
    #     executable='vehicle',
    #     namespace=f'/drone3',
    #     name=f'vehicle')
    # )
    # ---- ARUCO NODE --- #
    # nodes_list.append(
    #     launch_ros.actions.Node(
    #     package='drone_tracker',
    #     executable='aruco_pose',
    #     name=f'aruco_pose_2',
    #     namespace=f'/drone2')
    # )
    # # ---- CAMERA NODE --- #
    # nodes_list.append(
    #     launch_ros.actions.Node(
    #     package='drone_tracker',
    #     executable='openmv_camera',
    #     name=f'openmv_camera_2',
    #     namespace=f'/drone2')
    # )
    # # ------ ESTIMATOR ------ #
    # nodes_list.append(
    #     launch_ros.actions.Node(
    #     package='drone_tracker',
    #     executable='estimator',
    #     name=f'estimator',
    #     namespace=f'/drone2',
    #     parameters=[
    #         {"self_id":"0"},
    #         {"nA_id":"1"},
    #         {"nB_id":"2"}
    #     ])
    # )

    return launch.LaunchDescription(nodes_list)