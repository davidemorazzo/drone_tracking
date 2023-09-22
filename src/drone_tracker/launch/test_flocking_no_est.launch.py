import launch
import launch_ros.actions

def generate_launch_description():

    nodes_list = []

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='vehicle',
        namespace=f'/drone2',
        name=f'drone2',
        parameters=[{"height":-0.5}])
    )

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='vehicle',
        namespace=f'/drone3',
        name=f'drone3',
        parameters=[{"height":-0.5}])
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
            {"nA_id":"1"}
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
            {"nA_id":"0"}
        ])
    )

    nodes_list.append(
        launch.actions.ExecuteProcess(
        cmd=[[
            'ros2 ', 'bag ', 'record ', 
            '-a ',
            '-x ', "'\/(drone[0-9]\/camera\/(?!marker_pos).*)' ",
        ]],
        shell=True,
        name="rosbag2"))

    return launch.LaunchDescription(nodes_list)