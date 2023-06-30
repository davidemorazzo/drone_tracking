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

    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker',
        executable='vehicle',
        namespace=f'/drone1',
        name=f'vehicle',
        parameters=[
            {'simulation':True}])
    )
    # ---- MOTION CAPTURE --- #
    nodes_list.append(
        launch_ros.actions.Node(
        package='drone_tracker_py',
        executable='mocap_gazebo',
        name=f'mocap_node',
        namespace=f'/drone1')
    )
    # ---- CAMERA SIMULATION --- #
    # nodes_list.append(
    #     launch_ros.actions.Node(
    #     package='drone_tracker_py',
    #     executable='camera_sim',
    #     name=f'drone_tracker_py{i+1}',
    #     namespace=f'/drone{i+1}')
    # )

    return launch.LaunchDescription(nodes_list)