# Drone tracking simulation

The simulation is based on the PX4 sitl simulation using gazebo-classic. In the folder `/px4_files` some files needed for the simulation:
- Place the world file `aruco-world.world` in `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/`
- Replace the model file `iris.sdf.jinja` in `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/`. This adds to the iris model a down facing camera and an odometry plugin. 
- Replace the start script `sit_multiple_run.sh` in `PX4-Autopilot/Tools/simulation/gazebo-classic/` 

Note that if the gazebo fails or freeze during the load of the world probably some models in the simulation are missing. Check the world file and adjust accordingly. 

Install the gazebo plugin package to make the prev. files work correctly:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

To run the simulation open 4 terminals and run the following:
Run the gazebo simulation
```bash
cd ~/PX4-Autopilot
./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -t px4_sitl_links -w aruco-world -s iris:1:0.5:0.5,iris:1:-0.5:0.5,iris:1:-0.5:-0.5
```

Execute the ROS 2 nodes 
```bash
cd ~/drone_tracking/rosbag
source ../install/local_setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
ros2 launch drone_tracker multi_tracker_simulation.launch.py
```

Command to be executed after the drones takeoff to start the flocking algorithm. When the command will be stopped wit Ctrl-C the drones will land.
```bash
ros2 topic pub -r 5 /start_flocking std_msgs/msg/Bool "{data: true}"
```

Command to impose a linear velocity to the aruco marker.
```bash
ros2 topic pub --once /aruco_marker/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The simulation will be recorded into a ros2 bag so the data can be reviewed later. 