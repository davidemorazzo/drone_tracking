# Simulazione su Gazebo

La simulazione è fatta su Gazebo-Classic. Viene usato il drone modello Iris, con in aggiunta un plugin per l'Odometria esterna per simulare il VICON e un plugin per la videocamera. Entrambi i plugin rendono disponibili le informazioni su topic ROS 2. 

Per aggiungere i plugin modificare il file in PX4 `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja`, aggiungendo il seguente codice all'interno del tag `<model>`
```bash
            <plugin name="gazebo_ros_p3d_{{mavlink_id}}" filename="libgazebo_ros_p3d.so">
      <ros>
        <namespace>/drone{{mavlink_id}}</namespace>
        <remapping>odom:=odom</remapping>
      </ros>
      <body_name>base_link</body_name>
      <frame_name>map</frame_name>
      <update_rate>50</update_rate>
      <xyz_offset>0 0 0</xyz_offset>
      <rpy_offset>0 0 0</rpy_offset>
      <gaussian_noise>0.01</gaussian_noise>
    </plugin>

    <link name="camera_link">
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.0001</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0002</izz>
        </inertia>
      </inertial>
      <pose>0.15 0 0 0 1.57 0</pose>
      <sensor type="camera" name="camera1">
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <camera name="head">
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>300</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <plugin name="camera_controller_{{mavlink_id}}" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/drone{{mavlink_id}}</namespace>
            <!-- TODO(louise) Remapping not working due to https://github.com/ros-perception/image_common/issues/93 -->
            <argument>--ros-args --remap image_raw:=image_demo</argument>
            <argument>--ros-args --remap camera_info:=camera_info_demo</argument>
          </ros>
          <!-- camera_name>omit so it defaults to sensor name</camera_name-->
          <!-- frame_name>omit so it defaults to link name</frameName-->
        </plugin>
      </sensor>
    </link>

    <joint name='camera_joint' type='fixed'>
      <child>base_link</child>
      <parent>camera_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
    </joint>
```

Notare la parametrizzazione di alcuni argomenti con `{{mavlink_id}}`, necessario a jinja per generare modelli con namespace diversi. 

Per avviare la simulazione usare il seguente comando:
```bash
./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -t px4_sitl_links -w aruco-world -s iris:1:1:1,iris:1:-1:1,iris:1:-1:-1
```
L'argomento `-s` permette di generare diversi modelli (separati da virgola) e la sintassi è la seguente: `<nome_modello>:<numero di instanze>:x_coord:y_coord`

# Custom world
Inserire il file `aruco-world.world` nella cartella `Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds`. 