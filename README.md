# Autodock_Lateralside_ROS2

<p align="center">
  <img src="readmefiles.gif" alt="说明文本" width="800">
</p>

## Install dependencies
```bash
cd ~/Autodock_Lateralside_ROS2/
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## Build packages
```bash
cd ~/Autodock_Lateralside_ROS2/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
``` 
## Simulation in Gazebo

```bash
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=tag.model use_camera:=top
ros2 launch apriltag_docking autodock_gazebo.launch.py open_rviz:=true
ros2 run apriltag_docking docking_client --ros-args -p docking:=start
```

## Use Physical Robot and Camera

```bash
git clone https://github.com/Hongtai-Yuan/Autodock_Frontside_ROS2.git
ros2 launch apriltag_docking autodock_neuronbot.launch.py open_rviz:=true
```

## Change Src And Include Then
```bash
# Use Autodock_Lateralside_ROS2/src/apriltag_docking/autodock_controller/src/ To Change Autodock_Frontside_ROS2/src/apriltag_docking/autodock_controller/src
# Use Autodock_Lateralside_ROS2/src/apriltag_docking/autodock_controller/include/ To Change Autodock_Frontside_ROS2/src/apriltag_docking/autodock_controller/include/

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch apriltag_docking autodock_gazebo.launch.py open_rviz:=true
ros2 run apriltag_docking docking_client --ros-args -p docking:=start
```

## Integrating with BehaviorTree

Download BT_ros2:

```bash
cd ~/autodock_ros2_ws/src
[git clone https://github.com/Adlink-ROS/BT_ros2.git](https://github.com/Hongtai-Yuan/Behavior_ROS2.git)
```

Build BT_ros2 with auto-docking workspace:

```bash
cd ~/autodock_ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_AUTODOCK=ON
ros2 launch bt_ros2 bt_ros2.launch.py
```

## Reference link
https://github.com/Adlink-ROS/apriltag_docking

https://github.com/Hongtai-Yuan/Autodock_Frontside_ROS2

https://github.com/Hongtai-Yuan/Behavior_ROS2
