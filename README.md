# Gazebo SAUVC environment

Gazebo classic SAUVC environment with ROS2 humble

[![](https://img.youtube.com/vi/jII8SlZvBcM/0.jpg)](https://www.youtube.com/watch?v=jII8SlZvBcM)

## Getting started

Build the code:

```
colcon build --symlink-install
. install/setup.sh
```

Launch the environment

```sh
ros2 launch sauvc_sim sauvc_launch.py
```

Topics:

```sh
# ros2 topic list
/sauvc_sim/bottom_camera/camera_info
/sauvc_sim/bottom_camera/image_raw
/sauvc_sim/front_camera/camera_info
/sauvc_sim/front_camera/depth/camera_info
/sauvc_sim/front_camera/depth/image_raw
/sauvc_sim/front_camera/image_raw
/sauvc_sim/front_camera/points
/sauvc_sim/link_states
/sauvc_sim/model_states
```

Keyboard control:

```
ros2 run sauvc_sim teleop.py
```
