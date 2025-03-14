### Download World Repo and add path to environment variable
```sh
git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to this repo>/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:<path to this repo>/worlds
```
### Build the repo
```
rm -rf install/ log/ build/; colcon build --symlink-install
```

### Source the repo
```
source install/setup.bash; ros2 launch mobile_manipulator_body arm_gazebo_control.launch.py
```

### Drive the robot
In another terminal
```
ros2 run rqt_robot_steering rqt_robot_steering
```

### Moving the Robotic Arm
Move robot arm to some fixed position
```
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["arm_base_joint", "shoulder_joint", "bottom_wrist_joint", "elbow_joint", "top_wrist_joint"],
  points: [{
    positions: [-0.1, 0.5, 0.02, 0, 0],
    time_from_start: {sec: 1, nanosec: 0}
  }]
}'
```

Move robot arm to home position
```
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["arm_base_joint", "shoulder_joint", "bottom_wrist_joint", "elbow_joint", "top_wrist_joint"],
  points: [{
    positions: [0, 0, 0, 0, 0],
    time_from_start: {sec: 1, nanosec: 0}
  }]
}'
```

### For Localization and Navigation
(slam_toolbox for localization)`
```sh
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/mobile_robot_manipulator/src/mobile_manipulator_body/config/mapper_params_online_async.yaml use_sim_time:=true
```
(nav2 for navigation)
```sh
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```