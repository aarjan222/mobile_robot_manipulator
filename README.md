### Build the repo
```
rm -rf install/ log/ build/; colcon build --symlink-install
```

### Source the repo
```
source install/setup.bash
```

### Launch file for Gazebo + Rviz2 + Moveit2 (Move base & robotic arm)
```
ros2 launch irb120_ros2_moveit2 irb120.launch.py
```

### Launch file for grasping the object
```
ros2 launch irb120_ros2_moveit2 irb120_interface.launch.py
```

### For spawning the object
```
ros2 run ros2_grasping spawn_object.py --package "ros2_grasping" --urdf "box.urdf" --name "box" --x 0.5 --y -0.3 --z 0.75
```

### For picking and placing the object
```
ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="cubePP" -p ROBOT_MODEL:="irb120" -p EE_MODEL:="schunk"
```

### Drive the robot
In another terminal
```
ros2 run rqt_robot_steering rqt_robot_steering
```