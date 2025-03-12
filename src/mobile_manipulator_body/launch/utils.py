clr; colcon build --symlink-install; source install/setup.bash; ros2 launch mobile_manipulator_body arm_gazebo_control.launch.py 


ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["arm_base_joint", "shoulder_joint", "bottom_wrist_joint", "elbow_joint", "top_wrist_joint"],
  points: [{
    positions: [-0.1, 0.5, 0.02, 0, 0],
    time_from_start: {sec: 1, nanosec: 0}
  }]
}'

ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["arm_base_joint", "shoulder_joint", "bottom_wrist_joint", "elbow_joint", "top_wrist_joint"],
  points: [{
    positions: [0, 0, 0, 0, 0],
    time_from_start: {sec: 1, nanosec: 0}
  }]
}'