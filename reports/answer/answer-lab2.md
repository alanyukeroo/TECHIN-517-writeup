Lab 2 Submission Packet
Course Lab: Lab 2: Lerobot Ex ROS
Date: 2026-04-15

1. Rosetta Contract
robot_type: soa

fps: 30
max_duration_s: 15.0

observations:
  - key: observation.images.wrist
    topic: /follower/image_raw
    type: sensor_msgs/msg/Image
    image:
      resize: [480, 640]
    align: {strategy: hold, stamp: header}
    qos: {reliability: best_effort, history: keep_last, depth: 10}

  - key: observation.images.overhead
    topic: /static_camera/overhead_cam/color/image_raw
    type: sensor_msgs/msg/Image
    image:
      resize: [720, 1280]
    align: {strategy: hold, stamp: header}
    qos: {reliability: best_effort, history: keep_last, depth: 10}

  - key: observation.state
    topic: /follower/joint_states
    type: sensor_msgs/msg/JointState
    selector:
      names:
        - position.shoulder_pan
        - position.shoulder_lift
        - position.elbow_flex
        - position.wrist_flex
        - position.wrist_roll
        - position.gripper
    align: {strategy: hold, stamp: header}
    qos: {reliability: best_effort, history: keep_last, depth: 50}
    unit_conversion: rad2deg

actions:
  - key: action
    publish:
      topic: /follower/arm_fwd_controller/commands
      type: std_msgs/msg/Float64MultiArray
      qos: {reliability: reliable, history: keep_last, depth: 10}
    selector:
      names:
        - position.shoulder_pan
        - position.shoulder_lift
        - position.elbow_flex
        - position.wrist_flex
        - position.wrist_roll
    unit_conversion: rad2deg
    safety_behavior: hold

  - key: action
    publish:
      topic: /follower/gripper_fwd_controller/commands
      type: std_msgs/msg/Float64MultiArray
      qos: {reliability: reliable, history: keep_last, depth: 10}
    selector:
      names:
        - position.gripper
    unit_conversion: rad2deg
    safety_behavior: hold

recording:
  storage: mcap
2. Model And Task
• Trained run: act_so101_2cam_v3-20260416T004131Z-3-002
• Deployment model path: /home/ubuntu/techin517/act_so101_2cam_v3-20260416T004131Z-3-002/act_so101_2cam_v3/checkpoints/last/pretrained_model
• Task prompt: grab the aruco and put it on the yellow basket
• Camera setup: wrist camera + overhead Intel RealSense
3. Commands Used
Bring up the robot in forward-controller mode
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 launch soa_bringup soa_bringup.launch.py controller:=forward cameras:=true
Start the controller switch service
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 run soa_functions controller_switcher
Start Rosetta with the ACT policy
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 launch rosetta rosetta_client_launch.py \
  contract_path:=/home/ubuntu/techin517/ros2_ws/src/soa_ros2/soa_bringup/rosetta_contracts/soa_act_contract.yaml \
  pretrained_name_or_path:=/home/ubuntu/techin517/act_so101_2cam_v3-20260416T004131Z-3-002/act_so101_2cam_v3/checkpoints/last/pretrained_model
Run the policy
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 action send_goal /run_policy \
  rosetta_interfaces/action/RunPolicy \
  "{prompt: 'grab the aruco and put it on the yellow basket'}"
4. Controller Switching Commands
List controllers
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 control list_controllers --controller-manager /follower/controller_manager
Switch from forward controllers to joint trajectory controllers
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 control switch_controllers \
  --controller-manager /follower/controller_manager \
  --deactivate arm_fwd_controller gripper_fwd_controller \
  --activate arm_controller gripper_controller
Switch back from joint trajectory controllers to forward controllers
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 control switch_controllers \
  --controller-manager /follower/controller_manager \
  --deactivate arm_controller gripper_controller \
  --activate arm_fwd_controller gripper_fwd_controller
Switch controllers using the Lab 2 service
cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 service call /controller_switcher/switch_controller \
  soa_interfaces/srv/SwitchController \
  "{controller_type: forward}"

cd /home/ubuntu/techin517
source /opt/ros/humble/setup.bash
source /home/ubuntu/techin517/ros2_ws/install/setup.bash
ros2 service call /controller_switcher/switch_controller \
  soa_interfaces/srv/SwitchController \
  "{controller_type: jtc}"
5. Video Link
https://drive.google.com/file/d/1OsM2ZlLQ8uVritTBjHpV-IHZaXtttfN4/view?usp=sharing
6. Reflection Paragraph
To be written by student without AI, per assignment instructions.
