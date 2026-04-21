# Lab 3 Commands

## Setup (run once)
```bash
cd ~/techin517/ros2_ws
colcon build --symlink-install
source ~/techin517/ros2_ws/install/setup.bash
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1
```

---

## Part A: Record Joint States via Teleoperation

**Terminal 1** — Launch robot + teleop:
```bash
source ~/techin517/ros2_ws/install/setup.bash
ros2 launch soa_bringup soa_bringup.launch.py leader:=true
```

**Terminal 2** — Start save service:
```bash
source ~/techin517/ros2_ws/install/setup.bash
ros2 run soa_functions save_joint_states
```

**Terminal 3** — Create CSV folder, then save one pose at a time:
```bash
mkdir -p /home/ubuntu/techin517/soa_ws

ros2 service call /follower/save_joint_states soa_interfaces/srv/SaveJointStates \
  "{csv_path: '/home/ubuntu/techin517/soa_ws/joints.csv'}"
```
> Move leader arm to each pose, then call the service. Repeat 5+ times.

---

## Part B: Run the Full Demo with MoveIt

> Stop all terminals from Part A first (Ctrl+C), then:

```bash
sudo pkill -f ros2
```

**Terminal 1** — Launch MoveIt:
```bash
source ~/techin517/ros2_ws/install/setup.bash
ros2 launch soa_moveit_config soa_moveit_bringup.launch.py
```

**Terminal 2** — Joint states action server:
```bash
source ~/techin517/ros2_ws/install/setup.bash
ros2 run soa_functions move_to_joint_states_server
```

**Terminal 3** — Gripper action server:
```bash
source ~/techin517/ros2_ws/install/setup.bash
ros2 run soa_functions gripper_server
```

**Terminal 4** — Run the app:
```bash
source ~/techin517/ros2_ws/install/setup.bash
ros2 run soa_apps go_to_joint_states
```

> The arm will move through all poses in the CSV. Film this for Deliverable 6.

---

## Calibration (only if needed)
```bash
# Follower
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=gix-follower1 \
  --robot.calibration_dir=/home/ubuntu/techin517/my_calibration

# Leader
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=gix-leader1 \
  --teleop.calibration_dir=/home/ubuntu/techin517/my_calibration
```

## Teleop Test (only if needed)
```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=gix-follower1 \
  --robot.calibration_dir=/home/ubuntu/techin517/my_calibration \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=gix-leader1 \
  --teleop.calibration_dir=/home/ubuntu/techin517/my_calibration
```
