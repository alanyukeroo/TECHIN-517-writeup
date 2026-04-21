# Lab 3 Report: MoveIt Forward Kinematics

## Deliverable 1: `save_joint_states` Service

```python
"""Save joint states service node.

Provides the /follower/save_joint_states service (soa_interfaces/srv/SaveJointStates)
to capture the current joint states of the follower arm and optionally append them
to a CSV file for later analysis or replay.

The joint state positions are recorded with column headers derived from the joint
names in the JointState message.

Can be run standalone without a namespace argument:
    ros2 run soa_functions save_joint_states

Services:
    /follower/save_joint_states (soa_interfaces/srv/SaveJointStates)
        request:  csv_path — path to CSV file; if empty, joint states are not saved
        response: success, joint_states

Subscriptions:
    /follower/joint_states (sensor_msgs/JointState)
"""

import csv
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from soa_interfaces.srv import SaveJointStates


class SaveJointStatesNode(Node):

    def __init__(self):
        super().__init__('save_joint_states')

        self._latest_js: JointState | None = None
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            JointState,
            '/follower/joint_states',
            lambda msg: setattr(self, '_latest_js', msg),
            10,
            callback_group=self._cb_group,
        )

        self.create_service(
            SaveJointStates,
            '/follower/save_joint_states',
            self._handle_save_joint_states,
            callback_group=self._cb_group,
        )

        self.get_logger().info('SaveJointStates service ready.')

    def _handle_save_joint_states(self, req, res):
        if self._latest_js is None:
            self.get_logger().warn('No joint states received yet')
            res.success = False
            return res

        res.joint_states = self._latest_js
        res.success = True

        if req.csv_path:
            try:
                self._append_to_csv(req.csv_path, self._latest_js)
            except OSError as e:
                self.get_logger().error(f'Failed to write CSV: {e}')
                res.success = False

        return res

    def _append_to_csv(self, path: str, js: JointState) -> None:
        file_exists = os.path.isfile(path)
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=js.name)
            if not file_exists:
                writer.writeheader()
            writer.writerow(dict(zip(js.name, js.position)))
        self.get_logger().info(f'Saved joint states to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = SaveJointStatesNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

---

## Deliverable 2: JointState Message Type Documentation

Official ROS2 Humble documentation for the `sensor_msgs/JointState` message type:

https://docs.ros2.org/humble/api/sensor_msgs/msg/JointState.html

The `JointState` message contains:
- `header` — timestamp and frame ID
- `name` — list of joint names (string[])
- `position` — joint positions in radians (float64[])
- `velocity` — joint velocities in rad/s (float64[])
- `effort` — joint efforts in N·m (float64[])

---

## Deliverable 3: Joint States CSV

Poses recorded via teleoperation for a pick-and-place sequence:

```
shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper
-0.02914563496982718,-0.13652429012182207,-0.6580777580029401,1.021631204731837,1.4618836908550161,-0.09970875121256667
-0.010737865515199488,1.009359358428752,-0.5859806609723149,1.6152817696435802,-0.13192234775816514,1.932815792735908
-0.039883500485026674,1.009359358428752,-0.7317088358214509,1.4956312681885002,-0.04141748127291231,0.0260776733940559
0.2638446955163303,0.05522330836388308,-0.9035146840646426,1.6505633277649499,-0.056757289151768725,0.0260776733940559
0.6826214506091103,0.6642136811544826,-0.9019807032767571,0.4724660826687775,0.7133010663668231,0.0260776733940559
0.6089903727905995,0.6642136811544826,-1.0799224746714913,0.8130098175793898,0.6442719309119693,1.2501943421267976
0.0046019423636569235,-1.8116313104929422,1.5508545765523831,-1.8285050991596843,0.0337475773334841,1.2578642460662257
```

---

## Deliverable 4: `move_to_joint_states_server`

```python
#!/usr/bin/env python3
"""
MoveToJointStates action server for the SOA 5-DOF arm.

Uses pymoveit2 to plan and execute joint-space motion to a target configuration.
Joint names may be specified in the goal; if omitted, all 5 arm joints are assumed
in order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.

Usage:
    ros2 run soa_functions move_to_joint_states_server
"""

import time
from threading import Thread

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State

from soa_interfaces.action import MoveToJointStates
from soa_functions import soa_robot


_JOINT_LIMITS = {
    'shoulder_pan':  (-2.04786, 2.04786),
    'shoulder_lift': (-1.89089, 1.89089),
    'elbow_flex':    (-1.69021, 1.69021),
    'wrist_flex':    (-1.78913, 1.78913),
    'wrist_roll':    (-2.99306, 2.99306),
}


class MoveToJointStatesServer(Node):

    def __init__(self):
        super().__init__('move_to_joint_states_server')

        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('allowed_planning_time', 3.0)

        self._cb_group = ReentrantCallbackGroup()

        self._moveit2 = MoveIt2(
            node=self,
            joint_names=soa_robot.joint_names(),
            base_link_name=soa_robot.base_link_name(),
            end_effector_name=soa_robot.end_effector_name(),
            group_name=soa_robot.MOVE_GROUP_ARM,
            callback_group=self._cb_group,
        )

        self._moveit2.max_velocity = (
            self.get_parameter('max_velocity').get_parameter_value().double_value
        )
        self._moveit2.max_acceleration = (
            self.get_parameter('max_acceleration').get_parameter_value().double_value
        )
        self._moveit2.num_planning_attempts = (
            self.get_parameter('num_planning_attempts').get_parameter_value().integer_value
        )
        self._moveit2.allowed_planning_time = (
            self.get_parameter('allowed_planning_time').get_parameter_value().double_value
        )

        self._action_server = ActionServer(
            self,
            MoveToJointStates,
            'move_to_joint_states',
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('MoveToJointStates action server ready')

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Received MoveToJointStates goal')

        joint_positions = list(goal_handle.request.joint_positions)
        joint_names = list(goal_handle.request.joint_names)

        if not joint_names:
            joint_names = soa_robot.joint_names()

        result = MoveToJointStates.Result()

        # Validation: length
        if len(joint_positions) != len(joint_names):
            goal_handle.abort()
            result.success = False
            result.message = (
                f'Length mismatch: {len(joint_positions)} positions vs {len(joint_names)} names'
            )
            self.get_logger().warn(result.message)
            return result

        # Validation: joint limits
        for name, pos in zip(joint_names, joint_positions):
            if name not in _JOINT_LIMITS:
                goal_handle.abort()
                result.success = False
                result.message = f'Unknown joint name: {name}'
                self.get_logger().warn(result.message)
                return result
            lo, hi = _JOINT_LIMITS[name]
            if not (lo <= pos <= hi):
                goal_handle.abort()
                result.success = False
                result.message = f'{name}={pos:.4f} out of limits [{lo:.4f}, {hi:.4f}]'
                self.get_logger().warn(result.message)
                return result

        future = self._moveit2.plan_async(
            joint_positions=joint_positions,
            joint_names=joint_names,
            start_joint_state=self._moveit2.joint_state,
        )

        if future is None:
            goal_handle.abort()
            result.success = False
            result.message = 'plan_async() returned None'
            return result

        while not future.done():
            time.sleep(0.1)

        trajectory = self._moveit2.get_trajectory(future)
        if trajectory is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Planning failed: no trajectory found'
            return result

        self._moveit2.execute(trajectory)
        success = self._wait_and_publish_feedback(goal_handle, joint_names, joint_positions)

        if success:
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached target joint configuration'
        else:
            goal_handle.abort()
            result.success = False
            result.message = 'Execution failed'

        return result

    def _wait_and_publish_feedback(self, goal_handle, joint_names, target_positions):
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            self._publish_feedback(goal_handle, joint_names, target_positions)
            time.sleep(0.1)
        self._publish_feedback(goal_handle, joint_names, target_positions)
        return self._moveit2.motion_suceeded

    def _publish_feedback(self, goal_handle, joint_names, target_positions):
        feedback = MoveToJointStates.Feedback()
        try:
            js = self._moveit2.joint_state
            if js is not None and js.name:
                name_to_pos = dict(zip(js.name, js.position))
                errors = [
                    abs(name_to_pos[n] - t)
                    for n, t in zip(joint_names, target_positions)
                    if n in name_to_pos
                ]
                feedback.max_joint_error = max(errors) if errors else -1.0
            else:
                feedback.max_joint_error = -1.0
        except Exception:
            feedback.max_joint_error = -1.0
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToJointStatesServer()
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    time.sleep(1.0)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```

---

## Deliverable 5: `go_to_joint_states` App

```python
#!/usr/bin/env python3
"""
Iterate through a CSV of joint states and drive the SOA arm + gripper to each row in order.
"""

import csv

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from soa_interfaces.action import Gripper, MoveToJointStates


DEFAULT_CSV_PATH = '/home/ubuntu/techin517/soa_ws/joints.csv'

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def load_rows(path: str) -> list:
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


class GoToJointStates(Node):

    def __init__(self):
        super().__init__('go_to_joint_states')

        self.declare_parameter('csv_path', DEFAULT_CSV_PATH)

        self._joint_client = ActionClient(self, MoveToJointStates, 'move_to_joint_states')
        self._gripper_client = ActionClient(self, Gripper, 'gripper_command')

    def send_joint_goal(self, joint_positions: list, joint_names: list, label: str = '') -> bool:
        goal = MoveToJointStates.Goal()
        goal.joint_positions = joint_positions
        goal.joint_names = joint_names

        self.get_logger().info(
            f'Sending joint goal ({label}): '
            + ', '.join(f'{n}={p:.4f}' for n, p in zip(joint_names, joint_positions))
        )

        self._joint_client.wait_for_server()

        future = self._joint_client.send_goal_async(
            goal, feedback_callback=self._joint_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Joint goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Joint goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Joint goal failed: {result.message}')
        return result.success

    def send_gripper_goal(self, target_position: float, label: str = '') -> bool:
        goal = Gripper.Goal()
        goal.target_position = target_position

        self.get_logger().info(f'Sending gripper goal ({label}): position={target_position:.4f}')

        self._gripper_client.wait_for_server()

        future = self._gripper_client.send_goal_async(
            goal, feedback_callback=self._gripper_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Gripper goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Gripper goal failed: {result.message}')
        return result.success

    def _joint_feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Joint feedback: max_error={feedback_msg.feedback.max_joint_error:.4f} rad'
        )

    def _gripper_feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Gripper feedback: position={feedback_msg.feedback.current_position:.4f}'
        )

    def run(self):
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading joint states from: {csv_path}')

        rows = load_rows(csv_path)
        self.get_logger().info(f'Loaded {len(rows)} row(s)')

        self.get_logger().info('=== Starting go_to_joint_states sequence ===')

        for i, row in enumerate(rows):
            self.get_logger().info(f'--- Row {i + 1}/{len(rows)} ---')

            positions = [float(row[j]) for j in ARM_JOINTS]
            success = self.send_joint_goal(positions, ARM_JOINTS, label=f'row {i}')
            if not success:
                self.get_logger().error(f'Arm goal failed at row {i}. Aborting.')
                return

            success = self.send_gripper_goal(float(row['gripper']), label=f'row {i} gripper')
            if not success:
                self.get_logger().error(f'Gripper goal failed at row {i}. Aborting.')
                return

        self.get_logger().info('=== go_to_joint_states sequence complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = GoToJointStates()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```

---

## Deliverable 6: Demo Video

https://youtube.com/shorts/5iH9mMm9pj4?si=EBt3lH0F7Z8JFzuq

---

## Deliverable 7: Forward Kinematics — Strengths and Weaknesses

<!-- Write this paragraph yourself (no AI) -->
<!-- Think about: precision, repeatability, simplicity vs. adaptability, calibration sensitivity -->

Forward kinematics works best in situations where the path or pose of every joint is known ahead of time and needs to be replayed consistently. Animation and game engines are a natural fit. A character's walk cycle, for example, can be authored joint by joint and then looped without the system ever needing to figure out where the foot should land. Similarly, industrial robots running preprogrammed assembly routines benefit from forward kinematics because the motion is fixed, repeatable, and does not require any real-time solving.
The main strength of controlling joints directly is simplicity and speed. There is no math-heavy solving step. You set the angles, and the end-effector position follows immediately. This makes the system predictable and easy to debug.
The weakness shows up the moment you care more about where the end-effector ends up than about how the joints got there. If you need a robotic arm to reach a specific point in space, forward kinematics gives you no direct way to figure out what joint angles produce that result. You would have to guess and check, which is the job of inverse kinematics. Forward kinematics also struggles with tasks that require reacting to the environment, since it has no built-in mechanism to adjust joint values based on where a target actually is.
