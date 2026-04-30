# ROBOT SPECIFIC IMPORTS
import time
import threading
from typing import Optional

import numpy as np
import torch

# Franky imports
try:
    import franky
    HAS_FRANKY = True
except ImportError:
    HAS_FRANKY = False

# Gripper imports
try:
    import pyrobotiqgripper as rq
    HAS_ROBOTIQ = True
except ImportError:
    HAS_ROBOTIQ = False

from droid.misc.parameters import robot_ip
from droid.robot_ik.robot_ik_solver import RobotIKSolver

# UTILITY SPECIFIC IMPORTS
from droid.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler


class FrankyRobot:
    """Franky-based robot control for Franka robots."""

import os

    ROBOT_IP = os.environ.get("ROBOT_IP", "172.16.0.3")  # FR3 Robot IP from env

    def __init__(self):
        if not HAS_FRANKY:
            raise ImportError("franky is not installed. Install with: pip install franky-control")

        self._ik_solver = RobotIKSolver()
        self._robot: Optional[franky.Robot] = None
        self._gripper: Optional[object] = None
        self._control_thread: Optional[threading.Thread] = None
        self._stop_control = threading.Event()
        self._current_joint_positions: Optional[np.ndarray] = None
        self._joint_position_lock = threading.Lock()
        self._gripper_width = 0.0  # Current gripper position (0-1 normalized)
        self._max_gripper_width = 0.085  # Default for Robotiq 2F

    def launch_controller(self):
        """No-op for compatibility."""
        pass

    def launch_robot(self):
        """Initialize robot and gripper connection via Franky."""
        if self._robot is None:
            self._robot = franky.Robot(self.ROBOT_IP)
            self._robot.relative_dynamics_factor = 0.1  # Start with 10% speed

        # Gripper initialization using pyRobotiqGripper
        gripper_com_port = os.environ.get("GRIPPER_COM_PORT")
        if self._gripper is None and HAS_ROBOTIQ and gripper_com_port:
            try:
                self._gripper = rq.RobotiqGripper(com_port=gripper_com_port)
                self._gripper.connect()
                self._gripper.activate()
                time.sleep(2)
                self._gripper.calibrate_speed()
                time.sleep(1)
                self._gripper_width = 1.0  # normalized position (1 = open)
            except Exception as e:
                print(f"Warning: Gripper not available ({e}). Continuing without gripper.")
                self._gripper = None

        # Read initial state
        state = self._robot.state
        with self._joint_position_lock:
            self._current_joint_positions = np.array(state.q)

        self._controller_not_loaded = False
        print(f"Franky robot initialized: IP={self.ROBOT_IP}")

    def kill_controller(self):
        """Stop any ongoing control."""
        self._stop_control.set()
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2)

    def read_once(self):
        """Read robot state (simplified for Franky compatibility)."""
        if self._robot is None:
            raise RuntimeError("Robot not initialized. Call launch_robot() first.")
        state = self._robot.state
        return {
            "q": list(state.q),
            "dq": list(state.dq),
            "O_T_EE": list(state.O_T_EE),
            "tau_J": list(state.tau_J) if hasattr(state, 'tau_J') else [0.0] * 7,
            "time": state.time if hasattr(state, 'time') else 0,
        }

    def _enter_joint_control_mode(self):
        """Enter joint position control mode - must be called before motion."""
        # Recover from any errors
        try:
            self._robot.recover_from_errors()
        except:
            pass

        # Stop any current motion
        self._robot.stop()

        # Wait for velocities to settle
        for _ in range(40):  # Wait up to 2 seconds
            dq = np.abs(np.array(self._robot.state.dq))
            if dq.max() < 0.005:
                break
            time.sleep(0.05)

        # Enter joint position control mode with explicit stop
        try:
            self._robot.move(franky.JointStopMotion())
        except:
            pass

        # Wait longer for control to stabilize
        time.sleep(1.0)

    def move_to_joint_positions(self, positions, speed_factor=0.1, timeout=30):
        """Move to joint positions using Franky."""
        if self._robot is None:
            raise RuntimeError("Robot not initialized")

        positions = np.array(positions)
        if len(positions) != 7:
            raise ValueError("Expected 7 joint positions")

        # Set speed
        self._robot.relative_dynamics_factor = speed_factor

        # Use JointMotion for smooth trajectory (ruckig handles this internally)
        motion = franky.JointMotion(positions.tolist())

        # Execute motion (asynchronous so new motions preempt old ones automatically)
        try:
            self._robot.move(motion, asynchronous=True)
        except franky.ControlException:
            # Async move preempted by new command - this is OK for teleop
            pass

        # Update current positions
        with self._joint_position_lock:
            self._current_joint_positions = positions

        return True

    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        """Update joint positions."""
        command = np.array(command)

        if velocity:
            with self._joint_position_lock:
                current = np.array(self._current_joint_positions) if self._current_joint_positions is not None else np.zeros(7)
            joint_delta = self._ik_solver.joint_velocity_to_delta(command)
            target = (joint_delta + current).tolist()
        else:
            target = command.tolist()

        if blocking:
            self.move_to_joint_positions(target)
        else:
            thread = threading.Thread(target=self.move_to_joint_positions, args=(target,))
            thread.start()

    def update_gripper(self, command, velocity=True, blocking=False):
        """Update gripper position using pyRobotiqGripper."""
        if self._gripper is None:
            print("Warning: Gripper not initialized")
            return

        if velocity:
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(command)
            target_pos = self._gripper_width + gripper_delta
            target_pos = float(np.clip(target_pos, 0, 1))
        else:
            # command is 0-1 normalized, 1=open, 0=closed
            target_pos = float(np.clip(command, 0, 1))

        # Convert normalized position (0-1) to gripper command (0-255)
        # 1 (open) → 255, 0 (closed) → 0
        target_bits = int(target_pos * 255)

        print(f"[FRANKY] update_gripper: cmd={command}, velocity={velocity}, target_pos={target_pos:.3f}, target_bits={target_bits}, blocking={blocking}")

        try:
            self._gripper.realTimeMove(target_bits)
            self._gripper_width = target_pos
        except Exception as e:
            print(f"Error updating gripper: {e}")

    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space="velocity", blocking=False):
        """Main entry point for RobotEnv.update_robot() - maps action to robot command."""
        action_dict = self.create_action_dict(
            action=np.array(command),
            action_space=action_space,
            gripper_action_space=gripper_action_space
        )
        
        # Execute based on action_space type
        if "cartesian" in action_space:
            joint_position = action_dict.get("joint_position")
            if joint_position is not None:
                self.update_joints(joint_position, velocity=False, blocking=blocking)
        elif "joint" in action_space:
            joint_position = action_dict.get("joint_position")
            if joint_position is not None:
                self.update_joints(joint_position, velocity=False, blocking=blocking)

        # Update gripper
        gripper_pos = action_dict.get("gripper_position")
        if gripper_pos is not None:
            self.update_gripper(gripper_pos, velocity=False, blocking=blocking)

        # Convert numpy arrays to lists for zerorpc serialization
        return self._convert_action_dict(action_dict)

    def _convert_action_dict(self, action_dict):
        """Convert numpy arrays to lists for zerorpc serialization."""
        result = {}
        for key, value in action_dict.items():
            if key == "robot_state":
                result[key] = value  # robot_state is already a dict
            elif isinstance(value, np.ndarray):
                result[key] = value.tolist()
            elif isinstance(value, (list, tuple)):
                result[key] = list(value)
            else:
                result[key] = value
        return result

    def get_joint_positions(self):
        """Get current joint positions."""
        with self._joint_position_lock:
            if self._current_joint_positions is not None:
                return list(self._current_joint_positions)
            else:
                state = self._robot.state
                return list(state.q)

    def get_joint_velocities(self):
        """Get current joint velocities."""
        state = self._robot.state
        return list(state.dq)

    def get_gripper_position(self):
        """Get gripper position (normalized 0-1, 1=open, 0=closed)."""
        if self._gripper is None:
            return self._gripper_width
        try:
            pos_bits = self._gripper.position()
            self._gripper_width = pos_bits / 255.0
            return float(np.clip(self._gripper_width, 0, 1))
        except Exception as e:
            print(f"Error getting gripper position: {e}")
            return self._gripper_width

    def get_ee_pose(self):
        """Get end-effector pose as [position, euler_angles]."""
        cartesian_state = self._robot.current_cartesian_state
        # pose is RobotPose with end_effector_pose (Affine)
        ee_pose = cartesian_state.pose.end_effector_pose

        # Get position - translation can be numpy array
        position = np.array(ee_pose.translation)
        if position.ndim > 0:
            position = position.flatten()

        # Get rotation as quaternion and convert to euler
        quat = np.array(ee_pose.quaternion)
        if quat.ndim > 0:
            quat = quat.flatten()
        euler = self._quat_to_euler(quat.tolist())

        return (np.concatenate([position, euler])).tolist()

    def _quat_to_euler(self, quat):
        """Convert quaternion to Euler angles (XYZ convention)."""
        x, y, z, w = quat
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return np.array([roll, pitch, yaw])

    def get_robot_state(self):
        """Get full robot state as a tuple (state_dict, timestamp_dict)."""
        state = self._robot.state

        ee_pose = self.get_ee_pose()
        gripper_position = self.get_gripper_position()

        state_dict = {
            "cartesian_position": ee_pose,
            "gripper_position": gripper_position,
            "joint_positions": list(state.q),
            "joint_velocities": list(state.dq),
            "joint_torques_computed": list(state.tau_J) if hasattr(state, 'tau_J') else [0.0] * 7,
            "prev_joint_torques_computed": list(state.tau_J) if hasattr(state, 'tau_J') else [0.0] * 7,
            "prev_joint_torques_computed_safened": list(state.tau_J) if hasattr(state, 'tau_J') else [0.0] * 7,
            "motor_torques_measured": list(state.tau_J) if hasattr(state, 'tau_J') else [0.0] * 7,
            "prev_controller_latency_ms": 0,
            "prev_command_successful": True,
        }

        timestamp_dict = {
            "robot_timestamp_seconds": int(state.time.to_sec()) if hasattr(state.time, 'to_sec') else 0,
            "robot_timestamp_nanos": 0,
        }

        return state_dict, timestamp_dict

    def is_running_policy(self):
        """Check if robot is currently moving."""
        return False

    def start_cartesian_impedance(self):
        """Start Cartesian impedance control mode. No-op in this version."""
        pass

    def update_desired_joint_positions(self, positions):
        """Update desired joint positions (non-blocking)."""
        positions_list = positions.tolist() if hasattr(positions, 'tolist') else list(positions)
        thread = threading.Thread(target=self.move_to_joint_positions, args=(positions_list,))
        thread.start()

    def terminate_current_policy(self):
        """Terminate current motion."""
        self.kill_controller()

    def adaptive_time_to_go(self, desired_joint_position, t_min=0, t_max=4):
        """Calculate time to go for motion."""
        curr = np.array(self.get_joint_positions())
        displacement = np.abs(np.array(desired_joint_position) - curr).max()
        return min(t_max, max(t_min, displacement * 2))

    def create_action_dict(self, action, action_space, gripper_action_space=None, robot_state=None):
        """Create action dictionary from raw action."""
        action = np.array(action)
        if robot_state is None:
            robot_state = self.get_robot_state()[0]

        action_dict = {"robot_state": robot_state}
        velocity = "velocity" in action_space

        if gripper_action_space is None:
            gripper_action_space = "velocity"

        # Handle gripper action
        if gripper_action_space == "velocity":
            action_dict["gripper_velocity"] = float(action[-1])
            # action[-1] is trigger value (0-1): 1=fully pressed (close), 0=released (open)
            # Direct position mapping: trigger directly maps to target position
            trigger_value = float(action[-1])
            target_pos = trigger_value  # No inversion - press=close, release=open
            print(f"[FRANKY] gripper: trigger={trigger_value:.3f}, target={target_pos:.3f}")
            action_dict["gripper_position"] = float(np.clip(target_pos, 0, 1))
        else:
            action_dict["gripper_position"] = float(np.clip(action[-1], 0, 1))

        # Handle position/velocity control
        if "cartesian" in action_space:
            if velocity:
                action_dict["cartesian_velocity"] = action[:-1].tolist()
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(action[:-1])
                action_dict["cartesian_position"] = add_poses(
                    cartesian_delta, robot_state["cartesian_position"]
                ).tolist()
                action_dict["joint_velocity"] = self._ik_solver.cartesian_velocity_to_joint_velocity(
                    action_dict["cartesian_velocity"], robot_state=robot_state
                ).tolist()
                joint_delta = self._ik_solver.joint_velocity_to_delta(action_dict["joint_velocity"])
                action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
            else:
                action_dict["cartesian_position"] = action[:-1].tolist()
                cartesian_delta = pose_diff(action[:-1], robot_state["cartesian_position"])
                cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
                action_dict["joint_velocity"] = self._ik_solver.cartesian_velocity_to_joint_velocity(
                    cartesian_velocity, robot_state=robot_state
                ).tolist()
                joint_delta = self._ik_solver.joint_velocity_to_delta(action_dict["joint_velocity"])
                action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()

        if "joint" in action_space:
            if velocity:
                action_dict["joint_velocity"] = action[:7].tolist()
                joint_delta = self._ik_solver.joint_velocity_to_delta(action[:7])
                action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
            else:
                action_dict["joint_position"] = action.tolist()

        return action_dict
