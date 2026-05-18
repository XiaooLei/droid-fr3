import math
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def add_dynamixel_sdk_path(sample_dir: Optional[str] = None):
    candidate_dirs = []
    if sample_dir is not None:
        candidate_dirs.append(Path(sample_dir).expanduser())
    candidate_dirs.extend([PROJECT_ROOT / "dynamixel_sample_test", Path("/tmp/dynamixel_sample_test")])

    for candidate_dir in candidate_dirs:
        sdk_src = candidate_dir / "sdk" / "src"
        if sdk_src.exists():
            if str(sdk_src) not in sys.path:
                sys.path.insert(0, str(sdk_src))
            return


add_dynamixel_sdk_path()

COMM_SUCCESS = GroupSyncRead = PacketHandler = PortHandler = None
_DYNAMIXEL_IMPORT_ERROR = None


def import_dynamixel_sdk():
    global COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler, _DYNAMIXEL_IMPORT_ERROR
    if GroupSyncRead is not None:
        return
    try:
        from dynamixel_sdk import COMM_SUCCESS as sdk_comm_success
        from dynamixel_sdk import GroupSyncRead as sdk_group_sync_read
        from dynamixel_sdk import PacketHandler as sdk_packet_handler
        from dynamixel_sdk import PortHandler as sdk_port_handler
    except ModuleNotFoundError as exc:
        _DYNAMIXEL_IMPORT_ERROR = exc
        if exc.name == "serial":
            raise ModuleNotFoundError("缺少 pyserial，请先执行: python3 -m pip install pyserial") from exc
        raise ModuleNotFoundError(
            "找不到 dynamixel_sdk，请设置 --master-sample-dir 指向 dynamixel_sample_test，"
            "或把 dynamixel_sample_test 放到 repo 根目录或 /tmp。"
        ) from exc

    COMM_SUCCESS = sdk_comm_success
    GroupSyncRead = sdk_group_sync_read
    PacketHandler = sdk_packet_handler
    PortHandler = sdk_port_handler


try:
    import_dynamixel_sdk()
except ModuleNotFoundError as exc:
    _DYNAMIXEL_IMPORT_ERROR = exc


PROTOCOL_VERSION = 2.0
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
PULSE_PER_REVOLUTION = 4096
RAD_PER_PULSE = (2.0 * math.pi) / PULSE_PER_REVOLUTION


def uint32_to_int32(value: int) -> int:
    value &= 0xFFFFFFFF
    if value & 0x80000000:
        return value - 0x100000000
    return value


def position_to_radian(raw_position: int) -> float:
    return uint32_to_int32(raw_position) * RAD_PER_PULSE


class DynamixelReader:
    def __init__(
        self,
        port: str,
        ids: List[int],
        baudrate: int = 1000000,
        frequency: float = 50.0,
        sample_dir: Optional[str] = None,
    ):
        add_dynamixel_sdk_path(sample_dir)
        import_dynamixel_sdk()
        if frequency <= 0:
            raise ValueError("frequency must be greater than 0")
        if not ids:
            raise ValueError("ids must not be empty")

        self.port = port
        self.ids = list(ids)
        self.baudrate = baudrate
        self.frequency = frequency

        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(PROTOCOL_VERSION)
        self._group_sync_read = None
        self._lock = threading.Lock()
        self._positions_rad: Dict[int, float] = {}
        self._last_read_time = 0.0
        self._last_error: Optional[str] = None
        self._running = False
        self._thread = None

    def start(self):
        if self._running:
            return

        if not self._port_handler.openPort():
            raise RuntimeError(f"打开串口失败: {self.port}")
        if not self._port_handler.setBaudRate(self.baudrate):
            self._port_handler.closePort()
            raise RuntimeError(f"设置波特率失败: {self.baudrate}")

        self._group_sync_read = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        for dxl_id in self.ids:
            if not self._group_sync_read.addParam(dxl_id):
                self._port_handler.closePort()
                raise RuntimeError(f"[ID:{dxl_id:03d}] groupSyncRead addParam 失败")

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1)
        if self._group_sync_read is not None:
            self._group_sync_read.clearParam()
        self._port_handler.closePort()

    def _read_loop(self):
        period_s = 1.0 / self.frequency
        next_tick = time.monotonic()
        while self._running:
            try:
                positions_rad = self.read_once()
                with self._lock:
                    self._positions_rad = positions_rad
                    self._last_read_time = time.time()
                    self._last_error = None
            except RuntimeError as exc:
                with self._lock:
                    self._last_error = str(exc)

            next_tick += period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    def read_once(self) -> Dict[int, float]:
        dxl_comm_result = self._group_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(dxl_comm_result))

        positions_rad = {}
        for dxl_id in self.ids:
            if not self._group_sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"[ID:{dxl_id:03d}] groupSyncRead 数据不可用")
            raw_position = self._group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            positions_rad[dxl_id] = position_to_radian(raw_position)
        return positions_rad

    def get_positions(self) -> Dict[int, float]:
        with self._lock:
            return dict(self._positions_rad)

    def get_status(self, timeout_s: float = 1.0) -> Dict[str, object]:
        with self._lock:
            last_read_age = time.time() - self._last_read_time if self._last_read_time else float("inf")
            return {
                "connected": last_read_age < timeout_s,
                "last_read_time": self._last_read_time,
                "last_read_age": last_read_age,
                "last_error": self._last_error,
            }


class DynamixelMasterPolicy:
    def __init__(
        self,
        port: str,
        ids: List[int],
        joint_ids: Optional[List[int]] = None,
        gripper_id: Optional[int] = 8,
        sample_dir: Optional[str] = None,
        baudrate: int = 1000000,
        frequency: float = 50.0,
        joint_signs: Optional[List[float]] = None,
        joint_offsets: Optional[List[float]] = None,
        active_joints: Optional[List[int]] = None,
        joint_action_gain: float = 0.9,
        max_target_delta: float = 2.0,
        gripper_mode: str = "calibrated",
        gripper_action_gain: float = 3.0,
        gripper_master_open: Optional[float] = 3.446855,
        gripper_master_closed: Optional[float] = 2.523398,
        gripper_open_command: float = 0.0,
        gripper_closed_command: float = 1.0,
    ):
        self.ids = list(ids)
        self.joint_ids = list(joint_ids) if joint_ids is not None else self.ids[:7]
        self.gripper_id = gripper_id
        if len(self.joint_ids) != 7:
            raise ValueError("joint_ids must contain exactly 7 ids")

        self.joint_signs = np.array(joint_signs if joint_signs is not None else [1.0] * 7, dtype=float)
        self.joint_offsets = np.array(joint_offsets if joint_offsets is not None else [0.0] * 7, dtype=float)
        if self.joint_signs.shape != (7,) or self.joint_offsets.shape != (7,):
            raise ValueError("joint_signs and joint_offsets must contain exactly 7 values")

        self.active_mask = np.ones(7, dtype=float)
        if active_joints is not None:
            self.active_mask = np.zeros(7, dtype=float)
            for joint_index in active_joints:
                if joint_index < 1 or joint_index > 7:
                    raise ValueError("active_joints values must be in 1..7")
                self.active_mask[joint_index - 1] = 1.0

        self.joint_action_gain = joint_action_gain
        self.max_target_delta = max_target_delta
        if gripper_mode not in ["continuous", "calibrated"]:
            raise ValueError("gripper_mode must be 'continuous' or 'calibrated'")
        self.gripper_mode = gripper_mode
        self.gripper_action_gain = gripper_action_gain
        self.gripper_master_open = gripper_master_open
        self.gripper_master_closed = gripper_master_closed
        self.gripper_open_command = gripper_open_command
        self.gripper_closed_command = gripper_closed_command
        self.reader = DynamixelReader(
            port=port,
            ids=self.ids,
            baudrate=baudrate,
            frequency=frequency,
            sample_dir=sample_dir,
        )
        self.reader.start()

        self._key_lock = threading.Lock()
        self._keys = {
            "movement_enabled": True,
            "success": False,
            "failure": False,
        }
        self.reset_state()

    def reset_state(self):
        self.reset_origin = True
        self.master_joint_origin = None
        self.robot_joint_origin = None
        self.master_gripper_origin = None
        self.robot_gripper_origin = None
        self.last_gripper_command = None
        with self._key_lock:
            self._keys["movement_enabled"] = True
            self._keys["success"] = False
            self._keys["failure"] = False

    def handle_key_press(self, event):
        key = getattr(event, "keysym", "").lower()
        with self._key_lock:
            if key == "space":
                self.reset_origin = True
            elif key in ["a", "s"]:
                self._keys["success"] = True
            elif key in ["b", "f"]:
                self._keys["failure"] = True

    def handle_key_release(self, event):
        key = getattr(event, "keysym", "").lower()
        with self._key_lock:
            if key in ["a", "s"]:
                self._keys["success"] = False
            elif key in ["b", "f"]:
                self._keys["failure"] = False

    def get_info(self):
        status = self.reader.get_status()
        with self._key_lock:
            info = {
                "success": self._keys["success"],
                "failure": self._keys["failure"],
                "movement_enabled": self._keys["movement_enabled"],
                "controller_on": status["connected"],
            }
        return info

    def get_metadata(self):
        return {
            "master_ids": self.ids,
            "master_joint_ids": self.joint_ids,
            "master_gripper_id": -1 if self.gripper_id is None else self.gripper_id,
            "master_joint_signs": self.joint_signs.tolist(),
            "master_joint_offsets": self.joint_offsets.tolist(),
            "master_active_mask": self.active_mask.tolist(),
            "master_joint_gain": self.joint_action_gain,
            "master_max_target_delta": self.max_target_delta,
            "master_gripper_mode": self.gripper_mode,
            "master_gripper_open": -1.0 if self.gripper_master_open is None else self.gripper_master_open,
            "master_gripper_closed": -1.0 if self.gripper_master_closed is None else self.gripper_master_closed,
            "master_gripper_open_command": self.gripper_open_command,
            "master_gripper_closed_command": self.gripper_closed_command,
        }

    def _read_master_joints(self):
        positions = self.reader.get_positions()
        if any(dxl_id not in positions for dxl_id in self.joint_ids):
            return None
        raw_joints = np.array([positions[dxl_id] for dxl_id in self.joint_ids], dtype=float)
        return self.joint_signs * raw_joints + self.joint_offsets

    def _read_master_gripper(self):
        if self.gripper_id is None:
            return None
        positions = self.reader.get_positions()
        if self.gripper_id not in positions:
            return None
        return positions[self.gripper_id]

    def forward(self, obs_dict, include_info=False):
        action = np.zeros(8)
        info_dict = {}

        controller_info = self.get_info()
        master_joints = self._read_master_joints()
        if master_joints is None or not controller_info["movement_enabled"]:
            if include_info:
                return action, info_dict
            return action

        robot_state = obs_dict["robot_state"]
        robot_joints = np.array(robot_state["joint_positions"], dtype=float)
        robot_gripper = float(robot_state["gripper_position"])

        if self.reset_origin:
            self.master_joint_origin = master_joints
            self.robot_joint_origin = robot_joints
            self.master_gripper_origin = self._read_master_gripper()
            self.robot_gripper_origin = robot_gripper
            self.last_gripper_command = robot_gripper
            self.reset_origin = False

        master_joint_offset = master_joints - self.master_joint_origin
        robot_joint_offset = robot_joints - self.robot_joint_origin
        target_joint_offset = np.clip(
            master_joint_offset * self.joint_action_gain,
            -self.max_target_delta,
            self.max_target_delta,
        )
        target_joint_offset = target_joint_offset * self.active_mask
        target_joints = self.robot_joint_origin + target_joint_offset
        action[:7] = target_joints

        master_gripper = self._read_master_gripper()
        target_gripper = robot_gripper
        if master_gripper is not None and self.master_gripper_origin is not None:
            if self.gripper_mode == "calibrated":
                if self.gripper_master_open is None or self.gripper_master_closed is None:
                    raise RuntimeError("calibrated gripper mode requires open/closed master endpoints")
                span = self.gripper_master_closed - self.gripper_master_open
                if abs(span) < 1e-6:
                    raise RuntimeError("gripper calibration span is too small")
                closed_ratio = np.clip((master_gripper - self.gripper_master_open) / span, 0.0, 1.0)
                target_gripper = float(
                    self.gripper_open_command
                    + closed_ratio * (self.gripper_closed_command - self.gripper_open_command)
                )
            else:
                master_gripper_offset = master_gripper - self.master_gripper_origin
                target_gripper = self.robot_gripper_origin + master_gripper_offset * self.gripper_action_gain
                target_gripper = float(np.clip(target_gripper, 0.0, 1.0))

        action[7] = target_gripper
        self.last_gripper_command = target_gripper

        info_dict = {
            "target_joint_position": target_joints.tolist(),
            "target_gripper_position": target_gripper,
            "master_joint_positions": master_joints.tolist(),
            "master_joint_offset": master_joint_offset.tolist(),
            "robot_joint_offset": robot_joint_offset.tolist(),
        }
        if master_gripper is not None:
            info_dict["master_gripper_position"] = master_gripper
        if include_info:
            return action, info_dict
        return action
