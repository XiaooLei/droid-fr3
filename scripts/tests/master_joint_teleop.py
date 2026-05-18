#!/usr/bin/env python3

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import zerorpc


PROTOCOL_VERSION = 2.0
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
PULSE_PER_REVOLUTION = 4096
RAD_PER_PULSE = (2.0 * math.pi) / PULSE_PER_REVOLUTION


def add_dynamixel_sdk_path(sample_dir):
    sdk_src = Path(sample_dir).expanduser().resolve() / "sdk" / "src"
    if str(sdk_src) not in sys.path:
        sys.path.insert(0, str(sdk_src))


def uint32_to_int32(value):
    value &= 0xFFFFFFFF
    if value & 0x80000000:
        return value - 0x100000000
    return value


def position_to_radian(raw_position):
    return uint32_to_int32(raw_position) * RAD_PER_PULSE


class DynamixelSyncReader:
    def __init__(self, port, ids, baudrate, sample_dir):
        add_dynamixel_sdk_path(sample_dir)
        from dynamixel_sdk import GroupSyncRead, PacketHandler, PortHandler

        self.ids = list(ids)
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(f"打开串口失败: {port}")
        if not self.port_handler.setBaudRate(baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"设置波特率失败: {baudrate}")

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        for dxl_id in self.ids:
            if not self.group_sync_read.addParam(dxl_id):
                self.close()
                raise RuntimeError(f"[ID:{dxl_id:03d}] groupSyncRead addParam 失败")

    def read(self):
        from dynamixel_sdk import COMM_SUCCESS

        dxl_comm_result = self.group_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(self.packet_handler.getTxRxResult(dxl_comm_result))

        positions = {}
        for dxl_id in self.ids:
            if not self.group_sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"[ID:{dxl_id:03d}] groupSyncRead 数据不可用")
            raw_position = self.group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            positions[dxl_id] = position_to_radian(raw_position)
        return positions

    def close(self):
        if hasattr(self, "group_sync_read"):
            self.group_sync_read.clearParam()
        if hasattr(self, "port_handler"):
            self.port_handler.closePort()


def parse_args():
    parser = argparse.ArgumentParser(description="Low-speed joint-space master arm teleop test.")
    parser.add_argument("--nuc-ip", default="172.16.0.2")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--frequency", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--sample-dir", default="/tmp/dynamixel_sample_test")
    parser.add_argument("--ids", type=int, nargs="+", default=list(range(1, 9)))
    parser.add_argument("--joint-ids", type=int, nargs=7, default=list(range(1, 8)))
    parser.add_argument("--joint-signs", type=float, nargs=7, default=[1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0])
    parser.add_argument("--gripper-id", type=int, default=8, help="DYNAMIXEL ID used to control the gripper.")
    parser.add_argument("--gripper-sign", type=float, default=1.0)
    parser.add_argument("--gripper-gain", type=float, default=0.5)
    parser.add_argument("--max-gripper-delta", type=float, default=1.0)
    parser.add_argument("--gripper-mode", choices=["continuous", "binary", "calibrated"], default="calibrated")
    parser.add_argument("--gripper-threshold", type=float, default=0.01)
    parser.add_argument("--gripper-open-command", type=float, default=0.0)
    parser.add_argument("--gripper-closed-command", type=float, default=1.0)
    parser.add_argument("--gripper-command-deadband", type=float, default=0.02)
    parser.add_argument("--gripper-master-open", type=float, default=3.446855)
    parser.add_argument("--gripper-master-closed", type=float, default=2.523398)
    parser.add_argument(
        "--active-joints",
        type=int,
        nargs="+",
        default=list(range(1, 8)),
        help="1-based Franka joints allowed to move; other joint actions are forced to zero.",
    )
    parser.add_argument("--gain", type=float, default=0.9)
    parser.add_argument("--max-action", type=float, default=0.08)
    parser.add_argument("--position-mode", action="store_true", help="Send joint position targets instead of velocity actions.")
    parser.add_argument("--max-target-delta", type=float, default=2.0, help="Max target offset from robot origin in radians.")
    parser.add_argument("--enable-control", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    period_s = 1.0 / args.frequency
    active_mask = np.zeros(7, dtype=float)
    for joint_index in args.active_joints:
        if joint_index < 1 or joint_index > 7:
            raise ValueError("--active-joints values must be in 1..7")
        active_mask[joint_index - 1] = 1.0

    master_reader = DynamixelSyncReader(
        port=args.port,
        ids=args.ids,
        baudrate=args.baudrate,
        sample_dir=args.sample_dir,
    )
    robot = zerorpc.Client(heartbeat=20, timeout=10)
    robot.connect(f"tcp://{args.nuc_ip}:4242")

    try:
        master_positions = master_reader.read()
        master_origin = np.array([master_positions[dxl_id] for dxl_id in args.joint_ids], dtype=float)
        master_origin = master_origin * np.array(args.joint_signs, dtype=float)
        master_gripper_origin = None
        robot_gripper_origin = None
        if args.gripper_id is not None:
            if args.gripper_id not in master_positions:
                raise RuntimeError(f"--gripper-id {args.gripper_id} was not read; include it in --ids")
            master_gripper_origin = master_positions[args.gripper_id] * args.gripper_sign
            robot_gripper_origin = float(robot.get_gripper_position())

        robot_origin = np.array(robot.get_joint_positions(), dtype=float)

        print("master_origin:", master_origin.tolist(), flush=True)
        print("robot_origin:", robot_origin.tolist(), flush=True)
        if args.gripper_id is not None:
            print(
                f"gripper_origin: master={master_gripper_origin:.4f}, robot={robot_gripper_origin:.4f}",
                flush=True,
            )
        print(
            f"mode: {'CONTROL' if args.enable_control else 'DRY-RUN'}, "
            f"frequency={args.frequency:g}Hz, gain={args.gain:g}, max_action={args.max_action:g}",
            flush=True,
        )

        start = time.monotonic()
        next_tick = start
        last_gripper_command = None
        gripper_closed = False
        while time.monotonic() - start < args.duration:
            master_positions = master_reader.read()
            master_joints = np.array([master_positions[dxl_id] for dxl_id in args.joint_ids], dtype=float)
            master_joints = master_joints * np.array(args.joint_signs, dtype=float)
            robot_joints = np.array(robot.get_joint_positions(), dtype=float)

            master_delta = master_joints - master_origin
            robot_delta = robot_joints - robot_origin
            target_delta = np.clip(master_delta * args.gain, -args.max_target_delta, args.max_target_delta)
            target_delta = target_delta * active_mask
            target_joints = robot_origin + target_delta
            action = np.clip((target_delta - robot_delta) * args.gain, -args.max_action, args.max_action)
            action = action * active_mask
            target_gripper = None
            master_gripper_delta = None
            if args.gripper_id is not None:
                master_gripper = master_positions[args.gripper_id] * args.gripper_sign
                master_gripper_delta = master_gripper - master_gripper_origin
                if args.gripper_mode == "calibrated":
                    if args.gripper_master_open is None or args.gripper_master_closed is None:
                        raise RuntimeError(
                            "--gripper-mode calibrated requires --gripper-master-open and --gripper-master-closed"
                        )
                    span = args.gripper_master_closed - args.gripper_master_open
                    if abs(span) < 1e-6:
                        raise RuntimeError("gripper master open/closed calibration span is too small")
                    closed_ratio = np.clip((master_gripper - args.gripper_master_open) / span, 0.0, 1.0)
                    target_gripper = float(
                        args.gripper_open_command
                        + closed_ratio * (args.gripper_closed_command - args.gripper_open_command)
                    )
                elif args.gripper_mode == "binary":
                    if master_gripper_delta > args.gripper_threshold:
                        gripper_closed = True
                    elif master_gripper_delta < args.gripper_threshold * 0.5:
                        gripper_closed = False
                    target_gripper = args.gripper_closed_command if gripper_closed else args.gripper_open_command
                else:
                    gripper_delta = np.clip(
                        master_gripper_delta * args.gripper_gain,
                        -args.max_gripper_delta,
                        args.max_gripper_delta,
                    )
                    target_gripper = float(np.clip(robot_gripper_origin + gripper_delta, 0.0, 1.0))

            if args.enable_control:
                if args.position_mode:
                    robot.update_joints(target_joints.tolist(), False, False)
                else:
                    robot.update_joints(action.tolist(), True, False)
                if target_gripper is not None and (
                    last_gripper_command is None
                    or abs(target_gripper - last_gripper_command) >= args.gripper_command_deadband
                ):
                    robot.update_gripper(target_gripper, False, False)
                    last_gripper_command = target_gripper

            log_line = (
                "master_delta=" + np.array2string(master_delta, precision=3, suppress_small=True)
                + " robot_delta=" + np.array2string(robot_delta, precision=3, suppress_small=True)
                + " target_delta=" + np.array2string(target_delta, precision=3, suppress_small=True)
                + " action=" + np.array2string(action, precision=3, suppress_small=True)
            )
            if target_gripper is not None:
                log_line += f" master_gripper_delta={master_gripper_delta:.3f} target_gripper={target_gripper:.3f}"
            print(log_line, flush=True)

            next_tick += period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()
    finally:
        try:
            robot.update_joints([0.0] * 7, True, False)
        except Exception:
            pass
        master_reader.close()


if __name__ == "__main__":
    main()
