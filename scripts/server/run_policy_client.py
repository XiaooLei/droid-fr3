"""
PC2 policy client for pi0_fast_droid inference.

Connects to an openpi WebSocket server, collects observations from the
robot and cameras, sends them to the policy, and executes the returned
action chunk on the FR3 robot.

Usage:
    PYTHONPATH=. python3 scripts/server/run_policy_client.py \
        --server-host <VLA_SERVER_IP> \
        --server-port 8000 \
        --instruction "pick up the apple"
"""

import argparse
import os
import signal
import sys
import time

# Clear proxy env vars so WebSocket can connect directly
for _var in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY"):
    os.environ.pop(_var, None)

import numpy as np

from droid.robot_env import RobotEnv

try:
    from openpi_client import image_tools, websocket_client_policy
except ImportError:
    raise ImportError("openpi-client not installed. Run: pip install -e $OPENPI_ROOT/packages/openpi-client")

# Camera serials
WRIST_CAM_ID = "19006932"
EXTERIOR_CAM_ID = "37322041"   # left third-person camera

# pi0_fast_droid image resolution
IMAGE_SIZE = (224, 224)

# Action chunk size and control frequency
CHUNK_SIZE = 10
CONTROL_HZ = 15.0

_stop_requested = False


def _handle_sigint(sig, frame):
    global _stop_requested
    print("\n[CLIENT] Ctrl+C received, stopping after current step...")
    _stop_requested = True


signal.signal(signal.SIGINT, _handle_sigint)


def _to_rgb(img: np.ndarray, name: str) -> np.ndarray:
    if img is None:
        sys.exit(f"[FATAL] Camera '{name}' image is missing (None). Check camera connection and initialization.")
    if img.shape[2] == 4:
        import cv2
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    result = image_tools.convert_to_uint8(image_tools.resize_with_pad(img, *IMAGE_SIZE))
    if result.max() == 0:
        sys.exit(f"[FATAL] Camera '{name}' image is all black after conversion. "
                 "Check camera connection and calibration.")
    return result


def _show_obs(wrist_img: np.ndarray, exterior_img: np.ndarray, robot_state: dict, step: int):
    import cv2

    # Build text overlay with joint + gripper state
    joints = robot_state["joint_positions"]
    gripper = robot_state["gripper_position"]
    lines = [f"step={step}"] + [f"j{i}={v:.3f}" for i, v in enumerate(joints)] + [f"grip={gripper:.3f}"]

    # Scale up to 336x336 for readability, then add text sidebar
    scale = 336
    wrist_bgr = cv2.cvtColor(cv2.resize(wrist_img, (scale, scale)), cv2.COLOR_RGB2BGR)
    ext_bgr = cv2.cvtColor(cv2.resize(exterior_img, (scale, scale)), cv2.COLOR_RGB2BGR)

    sidebar_w = 160
    sidebar = np.zeros((scale, sidebar_w, 3), dtype=np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(sidebar, line, (6, 20 + i * 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)

    # Label each image
    cv2.putText(wrist_bgr, "wrist", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2)
    cv2.putText(ext_bgr, "exterior", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2)

    frame = np.hstack([wrist_bgr, ext_bgr, sidebar])
    cv2.imshow("VLA Observation", frame)
    cv2.waitKey(1)


def build_obs(env: RobotEnv, instruction: str, show: bool = False,
              debug_save_dir: str = None, step: int = 0) -> dict:
    obs = env.get_observation()
    images = obs.get("image", {})
    robot_state = obs["robot_state"]

    raw_wrist = images.get(f"{WRIST_CAM_ID}_left")
    raw_exterior = images.get(f"{EXTERIOR_CAM_ID}_left")

    # Log image availability and shape
    wrist_info = f"shape={raw_wrist.shape} dtype={raw_wrist.dtype}" if raw_wrist is not None else "MISSING"
    ext_info = f"shape={raw_exterior.shape} dtype={raw_exterior.dtype}" if raw_exterior is not None else "MISSING"
    print(f"[OBS] wrist({WRIST_CAM_ID}): {wrist_info}")
    print(f"[OBS] exterior({EXTERIOR_CAM_ID}): {ext_info}")
    print(f"[OBS] available image keys: {list(images.keys())}")

    wrist_img = _to_rgb(raw_wrist, f"wrist/{WRIST_CAM_ID}")
    exterior_img = _to_rgb(raw_exterior, f"exterior/{EXTERIOR_CAM_ID}")

    if show:
        _show_obs(wrist_img, exterior_img, robot_state, step)

    # Save first frame to disk for visual verification
    if debug_save_dir and step == 0:
        import cv2, os
        os.makedirs(debug_save_dir, exist_ok=True)
        cv2.imwrite(os.path.join(debug_save_dir, "wrist.png"), cv2.cvtColor(wrist_img, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(debug_save_dir, "exterior.png"), cv2.cvtColor(exterior_img, cv2.COLOR_RGB2BGR))
        print(f"[OBS] saved debug images to {debug_save_dir}/")

    return {
        "observation/exterior_image_1_left": exterior_img,
        "observation/wrist_image_left": wrist_img,
        "observation/joint_position": np.array(robot_state["joint_positions"], dtype=np.float32),
        "observation/gripper_position": np.array([robot_state["gripper_position"]], dtype=np.float32),
        "prompt": instruction,
    }


def run_episode(env: RobotEnv, policy, instruction: str, max_steps: int, exec_horizon: int,
                show: bool = False):
    global _stop_requested
    period = 1.0 / CONTROL_HZ
    step = 0

    print(f"[CLIENT] Starting episode. instruction='{instruction}', max_steps={max_steps}, exec_horizon={exec_horizon}")
    print("[CLIENT] Press Ctrl+C to stop.")

    while step < max_steps and not _stop_requested:
        # Query policy with current observation
        obs = build_obs(env, instruction, show=show, debug_save_dir="/tmp/droid_obs_debug", step=step)
        result = policy.infer(obs)
        actions = result["actions"]  # (CHUNK_SIZE, 8): 7 joint velocities + 1 gripper

        actions_arr = np.array(actions)
        print(f"[CLIENT] step={step}, received {len(actions)} actions | "
              f"shape={actions_arr.shape} min={actions_arr.min():.4f} max={actions_arr.max():.4f}")

        # Execute only the first exec_horizon actions, then re-query with fresh observation
        for action in actions[:exec_horizon]:
            if step >= max_steps or _stop_requested:
                break
            t0 = time.time()
            action_arr = np.clip(np.array(action, dtype=np.float64), -1.0, 1.0)
            print(f"[CLIENT]   exec step={step} | joints={np.round(action_arr[:7], 4).tolist()} gripper={action_arr[7]:.4f}")
            env.step(action_arr)
            elapsed = time.time() - t0
            sleep_left = period - elapsed
            if sleep_left > 0:
                time.sleep(sleep_left)
            step += 1

    if _stop_requested:
        print(f"[CLIENT] Stopped by user at step={step}")
    else:
        print(f"[CLIENT] Episode complete. steps={step}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-host", required=True, help="openpi server IP")
    parser.add_argument("--server-port", type=int, default=8000)
    parser.add_argument("--instruction", default="pick up the object")
    parser.add_argument("--max-steps", type=int, default=300, help="max steps per episode")
    parser.add_argument("--exec-horizon", type=int, default=10,
                        help="actions to execute per policy query (1=re-query every step, 10=full chunk)")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--show-obs", action="store_true", help="show live observation window (wrist + exterior + joint state)")
    args = parser.parse_args()

    print(f"[CLIENT] Connecting to openpi server at {args.server_host}:{args.server_port} ...")
    policy = websocket_client_policy.WebsocketClientPolicy(
        host=args.server_host,
        port=args.server_port,
    )
    print("[CLIENT] Connected.")

    print("[CLIENT] Initializing RobotEnv ...")
    # joint_velocity: DoF=8 (7 normalized joint velocities + 1 gripper), matches pi0_fast_droid output
    env = RobotEnv(action_space="joint_velocity", do_reset=False)
    print("[CLIENT] RobotEnv ready.")

    for ep in range(args.episodes):
        print(f"\n[CLIENT] ── Episode {ep + 1}/{args.episodes} ──")
        run_episode(env, policy, args.instruction, args.max_steps, args.exec_horizon, show=args.show_obs)


if __name__ == "__main__":
    main()
