"""
PC2 HTTP observation server for VLA inference.

Exposes:
  GET  /observation           - current robot state + camera images (JPEG base64)
  POST /step                  - execute single cartesian_velocity action
  POST /step_chunk            - execute a sequence of actions at specified hz
  GET  /health                - liveness check

Action format: list of 7 floats [vx, vy, vz, vrx, vry, vrz, gripper]
  - cartesian velocity, normalized to [-1, 1]
  - gripper: 0=open, 1=closed
"""

# Must be first: patch stdlib to use gevent so zerorpc's event loop works
import gevent.monkey
gevent.monkey.patch_all()

import base64
import time

import cv2
import numpy as np
from flask import Flask, jsonify, request

from droid.robot_env import RobotEnv

app = Flask(__name__)
env: RobotEnv = None


def _encode_image(img: np.ndarray, quality: int = 85) -> str:
    if img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode("utf-8")


def _serialize_obs(obs: dict) -> dict:
    images = {cam_id: _encode_image(img) for cam_id, img in obs["image"].items()}

    intrinsics = {
        k: v.tolist() if isinstance(v, np.ndarray) else v
        for k, v in obs.get("camera_intrinsics", {}).items()
    }
    extrinsics = {
        k: v.tolist() if isinstance(v, np.ndarray) else v
        for k, v in obs.get("camera_extrinsics", {}).items()
    }
    robot_state = {
        k: v.tolist() if isinstance(v, np.ndarray) else v
        for k, v in obs["robot_state"].items()
    }

    return {
        "robot_state": robot_state,
        "images": images,
        "camera_intrinsics": intrinsics,
        "camera_extrinsics": extrinsics,
    }


@app.get("/health")
def health():
    return jsonify({"status": "ok"})


@app.get("/observation")
def get_observation():
    obs = env.get_observation()
    print(f"[OBS SERVER] obs keys: {list(obs.keys())}")
    if "image" in obs:
        print(f"[OBS SERVER] image keys: {list(obs['image'].keys())}")
    else:
        print("[OBS SERVER] WARNING: 'image' key missing from obs")
        camera_obs, _ = env.read_cameras()
        print(f"[OBS SERVER] camera_obs keys: {list(camera_obs.keys())}")
        print(f"[OBS SERVER] cameras: {list(env.camera_reader.camera_dict.keys())}")
        for cid, cam in env.camera_reader.camera_dict.items():
            print(f"[OBS SERVER]   cam {cid}: mode={cam.current_mode}, skip={cam.skip_reading}, image={cam.image}")
    return jsonify(_serialize_obs(obs))


@app.post("/step")
def step():
    data = request.get_json()
    action = data.get("action")
    if not action or len(action) != 7:
        return jsonify({"error": f"Expected 7-dim action, got {len(action) if action else 0}"}), 400
    env.step(np.array(action))
    return jsonify({"success": True})


@app.post("/step_chunk")
def step_chunk():
    data = request.get_json()
    actions = data.get("actions")
    hz = data.get("hz", 15.0)

    if not actions:
        return jsonify({"error": "actions list is empty"}), 400
    if any(len(a) != 7 for a in actions):
        return jsonify({"error": "Each action must be 7-dim"}), 400

    period = 1.0 / hz
    for action in actions:
        t0 = time.time()
        env.step(np.array(action))
        elapsed = time.time() - t0
        sleep_left = period - elapsed
        if sleep_left > 0:
            time.sleep(sleep_left)

    return jsonify({"success": True, "steps_executed": len(actions)})


if __name__ == "__main__":
    print("[OBS SERVER] Initializing RobotEnv...")
    env = RobotEnv(action_space="cartesian_velocity", do_reset=False)
    print("[OBS SERVER] Ready.")
    app.run(host="0.0.0.0", port=8000)
