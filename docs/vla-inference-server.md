## VLA Inference Server

PC2 上运行的 HTTP 服务，供外部 VLA 模型获取机器人观测数据并下发动作指令。

### 架构

```
VLA Service (外部)
    ↕ HTTP (port 8000)
PC2 obs server  (172.16.0.1)
    ↕ zerorpc (port 4242)
NUC robot server (172.16.0.2)
    ↕
FR3 机械臂
```

### 启动

**前提**：NUC 上的机械臂 zerorpc 服务已启动。

```bash
# 安装依赖（首次）
pip install fastapi uvicorn

# 启动服务
cd ~/dev/droid
python scripts/server/run_obs_server.py
```

启动后访问 `http://172.16.0.1:8000/docs` 查看交互式 API 文档。

### 接口

#### GET `/health`

存活检查。

**响应**
```json
{ "status": "ok" }
```

---

#### GET `/observation`

获取当前时刻的完整观测。

**响应字段**

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_state.cartesian_position` | `[float x6]` | EEF 位姿 `[x, y, z, roll, pitch, yaw]`，单位 m/rad |
| `robot_state.gripper_position` | `float` | 夹爪开合，0=闭合，1=全开 |
| `robot_state.joint_positions` | `[float x7]` | 7 关节角度，单位 rad |
| `robot_state.joint_velocities` | `[float x7]` | 7 关节速度，单位 rad/s |
| `images` | `dict[str, str]` | 相机图像，key 为 `{serial}_left` / `{serial}_right`，value 为 JPEG base64 |
| `camera_intrinsics` | `dict[str, [[float]]]` | 各路相机 3×3 内参矩阵 |
| `camera_extrinsics` | `dict[str, [float x6]]` | 各路相机外参，相机到机器人基座的变换 `[x, y, z, roll, pitch, yaw]` |

**相机 key 对应关系**

| Key | 说明 |
|-----|------|
| `19006932_left` / `_right` | 腕部相机（ZED 左/右目） |
| `37322041_left` / `_right` | 左侧第三方相机 |
| `37818728_left` / `_right` | 右侧第三方相机 |

**图像解码示例（Python）**
```python
import base64, cv2, numpy as np

def decode_image(b64_str: str) -> np.ndarray:
    buf = base64.b64decode(b64_str)
    img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 转为 RGB
```

---

#### POST `/step`

执行单步动作，立即返回。控制频率由 VLA 侧决定（建议不超过 15 Hz）。

**请求体**
```json
{ "action": [vx, vy, vz, vrx, vry, vrz, gripper] }
```

**action 格式**

| 索引 | 含义 | 范围 |
|------|------|------|
| 0–5 | EEF 笛卡尔速度 `[vx, vy, vz, vrx, vry, vrz]` | `[-1, 1]`，归一化 |
| 6 | 夹爪目标位置 | `[0, 1]`，0=闭合，1=全开 |

**响应**
```json
{ "success": true }
```

---

#### POST `/step_chunk`

批量执行一组动作，服务端按指定频率依次下发，全部执行完后返回。适合 action chunking 类模型（如 ACT、Diffusion Policy）。

**请求体**
```json
{
  "actions": [
    [vx, vy, vz, vrx, vry, vrz, gripper],
    ...
  ],
  "hz": 15.0
}
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `actions` | `[[float x7]]` | 必填 | 动作序列 |
| `hz` | `float` | `15.0` | 执行频率，建议 ≤ 15 |

**响应**
```json
{ "success": true, "steps_executed": 10 }
```

---

### 使用示例

#### 单步推理循环

```python
import requests
import numpy as np

BASE = "http://172.16.0.1:8000"

while True:
    obs = requests.get(f"{BASE}/observation").json()
    action = vla_model.infer(obs)  # → list of 7 floats
    requests.post(f"{BASE}/step", json={"action": action})
```

#### Action Chunking

```python
obs = requests.get(f"{BASE}/observation").json()
actions = vla_model.infer_chunk(obs)  # → list of N x 7 floats
requests.post(f"{BASE}/step_chunk", json={"actions": actions, "hz": 15.0})
```

### 注意事项

- 图像为 **JPEG 有损压缩**（质量 85），如需无损请联系修改
- `step` 和 `step_chunk` 均为阻塞调用，返回后动作已执行完毕
- 服务启动时 `do_reset=False`，不会自动移动机械臂到初始位置，需要 VLA 侧自行管理初始状态
- 机械臂控制频率上限 **15 Hz**，`step_chunk` 的 `hz` 参数建议不超过此值
