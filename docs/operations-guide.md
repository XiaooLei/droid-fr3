# DROID FR3 操作手册

本文覆盖机器人系统的完整操作流程：服务启动、数据采集、相机标定、VLA 评测。

---

## 硬件与网络

| 节点 | IP | 角色 |
|------|----|------|
| NUC | 172.16.0.2 | 机械臂控制（zerorpc 服务） |
| PC2（笔记本） | 172.16.0.1 | 相机 + 控制器 + GUI |
| GPU 服务器 | 自定义 | VLA policy server（评测时使用） |

相机序列号：
- `19006932`：腕部相机
- `37322041`：左侧第三方相机
- `37818728`：右侧第三方相机

---

## 一、启动服务

### 1. NUC：启动机械臂控制服务

SSH 登录 NUC，启动 zerorpc 服务：

```bash
ssh robotiq@172.16.0.2
cd ~/dev/droid
PYTHONPATH=. python3 scripts/server/run_server.py
```

启动成功后会看到：
```
Franky robot initialized: IP=172.16.0.3
```

服务监听 `0.0.0.0:4242`，保持此终端运行。

> **注意**：每次重启前确认 FR3 已上电、解除 brakes、处于就绪状态（指示灯绿色）。

### 2. PC2：确认连接

```bash
ssh fnlp@172.16.0.1
cd ~/dev/droid
PYTHONPATH=. python3 -c "
from droid.misc.server_interface import ServerInterface
r = ServerInterface('172.16.0.2')
state, _ = r.get_robot_state()
print('关节角度:', state['joint_positions'])
"
```

输出关节角度说明连接正常。

---

## 二、数据采集

### 启动采集 GUI

在 PC2 上：

```bash
cd ~/dev/droid
PYTHONPATH=. python3 scripts/main.py --right_controller
# 如果使用左手控制器：
PYTHONPATH=. python3 scripts/main.py --left_controller
```

### GUI 操作说明

| 按键 | 功能 |
|------|------|
| A | 开始 / 确认 |
| B | 取消 / 失败结束 |
| 右摇杆 | 移动机械臂（EEF 速度控制） |
| 右扳机 | 控制夹爪 |

**采集流程**：
1. GUI 启动后机械臂自动复位到初始位置
2. 按 A 开始录制轨迹
3. 操控机械臂完成任务
4. 按 A 标记成功，或按 B 标记失败
5. 数据自动保存到 `droid/data/success/` 或 `droid/data/failure/`

---

## 三、相机标定

> 标定结果保存在 `droid/calibration/calibration_info.json`，每次重新安装相机后需要重新标定。

### 标定板要求

使用与代码匹配的 ChArUco 标定板：
- 11 列 × 8 行，黑色方块 24.8mm，ArUco 标记 14.88mm
- 字典：DICT_4X4_100
- 打印时确保比例正确（不要缩放）

### 标定流程

标定通过 GUI 触发，每个相机分别标定：

**腕部相机（19006932）**：
1. 手持标定板，在 GUI 中选择腕部相机标定
2. 按 A 进入调整阶段：将标定板放在相机可见位置
3. 再次按 A 开始自动采集轨迹（机械臂做正弦运动约 1 分钟）
4. 采集完成后自动计算标定结果

**第三方相机（37322041 / 37818728）**：
1. 将标定板固定放置在相机视野内
2. 在 GUI 中选择对应相机标定
3. 按 A 进入调整阶段：移动机械臂直到标定板出现在相机画面中
4. 再次按 A 开始自动轨迹采集
5. 采集完成后自动计算并保存

标定成功后终端输出：
```
[Calibration] OVERALL: SUCCESS
```

查看当前标定状态：
```bash
cat ~/dev/droid/droid/calibration/calibration_info.json | python3 -m json.tool
```

---

## 四、VLA 评测

### 1. GPU 服务器：启动 policy server

```bash
cd $OPENPI_ROOT

# 使用官方预训练 checkpoint
uv run scripts/serve_policy.py --env=DROID

# 使用自己训练的 checkpoint
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_fast_droid \
    --policy.dir=<CHECKPOINT_DIR>
```

启动成功后输出：
```
Serving on 0.0.0.0:8000
```

### 2. PC2：运行评测 client

确保 NUC 服务已启动，然后：

```bash
cd ~/dev/droid
PYTHONPATH=. python3 scripts/server/run_policy_client.py \
    --server-host <GPU_SERVER_IP> \
    --server-port 8000 \
    --instruction "pick up the apple" \
    --max-steps 300 \
    --episodes 5
```

### 评测参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--server-host` | 必填 | GPU 服务器 IP |
| `--server-port` | `8000` | policy server 端口 |
| `--instruction` | `"pick up the object"` | 任务语言指令 |
| `--max-steps` | `300` | 每 episode 最大步数（300 步 ≈ 20 秒） |
| `--episodes` | `1` | 运行 episode 数量 |

### obs / action 规格

**发给 policy server 的 obs：**

| 字段 | 说明 | 格式 |
|------|------|------|
| `observation/exterior_image_1_left` | 左侧第三方相机 | uint8 RGB 224×224 |
| `observation/wrist_image_left` | 腕部相机 | uint8 RGB 224×224 |
| `observation/joint_position` | 7 关节角度（rad） | float32 (7,) |
| `observation/gripper_position` | 夹爪开合（0=闭，1=开） | float32 (1,) |
| `prompt` | 任务指令 | str |

**policy server 返回的 action：**

`actions`：shape **(10, 8)**，每步 = 7 关节目标位置（rad）+ 1 夹爪位置，以 15 Hz 执行。

---

## 五、常见问题

**机械臂进入 reflex 模式（停止运动）**
: 在 FR3 手持示教器或 Desk 界面解锁，client 会自动继续发指令。

**标定失败（Lin error 超标）**
: 重新标定，调整初始位置使标定板在整个运动轨迹中保持可见；线性误差通常由视角变化不足导致。

**相机未检测到**
: 检查 USB 连接，确认没有其他进程占用相机（`fuser /dev/video*`）。

**zerorpc 连接超时**
: 确认 NUC 服务正在运行，检查防火墙是否放行 4242 端口（`sudo ufw allow 4242`）。
