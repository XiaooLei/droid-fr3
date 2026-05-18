# DROID FR3 Setup & Operations

本文档面向接手这套 DROID 机器人系统的人，覆盖从零开始的硬件组装、软件配置、数据采集、VLA 评测全流程。

---

## 目录

1. [硬件组装](#1-硬件组装)
2. [网络配置](#2-网络配置)
3. [软件环境](#3-软件环境)
4. [快速验证](#4-快速验证)
5. [数据采集](#5-数据采集)
6. [相机标定](#6-相机标定)
7. [VLA 评测（pi0_fast_droid）](#7-vla-评测pi0_fast_droid)
8. [主从臂数据采集模式](#8-主从臂数据采集模式)
9. [踩坑记录](#9-踩坑记录)

---

## 1. 硬件组装

### 整体方案

四脚升降桌 + 铝型材固定 + 海洋板桌面 + 网格板理线

### 采购清单参考

| 配置项 | 最低要求 | 推荐 |
|--------|----------|------|
| GPU | ≥ GTX 1650 | ≥ RTX 2060 |
| CPU | ≥ 4 核 | ≥ 6 核 |
| RAM | ≥ 16 GB | ≥ 32 GB |
| SSD | ≥ 512 GB | ≥ 1 TB |
| 接口 | 3× USB-A 3.x + USB-C + RJ-45 | 同左，不能用拓展坞 |

> PC2（笔记本）的 GPU 需满足 ZED SDK 要求：compute capability ≥ 7.5（GTX 1650 / RTX 2060 以上）。USB 端口必须直连，拓展坞会导致 ZED 相机不稳定。

---

## 2. 网络配置

### IP 分配

| 节点 | IP | 角色 |
|------|----|------|
| 路由器 | 172.16.0.250 | 局域网网关 |
| NUC（PC1） | 172.16.0.2 | 机械臂控制服务 |
| 笔记本（PC2） | 172.16.0.1 | 相机 + 控制器 + GUI |
| FR3 控制箱 | 172.16.0.3 | 机械臂本体 |

子网掩码均为 `255.255.255.0`。

### FR3 Desk 验证

浏览器访问 `http://172.16.0.3`，登录后确认机械臂处于就绪状态（指示灯绿色、brakes 已解除）。

---

## 3. 软件环境

### 已配置内容

NUC 和 PC2 上的环境已经配置完毕，无需 Docker，直接在宿主机 conda 环境中运行。

### 软件栈

| 组件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 22.04 | 宿主机 |
| conda env | `droid` | 两台机器均已配置 |
| Franky SDK | 最新 | 替代 polymetis，控制 FR3 |
| libfranka | 0.15.2 | 兼容 FR3 Server 5.8.0 |
| ZED SDK | 4.x | 相机驱动 |
| zerorpc | — | NUC ↔ PC2 通信 |

### 系统级依赖（conda 之外需单独安装）

**NUC：**

| 依赖 | 版本 | 安装方式 |
|------|------|----------|
| franky-control | 1.1.3 | `pip install franky-control==1.1.3` |
| pyrobotiqgripper | 3.2.6 | `pip install pyrobotiqgripper==3.2.6` |

**PC2：**

| 依赖 | 版本 | 安装方式 |
|------|------|----------|
| CUDA | 13.0 | [NVIDIA 官方安装包](https://developer.nvidia.com/cuda-downloads) |
| ZED SDK | 4.x（含 pyzed 5.1） | [ZED SDK 官方安装包](https://www.stereolabs.com/docs/development/zed-sdk/linux) |
| oculus_reader | 1.0.0 | 仓库内 `droid/oculus_reader` 目录，`pip install -e droid/oculus_reader` |

> ZED SDK 必须在安装 pyzed 之前装好，否则 `import pyzed` 会失败。

### 重建 conda 环境

系统级依赖安装完成后，用仓库根目录的环境文件重建 conda 环境：

```bash
# NUC 上
conda env create -f environment_nuc.yml
conda activate droid

# PC2 上
conda env create -f environment_pc2.yml
conda activate droid
```

> 这两份环境是当前已验证可用的版本快照，建议严格按照此文件安装，避免依赖版本不兼容。

---

## 4. 快速验证

每次开机后按以下顺序检查系统是否正常。

### Step 1：启动 NUC 服务

```bash
ssh robotiq@172.16.0.2
cd ~/dev/droid
conda activate droid
PYTHONPATH=. python3 scripts/server/run_server.py
```

启动成功输出：

```
[SERVER] Starting DROID robot server...
Franky robot initialized: IP=172.16.0.3
[SERVER] Listening on tcp://0.0.0.0:4242
```

保持此终端运行。

### Step 2：PC2 验证连接

```bash
ssh fnlp@172.16.0.1
cd ~/dev/droid
conda activate droid
python3 -c "
import zerorpc

c = zerorpc.Client(heartbeat=20)
c.connect('tcp://172.16.0.2:4242')
state, _ = c.get_robot_state()
print('关节角度:', state['joint_positions'])
"
```

输出关节角度说明 NUC ↔ PC2 ↔ FR3 全链路正常。

### 查看 Droid 当前末端位姿

如果只想查看 Droid/FR3 机械臂当前末端位姿，在 PC2 上运行：

```bash
ssh fnlp@172.16.0.1
cd ~/dev/droid
conda activate droid
python3 -c "
import zerorpc

c = zerorpc.Client(heartbeat=20)
c.connect('tcp://172.16.0.2:4242')
state, timestamp = c.get_robot_state()

pose = state['cartesian_position']
print('末端位姿 [x, y, z, roll, pitch, yaw]:', pose)
print('位置 xyz (m):', pose[:3])
print('姿态 rpy (rad):', pose[3:])
print('关节角度 (rad):', state['joint_positions'])
print('机器人时间戳:', timestamp)
"
```

也可以直接读取末端位姿：

```bash
python3 -c "
import zerorpc

c = zerorpc.Client(heartbeat=20)
c.connect('tcp://172.16.0.2:4242')
print(c.get_ee_pose())
"
```

`cartesian_position` / `get_ee_pose()` 的格式均为 `[x, y, z, roll, pitch, yaw]`，其中位置单位是米，姿态单位是弧度，坐标系为机器人 base 坐标系。

---

## 5. 数据采集

### 启动采集 GUI

确认 NUC 服务（Step 1）已启动，然后在 PC2 上连接好以下设备：
- 3 个 ZED 相机（USB-A 3.x 直连，不能用拓展坞）
- VR 模式：Quest3 控制器（USB-C 直连，需开启 Quest Link 或 Air Link）
- 主从臂模式：Dynamixel 主臂串口

VR / Oculus 模式：

```bash
cd ~/dev/droid
conda activate droid
python3 scripts/main.py
```

主从臂 / Dynamixel 模式：

```bash
cd ~/dev/droid
conda activate droid
python3 scripts/main.py --master
```

GUI 启动后确认 3 个 ZED 相机和对应控制器均已连接。

### 控制器操作

| 按键 | 功能 |
|------|------|
| A | 开始录制 / 标记成功 |
| B | 取消 / 标记失败 |
| 右摇杆 | 控制末端执行器移动 |
| 右扳机 | 控制夹爪开合 |

主从臂模式：

| 按键 / 动作 | 功能 |
|-------------|------|
| 移动主臂 | 直接控制从臂 |
| Space | 用当前主臂和当前从臂位姿重新对齐 |
| A | 开始录制 / 标记成功 |
| B | 取消 / 标记失败 |

### 采集流程

1. GUI 启动后机械臂自动复位到初始位置
2. 点击 **PRACTICE** 先熟悉操控
3. 点击 **Collect** 开始正式采集
4. 操控机械臂完成任务后按 A（成功）或 B（失败）
5. 数据保存在 PC2 的 `~/dev/droid/data/success/` 或 `~/dev/droid/data/failure/`

### 回放轨迹

```bash
cd ~/dev/droid
conda activate droid
python3 - <<'PY'
from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import replay_trajectory

trajectory_folderpath = "/home/fnlp/dev/droid/data/success/2026-05-07/Thu_May__7_23:28:36_2026"
env = RobotEnv(action_space="joint_position", gripper_action_space="position")
replay_trajectory(env, filepath=trajectory_folderpath + "/trajectory.h5")
PY
```

---

## 6. 相机标定

> 标定结果保存在 `droid/calibration/calibration_info.json`。重新安装相机后需重新标定。

### 标定板规格

使用与代码匹配的 ChArUco 标定板：

- 11 列 × 8 行
- 黑色方块：24.8 mm，ArUco 标记：14.88 mm
- 字典：DICT_4X4_100
- 打印时**不要缩放**，保持比例正确

### 标定流程

标定通过脚本触发，每个相机分别标定。先修改 `scripts/tests/calibrate_cameras.py` 中的 `camera_id` 为目标相机序列号，然后运行：

```bash
cd ~/dev/droid
conda activate droid
PYTHONPATH=. python3 scripts/tests/calibrate_cameras.py
```

**腕部相机（19006932）：**
1. 将 `camera_id` 改为 `"19006932"`，运行脚本
2. 手持标定板放在相机可见位置
3. 按控制器 A 开始自动采集（机械臂做圆周运动，约 1 分钟）
4. 自动计算并保存标定结果

**第三方相机（37322041 / 37818728）：**
1. 将 `camera_id` 改为对应序列号，运行脚本
2. 将标定板固定放置在相机视野内
3. 按控制器 A 进入调整阶段，移动机械臂直到标定板出现在相机画面中
4. 再次按 A 开始自动轨迹采集
5. 自动计算并保存

标定成功后终端输出：

```
[Calibration] OVERALL: SUCCESS
```

查看当前标定状态：

```bash
cat ~/dev/droid/droid/calibration/calibration_info.json | python3 -m json.tool
```

---

## 7. VLA 评测（pi0_fast_droid）

本节面向算法开发者，说明如何在已配置好的 FR3 上跑 VLA 模型评测。

### 整体架构

```
GPU 服务器                        PC2 (172.16.0.1)
─────────────────                 ──────────────────────
openpi policy server   WebSocket  run_policy_client.py
(pi0_fast_droid)      ────────►  ↓
WebSocket :8001                   NUC (172.16.0.2) → FR3
```

### Step 1：GPU 服务器启动 policy server

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
Serving on 0.0.0.0:8001
```

确保 PC2 能访问 GPU 服务器的 8001 端口。

### Step 2：PC2 启动评测 client

确认 NUC 服务已运行，然后：

```bash
cd ~/dev/droid
conda activate droid
PYTHONPATH=. python3 scripts/server/run_policy_client.py \
    --server-host <GPU_SERVER_IP> \
    --server-port 8001 \
    --instruction "pick up the banana" \
    --max-steps 300 \
    --exec-horizon 5 \
    --show-obs
```

`--show-obs` 会弹出实时观测窗口（左：wrist 图像，中：exterior 图像，右：关节状态），方便验证图像是否正确传递。

### 评测参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--server-host` | 必填 | GPU 服务器 IP |
| `--server-port` | `8000` | policy server 端口 |
| `--instruction` | `"pick up the object"` | 任务语言指令 |
| `--max-steps` | `300` | 每 episode 最大步数（300 步 ≈ 20 秒） |
| `--exec-horizon` | `10` | 每次 query 后执行的 action 步数（建议 3–10） |
| `--episodes` | `1` | 连续运行的 episode 数量 |
| `--show-obs` | 关闭 | 开启实时观测可视化窗口 |

### Observation / Action 规格

**PC2 发给 policy server 的 obs：**

| 字段 | 说明 | 格式 |
|------|------|------|
| `observation/exterior_image_1_left` | 左侧第三方相机（37322041） | uint8 RGB 224×224 |
| `observation/wrist_image_left` | 腕部相机（19006932） | uint8 RGB 224×224 |
| `observation/joint_position` | 7 关节角度（rad） | float32 (7,) |
| `observation/gripper_position` | 夹爪开合（0=开，1=闭） | float32 (1,) |
| `prompt` | 任务语言指令 | str |

**policy server 返回的 action：**

`actions`：shape **(10, 8)**，每步 = 7 关节速度 + 1 夹爪，以 15 Hz 执行。

---

## 8. 主从臂数据采集模式

当前 GUI 保留两种采集入口：

```bash
# VR / Oculus，默认模式
python3 scripts/main.py --control-source vr --right_controller

# 3D 打印主臂 / Dynamixel，PC2 当前实测参数
python3 scripts/main.py --master
```

`--master` 会使用 PC2 当前默认串口 `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6023466-if00`，并自动从 repo 根目录或 `/tmp/dynamixel_sample_test` 寻找 Dynamixel SDK。只有换串口、换主臂、重新标定 gripper 或调 gain/限幅时，才需要手动传下面这些高级参数。

主臂模式使用 `joint_position` action space：7 个 Franka joint target + 1 个 gripper position。当前实测映射：

```text
joint ids:      1 2 3 4 5 6 7
joint signs:    1 -1 1 1 1 -1 1
joint gain:     0.90
target limit:   2.00 rad
gripper id:     8
gripper open:   master 3.446855 -> Franka 0.0
gripper closed: master 2.523398 -> Franka 1.0
```

GUI 里主臂模式的操作方式：

```text
进入 practice/trajectory 后：直接用主臂控制从臂
Space：用当前主臂和当前从臂位姿重新对齐
A：标记成功
B：标记失败
Return 连按：进入 robot reset 页面
```

数据仍然写原来的 DROID HDF5 trajectory。metadata 会额外记录 `control_source=master`、master joint IDs、signs、gain、限幅、gripper 标定值；每个 action timestep 里会额外包含 `master_joint_positions`、`master_joint_offset`、`target_joint_position`、`target_gripper_position`。

### 数据格式检查

新采集一条 trajectory 后，可以在 PC2 上快速确认数据是否完整：

```bash
cd ~/dev/droid
conda activate droid
python3 - <<'PY'
import h5py

path = "data/success/2026-05-18/Mon_May_18_17:25:39_2026/trajectory.h5"
with h5py.File(path, "r") as f:
    print("success:", f.attrs.get("success"))
    print("control_source:", f.attrs.get("control_source"))
    for key in [
        "action/joint_position",
        "action/gripper_position",
        "action/target_joint_position",
        "action/target_gripper_position",
        "observation/robot_state/joint_positions",
        "observation/robot_state/cartesian_position",
        "observation/robot_state/gripper_position",
    ]:
        print(key, f[key].shape)
PY
```

主臂模式下标准输出应包含：

```text
control_source: master
action/joint_position                  (T, 7)
action/gripper_position                (T,)
observation/robot_state/joint_positions (T, 7)
observation/robot_state/cartesian_position (T, 6)
```

图像数据使用 DROID 原来的 recording 方式保存到 `recordings/SVO/*.svo2`；HDF5 中保存 robot state、controller info、camera intrinsics/extrinsics 和 timestamps。

### 主臂 SDK 依赖

主臂读取依赖 Dynamixel SDK。当前代码会按顺序寻找：

1. `--master-sample-dir` 指定的目录
2. repo 根目录下的 `dynamixel_sample_test/sdk/src`
3. `/tmp/dynamixel_sample_test/sdk/src`

临时测试包 `dynamixel_sample_test/` 不提交到 git。PC2 上已验证的路径是 `/tmp/dynamixel_sample_test`。

### 已验证结果

- practice 模式：主臂可直接控制从臂
- collect 模式：可正常写入 `success/` trajectory
- replay：`RobotEnv(action_space="joint_position", gripper_action_space="position")` 可回放主臂采集轨迹
- 最新验证数据：`action/joint_position` 已修正为 `(T, 7)`，gripper 独立写入 `action/gripper_position`

---

## 9. 踩坑记录

### 问题 1：polymetis SIGKILL 崩溃

**现象**：在 FR3 Server 9 上运行 polymetis 时频繁 SIGKILL，控制系统不稳定。

**根本原因**：polymetis 多年未更新，无法适配新版 Franka Server。

**解决方案**：完全绕过 polymetis，改用 [Franky SDK](https://github.com/TimSchneider42/franky) 直接控制 FR3。Franky 通过 TCP/IP 通信，稳定性更好。

---

### 问题 2：libfranka 版本不兼容

**版本对应关系：**

| libfranka | FR3 Server | 状态 |
|-----------|------------|------|
| 0.15.2 | 5.8.0 | 兼容 ✓ |
| 0.21.1 | 5.9.2 | 兼容 ✓（当前） |
| 0.18+ | — | 不兼容 ✗ |

**解决方案**：使用 Franky SDK，绕过 libfranka 直接控制，无需关心版本。

---

### 问题 3：FR3 Server 5.9.2 报错 `GetRobotModel Command Returns Incomplete Data`

**解决方案**：在 FR3 Desk 界面把所有 safety rule 全部移除。

---

### 问题 4：夹爪控制不可用

**现象**：启动时提示夹爪初始化失败，程序仍继续运行。

**原因**：`/dev/ttyUSB0` 串口未连接或权限不足。

**检查方法**：
```bash
ls -la /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

---

### 问题 5：ZED 相机报 `POTENTIAL CALIBRATION ISSUE`

**现象**：启动时某台 ZED 相机 pretest 失败，程序报错退出。

**原因**：工厂标定文件损坏，或 USB 接触不良。

**解决方案**：重新拔插 USB，若问题持续用 ZED Explorer 工具诊断相机状态。

> 注：`Self-calibration skipped. Scene may be occluded or lack texture.` 是 WARNING，不影响使用，可忽略。

---

## References

- [DROID 官方文档](https://droid-dataset.github.io)
- [Franky SDK](https://github.com/TimSchneider42/franky)
- [Franka FCI 文档](https://frankaemika.github.io/docs/)
- [ZED SDK 安装指南](https://www.stereolabs.com/docs/development/zed-sdk/linux)
- [openpi（pi0）](https://github.com/Physical-Intelligence/openpi)
- 本仓库适配记录：[droid-fr3](https://github.com/XiaooLei/droid-fr3)
