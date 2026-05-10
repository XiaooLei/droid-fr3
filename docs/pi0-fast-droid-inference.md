## pi0_fast_droid 评测教程

本文面向算法开发者，说明如何在已配置好的 FR3 机器人上运行 pi0_fast_droid 模型评测。

---

### 前提条件

以下工作已由机器人侧完成，无需重复操作：

- FR3 机器人 + 3 路 ZED 相机已标定
- NUC 机械臂控制服务已部署（172.16.0.2）
- PC2（172.16.0.1）已安装 `openpi-client` 和所有依赖

---

### 整体架构

```
你的 GPU 服务器                  PC2 (172.16.0.1)
─────────────────               ──────────────────
openpi policy server    ←→      run_policy_client.py
(pi0_fast_droid)                ↓
WebSocket :8000                 NUC → FR3 机械臂
```

---

### 第一步：在 GPU 服务器上启动 policy server

```bash
# 使用官方预训练 checkpoint
uv run scripts/serve_policy.py --env=DROID

# 使用自己训练的 checkpoint
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_fast_droid \
    --policy.dir=<YOUR_CHECKPOINT_DIR>
```

启动成功后会看到：
```
Serving on 0.0.0.0:8000
```

> 确保 PC2 能访问你的 GPU 服务器的 8000 端口。

---

### 第二步：在 PC2 上运行评测脚本

SSH 登录 PC2：
```bash
ssh fnlp@172.16.0.1
```

运行评测：
```bash
cd ~/dev/droid
PYTHONPATH=. python3 scripts/server/run_policy_client.py \
    --server-host <YOUR_GPU_SERVER_IP> \
    --server-port 8000 \
    --instruction "pick up the apple" \
    --max-steps 300 \
    --episodes 5
```

---

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--server-host` | 必填 | GPU 服务器 IP |
| `--server-port` | `8000` | policy server 端口 |
| `--instruction` | `"pick up the object"` | 任务语言指令 |
| `--max-steps` | `300` | 每个 episode 最大步数（300 步 ≈ 20 秒） |
| `--episodes` | `1` | 连续运行的 episode 数量 |

---

### obs / action 规格

**PC2 发给 policy server 的 obs：**

| 字段 | 说明 | 格式 |
|------|------|------|
| `observation/exterior_image_1_left` | 左侧第三方相机 | uint8 RGB 224×224 |
| `observation/wrist_image_left` | 腕部相机 | uint8 RGB 224×224 |
| `observation/joint_position` | 7 关节角度（rad） | float32 (7,) |
| `observation/gripper_position` | 夹爪开合（0=闭，1=开） | float32 (1,) |
| `prompt` | 任务语言指令 | str |

**policy server 返回的 action：**

| 字段 | 说明 |
|------|------|
| `actions` | shape **(10, 8)**：10 步 action chunk，每步 = 7 关节目标位置（rad）+ 1 夹爪位置 |

PC2 以 **15 Hz** 依次执行 10 步，执行完后重新查询 server。

---

### 自定义评测脚本

如果需要在每个 episode 之间做自定义操作（记录结果、重置场景等），可以修改 `scripts/server/run_policy_client.py` 中的 `run_episode()` 函数，或在 `main()` 的 episode 循环里添加逻辑：

```python
for ep in range(args.episodes):
    # 在这里做 episode 前的准备，比如摆放物体
    input("准备好后按 Enter 开始...")

    run_episode(env, policy, args.instruction, args.max_steps)

    # 在这里记录结果
    success = input("本次成功？(y/n): ").strip().lower() == "y"
    print(f"Episode {ep+1}: {'success' if success else 'fail'}")
```

---

### 注意事项

- 每次 episode 开始前请确保机械臂在安全初始位置，可手动引导或用示教器复位
- `max-steps=300` 对应约 20 秒，根据任务难度调整
- policy server 和 PC2 之间的网络延迟会影响控制频率，建议在同一局域网内
- 如遇机械臂进入 reflex 模式（停止运动），解锁后 client 会自动继续发送指令
