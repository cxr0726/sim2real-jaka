# Pico Teleoperation

这个教程使用 teleop publisher 提供实时 Pico / XR retarget，用 realtime viewer 检查 retarget 结果，再用 root project 的 tracking policy 做执行。支持 `g1`（默认）和 `jaka` 机器人。

## 1. 启动 Pico retarget publisher

```bash
# 对于 G1 (默认):
uv --project venv/teleop run python sim2real/teleop/pico_retarget_pub.py \
  --bind tcp://*:28701 \
  --publish_hz 50 \
  --actual_human_height 1.80 \
  --robot g1

# 对于 Jaka:
uv --project venv/teleop run python sim2real/teleop/pico_retarget_pub.py \
  --bind tcp://*:28701 \
  --publish_hz 50 \
  --actual_human_height 1.80 \
  --robot jaka
```

## 2. 用 realtime viewer 检查 retarget

```bash
# 对于 G1 (默认):
uv --project venv/teleop run python sim2real/teleop/realtime_viewer.py \
  --connect tcp://127.0.0.1:28701 \
  --viewer_hz 50 \
  --robot g1

# 对于 Jaka:
uv --project venv/teleop run python sim2real/teleop/realtime_viewer.py \
  --connect tcp://127.0.0.1:28701 \
  --viewer_hz 50 \
  --robot jaka
```

先确认 viewer 里的机器人 retarget 动作是对的，再继续执行。

## 3. 选择执行后端

### Sim2Sim

启动 MuJoCo 执行进程：

```bash
# 对于 G1:
uv run sim2real/sim_env/base_sim.py --robot g1

# 对于 Jaka:
uv run sim2real/sim_env/base_sim.py --robot jaka
```

在另一个终端，把 tracking policy 接到实时 motion stream：

```bash
# 对于 G1:
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy_config checkpoints/lafan-aa/policy-ec592bb4_lafan_100style_student-5000.yaml \
  --motion_backend zmq \
  --motion_zmq_connect tcp://127.0.0.1:28701

# 对于 Jaka:
uv run sim2real/rl_policy/tracking.py \
  --robot jaka \
  --policy_config checkpoints/jaka/latest35knew.yaml \
  --motion_backend zmq \
  --motion_zmq_connect tcp://127.0.0.1:28701
```

### Sim2Real

把 MuJoCo 执行进程换成 real bridge（目前支持 G1）：

```bash
uv run scripts/real_bridge.py
```

然后运行相同的 tracking policy：

```bash
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy_config checkpoints/lafan-aa/policy-ec592bb4_lafan_100style_student-5000.yaml \
  --motion_backend zmq \
  --motion_zmq_connect tcp://127.0.0.1:28701
```

## Notes

- `pico_retarget_pub.py` 发布的实时 motion stream 同时给 realtime viewer 和 tracking policy 使用
- `sim2real/sim_env/base_sim.py` 是 sim2sim 的执行后端
- `scripts/real_bridge.py` 是 sim2real 的执行后端
- 如果希望 policy control mode 跟着 Pico controller topic 走，而不是键盘控制，可以在 tracking 命令里加 `--controller pico`
- 如果 publisher 和 policy 跑在不同机器上，把 `tcp://127.0.0.1:28701` 换成 publisher 所在机器的 IP

## Next Steps

- [Motion Recording](./motion-recording.md)
