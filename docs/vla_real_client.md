# 实机 VLA 交互 Client 启动指南

本文描述如何在真机（Go2 + X5）上启动远程 StarVLA 交互 client 及配套服务
（腿部 WBC、机械臂 CAN owner、SSH 双跳隧道）。所有参数集中在 YAML，
一条命令启动/停止。

## 1. 配置（YAML）

`configs/vla_eval/real_go2_x5.yaml`：

```yaml
server:
  endpoint: ws://127.0.0.1:10093
  connect_timeout_s: 10.0
  response_timeout_s: 120.0

deploy:
  server_ssh: zju-server
  robot_ssh: robodog
  tmux_session: vla_real_all

task:
  instruction: "Pick up the coke can on box1 and place it on box2."
  episode_id: "real_vla_go2_x5_0001"

real:
  mode: shadow
  enable_live_output: false
  confirm_live_output: ""
  front_topic: /camera/front/image_raw/compressed
  wrist_topic: /camera/wrist/image_raw/compressed
  arm_target_state_topic: /arm/target_state
  base_command_topic: /vla/base_cmd
  arm_command_topic: /vla/arm_target
  odom_topic: /odom
  nav_anchor_mode: body
  odom_timeout_s: 0.5
  image_timeout_s: 0.75
  decision_watchdog_s: 0.75
  response_timeout_s: 120.0

interactive:
  nav_waypoint_tolerance_m: 0.12
  nav_yaw_tolerance_rad: 0.14
  nav_max_steps_per_waypoint: 500
  nav_max_replans: 64
  nav_body_hold_steps: 10
  arm_confirm_prompt: "是否执行 {route}？输入 1=执行 0=跳过: "
  arm_confirm_timeout_s: null
  show_raw_text: true
  show_timing: true
  jsonl_out: logs/vla_eval/real_vla_interactive.jsonl
```

关键项：

- `deploy.server_ssh` / `deploy.robot_ssh`：工作站 SSH alias，用于自动建隧道与
  在 robodog 上起 tmux；置空则跳过自动部署；
- `real.mode`：`shadow`（只记录不发布）/ `live`（需同时置
  `enable_live_output=true` 与 `confirm_live_output=I_UNDERSTAND_LIVE_OUTPUT`）；
- `real.nav_anchor_mode`：`body`（无里程计，比例速度）/ `odom`（世界系锚定，
  需要 `/odom` 话题）；
- `interactive.*`：NAV 容差、人工门控提示、JSONL。

## 2. 启动前提

1. 推理服务已启动（见 `starVLA_sc/docs/vla_remote_inference_server.md`）；
2. 工作站可 SSH 到 `deploy.server_ssh` 与 `deploy.robot_ssh`（robodog 需公钥免密）；
3. robodog 已 source ROS 环境，front/wrist 相机发布 JPEG，X5 供电、`can0` 就绪；
4. 已按 `docs/实机测试指南.md` 完成必要前检，操作者握住手柄与急停。

## 3. 启动

在工作站执行：

```bash
cd /home/natural/Desktop/mtr/gx-real
bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml start
```

该命令会：

1. 建立双跳 SSH 隧道（`robodog 127.0.0.1:10093 -> 工作站 -> 服务器`）；
2. 在 robodog 的 tmux 中启动腿部 WBC（`run_vla_leg12_real.sh`）；
3. 在 robodog 的 tmux 中启动 X5 机械臂 CAN owner（`run_vla_arm_real.sh`）；
4. 在 robodog 的 tmux 中启动交互 client（`run_vla_real_interactive.py`）。

之后按实机指南用 `R1` 起身、确认臂状态后用 `L2` 进入 policy；NAV 只执行第一个
waypoint 后请求下一次推理；收到 GRASP/PLACE 时终端阻塞，输入 `1` 执行、`0` 跳过。

检查状态：

```bash
bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml check
ssh robodog 'tmux ls'
```

## 4. 停止

```bash
bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml stop
```

正常收尾顺序：先停止 client，对 WBC 按 `L1` 等待零速/趴下/`/arm/home`，再停机械臂，
最后关闭隧道。

## 5. 安全边界

- 远程 client 不打开 CAN，不替换 `policies/policy.onnx`；
- X5 只有一个 CAN owner（`run_vla_arm_real.sh`）；
- 网络/图像/推理/命令过期一律零速或保持，不沿用旧动作；
- `live` 必须显式双确认，`shadow` 为默认。
