# 实机 VLA 交互 Client 启动指南

本文描述如何在真机（Go2 + X5）上启动远程 StarVLA 交互 client 及配套服务
（腿部 WBC、机械臂 CAN owner）。所有参数集中在 YAML，在 robodog 本机
一条命令启动/停止。

链路跨三台机器，**各机器分别登录、分别启动**：

```text
GPU 服务器 (starVLA_sc)        工作站                         robodog (gx-real)
  推理服务 127.0.0.1:10093  <-SSH 本地转发->   <-SSH 反向隧道->   client 连 127.0.0.1:10093
  登录服务器启动               登录工作站建隧道                 登录 robodog 启动 leg/arm/client
```

robodog 没有默认路由，隧道必须由工作站主动发起；除此之外三个进程组各自独立。

## 1. 配置（YAML）

`configs/vla_eval/real_go2_x5.yaml`：

```yaml
server:
  endpoint: ws://127.0.0.1:10093
  connect_timeout_s: 10.0
  response_timeout_s: 120.0

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

- `real.mode`：`shadow`（只记录不发布）/ `live`（需同时置
  `enable_live_output=true` 与 `confirm_live_output=I_UNDERSTAND_LIVE_OUTPUT`）；
- `real.nav_anchor_mode`：`body`（无里程计，比例速度）/ `odom`（世界系锚定，
  需要 `/odom` 话题）；
- `interactive.*`：NAV 容差、人工门控提示、JSONL。

## 2. 启动前提（三台机器分别准备）

1. **GPU 服务器**：推理服务已启动（见
   `starVLA_sc/docs/vla_remote_inference_server.md`），监听 `127.0.0.1:10093`；
2. **工作站**：可 SSH 到服务器与 robodog（robodog 需公钥免密），并已配好
   `manage_remote_vla_tunnel.sh`；
3. **robodog**：仓库位于 `~/gx-real`，front/wrist 相机发布 JPEG，X5 供电、
   `can0` 就绪；已按 `docs/实机测试指南.md` 完成必要前检，操作者握住手柄与急停。

### 2.1 真机相机（RealSense）

robodog 上通过 `scripts/publish_real_cameras.py` 把两台 RealSense 的 RGB 流转成
JPEG 发布到 client 订阅的 topic（已集成进 `run_vla_real_all.sh` 的 `vla_cams`
会话）：

```bash
cd ~/gx-real && source scripts/setup_env.sh
python3 scripts/publish_real_cameras.py \
  --front-dev /dev/video4 \    # 前置（8086:0b3a）彩色节点，USB2 下 424x240
  --wrist-dev /dev/video10 \   # 腕部 D436（8086:1156）彩色节点，640x480
  --rate 5.0
```

注意：脚本强制 V4L2 后端（Jetson 默认 GStreamer 读不了 `/dev/video*`）；
设备节点会随 USB 枚举顺序变化，现场用 `lsusb` / `v4l2-ctl --list-devices` 确认。
两个相机目前都在 USB2 上只有低分辨率模式，VLA 输入 224x224 够用；要跑满
D436 的 1280x800@60 需改插 Jetson USB3 口。

## 3. 上机前检查（参考 README 第 5 节）

真机启动前必须在 Jetson 上完成以下检查，全部通过后再进入 VLA 链路。任何一项失败
都要先修复，不要继续 rollout。

### 3.1 环境预检

```bash
conda deactivate
cd ~/gx-real
git pull                       # robodog 无 GitHub 访问，改为 rsync/bundle 同步
source scripts/setup_env.sh
scripts/check_env.sh           # 通过时输出 [gx-real] python imports OK
```

若使用 SpaceMouse 辅助，再加：

```bash
scripts/check_env.sh --spacemouse
```

确认确实在 Jetson 上，且 Python 是系统解释器：

```bash
uname -m                       # 应为 aarch64
which python3
echo "${GX_REAL_PYTHON_BIN}"   # 应为 /usr/bin/python3
```

### 3.2 Go2 网络与 ROS2 topic

```bash
ip a
ip route
ros2 topic list
timeout 8 ros2 topic list | grep camera   # 应有 front/wrist 两个 topic
ros2 topic echo /lowstate --once
ros2 topic echo /wirelesscontroller --once
ros2 topic echo lf/sportmodestate --once
```

这些 topic 必须有数据；没有数据先修网络与 CycloneDDS，不要进入低层 rollout。

### 3.3 SocketCAN 与 MCF 释放

```bash
ip -details link show can0
```

没有 `can0` 或不是 `UP` 时：

```bash
scripts/setup_arx_can.sh
ip -details link show can0
```

释放 Go2 MCF（`eth0` 换成 Jetson 实际连 Go2 的网卡，可用 `ip a` 找
`192.168.123.xxx` 所在接口）：

```bash
scripts/disable_sports_mode_go2.sh eth0
```

该工具会用 `CheckMode()` 检查 motion mode、`ReleaseMode()` 后再次验证；任何 SDK
错误或仍有活动模式都会失败。不要绕过这一步，否则 Go2 原厂高层控制会和低层
`lowcmd` 抢控制权。

### 3.4 CAN owner 唯一性

X5 只允许一个写控制进程打开 `can0`。不要同时运行
`arx5-sdk/python/examples/spacemouse_teleop.py`、
`scripts/run_arm_spacemouse_test.sh` 或 WBC legacy arm write 模式。

## 4. 启动（按顺序在对应机器执行）

### 4.1 GPU 服务器：启动推理服务

```bash
ssh zju-server
cd /hdd4/MaTianran/pct_workspace/starVLA_sc
bash scripts/evaluation/start_vla_inference.sh --config configs/vla_eval/server.yaml
```

### 4.2 工作站：建立双跳隧道

```bash
cd /home/natural/Desktop/mtr/gx-real
scripts/evaluation/manage_remote_vla_tunnel.sh start zju-server robodog
scripts/evaluation/manage_remote_vla_tunnel.sh check zju-server robodog
```

链路：`robodog 127.0.0.1:10093 -> 工作站 10094 -> 服务器 127.0.0.1:10093`。
robodog 无需出网，隧道由工作站主动发起。

### 4.3 robodog：启动腿部 / 机械臂 / 交互 client

```bash
ssh robodog
cd ~/gx-real
bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml start
```

该命令会：

1. 在 robodog 上做上机前检查（`check_env.sh`、`can0` UP 等，任一失败即中止）；
2. 在 tmux 中启动腿部 WBC（`run_vla_leg12_real.sh`）；
3. 在 tmux 中启动 X5 机械臂 CAN owner（`run_vla_arm_real.sh`）；
4. 在 tmux 中启动交互 client（`run_vla_real_interactive.py`，连接本机回环
   `ws://127.0.0.1:10093`，经工作站隧道到服务器）。

之后按实机指南用 `R1` 起身、确认臂状态后用 `L2` 进入 policy；NAV 只执行第一个
waypoint 后请求下一次推理；收到 GRASP/PLACE 时终端阻塞，输入 `1` 执行、`0` 跳过。

检查状态：

```bash
bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml check
tmux ls
```

## 5. 停止

```bash
ssh robodog 'cd ~/gx-real && bash scripts/run_vla_real_all.sh --config configs/vla_eval/real_go2_x5.yaml stop'
cd /home/natural/Desktop/mtr/gx-real
scripts/evaluation/manage_remote_vla_tunnel.sh stop zju-server robodog
```

正常收尾顺序：先停止 client，对 WBC 按 `L1` 等待零速/趴下/`/arm/home`，再停机械臂，
最后在工作站关闭隧道、停服务器推理服务。

## 6. 安全边界

- 远程 client 不打开 CAN，不替换 `policies/policy.onnx`；
- X5 只有一个 CAN owner（`run_vla_arm_real.sh`）；
- 网络/图像/推理/命令过期一律零速或保持，不沿用旧动作；
- `live` 必须显式双确认，`shadow` 为默认。
