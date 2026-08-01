# StarVLA 真机测评教程

## 1. 当前支持状态

当前 gx-real 已具备统一 10 维动作合同、机体系 NAV/ARM 拆分和外部速度 watchdog，但尚未具备可直接启动的远程 VLA 真机闭环节点。以下能力已经可用：

- 离线评价 VLM 的 route/subtask。
- 纯 Python 校验模型动作合同。
- 用现有 SpaceMouse 和固定速度流程评价机器狗 policy、X5、急停与 CAN 基线。
- 以 dry-run 方式开发 VLA adapter。

以下能力尚未完成，不能用占位命令绕过：同步 front/wrist 观测上传、NAV waypoint 到速度的真机 adapter、VLA TCP 目标到标定/IK/规划/安全层的机械臂执行器，以及统一的真机 episode recorder。完成这些模块并通过逐级 smoke test 前，模型输出不得交给 `set_eef_cmd`、关节控制器或 CAN。

## 2. 评测分级

### A. 离线模型评测

先在推理服务器对 held-out episode 统计 route accuracy、subtask exact match 和联合准确率。只有 VLM 路由达到要求后才训练或接入 action head。

### B. 本地动作合同测试

在开发机或 Jetson 上运行，不连接 CAN：

```bash
cd ~/gx-real
python -m pytest -q tests/test_vla_command_contract.py tests/test_base_command_provider.py
```

这一步验证 10 维动作拆分、夹爪范围和 watchdog；不验证机器人运动。

### C. 真机底层基线

确认机器人周围无人、机械臂工作空间清空、手柄和急停可用。先执行环境与 CAN 检查：

```bash
conda deactivate 2>/dev/null || true
cd ~/gx-real
export GX_REAL_NETWORK_IFACE=eth0
source scripts/setup_env.sh
scripts/prepare_real_run.sh
```

终端 A 启动 X5 固定训练姿态：

```bash
cd ~/gx-real
scripts/run_arm_training_hold.sh
```

终端 B 启动 Go2 固定速度基线：

```bash
cd ~/gx-real
export GX_REAL_NETWORK_IFACE=eth0
scripts/run_fixed_03_real.sh
```

按当前手柄合同执行 R1 站立、L2 policy handover 和 L1 受控停机。该流程只验证底层 locomotion policy、机械臂状态发布、CAN owner 和急停，不评价 VLA。

## 3. VLA 闭环接入验收顺序

后续 adapter 完成后必须按以下顺序验收，每一级通过后才能进入下一级：

1. 录包回放：读取已录图像和状态，打印 route/subtask/action，不发布 ROS/CAN。
2. 在线 shadow mode：实时推理并记录输出，真实机器人仍由人工控制。
3. 底盘架空或安全支撑：NAV adapter 输出速度，但安全 gate 强制零速度。
4. 低速空场 NAV：启用速度限幅、watchdog 和遥控器即时接管。
5. X5 dry-run：计算标定、IK/规划和限幅结果，不创建 CAN owner。
6. X5 单步目标：每次只执行一个经审核的 TCP 小位移，夹爪独立验证。
7. 分 subtask 闭环：分别评价 NAV、GRASP、PLACE。
8. 最后才运行完整任务。

## 4. 真机执行边界

- NAV 模型输出是 `[dx_body,dy_body,dyaw]` waypoint，不是速度。真机本地 adapter 必须转换为 `[vx,vy,wz]`，再写入 `ExternalVelocityCommandProvider`。
- GRASP/PLACE 输出是 `[tcp_x,tcp_y,tcp_z,roll,pitch,yaw,gripper]`，必须经过 X5 标定、IK 或局部规划、工作空间/关节/速度限制和碰撞检查。
- SpaceMouse 与 VLA 机械臂执行器互斥，只能有一个 CAN owner。
- 推理超时、观测过期、协议错误、NaN/Inf、规划失败或急停必须立即发送零速度并让 X5 进入安全状态。
- 远程服务只绑定服务器 `127.0.0.1`，Jetson 通过 SSH tunnel 访问；不得暴露公网端口。

## 5. 结果记录

每条 episode 至少记录：代码 commit、checkpoint、协议版本、任务 instruction、route/subtask、模型动作、适配后速度或规划目标、推理延迟、watchdog 状态、人工接管、急停、任务阶段结果和失败原因。

真机主要指标应分层统计：VLM route/subtask 准确率、NAV 到达率、机械臂规划成功率、抓取成功率、放置成功率、完整任务成功率、安全 gate 触发率、人工接管率和超时率。不能只报告完整任务成功率。
