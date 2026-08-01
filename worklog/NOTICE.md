# 项目工作约定

- 本地仓库：`/home/natural/Desktop/mtr/gx-real`
- 当前分支：`mtr_dev`
- Fork：`https://github.com/Natural-Horse/gx-real.git`（`origin`）
- 上游：`https://github.com/lemonoscar/gx-real.git`（`upstream`）
- 模型仓库：`/home/natural/Desktop/mtr/starVLA_sc`，分支 `robodog`
- 仿真仓库：`/home/natural/Desktop/mtr/pct_scene`，分支 `mtr_dev`

工作时保留用户已有改动。先本地修改并做纯 Python 测试；涉及真机启动、CAN、急停、网络接口和 ROS 节点时，必须先展示准确命令并由用户确认。不得停止或复用不属于本任务的进程。

三仓库统一动作协议为 10 维 `[dx_body,dy_body,dyaw,tcp_x_base,tcp_y_base,tcp_z_base,roll_base,pitch_base,yaw_base,gripper]`。NAV 只使用前三维；GRASP/PLACE 只使用后七维。模型机械臂输出是 base frame TCP 目标，不是关节角。gx-real 必须在本地完成坐标标定、IK/规划、工作空间与速率限制、碰撞检查、急停、看门狗，并由唯一 CAN owner 写入硬件。

SpaceMouse 和 VLA 是可替换的上层目标来源，不得同时拥有机械臂 CAN。现有 SpaceMouse 默认路径保持不变；VLA 执行适配器未完成真机 smoke test 前只允许 dry-run 或记录目标。

所有日志和说明文档使用中文。每天只写 `worklog/YYYY-MM-DD.md`，结束时刷新精简的 `## 全日总结`。
