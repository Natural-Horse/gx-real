# VLA 仿真与真机控制接口

## 统一动作语义

模型使用固定 10 维输出：

```text
[dx_body, dy_body, dyaw,
 tcp_x_base, tcp_y_base, tcp_z_base,
 roll_base, pitch_base, yaw_base, gripper]
```

- `NAV` 只读取前三维机体系稀疏 waypoint，由本地导航适配器转换为速度，再送入已有机器狗底层 policy。
- `GRASP`、`PLACE` 只读取后七维机体系 TCP 位姿和归一化夹爪目标。
- `DONE`、`RECOVER` 不直接产生连续硬件动作。

固定 10 维只是协议容器，不表示所有 route 同时控制 10 维。训练使用 route 对应的维度 mask，未启用维度不参与 loss。

## 真机执行边界

`real-wbc/modules/vla_command_contract.py` 只校验和拆分模型动作，不接触 CAN。机械臂真机执行器必须依次完成：坐标标定、IK 或局部规划、工作空间/关节/速度限制、碰撞检查、急停和命令看门狗，最后才由唯一 CAN owner 写入 X5。

SpaceMouse 与 VLA 是互斥的目标来源。现有 `SpaceMouseArmNode` 已拥有 CAN 时，VLA 执行器不得启动；VLA 接入未通过 dry-run 和真机 smoke test 前，不允许把解析结果直接交给 `set_eef_cmd`。
