# ws_arm

轮臂复合机器人工作空间：ROS 2 Humble 上位机 + `ws_arm_base` STM32 底层固件（git 子模块）。
上层模型与运动学以底层固件为权威参考（`ws_arm_base/bsp/manipulator/kinematics.c`），底层不修改。

## 功能包

| 包 | 构建类型 | 内容 |
|---|---|---|
| `src/custom_3dof_description` | ament_cmake | 三自由度机械臂 URDF/Xacro（与底层 FK 对齐）、零位验证脚本、RViz 可视化 |
| `src/custom_3dof_kinematics` | ament_python | 底层对齐的逆运动学、`0x12` 控制帧离线预览窗口（PyQt5，不发送指令） |

## 快速开始

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# RViz 可视化
ros2 launch custom_3dof_description display.launch.py

# 逆运动学测试窗口（输入 XYZ，预览底层控制参数）
ros2 run custom_3dof_kinematics ik_test_gui
```

各包详情见 `src/custom_3dof_description/README.md` 与 `src/custom_3dof_kinematics/README.MD`。

# TODO

## 已完成（2026-09-08）

- [x] 第一步（运动学部分）：URDF 与底层正运动学对拍一致（含底盘镜像链与安装偏置），零位脚本断言 tcp = (0.349, 0, 0.612) m（world 系），逆解与固件 C 正解逐点对照通过。
- [x] 新增 `custom_3dof_kinematics`：底层对齐的 XYZ→关节角逆解、`Joint_Control(...)` 参数与 `0x12` 控制帧离线预览窗口（不发送指令，154 项测试通过）。
- [x] 软件限位标明并在逆解中强制执行：q1 [-90,90]、q2 [0,90]、q3 [-90,0] 度（manipulator.c）。
- [x] 关节-电机-减速比对应关系：joint1→电机1（×3）、joint2→电机2+3 镜像（×6）、joint3→电机4（×4.8）；关节正方向已与底层 MDH 对拍确定（joint4 为固定连接，腕部/夹爪舵机不在链内）。
- [x] 底层通信协议标明：0x10 单关节（含速度/加速度）、0x12 三关节同步、0x30 底盘 PWM、0x31 夹爪、0x40 三关节力矩模式；状态上报帧含轮角度/速度、末端位置/速度、关节角度/速度。

## 待办

- [ ] RViz 桌面显示验收：上次记录（validation_results.md，旧四自由度模型）中 RViz 未显示模型且原因未查明；需按当前三自由度模型重新执行完整验收（滑块独立运动、RobotModel、TF 树）。
- [ ] 更新 `custom_3dof_description/docs/coordinate_conventions.md` 与 `validation_results.md`：仍是旧四自由度内容直接改名，未反映三自由度模型。
- [ ] 若需要动力学仿真：补齐各连杆质量、质心、惯量、实际碰撞外形，以及关节摩擦和阻尼数据，再建立并验证仿真模型。
- [ ] 接入实物前，确认实测连杆尺寸、机械零位和装配偏差（关节轴方向已与底层对拍确定）。
- [ ] 取得实测机械限位后，将临时 continuous 关节改为 revolute；补齐各关节最大允许速度、加速度和力矩。
- [ ] 标明各电机和驱动器型号、供电要求、设备地址表（通信链路已明确：UART4 位置/速度/力矩模式，UART5 腕部/夹爪舵机反馈；换算已知：Emm 3200 脉冲/圈、X 系列 0.1°/计数，反馈角 = 电机角 ÷ 减速比）。
- [ ] 标明回零方式、限位开关位置、上电姿态要求，以及断电或急停时的制动、防坠落措施。
- [ ] 标定 TCP 的实际位置和朝向；当前沿 X 的 0.092 m、沿 Z 的 0.020 m 均为 provisional 工程图估计，不能作为最终标定值。
- [ ] 标明夹爪独立电机的接口、开合范围、夹持力限制和反馈方式；当前描述链不包含夹爪张合自由度。
- [ ] 标明允许负载、机械干涉区域、线缆活动范围和安全操作距离（逆解的 0.636 m 伸展上限仅为几何值，不代表负载能力）。
- [ ] 定义急停、通信超时、编码器异常、过流过温、越限和失电时的安全行为，以及恢复运行条件。
- [ ] 完成硬件接口、状态反馈与控制实现后，按单关节、低速、空载、多关节的顺序验证；确认方向、回零、限位和急停有效后，再逐步进行带载测试。
