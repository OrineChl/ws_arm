# custom_3dof_kinematics

三自由度机械臂逆运动学与底层控制参数预览（ROS 2 Humble / ament_python）。

输入末端 XYZ（米），输出符合 `ws_arm_base` 底层接口的三关节角度、
`Joint_Control(...)` 调用参数和完整 `0x12` 串口帧。
**仅离线计算与预览：不打开串口、不发送任何指令。**

模型参数以底层固件为权威参考（`ws_arm_base/bsp/manipulator/kinematics.c`），
逐项保留固件的十进制近似角度（1.570796 / -3.141593），未替换为精确 π。

## 模型约定

- 9 关节 MDH 链：3 个主动关节 + 6 个固定关节；底盘位姿固定为零（固件当前硬编码）。
- 臂在底盘上的安装偏置 `base_offset = (0.085, 0, 0.240) m`。
- 输入 XYZ 使用底层末端上报坐标系（底盘固定时等同 chassis / world），**不是**机械臂 base_link 系。
- 零位（工程图姿态）末端位置约 `(0.349, 0, 0.612) m`。
- 仅求末端位置，不支持姿态输入（3DOF 无法任意定姿）。

### 关节 ↔ 电机映射

发送的是**逻辑关节角（度）**，减速比由底层 `Joint_Control()` 内部处理，上位机不要重复乘。

| 关节 | 驱动电机 | 底层减速比 | 软件限位（度） |
|---|---|---|---|
| q1 | Joint_1 | 3 | [-90, 90] |
| q2 | Joint_2 + Joint_3（镜像驱动） | 6 | [0, 90] |
| q3 | Joint_4 | 4.8 | [-90, 0] |

限位来源：`ws_arm_base/bsp/manipulator/manipulator.c` 的 `Check_Join_Limits()`。

### 逆解策略

- SciPy `least_squares`（trf，带限位），先取限位区间中点，失败后按各关节
  5%/50%/95% 的 3×3×3 确定性初值网格重试，最多 27 组初值。
- 验收不依赖优化器的 success 标志：解必须在限位内、按 float32 量化后回代正解，
  位置欧氏残差 ≤ 1e-5 m（该阈值仅代表模型计算精度，不是实机标定精度）。
- 返回第一个通过验证的解；3DOF 位置逆解可能存在多解，本工具不枚举全部解。
- 全部初值失败时报“未找到符合限位和精度要求的解”，不代表已证明几何不可达。

## 使用

### 构建（工作空间根目录）

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_3dof_kinematics
source install/setup.bash
```

### 打开测试窗口

```bash
ros2 run custom_3dof_kinematics ik_test_gui
```

窗口功能：输入 XYZ → 求解；显示三关节角度（度/弧度）、电机映射与限位、
目标/回算 XYZ 与位置残差、`Joint_Control(...)` 调用参数、完整 `0x12` 帧十六进制。
“复制控制帧”将帧复制到剪贴板；“加载零位示例”填入零位 XYZ（逆解可能返回位置等价的另一组关节角）。
任何输入修改都会清空旧结果并禁用复制，防止把旧结果误用于新目标。
求解在后台线程执行，失败时给出中文原因（超出伸展范围 / 越限 / NaN 等）。

WSL 提示：界面中文显示为方框是系统缺 CJK 字体，可链接 Windows 字体：

```bash
mkdir -p ~/.local/share/fonts
ln -s /mnt/c/Windows/Fonts/msyh.ttc ~/.local/share/fonts/
fc-cache ~/.local/share/fonts
```

### Python API

```python
from custom_3dof_kinematics.kinematics import forward_kinematics, inverse_kinematics
from custom_3dof_kinematics.protocol import encode_joint_command

target = forward_kinematics([30.0, 45.0, -60.0])  # 正解：关节角(度) → 末端 XYZ(m)
angles = inverse_kinematics(target)               # 逆解：末端 XYZ(m) → 关节角(度)
frame = encode_joint_command(angles)              # 0x12 帧（17 字节）
```

输入非法（维度错误、NaN/Inf）、越限或不可达时抛 `ValueError`，不产生控制参数。

## 0x12 控制帧格式

```
AA 55 | 12 | 0C | float32 × 3（小端，逻辑关节角，度） | 校验和
```

- 帧长 17 字节；校验和 = 从 `0x12` 起对命令、长度、负载逐字节累加取低 8 位。
- 示例（q = 30, 45, -60 度）：
  `AA 55 12 0C 00 00 F0 41 00 00 34 42 00 00 70 C2 F7`
- 解析端：`ws_arm_base/bsp/RK/uart1_cmd.c` 的 `ExecuteCommand()`（cmd 0x12 分支），
  收到后调用 `Joint_Control(q1, q2, q3, 0, 0)`。
- 注意：`0x12` 不含速度/加速度字段，固件沿用当前 `Velocity_Control` /
  `Acceleration_Control` 设置；且该分支会以零参数下发腕部/夹爪舵机，
  并非“其余关节保持不动”。

## 测试

```bash
colcon test --packages-select custom_3dof_kinematics
colcon test-result --verbose
```

覆盖内容：

- FK↔IK 往返：限位角点、直臂奇异附近（q3 ≈ -85.67°）、随机位形；比较末端位置而非关节角唯一性。
- 非法/不可达输入（越界、NaN/Inf、维度错误）均拒绝且不产生控制帧。
- `0x12` 帧长、头、长度、小端 float32、校验和逐项断言。
- **固件 C 对拍**：用主机 gcc 将 `ws_arm_base/bsp/manipulator/kinematics.c`
  编译为共享库，与 Python 正解逐点对照（缺 gcc 或固件检出时自动跳过）。
- QtTest 驱动真实窗口：求解、复制、输入修改清除结果、错误恢复、求解中关闭窗口。

## 文件结构

```
custom_3dof_kinematics/
├── custom_3dof_kinematics/
│   ├── kinematics.py    # 固件 MDH 正解、带限位逆解（SciPy）
│   ├── protocol.py      # 0x12 帧打包与校验
│   └── gui.py           # PyQt5 测试窗口（求解在后台线程）
├── test/
│   ├── test_kinematics.py
│   └── test_gui.py
├── package.xml / setup.py / setup.cfg / resource/
```

## 安全须知

- 本包绝不打开串口或发送指令；生成参数不等于实机运动安全。
- 未校验碰撞、回零状态、机械限位；TCP 尺寸为 provisional 工程图估计（见描述包）。
- 实机发送前必须按底层要求完成回零、方向与限位验证，并逐级低速试运行。
