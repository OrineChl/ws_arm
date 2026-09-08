# 自制三自由度机械臂描述包

目标环境：Ubuntu 22.04 / ROS 2 Humble。模型为 **visualization-only**，
不能用于动力学仿真或真实控制。没有质量、质心、惯量或实测机械限位。

权威参考是底层固件 `ws_arm_base/bsp/manipulator/kinematics.c`：URDF 与底层正运动学对齐，
`T = T_car(x,y,phi) · T_base(0.085,0,0.24) · MDH(theta1..3)`，详见 URDF 头注释。

- 三个运动关节 joint1～joint3 临时使用 continuous（底层软件限位
  [-90,90]/[0,90]/[-90,0] 度，机械限位未实测，取得后改 revolute 并填入实测值）。
- joint4 在底层 FK 中为固定连接，已按 fixed 建模；腕部舵机与夹爪不在链内。
- 底盘按固件 q = [x, y, phi] 镜像为 car_x / car_y / car_yaw 三关节
  （KDL 不支持 planar 关节故拆分；固件当前硬编码为零，world 与 chassis 重合）。
- 轴方向由底层 MDH 链对拍确定：joint1 (0,0,1)，joint2/joint3 (0,1,0)
  （旧版 (0,-1,0) 与底层反号，已翻转）。

逆运动学与底层控制参数预览由姊妹包 `custom_3dof_kinematics` 提供。

## 构建与运行

运行依赖：xacro、robot_state_publisher、joint_state_publisher、
joint_state_publisher_gui、rviz2。检查器 check_urdf 用于验收。
缺失软件须由用户自行决定安装，本工程不自动安装。

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
mkdir -p build/custom_3dof_description/validation
xacro src/custom_3dof_description/urdf/custom_3dof.urdf.xacro \
  -o build/custom_3dof_description/validation/custom_3dof.urdf
check_urdf build/custom_3dof_description/validation/custom_3dof.urdf
ros2 run custom_3dof_description verify_zero_pose.py \
  build/custom_3dof_description/validation/custom_3dof.urdf
ros2 launch custom_3dof_description display.launch.py
```

验证脚本也可直接执行，默认展开包内 Xacro；失败返回非零退出码。
它读取实际 URDF，累乘 joint origin（含 rpy），检查链、关节类型、轴和零位位置
（world 系期望：base_link (0.085,0,0.24)、tool0 (0.257,0,0.592)、
tcp (0.349,0,0.612) m），容差 1e-9 m。缺少 xacro 返回错误，不视为验证通过。

启动参数：

- `use_gui:=true`：默认启用关节滑块；false 使用非 GUI joint_state_publisher 发布零位。
- `model:=/absolute/path/model.urdf.xacro`：替换描述文件。
- `rviz_config:=/absolute/path/config.rviz`：替换 RViz 配置。

`use_gui:=false` 仍启动 RViz，因此并非纯无头运行模式。
RViz Fixed Frame 为 base_link，RobotModel 订阅 /robot_description。
桌面验收时拖动滑块确认 joint1～joint3 可独立运动、子链跟随、TF 连通、无 RobotModel 错误。
无图形界面时先完成构建、Xacro/URDF 和零位验证，RViz 留待有桌面的主机验证。

## 参数与模型约定

几何长度集中在 `urdf/custom_3dof.urdf.xacro` 的 property 中；
`config/robot_parameters.yaml` 是工程参数记录，不是传给 ROS 节点的参数文件，修改尺寸时同步更新记录。
零位验证的期望值保持独立，防止模型尺寸被误改却仍然通过。
简单几何体的宽度、底座半径等为示意尺寸，visual/collision 相同，不表示精确实物包络。

坐标转换见 [coordinate_conventions.md](docs/coordinate_conventions.md)。
tool0 与 link4 重合，tcp 相对 tool0 平移 (0.092, 0, 0.020) m。
这两个 TCP 尺寸均为 **provisional**，不是最终标定值。

下一阶段须提供各关节机械上下限、允许速度/力矩、实物零位与电机正方向、
精确 TCP 位置与朝向。动力学工作还需要逐连杆质量、质心和惯量，精确碰撞建模需要外形/CAD。
若进入真实控制阶段，另需电机/驱动器型号、协议、减速比、编码器换算、回零和安全停机要求；
底层关节-电机-减速比对应关系见 `custom_3dof_kinematics` 的 README。

当前环境验收记录见 [validation_results.md](docs/validation_results.md)。
